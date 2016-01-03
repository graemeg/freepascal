{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       QD/Video.h
 
     Contains:   Video Driver Interfaces.
 
     Version:    Quickdraw-262~1
 
     Copyright:  © 1986-2008 by Apple Computer, Inc., all rights reserved
=======
=======
>>>>>>> origin/fixes_2_2
     File:       Video.p
=======
     File:       QD/Video.h
>>>>>>> origin/fixes_2.4
 
     Contains:   Video Driver Interfaces.
 
     Version:    Quickdraw-262~1
 
<<<<<<< HEAD
     Copyright:  © 1986-2002 by Apple Computer, Inc., all rights reserved
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  © 1986-2008 by Apple Computer, Inc., all rights reserved
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

unit Video;
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
>>>>>>> origin/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
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
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
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
uses MacTypes,QuickdrawTypes;
{$endc} {not MACOSALLINCLUDE}
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}

=======


>>>>>>> origin/cpstrnew
{$ifc TARGET_OS_MAC}
=======
uses MacTypes,NameRegistry,Quickdraw;

>>>>>>> graemeg/fixes_2_2
=======
uses MacTypes,NameRegistry,Quickdraw;

>>>>>>> origin/fixes_2_2
=======
uses MacTypes,QuickdrawTypes;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}
>>>>>>> origin/fixes_2.4

{$ALIGN MAC68K}


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{******************* DEPRECATION NOTICE *********************
 *
 * All declarations in this file are deprecated; their use 
 * doesn't make sense on Mac OS X. Some structures are used
 * in the DisplayMgr API, which is being deprecated as well.
 * Use the CGDirectDisplay API in the CoreGraphics framework in 
 * ApplicationServices.framework as replacement.
 *
 ************************************************************}

const
	mBaseOffset = 1;    {Id of mBaseOffset.}
	mRowBytes = 2;    {Video sResource parameter Id's }
	mBounds = 3;    {Video sResource parameter Id's }
	mVersion = 4;    {Video sResource parameter Id's }
	mHRes = 5;    {Video sResource parameter Id's }
	mVRes = 6;    {Video sResource parameter Id's }
	mPixelType = 7;    {Video sResource parameter Id's }
	mPixelSize = 8;    {Video sResource parameter Id's }
	mCmpCount = 9;    {Video sResource parameter Id's }
	mCmpSize = 10;   {Video sResource parameter Id's }
	mPlaneBytes = 11;   {Video sResource parameter Id's }
	mVertRefRate = 14;   {Video sResource parameter Id's }
	mVidParams = 1;    {Video parameter block id.}
	mTable = 2;    {Offset to the table.}
	mPageCnt = 3;    {Number of pages}
	mDevType = 4;    {Device Type}
	oneBitMode = 128;  {Id of OneBitMode Parameter list.}
	twoBitMode = 129;  {Id of TwoBitMode Parameter list.}
	fourBitMode = 130;  {Id of FourBitMode Parameter list.}
	eightBitMode = 131;   {Id of EightBitMode Parameter list.}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
const
	sixteenBitMode = 132;  {Id of SixteenBitMode Parameter list.}
	thirtyTwoBitMode = 133;  {Id of ThirtyTwoBitMode Parameter list.}
	firstVidMode = 128;  {The new, better way to do the above. }
	secondVidMode = 129;  { QuickDraw only supports six video }
	thirdVidMode = 130;  { at this time.      }
	fourthVidMode = 131;
	fifthVidMode = 132;
	sixthVidMode = 133;
	spGammaDir = 64;
	spVidNamesDir = 65;


{ csTimingFormat values in VDTimingInfo }
{ look in the declaration rom for timing info }
const
	kDeclROMtables = FourCharCode('decl');
	kDetailedTimingFormat = FourCharCode('arba'); { Timing is a detailed timing}

{ Size of a block of EDID (Extended Display Identification Data) }
const
	kDDCBlockSize = 128;

{ ddcBlockType constants}
const
=======
const
	sixteenBitMode = 132;  {Id of SixteenBitMode Parameter list.}
	thirtyTwoBitMode = 133;  {Id of ThirtyTwoBitMode Parameter list.}
	firstVidMode = 128;  {The new, better way to do the above. }
	secondVidMode = 129;  { QuickDraw only supports six video }
	thirdVidMode = 130;  { at this time.      }
	fourthVidMode = 131;
	fifthVidMode = 132;
	sixthVidMode = 133;
	spGammaDir = 64;
	spVidNamesDir = 65;


{ csTimingFormat values in VDTimingInfo }
{ look in the declaration rom for timing info }
const
	kDeclROMtables = FourCharCode('decl');
	kDetailedTimingFormat = FourCharCode('arba'); { Timing is a detailed timing}

{ Size of a block of EDID (Extended Display Identification Data) }
const
	kDDCBlockSize = 128;

{ ddcBlockType constants}
const
>>>>>>> graemeg/cpstrnew
	kDDCBlockTypeEDID = 0;     { EDID block type. }

{ ddcFlags constants}
const
	kDDCForceReadBit = 0;    { Force a new read of the EDID. }
	kDDCForceReadMask = 1 shl kDDCForceReadBit; { Mask for kddcForceReadBit. }


{ Timing mode constants for Display Manager MultiMode support
    Corresponding   .h equates are in Video.h
                    .a equates are in Video.a
                    .r equates are in DepVideoEqu.r
    
    The second enum is the old names (for compatibility).
    The first enum is the new names.
}
const
	timingInvalid = 0;    {    Unknown timing… force user to confirm. }
	timingInvalid_SM_T24 = 8;    {    Work around bug in SM Thunder24 card.}
	timingApple_FixedRateLCD = 42;   {    Lump all fixed-rate LCDs into one category.}
	timingApple_512x384_60hz = 130;  {  512x384  (60 Hz) Rubik timing. }
	timingApple_560x384_60hz = 135;  {  560x384  (60 Hz) Rubik-560 timing. }
	timingApple_640x480_67hz = 140;  {  640x480  (67 Hz) HR timing. }
	timingApple_640x400_67hz = 145;  {  640x400  (67 Hz) HR-400 timing. }
	timingVESA_640x480_60hz = 150;  {  640x480  (60 Hz) VGA timing. }
	timingVESA_640x480_72hz = 152;  {  640x480  (72 Hz) VGA timing. }
	timingVESA_640x480_75hz = 154;  {  640x480  (75 Hz) VGA timing. }
	timingVESA_640x480_85hz = 158;  {  640x480  (85 Hz) VGA timing. }
	timingGTF_640x480_120hz = 159;  {  640x480  (120 Hz) VESA Generalized Timing Formula }
	timingApple_640x870_75hz = 160;  {  640x870  (75 Hz) FPD timing.}
	timingApple_640x818_75hz = 165;  {  640x818  (75 Hz) FPD-818 timing.}
	timingApple_832x624_75hz = 170;  {  832x624  (75 Hz) GoldFish timing.}
	timingVESA_800x600_56hz = 180;  {  800x600  (56 Hz) SVGA timing. }
	timingVESA_800x600_60hz = 182;  {  800x600  (60 Hz) SVGA timing. }
	timingVESA_800x600_72hz = 184;  {  800x600  (72 Hz) SVGA timing. }
	timingVESA_800x600_75hz = 186;  {  800x600  (75 Hz) SVGA timing. }
	timingVESA_800x600_85hz = 188;  {  800x600  (85 Hz) SVGA timing. }
	timingVESA_1024x768_60hz = 190;  { 1024x768  (60 Hz) VESA 1K-60Hz timing. }
	timingVESA_1024x768_70hz = 200;  { 1024x768  (70 Hz) VESA 1K-70Hz timing. }
	timingVESA_1024x768_75hz = 204;  { 1024x768  (75 Hz) VESA 1K-75Hz timing (very similar to timingApple_1024x768_75hz). }
	timingVESA_1024x768_85hz = 208;  { 1024x768  (85 Hz) VESA timing. }
	timingApple_1024x768_75hz = 210;  { 1024x768  (75 Hz) Apple 19" RGB. }
	timingApple_1152x870_75hz = 220;  { 1152x870  (75 Hz) Apple 21" RGB. }
	timingAppleNTSC_ST = 230;  {  512x384  (60 Hz, interlaced, non-convolved). }
	timingAppleNTSC_FF = 232;  {  640x480  (60 Hz, interlaced, non-convolved). }
	timingAppleNTSC_STconv = 234;  {  512x384  (60 Hz, interlaced, convolved). }
	timingAppleNTSC_FFconv = 236;  {  640x480  (60 Hz, interlaced, convolved). }
	timingApplePAL_ST = 238;  {  640x480  (50 Hz, interlaced, non-convolved). }
	timingApplePAL_FF = 240;  {  768x576  (50 Hz, interlaced, non-convolved). }
	timingApplePAL_STconv = 242;  {  640x480  (50 Hz, interlaced, convolved). }
	timingApplePAL_FFconv = 244;  {  768x576  (50 Hz, interlaced, convolved). }
	timingVESA_1280x960_75hz = 250;  { 1280x960  (75 Hz) }
	timingVESA_1280x960_60hz = 252;  { 1280x960  (60 Hz) }
	timingVESA_1280x960_85hz = 254;  { 1280x960  (85 Hz) }
	timingVESA_1280x1024_60hz = 260;  { 1280x1024 (60 Hz) }
	timingVESA_1280x1024_75hz = 262;  { 1280x1024 (75 Hz) }
	timingVESA_1280x1024_85hz = 268;  { 1280x1024 (85 Hz) }
	timingVESA_1600x1200_60hz = 280;  { 1600x1200 (60 Hz) VESA timing. }
	timingVESA_1600x1200_65hz = 282;  { 1600x1200 (65 Hz) VESA timing. }
	timingVESA_1600x1200_70hz = 284;  { 1600x1200 (70 Hz) VESA timing. }
	timingVESA_1600x1200_75hz = 286;  { 1600x1200 (75 Hz) VESA timing (pixel clock is 189.2 Mhz dot clock). }
	timingVESA_1600x1200_80hz = 288;  { 1600x1200 (80 Hz) VESA timing (pixel clock is 216>? Mhz dot clock) - proposed only. }
	timingVESA_1600x1200_85hz = 289;  { 1600x1200 (85 Hz) VESA timing (pixel clock is 229.5 Mhz dot clock). }
	timingVESA_1792x1344_60hz = 296;  { 1792x1344 (60 Hz) VESA timing (204.75 Mhz dot clock). }
	timingVESA_1792x1344_75hz = 298;  { 1792x1344 (75 Hz) VESA timing (261.75 Mhz dot clock). }
	timingVESA_1856x1392_60hz = 300;  { 1856x1392 (60 Hz) VESA timing (218.25 Mhz dot clock). }
	timingVESA_1856x1392_75hz = 302;  { 1856x1392 (75 Hz) VESA timing (288 Mhz dot clock). }
	timingVESA_1920x1440_60hz = 304;  { 1920x1440 (60 Hz) VESA timing (234 Mhz dot clock). }
	timingVESA_1920x1440_75hz = 306;  { 1920x1440 (75 Hz) VESA timing (297 Mhz dot clock). }
	timingSMPTE240M_60hz = 400;  { 60Hz V, 33.75KHz H, interlaced timing, 16:9 aspect, typical resolution of 1920x1035. }
	timingFilmRate_48hz = 410;  { 48Hz V, 25.20KHz H, non-interlaced timing, typical resolution of 640x480. }
	timingSony_1600x1024_76hz = 500;  { 1600x1024 (76 Hz) Sony timing (pixel clock is 170.447 Mhz dot clock). }
	timingSony_1920x1080_60hz = 510;  { 1920x1080 (60 Hz) Sony timing (pixel clock is 159.84 Mhz dot clock). }
	timingSony_1920x1080_72hz = 520;  { 1920x1080 (72 Hz) Sony timing (pixel clock is 216.023 Mhz dot clock). }
	timingSony_1920x1200_76hz = 540;  { 1900x1200 (76 Hz) Sony timing (pixel clock is 243.20 Mhz dot clock). }
	timingApple_0x0_0hz_Offline = 550;   { Indicates that this timing will take the display off-line and remove it from the system. }


{ Deprecated timing names.}
const
	timingApple12 = timingApple_512x384_60hz;
	timingApple12x = timingApple_560x384_60hz;
	timingApple13 = timingApple_640x480_67hz;
	timingApple13x = timingApple_640x400_67hz;
	timingAppleVGA = timingVESA_640x480_60hz;
	timingApple15 = timingApple_640x870_75hz;
	timingApple15x = timingApple_640x818_75hz;
	timingApple16 = timingApple_832x624_75hz;
	timingAppleSVGA = timingVESA_800x600_56hz;
	timingApple1Ka = timingVESA_1024x768_60hz;
	timingApple1Kb = timingVESA_1024x768_70hz;
	timingApple19 = timingApple_1024x768_75hz;
	timingApple21 = timingApple_1152x870_75hz;
	timingSony_1900x1200_74hz = 530;  { 1900x1200 (74 Hz) Sony timing (pixel clock is 236.25 Mhz dot clock). }
	timingSony_1900x1200_76hz = timingSony_1920x1200_76hz; { 1900x1200 (76 Hz) Sony timing (pixel clock is 245.48 Mhz dot clock). }

{ csConnectFlags values in VDDisplayConnectInfo }
const
	kAllModesValid = 0;    { All modes not trimmed by primary init are good close enough to try }
	kAllModesSafe = 1;    { All modes not trimmed by primary init are know to be safe }
	kReportsTagging = 2;    { Can detect tagged displays (to identify smart monitors) }
	kHasDirectConnection = 3;    { True implies that driver can talk directly to device (e.g. serial data link via sense lines) }
	kIsMonoDev = 4;    { Says whether there’s an RGB (0) or Monochrome (1) connection. }
	kUncertainConnection = 5;    { There may not be a display (no sense lines?). }
	kTaggingInfoNonStandard = 6;    { Set when csConnectTaggedType/csConnectTaggedData are non-standard (i.e., not the Apple CRT sense codes). }
	kReportsDDCConnection = 7;    { Card can do ddc (set kHasDirectConnect && kHasDDCConnect if you actually found a ddc display). }
	kHasDDCConnection = 8;    { Card has ddc connect now. }
	kConnectionInactive = 9;    { Set when the connection is NOT currently active (generally used in a multiconnection environment). }
	kDependentConnection = 10;   { Set when some ascpect of THIS connection depends on another (will generally be set in a kModeSimulscan environment). }
	kBuiltInConnection = 11;   { Set when connection is KNOWN to be built-in (this is not the same as kHasDirectConnection). }
	kOverrideConnection = 12;   { Set when the reported connection is not the true one, but is one that has been forced through a SetConnection call }
	kFastCheckForDDC = 13;   { Set when all 3 are true: 1) sense codes indicate DDC display could be attached 2) attempted fast check 3) DDC failed }
	kReportsHotPlugging = 14;    { Detects and reports hot pluggging on connector (via VSL also implies DDC will be up to date w/o force read) }


{ csDisplayType values in VDDisplayConnectInfo }
const
	kUnknownConnect = 1;    { Not sure how we’ll use this, but seems like a good idea. }
	kPanelConnect = 2;    { For use with fixed-in-place LCD panels. }
	kPanelTFTConnect = 2;    { Alias for kPanelConnect }
	kFixedModeCRTConnect = 3;    {  For use with fixed-mode (i.e., very limited range) displays. }
	kMultiModeCRT1Connect = 4;    { 320x200 maybe, 12" maybe, 13" (default), 16" certain, 19" maybe, 21" maybe }
	kMultiModeCRT2Connect = 5;    { 320x200 maybe, 12" maybe, 13" certain, 16" (default), 19" certain, 21" maybe }
	kMultiModeCRT3Connect = 6;    { 320x200 maybe, 12" maybe, 13" certain, 16" certain, 19" default, 21" certain }
	kMultiModeCRT4Connect = 7;    { Expansion to large multi mode (not yet used) }
	kModelessConnect = 8;    { Expansion to modeless model (not yet used) }
	kFullPageConnect = 9;    { 640x818 (to get 8bpp in 512K case) and 640x870 (these two only) }
	kVGAConnect = 10;   { 640x480 VGA default -- question everything else }
	kNTSCConnect = 11;   { NTSC ST (default), FF, STconv, FFconv }
	kPALConnect = 12;   { PAL ST (default), FF, STconv, FFconv }
	kHRConnect = 13;   { Straight-6 connect -- 640x480 and 640x400 (to get 8bpp in 256K case) (these two only) }
	kPanelFSTNConnect = 14;   { For use with fixed-in-place LCD FSTN (aka “Supertwist”) panels }
	kMonoTwoPageConnect = 15;   { 1152x870 Apple color two-page display }
	kColorTwoPageConnect = 16;   { 1152x870 Apple B&W two-page display }
	kColor16Connect = 17;   { 832x624 Apple B&W two-page display }
	kColor19Connect = 18;   { 1024x768 Apple B&W two-page display }
	kGenericCRT = 19;   { Indicates nothing except that connection is CRT in nature. }
	kGenericLCD = 20;   { Indicates nothing except that connection is LCD in nature. }
	kDDCConnect = 21;   { DDC connection, always set kHasDDCConnection }
	kNoConnect = 22;    { No display is connected - load sensing or similar level of hardware detection is assumed (used by resident drivers that support hot plugging when nothing is currently connected) }

{ csTimingFlags values in VDTimingInfoRec }
const
	kModeValid = 0;    { Says that this mode should NOT be trimmed. }
	kModeSafe = 1;    { This mode does not need confirmation }
	kModeDefault = 2;    { This is the default mode for this type of connection }
	kModeShowNow = 3;    { This mode should always be shown (even though it may require a confirm) }
	kModeNotResize = 4;    { This mode should not be used to resize the display (eg. mode selects a different connector on card) }
	kModeRequiresPan = 5;    { This mode has more pixels than are actually displayed }
	kModeInterlaced = 6;    { This mode is interlaced (single pixel lines look bad). }
	kModeShowNever = 7;    { This mode should not be shown in the user interface. }
	kModeSimulscan = 8;    { Indicates that more than one display connection can be driven from a single framebuffer controller. }
	kModeNotPreset = 9;    { Indicates that the timing is not a factory preset for the current display (geometry may need correction) }
	kModeBuiltIn = 10;   { Indicates that the display mode is for the built-in connect only (on multiconnect devices like the PB 3400) Only the driver is quieried }
	kModeStretched = 11;    { Indicates that the display mode will be stretched/distorted to match the display aspect ratio }

{ csDepthFlags in VDVideoParametersInfoRec }
const
	kDepthDependent = 0;     { Says that this depth mode may cause dependent changes in other framebuffers (and . }

{ csResolutionFlags bit flags for VDResolutionInfoRec }
const
=======
=======
>>>>>>> origin/cpstrnew
const
	sixteenBitMode = 132;  {Id of SixteenBitMode Parameter list.}
	thirtyTwoBitMode = 133;  {Id of ThirtyTwoBitMode Parameter list.}
	firstVidMode = 128;  {The new, better way to do the above. }
	secondVidMode = 129;  { QuickDraw only supports six video }
	thirdVidMode = 130;  { at this time.      }
	fourthVidMode = 131;
	fifthVidMode = 132;
	sixthVidMode = 133;
	spGammaDir = 64;
	spVidNamesDir = 65;


{ csTimingFormat values in VDTimingInfo }
{ look in the declaration rom for timing info }
const
	kDeclROMtables = FourCharCode('decl');
	kDetailedTimingFormat = FourCharCode('arba'); { Timing is a detailed timing}

{ Size of a block of EDID (Extended Display Identification Data) }
const
	kDDCBlockSize = 128;

{ ddcBlockType constants}
const
	kDDCBlockTypeEDID = 0;     { EDID block type. }

{ ddcFlags constants}
const
	kDDCForceReadBit = 0;    { Force a new read of the EDID. }
	kDDCForceReadMask = 1 shl kDDCForceReadBit; { Mask for kddcForceReadBit. }


{ Timing mode constants for Display Manager MultiMode support
    Corresponding   .h equates are in Video.h
                    .a equates are in Video.a
                    .r equates are in DepVideoEqu.r
    
    The second enum is the old names (for compatibility).
    The first enum is the new names.
}
const
	timingInvalid = 0;    {    Unknown timing… force user to confirm. }
	timingInvalid_SM_T24 = 8;    {    Work around bug in SM Thunder24 card.}
	timingApple_FixedRateLCD = 42;   {    Lump all fixed-rate LCDs into one category.}
	timingApple_512x384_60hz = 130;  {  512x384  (60 Hz) Rubik timing. }
	timingApple_560x384_60hz = 135;  {  560x384  (60 Hz) Rubik-560 timing. }
	timingApple_640x480_67hz = 140;  {  640x480  (67 Hz) HR timing. }
	timingApple_640x400_67hz = 145;  {  640x400  (67 Hz) HR-400 timing. }
	timingVESA_640x480_60hz = 150;  {  640x480  (60 Hz) VGA timing. }
	timingVESA_640x480_72hz = 152;  {  640x480  (72 Hz) VGA timing. }
	timingVESA_640x480_75hz = 154;  {  640x480  (75 Hz) VGA timing. }
	timingVESA_640x480_85hz = 158;  {  640x480  (85 Hz) VGA timing. }
	timingGTF_640x480_120hz = 159;  {  640x480  (120 Hz) VESA Generalized Timing Formula }
	timingApple_640x870_75hz = 160;  {  640x870  (75 Hz) FPD timing.}
	timingApple_640x818_75hz = 165;  {  640x818  (75 Hz) FPD-818 timing.}
	timingApple_832x624_75hz = 170;  {  832x624  (75 Hz) GoldFish timing.}
	timingVESA_800x600_56hz = 180;  {  800x600  (56 Hz) SVGA timing. }
	timingVESA_800x600_60hz = 182;  {  800x600  (60 Hz) SVGA timing. }
	timingVESA_800x600_72hz = 184;  {  800x600  (72 Hz) SVGA timing. }
	timingVESA_800x600_75hz = 186;  {  800x600  (75 Hz) SVGA timing. }
	timingVESA_800x600_85hz = 188;  {  800x600  (85 Hz) SVGA timing. }
	timingVESA_1024x768_60hz = 190;  { 1024x768  (60 Hz) VESA 1K-60Hz timing. }
	timingVESA_1024x768_70hz = 200;  { 1024x768  (70 Hz) VESA 1K-70Hz timing. }
	timingVESA_1024x768_75hz = 204;  { 1024x768  (75 Hz) VESA 1K-75Hz timing (very similar to timingApple_1024x768_75hz). }
	timingVESA_1024x768_85hz = 208;  { 1024x768  (85 Hz) VESA timing. }
	timingApple_1024x768_75hz = 210;  { 1024x768  (75 Hz) Apple 19" RGB. }
	timingApple_1152x870_75hz = 220;  { 1152x870  (75 Hz) Apple 21" RGB. }
	timingAppleNTSC_ST = 230;  {  512x384  (60 Hz, interlaced, non-convolved). }
	timingAppleNTSC_FF = 232;  {  640x480  (60 Hz, interlaced, non-convolved). }
	timingAppleNTSC_STconv = 234;  {  512x384  (60 Hz, interlaced, convolved). }
	timingAppleNTSC_FFconv = 236;  {  640x480  (60 Hz, interlaced, convolved). }
	timingApplePAL_ST = 238;  {  640x480  (50 Hz, interlaced, non-convolved). }
	timingApplePAL_FF = 240;  {  768x576  (50 Hz, interlaced, non-convolved). }
	timingApplePAL_STconv = 242;  {  640x480  (50 Hz, interlaced, convolved). }
	timingApplePAL_FFconv = 244;  {  768x576  (50 Hz, interlaced, convolved). }
	timingVESA_1280x960_75hz = 250;  { 1280x960  (75 Hz) }
	timingVESA_1280x960_60hz = 252;  { 1280x960  (60 Hz) }
	timingVESA_1280x960_85hz = 254;  { 1280x960  (85 Hz) }
	timingVESA_1280x1024_60hz = 260;  { 1280x1024 (60 Hz) }
	timingVESA_1280x1024_75hz = 262;  { 1280x1024 (75 Hz) }
	timingVESA_1280x1024_85hz = 268;  { 1280x1024 (85 Hz) }
	timingVESA_1600x1200_60hz = 280;  { 1600x1200 (60 Hz) VESA timing. }
	timingVESA_1600x1200_65hz = 282;  { 1600x1200 (65 Hz) VESA timing. }
	timingVESA_1600x1200_70hz = 284;  { 1600x1200 (70 Hz) VESA timing. }
	timingVESA_1600x1200_75hz = 286;  { 1600x1200 (75 Hz) VESA timing (pixel clock is 189.2 Mhz dot clock). }
	timingVESA_1600x1200_80hz = 288;  { 1600x1200 (80 Hz) VESA timing (pixel clock is 216>? Mhz dot clock) - proposed only. }
	timingVESA_1600x1200_85hz = 289;  { 1600x1200 (85 Hz) VESA timing (pixel clock is 229.5 Mhz dot clock). }
	timingVESA_1792x1344_60hz = 296;  { 1792x1344 (60 Hz) VESA timing (204.75 Mhz dot clock). }
	timingVESA_1792x1344_75hz = 298;  { 1792x1344 (75 Hz) VESA timing (261.75 Mhz dot clock). }
	timingVESA_1856x1392_60hz = 300;  { 1856x1392 (60 Hz) VESA timing (218.25 Mhz dot clock). }
	timingVESA_1856x1392_75hz = 302;  { 1856x1392 (75 Hz) VESA timing (288 Mhz dot clock). }
	timingVESA_1920x1440_60hz = 304;  { 1920x1440 (60 Hz) VESA timing (234 Mhz dot clock). }
	timingVESA_1920x1440_75hz = 306;  { 1920x1440 (75 Hz) VESA timing (297 Mhz dot clock). }
	timingSMPTE240M_60hz = 400;  { 60Hz V, 33.75KHz H, interlaced timing, 16:9 aspect, typical resolution of 1920x1035. }
	timingFilmRate_48hz = 410;  { 48Hz V, 25.20KHz H, non-interlaced timing, typical resolution of 640x480. }
	timingSony_1600x1024_76hz = 500;  { 1600x1024 (76 Hz) Sony timing (pixel clock is 170.447 Mhz dot clock). }
	timingSony_1920x1080_60hz = 510;  { 1920x1080 (60 Hz) Sony timing (pixel clock is 159.84 Mhz dot clock). }
	timingSony_1920x1080_72hz = 520;  { 1920x1080 (72 Hz) Sony timing (pixel clock is 216.023 Mhz dot clock). }
	timingSony_1920x1200_76hz = 540;  { 1900x1200 (76 Hz) Sony timing (pixel clock is 243.20 Mhz dot clock). }
	timingApple_0x0_0hz_Offline = 550;   { Indicates that this timing will take the display off-line and remove it from the system. }


{ Deprecated timing names.}
const
	timingApple12 = timingApple_512x384_60hz;
	timingApple12x = timingApple_560x384_60hz;
	timingApple13 = timingApple_640x480_67hz;
	timingApple13x = timingApple_640x400_67hz;
	timingAppleVGA = timingVESA_640x480_60hz;
	timingApple15 = timingApple_640x870_75hz;
	timingApple15x = timingApple_640x818_75hz;
	timingApple16 = timingApple_832x624_75hz;
	timingAppleSVGA = timingVESA_800x600_56hz;
	timingApple1Ka = timingVESA_1024x768_60hz;
	timingApple1Kb = timingVESA_1024x768_70hz;
	timingApple19 = timingApple_1024x768_75hz;
	timingApple21 = timingApple_1152x870_75hz;
	timingSony_1900x1200_74hz = 530;  { 1900x1200 (74 Hz) Sony timing (pixel clock is 236.25 Mhz dot clock). }
	timingSony_1900x1200_76hz = timingSony_1920x1200_76hz; { 1900x1200 (76 Hz) Sony timing (pixel clock is 245.48 Mhz dot clock). }

{ csConnectFlags values in VDDisplayConnectInfo }
const
	kAllModesValid = 0;    { All modes not trimmed by primary init are good close enough to try }
	kAllModesSafe = 1;    { All modes not trimmed by primary init are know to be safe }
	kReportsTagging = 2;    { Can detect tagged displays (to identify smart monitors) }
	kHasDirectConnection = 3;    { True implies that driver can talk directly to device (e.g. serial data link via sense lines) }
	kIsMonoDev = 4;    { Says whether there’s an RGB (0) or Monochrome (1) connection. }
	kUncertainConnection = 5;    { There may not be a display (no sense lines?). }
	kTaggingInfoNonStandard = 6;    { Set when csConnectTaggedType/csConnectTaggedData are non-standard (i.e., not the Apple CRT sense codes). }
	kReportsDDCConnection = 7;    { Card can do ddc (set kHasDirectConnect && kHasDDCConnect if you actually found a ddc display). }
	kHasDDCConnection = 8;    { Card has ddc connect now. }
	kConnectionInactive = 9;    { Set when the connection is NOT currently active (generally used in a multiconnection environment). }
	kDependentConnection = 10;   { Set when some ascpect of THIS connection depends on another (will generally be set in a kModeSimulscan environment). }
	kBuiltInConnection = 11;   { Set when connection is KNOWN to be built-in (this is not the same as kHasDirectConnection). }
	kOverrideConnection = 12;   { Set when the reported connection is not the true one, but is one that has been forced through a SetConnection call }
	kFastCheckForDDC = 13;   { Set when all 3 are true: 1) sense codes indicate DDC display could be attached 2) attempted fast check 3) DDC failed }
	kReportsHotPlugging = 14;    { Detects and reports hot pluggging on connector (via VSL also implies DDC will be up to date w/o force read) }


{ csDisplayType values in VDDisplayConnectInfo }
const
	kUnknownConnect = 1;    { Not sure how we’ll use this, but seems like a good idea. }
	kPanelConnect = 2;    { For use with fixed-in-place LCD panels. }
	kPanelTFTConnect = 2;    { Alias for kPanelConnect }
	kFixedModeCRTConnect = 3;    {  For use with fixed-mode (i.e., very limited range) displays. }
	kMultiModeCRT1Connect = 4;    { 320x200 maybe, 12" maybe, 13" (default), 16" certain, 19" maybe, 21" maybe }
	kMultiModeCRT2Connect = 5;    { 320x200 maybe, 12" maybe, 13" certain, 16" (default), 19" certain, 21" maybe }
	kMultiModeCRT3Connect = 6;    { 320x200 maybe, 12" maybe, 13" certain, 16" certain, 19" default, 21" certain }
	kMultiModeCRT4Connect = 7;    { Expansion to large multi mode (not yet used) }
	kModelessConnect = 8;    { Expansion to modeless model (not yet used) }
	kFullPageConnect = 9;    { 640x818 (to get 8bpp in 512K case) and 640x870 (these two only) }
	kVGAConnect = 10;   { 640x480 VGA default -- question everything else }
	kNTSCConnect = 11;   { NTSC ST (default), FF, STconv, FFconv }
	kPALConnect = 12;   { PAL ST (default), FF, STconv, FFconv }
	kHRConnect = 13;   { Straight-6 connect -- 640x480 and 640x400 (to get 8bpp in 256K case) (these two only) }
	kPanelFSTNConnect = 14;   { For use with fixed-in-place LCD FSTN (aka “Supertwist”) panels }
	kMonoTwoPageConnect = 15;   { 1152x870 Apple color two-page display }
	kColorTwoPageConnect = 16;   { 1152x870 Apple B&W two-page display }
	kColor16Connect = 17;   { 832x624 Apple B&W two-page display }
	kColor19Connect = 18;   { 1024x768 Apple B&W two-page display }
	kGenericCRT = 19;   { Indicates nothing except that connection is CRT in nature. }
	kGenericLCD = 20;   { Indicates nothing except that connection is LCD in nature. }
	kDDCConnect = 21;   { DDC connection, always set kHasDDCConnection }
	kNoConnect = 22;    { No display is connected - load sensing or similar level of hardware detection is assumed (used by resident drivers that support hot plugging when nothing is currently connected) }

{ csTimingFlags values in VDTimingInfoRec }
const
<<<<<<< HEAD
=======
const
	sixteenBitMode = 132;  {Id of SixteenBitMode Parameter list.}
	thirtyTwoBitMode = 133;  {Id of ThirtyTwoBitMode Parameter list.}
	firstVidMode = 128;  {The new, better way to do the above. }
	secondVidMode = 129;  { QuickDraw only supports six video }
	thirdVidMode = 130;  { at this time.      }
	fourthVidMode = 131;
	fifthVidMode = 132;
	sixthVidMode = 133;
	spGammaDir = 64;
	spVidNamesDir = 65;


{ csTimingFormat values in VDTimingInfo }
{ look in the declaration rom for timing info }
const
	kDeclROMtables = FourCharCode('decl');
	kDetailedTimingFormat = FourCharCode('arba'); { Timing is a detailed timing}

{ Size of a block of EDID (Extended Display Identification Data) }
const
	kDDCBlockSize = 128;

{ ddcBlockType constants}
const
	kDDCBlockTypeEDID = 0;     { EDID block type. }

{ ddcFlags constants}
const
	kDDCForceReadBit = 0;    { Force a new read of the EDID. }
	kDDCForceReadMask = 1 shl kDDCForceReadBit; { Mask for kddcForceReadBit. }


{ Timing mode constants for Display Manager MultiMode support
    Corresponding   .h equates are in Video.h
                    .a equates are in Video.a
                    .r equates are in DepVideoEqu.r
    
    The second enum is the old names (for compatibility).
    The first enum is the new names.
}
const
	timingInvalid = 0;    {    Unknown timing… force user to confirm. }
	timingInvalid_SM_T24 = 8;    {    Work around bug in SM Thunder24 card.}
	timingApple_FixedRateLCD = 42;   {    Lump all fixed-rate LCDs into one category.}
	timingApple_512x384_60hz = 130;  {  512x384  (60 Hz) Rubik timing. }
	timingApple_560x384_60hz = 135;  {  560x384  (60 Hz) Rubik-560 timing. }
	timingApple_640x480_67hz = 140;  {  640x480  (67 Hz) HR timing. }
	timingApple_640x400_67hz = 145;  {  640x400  (67 Hz) HR-400 timing. }
	timingVESA_640x480_60hz = 150;  {  640x480  (60 Hz) VGA timing. }
	timingVESA_640x480_72hz = 152;  {  640x480  (72 Hz) VGA timing. }
	timingVESA_640x480_75hz = 154;  {  640x480  (75 Hz) VGA timing. }
	timingVESA_640x480_85hz = 158;  {  640x480  (85 Hz) VGA timing. }
	timingGTF_640x480_120hz = 159;  {  640x480  (120 Hz) VESA Generalized Timing Formula }
	timingApple_640x870_75hz = 160;  {  640x870  (75 Hz) FPD timing.}
	timingApple_640x818_75hz = 165;  {  640x818  (75 Hz) FPD-818 timing.}
	timingApple_832x624_75hz = 170;  {  832x624  (75 Hz) GoldFish timing.}
	timingVESA_800x600_56hz = 180;  {  800x600  (56 Hz) SVGA timing. }
	timingVESA_800x600_60hz = 182;  {  800x600  (60 Hz) SVGA timing. }
	timingVESA_800x600_72hz = 184;  {  800x600  (72 Hz) SVGA timing. }
	timingVESA_800x600_75hz = 186;  {  800x600  (75 Hz) SVGA timing. }
	timingVESA_800x600_85hz = 188;  {  800x600  (85 Hz) SVGA timing. }
	timingVESA_1024x768_60hz = 190;  { 1024x768  (60 Hz) VESA 1K-60Hz timing. }
	timingVESA_1024x768_70hz = 200;  { 1024x768  (70 Hz) VESA 1K-70Hz timing. }
	timingVESA_1024x768_75hz = 204;  { 1024x768  (75 Hz) VESA 1K-75Hz timing (very similar to timingApple_1024x768_75hz). }
	timingVESA_1024x768_85hz = 208;  { 1024x768  (85 Hz) VESA timing. }
	timingApple_1024x768_75hz = 210;  { 1024x768  (75 Hz) Apple 19" RGB. }
	timingApple_1152x870_75hz = 220;  { 1152x870  (75 Hz) Apple 21" RGB. }
	timingAppleNTSC_ST = 230;  {  512x384  (60 Hz, interlaced, non-convolved). }
	timingAppleNTSC_FF = 232;  {  640x480  (60 Hz, interlaced, non-convolved). }
	timingAppleNTSC_STconv = 234;  {  512x384  (60 Hz, interlaced, convolved). }
	timingAppleNTSC_FFconv = 236;  {  640x480  (60 Hz, interlaced, convolved). }
	timingApplePAL_ST = 238;  {  640x480  (50 Hz, interlaced, non-convolved). }
	timingApplePAL_FF = 240;  {  768x576  (50 Hz, interlaced, non-convolved). }
	timingApplePAL_STconv = 242;  {  640x480  (50 Hz, interlaced, convolved). }
	timingApplePAL_FFconv = 244;  {  768x576  (50 Hz, interlaced, convolved). }
	timingVESA_1280x960_75hz = 250;  { 1280x960  (75 Hz) }
	timingVESA_1280x960_60hz = 252;  { 1280x960  (60 Hz) }
	timingVESA_1280x960_85hz = 254;  { 1280x960  (85 Hz) }
	timingVESA_1280x1024_60hz = 260;  { 1280x1024 (60 Hz) }
	timingVESA_1280x1024_75hz = 262;  { 1280x1024 (75 Hz) }
	timingVESA_1280x1024_85hz = 268;  { 1280x1024 (85 Hz) }
	timingVESA_1600x1200_60hz = 280;  { 1600x1200 (60 Hz) VESA timing. }
	timingVESA_1600x1200_65hz = 282;  { 1600x1200 (65 Hz) VESA timing. }
	timingVESA_1600x1200_70hz = 284;  { 1600x1200 (70 Hz) VESA timing. }
	timingVESA_1600x1200_75hz = 286;  { 1600x1200 (75 Hz) VESA timing (pixel clock is 189.2 Mhz dot clock). }
	timingVESA_1600x1200_80hz = 288;  { 1600x1200 (80 Hz) VESA timing (pixel clock is 216>? Mhz dot clock) - proposed only. }
	timingVESA_1600x1200_85hz = 289;  { 1600x1200 (85 Hz) VESA timing (pixel clock is 229.5 Mhz dot clock). }
	timingVESA_1792x1344_60hz = 296;  { 1792x1344 (60 Hz) VESA timing (204.75 Mhz dot clock). }
	timingVESA_1792x1344_75hz = 298;  { 1792x1344 (75 Hz) VESA timing (261.75 Mhz dot clock). }
	timingVESA_1856x1392_60hz = 300;  { 1856x1392 (60 Hz) VESA timing (218.25 Mhz dot clock). }
	timingVESA_1856x1392_75hz = 302;  { 1856x1392 (75 Hz) VESA timing (288 Mhz dot clock). }
	timingVESA_1920x1440_60hz = 304;  { 1920x1440 (60 Hz) VESA timing (234 Mhz dot clock). }
	timingVESA_1920x1440_75hz = 306;  { 1920x1440 (75 Hz) VESA timing (297 Mhz dot clock). }
	timingSMPTE240M_60hz = 400;  { 60Hz V, 33.75KHz H, interlaced timing, 16:9 aspect, typical resolution of 1920x1035. }
	timingFilmRate_48hz = 410;  { 48Hz V, 25.20KHz H, non-interlaced timing, typical resolution of 640x480. }
	timingSony_1600x1024_76hz = 500;  { 1600x1024 (76 Hz) Sony timing (pixel clock is 170.447 Mhz dot clock). }
	timingSony_1920x1080_60hz = 510;  { 1920x1080 (60 Hz) Sony timing (pixel clock is 159.84 Mhz dot clock). }
	timingSony_1920x1080_72hz = 520;  { 1920x1080 (72 Hz) Sony timing (pixel clock is 216.023 Mhz dot clock). }
	timingSony_1920x1200_76hz = 540;  { 1900x1200 (76 Hz) Sony timing (pixel clock is 243.20 Mhz dot clock). }
	timingApple_0x0_0hz_Offline = 550;   { Indicates that this timing will take the display off-line and remove it from the system. }


{ Deprecated timing names.}
const
	timingApple12 = timingApple_512x384_60hz;
	timingApple12x = timingApple_560x384_60hz;
	timingApple13 = timingApple_640x480_67hz;
	timingApple13x = timingApple_640x400_67hz;
	timingAppleVGA = timingVESA_640x480_60hz;
	timingApple15 = timingApple_640x870_75hz;
	timingApple15x = timingApple_640x818_75hz;
	timingApple16 = timingApple_832x624_75hz;
	timingAppleSVGA = timingVESA_800x600_56hz;
	timingApple1Ka = timingVESA_1024x768_60hz;
	timingApple1Kb = timingVESA_1024x768_70hz;
	timingApple19 = timingApple_1024x768_75hz;
	timingApple21 = timingApple_1152x870_75hz;
	timingSony_1900x1200_74hz = 530;  { 1900x1200 (74 Hz) Sony timing (pixel clock is 236.25 Mhz dot clock). }
	timingSony_1900x1200_76hz = timingSony_1920x1200_76hz; { 1900x1200 (76 Hz) Sony timing (pixel clock is 245.48 Mhz dot clock). }

{ csConnectFlags values in VDDisplayConnectInfo }
const
	kAllModesValid = 0;    { All modes not trimmed by primary init are good close enough to try }
	kAllModesSafe = 1;    { All modes not trimmed by primary init are know to be safe }
	kReportsTagging = 2;    { Can detect tagged displays (to identify smart monitors) }
	kHasDirectConnection = 3;    { True implies that driver can talk directly to device (e.g. serial data link via sense lines) }
	kIsMonoDev = 4;    { Says whether there’s an RGB (0) or Monochrome (1) connection. }
	kUncertainConnection = 5;    { There may not be a display (no sense lines?). }
	kTaggingInfoNonStandard = 6;    { Set when csConnectTaggedType/csConnectTaggedData are non-standard (i.e., not the Apple CRT sense codes). }
	kReportsDDCConnection = 7;    { Card can do ddc (set kHasDirectConnect && kHasDDCConnect if you actually found a ddc display). }
	kHasDDCConnection = 8;    { Card has ddc connect now. }
	kConnectionInactive = 9;    { Set when the connection is NOT currently active (generally used in a multiconnection environment). }
	kDependentConnection = 10;   { Set when some ascpect of THIS connection depends on another (will generally be set in a kModeSimulscan environment). }
	kBuiltInConnection = 11;   { Set when connection is KNOWN to be built-in (this is not the same as kHasDirectConnection). }
	kOverrideConnection = 12;   { Set when the reported connection is not the true one, but is one that has been forced through a SetConnection call }
	kFastCheckForDDC = 13;   { Set when all 3 are true: 1) sense codes indicate DDC display could be attached 2) attempted fast check 3) DDC failed }
	kReportsHotPlugging = 14;    { Detects and reports hot pluggging on connector (via VSL also implies DDC will be up to date w/o force read) }


{ csDisplayType values in VDDisplayConnectInfo }
const
	kUnknownConnect = 1;    { Not sure how we’ll use this, but seems like a good idea. }
	kPanelConnect = 2;    { For use with fixed-in-place LCD panels. }
	kPanelTFTConnect = 2;    { Alias for kPanelConnect }
	kFixedModeCRTConnect = 3;    {  For use with fixed-mode (i.e., very limited range) displays. }
	kMultiModeCRT1Connect = 4;    { 320x200 maybe, 12" maybe, 13" (default), 16" certain, 19" maybe, 21" maybe }
	kMultiModeCRT2Connect = 5;    { 320x200 maybe, 12" maybe, 13" certain, 16" (default), 19" certain, 21" maybe }
	kMultiModeCRT3Connect = 6;    { 320x200 maybe, 12" maybe, 13" certain, 16" certain, 19" default, 21" certain }
	kMultiModeCRT4Connect = 7;    { Expansion to large multi mode (not yet used) }
	kModelessConnect = 8;    { Expansion to modeless model (not yet used) }
	kFullPageConnect = 9;    { 640x818 (to get 8bpp in 512K case) and 640x870 (these two only) }
	kVGAConnect = 10;   { 640x480 VGA default -- question everything else }
	kNTSCConnect = 11;   { NTSC ST (default), FF, STconv, FFconv }
	kPALConnect = 12;   { PAL ST (default), FF, STconv, FFconv }
	kHRConnect = 13;   { Straight-6 connect -- 640x480 and 640x400 (to get 8bpp in 256K case) (these two only) }
	kPanelFSTNConnect = 14;   { For use with fixed-in-place LCD FSTN (aka “Supertwist”) panels }
	kMonoTwoPageConnect = 15;   { 1152x870 Apple color two-page display }
	kColorTwoPageConnect = 16;   { 1152x870 Apple B&W two-page display }
	kColor16Connect = 17;   { 832x624 Apple B&W two-page display }
	kColor19Connect = 18;   { 1024x768 Apple B&W two-page display }
	kGenericCRT = 19;   { Indicates nothing except that connection is CRT in nature. }
	kGenericLCD = 20;   { Indicates nothing except that connection is LCD in nature. }
	kDDCConnect = 21;   { DDC connection, always set kHasDDCConnection }
	kNoConnect = 22;    { No display is connected - load sensing or similar level of hardware detection is assumed (used by resident drivers that support hot plugging when nothing is currently connected) }

{ csTimingFlags values in VDTimingInfoRec }
const
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	kModeValid = 0;    { Says that this mode should NOT be trimmed. }
	kModeSafe = 1;    { This mode does not need confirmation }
	kModeDefault = 2;    { This is the default mode for this type of connection }
	kModeShowNow = 3;    { This mode should always be shown (even though it may require a confirm) }
	kModeNotResize = 4;    { This mode should not be used to resize the display (eg. mode selects a different connector on card) }
	kModeRequiresPan = 5;    { This mode has more pixels than are actually displayed }
	kModeInterlaced = 6;    { This mode is interlaced (single pixel lines look bad). }
	kModeShowNever = 7;    { This mode should not be shown in the user interface. }
	kModeSimulscan = 8;    { Indicates that more than one display connection can be driven from a single framebuffer controller. }
	kModeNotPreset = 9;    { Indicates that the timing is not a factory preset for the current display (geometry may need correction) }
	kModeBuiltIn = 10;   { Indicates that the display mode is for the built-in connect only (on multiconnect devices like the PB 3400) Only the driver is quieried }
	kModeStretched = 11;    { Indicates that the display mode will be stretched/distorted to match the display aspect ratio }

{ csDepthFlags in VDVideoParametersInfoRec }
const
	kDepthDependent = 0;     { Says that this depth mode may cause dependent changes in other framebuffers (and . }

{ csResolutionFlags bit flags for VDResolutionInfoRec }
const
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	kResolutionHasMultipleDepthSizes = 0;  { Says that this mode has different csHorizontalPixels, csVerticalLines at different depths (usually slightly larger at lower depths) }


const
{    Power Mode constants for VDPowerStateRec.powerState.  Note the numeric order does not match the power state order }
	kAVPowerOff = 0;    { Power fully off}
	kAVPowerStandby = 1;
	kAVPowerSuspend = 2;
	kAVPowerOn = 3;
	kHardwareSleep = 128;
	kHardwareWake = 129;
	kHardwareWakeFromSuspend = 130;
	kHardwareWakeToDoze = 131;
	kHardwareWakeToDozeFromSuspend = 132;

const
{    Power Mode masks and bits for VDPowerStateRec.powerFlags.  }
	kPowerStateNeedsRefresh = 0;    { When leaving this power mode, a display will need refreshing   }
	kPowerStateSleepAwareBit = 1;    { if gestaltPCCardDockingSelectorFix, Docking mgr checks this bit before checking kPowerStateSleepAllowedBit }
	kPowerStateSleepForbiddenBit = 2;    { if kPowerStateSleepAwareBit, Docking mgr checks this bit before sleeping }
	kPowerStateSleepCanPowerOffBit = 3;   { supports power down sleep (ie PCI power off)}
	kPowerStateSleepNoDPMSBit = 4;    { Bug #2425210.  Do not use DPMS with this display.}
	kPowerStateSleepWaketoDozeBit = 5;    { Supports Wake to Doze }
	kPowerStateNeedsRefreshMask = 1 shl kPowerStateNeedsRefresh;
	kPowerStateSleepAwareMask = 1 shl kPowerStateSleepAwareBit;
	kPowerStateSleepForbiddenMask = 1 shl kPowerStateSleepForbiddenBit;
	kPowerStateSleepCanPowerOffMask = 1 shl kPowerStateSleepCanPowerOffBit;
	kPowerStateSleepNoDPMSMask = 1 shl kPowerStateSleepNoDPMSBit;
	kPowerStateSleepWaketoDozeMask = 1 shl kPowerStateSleepWaketoDozeBit;


const
{ Control Codes }
	cscReset = 0;
	cscKillIO = 1;
	cscSetMode = 2;
	cscSetEntries = 3;
	cscSetGamma = 4;
	cscGrayPage = 5;
	cscGrayScreen = 5;
	cscSetGray = 6;
	cscSetInterrupt = 7;
	cscDirectSetEntries = 8;
	cscSetDefaultMode = 9;
	cscSwitchMode = 10;   { Takes a VDSwitchInfoPtr }
	cscSetSync = 11;   { Takes a VDSyncInfoPtr }
	cscSavePreferredConfiguration = 16;   { Takes a VDSwitchInfoPtr }
	cscSetHardwareCursor = 22;   { Takes a VDSetHardwareCursorPtr }
	cscDrawHardwareCursor = 23;   { Takes a VDDrawHardwareCursorPtr }
	cscSetConvolution = 24;   { Takes a VDConvolutionInfoPtr }
	cscSetPowerState = 25;   { Takes a VDPowerStatePtr }
	cscPrivateControlCall = 26;   { Takes a VDPrivateSelectorDataPtr }
	cscSetMultiConnect = 28;   { Takes a VDMultiConnectInfoPtr }
	cscSetClutBehavior = 29;   { Takes a VDClutBehavior }
	cscSetDetailedTiming = 31;   { Takes a VDDetailedTimingPtr }
	cscDoCommunication = 33;   { Takes a VDCommunicationPtr }
	cscProbeConnection = 34;   { Takes nil pointer (may generate a kFBConnectInterruptServiceType service interrupt)}
	cscUnusedCall = 127;   { This call used to expend the scrn resource.  Its imbedded data contains more control info }

const
{ Status Codes }
	cscGetMode = 2;
	cscGetEntries = 3;
	cscGetPageCnt = 4;
	cscGetPages = 4;    { This is what C&D 2 calls it. }
	cscGetPageBase = 5;
	cscGetBaseAddr = 5;    { This is what C&D 2 calls it. }
	cscGetGray = 6;
	cscGetInterrupt = 7;
	cscGetGamma = 8;
	cscGetDefaultMode = 9;
	cscGetCurMode = 10;   { Takes a VDSwitchInfoPtr }
	cscGetSync = 11;   { Takes a VDSyncInfoPtr }
	cscGetConnection = 12;   { Return information about the connection to the display }
	cscGetModeTiming = 13;   { Return timing info for a mode }
	cscGetModeBaseAddress = 14;   { Return base address information about a particular mode }
	cscGetScanProc = 15;   { QuickTime scan chasing routine }
	cscGetPreferredConfiguration = 16;   { Takes a VDSwitchInfoPtr }
	cscGetNextResolution = 17;   { Takes a VDResolutionInfoPtr }
	cscGetVideoParameters = 18;   { Takes a VDVideoParametersInfoPtr }
	cscGetGammaInfoList = 20;   { Takes a VDGetGammaListPtr }
	cscRetrieveGammaTable = 21;   { Takes a VDRetrieveGammaPtr }
	cscSupportsHardwareCursor = 22;   { Takes a VDSupportsHardwareCursorPtr }
	cscGetHardwareCursorDrawState = 23;   { Takes a VDHardwareCursorDrawStatePtr }
	cscGetConvolution = 24;   { Takes a VDConvolutionInfoPtr }
	cscGetPowerState = 25;   { Takes a VDPowerStatePtr }
	cscPrivateStatusCall = 26;   { Takes a VDPrivateSelectorDataPtr }
	cscGetDDCBlock = 27;   { Takes a VDDDCBlockPtr  }
	cscGetMultiConnect = 28;   { Takes a VDMultiConnectInfoPtr }
	cscGetClutBehavior = 29;   { Takes a VDClutBehaviorPtr }
	cscGetTimingRanges = 30;   { Takes a VDDisplayTimingRangePtr }
	cscGetDetailedTiming = 31;   { Takes a VDDetailedTimingPtr }
	cscGetCommunicationInfo = 32;    { Takes a VDCommunicationInfoPtr }

{ Bit definitions for the Get/Set Sync call}
const
	kDisableHorizontalSyncBit = 0;
	kDisableVerticalSyncBit = 1;
	kDisableCompositeSyncBit = 2;
	kEnableSyncOnBlue = 3;
	kEnableSyncOnGreen = 4;
	kEnableSyncOnRed = 5;
	kNoSeparateSyncControlBit = 6;
	kTriStateSyncBit = 7;
	kHorizontalSyncMask = $01;
	kVerticalSyncMask = $02;
	kCompositeSyncMask = $04;
	kDPMSSyncMask = $07;
	kTriStateSyncMask = $80;
	kSyncOnBlueMask = $08;
	kSyncOnGreenMask = $10;
	kSyncOnRedMask = $20;
	kSyncOnMask = $38;

const
{    Power Mode constants for translating DPMS modes to Get/SetSync calls.  }
	kDPMSSyncOn = 0;
	kDPMSSyncStandby = 1;
	kDPMSSyncSuspend = 2;
	kDPMSSyncOff = 7;

{ Bit definitions for the Get/Set Convolution call}
const
	kConvolved = 0;
	kLiveVideoPassThru = 1;
	kConvolvedMask = $01;
	kLiveVideoPassThruMask = $02;


type
	VPBlock = record
		vpBaseOffset: SIGNEDLONG;           {Offset to page zero of video RAM (From minorBaseOS).}
		vpRowBytes: SInt16;             {Width of each row of video memory.}
		vpBounds: Rect;               {BoundsRect for the video display (gives dimensions).}
		vpVersion: SInt16;              {PixelMap version number.}
		vpPackType: SInt16;
		vpPackSize: SIGNEDLONG;
		vpHRes: SIGNEDLONG;                 {Horizontal resolution of the device (pixels per inch).}
		vpVRes: SIGNEDLONG;                 {Vertical resolution of the device (pixels per inch).}
		vpPixelType: SInt16;            {Defines the pixel type.}
		vpPixelSize: SInt16;            {Number of bits in pixel.}
		vpCmpCount: SInt16;             {Number of components in pixel.}
		vpCmpSize: SInt16;              {Number of bits per component}
		vpPlaneBytes: SIGNEDLONG;           {Offset from one plane to the next.}
	end;
	VPBlockPtr = ^VPBlock;
type
	VDEntryRecordPtr = ^VDEntryRecord;
	VDEntryRecord = record
		csTable: Ptr;                {(long) pointer to color table entry=value, r,g,b:INTEGER}
	end;
type
	VDEntRecPtr = VDEntryRecordPtr;
{ Parm block for SetGray control call }
type
	VDGrayRecordPtr = ^VDGrayRecord;
	VDGrayRecord = record
		csMode: Boolean;                 {Same as GDDevType value (0=color, 1=mono)}
		filler: SInt8;
	end;
type
	VDGrayPtr = VDGrayRecordPtr;
{ Parm block for SetInterrupt call }
type
	VDFlagRecordPtr = ^VDFlagRecord;
	VDFlagRecord = record
		csMode: SInt8;
		filler: SInt8;
	end;
type
	VDFlagRecPtr = VDFlagRecordPtr;
{ Parm block for SetEntries control call }
type
	VDSetEntryRecordPtr = ^VDSetEntryRecord;
	VDSetEntryRecord = record
		csTable: ColorSpecPtr;                {Pointer to an array of color specs}
		csStart: SInt16;                {Which spec in array to start with, or -1}
		csCount: SInt16;                {Number of color spec entries to set}
	end;
type
	VDSetEntryPtr = VDSetEntryRecordPtr;
{ Parm block for SetGamma control call }
type
	VDGammaRecordPtr = ^VDGammaRecord;
	VDGammaRecord = record
		csGTable: Ptr;               {pointer to gamma table}
	end;
type
	VDGamRecPtr = VDGammaRecordPtr;
	VDBaseAddressInfoRecPtr = ^VDBaseAddressInfoRec;
	VDBaseAddressInfoRec = record
		csDevData: SIGNEDLONG;              { LONGINT - (long) timing mode }
		csDevBase: SIGNEDLONG;              { LONGINT - (long) base address of the mode }
		csModeReserved: SInt16;         { INTEGER - (short) will some day be the depth }
		csModeBase: SIGNEDLONG;             { LONGINT - (long) reserved }
	end;
type
	VDBaseAddressInfoPtr = VDBaseAddressInfoRecPtr;
	VDSwitchInfoRecPtr = ^VDSwitchInfoRec;
	VDSwitchInfoRec = record
		csMode: UInt16;                 {(word) mode depth}
		csData: UNSIGNEDLONG;                 {(long) functional sResource of mode}
		csPage: UInt16;                 {(word) page to switch in}
		csBaseAddr: Ptr;             {(long) base address of page (return value)}
		csReserved: UNSIGNEDLONG;             {(long) Reserved (set to 0) }
	end;
type
	VDSwitchInfoPtr = VDSwitchInfoRecPtr;
	VDTimingInfoRecPtr = ^VDTimingInfoRec;
	VDTimingInfoRec = record
		csTimingMode: UNSIGNEDLONG;           { LONGINT - (long) timing mode (a la InitGDevice) }
		csTimingReserved: UNSIGNEDLONG;       { LONGINT - (long) reserved }
		csTimingFormat: UNSIGNEDLONG;         { LONGINT - (long) what format is the timing info }
		csTimingData: UNSIGNEDLONG;           { LONGINT - (long) data supplied by driver }
		csTimingFlags: UNSIGNEDLONG;          { LONGINT - (long) mode within device }
	end;
type
	VDTimingInfoPtr = VDTimingInfoRecPtr;
	VDDisplayConnectInfoRecPtr = ^VDDisplayConnectInfoRec;
	VDDisplayConnectInfoRec = record
		csDisplayType: UInt16;          { INTEGER - (word) Type of display connected }
		csConnectTaggedType: UInt8;    { BYTE - type of tagging }
		csConnectTaggedData: UInt8;    { BYTE - tagging data }
		csConnectFlags: UNSIGNEDLONG;         { LONGINT - (long) tell us about the connection }
		csDisplayComponent: UNSIGNEDLONG;     { LONGINT - (long) if the card has a direct connection to the display, it returns the display component here (FUTURE) }
		csConnectReserved: UNSIGNEDLONG;      { LONGINT - (long) reserved }
	end;
type
	VDDisplayConnectInfoPtr = VDDisplayConnectInfoRecPtr;
	VDMultiConnectInfoRecPtr = ^VDMultiConnectInfoRec;
	VDMultiConnectInfoRec = record
		csDisplayCountOrNumber: UNSIGNEDLONG; { For GetMultiConnect, returns count n of 1..n connections; otherwise, indicates the ith connection.}
		csConnectInfo: VDDisplayConnectInfoRec;     { Standard VDDisplayConnectionInfo for connection i.}
	end;
type
	VDMultiConnectInfoPtr = VDMultiConnectInfoRecPtr;
{ RawSenseCode
    This abstract data type is not exactly abstract.  Rather, it is merely enumerated constants
    for the possible raw sense code values when 'standard' sense code hardware is implemented.

    For 'standard' sense code hardware, the raw sense is obtained as follows:
        • Instruct the frame buffer controller NOT to actively drive any of the monitor sense lines
        • Read the state of the monitor sense lines 2, 1, and 0.  (2 is the MSB, 0 the LSB)
<<<<<<< HEAD

    IMPORTANT Note: 
    When the 'kTaggingInfoNonStandard' bit of 'csConnectFlags' is FALSE, then these constants 
    are valid 'csConnectTaggedType' values in 'VDDisplayConnectInfo' 

}
type
	RawSenseCode = UInt8;
const
	kRSCZero = 0;
	kRSCOne = 1;
	kRSCTwo = 2;
	kRSCThree = 3;
	kRSCFour = 4;
	kRSCFive = 5;
	kRSCSix = 6;
	kRSCSeven = 7;


{ ExtendedSenseCode
    This abstract data type is not exactly abstract.  Rather, it is merely enumerated constants
    for the values which are possible when the extended sense algorithm is applied to hardware
    which implements 'standard' sense code hardware.

    For 'standard' sense code hardware, the extended sense code algorithm is as follows:
    (Note:  as described here, sense line 'A' corresponds to '2', 'B' to '1', and 'C' to '0')
        • Drive sense line 'A' low and read the values of 'B' and 'C'.  
        • Drive sense line 'B' low and read the values of 'A' and 'C'.
        • Drive sense line 'C' low and read the values of 'A' and 'B'.

=======

    IMPORTANT Note: 
    When the 'kTaggingInfoNonStandard' bit of 'csConnectFlags' is FALSE, then these constants 
    are valid 'csConnectTaggedType' values in 'VDDisplayConnectInfo' 

}
type
	RawSenseCode = UInt8;
const
	kRSCZero = 0;
	kRSCOne = 1;
	kRSCTwo = 2;
	kRSCThree = 3;
	kRSCFour = 4;
	kRSCFive = 5;
	kRSCSix = 6;
	kRSCSeven = 7;


{ ExtendedSenseCode
    This abstract data type is not exactly abstract.  Rather, it is merely enumerated constants
    for the values which are possible when the extended sense algorithm is applied to hardware
    which implements 'standard' sense code hardware.

    For 'standard' sense code hardware, the extended sense code algorithm is as follows:
    (Note:  as described here, sense line 'A' corresponds to '2', 'B' to '1', and 'C' to '0')
        • Drive sense line 'A' low and read the values of 'B' and 'C'.  
        • Drive sense line 'B' low and read the values of 'A' and 'C'.
        • Drive sense line 'C' low and read the values of 'A' and 'B'.

>>>>>>> origin/cpstrnew
    In this way, a six-bit number of the form BC/AC/AB is generated. 

    IMPORTANT Note: 
    When the 'kTaggingInfoNonStandard' bit of 'csConnectFlags' is FALSE, then these constants 
    are valid 'csConnectTaggedData' values in 'VDDisplayConnectInfo' 

}
type
	ExtendedSenseCode = UInt8;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
const
	kESCZero21Inch = $00; { 21" RGB                     }
	kESCOnePortraitMono = $14; { Portrait Monochrome              }
	kESCTwo12Inch = $21; { 12" RGB                    }
	kESCThree21InchRadius = $31; { 21" RGB (Radius)               }
	kESCThree21InchMonoRadius = $34; { 21" Monochrome (Radius)           }
	kESCThree21InchMono = $35; { 21" Monochrome               }
	kESCFourNTSC = $0A; { NTSC                     }
	kESCFivePortrait = $1E; { Portrait RGB              }
	kESCSixMSB1 = $03; { MultiScan Band-1 (12" thru 1Six")  }
	kESCSixMSB2 = $0B; { MultiScan Band-2 (13" thru 19")       }
	kESCSixMSB3 = $23; { MultiScan Band-3 (13" thru 21")       }
	kESCSixStandard = $2B; { 13"/14" RGB or 12" Monochrome   }
	kESCSevenPAL = $00; { PAL                        }
	kESCSevenNTSC = $14; { NTSC                     }
	kESCSevenVGA = $17; { VGA                        }
	kESCSeven16Inch = $2D; { 16" RGB (GoldFish)               }
	kESCSevenPALAlternate = $30; { PAL (Alternate)                }
	kESCSeven19Inch = $3A; { Third-Party 19”                 }
	kESCSevenDDC = $3E; { DDC display                   }
	kESCSevenNoDisplay = $3F;  { No display connected           }

{ DepthMode
    This abstract data type is used to to reference RELATIVE pixel depths.
    Its definition is largely derived from its past usage, analogous to 'xxxVidMode'

    Bits per pixel DOES NOT directly map to 'DepthMode'  For example, on some
    graphics hardware, 'kDepthMode1' may represent 1 BPP, whereas on other
    hardware, 'kDepthMode1' may represent 8BPP.

    DepthMode IS considered to be ordinal, i.e., operations such as <, >, ==, etc.
    behave as expected.  The values of the constants which comprise the set are such
    that 'kDepthMode4 < kDepthMode6' behaves as expected.
}
type
	DepthMode = UInt16;
const
=======
const
	kESCZero21Inch = $00; { 21" RGB                     }
	kESCOnePortraitMono = $14; { Portrait Monochrome              }
	kESCTwo12Inch = $21; { 12" RGB                    }
	kESCThree21InchRadius = $31; { 21" RGB (Radius)               }
	kESCThree21InchMonoRadius = $34; { 21" Monochrome (Radius)           }
	kESCThree21InchMono = $35; { 21" Monochrome               }
	kESCFourNTSC = $0A; { NTSC                     }
	kESCFivePortrait = $1E; { Portrait RGB              }
	kESCSixMSB1 = $03; { MultiScan Band-1 (12" thru 1Six")  }
	kESCSixMSB2 = $0B; { MultiScan Band-2 (13" thru 19")       }
	kESCSixMSB3 = $23; { MultiScan Band-3 (13" thru 21")       }
	kESCSixStandard = $2B; { 13"/14" RGB or 12" Monochrome   }
	kESCSevenPAL = $00; { PAL                        }
	kESCSevenNTSC = $14; { NTSC                     }
	kESCSevenVGA = $17; { VGA                        }
	kESCSeven16Inch = $2D; { 16" RGB (GoldFish)               }
	kESCSevenPALAlternate = $30; { PAL (Alternate)                }
	kESCSeven19Inch = $3A; { Third-Party 19”                 }
	kESCSevenDDC = $3E; { DDC display                   }
	kESCSevenNoDisplay = $3F;  { No display connected           }

{ DepthMode
    This abstract data type is used to to reference RELATIVE pixel depths.
    Its definition is largely derived from its past usage, analogous to 'xxxVidMode'

    Bits per pixel DOES NOT directly map to 'DepthMode'  For example, on some
    graphics hardware, 'kDepthMode1' may represent 1 BPP, whereas on other
    hardware, 'kDepthMode1' may represent 8BPP.

    DepthMode IS considered to be ordinal, i.e., operations such as <, >, ==, etc.
    behave as expected.  The values of the constants which comprise the set are such
    that 'kDepthMode4 < kDepthMode6' behaves as expected.
}
type
	DepthMode = UInt16;
const
>>>>>>> graemeg/cpstrnew
=======
const
	kESCZero21Inch = $00; { 21" RGB                     }
	kESCOnePortraitMono = $14; { Portrait Monochrome              }
	kESCTwo12Inch = $21; { 12" RGB                    }
	kESCThree21InchRadius = $31; { 21" RGB (Radius)               }
	kESCThree21InchMonoRadius = $34; { 21" Monochrome (Radius)           }
	kESCThree21InchMono = $35; { 21" Monochrome               }
	kESCFourNTSC = $0A; { NTSC                     }
	kESCFivePortrait = $1E; { Portrait RGB              }
	kESCSixMSB1 = $03; { MultiScan Band-1 (12" thru 1Six")  }
	kESCSixMSB2 = $0B; { MultiScan Band-2 (13" thru 19")       }
	kESCSixMSB3 = $23; { MultiScan Band-3 (13" thru 21")       }
	kESCSixStandard = $2B; { 13"/14" RGB or 12" Monochrome   }
	kESCSevenPAL = $00; { PAL                        }
	kESCSevenNTSC = $14; { NTSC                     }
	kESCSevenVGA = $17; { VGA                        }
	kESCSeven16Inch = $2D; { 16" RGB (GoldFish)               }
	kESCSevenPALAlternate = $30; { PAL (Alternate)                }
	kESCSeven19Inch = $3A; { Third-Party 19”                 }
	kESCSevenDDC = $3E; { DDC display                   }
	kESCSevenNoDisplay = $3F;  { No display connected           }

{ DepthMode
    This abstract data type is used to to reference RELATIVE pixel depths.
    Its definition is largely derived from its past usage, analogous to 'xxxVidMode'

    Bits per pixel DOES NOT directly map to 'DepthMode'  For example, on some
    graphics hardware, 'kDepthMode1' may represent 1 BPP, whereas on other
    hardware, 'kDepthMode1' may represent 8BPP.

    DepthMode IS considered to be ordinal, i.e., operations such as <, >, ==, etc.
    behave as expected.  The values of the constants which comprise the set are such
    that 'kDepthMode4 < kDepthMode6' behaves as expected.
}
type
	DepthMode = UInt16;
const
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/cpstrnew
const
	kESCZero21Inch = $00; { 21" RGB                     }
	kESCOnePortraitMono = $14; { Portrait Monochrome              }
	kESCTwo12Inch = $21; { 12" RGB                    }
	kESCThree21InchRadius = $31; { 21" RGB (Radius)               }
	kESCThree21InchMonoRadius = $34; { 21" Monochrome (Radius)           }
	kESCThree21InchMono = $35; { 21" Monochrome               }
	kESCFourNTSC = $0A; { NTSC                     }
	kESCFivePortrait = $1E; { Portrait RGB              }
	kESCSixMSB1 = $03; { MultiScan Band-1 (12" thru 1Six")  }
	kESCSixMSB2 = $0B; { MultiScan Band-2 (13" thru 19")       }
	kESCSixMSB3 = $23; { MultiScan Band-3 (13" thru 21")       }
	kESCSixStandard = $2B; { 13"/14" RGB or 12" Monochrome   }
	kESCSevenPAL = $00; { PAL                        }
	kESCSevenNTSC = $14; { NTSC                     }
	kESCSevenVGA = $17; { VGA                        }
	kESCSeven16Inch = $2D; { 16" RGB (GoldFish)               }
	kESCSevenPALAlternate = $30; { PAL (Alternate)                }
	kESCSeven19Inch = $3A; { Third-Party 19”                 }
	kESCSevenDDC = $3E; { DDC display                   }
	kESCSevenNoDisplay = $3F;  { No display connected           }

{ DepthMode
    This abstract data type is used to to reference RELATIVE pixel depths.
    Its definition is largely derived from its past usage, analogous to 'xxxVidMode'

    Bits per pixel DOES NOT directly map to 'DepthMode'  For example, on some
    graphics hardware, 'kDepthMode1' may represent 1 BPP, whereas on other
    hardware, 'kDepthMode1' may represent 8BPP.

    DepthMode IS considered to be ordinal, i.e., operations such as <, >, ==, etc.
    behave as expected.  The values of the constants which comprise the set are such
    that 'kDepthMode4 < kDepthMode6' behaves as expected.
}
type
	DepthMode = UInt16;
const
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	kDepthMode1 = 128;
	kDepthMode2 = 129;
	kDepthMode3 = 130;
	kDepthMode4 = 131;
	kDepthMode5 = 132;
	kDepthMode6 = 133;

const
	kFirstDepthMode = 128;  { These constants are obsolete, and just included    }
	kSecondDepthMode = 129;  { for clients that have converted to the above     }
	kThirdDepthMode = 130;  { kDepthModeXXX constants.                }
	kFourthDepthMode = 131;
	kFifthDepthMode = 132;
	kSixthDepthMode = 133;
=======
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
const
	sixteenBitMode = 132;  {Id of SixteenBitMode Parameter list.}
	thirtyTwoBitMode = 133;  {Id of ThirtyTwoBitMode Parameter list.}
	firstVidMode = 128;  {The new, better way to do the above. }
	secondVidMode = 129;  { QuickDraw only supports six video }
	thirdVidMode = 130;  { at this time.      }
	fourthVidMode = 131;
	fifthVidMode = 132;
	sixthVidMode = 133;
	spGammaDir = 64;
	spVidNamesDir = 65;


{ csTimingFormat values in VDTimingInfo }
{ look in the declaration rom for timing info }
const
	kDeclROMtables = FourCharCode('decl');
	kDetailedTimingFormat = FourCharCode('arba'); { Timing is a detailed timing}

{ Size of a block of EDID (Extended Display Identification Data) }
const
	kDDCBlockSize = 128;

{ ddcBlockType constants}
const
	kDDCBlockTypeEDID = 0;     { EDID block type. }

{ ddcFlags constants}
const
	kDDCForceReadBit = 0;    { Force a new read of the EDID. }
	kDDCForceReadMask = 1 shl kDDCForceReadBit; { Mask for kddcForceReadBit. }


{ Timing mode constants for Display Manager MultiMode support
    Corresponding   .h equates are in Video.h
                    .a equates are in Video.a
                    .r equates are in DepVideoEqu.r
    
    The second enum is the old names (for compatibility).
    The first enum is the new names.
}
const
	timingInvalid = 0;    {    Unknown timing… force user to confirm. }
	timingInvalid_SM_T24 = 8;    {    Work around bug in SM Thunder24 card.}
	timingApple_FixedRateLCD = 42;   {    Lump all fixed-rate LCDs into one category.}
	timingApple_512x384_60hz = 130;  {  512x384  (60 Hz) Rubik timing. }
	timingApple_560x384_60hz = 135;  {  560x384  (60 Hz) Rubik-560 timing. }
	timingApple_640x480_67hz = 140;  {  640x480  (67 Hz) HR timing. }
	timingApple_640x400_67hz = 145;  {  640x400  (67 Hz) HR-400 timing. }
	timingVESA_640x480_60hz = 150;  {  640x480  (60 Hz) VGA timing. }
	timingVESA_640x480_72hz = 152;  {  640x480  (72 Hz) VGA timing. }
	timingVESA_640x480_75hz = 154;  {  640x480  (75 Hz) VGA timing. }
	timingVESA_640x480_85hz = 158;  {  640x480  (85 Hz) VGA timing. }
	timingGTF_640x480_120hz = 159;  {  640x480  (120 Hz) VESA Generalized Timing Formula }
	timingApple_640x870_75hz = 160;  {  640x870  (75 Hz) FPD timing.}
	timingApple_640x818_75hz = 165;  {  640x818  (75 Hz) FPD-818 timing.}
	timingApple_832x624_75hz = 170;  {  832x624  (75 Hz) GoldFish timing.}
	timingVESA_800x600_56hz = 180;  {  800x600  (56 Hz) SVGA timing. }
	timingVESA_800x600_60hz = 182;  {  800x600  (60 Hz) SVGA timing. }
	timingVESA_800x600_72hz = 184;  {  800x600  (72 Hz) SVGA timing. }
	timingVESA_800x600_75hz = 186;  {  800x600  (75 Hz) SVGA timing. }
	timingVESA_800x600_85hz = 188;  {  800x600  (85 Hz) SVGA timing. }
	timingVESA_1024x768_60hz = 190;  { 1024x768  (60 Hz) VESA 1K-60Hz timing. }
	timingVESA_1024x768_70hz = 200;  { 1024x768  (70 Hz) VESA 1K-70Hz timing. }
	timingVESA_1024x768_75hz = 204;  { 1024x768  (75 Hz) VESA 1K-75Hz timing (very similar to timingApple_1024x768_75hz). }
	timingVESA_1024x768_85hz = 208;  { 1024x768  (85 Hz) VESA timing. }
	timingApple_1024x768_75hz = 210;  { 1024x768  (75 Hz) Apple 19" RGB. }
	timingApple_1152x870_75hz = 220;  { 1152x870  (75 Hz) Apple 21" RGB. }
	timingAppleNTSC_ST = 230;  {  512x384  (60 Hz, interlaced, non-convolved). }
	timingAppleNTSC_FF = 232;  {  640x480  (60 Hz, interlaced, non-convolved). }
	timingAppleNTSC_STconv = 234;  {  512x384  (60 Hz, interlaced, convolved). }
	timingAppleNTSC_FFconv = 236;  {  640x480  (60 Hz, interlaced, convolved). }
	timingApplePAL_ST = 238;  {  640x480  (50 Hz, interlaced, non-convolved). }
	timingApplePAL_FF = 240;  {  768x576  (50 Hz, interlaced, non-convolved). }
	timingApplePAL_STconv = 242;  {  640x480  (50 Hz, interlaced, convolved). }
	timingApplePAL_FFconv = 244;  {  768x576  (50 Hz, interlaced, convolved). }
	timingVESA_1280x960_75hz = 250;  { 1280x960  (75 Hz) }
	timingVESA_1280x960_60hz = 252;  { 1280x960  (60 Hz) }
	timingVESA_1280x960_85hz = 254;  { 1280x960  (85 Hz) }
	timingVESA_1280x1024_60hz = 260;  { 1280x1024 (60 Hz) }
	timingVESA_1280x1024_75hz = 262;  { 1280x1024 (75 Hz) }
	timingVESA_1280x1024_85hz = 268;  { 1280x1024 (85 Hz) }
	timingVESA_1600x1200_60hz = 280;  { 1600x1200 (60 Hz) VESA timing. }
	timingVESA_1600x1200_65hz = 282;  { 1600x1200 (65 Hz) VESA timing. }
	timingVESA_1600x1200_70hz = 284;  { 1600x1200 (70 Hz) VESA timing. }
	timingVESA_1600x1200_75hz = 286;  { 1600x1200 (75 Hz) VESA timing (pixel clock is 189.2 Mhz dot clock). }
	timingVESA_1600x1200_80hz = 288;  { 1600x1200 (80 Hz) VESA timing (pixel clock is 216>? Mhz dot clock) - proposed only. }
	timingVESA_1600x1200_85hz = 289;  { 1600x1200 (85 Hz) VESA timing (pixel clock is 229.5 Mhz dot clock). }
	timingVESA_1792x1344_60hz = 296;  { 1792x1344 (60 Hz) VESA timing (204.75 Mhz dot clock). }
	timingVESA_1792x1344_75hz = 298;  { 1792x1344 (75 Hz) VESA timing (261.75 Mhz dot clock). }
	timingVESA_1856x1392_60hz = 300;  { 1856x1392 (60 Hz) VESA timing (218.25 Mhz dot clock). }
	timingVESA_1856x1392_75hz = 302;  { 1856x1392 (75 Hz) VESA timing (288 Mhz dot clock). }
	timingVESA_1920x1440_60hz = 304;  { 1920x1440 (60 Hz) VESA timing (234 Mhz dot clock). }
	timingVESA_1920x1440_75hz = 306;  { 1920x1440 (75 Hz) VESA timing (297 Mhz dot clock). }
	timingSMPTE240M_60hz = 400;  { 60Hz V, 33.75KHz H, interlaced timing, 16:9 aspect, typical resolution of 1920x1035. }
	timingFilmRate_48hz = 410;  { 48Hz V, 25.20KHz H, non-interlaced timing, typical resolution of 640x480. }
	timingSony_1600x1024_76hz = 500;  { 1600x1024 (76 Hz) Sony timing (pixel clock is 170.447 Mhz dot clock). }
	timingSony_1920x1080_60hz = 510;  { 1920x1080 (60 Hz) Sony timing (pixel clock is 159.84 Mhz dot clock). }
	timingSony_1920x1080_72hz = 520;  { 1920x1080 (72 Hz) Sony timing (pixel clock is 216.023 Mhz dot clock). }
	timingSony_1920x1200_76hz = 540;  { 1900x1200 (76 Hz) Sony timing (pixel clock is 243.20 Mhz dot clock). }
	timingApple_0x0_0hz_Offline = 550;   { Indicates that this timing will take the display off-line and remove it from the system. }


{ Deprecated timing names.}
const
	timingApple12 = timingApple_512x384_60hz;
	timingApple12x = timingApple_560x384_60hz;
	timingApple13 = timingApple_640x480_67hz;
	timingApple13x = timingApple_640x400_67hz;
	timingAppleVGA = timingVESA_640x480_60hz;
	timingApple15 = timingApple_640x870_75hz;
	timingApple15x = timingApple_640x818_75hz;
	timingApple16 = timingApple_832x624_75hz;
	timingAppleSVGA = timingVESA_800x600_56hz;
	timingApple1Ka = timingVESA_1024x768_60hz;
	timingApple1Kb = timingVESA_1024x768_70hz;
	timingApple19 = timingApple_1024x768_75hz;
	timingApple21 = timingApple_1152x870_75hz;
	timingSony_1900x1200_74hz = 530;  { 1900x1200 (74 Hz) Sony timing (pixel clock is 236.25 Mhz dot clock). }
	timingSony_1900x1200_76hz = timingSony_1920x1200_76hz; { 1900x1200 (76 Hz) Sony timing (pixel clock is 245.48 Mhz dot clock). }

{ csConnectFlags values in VDDisplayConnectInfo }
const
	kAllModesValid = 0;    { All modes not trimmed by primary init are good close enough to try }
	kAllModesSafe = 1;    { All modes not trimmed by primary init are know to be safe }
	kReportsTagging = 2;    { Can detect tagged displays (to identify smart monitors) }
	kHasDirectConnection = 3;    { True implies that driver can talk directly to device (e.g. serial data link via sense lines) }
	kIsMonoDev = 4;    { Says whether there’s an RGB (0) or Monochrome (1) connection. }
	kUncertainConnection = 5;    { There may not be a display (no sense lines?). }
	kTaggingInfoNonStandard = 6;    { Set when csConnectTaggedType/csConnectTaggedData are non-standard (i.e., not the Apple CRT sense codes). }
	kReportsDDCConnection = 7;    { Card can do ddc (set kHasDirectConnect && kHasDDCConnect if you actually found a ddc display). }
	kHasDDCConnection = 8;    { Card has ddc connect now. }
	kConnectionInactive = 9;    { Set when the connection is NOT currently active (generally used in a multiconnection environment). }
	kDependentConnection = 10;   { Set when some ascpect of THIS connection depends on another (will generally be set in a kModeSimulscan environment). }
	kBuiltInConnection = 11;   { Set when connection is KNOWN to be built-in (this is not the same as kHasDirectConnection). }
	kOverrideConnection = 12;   { Set when the reported connection is not the true one, but is one that has been forced through a SetConnection call }
	kFastCheckForDDC = 13;   { Set when all 3 are true: 1) sense codes indicate DDC display could be attached 2) attempted fast check 3) DDC failed }
	kReportsHotPlugging = 14;    { Detects and reports hot pluggging on connector (via VSL also implies DDC will be up to date w/o force read) }


{ csDisplayType values in VDDisplayConnectInfo }
const
	kUnknownConnect = 1;    { Not sure how we’ll use this, but seems like a good idea. }
	kPanelConnect = 2;    { For use with fixed-in-place LCD panels. }
	kPanelTFTConnect = 2;    { Alias for kPanelConnect }
	kFixedModeCRTConnect = 3;    {  For use with fixed-mode (i.e., very limited range) displays. }
	kMultiModeCRT1Connect = 4;    { 320x200 maybe, 12" maybe, 13" (default), 16" certain, 19" maybe, 21" maybe }
	kMultiModeCRT2Connect = 5;    { 320x200 maybe, 12" maybe, 13" certain, 16" (default), 19" certain, 21" maybe }
	kMultiModeCRT3Connect = 6;    { 320x200 maybe, 12" maybe, 13" certain, 16" certain, 19" default, 21" certain }
	kMultiModeCRT4Connect = 7;    { Expansion to large multi mode (not yet used) }
	kModelessConnect = 8;    { Expansion to modeless model (not yet used) }
	kFullPageConnect = 9;    { 640x818 (to get 8bpp in 512K case) and 640x870 (these two only) }
	kVGAConnect = 10;   { 640x480 VGA default -- question everything else }
	kNTSCConnect = 11;   { NTSC ST (default), FF, STconv, FFconv }
	kPALConnect = 12;   { PAL ST (default), FF, STconv, FFconv }
	kHRConnect = 13;   { Straight-6 connect -- 640x480 and 640x400 (to get 8bpp in 256K case) (these two only) }
	kPanelFSTNConnect = 14;   { For use with fixed-in-place LCD FSTN (aka “Supertwist”) panels }
	kMonoTwoPageConnect = 15;   { 1152x870 Apple color two-page display }
	kColorTwoPageConnect = 16;   { 1152x870 Apple B&W two-page display }
	kColor16Connect = 17;   { 832x624 Apple B&W two-page display }
	kColor19Connect = 18;   { 1024x768 Apple B&W two-page display }
	kGenericCRT = 19;   { Indicates nothing except that connection is CRT in nature. }
	kGenericLCD = 20;   { Indicates nothing except that connection is LCD in nature. }
	kDDCConnect = 21;   { DDC connection, always set kHasDDCConnection }
	kNoConnect = 22;    { No display is connected - load sensing or similar level of hardware detection is assumed (used by resident drivers that support hot plugging when nothing is currently connected) }

{ csTimingFlags values in VDTimingInfoRec }
const
	kModeValid = 0;    { Says that this mode should NOT be trimmed. }
	kModeSafe = 1;    { This mode does not need confirmation }
	kModeDefault = 2;    { This is the default mode for this type of connection }
	kModeShowNow = 3;    { This mode should always be shown (even though it may require a confirm) }
	kModeNotResize = 4;    { This mode should not be used to resize the display (eg. mode selects a different connector on card) }
	kModeRequiresPan = 5;    { This mode has more pixels than are actually displayed }
	kModeInterlaced = 6;    { This mode is interlaced (single pixel lines look bad). }
	kModeShowNever = 7;    { This mode should not be shown in the user interface. }
	kModeSimulscan = 8;    { Indicates that more than one display connection can be driven from a single framebuffer controller. }
	kModeNotPreset = 9;    { Indicates that the timing is not a factory preset for the current display (geometry may need correction) }
	kModeBuiltIn = 10;   { Indicates that the display mode is for the built-in connect only (on multiconnect devices like the PB 3400) Only the driver is quieried }
	kModeStretched = 11;    { Indicates that the display mode will be stretched/distorted to match the display aspect ratio }

{ csDepthFlags in VDVideoParametersInfoRec }
const
	kDepthDependent = 0;     { Says that this depth mode may cause dependent changes in other framebuffers (and . }

{ csResolutionFlags bit flags for VDResolutionInfoRec }
const
	kResolutionHasMultipleDepthSizes = 0;  { Says that this mode has different csHorizontalPixels, csVerticalLines at different depths (usually slightly larger at lower depths) }


const
{    Power Mode constants for VDPowerStateRec.powerState.  Note the numeric order does not match the power state order }
	kAVPowerOff = 0;    { Power fully off}
	kAVPowerStandby = 1;
	kAVPowerSuspend = 2;
	kAVPowerOn = 3;
	kHardwareSleep = 128;
	kHardwareWake = 129;
	kHardwareWakeFromSuspend = 130;
	kHardwareWakeToDoze = 131;
	kHardwareWakeToDozeFromSuspend = 132;

const
{    Power Mode masks and bits for VDPowerStateRec.powerFlags.  }
	kPowerStateNeedsRefresh = 0;    { When leaving this power mode, a display will need refreshing   }
	kPowerStateSleepAwareBit = 1;    { if gestaltPCCardDockingSelectorFix, Docking mgr checks this bit before checking kPowerStateSleepAllowedBit }
	kPowerStateSleepForbiddenBit = 2;    { if kPowerStateSleepAwareBit, Docking mgr checks this bit before sleeping }
	kPowerStateSleepCanPowerOffBit = 3;   { supports power down sleep (ie PCI power off)}
	kPowerStateSleepNoDPMSBit = 4;    { Bug #2425210.  Do not use DPMS with this display.}
	kPowerStateSleepWaketoDozeBit = 5;    { Supports Wake to Doze }
	kPowerStateNeedsRefreshMask = 1 shl kPowerStateNeedsRefresh;
	kPowerStateSleepAwareMask = 1 shl kPowerStateSleepAwareBit;
	kPowerStateSleepForbiddenMask = 1 shl kPowerStateSleepForbiddenBit;
	kPowerStateSleepCanPowerOffMask = 1 shl kPowerStateSleepCanPowerOffBit;
	kPowerStateSleepNoDPMSMask = 1 shl kPowerStateSleepNoDPMSBit;
	kPowerStateSleepWaketoDozeMask = 1 shl kPowerStateSleepWaketoDozeBit;


const
{ Control Codes }
	cscReset = 0;
	cscKillIO = 1;
	cscSetMode = 2;
	cscSetEntries = 3;
	cscSetGamma = 4;
	cscGrayPage = 5;
	cscGrayScreen = 5;
	cscSetGray = 6;
	cscSetInterrupt = 7;
	cscDirectSetEntries = 8;
	cscSetDefaultMode = 9;
	cscSwitchMode = 10;   { Takes a VDSwitchInfoPtr }
	cscSetSync = 11;   { Takes a VDSyncInfoPtr }
	cscSavePreferredConfiguration = 16;   { Takes a VDSwitchInfoPtr }
	cscSetHardwareCursor = 22;   { Takes a VDSetHardwareCursorPtr }
	cscDrawHardwareCursor = 23;   { Takes a VDDrawHardwareCursorPtr }
	cscSetConvolution = 24;   { Takes a VDConvolutionInfoPtr }
	cscSetPowerState = 25;   { Takes a VDPowerStatePtr }
	cscPrivateControlCall = 26;   { Takes a VDPrivateSelectorDataPtr }
	cscSetMultiConnect = 28;   { Takes a VDMultiConnectInfoPtr }
	cscSetClutBehavior = 29;   { Takes a VDClutBehavior }
	cscSetDetailedTiming = 31;   { Takes a VDDetailedTimingPtr }
	cscDoCommunication = 33;   { Takes a VDCommunicationPtr }
	cscProbeConnection = 34;   { Takes nil pointer (may generate a kFBConnectInterruptServiceType service interrupt)}
	cscUnusedCall = 127;   { This call used to expend the scrn resource.  Its imbedded data contains more control info }

const
{ Status Codes }
	cscGetMode = 2;
	cscGetEntries = 3;
	cscGetPageCnt = 4;
	cscGetPages = 4;    { This is what C&D 2 calls it. }
	cscGetPageBase = 5;
	cscGetBaseAddr = 5;    { This is what C&D 2 calls it. }
	cscGetGray = 6;
	cscGetInterrupt = 7;
	cscGetGamma = 8;
	cscGetDefaultMode = 9;
	cscGetCurMode = 10;   { Takes a VDSwitchInfoPtr }
	cscGetSync = 11;   { Takes a VDSyncInfoPtr }
	cscGetConnection = 12;   { Return information about the connection to the display }
	cscGetModeTiming = 13;   { Return timing info for a mode }
	cscGetModeBaseAddress = 14;   { Return base address information about a particular mode }
	cscGetScanProc = 15;   { QuickTime scan chasing routine }
	cscGetPreferredConfiguration = 16;   { Takes a VDSwitchInfoPtr }
	cscGetNextResolution = 17;   { Takes a VDResolutionInfoPtr }
	cscGetVideoParameters = 18;   { Takes a VDVideoParametersInfoPtr }
	cscGetGammaInfoList = 20;   { Takes a VDGetGammaListPtr }
	cscRetrieveGammaTable = 21;   { Takes a VDRetrieveGammaPtr }
	cscSupportsHardwareCursor = 22;   { Takes a VDSupportsHardwareCursorPtr }
	cscGetHardwareCursorDrawState = 23;   { Takes a VDHardwareCursorDrawStatePtr }
	cscGetConvolution = 24;   { Takes a VDConvolutionInfoPtr }
	cscGetPowerState = 25;   { Takes a VDPowerStatePtr }
	cscPrivateStatusCall = 26;   { Takes a VDPrivateSelectorDataPtr }
	cscGetDDCBlock = 27;   { Takes a VDDDCBlockPtr  }
	cscGetMultiConnect = 28;   { Takes a VDMultiConnectInfoPtr }
	cscGetClutBehavior = 29;   { Takes a VDClutBehaviorPtr }
	cscGetTimingRanges = 30;   { Takes a VDDisplayTimingRangePtr }
	cscGetDetailedTiming = 31;   { Takes a VDDetailedTimingPtr }
	cscGetCommunicationInfo = 32;    { Takes a VDCommunicationInfoPtr }

{ Bit definitions for the Get/Set Sync call}
const
	kDisableHorizontalSyncBit = 0;
	kDisableVerticalSyncBit = 1;
	kDisableCompositeSyncBit = 2;
	kEnableSyncOnBlue = 3;
	kEnableSyncOnGreen = 4;
	kEnableSyncOnRed = 5;
	kNoSeparateSyncControlBit = 6;
	kTriStateSyncBit = 7;
	kHorizontalSyncMask = $01;
	kVerticalSyncMask = $02;
	kCompositeSyncMask = $04;
	kDPMSSyncMask = $07;
	kTriStateSyncMask = $80;
	kSyncOnBlueMask = $08;
	kSyncOnGreenMask = $10;
	kSyncOnRedMask = $20;
	kSyncOnMask = $38;

const
{    Power Mode constants for translating DPMS modes to Get/SetSync calls.  }
	kDPMSSyncOn = 0;
	kDPMSSyncStandby = 1;
	kDPMSSyncSuspend = 2;
	kDPMSSyncOff = 7;

{ Bit definitions for the Get/Set Convolution call}
const
	kConvolved = 0;
	kLiveVideoPassThru = 1;
	kConvolvedMask = $01;
	kLiveVideoPassThruMask = $02;


type
	VPBlock = record
		vpBaseOffset: SIGNEDLONG;           {Offset to page zero of video RAM (From minorBaseOS).}
		vpRowBytes: SInt16;             {Width of each row of video memory.}
		vpBounds: Rect;               {BoundsRect for the video display (gives dimensions).}
		vpVersion: SInt16;              {PixelMap version number.}
		vpPackType: SInt16;
		vpPackSize: SIGNEDLONG;
		vpHRes: SIGNEDLONG;                 {Horizontal resolution of the device (pixels per inch).}
		vpVRes: SIGNEDLONG;                 {Vertical resolution of the device (pixels per inch).}
		vpPixelType: SInt16;            {Defines the pixel type.}
		vpPixelSize: SInt16;            {Number of bits in pixel.}
		vpCmpCount: SInt16;             {Number of components in pixel.}
		vpCmpSize: SInt16;              {Number of bits per component}
		vpPlaneBytes: SIGNEDLONG;           {Offset from one plane to the next.}
	end;
	VPBlockPtr = ^VPBlock;
type
	VDEntryRecordPtr = ^VDEntryRecord;
	VDEntryRecord = record
		csTable: Ptr;                {(long) pointer to color table entry=value, r,g,b:INTEGER}
	end;
type
	VDEntRecPtr = VDEntryRecordPtr;
{ Parm block for SetGray control call }
type
	VDGrayRecordPtr = ^VDGrayRecord;
	VDGrayRecord = record
		csMode: Boolean;                 {Same as GDDevType value (0=color, 1=mono)}
		filler: SInt8;
	end;
type
	VDGrayPtr = VDGrayRecordPtr;
{ Parm block for SetInterrupt call }
type
	VDFlagRecordPtr = ^VDFlagRecord;
	VDFlagRecord = record
		csMode: SInt8;
		filler: SInt8;
	end;
type
	VDFlagRecPtr = VDFlagRecordPtr;
{ Parm block for SetEntries control call }
type
	VDSetEntryRecordPtr = ^VDSetEntryRecord;
	VDSetEntryRecord = record
		csTable: ColorSpecPtr;                {Pointer to an array of color specs}
		csStart: SInt16;                {Which spec in array to start with, or -1}
		csCount: SInt16;                {Number of color spec entries to set}
	end;
type
	VDSetEntryPtr = VDSetEntryRecordPtr;
{ Parm block for SetGamma control call }
type
	VDGammaRecordPtr = ^VDGammaRecord;
	VDGammaRecord = record
		csGTable: Ptr;               {pointer to gamma table}
	end;
type
	VDGamRecPtr = VDGammaRecordPtr;
	VDBaseAddressInfoRecPtr = ^VDBaseAddressInfoRec;
	VDBaseAddressInfoRec = record
		csDevData: SIGNEDLONG;              { LONGINT - (long) timing mode }
		csDevBase: SIGNEDLONG;              { LONGINT - (long) base address of the mode }
		csModeReserved: SInt16;         { INTEGER - (short) will some day be the depth }
		csModeBase: SIGNEDLONG;             { LONGINT - (long) reserved }
	end;
type
	VDBaseAddressInfoPtr = VDBaseAddressInfoRecPtr;
	VDSwitchInfoRecPtr = ^VDSwitchInfoRec;
	VDSwitchInfoRec = record
		csMode: UInt16;                 {(word) mode depth}
		csData: UNSIGNEDLONG;                 {(long) functional sResource of mode}
		csPage: UInt16;                 {(word) page to switch in}
		csBaseAddr: Ptr;             {(long) base address of page (return value)}
		csReserved: UNSIGNEDLONG;             {(long) Reserved (set to 0) }
	end;
type
	VDSwitchInfoPtr = VDSwitchInfoRecPtr;
	VDTimingInfoRecPtr = ^VDTimingInfoRec;
	VDTimingInfoRec = record
		csTimingMode: UNSIGNEDLONG;           { LONGINT - (long) timing mode (a la InitGDevice) }
		csTimingReserved: UNSIGNEDLONG;       { LONGINT - (long) reserved }
		csTimingFormat: UNSIGNEDLONG;         { LONGINT - (long) what format is the timing info }
		csTimingData: UNSIGNEDLONG;           { LONGINT - (long) data supplied by driver }
		csTimingFlags: UNSIGNEDLONG;          { LONGINT - (long) mode within device }
	end;
type
	VDTimingInfoPtr = VDTimingInfoRecPtr;
	VDDisplayConnectInfoRecPtr = ^VDDisplayConnectInfoRec;
	VDDisplayConnectInfoRec = record
		csDisplayType: UInt16;          { INTEGER - (word) Type of display connected }
		csConnectTaggedType: UInt8;    { BYTE - type of tagging }
		csConnectTaggedData: UInt8;    { BYTE - tagging data }
		csConnectFlags: UNSIGNEDLONG;         { LONGINT - (long) tell us about the connection }
		csDisplayComponent: UNSIGNEDLONG;     { LONGINT - (long) if the card has a direct connection to the display, it returns the display component here (FUTURE) }
		csConnectReserved: UNSIGNEDLONG;      { LONGINT - (long) reserved }
	end;
type
	VDDisplayConnectInfoPtr = VDDisplayConnectInfoRecPtr;
	VDMultiConnectInfoRecPtr = ^VDMultiConnectInfoRec;
	VDMultiConnectInfoRec = record
		csDisplayCountOrNumber: UNSIGNEDLONG; { For GetMultiConnect, returns count n of 1..n connections; otherwise, indicates the ith connection.}
		csConnectInfo: VDDisplayConnectInfoRec;     { Standard VDDisplayConnectionInfo for connection i.}
	end;
type
	VDMultiConnectInfoPtr = VDMultiConnectInfoRecPtr;
{ RawSenseCode
    This abstract data type is not exactly abstract.  Rather, it is merely enumerated constants
    for the possible raw sense code values when 'standard' sense code hardware is implemented.

    For 'standard' sense code hardware, the raw sense is obtained as follows:
        • Instruct the frame buffer controller NOT to actively drive any of the monitor sense lines
        • Read the state of the monitor sense lines 2, 1, and 0.  (2 is the MSB, 0 the LSB)

    IMPORTANT Note: 
    When the 'kTaggingInfoNonStandard' bit of 'csConnectFlags' is FALSE, then these constants 
    are valid 'csConnectTaggedType' values in 'VDDisplayConnectInfo' 

}
type
	RawSenseCode = UInt8;
const
	kRSCZero = 0;
	kRSCOne = 1;
	kRSCTwo = 2;
	kRSCThree = 3;
	kRSCFour = 4;
	kRSCFive = 5;
	kRSCSix = 6;
	kRSCSeven = 7;


{ ExtendedSenseCode
    This abstract data type is not exactly abstract.  Rather, it is merely enumerated constants
    for the values which are possible when the extended sense algorithm is applied to hardware
    which implements 'standard' sense code hardware.

    For 'standard' sense code hardware, the extended sense code algorithm is as follows:
    (Note:  as described here, sense line 'A' corresponds to '2', 'B' to '1', and 'C' to '0')
        • Drive sense line 'A' low and read the values of 'B' and 'C'.  
        • Drive sense line 'B' low and read the values of 'A' and 'C'.
        • Drive sense line 'C' low and read the values of 'A' and 'B'.

    In this way, a six-bit number of the form BC/AC/AB is generated. 

    IMPORTANT Note: 
    When the 'kTaggingInfoNonStandard' bit of 'csConnectFlags' is FALSE, then these constants 
    are valid 'csConnectTaggedData' values in 'VDDisplayConnectInfo' 

}
type
	ExtendedSenseCode = UInt8;
const
	kESCZero21Inch = $00; { 21" RGB                     }
	kESCOnePortraitMono = $14; { Portrait Monochrome              }
	kESCTwo12Inch = $21; { 12" RGB                    }
	kESCThree21InchRadius = $31; { 21" RGB (Radius)               }
	kESCThree21InchMonoRadius = $34; { 21" Monochrome (Radius)           }
	kESCThree21InchMono = $35; { 21" Monochrome               }
	kESCFourNTSC = $0A; { NTSC                     }
	kESCFivePortrait = $1E; { Portrait RGB              }
	kESCSixMSB1 = $03; { MultiScan Band-1 (12" thru 1Six")  }
	kESCSixMSB2 = $0B; { MultiScan Band-2 (13" thru 19")       }
	kESCSixMSB3 = $23; { MultiScan Band-3 (13" thru 21")       }
	kESCSixStandard = $2B; { 13"/14" RGB or 12" Monochrome   }
	kESCSevenPAL = $00; { PAL                        }
	kESCSevenNTSC = $14; { NTSC                     }
	kESCSevenVGA = $17; { VGA                        }
	kESCSeven16Inch = $2D; { 16" RGB (GoldFish)               }
	kESCSevenPALAlternate = $30; { PAL (Alternate)                }
	kESCSeven19Inch = $3A; { Third-Party 19”                 }
	kESCSevenDDC = $3E; { DDC display                   }
	kESCSevenNoDisplay = $3F;  { No display connected           }

{ DepthMode
    This abstract data type is used to to reference RELATIVE pixel depths.
    Its definition is largely derived from its past usage, analogous to 'xxxVidMode'

    Bits per pixel DOES NOT directly map to 'DepthMode'  For example, on some
    graphics hardware, 'kDepthMode1' may represent 1 BPP, whereas on other
    hardware, 'kDepthMode1' may represent 8BPP.

    DepthMode IS considered to be ordinal, i.e., operations such as <, >, ==, etc.
    behave as expected.  The values of the constants which comprise the set are such
    that 'kDepthMode4 < kDepthMode6' behaves as expected.
}
type
	DepthMode = UInt16;
const
	kDepthMode1 = 128;
	kDepthMode2 = 129;
	kDepthMode3 = 130;
	kDepthMode4 = 131;
	kDepthMode5 = 132;
	kDepthMode6 = 133;

<<<<<<< HEAD
	kFirstDepthMode				= 128;							{  These constants are obsolete, and just included     }
	kSecondDepthMode			= 129;							{  for clients that have converted to the above      }
	kThirdDepthMode				= 130;							{  kDepthModeXXX constants.                 }
	kFourthDepthMode			= 131;
	kFifthDepthMode				= 132;
	kSixthDepthMode				= 133;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
const
	kFirstDepthMode = 128;  { These constants are obsolete, and just included    }
	kSecondDepthMode = 129;  { for clients that have converted to the above     }
	kThirdDepthMode = 130;  { kDepthModeXXX constants.                }
	kFourthDepthMode = 131;
	kFifthDepthMode = 132;
	kSixthDepthMode = 133;
>>>>>>> origin/fixes_2.4


type
	VDPageInfoPtr = ^VDPageInfo;
	VDPageInfo = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		csMode: SInt16;                 {(word) mode within device}
		csData: SIGNEDLONG;                 {(long) data supplied by driver}
		csPage: SInt16;                 {(word) page to switch in}
		csBaseAddr: Ptr;             {(long) base address of page}
<<<<<<< HEAD
	end;
type
	VDPgInfoPtr = VDPageInfoPtr;
	VDSizeInfoPtr = ^VDSizeInfo;
	VDSizeInfo = record
		csHSize: SInt16;                {(word) desired/returned h size}
		csHPos: SInt16;                 {(word) desired/returned h position}
		csVSize: SInt16;                {(word) desired/returned v size}
		csVPos: SInt16;                 {(word) desired/returned v position}
	end;
type
	VDSzInfoPtr = VDSizeInfoPtr;
	VDSettings = record
		csParamCnt: SInt16;             {(word) number of params}
		csBrightMax: SInt16;            {(word) max brightness}
		csBrightDef: SInt16;            {(word) default brightness}
		csBrightVal: SInt16;            {(word) current brightness}
		csCntrstMax: SInt16;            {(word) max contrast}
		csCntrstDef: SInt16;            {(word) default contrast}
		csCntrstVal: SInt16;            {(word) current contrast}
		csTintMax: SInt16;              {(word) max tint}
		csTintDef: SInt16;              {(word) default tint}
		csTintVal: SInt16;              {(word) current tint}
		csHueMax: SInt16;               {(word) max hue}
		csHueDef: SInt16;               {(word) default hue}
		csHueVal: SInt16;               {(word) current hue}
		csHorizDef: SInt16;             {(word) default horizontal}
		csHorizVal: SInt16;             {(word) current horizontal}
		csHorizMax: SInt16;             {(word) max horizontal}
		csVertDef: SInt16;              {(word) default vertical}
		csVertVal: SInt16;              {(word) current vertical}
		csVertMax: SInt16;              {(word) max vertical}
	end;
	VDSettingsPtr = ^VDSettings;
type
	VDDefMode = record
		csID: UInt8;
		filler: SInt8;
	end;
	VDDefModePtr = ^VDDefMode;
type
	VDSyncInfoRecPtr = ^VDSyncInfoRec;
	VDSyncInfoRec = record
		csMode: UInt8;
		csFlags: UInt8;
	end;
type
	VDSyncInfoPtr = VDSyncInfoRecPtr;
	AVIDType = UInt32;
	DisplayIDType = AVIDType;
	DisplayModeID = UInt32;
	VideoDeviceType = UInt32;
	GammaTableID = UInt32;
{
   All displayModeID values from 0x80000000 to 0xFFFFFFFF and 0x00
   are reserved for Apple Computer.
}
{ Constants for the cscGetNextResolution call }
const
	kDisplayModeIDCurrent = $00; { Reference the Current DisplayModeID }
	kDisplayModeIDInvalid = -1; { A bogus DisplayModeID in all cases }
	kDisplayModeIDFindFirstResolution = -2; { Used in cscGetNextResolution to reset iterator }
	kDisplayModeIDNoMoreResolutions = -3; { Used in cscGetNextResolution to indicate End Of List }
	kDisplayModeIDFindFirstProgrammable = -4; { Used in cscGetNextResolution to find unused programmable timing }
	kDisplayModeIDBootProgrammable = -5; { This is the ID given at boot time by the OF driver to a programmable timing }
	kDisplayModeIDReservedBase = $80000000; { Lowest (unsigned) DisplayModeID reserved by Apple }

{ Constants for the GetGammaInfoList call }
const
	kGammaTableIDFindFirst = -2; { Get the first gamma table ID }
	kGammaTableIDNoMoreTables = -3; { Used to indicate end of list }
	kGammaTableIDSpecific = $00;  { Return the info for the given table id }

{ Constants for GetMultiConnect call}
const
	kGetConnectionCount = -1; { Used to get the number of possible connections in a “multi-headed” framebuffer environment.}
	kActivateConnection = 0 shl kConnectionInactive; { Used for activating a connection (csConnectFlags value).}
	kDeactivateConnection = 1 shl kConnectionInactive; { Used for deactivating a connection (csConnectFlags value.)}

{ VDCommunicationRec.csBusID values}
const
	kVideoDefaultBus = 0;


{ VDCommunicationInfoRec.csBusType values}
const
	kVideoBusTypeInvalid = 0;
	kVideoBusTypeI2C = 1;


{ VDCommunicationRec.csSendType and VDCommunicationRec.csReplyType values and bits in VDCommunicationInfoRec.csSupportedTypes.}
const
	kVideoNoTransactionType = 0;    { No transaction}
	kVideoSimpleI2CType = 1;    { Simple I2C message}
	kVideoDDCciReplyType = 2;     { DDC/ci message (with imbedded length)}

{ VDCommunicationRec.csCommFlags and VDCommunicationInfoRec.csSupportedCommFlags}
const
	kVideoReplyMicroSecDelayMask = 1 shl 0; { If set, the driver should delay csMinReplyDelay micro seconds between send and recieve}
=======
=======
>>>>>>> origin/fixes_2_2
		csMode:					SInt16;								{ (word) mode within device }
		csData:					SInt32;								{ (long) data supplied by driver }
		csPage:					SInt16;								{ (word) page to switch in }
		csBaseAddr:				Ptr;									{ (long) base address of page }
=======
>>>>>>> origin/fixes_2.4
	end;
type
	VDPgInfoPtr = VDPageInfoPtr;
	VDSizeInfoPtr = ^VDSizeInfo;
	VDSizeInfo = record
		csHSize: SInt16;                {(word) desired/returned h size}
		csHPos: SInt16;                 {(word) desired/returned h position}
		csVSize: SInt16;                {(word) desired/returned v size}
		csVPos: SInt16;                 {(word) desired/returned v position}
	end;
type
	VDSzInfoPtr = VDSizeInfoPtr;
	VDSettings = record
		csParamCnt: SInt16;             {(word) number of params}
		csBrightMax: SInt16;            {(word) max brightness}
		csBrightDef: SInt16;            {(word) default brightness}
		csBrightVal: SInt16;            {(word) current brightness}
		csCntrstMax: SInt16;            {(word) max contrast}
		csCntrstDef: SInt16;            {(word) default contrast}
		csCntrstVal: SInt16;            {(word) current contrast}
		csTintMax: SInt16;              {(word) max tint}
		csTintDef: SInt16;              {(word) default tint}
		csTintVal: SInt16;              {(word) current tint}
		csHueMax: SInt16;               {(word) max hue}
		csHueDef: SInt16;               {(word) default hue}
		csHueVal: SInt16;               {(word) current hue}
		csHorizDef: SInt16;             {(word) default horizontal}
		csHorizVal: SInt16;             {(word) current horizontal}
		csHorizMax: SInt16;             {(word) max horizontal}
		csVertDef: SInt16;              {(word) default vertical}
		csVertVal: SInt16;              {(word) current vertical}
		csVertMax: SInt16;              {(word) max vertical}
	end;
	VDSettingsPtr = ^VDSettings;
type
	VDDefMode = record
		csID: UInt8;
		filler: SInt8;
	end;
	VDDefModePtr = ^VDDefMode;
type
	VDSyncInfoRecPtr = ^VDSyncInfoRec;
	VDSyncInfoRec = record
		csMode: UInt8;
		csFlags: UInt8;
	end;
type
	VDSyncInfoPtr = VDSyncInfoRecPtr;
	AVIDType = UInt32;
	DisplayIDType = AVIDType;
	DisplayModeID = UInt32;
	VideoDeviceType = UInt32;
	GammaTableID = UInt32;
{
   All displayModeID values from 0x80000000 to 0xFFFFFFFF and 0x00
   are reserved for Apple Computer.
}
{ Constants for the cscGetNextResolution call }
const
	kDisplayModeIDCurrent = $00; { Reference the Current DisplayModeID }
	kDisplayModeIDInvalid = -1; { A bogus DisplayModeID in all cases }
	kDisplayModeIDFindFirstResolution = -2; { Used in cscGetNextResolution to reset iterator }
	kDisplayModeIDNoMoreResolutions = -3; { Used in cscGetNextResolution to indicate End Of List }
	kDisplayModeIDFindFirstProgrammable = -4; { Used in cscGetNextResolution to find unused programmable timing }
	kDisplayModeIDBootProgrammable = -5; { This is the ID given at boot time by the OF driver to a programmable timing }
	kDisplayModeIDReservedBase = $80000000; { Lowest (unsigned) DisplayModeID reserved by Apple }

{ Constants for the GetGammaInfoList call }
const
	kGammaTableIDFindFirst = -2; { Get the first gamma table ID }
	kGammaTableIDNoMoreTables = -3; { Used to indicate end of list }
	kGammaTableIDSpecific = $00;  { Return the info for the given table id }

{ Constants for GetMultiConnect call}
const
	kGetConnectionCount = -1; { Used to get the number of possible connections in a “multi-headed” framebuffer environment.}
	kActivateConnection = 0 shl kConnectionInactive; { Used for activating a connection (csConnectFlags value).}
	kDeactivateConnection = 1 shl kConnectionInactive; { Used for deactivating a connection (csConnectFlags value.)}

{ VDCommunicationRec.csBusID values}
const
	kVideoDefaultBus = 0;


{ VDCommunicationInfoRec.csBusType values}
const
	kVideoBusTypeInvalid = 0;
	kVideoBusTypeI2C = 1;


{ VDCommunicationRec.csSendType and VDCommunicationRec.csReplyType values and bits in VDCommunicationInfoRec.csSupportedTypes.}
const
	kVideoNoTransactionType = 0;    { No transaction}
	kVideoSimpleI2CType = 1;    { Simple I2C message}
	kVideoDDCciReplyType = 2;     { DDC/ci message (with imbedded length)}

<<<<<<< HEAD
	{  VDCommunicationRec.csCommFlags and VDCommunicationInfoRec.csSupportedCommFlags }
	kVideoReplyMicroSecDelayMask = $01;							{  If set, the driver should delay csMinReplyDelay micro seconds between send and recieve }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ VDCommunicationRec.csCommFlags and VDCommunicationInfoRec.csSupportedCommFlags}
const
	kVideoReplyMicroSecDelayMask = 1 shl 0; { If set, the driver should delay csMinReplyDelay micro seconds between send and recieve}
>>>>>>> origin/fixes_2.4


type
	VDResolutionInfoRecPtr = ^VDResolutionInfoRec;
	VDResolutionInfoRec = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		csPreviousDisplayModeID: DisplayModeID; { ID of the previous resolution in a chain }
		csDisplayModeID: DisplayModeID;        { ID of the next resolution }
		csHorizontalPixels: UNSIGNEDLONG;     { # of pixels in a horizontal line at the max depth }
		csVerticalLines: UNSIGNEDLONG;        { # of lines in a screen at the max depth }
		csRefreshRate: Fixed;          { Vertical Refresh Rate in Hz }
		csMaxDepthMode: DepthMode;         { 0x80-based number representing max bit depth }
		csResolutionFlags: UNSIGNEDLONG;      { Reserved - flag bits }
		csReserved: UNSIGNEDLONG;             { Reserved }
<<<<<<< HEAD
	end;
type
	VDResolutionInfoPtr = VDResolutionInfoRecPtr;
	VDVideoParametersInfoRecPtr = ^VDVideoParametersInfoRec;
	VDVideoParametersInfoRec = record
		csDisplayModeID: DisplayModeID;        { the ID of the resolution we want info on }
		csDepthMode: DepthMode;            { The bit depth we want the info on (0x80 based) }
		csVPBlockPtr: VPBlockPtr;           { Pointer to a video parameter block }
		csPageCount: UNSIGNEDLONG;            { Number of pages supported by the resolution }
		csDeviceType: VideoDeviceType;           { Device Type:  Direct, Fixed or CLUT; }
		csDepthFlags: UInt32;           { Flags }
	end;
type
	VDVideoParametersInfoPtr = VDVideoParametersInfoRecPtr;
	VDGammaInfoRecPtr = ^VDGammaInfoRec;
	VDGammaInfoRec = record
		csLastGammaID: GammaTableID;          { the ID of the previous gamma table }
		csNextGammaID: GammaTableID;          { the ID of the next gamma table }
		csGammaPtr: Ptr;             { Ptr to a gamma table data }
		csReserved: UNSIGNEDLONG;             { Reserved }
	end;
type
	VDGammaInfoPtr = VDGammaInfoRecPtr;
	VDGetGammaListRecPtr = ^VDGetGammaListRec;
	VDGetGammaListRec = record
		csPreviousGammaTableID: GammaTableID; { ID of the previous gamma table }
		csGammaTableID: GammaTableID;         { ID of the gamma table following csPreviousDisplayModeID }
		csGammaTableSize: UNSIGNEDLONG;       { Size of the gamma table in bytes }
		csGammaTableName: CStringPtr;       { Gamma table name (c-string) }
	end;
type
	VDGetGammaListPtr = VDGetGammaListRecPtr;
	VDRetrieveGammaRecPtr = ^VDRetrieveGammaRec;
	VDRetrieveGammaRec = record
		csGammaTableID: GammaTableID;         { ID of gamma table to retrieve }
		csGammaTablePtr: GammaTblPtr;        { Location to copy desired gamma to }
	end;
type
	VDRetrieveGammaPtr = VDRetrieveGammaRecPtr;
	VDSetHardwareCursorRecPtr = ^VDSetHardwareCursorRec;
	VDSetHardwareCursorRec = record
		csCursorRef: UnivPtr;            { reference to cursor data }
		csReserved1: UInt32;            { reserved for future use }
		csReserved2: UInt32;            { should be ignored }
	end;
type
	VDSetHardwareCursorPtr = VDSetHardwareCursorRecPtr;
	VDDrawHardwareCursorRecPtr = ^VDDrawHardwareCursorRec;
	VDDrawHardwareCursorRec = record
		csCursorX: SInt32;              { x coordinate }
		csCursorY: SInt32;              { y coordinate }
		csCursorVisible: UInt32;        { true if cursor is must be visible }
		csReserved1: UInt32;            { reserved for future use }
		csReserved2: UInt32;            { should be ignored }
	end;
type
	VDDrawHardwareCursorPtr = VDDrawHardwareCursorRecPtr;
	VDSupportsHardwareCursorRecPtr = ^VDSupportsHardwareCursorRec;
	VDSupportsHardwareCursorRec = record
		csSupportsHardwareCursor: UInt32;
                                              { true if hardware cursor is supported }
		csReserved1: UInt32;            { reserved for future use }
		csReserved2: UInt32;            { must be zero }
	end;
type
	VDSupportsHardwareCursorPtr = VDSupportsHardwareCursorRecPtr;
	VDHardwareCursorDrawStateRecPtr = ^VDHardwareCursorDrawStateRec;
	VDHardwareCursorDrawStateRec = record
		csCursorX: SInt32;              { x coordinate }
		csCursorY: SInt32;              { y coordinate }
		csCursorVisible: UInt32;        { true if cursor is visible }
		csCursorSet: UInt32;            { true if cursor successfully set by last set control call }
		csReserved1: UInt32;            { reserved for future use }
		csReserved2: UInt32;            { must be zero }
	end;
type
	VDHardwareCursorDrawStatePtr = VDHardwareCursorDrawStateRecPtr;
	VDConvolutionInfoRecPtr = ^VDConvolutionInfoRec;
	VDConvolutionInfoRec = record
		csDisplayModeID: DisplayModeID;        { the ID of the resolution we want info on }
		csDepthMode: DepthMode;            { The bit depth we want the info on (0x80 based) }
		csPage: UNSIGNEDLONG;
		csFlags: UInt32;
		csReserved: UInt32;
	end;
type
	VDConvolutionInfoPtr = VDConvolutionInfoRecPtr;
	VDPowerStateRecPtr = ^VDPowerStateRec;
	VDPowerStateRec = record
		powerState: UNSIGNEDLONG;
		powerFlags: UNSIGNEDLONG;

		powerReserved1: UNSIGNEDLONG;
		powerReserved2: UNSIGNEDLONG;
	end;
type
	VDPowerStatePtr = VDPowerStateRecPtr;
{
    Private Data to video drivers.
    
    In versions of MacOS with multiple address spaces (System 8), the OS 
    must know the extent of parameters in order to move them between the caller
    and driver.  The old private-selector model for video drivers does not have
    this information so:
    
    For post-7.x Systems private calls should be implemented using the cscPrivateCall
}
type
	VDPrivateSelectorDataRecPtr = ^VDPrivateSelectorDataRec;
	VDPrivateSelectorDataRec = record
		privateParameters: LogicalAddress;      { Caller's parameters}
		privateParametersSize: ByteCount;  { Size of data sent from caller to driver}
		privateResults: LogicalAddress;         { Caller's return area. Can be nil, or same as privateParameters.}
		privateResultsSize: ByteCount;     { Size of data driver returns to caller. Can be nil, or same as privateParametersSize.}
	end;

type
	VDPrivateSelectorRecPtr = ^VDPrivateSelectorRec;
	VDPrivateSelectorRec = record
		reserved: UInt32;               { Reserved (set to 0). }
		data: array [0..0] of VDPrivateSelectorDataRec;
	end;
type
	VDDDCBlockRecPtr = ^VDDDCBlockRec;
	VDDDCBlockRec = record
		ddcBlockNumber: UInt32;         { Input -- DDC EDID (Extended Display Identification Data) number (1-based) }
		ddcBlockType: ResType;           { Input -- DDC block type (EDID/VDIF) }
		ddcFlags: UInt32;               { Input -- DDC Flags}
		ddcReserved: UInt32;            { Reserved }
   ddcBlockData: array [0..127] of UInt8;      { Output -- DDC EDID/VDIF data (kDDCBlockSize) }
	end;
type
	VDDDCBlockPtr = VDDDCBlockRecPtr;

const
{ timingSyncConfiguration}
	kSyncInterlaceMask = 1 shl 7;
	kSyncAnalogCompositeMask = 0;
	kSyncAnalogCompositeSerrateMask = 1 shl 2;
	kSyncAnalogCompositeRGBSyncMask = 1 shl 1;
	kSyncAnalogBipolarMask = 1 shl 3;
	kSyncAnalogBipolarSerrateMask = 1 shl 2;
	kSyncAnalogBipolarSRGBSyncMask = 1 shl 1;
	kSyncDigitalCompositeMask = 1 shl 4;
	kSyncDigitalCompositeSerrateMask = 1 shl 2;
	kSyncDigitalCompositeMatchHSyncMask = 1 shl 2;
	kSyncDigitalSeperateMask = (1 shl 4) + (1 shl 3);
	kSyncDigitalVSyncPositiveMask = 1 shl 2;
	kSyncDigitalHSyncPositiveMask = 1 shl 1;
=======
=======
>>>>>>> origin/fixes_2_2
		csPreviousDisplayModeID: DisplayModeID;							{  ID of the previous resolution in a chain  }
		csDisplayModeID:		DisplayModeID;							{  ID of the next resolution  }
		csHorizontalPixels:		UInt32;									{  # of pixels in a horizontal line at the max depth  }
		csVerticalLines:		UInt32;									{  # of lines in a screen at the max depth  }
		csRefreshRate:			Fixed;									{  Vertical Refresh Rate in Hz  }
		csMaxDepthMode:			DepthMode;								{  0x80-based number representing max bit depth  }
		csResolutionFlags:		UInt32;									{  Reserved - flag bits  }
		csReserved:				UInt32;									{  Reserved  }
=======
>>>>>>> origin/fixes_2.4
	end;
type
	VDResolutionInfoPtr = VDResolutionInfoRecPtr;
	VDVideoParametersInfoRecPtr = ^VDVideoParametersInfoRec;
	VDVideoParametersInfoRec = record
		csDisplayModeID: DisplayModeID;        { the ID of the resolution we want info on }
		csDepthMode: DepthMode;            { The bit depth we want the info on (0x80 based) }
		csVPBlockPtr: VPBlockPtr;           { Pointer to a video parameter block }
		csPageCount: UNSIGNEDLONG;            { Number of pages supported by the resolution }
		csDeviceType: VideoDeviceType;           { Device Type:  Direct, Fixed or CLUT; }
		csDepthFlags: UInt32;           { Flags }
	end;
type
	VDVideoParametersInfoPtr = VDVideoParametersInfoRecPtr;
	VDGammaInfoRecPtr = ^VDGammaInfoRec;
	VDGammaInfoRec = record
		csLastGammaID: GammaTableID;          { the ID of the previous gamma table }
		csNextGammaID: GammaTableID;          { the ID of the next gamma table }
		csGammaPtr: Ptr;             { Ptr to a gamma table data }
		csReserved: UNSIGNEDLONG;             { Reserved }
	end;
type
	VDGammaInfoPtr = VDGammaInfoRecPtr;
	VDGetGammaListRecPtr = ^VDGetGammaListRec;
	VDGetGammaListRec = record
		csPreviousGammaTableID: GammaTableID; { ID of the previous gamma table }
		csGammaTableID: GammaTableID;         { ID of the gamma table following csPreviousDisplayModeID }
		csGammaTableSize: UNSIGNEDLONG;       { Size of the gamma table in bytes }
		csGammaTableName: CStringPtr;       { Gamma table name (c-string) }
	end;
type
	VDGetGammaListPtr = VDGetGammaListRecPtr;
	VDRetrieveGammaRecPtr = ^VDRetrieveGammaRec;
	VDRetrieveGammaRec = record
		csGammaTableID: GammaTableID;         { ID of gamma table to retrieve }
		csGammaTablePtr: GammaTblPtr;        { Location to copy desired gamma to }
	end;
type
	VDRetrieveGammaPtr = VDRetrieveGammaRecPtr;
	VDSetHardwareCursorRecPtr = ^VDSetHardwareCursorRec;
	VDSetHardwareCursorRec = record
		csCursorRef: UnivPtr;            { reference to cursor data }
		csReserved1: UInt32;            { reserved for future use }
		csReserved2: UInt32;            { should be ignored }
	end;
type
	VDSetHardwareCursorPtr = VDSetHardwareCursorRecPtr;
	VDDrawHardwareCursorRecPtr = ^VDDrawHardwareCursorRec;
	VDDrawHardwareCursorRec = record
		csCursorX: SInt32;              { x coordinate }
		csCursorY: SInt32;              { y coordinate }
		csCursorVisible: UInt32;        { true if cursor is must be visible }
		csReserved1: UInt32;            { reserved for future use }
		csReserved2: UInt32;            { should be ignored }
	end;
type
	VDDrawHardwareCursorPtr = VDDrawHardwareCursorRecPtr;
	VDSupportsHardwareCursorRecPtr = ^VDSupportsHardwareCursorRec;
	VDSupportsHardwareCursorRec = record
		csSupportsHardwareCursor: UInt32;
                                              { true if hardware cursor is supported }
		csReserved1: UInt32;            { reserved for future use }
		csReserved2: UInt32;            { must be zero }
	end;
type
	VDSupportsHardwareCursorPtr = VDSupportsHardwareCursorRecPtr;
	VDHardwareCursorDrawStateRecPtr = ^VDHardwareCursorDrawStateRec;
	VDHardwareCursorDrawStateRec = record
		csCursorX: SInt32;              { x coordinate }
		csCursorY: SInt32;              { y coordinate }
		csCursorVisible: UInt32;        { true if cursor is visible }
		csCursorSet: UInt32;            { true if cursor successfully set by last set control call }
		csReserved1: UInt32;            { reserved for future use }
		csReserved2: UInt32;            { must be zero }
	end;
type
	VDHardwareCursorDrawStatePtr = VDHardwareCursorDrawStateRecPtr;
	VDConvolutionInfoRecPtr = ^VDConvolutionInfoRec;
	VDConvolutionInfoRec = record
		csDisplayModeID: DisplayModeID;        { the ID of the resolution we want info on }
		csDepthMode: DepthMode;            { The bit depth we want the info on (0x80 based) }
		csPage: UNSIGNEDLONG;
		csFlags: UInt32;
		csReserved: UInt32;
	end;
type
	VDConvolutionInfoPtr = VDConvolutionInfoRecPtr;
	VDPowerStateRecPtr = ^VDPowerStateRec;
	VDPowerStateRec = record
		powerState: UNSIGNEDLONG;
		powerFlags: UNSIGNEDLONG;

		powerReserved1: UNSIGNEDLONG;
		powerReserved2: UNSIGNEDLONG;
	end;
type
	VDPowerStatePtr = VDPowerStateRecPtr;
{
    Private Data to video drivers.
    
    In versions of MacOS with multiple address spaces (System 8), the OS 
    must know the extent of parameters in order to move them between the caller
    and driver.  The old private-selector model for video drivers does not have
    this information so:
    
    For post-7.x Systems private calls should be implemented using the cscPrivateCall
}
type
	VDPrivateSelectorDataRecPtr = ^VDPrivateSelectorDataRec;
	VDPrivateSelectorDataRec = record
		privateParameters: LogicalAddress;      { Caller's parameters}
		privateParametersSize: ByteCount;  { Size of data sent from caller to driver}
		privateResults: LogicalAddress;         { Caller's return area. Can be nil, or same as privateParameters.}
		privateResultsSize: ByteCount;     { Size of data driver returns to caller. Can be nil, or same as privateParametersSize.}
	end;

type
	VDPrivateSelectorRecPtr = ^VDPrivateSelectorRec;
	VDPrivateSelectorRec = record
		reserved: UInt32;               { Reserved (set to 0). }
		data: array [0..0] of VDPrivateSelectorDataRec;
	end;
type
	VDDDCBlockRecPtr = ^VDDDCBlockRec;
	VDDDCBlockRec = record
		ddcBlockNumber: UInt32;         { Input -- DDC EDID (Extended Display Identification Data) number (1-based) }
		ddcBlockType: ResType;           { Input -- DDC block type (EDID/VDIF) }
		ddcFlags: UInt32;               { Input -- DDC Flags}
		ddcReserved: UInt32;            { Reserved }
   ddcBlockData: array [0..127] of UInt8;      { Output -- DDC EDID/VDIF data (kDDCBlockSize) }
	end;
type
	VDDDCBlockPtr = VDDDCBlockRecPtr;

const
<<<<<<< HEAD
																{  timingSyncConfiguration }
	kSyncInterlaceMask			= $80;
	kSyncAnalogCompositeMask	= 0;
	kSyncAnalogCompositeSerrateMask = $04;
	kSyncAnalogCompositeRGBSyncMask = $02;
	kSyncAnalogBipolarMask		= $08;
	kSyncAnalogBipolarSerrateMask = $04;
	kSyncAnalogBipolarSRGBSyncMask = $02;
	kSyncDigitalCompositeMask	= $10;
	kSyncDigitalCompositeSerrateMask = $04;
	kSyncDigitalCompositeMatchHSyncMask = $04;
	kSyncDigitalSeperateMask	= $18;
	kSyncDigitalVSyncPositiveMask = $04;
	kSyncDigitalHSyncPositiveMask = $02;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ timingSyncConfiguration}
	kSyncInterlaceMask = 1 shl 7;
	kSyncAnalogCompositeMask = 0;
	kSyncAnalogCompositeSerrateMask = 1 shl 2;
	kSyncAnalogCompositeRGBSyncMask = 1 shl 1;
	kSyncAnalogBipolarMask = 1 shl 3;
	kSyncAnalogBipolarSerrateMask = 1 shl 2;
	kSyncAnalogBipolarSRGBSyncMask = 1 shl 1;
	kSyncDigitalCompositeMask = 1 shl 4;
	kSyncDigitalCompositeSerrateMask = 1 shl 2;
	kSyncDigitalCompositeMatchHSyncMask = 1 shl 2;
	kSyncDigitalSeperateMask = (1 shl 4) + (1 shl 3);
	kSyncDigitalVSyncPositiveMask = 1 shl 2;
	kSyncDigitalHSyncPositiveMask = 1 shl 1;
>>>>>>> origin/fixes_2.4


type
	VDDisplayTimingRangeRecPtr = ^VDDisplayTimingRangeRec;
	VDDisplayTimingRangeRec = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		csRangeSize: UInt32;            { Init to sizeof(VDDisplayTimingRangeRec) }
		csRangeType: UInt32;            { Init to 0 }
		csRangeVersion: UInt32;         { Init to 0 }
		csRangeReserved: UInt32;        { Init to 0 }

		csRangeBlockIndex: UInt32;      { Requested block (first index is 0)}
		csRangeGroup: UInt32;           { set to 0 }
		csRangeBlockCount: UInt32;      { # blocks }
		csRangeFlags: UInt32;           { dependent video }

		csMinPixelClock: UInt64;        { Min dot clock in Hz }
		csMaxPixelClock: UInt64;        { Max dot clock in Hz }

		csMaxPixelError: UInt32;        { Max dot clock error }
		csTimingRangeSyncFlags: UInt32;
<<<<<<< HEAD
		csTimingRangeSignalLevels: UInt32;
		csReserved0: UInt32;

		csMinFrameRate: UInt32;         { Hz }
		csMaxFrameRate: UInt32;         { Hz }
		csMinLineRate: UInt32;          { Hz }
		csMaxLineRate: UInt32;          { Hz }


		csMaxHorizontalTotal: UInt32;   { Clocks - Maximum total (active + blanking) }
		csMaxVerticalTotal: UInt32;     { Clocks - Maximum total (active + blanking) }
		csMaxTotalReserved1: UInt32;    { Reserved }
		csMaxTotalReserved2: UInt32;    { Reserved }


                                              { Some cards require that some timing elements}
                                              { be multiples of a "character size" (often 8}
                                              { clocks).  The "xxxxCharSize" fields document}
                                              { those requirements.}


		csCharSizeHorizontalActive: UInt8; { Character size }
		csCharSizeHorizontalBlanking: UInt8; { Character size }
		csCharSizeHorizontalSyncOffset: UInt8; { Character size }
		csCharSizeHorizontalSyncPulse: UInt8; { Character size }

		csCharSizeVerticalActive: UInt8; { Character size }
		csCharSizeVerticalBlanking: UInt8; { Character size }
		csCharSizeVerticalSyncOffset: UInt8; { Character size }
		csCharSizeVerticalSyncPulse: UInt8; { Character size }

		csCharSizeHorizontalBorderLeft: UInt8; { Character size }
		csCharSizeHorizontalBorderRight: UInt8; { Character size }
		csCharSizeVerticalBorderTop: UInt8; { Character size }
		csCharSizeVerticalBorderBottom: UInt8; { Character size }

		csCharSizeHorizontalTotal: UInt8; { Character size for active + blanking }
		csCharSizeVerticalTotal: UInt8; { Character size for active + blanking }
		csCharSizeReserved1: UInt16;    { Reserved (Init to 0) }


=======
=======
>>>>>>> origin/fixes_2_2
		csRangeSize:			UInt32;									{  Init to sizeof(VDDisplayTimingRangeRec)  }
		csRangeType:			UInt32;									{  Init to 0  }
		csRangeVersion:			UInt32;									{  Init to 0  }
		csRangeReserved:		UInt32;									{  Init to 0  }
		csRangeBlockIndex:		UInt32;									{  Requested block (first index is 0) }
		csRangeGroup:			UInt32;									{  set to 0  }
		csRangeBlockCount:		UInt32;									{  # blocks  }
		csRangeFlags:			UInt32;									{  dependent video  }
		csMinPixelClock:		UInt64;									{  Min dot clock in Hz  }
		csMaxPixelClock:		UInt64;									{  Max dot clock in Hz  }
		csMaxPixelError:		UInt32;									{  Max dot clock error  }
		csTimingRangeSyncFlags:	UInt32;
		csTimingRangeSignalLevels: UInt32;
		csReserved0:			UInt32;
		csMinFrameRate:			UInt32;									{  Hz  }
		csMaxFrameRate:			UInt32;									{  Hz  }
		csMinLineRate:			UInt32;									{  Hz  }
		csMaxLineRate:			UInt32;									{  Hz  }
		csMaxHorizontalTotal:	UInt32;									{  Clocks - Maximum total (active + blanking)  }
		csMaxVerticalTotal:		UInt32;									{  Clocks - Maximum total (active + blanking)  }
		csMaxTotalReserved1:	UInt32;									{  Reserved  }
		csMaxTotalReserved2:	UInt32;									{  Reserved  }
																		{  Some cards require that some timing elements }
																		{  be multiples of a "character size" (often 8 }
																		{  clocks).  The "xxxxCharSize" fields document }
																		{  those requirements. }
		csCharSizeHorizontalActive: SInt8;								{  Character size  }
		csCharSizeHorizontalBlanking: SInt8;							{  Character size  }
		csCharSizeHorizontalSyncOffset: SInt8;							{  Character size  }
		csCharSizeHorizontalSyncPulse: SInt8;							{  Character size  }
		csCharSizeVerticalActive: SInt8;								{  Character size  }
		csCharSizeVerticalBlanking: SInt8;								{  Character size  }
		csCharSizeVerticalSyncOffset: SInt8;							{  Character size  }
		csCharSizeVerticalSyncPulse: SInt8;								{  Character size  }
		csCharSizeHorizontalBorderLeft: SInt8;							{  Character size  }
		csCharSizeHorizontalBorderRight: SInt8;							{  Character size  }
		csCharSizeVerticalBorderTop: SInt8;								{  Character size  }
		csCharSizeVerticalBorderBottom: SInt8;							{  Character size  }
		csCharSizeHorizontalTotal: SInt8;								{  Character size for active + blanking  }
		csCharSizeVerticalTotal: SInt8;									{  Character size for active + blanking  }
		csCharSizeReserved1:	UInt16;									{  Reserved (Init to 0)  }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
		csTimingRangeSignalLevels: UInt32;
		csReserved0: UInt32;

		csMinFrameRate: UInt32;         { Hz }
		csMaxFrameRate: UInt32;         { Hz }
		csMinLineRate: UInt32;          { Hz }
		csMaxLineRate: UInt32;          { Hz }


		csMaxHorizontalTotal: UInt32;   { Clocks - Maximum total (active + blanking) }
		csMaxVerticalTotal: UInt32;     { Clocks - Maximum total (active + blanking) }
		csMaxTotalReserved1: UInt32;    { Reserved }
		csMaxTotalReserved2: UInt32;    { Reserved }


                                              { Some cards require that some timing elements}
                                              { be multiples of a "character size" (often 8}
                                              { clocks).  The "xxxxCharSize" fields document}
                                              { those requirements.}


		csCharSizeHorizontalActive: UInt8; { Character size }
		csCharSizeHorizontalBlanking: UInt8; { Character size }
		csCharSizeHorizontalSyncOffset: UInt8; { Character size }
		csCharSizeHorizontalSyncPulse: UInt8; { Character size }

		csCharSizeVerticalActive: UInt8; { Character size }
		csCharSizeVerticalBlanking: UInt8; { Character size }
		csCharSizeVerticalSyncOffset: UInt8; { Character size }
		csCharSizeVerticalSyncPulse: UInt8; { Character size }

		csCharSizeHorizontalBorderLeft: UInt8; { Character size }
		csCharSizeHorizontalBorderRight: UInt8; { Character size }
		csCharSizeVerticalBorderTop: UInt8; { Character size }
		csCharSizeVerticalBorderBottom: UInt8; { Character size }

		csCharSizeHorizontalTotal: UInt8; { Character size for active + blanking }
		csCharSizeVerticalTotal: UInt8; { Character size for active + blanking }
		csCharSizeReserved1: UInt16;    { Reserved (Init to 0) }


>>>>>>> origin/fixes_2.4
		csMinHorizontalActiveClocks: UInt32;
		csMaxHorizontalActiveClocks: UInt32;
		csMinHorizontalBlankingClocks: UInt32;
		csMaxHorizontalBlankingClocks: UInt32;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
		csMinHorizontalSyncOffsetClocks: UInt32;
		csMaxHorizontalSyncOffsetClocks: UInt32;
		csMinHorizontalPulseWidthClocks: UInt32;
		csMaxHorizontalPulseWidthClocks: UInt32;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
		csMinVerticalActiveClocks: UInt32;
		csMaxVerticalActiveClocks: UInt32;
		csMinVerticalBlankingClocks: UInt32;
		csMaxVerticalBlankingClocks: UInt32;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
		csMinVerticalSyncOffsetClocks: UInt32;
		csMaxVerticalSyncOffsetClocks: UInt32;
		csMinVerticalPulseWidthClocks: UInt32;
		csMaxVerticalPulseWidthClocks: UInt32;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD


=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======


>>>>>>> origin/fixes_2.4
		csMinHorizontalBorderLeft: UInt32;
		csMaxHorizontalBorderLeft: UInt32;
		csMinHorizontalBorderRight: UInt32;
		csMaxHorizontalBorderRight: UInt32;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

		csMinVerticalBorderTop: UInt32;
		csMaxVerticalBorderTop: UInt32;
		csMinVerticalBorderBottom: UInt32;
		csMaxVerticalBorderBottom: UInt32;

		csReserved1: UInt32;            { Reserved (Init to 0)}
		csReserved2: UInt32;            { Reserved (Init to 0)}
		csReserved3: UInt32;            { Reserved (Init to 0)}
		csReserved4: UInt32;            { Reserved (Init to 0)}

		csReserved5: UInt32;            { Reserved (Init to 0)}
		csReserved6: UInt32;            { Reserved (Init to 0)}
		csReserved7: UInt32;            { Reserved (Init to 0)}
		csReserved8: UInt32;            { Reserved (Init to 0)}
	end;
type
	VDDisplayTimingRangePtr = VDDisplayTimingRangeRecPtr;

const
{ csDisplayModeState}
	kDMSModeReady = 0;    { Display Mode ID is configured and ready}
	kDMSModeNotReady = 1;    { Display Mode ID is is being programmed}
	kDMSModeFree = 2;     { Display Mode ID is not associated with a timing}


{ Video driver Errors -10930 to -10959 }
const
	kTimingChangeRestrictedErr = -10930;
	kVideoI2CReplyPendingErr = -10931;
	kVideoI2CTransactionErr = -10932;
	kVideoI2CBusyErr = -10933;
	kVideoI2CTransactionTypeErr = -10934;
	kVideoBufferSizeErr = -10935;


const
{ csTimingRangeSignalLevels}
=======
=======
>>>>>>> origin/fixes_2_2
		csMinVerticalBorderTop:	UInt32;
		csMaxVerticalBorderTop:	UInt32;
=======

		csMinVerticalBorderTop: UInt32;
		csMaxVerticalBorderTop: UInt32;
>>>>>>> origin/fixes_2.4
		csMinVerticalBorderBottom: UInt32;
		csMaxVerticalBorderBottom: UInt32;

		csReserved1: UInt32;            { Reserved (Init to 0)}
		csReserved2: UInt32;            { Reserved (Init to 0)}
		csReserved3: UInt32;            { Reserved (Init to 0)}
		csReserved4: UInt32;            { Reserved (Init to 0)}

		csReserved5: UInt32;            { Reserved (Init to 0)}
		csReserved6: UInt32;            { Reserved (Init to 0)}
		csReserved7: UInt32;            { Reserved (Init to 0)}
		csReserved8: UInt32;            { Reserved (Init to 0)}
	end;
type
	VDDisplayTimingRangePtr = VDDisplayTimingRangeRecPtr;

const
{ csDisplayModeState}
	kDMSModeReady = 0;    { Display Mode ID is configured and ready}
	kDMSModeNotReady = 1;    { Display Mode ID is is being programmed}
	kDMSModeFree = 2;     { Display Mode ID is not associated with a timing}


{ Video driver Errors -10930 to -10959 }
const
	kTimingChangeRestrictedErr = -10930;
	kVideoI2CReplyPendingErr = -10931;
	kVideoI2CTransactionErr = -10932;
	kVideoI2CBusyErr = -10933;
	kVideoI2CTransactionTypeErr = -10934;
	kVideoBufferSizeErr = -10935;


<<<<<<< HEAD
																{  csTimingRangeSignalLevels }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
const
{ csTimingRangeSignalLevels}
>>>>>>> origin/fixes_2.4
	kRangeSupportsSignal_0700_0300_Bit = 0;
	kRangeSupportsSignal_0714_0286_Bit = 1;
	kRangeSupportsSignal_1000_0400_Bit = 2;
	kRangeSupportsSignal_0700_0000_Bit = 3;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
	kRangeSupportsSignal_0700_0300_Mask = 1 shl kRangeSupportsSignal_0700_0300_Bit;
	kRangeSupportsSignal_0714_0286_Mask = 1 shl kRangeSupportsSignal_0714_0286_Bit;
	kRangeSupportsSignal_1000_0400_Mask = 1 shl kRangeSupportsSignal_1000_0400_Bit;
	kRangeSupportsSignal_0700_0000_Mask = 1 shl kRangeSupportsSignal_0700_0000_Bit;
<<<<<<< HEAD


const
{ csSignalConfig}
	kDigitalSignalBit = 0;    { Do not set.  Mac OS does not currently support arbitrary digital timings}
	kAnalogSetupExpectedBit = 1;    { Analog displays - display expects a blank-to-black setup or pedestal.  See VESA signal standards.}
	kDigitalSignalMask = 1 shl kDigitalSignalBit;
	kAnalogSetupExpectedMask = 1 shl kAnalogSetupExpectedBit;


const
{ csSignalLevels for analog}
=======
=======
>>>>>>> origin/fixes_2_2
	kRangeSupportsSignal_0700_0300_Mask = $01;
	kRangeSupportsSignal_0714_0286_Mask = $02;
	kRangeSupportsSignal_1000_0400_Mask = $04;
	kRangeSupportsSignal_0700_0000_Mask = $08;
=======
>>>>>>> origin/fixes_2.4


const
{ csSignalConfig}
	kDigitalSignalBit = 0;    { Do not set.  Mac OS does not currently support arbitrary digital timings}
	kAnalogSetupExpectedBit = 1;    { Analog displays - display expects a blank-to-black setup or pedestal.  See VESA signal standards.}
	kDigitalSignalMask = 1 shl kDigitalSignalBit;
	kAnalogSetupExpectedMask = 1 shl kAnalogSetupExpectedBit;


<<<<<<< HEAD
																{  csSignalLevels for analog }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
const
{ csSignalLevels for analog}
>>>>>>> origin/fixes_2.4
	kAnalogSignalLevel_0700_0300 = 0;
	kAnalogSignalLevel_0714_0286 = 1;
	kAnalogSignalLevel_1000_0400 = 2;
	kAnalogSignalLevel_0700_0000 = 3;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
const
{ csTimingRangeSyncFlags}
=======
																{  csTimingRangeSyncFlags }
>>>>>>> graemeg/fixes_2_2
=======
																{  csTimingRangeSyncFlags }
>>>>>>> origin/fixes_2_2
=======
const
{ csTimingRangeSyncFlags}
>>>>>>> origin/fixes_2.4
	kRangeSupportsSeperateSyncsBit = 0;
	kRangeSupportsSyncOnGreenBit = 1;
	kRangeSupportsCompositeSyncBit = 2;
	kRangeSupportsVSyncSerrationBit = 3;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
	kRangeSupportsSeperateSyncsMask = 1 shl kRangeSupportsSeperateSyncsBit;
	kRangeSupportsSyncOnGreenMask = 1 shl kRangeSupportsSyncOnGreenBit;
	kRangeSupportsCompositeSyncMask = 1 shl kRangeSupportsCompositeSyncBit;
	kRangeSupportsVSyncSerrationMask = 1 shl kRangeSupportsVSyncSerrationBit;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew

=======
>>>>>>> graemeg/cpstrnew

<<<<<<< HEAD
=======

<<<<<<< HEAD

>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======


>>>>>>> origin/cpstrnew
const
{ csHorizontalSyncConfig and csVerticalSyncConfig}
	kSyncPositivePolarityBit = 0;    { Digital separate sync polarity for analog interfaces (0 => negative polarity)}
	kSyncPositivePolarityMask = 1 shl kSyncPositivePolarityBit;


{ For timings with kDetailedTimingFormat.}
type
	VDDetailedTimingRecPtr = ^VDDetailedTimingRec;
	VDDetailedTimingRec = record
		csTimingSize: UInt32;           { Init to sizeof(VDDetailedTimingRec)}
		csTimingType: UInt32;           { Init to 0}
		csTimingVersion: UInt32;        { Init to 0}
		csTimingReserved: UInt32;       { Init to 0}

		csDisplayModeID: DisplayModeID;        { Init to 0}
		csDisplayModeSeed: UInt32;      { }
		csDisplayModeState: UInt32;     { Display Mode state}
		csDisplayModeAlias: UInt32;     { Mode to use when programmed.}

		csSignalConfig: UInt32;
		csSignalLevels: UInt32;

		csPixelClock: UInt64;           { Hz}

		csMinPixelClock: UInt64;        { Hz - With error what is slowest actual clock }
		csMaxPixelClock: UInt64;        { Hz - With error what is fasted actual clock }


		csHorizontalActive: UInt32;     { Pixels}
		csHorizontalBlanking: UInt32;   { Pixels}
		csHorizontalSyncOffset: UInt32; { Pixels}
		csHorizontalSyncPulseWidth: UInt32; { Pixels}

		csVerticalActive: UInt32;       { Lines}
		csVerticalBlanking: UInt32;     { Lines}
		csVerticalSyncOffset: UInt32;   { Lines}
		csVerticalSyncPulseWidth: UInt32; { Lines}

		csHorizontalBorderLeft: UInt32; { Pixels}
		csHorizontalBorderRight: UInt32; { Pixels}
		csVerticalBorderTop: UInt32;    { Lines}
		csVerticalBorderBottom: UInt32; { Lines}

		csHorizontalSyncConfig: UInt32;
		csHorizontalSyncLevel: UInt32;  { Future use (init to 0)}
		csVerticalSyncConfig: UInt32;
		csVerticalSyncLevel: UInt32;    { Future use (init to 0)}

		csReserved1: UInt32;            { Init to 0}
		csReserved2: UInt32;            { Init to 0}
		csReserved3: UInt32;            { Init to 0}
		csReserved4: UInt32;            { Init to 0}

		csReserved5: UInt32;            { Init to 0}
		csReserved6: UInt32;            { Init to 0}
		csReserved7: UInt32;            { Init to 0}
		csReserved8: UInt32;            { Init to 0}
	end;
type
	VDDetailedTimingPtr = VDDetailedTimingRecPtr;
	VDClutBehavior = UInt32;
	VDClutBehaviorPtr = ^VDClutBehavior;
const
	kSetClutAtSetEntries = 0;    { SetEntries behavior is to update clut during SetEntries call}
	kSetClutAtVBL = 1;     { SetEntries behavior is to upate clut at next vbl}
=======
=======
>>>>>>> origin/fixes_2_2
	kRangeSupportsSeperateSyncsMask = $01;
	kRangeSupportsSyncOnGreenMask = $02;
	kRangeSupportsCompositeSyncMask = $04;
	kRangeSupportsVSyncSerrationMask = $08;
=======
>>>>>>> origin/fixes_2.4


const
{ csHorizontalSyncConfig and csVerticalSyncConfig}
	kSyncPositivePolarityBit = 0;    { Digital separate sync polarity for analog interfaces (0 => negative polarity)}
	kSyncPositivePolarityMask = 1 shl kSyncPositivePolarityBit;


{ For timings with kDetailedTimingFormat.}
type
	VDDetailedTimingRecPtr = ^VDDetailedTimingRec;
	VDDetailedTimingRec = record
		csTimingSize: UInt32;           { Init to sizeof(VDDetailedTimingRec)}
		csTimingType: UInt32;           { Init to 0}
		csTimingVersion: UInt32;        { Init to 0}
		csTimingReserved: UInt32;       { Init to 0}

		csDisplayModeID: DisplayModeID;        { Init to 0}
		csDisplayModeSeed: UInt32;      { }
		csDisplayModeState: UInt32;     { Display Mode state}
		csDisplayModeAlias: UInt32;     { Mode to use when programmed.}

		csSignalConfig: UInt32;
		csSignalLevels: UInt32;

		csPixelClock: UInt64;           { Hz}

		csMinPixelClock: UInt64;        { Hz - With error what is slowest actual clock }
		csMaxPixelClock: UInt64;        { Hz - With error what is fasted actual clock }


		csHorizontalActive: UInt32;     { Pixels}
		csHorizontalBlanking: UInt32;   { Pixels}
		csHorizontalSyncOffset: UInt32; { Pixels}
		csHorizontalSyncPulseWidth: UInt32; { Pixels}

		csVerticalActive: UInt32;       { Lines}
		csVerticalBlanking: UInt32;     { Lines}
		csVerticalSyncOffset: UInt32;   { Lines}
		csVerticalSyncPulseWidth: UInt32; { Lines}

		csHorizontalBorderLeft: UInt32; { Pixels}
		csHorizontalBorderRight: UInt32; { Pixels}
		csVerticalBorderTop: UInt32;    { Lines}
		csVerticalBorderBottom: UInt32; { Lines}

		csHorizontalSyncConfig: UInt32;
		csHorizontalSyncLevel: UInt32;  { Future use (init to 0)}
		csVerticalSyncConfig: UInt32;
		csVerticalSyncLevel: UInt32;    { Future use (init to 0)}

		csReserved1: UInt32;            { Init to 0}
		csReserved2: UInt32;            { Init to 0}
		csReserved3: UInt32;            { Init to 0}
		csReserved4: UInt32;            { Init to 0}

		csReserved5: UInt32;            { Init to 0}
		csReserved6: UInt32;            { Init to 0}
		csReserved7: UInt32;            { Init to 0}
		csReserved8: UInt32;            { Init to 0}
	end;
type
	VDDetailedTimingPtr = VDDetailedTimingRecPtr;
	VDClutBehavior = UInt32;
	VDClutBehaviorPtr = ^VDClutBehavior;
const
<<<<<<< HEAD
	kSetClutAtSetEntries		= 0;							{  SetEntries behavior is to update clut during SetEntries call }
	kSetClutAtVBL				= 1;							{  SetEntries behavior is to upate clut at next vbl }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kSetClutAtSetEntries = 0;    { SetEntries behavior is to update clut during SetEntries call}
	kSetClutAtVBL = 1;     { SetEntries behavior is to upate clut at next vbl}
>>>>>>> origin/fixes_2.4


type
	VDCommunicationRecPtr = ^VDCommunicationRec;
	VDCommunicationRec = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		csBusID: SInt32;                { kVideoDefaultBus for single headed cards.}
		csCommFlags: UInt32;            { Always zero}
		csMinReplyDelay: UInt32;        { Minimum delay between send and reply transactions (units depend on csCommFlags)}
		csReserved2: UInt32;            { Always zero}

		csSendAddress: UInt32;          { Usually I2C address (eg 0x6E)}
		csSendType: UInt32;             { See kVideoSimpleI2CType etc.}
		csSendBuffer: LogicalAddress;           { Pointer to the send buffer}
		csSendSize: ByteCount;             { Number of bytes to send}

		csReplyAddress: UInt32;         { Address from which to read (eg 0x6F for kVideoDDCciReplyType I2C address)}
		csReplyType: UInt32;            { See kVideoDDCciReplyType etc.}
		csReplyBuffer: LogicalAddress;          { Pointer to the reply buffer}
		csReplySize: ByteCount;            { Max bytes to reply (size of csReplyBuffer)}

		csReserved3: UInt32;
		csReserved4: UInt32;
		csReserved5: UInt32;            { Always zero}
		csReserved6: UInt32;            { Always zero}
<<<<<<< HEAD
	end;
type
	VDCommunicationPtr = VDCommunicationRecPtr;
	VDCommunicationInfoRecPtr = ^VDCommunicationInfoRec;
	VDCommunicationInfoRec = record
		csBusID: SInt32;                { kVideoDefaultBus for single headed cards. }
		csBusType: UInt32;              { See kVideoBusI2C etc.}
		csMinBus: SInt32;               { Minimum bus (usually kVideoDefaultBus).  Used to probe additional busses}
		csMaxBus: SInt32;               { Max bus (usually kVideoDefaultBus).  Used to probe additional busses}

		csSupportedTypes: UInt32;       { Bit field for first 32 supported transaction types.  Eg. 0x07 => support for kVideoNoTransactionType, kVideoSimpleI2CType and kVideoDDCciReplyType.}
		csSupportedCommFlags: UInt32;   { Return the flags csCommFlags understood by this driver.}
		csReserved2: UInt32;            { Always zero}
		csReserved3: UInt32;            { Always zero}

		csReserved4: UInt32;            { Always zero}
		csReserved5: UInt32;            { Always zero}
		csReserved6: UInt32;            { Always zero}
		csReserved7: UInt32;            { Always zero}
	end;
type
	VDCommunicationInfoPtr = VDCommunicationInfoRecPtr;

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
		csBusID:				SInt32;									{  kVideoDefaultBus for single headed cards. }
		csCommFlags:			UInt32;									{  Always zero }
		csMinReplyDelay:		UInt32;									{  Minimum delay between send and reply transactions (units depend on csCommFlags) }
		csReserved2:			UInt32;									{  Always zero }
		csSendAddress:			UInt32;									{  Usually I2C address (eg 0x6E) }
		csSendType:				UInt32;									{  See kVideoSimpleI2CType etc. }
		csSendBuffer:			LogicalAddress;							{  Pointer to the send buffer }
		csSendSize:				ByteCount;								{  Number of bytes to send }
		csReplyAddress:			UInt32;									{  Address from which to read (eg 0x6F for kVideoDDCciReplyType I2C address) }
		csReplyType:			UInt32;									{  See kVideoDDCciReplyType etc. }
		csReplyBuffer:			LogicalAddress;							{  Pointer to the reply buffer }
		csReplySize:			ByteCount;								{  Max bytes to reply (size of csReplyBuffer) }
		csReserved3:			UInt32;
		csReserved4:			UInt32;
		csReserved5:			UInt32;									{  Always zero }
		csReserved6:			UInt32;									{  Always zero }
=======
>>>>>>> origin/fixes_2.4
	end;
type
	VDCommunicationPtr = VDCommunicationRecPtr;
	VDCommunicationInfoRecPtr = ^VDCommunicationInfoRec;
	VDCommunicationInfoRec = record
		csBusID: SInt32;                { kVideoDefaultBus for single headed cards. }
		csBusType: UInt32;              { See kVideoBusI2C etc.}
		csMinBus: SInt32;               { Minimum bus (usually kVideoDefaultBus).  Used to probe additional busses}
		csMaxBus: SInt32;               { Max bus (usually kVideoDefaultBus).  Used to probe additional busses}

		csSupportedTypes: UInt32;       { Bit field for first 32 supported transaction types.  Eg. 0x07 => support for kVideoNoTransactionType, kVideoSimpleI2CType and kVideoDDCciReplyType.}
		csSupportedCommFlags: UInt32;   { Return the flags csCommFlags understood by this driver.}
		csReserved2: UInt32;            { Always zero}
		csReserved3: UInt32;            { Always zero}

		csReserved4: UInt32;            { Always zero}
		csReserved5: UInt32;            { Always zero}
		csReserved6: UInt32;            { Always zero}
		csReserved7: UInt32;            { Always zero}
	end;
type
	VDCommunicationInfoPtr = VDCommunicationInfoRecPtr;

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
