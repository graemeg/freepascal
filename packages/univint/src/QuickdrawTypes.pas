{
     File:       QD/QuickdrawTypes.h
 
     Contains:   Type definitions from the former Quickdraw.i
 
     Version:    Quickdraw-262~1
 
     Copyright:  © 2005-2008 by Apple Inc. all rights reserved.
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
=======
                     http://www.freepascal.org/bugs.html
>>>>>>> graemeg/cpstrnew
=======
                     http://www.freepascal.org/bugs.html
>>>>>>> graemeg/cpstrnew
=======
                     http://www.freepascal.org/bugs.html
>>>>>>> graemeg/cpstrnew
=======
                     http://www.freepascal.org/bugs.html
>>>>>>> origin/cpstrnew
=======
                     http://www.freepascal.org/bugs.html
>>>>>>> origin/fixes_2.4
 
}
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{
    Modified for use with Free Pascal
    Version 308
    Please report any bugs to <gpc@microbizz.nl>
}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
{$mode macpas}
{$packenum 1}
{$macro on}
{$inline on}
{$calling mwpascal}

unit QuickdrawTypes;
interface
{$setc UNIVERSAL_INTERFACES_VERSION := $0400}
{$setc GAP_INTERFACES_VERSION := $0308}

{$ifc not defined USE_CFSTR_CONSTANT_MACROS}
    {$setc USE_CFSTR_CONSTANT_MACROS := TRUE}
{$endc}

{$ifc defined CPUPOWERPC and defined CPUI386}
	{$error Conflicting initial definitions for CPUPOWERPC and CPUI386}
{$endc}
{$ifc defined FPC_BIG_ENDIAN and defined FPC_LITTLE_ENDIAN}
	{$error Conflicting initial definitions for FPC_BIG_ENDIAN and FPC_LITTLE_ENDIAN}
{$endc}

{$ifc not defined __ppc__ and defined CPUPOWERPC32}
	{$setc __ppc__ := 1}
{$elsec}
	{$setc __ppc__ := 0}
{$endc}
{$ifc not defined __ppc64__ and defined CPUPOWERPC64}
	{$setc __ppc64__ := 1}
{$elsec}
	{$setc __ppc64__ := 0}
{$endc}
{$ifc not defined __i386__ and defined CPUI386}
	{$setc __i386__ := 1}
{$elsec}
	{$setc __i386__ := 0}
{$endc}
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


{$ifc defined __ppc__ and __ppc__ and defined __i386__ and __i386__}
	{$error Conflicting definitions for __ppc__ and __i386__}
{$endc}

{$ifc defined __ppc__ and __ppc__}
	{$setc TARGET_CPU_PPC := TRUE}
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
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/fixes_2.4
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
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/fixes_2.4
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
=======
>>>>>>> origin/fixes_2.4
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
=======
>>>>>>> origin/fixes_2.4
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
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/fixes_2.4
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
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
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
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> origin/cpstrnew
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> origin/fixes_2.4
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
  {$setc TARGET_CPU_64 := FALSE}
{$endc}

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
{$setc TARGET_OS_UNIX := FALSE}
{$setc TARGET_OS_WIN32 := FALSE}
{$setc TARGET_RT_MAC_68881 := FALSE}
{$setc TARGET_RT_MAC_CFM := FALSE}
{$setc TARGET_RT_MAC_MACHO := TRUE}
{$setc TYPED_FUNCTION_POINTERS := TRUE}
{$setc TYPE_BOOL := FALSE}
{$setc TYPE_EXTENDED := FALSE}
{$setc TYPE_LONGLONG := TRUE}
uses MacTypes,QuickdrawText;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}

const
	invalColReq = -1;    {invalid color table request}

const
{ transfer modes }
	srcCopy = 0;    {the 16 transfer modes}
	srcOr = 1;
	srcXor = 2;
	srcBic = 3;
	notSrcCopy = 4;
	notSrcOr = 5;
	notSrcXor = 6;
	notSrcBic = 7;
	patCopy = 8;
	patOr = 9;
	patXor = 10;
	patBic = 11;
	notPatCopy = 12;
	notPatOr = 13;
	notPatXor = 14;
	notPatBic = 15;   { Special Text Transfer Mode }
	grayishTextOr = 49;
	hilitetransfermode = 50;
	hilite = 50;   { Arithmetic transfer modes }
	blend = 32;
	addPin = 33;
	addOver = 34;
	subPin = 35;
	addMax = 37;
	adMax = 37;
	subOver = 38;
	adMin = 39;
	ditherCopy = 64;   { Transparent mode constant }
	transparent = 36;

const
	italicBit = 1;
	ulineBit = 2;
	outlineBit = 3;
	shadowBit = 4;
	condenseBit = 5;
	extendBit = 6;

const
{ QuickDraw color separation constants }
	normalBit = 0;    {normal screen mapping}
	inverseBit = 1;    {inverse screen mapping}
	redBit = 4;    {RGB additive mapping}
	greenBit = 3;
	blueBit = 2;
	cyanBit = 8;    {CMYBk subtractive mapping}
	magentaBit = 7;
	yellowBit = 6;
	blackBit = 5;

const
	blackColor = 33;   { colors expressed in these mappings }
	whiteColor = 30;
	redColor = 205;
	greenColor = 341;
	blueColor = 409;
	cyanColor = 273;
	magentaColor = 137;
	yellowColor = 69;

const
	picLParen = 0;    { standard picture comments }
	picRParen = 1;

{ gdType values }
const
	clutType = 0;    { lookup table }
	fixedType = 1;    { fixed table - now unused }
	directType = 2;     { direct values }

{ gdFlags bits. Bits 1..10 are legacy, and currently unused }
const
	gdDevType = 0;    { 0 = monochrome 1 = color }
	interlacedDevice = 2;
	hwMirroredDevice = 4;
	roundedDevice = 5;
	hasAuxMenuBar = 6;
	burstDevice = 7;
	ext32Device = 8;
	ramInit = 10;
	mainScreen = 11;   { 1 if main screen }
	allInit = 12;   { 1 if all devices initialized }
	screenDevice = 13;   { 1 if screen device }
	noDriver = 14;   { 1 if no driver for this GDevice }
	screenActive = 15;    { 1 if in use}

const
	hiliteBit = 7;    { flag bit in LMGet/SetHiliteMode }
	pHiliteBit = 0;     { flag bit in LMGet/SetHiliteMode when used with BitClr }

{ miscellaneous constants }
const
	defQDColors = 127;  { (legacy - now unused) }
	RGBDirect = 16;   { 16 & 32 bits/pixel pixelType value }
	baseAddr32 = 4;     { pmVersion value: pixmap base address is 32-bit address }

const
	sysPatListID = 0;
	iBeamCursor = 1;
	crossCursor = 2;
	plusCursor = 3;
	watchCursor = 4;

const
	kQDGrafVerbFrame = 0;
	kQDGrafVerbPaint = 1;
	kQDGrafVerbErase = 2;
	kQDGrafVerbInvert = 3;
	kQDGrafVerbFill = 4;

{$ifc OLDROUTINENAMES}
const
	frame = kQDGrafVerbFrame;
	paint = kQDGrafVerbPaint;
	erase = kQDGrafVerbErase;
	invert = kQDGrafVerbInvert;
	fill = kQDGrafVerbFill;

{$endc} {OLDROUTINENAMES}

type
	GrafVerb = SInt8;
const
	chunky = 0;
	chunkyPlanar = 1;
	planar = 2;

type
	PixelType = SInt8;
  Bits16 = array [0..15] of SInt16;

{**************   IMPORTANT NOTE REGARDING Pattern  **************************************
   Patterns were originally defined as:
   
        C:          typedef unsigned char Pattern[8];
        Pascal:     Pattern = PACKED ARRAY [0..7] OF 0..255;
        
   The old array definition of Pattern would cause 68000 based CPU's to crash in certain circum-
   stances. The new struct definition is safe, but may require source code changes to compile.
    
********************************************************************************************}
type
	PatternPtr = ^Pattern;
	Pattern = record
		pat: packed array [0..7] of UInt8;
	end;
{
 ConstPatternParam is no longer needed.  It was first created when Pattern was an array.
 Now that Pattern is a struct, it is more straight forward to just add the "const" qualifier
 on the parameter type (e.g. "const Pattern * pat" instead of "ConstPatternParam pat").

	ConstPatternParam = PatternPtr;
}
type
	PatPtr = PatternPtr;
	PatHandle = ^PatPtr;
	QDByte = SignedByte;
	QDPtr = ^QDByte;
	QDHandle = ^QDPtr;
	QDErr = SInt16;
const
	singleDevicesBit = 0;
	dontMatchSeedsBit = 1;
	allDevicesBit = 2;

const
	singleDevices = 1 shl singleDevicesBit;
	dontMatchSeeds = 1 shl dontMatchSeedsBit;
	allDevices = 1 shl allDevicesBit;

type
	DeviceLoopFlags = UNSIGNEDLONG;
{
    PrinterStatusOpcode.  For communication with downloading and printing services.
}
type
	PrinterStatusOpcode = SInt32;
const
	kPrinterFontStatus = 0;
	kPrinterScalingStatus = 1;

type
	PrinterFontStatusPtr = ^PrinterFontStatus;
	PrinterFontStatus = record
		oResult: SInt32;
		iFondID: SInt16;
		iStyle: Style;
	end;
type
	PrinterScalingStatusPtr = ^PrinterScalingStatus;
	PrinterScalingStatus = record
		oScalingFactors: Point;
	end;
type
	BitMap = record
		baseAddr: Ptr;
		rowBytes: SInt16;
		bounds: Rect;
	end;
	BitMapPtr = ^BitMap;
type
	BitMapHandle = ^BitMapPtr;
	CursorPtr = ^Cursor;
	Cursor = record
		data: Bits16;
		mask: Bits16;
		hotSpot: Point;
	end;
type
	CursPtr = CursorPtr;
	CursHandle = ^CursPtr;
	PenStatePtr = ^PenState;
	PenState = record
		pnLoc: Point;
		pnSize: Point;
		pnMode: SInt16;
		pnPat: Pattern;
	end;
{$ifc not OPAQUE_TOOLBOX_STRUCTS}
type
	MacRegionPtr = ^MacRegion;
	MacRegion = record
		rgnSize: UInt16;                { size in bytes; don't rely on it }
		rgnBBox: Rect;                { enclosing rectangle; in Carbon use GetRegionBounds }
	end;
{
   The type name "Region" has a name space collision on Win32.
   Use MacRegion to be cross-platfrom safe.
}
type
	Region = MacRegion;
	RgnPtr = MacRegionPtr;
	RgnHandle = ^RgnPtrP;
	RgnHandlePtr = ^RgnHandle;  { when a var xx:RgnHandle parameter can be nil, it is changed to xx: RgnHandlePtr }
{$elsec}
type
	RgnHandle = ^SInt32; { an opaque type }
{$endc} {not OPAQUE_TOOLBOX_STRUCTS}

type
	PicturePtr = ^Picture;
	Picture = record
		picSize: SInt16;
		picFrame: Rect;
	end;
type
	PicPtr = PicturePtr;
	PicHandle = ^PicPtr;
	MacPolygon = record
		polySize: SInt16;
		polyBBox: Rect;
		polyPoints: array [0..0] of Point;
	end;
{
   The type name "Polygon" has a name space collision on Win32.
   Use MacPolygon to be cross-platfrom safe.
}
type
	Polygon = MacPolygon;
	PolyPtr = ^MacPolygon;
	PolyHandle = ^PolyPtr;
	QDTextProcPtr = procedure( byteCount: SInt16; textBuf: {const} UnivPtr; numer: Point; denom: Point );
	QDLineProcPtr = procedure( newPt: Point );
	QDRectProcPtr = procedure( verb: GrafVerb; const (*var*) r: Rect );
	QDRRectProcPtr = procedure( verb: GrafVerb; const (*var*) r: Rect; ovalWidth: SInt16; ovalHeight: SInt16 );
	QDOvalProcPtr = procedure( verb: GrafVerb; const (*var*) r: Rect );
	QDArcProcPtr = procedure( verb: GrafVerb; const (*var*) r: Rect; startAngle: SInt16; arcAngle: SInt16 );
	QDPolyProcPtr = procedure( verb: GrafVerb; poly: PolyHandle );
	QDRgnProcPtr = procedure( verb: GrafVerb; rgn: RgnHandle );
	QDBitsProcPtr = procedure( const (*var*) srcBits: BitMap; const (*var*) srcRect: Rect; const (*var*) dstRect: Rect; mode: SInt16; maskRgn: RgnHandle );
	QDCommentProcPtr = procedure( kind: SInt16; dataSize: SInt16; dataHandle: Handle );
	QDTxMeasProcPtr = function( byteCount: SInt16; textAddr: {const} UnivPtr; var numer: Point; var denom: Point; var info: FontInfo ): SInt16;
	QDGetPicProcPtr = procedure( dataPtr: UnivPtr; byteCount: SInt16 );
	QDPutPicProcPtr = procedure( dataPtr: {const} UnivPtr; byteCount: SInt16 );
	QDOpcodeProcPtr = procedure( const (*var*) fromRect: Rect; const (*var*) toRect: Rect; opcode: UInt16; version: SInt16 );
{ The following is unused on Mac OS X - ignore it! }
type
	QDStdGlyphsProcPtr = function( dataStream: UnivPtr; size: ByteCount ): OSStatus;
	QDJShieldCursorProcPtr = procedure( left: SInt16; top: SInt16; right: SInt16; bottom: SInt16 );
	QDTextUPP = QDTextProcPtr;
	QDLineUPP = QDLineProcPtr;
	QDRectUPP = QDRectProcPtr;
	QDRRectUPP = QDRRectProcPtr;
	QDOvalUPP = QDOvalProcPtr;
	QDArcUPP = QDArcProcPtr;
	QDPolyUPP = QDPolyProcPtr;
	QDRgnUPP = QDRgnProcPtr;
	QDBitsUPP = QDBitsProcPtr;
	QDCommentUPP = QDCommentProcPtr;
	QDTxMeasUPP = QDTxMeasProcPtr;
	QDGetPicUPP = QDGetPicProcPtr;
	QDPutPicUPP = QDPutPicProcPtr;
	QDOpcodeUPP = QDOpcodeProcPtr;
	QDStdGlyphsUPP = QDStdGlyphsProcPtr;
	QDJShieldCursorUPP = QDJShieldCursorProcPtr;
	QDProcs = record
		textProc: QDTextUPP;
		lineProc: QDLineUPP;
		rectProc: QDRectUPP;
		rRectProc: QDRRectUPP;
		ovalProc: QDOvalUPP;
		arcProc: QDArcUPP;
		polyProc: QDPolyUPP;
		rgnProc: QDRgnUPP;
		bitsProc: QDBitsUPP;
		commentProc: QDCommentUPP;
		txMeasProc: QDTxMeasUPP;
		getPicProc: QDGetPicUPP;
		putPicProc: QDPutPicUPP;
	end;
	QDProcsPtr = ^QDProcs;
{
 *  NewQDTextUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewQDTextUPP( userRoutine: QDTextProcPtr ): QDTextUPP; external name '_NewQDTextUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  NewQDLineUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewQDLineUPP( userRoutine: QDLineProcPtr ): QDLineUPP; external name '_NewQDLineUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  NewQDRectUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewQDRectUPP( userRoutine: QDRectProcPtr ): QDRectUPP; external name '_NewQDRectUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  NewQDRRectUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewQDRRectUPP( userRoutine: QDRRectProcPtr ): QDRRectUPP; external name '_NewQDRRectUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  NewQDOvalUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewQDOvalUPP( userRoutine: QDOvalProcPtr ): QDOvalUPP; external name '_NewQDOvalUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  NewQDArcUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewQDArcUPP( userRoutine: QDArcProcPtr ): QDArcUPP; external name '_NewQDArcUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  NewQDPolyUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewQDPolyUPP( userRoutine: QDPolyProcPtr ): QDPolyUPP; external name '_NewQDPolyUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  NewQDRgnUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewQDRgnUPP( userRoutine: QDRgnProcPtr ): QDRgnUPP; external name '_NewQDRgnUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  NewQDBitsUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewQDBitsUPP( userRoutine: QDBitsProcPtr ): QDBitsUPP; external name '_NewQDBitsUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  NewQDCommentUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewQDCommentUPP( userRoutine: QDCommentProcPtr ): QDCommentUPP; external name '_NewQDCommentUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  NewQDTxMeasUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewQDTxMeasUPP( userRoutine: QDTxMeasProcPtr ): QDTxMeasUPP; external name '_NewQDTxMeasUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  NewQDGetPicUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewQDGetPicUPP( userRoutine: QDGetPicProcPtr ): QDGetPicUPP; external name '_NewQDGetPicUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  NewQDPutPicUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewQDPutPicUPP( userRoutine: QDPutPicProcPtr ): QDPutPicUPP; external name '_NewQDPutPicUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  NewQDOpcodeUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewQDOpcodeUPP( userRoutine: QDOpcodeProcPtr ): QDOpcodeUPP; external name '_NewQDOpcodeUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  NewQDStdGlyphsUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewQDStdGlyphsUPP( userRoutine: QDStdGlyphsProcPtr ): QDStdGlyphsUPP; external name '_NewQDStdGlyphsUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  NewQDJShieldCursorUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewQDJShieldCursorUPP( userRoutine: QDJShieldCursorProcPtr ): QDJShieldCursorUPP; external name '_NewQDJShieldCursorUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposeQDTextUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeQDTextUPP( userUPP: QDTextUPP ); external name '_DisposeQDTextUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposeQDLineUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeQDLineUPP( userUPP: QDLineUPP ); external name '_DisposeQDLineUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposeQDRectUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeQDRectUPP( userUPP: QDRectUPP ); external name '_DisposeQDRectUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposeQDRRectUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeQDRRectUPP( userUPP: QDRRectUPP ); external name '_DisposeQDRRectUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposeQDOvalUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeQDOvalUPP( userUPP: QDOvalUPP ); external name '_DisposeQDOvalUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposeQDArcUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeQDArcUPP( userUPP: QDArcUPP ); external name '_DisposeQDArcUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposeQDPolyUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeQDPolyUPP( userUPP: QDPolyUPP ); external name '_DisposeQDPolyUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposeQDRgnUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeQDRgnUPP( userUPP: QDRgnUPP ); external name '_DisposeQDRgnUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposeQDBitsUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeQDBitsUPP( userUPP: QDBitsUPP ); external name '_DisposeQDBitsUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposeQDCommentUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeQDCommentUPP( userUPP: QDCommentUPP ); external name '_DisposeQDCommentUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposeQDTxMeasUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeQDTxMeasUPP( userUPP: QDTxMeasUPP ); external name '_DisposeQDTxMeasUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposeQDGetPicUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeQDGetPicUPP( userUPP: QDGetPicUPP ); external name '_DisposeQDGetPicUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposeQDPutPicUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeQDPutPicUPP( userUPP: QDPutPicUPP ); external name '_DisposeQDPutPicUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposeQDOpcodeUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeQDOpcodeUPP( userUPP: QDOpcodeUPP ); external name '_DisposeQDOpcodeUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposeQDStdGlyphsUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeQDStdGlyphsUPP( userUPP: QDStdGlyphsUPP ); external name '_DisposeQDStdGlyphsUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposeQDJShieldCursorUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeQDJShieldCursorUPP( userUPP: QDJShieldCursorUPP ); external name '_DisposeQDJShieldCursorUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokeQDTextUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeQDTextUPP( byteCount: SInt16; textBuf: {const} UnivPtr; numer: Point; denom: Point; userUPP: QDTextUPP ); external name '_InvokeQDTextUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokeQDLineUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeQDLineUPP( newPt: Point; userUPP: QDLineUPP ); external name '_InvokeQDLineUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokeQDRectUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeQDRectUPP( verb: GrafVerb; const (*var*) r: Rect; userUPP: QDRectUPP ); external name '_InvokeQDRectUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokeQDRRectUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeQDRRectUPP( verb: GrafVerb; const (*var*) r: Rect; ovalWidth: SInt16; ovalHeight: SInt16; userUPP: QDRRectUPP ); external name '_InvokeQDRRectUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokeQDOvalUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeQDOvalUPP( verb: GrafVerb; const (*var*) r: Rect; userUPP: QDOvalUPP ); external name '_InvokeQDOvalUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokeQDArcUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeQDArcUPP( verb: GrafVerb; const (*var*) r: Rect; startAngle: SInt16; arcAngle: SInt16; userUPP: QDArcUPP ); external name '_InvokeQDArcUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokeQDPolyUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeQDPolyUPP( verb: GrafVerb; poly: PolyHandle; userUPP: QDPolyUPP ); external name '_InvokeQDPolyUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokeQDRgnUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeQDRgnUPP( verb: GrafVerb; rgn: RgnHandle; userUPP: QDRgnUPP ); external name '_InvokeQDRgnUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokeQDBitsUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeQDBitsUPP( const (*var*) srcBits: BitMap; const (*var*) srcRect: Rect; const (*var*) dstRect: Rect; mode: SInt16; maskRgn: RgnHandle; userUPP: QDBitsUPP ); external name '_InvokeQDBitsUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokeQDCommentUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeQDCommentUPP( kind: SInt16; dataSize: SInt16; dataHandle: Handle; userUPP: QDCommentUPP ); external name '_InvokeQDCommentUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokeQDTxMeasUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeQDTxMeasUPP( byteCount: SInt16; textAddr: {const} UnivPtr; var numer: Point; var denom: Point; var info: FontInfo; userUPP: QDTxMeasUPP ): SInt16; external name '_InvokeQDTxMeasUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokeQDGetPicUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeQDGetPicUPP( dataPtr: UnivPtr; byteCount: SInt16; userUPP: QDGetPicUPP ); external name '_InvokeQDGetPicUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokeQDPutPicUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeQDPutPicUPP( dataPtr: {const} UnivPtr; byteCount: SInt16; userUPP: QDPutPicUPP ); external name '_InvokeQDPutPicUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokeQDOpcodeUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeQDOpcodeUPP( const (*var*) fromRect: Rect; const (*var*) toRect: Rect; opcode: UInt16; version: SInt16; userUPP: QDOpcodeUPP ); external name '_InvokeQDOpcodeUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokeQDStdGlyphsUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeQDStdGlyphsUPP( dataStream: UnivPtr; size: ByteCount; userUPP: QDStdGlyphsUPP ): OSStatus; external name '_InvokeQDStdGlyphsUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokeQDJShieldCursorUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeQDJShieldCursorUPP( left: SInt16; top: SInt16; right: SInt16; bottom: SInt16; userUPP: QDJShieldCursorUPP ); external name '_InvokeQDJShieldCursorUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{$ifc not OPAQUE_TOOLBOX_STRUCTS}
type
	GrafPortPtr = ^GrafPort;
	GrafPort = record
		device: SInt16;                 { not available in Carbon}
		portBits: BitMap;               { in Carbon use GetPortBitMapForCopyBits or IsPortColor}
		portRect: Rect;               { in Carbon use Get/SetPortBounds}
		visRgn: RgnHandle;                 { in Carbon use Get/SetPortVisibleRegion}
		clipRgn: RgnHandle;                { in Carbon use Get/SetPortClipRegion}
		bkPat: Pattern;                  { not available in Carbon all GrafPorts are CGrafPorts}
		fillPat: Pattern;                { not available in Carbon all GrafPorts are CGrafPorts}
		pnLoc: Point;                  { in Carbon use GetPortPenLocation or MoveTo}
		pnSize: Point;                 { in Carbon use Get/SetPortPenSize}
		pnMode: SInt16;                 { in Carbon use Get/SetPortPenMode}
		pnPat: Pattern;                  { not available in Carbon all GrafPorts are CGrafPorts}
		pnVis: SInt16;                  { in Carbon use GetPortPenVisibility or Show/HidePen}
		txFont: SInt16;                 { in Carbon use GetPortTextFont or TextFont}
		txFace: StyleField;                 { in Carbon use GetPortTextFace or TextFace}
                                              {StyleField occupies 16-bits, but only first 8-bits are used}
		txMode: SInt16;                 { in Carbon use GetPortTextMode or TextMode}
		txSize: SInt16;                 { in Carbon use GetPortTextSize or TextSize}
		spExtra: Fixed;                { in Carbon use GetPortSpExtra or SpaceExtra}
		fgColor: SIGNEDLONG;                { not available in Carbon }
		bkColor: SIGNEDLONG;                { not available in Carbon}
		colrBit: SInt16;                { not available in Carbon}
		patStretch: SInt16;             { not available in Carbon}
		picSave: Handle;                { in Carbon use IsPortPictureBeingDefined}
		rgnSave: Handle;                { not available in Carbon}
		polySave: Handle;               { not available in Carbon}
		grafProcs: QDProcsPtr;              { not available in Carbon all GrafPorts are CGrafPorts}
	end;
type
	GrafPtr = GrafPortPtr;
{
 *  This set of definitions "belongs" in Windows.
 *  But, there is a circularity in the headers where Windows includes Controls and
 *  Controls includes Windows. To break the circle, the information
 *  needed by Controls is moved from Windows to Quickdraw.
 }
type
	WindowPtr = GrafPtr;
	DialogPtr = WindowPtr;
{$elsec}
type
	WindowPtr = ^SInt32; { an opaque type }
	WindowPtrPtr = ^WindowPtr;  { when a var xx:WindowPtr parameter can be nil, it is changed to xx: WindowPtrPtr }
	DialogPtr = ^SInt32; { an opaque type }
	DialogPtrPtr = ^DialogPtr;  { when a var xx:DialogPtr parameter can be nil, it is changed to xx: DialogPtrPtr }
	GrafPtr = ^SInt32; { an opaque type }
	GrafPtrPtr = ^GrafPtr;  { when a var xx:GrafPtr parameter can be nil, it is changed to xx: GrafPtrPtr }
{$endc} {not OPAQUE_TOOLBOX_STRUCTS}

type
	WindowRef = WindowPtr;
	WindowRefPtr = ^WindowRef;
{ DragConstraint constants to pass to DragGray,DragTheRgn, or ConstrainedDragRgn}
type
	DragConstraint = UInt16;
const
	kNoConstraint = 0;
	kVerticalConstraint = 1;
	kHorizontalConstraint = 2;


type
	DragGrayRgnProcPtr = procedure;
{
 *  Here ends the list of things that "belong" in Windows.
 }


type
	RGBColor = record
		red: UInt16;                    {magnitude of red component}
		green: UInt16;                  {magnitude of green component}
		blue: UInt16;                   {magnitude of blue component}
	end;
	RGBColorPtr = ^RGBColor;
type
	RGBColorHdl = ^RGBColorPtr;
	ColorSearchProcPtr = function( var rgb: RGBColor; var position: SIGNEDLONG ): Boolean;
	ColorComplementProcPtr = function( var rgb: RGBColor ): Boolean;
	DragGrayRgnUPP = DragGrayRgnProcPtr;
	ColorSearchUPP = ColorSearchProcPtr;
	ColorComplementUPP = ColorComplementProcPtr;
{
 *  NewDragGrayRgnUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDragGrayRgnUPP( userRoutine: DragGrayRgnProcPtr ): DragGrayRgnUPP; external name '_NewDragGrayRgnUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  NewColorSearchUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewColorSearchUPP( userRoutine: ColorSearchProcPtr ): ColorSearchUPP; external name '_NewColorSearchUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  NewColorComplementUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewColorComplementUPP( userRoutine: ColorComplementProcPtr ): ColorComplementUPP; external name '_NewColorComplementUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposeDragGrayRgnUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDragGrayRgnUPP( userUPP: DragGrayRgnUPP ); external name '_DisposeDragGrayRgnUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposeColorSearchUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeColorSearchUPP( userUPP: ColorSearchUPP ); external name '_DisposeColorSearchUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposeColorComplementUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeColorComplementUPP( userUPP: ColorComplementUPP ); external name '_DisposeColorComplementUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokeDragGrayRgnUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeDragGrayRgnUPP( userUPP: DragGrayRgnUPP ); external name '_InvokeDragGrayRgnUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokeColorSearchUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeColorSearchUPP( var rgb: RGBColor; var position: SIGNEDLONG; userUPP: ColorSearchUPP ): Boolean; external name '_InvokeColorSearchUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokeColorComplementUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeColorComplementUPP( var rgb: RGBColor; userUPP: ColorComplementUPP ): Boolean; external name '_InvokeColorComplementUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

type
	ColorSpec = record
		value: SInt16;                  {index or other value}
		rgb: RGBColor;                    {true color}
	end;
	ColorSpecPtr = ^ColorSpec;
	CSpecArray = array [0..0] of ColorSpec;
type
	ColorTablePtr = ^ColorTable;
	ColorTable = record
		ctSeed: SInt32;                 {unique identifier for table}
		ctFlags: SInt16;                {high bit: 0 = PixMap; 1 = device}
		ctSize: SInt16;                 {number of entries in CTTable}
		ctTable: CSpecArray;                {array [0..0] of ColorSpec}
	end;
type
	CTabPtr = ColorTablePtr;
	CTabHandle = ^CTabPtr;
	xColorSpec = record
		value: SInt16;                  {index or other value}
		rgb: RGBColor;                    {true color}
		xalpha: SInt16;
	end;
	xColorSpecPtr = ^xColorSpec;
	xCSpecArray							= array [0..0] of xColorSpec;
type
	MatchRecPtr = ^MatchRec;
	MatchRec = record
		red: UInt16;
		green: UInt16;
		blue: UInt16;
		matchData: SIGNEDLONG;
	end;
{ keep this around in case anybody relies on it }
const
	OLDPIXMAPSTRUCT = 0;
const
	NON_MAC_PIXEL_FORMATS = 1;

{ QuickTime 3.0 changed the original PixMap data structure, replacing the 
   old planeBytes field by "pixelFormat", and pmReserved by pmExt (a Handle to
   additional private info). On Mac OS X, most QuickTime-specific changes were
   merged into Quickdraw.
   32-bit constants for the new pixelFormat field are enumerated below.
   Please note that not all predefined pixelFormat constants are supported under
   all circumstances. For example, because of performance tuning on ppc systems,
   many QD drawing functions don't support little-endian pixelFormat destinations
   on ppc systems.
}
{ Values for PixMap.pixelFormat (BE = Big Endian, LE = Little Endian)}
const
{ The original pixel formats supported by 32-bit Color Quickdraw}
	k1MonochromePixelFormat = $00000001; { 1 bit indexed}
	k2IndexedPixelFormat = $00000002; { 2 bit indexed}
	k4IndexedPixelFormat = $00000004; { 4 bit indexed}
	k8IndexedPixelFormat = $00000008; { 8 bit indexed}
	k16BE555PixelFormat = $00000010; { 16 bit BE rgb 555 (Mac)}
	k24RGBPixelFormat = $00000018; { 24 bit rgb }
	k32ARGBPixelFormat = $00000020; { 32 bit argb    (Mac)}
                                        { QuickTime additions}
	k1IndexedGrayPixelFormat = $00000021; { 1 bit indexed gray}
	k2IndexedGrayPixelFormat = $00000022; { 2 bit indexed gray}
	k4IndexedGrayPixelFormat = $00000024; { 4 bit indexed gray}
	k8IndexedGrayPixelFormat = $00000028; { 8 bit indexed gray}
	k16LE555PixelFormat = FourCharCode('L555'); { 16 bit LE rgb 555 (PC)}
	k16LE5551PixelFormat = FourCharCode('5551'); { 16 bit LE rgb 5551}
	k16BE565PixelFormat = FourCharCode('B565'); { 16 bit BE rgb 565}
	k16LE565PixelFormat = FourCharCode('L565'); { 16 bit LE rgb 565}
	k24BGRPixelFormat = FourCharCode('24BG'); { 24 bit bgr }
	k32BGRAPixelFormat = FourCharCode('BGRA'); { 32 bit bgra    (Matrox)}
	k32ABGRPixelFormat = FourCharCode('ABGR'); { 32 bit abgr    }
	k32RGBAPixelFormat = FourCharCode('RGBA'); { 32 bit rgba    }
	kYUVSPixelFormat = FourCharCode('yuvs'); { YUV 4:2:2 byte ordering 16-unsigned = 'YUY2'}
	kYUVUPixelFormat = FourCharCode('yuvu'); { YUV 4:2:2 byte ordering 16-signed}
	kYVU9PixelFormat = FourCharCode('YVU9'); { YVU9 Planar    9}
	kYUV411PixelFormat = FourCharCode('Y411'); { YUV 4:1:1 Interleaved  16}
	kYVYU422PixelFormat = FourCharCode('YVYU'); { YVYU 4:2:2 byte ordering   16}
	kUYVY422PixelFormat = FourCharCode('UYVY'); { UYVY 4:2:2 byte ordering   16}
	kYUV211PixelFormat = FourCharCode('Y211'); { YUV 2:1:1 Packed   8}
	k2vuyPixelFormat = FourCharCode('2vuy'); { UYVY 4:2:2 byte ordering   16}


type
	PixMap = record
		baseAddr: Ptr;               { pointer to pixels }
		rowBytes: SInt16;               { offset to next line }
		bounds: Rect;                 { encloses bitmap }
		pmVersion: SInt16;              { pixMap version number }
		packType: SInt16;               { defines packing format }
		packSize: SInt32;               { length of pixel data }
		hRes: Fixed;                   { horiz. resolution (ppi) }
		vRes: Fixed;                   { vert. resolution (ppi) }
		pixelType: SInt16;              { defines pixel type }
		pixelSize: SInt16;              { # bits in pixel }
		cmpCount: SInt16;               { # components in pixel }
		cmpSize: SInt16;                { # bits per component }
		pixelFormat: OSType;            { fourCharCode representation }
		pmTable: CTabHandle;                { color map for this pixMap }
		pmExt: UnivPtr;                  { Handle to pixMap extension }
	end;
	PixMapPtr = ^PixMap;
type
	PixMapHandle = ^PixMapPtr;
	PixPat = record
		patType: SInt16;                {type of pattern}
		patMap: PixMapHandle;                 {the pattern's pixMap}
		patData: Handle;                {pixmap's data}
		patXData: Handle;               {expanded Pattern data}
		patXValid: SInt16;              {flags whether expanded Pattern valid}
		patXMap: Handle;                {Handle to expanded Pattern data}
		pat1Data: Pattern;               {old-Style pattern/RGB color}
	end;
	PixPatPtr = ^PixPat;
type
	PixPatHandle = ^PixPatPtr;
	CCrsr = record
		crsrType: SInt16;               {type of cursor}
		crsrMap: PixMapHandle;                {the cursor's pixmap}
		crsrData: Handle;               {cursor's data}
		crsrXData: Handle;              {expanded cursor data}
		crsrXValid: SInt16;             {depth of expanded data (0 if none)}
		crsrXHandle: Handle;            {future use}
		crsr1Data: Bits16;              {one-bit cursor}
		crsrMask: Bits16;               {cursor's mask}
		crsrHotSpot: Point;            {cursor's hotspot}
		crsrXTable: SInt32;             {private}
		crsrID: SInt32;                 {private}
	end;
	CCrsrPtr = ^CCrsr;
type
	CCrsrHandle = ^CCrsrPtr;
	GammaTbl = record
		gVersion: SInt16;               {gamma version number}
		gType: SInt16;                  {gamma data type}
		gFormulaSize: SInt16;           {Formula data size}
		gChanCnt: SInt16;               {number of channels of data}
		gDataCnt: SInt16;               {number of values/channel}
		gDataWidth: SInt16;             {bits/corrected value (data packed to next larger byte size)}
		gFormulaData: array [0..0] of SInt16;        {data for formulas followed by gamma values}
	end;
	GammaTblPtr = ^GammaTbl;
type
	GammaTblHandle = ^GammaTblPtr;
	ITab = record
		iTabSeed: SInt32;               {copy of CTSeed from source CTable}
		iTabRes: SInt16;                {bits/channel resolution of iTable}
		iTTable: array [0..0] of UInt8;             {byte colortable index values}
	end;
	ITabPtr = ^ITab;
type
	ITabHandle = ^ITabPtr;
	SProcRecPtr = ^SProcRec;
	SProcRec = record
		nxtSrch: Handle;                {SProcHndl Handle to next SProcRec}
		srchProc: ColorSearchUPP;               {search procedure proc ptr}
	end;
type
	SProcPtr = SProcRecPtr;
	SProcHndl = ^SProcPtr;
	CProcRecPtr = ^CProcRec;
	CProcRec = record
		nxtComp: Handle;                {CProcHndl Handle to next CProcRec}
		compProc: ColorComplementUPP;               {complement procedure proc ptr}
	end;
type
	CProcPtr = CProcRecPtr;
	CProcHndl = ^CProcPtr;
{ keep this around in case anybody relies on it }
const
	OLDGDEVICESTRUCT = 0;
type
	GDevicePtr = ^GDevice;
	GDPtr = ^GDevice;
	GDHandle = ^GDPtr;
	GDHandle_fix = GDHandle;  { used as field type when a record declaration contains a GDHandle field identifier }
	GDHandlePtr = ^GDHandle; { when a VAR xx: GDHandle parameter can be nil, it is changed to xx: GDHandlePtr }
	GDevice = record
		gdRefNum: SInt16;               {driver's unit number}
		gdID: SInt16;                   {client ID for search procs}
		gdType: SInt16;                 {fixed/CLUT/direct}
		gdITable: ITabHandle;               {Handle to inverse lookup table}
		gdResPref: SInt16;              {preferred resolution of GDITable}
		gdSearchProc: SProcHndl;           {search proc list head}
		gdCompProc: CProcHndl;             {complement proc list}
		gdFlags: SInt16;                {grafDevice flags word}
		gdPMap: PixMapHandle;                 {describing pixMap}
		gdRefCon: SInt32;               {reference value}
		gdNextGD: GDHandle;               {GDHandle Handle of next gDevice}
		gdRect: Rect;                 { device's bounds in global coordinates}
		gdMode: SInt32;                 {device's current mode}
		gdCCBytes: SInt16;              {depth of expanded cursor data}
		gdCCDepth: SInt16;              {depth of expanded cursor data}
		gdCCXData: Handle;              {Handle to cursor's expanded data}
		gdCCXMask: Handle;              {Handle to cursor's expanded mask}
		gdExt: Handle;                  {QuickTime 3.0 private info}
	end;

type
	GrafVarsPtr = ^GrafVars;
	GrafVars = record
		rgbOpColor: RGBColor;             {color for addPin  subPin and average}
		rgbHiliteColor: RGBColor;         {color for hiliting}
		pmFgColor: Handle;              {palette Handle for foreground color}
		pmFgIndex: SInt16;              {index value for foreground}
		pmBkColor: Handle;              {palette Handle for background color}
		pmBkIndex: SInt16;              {index value for background}
		pmFlags: SInt16;                {flags for Palette Manager}
	end;
type
	GVarPtr = GrafVarsPtr;
	GVarHandle = ^GVarPtr;

{$ifc not OPAQUE_TOOLBOX_STRUCTS}
type
	CGrafPort = struct CGrafPort;
	CGrafPtr = CGrafPortPtr;
{$elsec}
type
	CGrafPtr = GrafPtr;
{$endc} {not OPAQUE_TOOLBOX_STRUCTS}
	CGrafPtrPtr = ^CGrafPtr;

type
	QDPrinterStatusProcPtr = function( opcode: PrinterStatusOpcode; currentPort: CGrafPtr; printerStatus: UnivPtr ): OSStatus;
	QDPrinterStatusUPP = QDPrinterStatusProcPtr;

type
	CQDProcs = record
		textProc: QDTextUPP;
		lineProc: QDLineUPP;
		rectProc: QDRectUPP;
		rRectProc: QDRRectUPP;
		ovalProc: QDOvalUPP;
		arcProc: QDArcUPP;
		polyProc: QDPolyUPP;
		rgnProc: QDRgnUPP;
		bitsProc: QDBitsUPP;
		commentProc: QDCommentUPP;
		txMeasProc: QDTxMeasUPP;
		getPicProc: QDGetPicUPP;
		putPicProc: QDPutPicUPP;
		opcodeProc: QDOpcodeUPP;
		newProc1: UniversalProcPtr;               { this is the StdPix bottleneck -- see ImageCompression.h }
		glyphsProc: QDStdGlyphsUPP;             { unused on Mac OS X }
		printerStatusProc: QDPrinterStatusUPP;      { was newProc3;  now used to communicate status between Printing code and System imaging code }
		newProc4: UniversalProcPtr;
		newProc5: UniversalProcPtr;
		newProc6: UniversalProcPtr;
	end;
	CQDProcsPtr = ^CQDProcs;
{$ifc not OPAQUE_TOOLBOX_STRUCTS}
type
	CGrafPort = record
		device: SInt16;                 { not available in Carbon}
		portPixMap: PixMapHandle;             { in Carbon use GetPortPixMap}
		portVersion: SInt16;            { in Carbon use IsPortColor}
		grafVars: Handle;               { not available in Carbon}
		chExtra: SInt16;                { in Carbon use GetPortChExtra}
		pnLocHFrac: SInt16;             { in Carbon use Get/SetPortFracHPenLocation}
		portRect: Rect;               { in Carbon use Get/SetPortBounds}
		visRgn: RgnHandle;                 { in Carbon use Get/SetPortVisibleRegion}
		clipRgn: RgnHandle;                { in Carbon use Get/SetPortClipRegion}
		bkPixPat: PixPatHandle;               { in Carbon use GetPortBackPixPat or BackPixPat}
		rgbFgColor: RGBColor;             { in Carbon use GetPortForeColor or RGBForeColor}
		rgbBkColor: RGBColor;             { in Carbon use GetPortBackColor or RGBBackColor}
		pnLoc: Point;                  { in Carbon use GetPortPenLocation or MoveTo}
		pnSize: Point;                 { in Carbon use Get/SetPortPenSize}
		pnMode: SInt16;                 { in Carbon use Get/SetPortPenMode}
		pnPixPat: PixPatHandle;               { in Carbon use Get/SetPortPenPixPat}
		fillPixPat: PixPatHandle;             { in Carbon use GetPortFillPixPat}
		pnVis: SInt16;                  { in Carbon use GetPortPenVisibility or Show/HidePen}
		txFont: SInt16;                 { in Carbon use GetPortTextFont or TextFont}
		txFace: StyleField;                 { in Carbon use GetPortTextFace or TextFace}
                                              {StyleField occupies 16-bits, but only first 8-bits are used}
		txMode: SInt16;                 { in Carbon use GetPortTextMode or TextMode}
		txSize: SInt16;                 { in Carbon use GetPortTextSize or TextSize}
		spExtra: Fixed;                { in Carbon use GetPortSpExtra or SpaceExtra}
		fgColor: SInt32;                { not available in Carbon}
		bkColor: SInt32;                { not available in Carbon}
		colrBit: SInt16;                { not available in Carbon}
		patStretch: SInt16;             { not available in Carbon}
		picSave: Handle;                { in Carbon use IsPortPictureBeingDefined}
		rgnSave: Handle;                { in Carbon use IsPortRegionBeingDefined}
		polySave: Handle;               { in Carbon use IsPortPolyBeingDefined}
		grafProcs: CQDProcsPtr;              { in Carbon use Get/SetPortGrafProcs}
	end;

{$endc} {not OPAQUE_TOOLBOX_STRUCTS}

{$ifc OPAQUE_TOOLBOX_STRUCTS}
type
	CWindowPtr = WindowPtr;
{$elsec}
type
	CWindowPtr = CGrafPtr;
{$endc} {OPAQUE_TOOLBOX_STRUCTS}

type
	ReqListRecPtr = ^ReqListRec;
	ReqListRec = record
		reqLSize: SInt16;               {request list size}
		reqLData: array [0..0] of SInt16;            { request list data }
	end;
type
	OpenCPicParamsPtr = ^OpenCPicParams;
	OpenCPicParams = record
		srcRect: Rect;
		hRes: Fixed;
		vRes: Fixed;
		version: SInt16;
		reserved1: SInt16;
		reserved2: SInt32;
	end;
type
	DeviceLoopDrawingProcPtr = procedure( depth: SInt16; deviceFlags: SInt16; targetDevice: GDHandle; userData: SRefCon );
	DeviceLoopDrawingUPP = DeviceLoopDrawingProcPtr;
{
 *  NewQDPrinterStatusUPP()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   available as macro/inline
 }

{
 *  NewDeviceLoopDrawingUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDeviceLoopDrawingUPP( userRoutine: DeviceLoopDrawingProcPtr ): DeviceLoopDrawingUPP; external name '_NewDeviceLoopDrawingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposeQDPrinterStatusUPP()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   available as macro/inline
 }

{
 *  DisposeDeviceLoopDrawingUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDeviceLoopDrawingUPP( userUPP: DeviceLoopDrawingUPP ); external name '_DisposeDeviceLoopDrawingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokeQDPrinterStatusUPP()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   available as macro/inline
 }

{
 *  InvokeDeviceLoopDrawingUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeDeviceLoopDrawingUPP( depth: SInt16; deviceFlags: SInt16; targetDevice: GDHandle; userData: SRefCon; userUPP: DeviceLoopDrawingUPP ); external name '_InvokeDeviceLoopDrawingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{$ifc NOT OPAQUE_TOOLBOX_STRUCTS OR NOT TARGET_API_MAC_CARBON}
type
	QDGlobals = record
		privates: packed array [0..75] of char;
		randSeed: SInt32;               { in Carbon use GetQDGlobalsRandomSeed}
		screenBits: BitMap;             { in Carbon use GetQDGlobalsScreenBits}
		arrow: Cursor;                  { in Carbon use GetQDGlobalsArrow}
		dkGray: Pattern;                 { in Carbon use GetQDGlobalsDarkGray}
		ltGray: Pattern;                 { in Carbon use GetQDGlobalsLightGray}
		gray: Pattern;                   { in Carbon use GetQDGlobalsGray}
		black: Pattern;                  { in Carbon use GetQDGlobalsBlack}
		white: Pattern;                  { in Carbon use GetQDGlobalsWhite}
		thePort: GrafPtr;                { in Carbon use GetQDGlobalsThePort}
	end;
	QDGlobalsPtr = ^QDGlobals;
type
	QDGlobalsHdl = ^QDGlobalsPtr;
{ To be in sync with the C interface to QuickDraw globals, pascal code must now }
{ qualify the QuickDraw globals with “qd.” (e.g. InitGraf(@qd.thePort);  )       }
var qd: QDGlobals; external name '_qd'; (* attribute const *)
{$endc}

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
