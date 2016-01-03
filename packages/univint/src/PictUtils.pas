{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       QD/PictUtils.h
 
     Contains:   Picture Utilities Interfaces.
 
     Version:    Quickdraw-262~1
 
     Copyright:  © 1990-2008 by Apple Computer, Inc., all rights reserved
=======
=======
>>>>>>> origin/fixes_2_2
     File:       PictUtils.p
=======
     File:       QD/PictUtils.h
>>>>>>> origin/fixes_2.4
 
     Contains:   Picture Utilities Interfaces.
 
     Version:    Quickdraw-262~1
 
<<<<<<< HEAD
     Copyright:  © 1990-2002 by Apple Computer, Inc., all rights reserved
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  © 1990-2008 by Apple Computer, Inc., all rights reserved
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

unit PictUtils;
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
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
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
	{$setc TARGET_CPU_ARM64 := FALSE}
=======
>>>>>>> origin/cpstrnew
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
=======
>>>>>>> graemeg/cpstrnew
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
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> graemeg/cpstrnew
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
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
uses MacTypes,QuickdrawTypes,Palettes;
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
=======
uses MacTypes,Quickdraw,Palettes;

>>>>>>> graemeg/fixes_2_2
=======
uses MacTypes,Quickdraw,Palettes;

>>>>>>> origin/fixes_2_2
=======
uses MacTypes,QuickdrawTypes,Palettes;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}
>>>>>>> origin/fixes_2.4

{$ALIGN MAC68K}

{ verbs for the GetPictInfo, GetPixMapInfo, and NewPictInfo calls }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
const
	returnColorTable = $0001;
	returnPalette = $0002;
	recordComments = $0004;
	recordFontInfo = $0008;
	suppressBlackAndWhite = $0010;

const
{ color pick methods }
	systemMethod = 0;    { system color pick method }
	popularMethod = 1;    { method that chooses the most popular set of colors }
	medianMethod = 2;     { method that chooses a good average mix of colors }

const
{ color bank types }
	ColorBankIsCustom = -1;
	ColorBankIsExactAnd555 = 0;
	ColorBankIs555 = 1;

type
	PictInfoID = SIGNEDLONG;
	CommentSpec = record
		count: SInt16;                  { number of occurrances of this comment ID }
		ID: SInt16;                     { ID for the comment in the picture }
	end;
	CommentSpecPtr = ^CommentSpec;
type
	CommentSpecHandle = ^CommentSpecPtr;
	FontSpec = record
		pictFontID: SInt16;             { ID of the font in the picture }
		sysFontID: SInt16;              { ID of the same font in the current system file }
		size: array [0..3] of SInt32;                { bit array of all the sizes found (1..127) (bit 0 means > 127) }
		style: SInt16;                  { combined style of all occurrances of the font }
		nameOffset: SIGNEDLONG;             { offset into the fontNamesHdl handle for the font’s name }
	end;
	FontSpecPtr = ^FontSpec;
type
	FontSpecHandle = ^FontSpecPtr;
	PictInfo = record
		version: SInt16;                { this is always zero, for now }
		uniqueColors: SIGNEDLONG;           { the number of actual colors in the picture(s)/pixmap(s) }
		thePalette: PaletteHandle;             { handle to the palette information }
		theColorTable: CTabHandle;          { handle to the color table }
		hRes: Fixed;                   { maximum horizontal resolution for all the pixmaps }
		vRes: Fixed;                   { maximum vertical resolution for all the pixmaps }
		depth: SInt16;                  { maximum depth for all the pixmaps (in the picture) }
		sourceRect: Rect;             { the picture frame rectangle (this contains the entire picture) }
		textCount: SIGNEDLONG;              { total number of text strings in the picture }
		lineCount: SIGNEDLONG;              { total number of lines in the picture }
		rectCount: SIGNEDLONG;              { total number of rectangles in the picture }
		rRectCount: SIGNEDLONG;             { total number of round rectangles in the picture }
		ovalCount: SIGNEDLONG;              { total number of ovals in the picture }
		arcCount: SIGNEDLONG;               { total number of arcs in the picture }
		polyCount: SIGNEDLONG;              { total number of polygons in the picture }
		regionCount: SIGNEDLONG;            { total number of regions in the picture }
		bitMapCount: SIGNEDLONG;            { total number of bitmaps in the picture }
		pixMapCount: SIGNEDLONG;            { total number of pixmaps in the picture }
		commentCount: SIGNEDLONG;           { total number of comments in the picture }
		uniqueComments: SIGNEDLONG;         { the number of unique comments in the picture }
		commentHandle: CommentSpecHandle;          { handle to all the comment information }
		uniqueFonts: SIGNEDLONG;            { the number of unique fonts in the picture }
		fontHandle: FontSpecHandle;             { handle to the FontSpec information }
		fontNamesHandle: Handle;        { handle to the font names }
		reserved1: SIGNEDLONG;
		reserved2: SIGNEDLONG;
	end;
	PictInfoPtr = ^PictInfo;
type
	PictInfoHandle = ^PictInfoPtr;
	InitPickMethodProcPtr = function( colorsRequested: SInt16; var dataRef: UInt32; var colorBankType: SInt16 ): OSErr;
	RecordColorsProcPtr = function( dataRef: UInt32; var colorsArray: RGBColor; colorCount: SInt32; var uniqueColors: SInt32 ): OSErr;
	CalcColorTableProcPtr = function( dataRef: UInt32; colorsRequested: SInt16; colorBankPtr: UnivPtr; resultPtr: CSpecArray ): OSErr;
	DisposeColorPickMethodProcPtr = function( dataRef: UInt32 ): OSErr;
	InitPickMethodUPP = InitPickMethodProcPtr;
	RecordColorsUPP = RecordColorsProcPtr;
	CalcColorTableUPP = CalcColorTableProcPtr;
	DisposeColorPickMethodUPP = DisposeColorPickMethodProcPtr;
{
 *  NewInitPickMethodUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewInitPickMethodUPP( userRoutine: InitPickMethodProcPtr ): InitPickMethodUPP; external name '_NewInitPickMethodUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2

=======
>>>>>>> origin/fixes_2.4
const
	returnColorTable = $0001;
	returnPalette = $0002;
	recordComments = $0004;
	recordFontInfo = $0008;
	suppressBlackAndWhite = $0010;

const
{ color pick methods }
	systemMethod = 0;    { system color pick method }
	popularMethod = 1;    { method that chooses the most popular set of colors }
	medianMethod = 2;     { method that chooses a good average mix of colors }

const
{ color bank types }
	ColorBankIsCustom = -1;
	ColorBankIsExactAnd555 = 0;
	ColorBankIs555 = 1;

type
	PictInfoID = SIGNEDLONG;
	CommentSpec = record
		count: SInt16;                  { number of occurrances of this comment ID }
		ID: SInt16;                     { ID for the comment in the picture }
	end;
	CommentSpecPtr = ^CommentSpec;
type
	CommentSpecHandle = ^CommentSpecPtr;
	FontSpec = record
		pictFontID: SInt16;             { ID of the font in the picture }
		sysFontID: SInt16;              { ID of the same font in the current system file }
		size: array [0..3] of SInt32;                { bit array of all the sizes found (1..127) (bit 0 means > 127) }
		style: SInt16;                  { combined style of all occurrances of the font }
		nameOffset: SIGNEDLONG;             { offset into the fontNamesHdl handle for the font’s name }
	end;
	FontSpecPtr = ^FontSpec;
type
	FontSpecHandle = ^FontSpecPtr;
	PictInfo = record
		version: SInt16;                { this is always zero, for now }
		uniqueColors: SIGNEDLONG;           { the number of actual colors in the picture(s)/pixmap(s) }
		thePalette: PaletteHandle;             { handle to the palette information }
		theColorTable: CTabHandle;          { handle to the color table }
		hRes: Fixed;                   { maximum horizontal resolution for all the pixmaps }
		vRes: Fixed;                   { maximum vertical resolution for all the pixmaps }
		depth: SInt16;                  { maximum depth for all the pixmaps (in the picture) }
		sourceRect: Rect;             { the picture frame rectangle (this contains the entire picture) }
		textCount: SIGNEDLONG;              { total number of text strings in the picture }
		lineCount: SIGNEDLONG;              { total number of lines in the picture }
		rectCount: SIGNEDLONG;              { total number of rectangles in the picture }
		rRectCount: SIGNEDLONG;             { total number of round rectangles in the picture }
		ovalCount: SIGNEDLONG;              { total number of ovals in the picture }
		arcCount: SIGNEDLONG;               { total number of arcs in the picture }
		polyCount: SIGNEDLONG;              { total number of polygons in the picture }
		regionCount: SIGNEDLONG;            { total number of regions in the picture }
		bitMapCount: SIGNEDLONG;            { total number of bitmaps in the picture }
		pixMapCount: SIGNEDLONG;            { total number of pixmaps in the picture }
		commentCount: SIGNEDLONG;           { total number of comments in the picture }
		uniqueComments: SIGNEDLONG;         { the number of unique comments in the picture }
		commentHandle: CommentSpecHandle;          { handle to all the comment information }
		uniqueFonts: SIGNEDLONG;            { the number of unique fonts in the picture }
		fontHandle: FontSpecHandle;             { handle to the FontSpec information }
		fontNamesHandle: Handle;        { handle to the font names }
		reserved1: SIGNEDLONG;
		reserved2: SIGNEDLONG;
	end;
	PictInfoPtr = ^PictInfo;
type
	PictInfoHandle = ^PictInfoPtr;
	InitPickMethodProcPtr = function( colorsRequested: SInt16; var dataRef: UInt32; var colorBankType: SInt16 ): OSErr;
	RecordColorsProcPtr = function( dataRef: UInt32; var colorsArray: RGBColor; colorCount: SInt32; var uniqueColors: SInt32 ): OSErr;
	CalcColorTableProcPtr = function( dataRef: UInt32; colorsRequested: SInt16; colorBankPtr: UnivPtr; resultPtr: CSpecArray ): OSErr;
	DisposeColorPickMethodProcPtr = function( dataRef: UInt32 ): OSErr;
	InitPickMethodUPP = InitPickMethodProcPtr;
	RecordColorsUPP = RecordColorsProcPtr;
	CalcColorTableUPP = CalcColorTableProcPtr;
	DisposeColorPickMethodUPP = DisposeColorPickMethodProcPtr;
{
 *  NewInitPickMethodUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewInitPickMethodUPP( userRoutine: InitPickMethodProcPtr ): InitPickMethodUPP; external name '_NewInitPickMethodUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
	PictInfoHandle						= ^PictInfoPtr;
{$ifc TYPED_FUNCTION_POINTERS}
	InitPickMethodProcPtr = function(colorsRequested: SInt16; var dataRef: UInt32; var colorBankType: SInt16): OSErr;
{$elsec}
	InitPickMethodProcPtr = ProcPtr;
{$endc}

{$ifc TYPED_FUNCTION_POINTERS}
	RecordColorsProcPtr = function(dataRef: UInt32; var colorsArray: RGBColor; colorCount: SInt32; var uniqueColors: SInt32): OSErr;
{$elsec}
	RecordColorsProcPtr = ProcPtr;
{$endc}

{$ifc TYPED_FUNCTION_POINTERS}
	CalcColorTableProcPtr = function(dataRef: UInt32; colorsRequested: SInt16; colorBankPtr: UnivPtr; var resultPtr: CSpecArray): OSErr;
{$elsec}
	CalcColorTableProcPtr = ProcPtr;
{$endc}

{$ifc TYPED_FUNCTION_POINTERS}
	DisposeColorPickMethodProcPtr = function(dataRef: UInt32): OSErr;
{$elsec}
	DisposeColorPickMethodProcPtr = ProcPtr;
{$endc}

{$ifc OPAQUE_UPP_TYPES}
	InitPickMethodUPP = ^SInt32; { an opaque UPP }
{$elsec}
	InitPickMethodUPP = UniversalProcPtr;
{$endc}	
{$ifc OPAQUE_UPP_TYPES}
	RecordColorsUPP = ^SInt32; { an opaque UPP }
{$elsec}
	RecordColorsUPP = UniversalProcPtr;
{$endc}	
{$ifc OPAQUE_UPP_TYPES}
	CalcColorTableUPP = ^SInt32; { an opaque UPP }
{$elsec}
	CalcColorTableUPP = UniversalProcPtr;
{$endc}	
{$ifc OPAQUE_UPP_TYPES}
	DisposeColorPickMethodUPP = ^SInt32; { an opaque UPP }
{$elsec}
	DisposeColorPickMethodUPP = UniversalProcPtr;
{$endc}	

const
	uppInitPickMethodProcInfo = $00000FA0;
	uppRecordColorsProcInfo = $00003FE0;
	uppCalcColorTableProcInfo = $00003EE0;
	uppDisposeColorPickMethodProcInfo = $000000E0;
	{
	 *  NewInitPickMethodUPP()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   available as macro/inline
	 *    CarbonLib:        in CarbonLib 1.0 and later
	 *    Mac OS X:         in version 10.0 and later
	 	}
function NewInitPickMethodUPP(userRoutine: InitPickMethodProcPtr): InitPickMethodUPP; external name '_NewInitPickMethodUPP'; { old name was NewInitPickMethodProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{
 *  NewRecordColorsUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewRecordColorsUPP( userRoutine: RecordColorsProcPtr ): RecordColorsUPP; external name '_NewRecordColorsUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function NewRecordColorsUPP(userRoutine: RecordColorsProcPtr): RecordColorsUPP; external name '_NewRecordColorsUPP'; { old name was NewRecordColorsProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function NewRecordColorsUPP( userRoutine: RecordColorsProcPtr ): RecordColorsUPP; external name '_NewRecordColorsUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4
{
 *  NewCalcColorTableUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewCalcColorTableUPP( userRoutine: CalcColorTableProcPtr ): CalcColorTableUPP; external name '_NewCalcColorTableUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function NewCalcColorTableUPP(userRoutine: CalcColorTableProcPtr): CalcColorTableUPP; external name '_NewCalcColorTableUPP'; { old name was NewCalcColorTableProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function NewCalcColorTableUPP( userRoutine: CalcColorTableProcPtr ): CalcColorTableUPP; external name '_NewCalcColorTableUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4
{
 *  NewDisposeColorPickMethodUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDisposeColorPickMethodUPP( userRoutine: DisposeColorPickMethodProcPtr ): DisposeColorPickMethodUPP; external name '_NewDisposeColorPickMethodUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function NewDisposeColorPickMethodUPP(userRoutine: DisposeColorPickMethodProcPtr): DisposeColorPickMethodUPP; external name '_NewDisposeColorPickMethodUPP'; { old name was NewDisposeColorPickMethodProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function NewDisposeColorPickMethodUPP( userRoutine: DisposeColorPickMethodProcPtr ): DisposeColorPickMethodUPP; external name '_NewDisposeColorPickMethodUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4
{
 *  DisposeInitPickMethodUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeInitPickMethodUPP( userUPP: InitPickMethodUPP ); external name '_DisposeInitPickMethodUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure DisposeInitPickMethodUPP(userUPP: InitPickMethodUPP); external name '_DisposeInitPickMethodUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeInitPickMethodUPP( userUPP: InitPickMethodUPP ); external name '_DisposeInitPickMethodUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4
{
 *  DisposeRecordColorsUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeRecordColorsUPP( userUPP: RecordColorsUPP ); external name '_DisposeRecordColorsUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure DisposeRecordColorsUPP(userUPP: RecordColorsUPP); external name '_DisposeRecordColorsUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeRecordColorsUPP( userUPP: RecordColorsUPP ); external name '_DisposeRecordColorsUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4
{
 *  DisposeCalcColorTableUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeCalcColorTableUPP( userUPP: CalcColorTableUPP ); external name '_DisposeCalcColorTableUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure DisposeCalcColorTableUPP(userUPP: CalcColorTableUPP); external name '_DisposeCalcColorTableUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeCalcColorTableUPP( userUPP: CalcColorTableUPP ); external name '_DisposeCalcColorTableUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4
{
 *  DisposeDisposeColorPickMethodUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDisposeColorPickMethodUPP( userUPP: DisposeColorPickMethodUPP ); external name '_DisposeDisposeColorPickMethodUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure DisposeDisposeColorPickMethodUPP(userUPP: DisposeColorPickMethodUPP); external name '_DisposeDisposeColorPickMethodUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeDisposeColorPickMethodUPP( userUPP: DisposeColorPickMethodUPP ); external name '_DisposeDisposeColorPickMethodUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4
{
 *  InvokeInitPickMethodUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeInitPickMethodUPP( colorsRequested: SInt16; var dataRef: UInt32; var colorBankType: SInt16; userUPP: InitPickMethodUPP ): OSErr; external name '_InvokeInitPickMethodUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function InvokeInitPickMethodUPP(colorsRequested: SInt16; var dataRef: UInt32; var colorBankType: SInt16; userRoutine: InitPickMethodUPP): OSErr; external name '_InvokeInitPickMethodUPP'; { old name was CallInitPickMethodProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function InvokeInitPickMethodUPP( colorsRequested: SInt16; var dataRef: UInt32; var colorBankType: SInt16; userUPP: InitPickMethodUPP ): OSErr; external name '_InvokeInitPickMethodUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4
{
 *  InvokeRecordColorsUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeRecordColorsUPP( dataRef: UInt32; var colorsArray: RGBColor; colorCount: SInt32; var uniqueColors: SInt32; userUPP: RecordColorsUPP ): OSErr; external name '_InvokeRecordColorsUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function InvokeRecordColorsUPP(dataRef: UInt32; var colorsArray: RGBColor; colorCount: SInt32; var uniqueColors: SInt32; userRoutine: RecordColorsUPP): OSErr; external name '_InvokeRecordColorsUPP'; { old name was CallRecordColorsProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function InvokeRecordColorsUPP( dataRef: UInt32; var colorsArray: RGBColor; colorCount: SInt32; var uniqueColors: SInt32; userUPP: RecordColorsUPP ): OSErr; external name '_InvokeRecordColorsUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4
{
 *  InvokeCalcColorTableUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeCalcColorTableUPP( dataRef: UInt32; colorsRequested: SInt16; colorBankPtr: UnivPtr; resultPtr: CSpecArray; userUPP: CalcColorTableUPP ): OSErr; external name '_InvokeCalcColorTableUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function InvokeCalcColorTableUPP(dataRef: UInt32; colorsRequested: SInt16; colorBankPtr: UnivPtr; var resultPtr: CSpecArray; userRoutine: CalcColorTableUPP): OSErr; external name '_InvokeCalcColorTableUPP'; { old name was CallCalcColorTableProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function InvokeCalcColorTableUPP( dataRef: UInt32; colorsRequested: SInt16; colorBankPtr: UnivPtr; resultPtr: CSpecArray; userUPP: CalcColorTableUPP ): OSErr; external name '_InvokeCalcColorTableUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/fixes_2.4
{
 *  InvokeDisposeColorPickMethodUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeDisposeColorPickMethodUPP( dataRef: UInt32; userUPP: DisposeColorPickMethodUPP ): OSErr; external name '_InvokeDisposeColorPickMethodUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{$ifc not TARGET_CPU_64}
{
 *  GetPictInfo()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetPictInfo( thePictHandle: PicHandle; var thePictInfo: PictInfo; verb: SInt16; colorsRequested: SInt16; colorPickMethod: SInt16; version: SInt16 ): OSErr; external name '_GetPictInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  GetPixMapInfo()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetPixMapInfo( thePixMapHandle: PixMapHandle; var thePictInfo: PictInfo; verb: SInt16; colorsRequested: SInt16; colorPickMethod: SInt16; version: SInt16 ): OSErr; external name '_GetPixMapInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  NewPictInfo()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function NewPictInfo( var thePictInfoID: PictInfoID; verb: SInt16; colorsRequested: SInt16; colorPickMethod: SInt16; version: SInt16 ): OSErr; external name '_NewPictInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  RecordPictInfo()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function RecordPictInfo( thePictInfoID: PictInfoID; thePictHandle: PicHandle ): OSErr; external name '_RecordPictInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  RecordPixMapInfo()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function RecordPixMapInfo( thePictInfoID: PictInfoID; thePixMapHandle: PixMapHandle ): OSErr; external name '_RecordPixMapInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  RetrievePictInfo()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function RetrievePictInfo( thePictInfoID: PictInfoID; var thePictInfo: PictInfo; colorsRequested: SInt16 ): OSErr; external name '_RetrievePictInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DisposePictInfo()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DisposePictInfo( thePictInfoID: PictInfoID ): OSErr; external name '_DisposePictInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


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
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeDisposeColorPickMethodUPP( dataRef: UInt32; userUPP: DisposeColorPickMethodUPP ): OSErr; external name '_InvokeDisposeColorPickMethodUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{$ifc not TARGET_CPU_64}
{
 *  GetPictInfo()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetPictInfo( thePictHandle: PicHandle; var thePictInfo: PictInfo; verb: SInt16; colorsRequested: SInt16; colorPickMethod: SInt16; version: SInt16 ): OSErr; external name '_GetPictInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  GetPixMapInfo()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetPixMapInfo( thePixMapHandle: PixMapHandle; var thePictInfo: PictInfo; verb: SInt16; colorsRequested: SInt16; colorPickMethod: SInt16; version: SInt16 ): OSErr; external name '_GetPixMapInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  NewPictInfo()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function NewPictInfo( var thePictInfoID: PictInfoID; verb: SInt16; colorsRequested: SInt16; colorPickMethod: SInt16; version: SInt16 ): OSErr; external name '_NewPictInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  RecordPictInfo()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function RecordPictInfo( thePictInfoID: PictInfoID; thePictHandle: PicHandle ): OSErr; external name '_RecordPictInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  RecordPixMapInfo()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function RecordPixMapInfo( thePictInfoID: PictInfoID; thePixMapHandle: PixMapHandle ): OSErr; external name '_RecordPixMapInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  RetrievePictInfo()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function RetrievePictInfo( thePictInfoID: PictInfoID; var thePictInfo: PictInfo; colorsRequested: SInt16 ): OSErr; external name '_RetrievePictInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  DisposePictInfo()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function DisposePictInfo( thePictInfoID: PictInfoID ): OSErr; external name '_DisposePictInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


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
