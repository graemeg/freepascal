{
    This file is part of the Free Pascal run time library.

    A file in Amiga system run time library.
    Copyright (c) 1998-2003 by Nils Sjoholm
    member of the Amiga RTL development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{
    History:

    Added the defines use_amiga_smartlink and
    use_auto_openlib. Implemented openingcode
    for the library
    13 Jan 2003.

    Changed startcode for library.
    1 Feb 2003.

    Changed integer > smallint,
            cardinal > longword.
    09 Feb 2003.

    nils.sjoholm@mailbox.swipnet.se Nils Sjoholm
}

{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
   {$smartlink on}
{$endif use_amiga_smartlink}

unit bullet;

INTERFACE

uses exec, utility;

type

{ A GlyphEngine must be acquired via OpenEngine and is read-only }

 pGlyphEngine = ^tGlyphEngine;
 tGlyphEngine = record
    gle_Library  : pLibrary; { engine library }
    gle_Name     : STRPTR;     { library basename: e.g. "bullet" }
    { private library data follows... }
 end;

 FIXED = Longint;             { 32 bit signed w/ 16 bits of fraction }

 pGlyphMap = ^tGlyphMap;
 tGlyphMap = record
    glm_BMModulo,               { # of bytes in row: always multiple of 4 }
    glm_BMRows,                 { # of rows in bitmap }
    glm_BlackLeft,              { # of blank pixel columns at left }
    glm_BlackTop,               { # of blank rows at top }
    glm_BlackWidth,             { span of contiguous non-blank columns }
    glm_BlackHeight : WORD;     { span of contiguous non-blank rows }
    glm_XOrigin,                { distance from upper left corner of bitmap }
    glm_YOrigin     : FIXED;    {   to initial CP, in fractional pixels }
    glm_X0,                     { approximation of XOrigin in whole pixels }
    glm_Y0,                     { approximation of YOrigin in whole pixels }
    glm_X1,                     { approximation of XOrigin + Width }
    glm_Y1          : smallint;  { approximation of YOrigin + Width }
    glm_Width       : FIXED;    { character advance, as fraction of em width }
    glm_BitMap      : Pointer;  { actual glyph bitmap }
 end;

 pGlyphWidthEntry = ^tGlyphWidthEntry;
 tGlyphWidthEntry = record
    gwe_Node  : tMinNode;        { on list returned by OT_WidthList inquiry }
    gwe_Code  : WORD;           { entry's character code value }
    gwe_Width : FIXED;          { character advance, as fraction of em width }
 end;

const
{ Level 0 entries never appear in the .otag tag list, but appear in font
 * specifications }
     OT_Level0     = TAG_USER;
{ Level 1 entries are required to exist in the .otag tag list }
     OT_Level1     = (TAG_USER OR $1000);
{ Level 2 entries are optional typeface metric tags }
     OT_Level2     = (TAG_USER OR $2000);
{ Level 3 entries are required for some OT_Engines }
     OT_Level3     = (TAG_USER OR $3000);
{ Indirect entries are at (tag address + data offset) }
     OT_Indirect   = $8000;


{******************************************************************}
{ font specification and inquiry tags }

{ !  tags flagged with an exclaimation mark are valid for
 *    specification.
 *  ? tags flagged with a question mark are valid for inquiry
 *
 * fixed binary numbers are encoded as 16 bits of integer and
 * 16 bits of fraction.  Negative values are indicated by twos
 * complement of all 32 bits.
 }

{ !  OT_DeviceDPI specifies the target device dots per inch -- X DPI is
 *    in the high word, Y DPI in the low word. }
     OT_DeviceDPI  = (OT_Level0 OR $01);      { == TA_DeviceDPI }

{ !  OT_DotSize specifies the target device dot size as a percent of
 *    it's resolution-implied size -- X percent in high word, Y percent
 *    in low word. }
     OT_DotSize    = (OT_Level0 OR $02);

{ !  OT_PointHeight specifies the requested point height of a typeface,
 *    specifically, the height and nominal width of the em-square.
 *    The point referred to here is 1/72".  It is encoded as a fixed
 *    binary number. }
     OT_PointHeight = (OT_Level0 OR $08);

{ !  OT_SetFactor specifies the requested set width of a typeface.
 *    It distorts the width of the em-square from that specified by
 *    OT_PointHeight.  To compensate for a device with different
 *    horizontal and vertical resolutions, OT_DeviceDPI should be used
 *    instead.  For a normal aspect ratio, set to 1.0 (encoded as
 *    $00010000).  This is the default value. }
     OT_SetFactor   = (OT_Level0 OR $09);

{ !  OT_Shear... specifies the Sine and Cosine of the vertical stroke
 *    angle, as two fixed point binary fractions.  Both must be specified:
 *    first the Sine and then the Cosine.  Setting the sine component
 *    changes the Shear to an undefined value, setting the cosine
 *    component completes the Shear change to the new composite value.
 *    For no shear, set to 0.0, 1.0 (encoded as $00000000, $00010000).
 *    This is the default value. }
     OT_ShearSin   = (OT_Level0 OR $0a);
     OT_ShearCos   = (OT_Level0 OR $0b);

{ !  OT_Rotate... specifies the Sine and Cosine of the baselin rotation
 *    angle, as two fixed point binary fractions.  Both must be specified:
 *    first the Sine and then the Cosine.  Setting the sine component
 *    changes the Shear to an undefined value, setting the cosine
 *    component completes the Shear change to the new composite value.
 *    For no shear, set to 0.0, 1.0 (encoded as $00000000, $00010000).
 *    This is the default value. }
     OT_RotateSin  = (OT_Level0 OR $0c);
     OT_RotateCos  = (OT_Level0 OR $0d);

{ !  OT_Embolden... specifies values to algorithimically embolden -- or,
 *    when negative, lighten -- the glyph.  It is encoded as a fixed point
 *    binary fraction of the em-square.  The X and Y components can be
 *    changed indendently.  For normal characters, set to 0.0, 0.0
 *    (encoded as $00000000, $00000000).  This is the default value. }
     OT_EmboldenX  = (OT_Level0 OR $0e);
     OT_EmboldenY  = (OT_Level0 OR $0f);

{ !  OT_PointSize is an old method of specifying the point size,
 *    encoded as (points * 16). }
     OT_PointSize  = (OT_Level0 OR $10);

{ !  OT_GlyphCode specifies the glyph (character) code to use with
 *    subsequent operations.  For example, this is the code for an
 *    OT_Glyph inquiry }
     OT_GlyphCode  = (OT_Level0 OR $11);

{ !  OT_GlyphCode2 specifies the second glyph code.  For example,
 *    this is the right glyph of the two glyphs of an OT_KernPair
 *    inquiry }
     OT_GlyphCode2 = (OT_Level0 OR $12);

{ !  OT_GlyphWidth specifies a specific width for a glyph.
 *    It sets a specific escapement (advance) width for subsequent
 *    glyphs.  It is encoded as a fixed binary fraction of the em-square.
 *    To revert to using the font-defined escapement for each glyph, set
 *    to 0.0 (encoded as $00000000).  This is the default value. }
     OT_GlyphWidth = (OT_Level0 OR $13);

{ !  OT_OTagPath and
 * !  OT_OTagList specify the selected typeface.  Both must be specified:
 *    first the Path and then the List.  Setting the path name changes
 *    changes the typeface to an undefined value, providing the List
 *    completes the typeface selection to the new typeface.  OTagPath
 *    is the null terminated full file path of the .otag file associated
 *    with the typeface.  OTagList is a memory copy of the processed
 *    contents of that .otag file (i.e. with indirections resolved).
 *    There are no default values for the typeface. }
     OT_OTagPath   = (OT_Level0 OR OT_Indirect OR $14);
     OT_OTagList   = (OT_Level0 OR OT_Indirect OR $15);

{  ? OT_GlyphMap supplies a read-only struct GlyphMap pointer that
 *    describes a bitmap for a glyph with the current attributes. }
     OT_GlyphMap   = (OT_Level0 OR OT_Indirect OR $20);

{  ? OT_WidthList supplies a read-only struct MinList of struct
 *    GlyphWidthEntry nodes for glyphs that are defined from GlyphCode
 *    to GlyphCode2, inclusive.  The widths are represented as fixed
 *    binary fractions of the em-square, ignoring any effect of
 *    SetFactor or GlyphWidth.  A width would need to be converted to
 *    a distance along the baseline in device units by the
 *    application. }
     OT_WidthList  = (OT_Level0 OR OT_Indirect OR $21);

{  ? OT_...KernPair supplies the kern adjustment to be added to the
 *    current position after placement of the GlyphCode glyph and
 *    before placement of the GlyphCode2 glyph.  Text kern pairs are
 *    for rendering body text.  Display kern pairs are generally
 *    tighter values for display (e.g. headline) purposes.  The
 *    adjustment is represented as a fixed binary fraction of the
 *    em-square, ignoring any effect of SetFactor.  This number would
 *    need to be converted to a distance along the baseline in device
 *    units by the application. }
     OT_TextKernPair = (OT_Level0 OR OT_Indirect OR $22);
     OT_DesignKernPair = (OT_Level0 OR OT_Indirect OR $23);

{  ? OT_Underlined is an unsigned word which is used to request
 *    algorithimic underlining for the engine when rendering the glyph.
 *    Bullet.library currently does not support this tag, though it
 *    may be used by other engines in the future.  The default for
 *    any engine which supports this tag must be OTUL_None.  Engines which
 *    do not support this tag should return an appropriate OTERR value.
 *
 *    As of V39, diskfont.library will request underlining if specified
 *    in the TextAttr, or TTextAttr passed to OpenDiskFont().  Diskfont
 *    will first request Broken underlining (like the Text() function
 *    does when SetSoftStyle() is used), and then Solid underlining if
 *    the engine returns an error.  If the engine returns an error for
 *    both, then diskfont.library attempts to find, or create the best
 *    non-underlined font that it can. }
     OT_UnderLined        =  (OT_Level0 OR $24);

     OTUL_None            =  0;
     OTUL_Solid           =  1;
     OTUL_Broken          =  2;
     OTUL_DoubleSolid     =  3;
     OUTL_DoubleBroken    =  4;

{  ? OT_StrikeThrough is a boolean which is used to request
 *    algorithimic strike through when rendering the glyph.
 *    Bullet.library currently does not support this tag, though it
 *    may be used by other engines in the future.  The default for
 *    any engined which supports this tag must be FALSE.  Engines which
 *    do not support this tag should return an appropriate OTERR value. }
     OT_StrikeThrough     =  (OT_Level0 OR $25);


{******************************************************************}
{ .otag tags }

{ suffix for files in FONTS: that contain these tags }
     OTSUFFIX     : PChar =  '.otag';

{ OT_FileIdent both identifies this file and verifies its size.
 * It is required to be the first tag in the file. }
     OT_FileIdent =  (OT_Level1 OR $01);

{ OT_Engine specifies the font engine this file is designed to use }
     OT_Engine    =  (OT_Level1 OR OT_Indirect OR $02);
     OTE_Bullet   =  'bullet';

{ OT_Family is the family name of this typeface }
     OT_Family    =  (OT_Level1 OR OT_Indirect OR $03);

{ The name of this typeface is implicit in the name of the .otag file }
{ OT_BName is used to find the bold variant of this typeface }
     OT_BName     =  (OT_Level2 OR OT_Indirect OR $05);
{ OT_IName is used to find the italic variant of this typeface }
     OT_IName     =  (OT_Level2 OR OT_Indirect OR $06);
{ OT_BIName is used to find the bold italic variant of this typeface }
     OT_BIName    =  (OT_Level2 OR OT_Indirect OR $07);

{ OT_SymSet is used to select the symbol set that has the OT_YSizeFactor
 * described here.  Other symbol sets might have different extremes }
     OT_SymbolSet =  (OT_Level1 OR $10);

{ OT_YSizeFactor is a ratio to assist in calculating the Point height
 * to BlackHeight relationship -- high word: Point height term, low
 * word: Black height term -- pointSize = ysize*<high>/<low> }
     OT_YSizeFactor = (OT_Level1 OR $11);

{ OT_SpaceWidth specifies the width of the space character relative
 * to the character height }
     OT_SpaceWidth = (OT_Level2 OR $12);

{ OT_IsFixed is a boolean indicating that all the characters in the
 * typeface are intended to have the same character advance }
     OT_IsFixed    = (OT_Level2 OR $13);

{ OT_SerifFlag is a boolean indicating if the character has serifs }
     OT_SerifFlag  = (OT_Level1 OR $14);

{ OT_StemWeight is an unsigned byte indicating the weight of the character }
     OT_StemWeight = (OT_Level1 OR $15);

     OTS_UltraThin  =   8;     {   0- 15 }
     OTS_ExtraThin  =  24;     {  16- 31 }
     OTS_Thin       =  40;     {  32- 47 }
     OTS_ExtraLight =  56;     {  48- 63 }
     OTS_Light      =  72;     {  64- 79 }
     OTS_DemiLight  =  88;     {  80- 95 }
     OTS_SemiLight  = 104;     {  96-111 }
     OTS_Book       = 120;     { 112-127 }
     OTS_Medium     = 136;     { 128-143 }
     OTS_SemiBold   = 152;     { 144-159 }
     OTS_DemiBold   = 168;     { 160-175 }
     OTS_Bold       = 184;     { 176-191 }
     OTS_ExtraBold  = 200;     { 192-207 }
     OTS_Black      = 216;     { 208-223 }
     OTS_ExtraBlack = 232;     { 224-239 }
     OTS_UltraBlack = 248;     { 240-255 }

{ OT_SlantStyle is an unsigned byte indicating the font posture }
     OT_SlantStyle  = (OT_Level1 OR $16);
     OTS_Upright    = 0;
     OTS_Italic     = 1;       { Oblique, Slanted, etc. }
     OTS_LeftItalic = 2;       { Reverse Slant }

{ OT_HorizStyle is an unsigned byte indicating the appearance width }
     OT_HorizStyle  = (OT_Level1 OR $17);
     OTH_UltraCompressed  =   16;     {   0- 31 }
     OTH_ExtraCompressed  =   48;     {  32- 63 }
     OTH_Compressed       =   80;     {  64- 95 }
     OTH_Condensed        =  112;     {  96-127 }
     OTH_Normal           =  144;     { 128-159 }
     OTH_SemiExpanded     =  176;     { 160-191 }
     OTH_Expanded         =  208;     { 192-223 }
     OTH_ExtraExpanded    =  240;     { 224-255 }

{ OT_SpaceFactor specifies the width of the space character relative
 * to the character height }
     OT_SpaceFactor = (OT_Level2 OR $18);

{ OT_InhibitAlgoStyle indicates which ta_Style bits, if any, should
 * be ignored even if the font does not already have that quality.
 * For example, if FSF_BOLD is set and the typeface is not bold but
 * the user specifies bold, the application or diskfont library is
 * not to use OT_Embolden to achieve a bold result. }
     OT_InhibitAlgoStyle = (OT_Level2 OR $19);

{ OT_AvailSizes is an indirect pointer to sorted UWORDs, 0th is count }
     OT_AvailSizes  = (OT_Level1 OR OT_Indirect OR $20);
     OT_MAXAVAILSIZES    =   20;      { no more than 20 sizes allowed }

{ OT_SpecCount is the count number of parameters specified here }
     OT_SpecCount   = (OT_Level1 OR $100);

{ Specs can be created as appropriate for the engine by ORing in the
 * parameter number (1 is first, 2 is second, ... up to 15th) }
     OT_Spec        = (OT_Level1 OR $100);
{ OT_Spec1 is the (first) parameter to the font engine to select
 * this particular typeface }
     OT_Spec1       = (OT_Level1 OR $101) ;


const
{ PRELIMINARY }
    OTERR_Failure        =  -1;      { catch-all for error }
    OTERR_Success        =  0 ;      { no error }
    OTERR_BadTag         =  1 ;      { inappropriate tag for function }
    OTERR_UnknownTag     =  2 ;      { unknown tag for function }
    OTERR_BadData        =  3 ;      { catch-all for bad tag data }
    OTERR_NoMemory       =  4 ;      { insufficient memory for operation }
    OTERR_NoFace         =  5 ;      { no typeface currently specified }
    OTERR_BadFace        =  6 ;      { typeface specification problem }
    OTERR_NoGlyph        =  7 ;      { no glyph specified }
    OTERR_BadGlyph       =  8 ;      { bad glyph code or glyph range }
    OTERR_NoShear        =  9 ;      { shear only partially specified }
    OTERR_NoRotate       =  10;      { rotate only partially specified }
    OTERR_TooSmall       =  11;      { typeface metrics yield tiny glyphs }
    OTERR_UnknownGlyph   =  12;      { glyph not known by engine }

VAR BulletBase : pLibrary;

const
    BULLETNAME : PChar = 'bullet.library';


PROCEDURE CloseEngine(glyphEngine : pGlyphEngine);
FUNCTION ObtainInfoA(glyphEngine : pGlyphEngine; tagList : pTagItem) : ULONG;
FUNCTION OpenEngine : pGlyphEngine;
FUNCTION ReleaseInfoA(glyphEngine : pGlyphEngine; tagList : pTagItem) : ULONG;
FUNCTION SetInfoA(glyphEngine : pGlyphEngine; tagList : pTagItem) : ULONG;

{Here we read how to compile this unit}
{You can remove this include and use a define instead}
{$I useautoopenlib.inc}
{$ifdef use_init_openlib}
procedure InitBULLETLibrary;
{$endif use_init_openlib}

{This is a variable that knows how the unit is compiled}
var
    BULLETIsCompiledHow : longint;

IMPLEMENTATION

{
 If you don't use array of const then just remove tagsarray
}
uses
{$ifndef dont_use_openlib}
msgbox;
{$endif dont_use_openlib}

PROCEDURE CloseEngine(glyphEngine : pGlyphEngine);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L glyphEngine,A0
    MOVEA.L BulletBase,A6
    JSR -036(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION ObtainInfoA(glyphEngine : pGlyphEngine; tagList : pTagItem) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L glyphEngine,A0
    MOVEA.L tagList,A1
    MOVEA.L BulletBase,A6
    JSR -048(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION OpenEngine : pGlyphEngine;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L BulletBase,A6
    JSR -030(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION ReleaseInfoA(glyphEngine : pGlyphEngine; tagList : pTagItem) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L glyphEngine,A0
    MOVEA.L tagList,A1
    MOVEA.L BulletBase,A6
    JSR -054(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION SetInfoA(glyphEngine : pGlyphEngine; tagList : pTagItem) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L glyphEngine,A0
    MOVEA.L tagList,A1
    MOVEA.L BulletBase,A6
    JSR -042(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;


const
    { Change VERSION and LIBVERSION to proper values }

    VERSION : string[2] = '0';
    LIBVERSION : longword = 0;

{$ifdef use_init_openlib}
  {$Info Compiling initopening of bullet.library}
  {$Info don't forget to use InitBULLETLibrary in the beginning of your program}

var
    bullet_exit : Pointer;

procedure ClosebulletLibrary;
begin
    ExitProc := bullet_exit;
    if BulletBase <> nil then begin
        CloseLibrary(BulletBase);
        BulletBase := nil;
    end;
end;

procedure InitBULLETLibrary;
begin
    BulletBase := nil;
    BulletBase := OpenLibrary(BULLETNAME,LIBVERSION);
    if BulletBase <> nil then begin
        bullet_exit := ExitProc;
        ExitProc := @ClosebulletLibrary;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open bullet.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;
end;

begin
    BULLETIsCompiledHow := 2;
{$endif use_init_openlib}

{$ifdef use_auto_openlib}
  {$Info Compiling autoopening of bullet.library}

var
    bullet_exit : Pointer;

procedure ClosebulletLibrary;
begin
    ExitProc := bullet_exit;
    if BulletBase <> nil then begin
        CloseLibrary(BulletBase);
        BulletBase := nil;
    end;
end;

begin
    BulletBase := nil;
    BulletBase := OpenLibrary(BULLETNAME,LIBVERSION);
    if BulletBase <> nil then begin
        bullet_exit := ExitProc;
        ExitProc := @ClosebulletLibrary;
        BULLETIsCompiledHow := 1;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open bullet.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;

{$endif use_auto_openlib}

{$ifdef dont_use_openlib}
begin
    BULLETIsCompiledHow := 3;
   {$Warning No autoopening of bullet.library compiled}
   {$Warning Make sure you open bullet.library yourself}
{$endif dont_use_openlib}


END. (* UNIT BULLET *)



