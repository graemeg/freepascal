{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Michael Van Canneyt, member of the
    Free Pascal development team

    All major and minor RTF class definitions.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{ ---------------------------------------------------------------------
  Twentieths of a point (twips) per inch (Many RTF measurements
  are in twips per inch (tpi) units).  Assumes 72 points/inch.
  ---------------------------------------------------------------------}

Const
 rtfTpi         = 1440;
 rtfBufSiz      = 255; { buffer size}

{ ---------------------------------------------------------------------
        Tokens are associated with up to three classification numbers:

        Class number: Broadest (least detailed) breakdown.  For programs
                that only care about gross token distinctions.
        Major/minor numbers: Within their class, tokens have a major
                number, and may also have a minor number to further
                distinquish tokens with the same major number.

        *** Class, major and minor token numbers are all >= 0 ***

        Tokens that can't be classified are put in the "unknown" class.
        For such, the major and minor numbers are meaningless, although
        rtfTextBuf may be of interest then.

        Text tokens are a single character, and the major number indicates
        the character value (note: can be non-ascii, i.e., greater than 127).
        There is no minor number.

        Control symbols may have a parameter value, which will be found in
        rtfParam.  If no parameter was given, rtfParam = rtfNoParam.

        RTFGetToken() return value is the class number, but it sets all the
        global token vars.

        rtfEOF is a fake token used by the reader; the writer never sees
        it (except in the token reader hook, if it installs one).

        Information pertaining to last token read by RTFToken.  The
        text is exactly as it occurs in the input file, e.g.,
        will be found in rtfTextBuf as, even though it means .

        These variables are also set when styles are reprocessed.
 ----------------------------------------------------------------------}
        rtfNoParam = (-1000000);

{       Token classes (zero-based and sequential) }
        rtfUnknown      = 0;
        rtfGroup        = 1;
        rtfText         = 2;
        rtfControl      = 3;
        rtfEOF          = 4;
        rtfMaxClass     = 5     { highest class + 1 };

{       Group class major numbers }
        rtfBeginGroup   = 0;
        rtfEndGroup     = 1;
{       Control class major and minor numbers.}
        rtfVersion      = 0;

        rtfDefFont      = 1;

        rtfCharSet      = 2;
                rtfAnsiCharSet          = 0;
                rtfMacCharSet           = 1;
                rtfPcCharSet            = 2;
                rtfPcaCharSet           = 3;
{       destination minor numbers should be zero-based, sequential }
        rtfDestination  = 3;
                rtfPict                 = 0;
                rtfNeXTGraphic          = 1;
                rtfFootnote             = 2;
                rtfHeader               = 3;
                rtfHeaderLeft           = 4;
                rtfHeaderRight          = 5;
                rtfHeaderFirst          = 6;
                rtfFooter               = 7;
                rtfFooterLeft           = 8;
                rtfFooterRight          = 9;
                rtfFooterFirst          = 10;
                rtfFNSep                = 11;
                rtfFNContSep            = 12;
                rtfFNContNotice         = 13;
                rtfInfo                 = 14;
                rtfStyleSheet           = 15;
                rtfFontTbl              = 16;
                rtfColorTbl             = 17;
                rtfField                = 18;
                rtfFieldInst            = 19;
                rtfFieldResult          = 20;
                rtfIndex                = 21;
                rtfIndexBold            = 22;
                rtfIndexItalic          = 23;
                rtfIndexText            = 24;
                rtfIndexRange           = 25;
                rtfTOC                  = 26;
                rtfBookmarkStart        = 27;
                rtfBookmarkEnd          = 28;
                rtfITitle               = 29;
                rtfISubject             = 30;
                rtfIAuthor              = 31;
                rtfIOperator            = 32;
                rtfIKeywords            = 33;
                rtfIComment             = 34;
                rtfIVersion             = 35;
                rtfIDoccomm             = 36;
                rtfICompany             = 37;  (* Added *)
                rtfMaxDestination       = 38    { highest dest + 1 };

        rtfFontFamily   = 4;
                rtfFFNil                = 0;
                rtfFFRoman              = 1;
                rtfFFSwiss              = 2;
                rtfFFModern             = 3;
                rtfFFScript             = 4;
                rtfFFDecor              = 5;
                rtfFFTech               = 6;

        rtfColorName    = 5;
                rtfRed                  = 0;
                rtfGreen                = 1;
                rtfBlue                 = 2;

        rtfSpecialChar = 6;
                rtfCurHeadPage          = 0;
                rtfCurFNote             = 1;
                rtfCurHeadPict          = 2     { valid? };
                rtfCurHeadDate          = 3;
                rtfCurHeadTime          = 4;
                rtfFormula              = 5;
                rtfNoBrkSpace           = 6;
                rtfNoReqHyphen          = 7;
                rtfNoBrkHyphen          = 8;
                rtfPage                 = 9;
                rtfLine                 = 10;
                rtfPar                  = 11;
                rtfSect                 = 12;
                rtfTab                  = 13;
                rtfCell                 = 14;
                rtfRow                  = 15;
                rtfCurAnnot             = 16;
                rtfAnnotation           = 17;
                rtfAnnotID              = 18;
                rtfCurAnnotRef          = 19;
                rtfFNoteSep             = 20;
                rtfFNoteCont            = 21;
                rtfColumn               = 22;
                rtfOptDest              = 23;
                rtfIIntVersion          = 24;
                rtfICreateTime          = 25;
                rtfIRevisionTime        = 26;
                rtfIPrintTime           = 27;
                rtfIBackupTime          = 28;
                rtfIEditTime            = 29;
                rtfIYear                = 30;
                rtfIMonth               = 31;
                rtfIDay                 = 32;
                rtfIHour                = 33;
                rtfIMinute              = 34;
                rtfINPages              = 35;
                rtfINWords              = 36;
                rtfINChars              = 37;
                rtfIIntID               = 38;
<<<<<<< HEAD
<<<<<<< HEAD
                rtfUnicodeID            = 39;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

        rtfStyleAttr    = 7;
                rtfBasedOn              = 0;
                rtfNext                 = 1;

        rtfDocAttr      = 8;
                rtfPaperWidth           = 0;
                rtfPaperHeight          = 1;
                rtfLeftMargin           = 2;
                rtfRightMargin          = 3;
                rtfTopMargin            = 4;
                rtfBottomMargin         = 5;
                rtfFacingPage           = 6;
                rtfGutterWid            = 7;
                rtfDefTab               = 8;
                rtfWidowCtrl            = 9;
                rtfHyphHotZone          = 10;
                rtfFNoteEndSect         = 11;
                rtfFNoteEndDoc          = 12;
                rtfFNoteText            = 13;
                rtfFNoteBottom          = 14;
                rtfFNoteStart           = 15;
                rtfFNoteRestart         = 16;
                rtfPageStart            = 17;
                rtfLineStart            = 18;
                rtfLandscape            = 19;
                rtfFracWidth            = 20;
                rtfNextFile             = 21;
                rtfTemplate             = 22;
                rtfMakeBackup           = 23;
                rtfRTFDefault           = 24;
                rtfRevisions            = 25;
                rtfMirrorMargin         = 26;
                rtfRevDisplay           = 27;
                rtfRevBar               = 28;

        rtfSectAttr     = 9;
                rtfSectDef              = 0;
                rtfNoBreak              = 1;
                rtfColBreak             = 2;
                rtfPageBreak            = 3;
                rtfEvenBreak            = 4;
                rtfOddBreak             = 5;
                rtfPageStarts           = 6;
                rtfPageCont             = 7;
                rtfPageRestart          = 8;
                rtfPageDecimal          = 9;
                rtfPageURoman           = 10;
                rtfPageLRoman           = 11;
                rtfPageULetter          = 12;
                rtfPageLLetter          = 13;
                rtfPageNumLeft          = 14;
                rtfPageNumTop           = 15;
                rtfHeaderY              = 16;
                rtfFooterY              = 17;
                rtfLineModulus          = 18;
                rtfLineDist             = 19;
                rtfLineStarts           = 20;
                rtfLineRestart          = 21;
                rtfLineRestartPg        = 22;
                rtfLineCont             = 23;
                rtfTopVAlign            = 24;
                rtfBottomVAlign         = 25;
                rtfCenterVAlign         = 26;
                rtfJustVAlign           = 27;
                rtfColumns              = 28;
                rtfColumnSpace          = 29;
                rtfColumnLine           = 30;
                rtfENoteHere            = 31;
                rtfTitleSpecial         = 32;

        rtfTblAttr      = 10;
                rtfCellBordBottom       = 0;
                rtfCellBordTop          = 1;
                rtfCellBordLeft         = 2;
                rtfCellBordRight        = 3;
                rtfRowDef               = 4;
                rtfRowLeft              = 5;
                rtfRowRight             = 6;
                rtfRowCenter            = 7;
                rtfRowGapH              = 8;
                rtfRowHt                = 9;
                rtfRowLeftEdge          = 10;
                rtfRowHeader            = 11;  (* Added *)
                rtfCellPos              = 12;
                rtfCellBackColor        = 13;  (* Added *)
                rtfMergeRngFirst        = 14;
                rtfMergePrevious        = 15;

        rtfParAttr      = 11;
                rtfParDef               = 0;
                rtfStyleNum             = 1;
                rtfQuadLeft             = 2;
                rtfQuadRight            = 3;
                rtfQuadJust             = 4;
                rtfQuadCenter           = 5;
                rtfFirstIndent          = 6;
                rtfLeftIndent           = 7;
                rtfRightIndent          = 8;
                rtfSpaceBefore          = 9;
                rtfSpaceAfter           = 10;
                rtfSpaceBetween         = 11;
                rtfInTable              = 12;
                rtfKeep                 = 13;
                rtfKeepNext             = 14;
                rtfSideBySide           = 15;
                rtfPBBefore             = 16;
                rtfNoLineNum            = 17;
                rtfTabPos               = 18;
                rtfTabRight             = 19;
                rtfTabCenter            = 20;
                rtfTabDecimal           = 21;
                rtfTabBar               = 22;
                rtfBorderTop            = 23;
                rtfBorderBottom         = 24;
                rtfBorderLeft           = 25;
                rtfBorderRight          = 26;
                rtfBorderBox            = 27;
                rtfBorderBar            = 28;
                rtfBorderBetween        = 29;
                rtfBorderSingle         = 30;
                rtfBorderThick          = 31;
                rtfBorderShadow         = 32;
                rtfBorderDouble         = 33;
                rtfBorderDot            = 34;
                rtfBorderHair           = 35;
                rtfBorderSpace          = 36;
                rtfLeaderDot            = 37;
                rtfLeaderHyphen         = 38;
                rtfLeaderUnder          = 39;
                rtfLeaderThick          = 40;

        rtfCharAttr     = 12;
                rtfPlain                = 0;
                rtfBold                 = 1;
                rtfItalic               = 2;
                rtfStrikeThru           = 3;
                rtfOutline              = 4;
                rtfShadow               = 5;
                rtfSmallCaps            = 6;
                rtfAllCaps              = 7;
                rtfInvisible            = 8;
                rtfFontNum              = 9;
                rtfFontSize             = 10;
                rtfExpand               = 11;
                rtfUnderline            = 12;
                rtfWUnderline           = 13;
                rtfDUnderline           = 14;
                rtfDbUnderline          = 15;
                rtfNoUnderline          = 16;
                rtfSuperScript          = 17;
                rtfSubScript            = 18;
                rtfRevised              = 19;
                rtfForeColor            = 20;
                rtfBackColor            = 21;
                rtfGray                 = 22;

        rtfPictAttr     = 13;
                rtfMacQD                = 0;
                rtfWinMetafile          = 1;
                rtfWinBitmap            = 2;
                rtfPicWid               = 3;
                rtfPicHt                = 4;
                rtfPicGoalWid           = 5;
                rtfPicGoalHt            = 6;
                rtfPicScaleX            = 7;
                rtfPicScaleY            = 8;
                rtfPicScaled            = 9;
                rtfPicCropTop           = 10;
                rtfPicCropBottom        = 11;
                rtfPicCropLeft          = 12;
                rtfPicCropRight         = 13;
                rtfPixelBits            = 14;
                rtfBitmapPlanes         = 15;
                rtfBitmapWid            = 16;
                rtfPicBinary            = 17;

        rtfNeXTGrAttr   = 14;
                rtfNeXTGWidth           = 0;
                rtfNeXTGHeight          = 1;

        rtfFieldAttr    = 15;
                rtfFieldDirty           = 0;
                rtfFieldEdited          = 1;
                rtfFieldLocked          = 2;
                rtfFieldPrivate         = 3;

                rtfTOCAttr      = 16;
                rtfTOCType              = 0;
                rtfTOCLevel             = 1;

        rtfPosAttr      = 17;
                rtfPosX                 = 0;
                rtfPosXCenter           = 1;
                rtfPosXInside           = 2;
                rtfPosXLeft             = 3;
                rtfPosXOutSide          = 4;
                rtfPosXRight            = 5;
                rtfPosY                 = 6;
                rtfPosYInline           = 7;
                rtfPosYTop              = 8;
                rtfPosYCenter           = 9;
                rtfPosYBottom           = 10;
                rtfAbsWid               = 11;
                rtfTextDist             = 12;
                rtfRPosMargV            = 13;
                rtfRPosPageV            = 14;
                rtfRPosMargH            = 15;
                rtfRPosPageH            = 16;
                rtfRPosColH             = 17;

        rtfBasedOnNone  = 222;  { "no based-on style" }


Type

{ ---------------------------------------------------------------------
     Callback Types
  ---------------------------------------------------------------------}

TRTFFunc = Procedure of object;
TRTFFuncPtr = procedure of object;

{ ---------------------------------------------------------------------
    RTF font, color and style structures.  Used for font table,
    color table, and stylesheet processing.
  ---------------------------------------------------------------------}

PRTFFONT = ^TRTFFONT;
TRTFFont = Record
        rtfFName    : string;           { font name }
        rtfFNum     : integer;          { font number }
        rtfFFamily  : integer;          { font family }
        rtfNextFont : PRTFFONT;         { next font in list }
end;


{ ----------------------------------------------------------------------
        Color values are -1 if the default color for the the color
        number should be used.  The default color is writer-dependent.
  ----------------------------------------------------------------------}

PRTFColor = ^TRTFColor;
TRTFColor = Record
        rtfCNum : integer;      { color number }
        rtfCRed : INteger;      { red value }
        rtfCGreen : INteger;    { green value }
        rtfCBlue : integer;     { blue value }
        rtfNextColor : PRTFColor;       { next color in list }
end;

PRTFStyleElt = ^TRTFStyleElt;
TRTFStyleElt = record
        rtfSEClass,                     { token class }
        rtfSEMajor,                     { token major number }
        rtfSEMinor,                     { token minor number }
        rtfSEParam : Integer;           { control symbol parameter }
        rtfSEText : String;             { text of symbol }
        rtfNextSE : PRTFStyleElt;       { next element in style }
end;

PRTFSTyle = ^TRTFStyle;
TRTFStyle = record
        rtfSName : string;              { style name }
        rtfSNum,                        { style number }
        rtfSBasedOn,                    { style this one's based on }
        rtfSNextPar : integer;          { style next paragraph style }
        rtfSSEList : PRTFStyleElt;      { list of style words }
        rtfExpanding : Integer;         { non-zero = being expanded }
        rtfNextStyle : PRTFStyle;       { next style in style list }
end;

{ ---------------------------------------------------------------------
       Control symbol lookup routines
  ---------------------------------------------------------------------}


Type
  TRTFKey = record
    rtfKMajor : Integer;        { major number }
    rtfKMinor : Integer;        { minor number }
    rtfKStr   : string[20];     { symbol name }
    rtfKHash  : Integer;        { symbol name hash value }
    End;

{ ---------------------------------------------------------------------
    A minor number of -1 means the token has no minor number
   (all valid minor numbers are >= 0).
  ---------------------------------------------------------------------}

<<<<<<< HEAD
<<<<<<< HEAD
Const rtfKey : Array [0..286] of TRTFKey =
=======
Const rtfKey : Array [0..284] of TRTFKey =
>>>>>>> graemeg/fixes_2_2
=======
Const rtfKey : Array [0..284] of TRTFKey =
>>>>>>> origin/fixes_2_2
(
( rtfKMajor: RTFSPECIALCHAR; rtfKMinor : rtfCURHEADPICT; rtfKStr  : 'chpict'; rtfKhash :        0),
( rtfKMajor: rtfSpecialChar; rtfKMinor:         rtfCurHeadDate; rtfKstr : 'chdate'; rtfkHash : 0),
( rtfKMajor: rtfSpecialChar; rtfKMinor:         rtfCurHeadTime; rtfKstr : 'chtime'; rtfkHash : 0),
( rtfKMajor: rtfSpecialChar; rtfKMinor:         rtfCurHeadPage; rtfKstr : 'chpgn'; rtfkHash : 0),
( rtfKMajor: rtfSpecialChar; rtfKMinor:         rtfCurFNote; rtfKstr : 'chftn'; rtfkHash : 0),
( rtfKMajor: rtfSpecialChar; rtfKMinor:         rtfCurAnnotRef; rtfKstr : 'chatn'; rtfkHash : 0),
( rtfKMajor: rtfSpecialChar; rtfKMinor:         rtfAnnotation; rtfKstr : 'annotation'; rtfkHash : 0),  (* Moved to here *)
( rtfKMajor: rtfSpecialChar; rtfKMinor:         rtfAnnotID; rtfKstr : 'atnid'; rtfkHash : 0),  (* Moved to here *)
( rtfKMajor: rtfSpecialChar; rtfKMinor:         rtfFNoteSep; rtfKstr : 'chftnsep'; rtfkHash : 0),
( rtfKMajor: rtfSpecialChar; rtfKMinor:         rtfFNoteCont; rtfKstr : 'chftnsepc'; rtfkHash : 0),
( rtfKMajor: rtfSpecialChar; rtfKMinor:         rtfFormula; rtfKstr : '|'; rtfkHash : 0),
( rtfKMajor: rtfSpecialChar; rtfKMinor:         rtfNoBrkSpace; rtfKstr : '~'; rtfkHash : 0),
( rtfKMajor: rtfSpecialChar; rtfKMinor:         rtfNoReqHyphen; rtfKstr : '-'; rtfkHash : 0),
( rtfKMajor: rtfSpecialChar; rtfKMinor:         rtfNoBrkHyphen; rtfKstr : '_'; rtfkHash : 0),
( rtfKMajor: rtfSpecialChar; rtfKMinor:         rtfCell; rtfKstr : 'cell'; rtfkHash : 0),
( rtfKMajor: rtfSpecialChar; rtfKMinor:         rtfRow; rtfKstr : 'row'; rtfkHash : 0),
( rtfKMajor: rtfSpecialChar; rtfKMinor:         rtfPar; rtfKstr : 'par'; rtfkHash : 0),
( rtfKMajor: rtfSpecialChar; rtfKMinor:         rtfPar; rtfKstr : #10; rtfkHash : 0),
( rtfKMajor: rtfSpecialChar; rtfKMinor:         rtfPar; rtfKstr : #13; rtfkHash : 0),
( rtfKMajor: rtfSpecialChar; rtfKMinor:         rtfSect; rtfKstr : 'sect'; rtfkHash : 0),
( rtfKMajor: rtfSpecialChar; rtfKMinor:         rtfPage; rtfKstr : 'page'; rtfkHash : 0),
( rtfKMajor: rtfSpecialChar; rtfKMinor:         rtfColumn; rtfKstr : 'column'; rtfkHash : 0),
( rtfKMajor: rtfSpecialChar; rtfKMinor:         rtfLine; rtfKstr : 'line'; rtfkHash : 0),
( rtfKMajor: rtfSpecialChar; rtfKMinor:         rtfTab; rtfKstr : 'tab'; rtfkHash : 0),
( rtfKMajor: rtfSpecialChar; rtfKMinor:         rtfOptDest; rtfKstr : '*'; rtfkHash : 0),
( rtfKMajor: rtfSpecialChar; rtfKMinor:         rtfIIntVersion; rtfKstr : 'vern'; rtfkHash : 0),
( rtfKMajor: rtfSpecialChar; rtfKMinor:         rtfICreateTime; rtfKstr : 'creatim'; rtfkHash : 0),
( rtfKMajor: rtfSpecialChar; rtfKMinor:         rtfIRevisionTime; rtfKstr : 'revtim'; rtfkHash : 0),
( rtfKMajor: rtfSpecialChar; rtfKMinor:         rtfIPrintTime; rtfKstr : 'printim'; rtfkHash : 0),
( rtfKMajor: rtfSpecialChar; rtfKMinor:         rtfIBackupTime; rtfKstr : 'buptim'; rtfkHash : 0),
( rtfKMajor: rtfSpecialChar; rtfKMinor:         rtfIEditTime; rtfKstr : 'edmins'; rtfkHash : 0),
( rtfKMajor: rtfSpecialChar; rtfKMinor:         rtfIYear; rtfKstr : 'yr'; rtfkHash : 0),
( rtfKMajor: rtfSpecialChar; rtfKMinor:         rtfIMonth; rtfKstr : 'mo'; rtfkHash : 0),
( rtfKMajor: rtfSpecialChar; rtfKMinor:         rtfIDay; rtfKstr : 'dy'; rtfkHash : 0),
( rtfKMajor: rtfSpecialChar; rtfKMinor:         rtfIHour; rtfKstr : 'hr'; rtfkHash : 0),
( rtfKMajor: rtfSpecialChar; rtfKMinor:         rtfIMinute; rtfKstr : 'min'; rtfkHash : 0),
( rtfKMajor: rtfSpecialChar; rtfKMinor:         rtfINPages; rtfKstr : 'nofpages'; rtfkHash : 0),
( rtfKMajor: rtfSpecialChar; rtfKMinor:         rtfINWords; rtfKstr : 'nofwords'; rtfkHash : 0),
( rtfKMajor: rtfSpecialChar; rtfKMinor:         rtfINChars; rtfKstr : 'nofchars'; rtfkHash : 0),
( rtfKMajor: rtfSpecialChar; rtfKMinor:         rtfIIntID; rtfKstr : 'id'; rtfkHash : 0),
<<<<<<< HEAD
<<<<<<< HEAD
( rtfKMajor: rtfSpecialChar; rtfKMinor:         rtfUnicodeID; rtfKstr : 'u'; rtfkHash : 0),
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

( rtfKMajor: rtfCharAttr; rtfKMinor:    rtfPlain; rtfKstr : 'plain'; rtfkHash : 0),
( rtfKMajor: rtfCharAttr; rtfKMinor:    rtfBold; rtfKstr : 'b'; rtfkHash : 0),
( rtfKMajor: rtfCharAttr; rtfKMinor:    rtfItalic; rtfKstr : 'i'; rtfkHash : 0),
( rtfKMajor: rtfCharAttr; rtfKMinor:    rtfStrikeThru; rtfKstr : 'strike'; rtfkHash : 0),
( rtfKMajor: rtfCharAttr; rtfKMinor:    rtfOutline; rtfKstr : 'outl'; rtfkHash : 0),
( rtfKMajor: rtfCharAttr; rtfKMinor:    rtfShadow; rtfKstr : 'shad'; rtfkHash : 0),
( rtfKMajor: rtfCharAttr; rtfKMinor:    rtfSmallCaps; rtfKstr : 'scaps'; rtfkHash : 0),
( rtfKMajor: rtfCharAttr; rtfKMinor:    rtfAllCaps; rtfKstr : 'caps'; rtfkHash : 0),
( rtfKMajor: rtfCharAttr; rtfKMinor:    rtfInvisible; rtfKstr : 'v'; rtfkHash : 0),
( rtfKMajor: rtfCharAttr; rtfKMinor:    rtfFontNum; rtfKstr : 'f'; rtfkHash : 0),
( rtfKMajor: rtfCharAttr; rtfKMinor:    rtfFontSize; rtfKstr : 'fs'; rtfkHash : 0),
( rtfKMajor: rtfCharAttr; rtfKMinor:    rtfExpand; rtfKstr : 'expnd'; rtfkHash : 0),
( rtfKMajor: rtfCharAttr; rtfKMinor:    rtfUnderline; rtfKstr : 'ul'; rtfkHash : 0),
( rtfKMajor: rtfCharAttr; rtfKMinor:    rtfWUnderline; rtfKstr : 'ulw'; rtfkHash : 0),
( rtfKMajor: rtfCharAttr; rtfKMinor:    rtfDUnderline; rtfKstr : 'uld'; rtfkHash : 0),
( rtfKMajor: rtfCharAttr; rtfKMinor:    rtfDbUnderline; rtfKstr : 'uldb'; rtfkHash : 0),
( rtfKMajor: rtfCharAttr; rtfKMinor:    rtfNoUnderline; rtfKstr : 'ulnone'; rtfkHash : 0),
( rtfKMajor: rtfCharAttr; rtfKMinor:    rtfSuperScript; rtfKstr : 'up'; rtfkHash : 0),
<<<<<<< HEAD
<<<<<<< HEAD
( rtfKMajor: rtfCharAttr; rtfKMinor:    rtfSuperScript; rtfKstr : 'super'; rtfkHash : 0),
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
( rtfKMajor: rtfCharAttr; rtfKMinor:    rtfSubScript; rtfKstr : 'dn'; rtfkHash : 0),
( rtfKMajor: rtfCharAttr; rtfKMinor:    rtfRevised; rtfKstr : 'revised'; rtfkHash : 0),
( rtfKMajor: rtfCharAttr; rtfKMinor:    rtfForeColor; rtfKstr : 'cf'; rtfkHash : 0),
( rtfKMajor: rtfCharAttr; rtfKMinor:    rtfBackColor; rtfKstr : 'cb'; rtfkHash : 0),
( rtfKMajor: rtfCharAttr; rtfKMinor:    rtfGray; rtfKstr : 'gray'; rtfkHash : 0),

( rtfKMajor: rtfParAttr; rtfKMinor:     rtfParDef; rtfKstr : 'pard'; rtfkHash : 0),
( rtfKMajor: rtfParAttr; rtfKMinor:     rtfStyleNum; rtfKstr : 's'; rtfkHash : 0),
( rtfKMajor: rtfParAttr; rtfKMinor:     rtfQuadLeft; rtfKstr : 'ql'; rtfkHash : 0),
( rtfKMajor: rtfParAttr; rtfKMinor:     rtfQuadRight; rtfKstr : 'qr'; rtfkHash : 0),
( rtfKMajor: rtfParAttr; rtfKMinor:     rtfQuadJust; rtfKstr : 'qj'; rtfkHash : 0),
( rtfKMajor: rtfParAttr; rtfKMinor:     rtfQuadCenter; rtfKstr : 'qc'; rtfkHash : 0),
( rtfKMajor: rtfParAttr; rtfKMinor:     rtfFirstIndent; rtfKstr : 'fi'; rtfkHash : 0),
( rtfKMajor: rtfParAttr; rtfKMinor:     rtfLeftIndent; rtfKstr : 'li'; rtfkHash : 0),
( rtfKMajor: rtfParAttr; rtfKMinor:     rtfRightIndent; rtfKstr : 'ri'; rtfkHash : 0),
( rtfKMajor: rtfParAttr; rtfKMinor:     rtfSpaceBefore; rtfKstr : 'sb'; rtfkHash : 0),
( rtfKMajor: rtfParAttr; rtfKMinor:     rtfSpaceAfter; rtfKstr : 'sa'; rtfkHash : 0),
( rtfKMajor: rtfParAttr; rtfKMinor:     rtfSpaceBetween; rtfKstr : 'sl'; rtfkHash : 0),
( rtfKMajor: rtfParAttr; rtfKMinor:     rtfInTable; rtfKstr : 'intbl'; rtfkHash : 0),
( rtfKMajor: rtfParAttr; rtfKMinor:     rtfKeep; rtfKstr : 'keep'; rtfkHash : 0),
( rtfKMajor: rtfParAttr; rtfKMinor:     rtfKeepNext; rtfKstr : 'keepn'; rtfkHash : 0),
( rtfKMajor: rtfParAttr; rtfKMinor:     rtfSideBySide; rtfKstr : 'sbys'; rtfkHash : 0),
( rtfKMajor: rtfParAttr; rtfKMinor:     rtfPBBefore; rtfKstr : 'pagebb'; rtfkHash : 0),
( rtfKMajor: rtfParAttr; rtfKMinor:     rtfNoLineNum; rtfKstr : 'noline'; rtfkHash : 0),
( rtfKMajor: rtfParAttr; rtfKMinor:     rtfTabPos; rtfKstr : 'tx'; rtfkHash : 0),
( rtfKMajor: rtfParAttr; rtfKMinor:     rtfTabRight; rtfKstr : 'tqr'; rtfkHash : 0),
( rtfKMajor: rtfParAttr; rtfKMinor:     rtfTabCenter; rtfKstr : 'tqc'; rtfkHash : 0),
( rtfKMajor: rtfParAttr; rtfKMinor:     rtfTabDecimal; rtfKstr : 'tqdec'; rtfkHash : 0),
( rtfKMajor: rtfParAttr; rtfKMinor:     rtfTabBar; rtfKstr : 'tb'; rtfkHash : 0),
( rtfKMajor: rtfParAttr; rtfKMinor:     rtfBorderTop; rtfKstr : 'brdrt'; rtfkHash : 0),
( rtfKMajor: rtfParAttr; rtfKMinor:     rtfBorderBottom; rtfKstr : 'brdrb'; rtfkHash : 0),
( rtfKMajor: rtfParAttr; rtfKMinor:     rtfBorderLeft; rtfKstr : 'brdrl'; rtfkHash : 0),
( rtfKMajor: rtfParAttr; rtfKMinor:     rtfBorderRight; rtfKstr : 'brdrr'; rtfkHash : 0),
( rtfKMajor: rtfParAttr; rtfKMinor:     rtfBorderBar; rtfKstr : 'bar'; rtfkHash : 0),
( rtfKMajor: rtfParAttr; rtfKMinor:     rtfBorderBox; rtfKstr : 'box'; rtfkHash : 0),
( rtfKMajor: rtfParAttr; rtfKMinor:     rtfBorderBetween; rtfKstr : 'brdrbtw'; rtfkHash : 0),
( rtfKMajor: rtfParAttr; rtfKMinor:     rtfBorderSingle; rtfKstr : 'brdrs'; rtfkHash : 0),
( rtfKMajor: rtfParAttr; rtfKMinor:     rtfBorderThick; rtfKstr : 'brdrth'; rtfkHash : 0),
( rtfKMajor: rtfParAttr; rtfKMinor:     rtfBorderShadow; rtfKstr : 'brdrsh'; rtfkHash : 0),
( rtfKMajor: rtfParAttr; rtfKMinor:     rtfBorderDouble; rtfKstr : 'brdrdb'; rtfkHash : 0),
( rtfKMajor: rtfParAttr; rtfKMinor:     rtfBorderDot; rtfKstr : 'brdrdot'; rtfkHash : 0),
( rtfKMajor: rtfParAttr; rtfKMinor:     rtfBorderHair; rtfKstr : 'brdrhair'; rtfkHash : 0),
( rtfKMajor: rtfParAttr; rtfKMinor:     rtfLeaderDot; rtfKstr : 'tldot'; rtfkHash : 0),
( rtfKMajor: rtfParAttr; rtfKMinor:     rtfLeaderHyphen; rtfKstr : 'tlhyph'; rtfkHash : 0),
( rtfKMajor: rtfParAttr; rtfKMinor:     rtfLeaderUnder; rtfKstr : 'tlul'; rtfkHash : 0),
( rtfKMajor: rtfParAttr; rtfKMinor:     rtfLeaderThick; rtfKstr : 'tlth'; rtfkHash : 0),
( rtfKMajor: rtfParAttr; rtfKMinor:     rtfBorderSpace; rtfKstr : 'brsp'; rtfkHash : 0),

( rtfKMajor: rtfSectAttr; rtfKMinor:    rtfSectDef; rtfKstr : 'sectd'; rtfkHash : 0),
( rtfKMajor: rtfSectAttr; rtfKMinor:    rtfNoBreak; rtfKstr : 'sbknone'; rtfkHash : 0),
( rtfKMajor: rtfSectAttr; rtfKMinor:    rtfColBreak; rtfKstr : 'sbkcol'; rtfkHash : 0),
( rtfKMajor: rtfSectAttr; rtfKMinor:    rtfPageBreak; rtfKstr : 'sbkpage'; rtfkHash : 0),
( rtfKMajor: rtfSectAttr; rtfKMinor:    rtfEvenBreak; rtfKstr : 'sbkeven'; rtfkHash : 0),
( rtfKMajor: rtfSectAttr; rtfKMinor:    rtfOddBreak; rtfKstr : 'sbkodd'; rtfkHash : 0),
( rtfKMajor: rtfSectAttr; rtfKMinor:    rtfPageCont; rtfKstr : 'pgncont'; rtfkHash : 0),
( rtfKMajor: rtfSectAttr; rtfKMinor:    rtfPageStarts; rtfKstr : 'pgnstarts'; rtfkHash : 0),
( rtfKMajor: rtfSectAttr; rtfKMinor:    rtfPageRestart; rtfKstr : 'pgnrestart'; rtfkHash : 0),
( rtfKMajor: rtfSectAttr; rtfKMinor:    rtfPageDecimal; rtfKstr : 'pgndec'; rtfkHash : 0),
( rtfKMajor: rtfSectAttr; rtfKMinor:    rtfPageURoman; rtfKstr : 'pgnucrm'; rtfkHash : 0),
( rtfKMajor: rtfSectAttr; rtfKMinor:    rtfPageLRoman; rtfKstr : 'pgnlcrm'; rtfkHash : 0),
( rtfKMajor: rtfSectAttr; rtfKMinor:    rtfPageULetter; rtfKstr : 'pgnucltr'; rtfkHash : 0),
( rtfKMajor: rtfSectAttr; rtfKMinor:    rtfPageLLetter; rtfKstr : 'pgnlcltr'; rtfkHash : 0),
( rtfKMajor: rtfSectAttr; rtfKMinor:    rtfPageNumLeft; rtfKstr : 'pgnx'; rtfkHash : 0),
( rtfKMajor: rtfSectAttr; rtfKMinor:    rtfPageNumTop; rtfKstr : 'pgny'; rtfkHash : 0),
( rtfKMajor: rtfSectAttr; rtfKMinor:    rtfHeaderY; rtfKstr : 'headery'; rtfkHash : 0),
( rtfKMajor: rtfSectAttr; rtfKMinor:    rtfFooterY; rtfKstr : 'footery'; rtfkHash : 0),
( rtfKMajor: rtfSectAttr; rtfKMinor:    rtfLineModulus; rtfKstr : 'linemod'; rtfkHash : 0),
( rtfKMajor: rtfSectAttr; rtfKMinor:    rtfLineDist; rtfKstr : 'linex'; rtfkHash : 0),
( rtfKMajor: rtfSectAttr; rtfKMinor:    rtfLineStarts; rtfKstr : 'linestarts'; rtfkHash : 0),
( rtfKMajor: rtfSectAttr; rtfKMinor:    rtfLineRestart; rtfKstr : 'linerestart'; rtfkHash : 0),
( rtfKMajor: rtfSectAttr; rtfKMinor:    rtfLineRestartPg; rtfKstr : 'lineppage'; rtfkHash : 0),
( rtfKMajor: rtfSectAttr; rtfKMinor:    rtfLineCont; rtfKstr : 'linecont'; rtfkHash : 0),
( rtfKMajor: rtfSectAttr; rtfKMinor:    rtfTopVAlign; rtfKstr : 'vertalt'; rtfkHash : 0),
( rtfKMajor: rtfSectAttr; rtfKMinor:    rtfBottomVAlign; rtfKstr : 'vertal'; rtfkHash : 0),
( rtfKMajor: rtfSectAttr; rtfKMinor:    rtfCenterVAlign; rtfKstr : 'vertalc'; rtfkHash : 0),
( rtfKMajor: rtfSectAttr; rtfKMinor:    rtfJustVAlign; rtfKstr : 'vertalj'; rtfkHash : 0),
( rtfKMajor: rtfSectAttr; rtfKMinor:    rtfColumns; rtfKstr : 'cols'; rtfkHash : 0),
( rtfKMajor: rtfSectAttr; rtfKMinor:    rtfColumnSpace; rtfKstr : 'colsx'; rtfkHash : 0),
( rtfKMajor: rtfSectAttr; rtfKMinor:    rtfColumnLine; rtfKstr : 'linebetcol'; rtfkHash : 0),
( rtfKMajor: rtfSectAttr; rtfKMinor:    rtfENoteHere; rtfKstr : 'endnhere'; rtfkHash : 0),
( rtfKMajor: rtfSectAttr; rtfKMinor:    rtfTitleSpecial; rtfKstr : 'titlepg'; rtfkHash : 0),

( rtfKMajor: rtfDocAttr; rtfKMinor:     rtfPaperWidth; rtfKstr : 'paperw'; rtfkHash : 0),
( rtfKMajor: rtfDocAttr; rtfKMinor:     rtfPaperHeight; rtfKstr : 'paperh'; rtfkHash : 0),
( rtfKMajor: rtfDocAttr; rtfKMinor:     rtfLeftMargin; rtfKstr : 'margl'; rtfkHash : 0),
( rtfKMajor: rtfDocAttr; rtfKMinor:     rtfRightMargin; rtfKstr : 'margr'; rtfkHash : 0),
( rtfKMajor: rtfDocAttr; rtfKMinor:     rtfTopMargin; rtfKstr : 'margt'; rtfkHash : 0),
( rtfKMajor: rtfDocAttr; rtfKMinor:     rtfBottomMargin; rtfKstr : 'margb'; rtfkHash : 0),
( rtfKMajor: rtfDocAttr; rtfKMinor:     rtfFacingPage; rtfKstr : 'facingp'; rtfkHash : 0),
( rtfKMajor: rtfDocAttr; rtfKMinor:     rtfGutterWid; rtfKstr : 'gutter'; rtfkHash : 0),
( rtfKMajor: rtfDocAttr; rtfKMinor:     rtfDefTab; rtfKstr : 'deftab'; rtfkHash : 0),
( rtfKMajor: rtfDocAttr; rtfKMinor:     rtfWidowCtrl; rtfKstr : 'widowctrl'; rtfkHash : 0),
( rtfKMajor: rtfDocAttr; rtfKMinor:     rtfHyphHotZone; rtfKstr : 'hyphhotz'; rtfkHash : 0),
( rtfKMajor: rtfDocAttr; rtfKMinor:     rtfFNoteEndSect; rtfKstr : 'endnotes'; rtfkHash : 0),
( rtfKMajor: rtfDocAttr; rtfKMinor:     rtfFNoteEndDoc; rtfKstr : 'enddoc'; rtfkHash : 0),
( rtfKMajor: rtfDocAttr; rtfKMinor:     rtfFNoteBottom; rtfKstr : 'ftnbj'; rtfkHash : 0),
( rtfKMajor: rtfDocAttr; rtfKMinor:     rtfFNoteText; rtfKstr : 'ftntj'; rtfkHash : 0),
( rtfKMajor: rtfDocAttr; rtfKMinor:     rtfFNoteStart; rtfKstr : 'ftnstart'; rtfkHash : 0),
( rtfKMajor: rtfDocAttr; rtfKMinor:     rtfFNoteRestart; rtfKstr : 'ftnrestart'; rtfkHash : 0),
( rtfKMajor: rtfDocAttr; rtfKMinor:     rtfPageStart; rtfKstr : 'pgnstart'; rtfkHash : 0),
( rtfKMajor: rtfDocAttr; rtfKMinor:     rtfLineStart; rtfKstr : 'linestart'; rtfkHash : 0),
( rtfKMajor: rtfDocAttr; rtfKMinor:     rtfLandscape; rtfKstr : 'landscape'; rtfkHash : 0),
( rtfKMajor: rtfDocAttr; rtfKMinor:     rtfFracWidth; rtfKstr : 'fracwidth'; rtfkHash : 0),
( rtfKMajor: rtfDocAttr; rtfKMinor:     rtfNextFile; rtfKstr : 'nextfile'; rtfkHash : 0),
( rtfKMajor: rtfDocAttr; rtfKMinor:     rtfTemplate; rtfKstr : 'template'; rtfkHash : 0),
( rtfKMajor: rtfDocAttr; rtfKMinor:     rtfMakeBackup; rtfKstr : 'makeback'; rtfkHash : 0),
( rtfKMajor: rtfDocAttr; rtfKMinor:     rtfRTFDefault; rtfKstr : 'defformat'; rtfkHash : 0),
( rtfKMajor: rtfDocAttr; rtfKMinor:     rtfRevisions; rtfKstr : 'revisions'; rtfkHash : 0),
( rtfKMajor: rtfDocAttr; rtfKMinor:     rtfMirrorMargin; rtfKstr : 'margmirror'; rtfkHash : 0),
( rtfKMajor: rtfDocAttr; rtfKMinor:     rtfRevDisplay; rtfKstr : 'revprop'; rtfkHash : 0),
( rtfKMajor: rtfDocAttr; rtfKMinor:     rtfRevBar; rtfKstr : 'revbar'; rtfkHash : 0),

( rtfKMajor: rtfStyleAttr; rtfKMinor:   rtfBasedOn; rtfKstr : 'sbasedon'; rtfkHash : 0),
( rtfKMajor: rtfStyleAttr; rtfKMinor:   rtfNext; rtfKstr : 'snext'; rtfkHash : 0),

( rtfKMajor: rtfPictAttr; rtfKMinor:    rtfMacQD; rtfKstr : 'macpict'; rtfkHash : 0),
( rtfKMajor: rtfPictAttr; rtfKMinor:    rtfWinMetafile; rtfKstr : 'wmetafile'; rtfkHash : 0),
( rtfKMajor: rtfPictAttr; rtfKMinor:    rtfWinBitmap; rtfKstr : 'wbitmap'; rtfkHash : 0),
( rtfKMajor: rtfPictAttr; rtfKMinor:    rtfPicWid; rtfKstr : 'picw'; rtfkHash : 0),
( rtfKMajor: rtfPictAttr; rtfKMinor:    rtfPicHt; rtfKstr : 'pich'; rtfkHash : 0),
( rtfKMajor: rtfPictAttr; rtfKMinor:    rtfPicGoalWid; rtfKstr : 'picwgoal'; rtfkHash : 0),
( rtfKMajor: rtfPictAttr; rtfKMinor:    rtfPicGoalWid; rtfKstr : 'picwGoal'; rtfkHash : 0),
( rtfKMajor: rtfPictAttr; rtfKMinor:    rtfPicGoalHt; rtfKstr : 'pichgoal'; rtfkHash : 0),
( rtfKMajor: rtfPictAttr; rtfKMinor:    rtfPicGoalHt; rtfKstr : 'pichGoal'; rtfkHash : 0),
( rtfKMajor: rtfPictAttr; rtfKMinor:    rtfPicScaleX; rtfKstr : 'picscalex'; rtfkHash : 0),
( rtfKMajor: rtfPictAttr; rtfKMinor:    rtfPicScaleY; rtfKstr : 'picscaley'; rtfkHash : 0),
( rtfKMajor: rtfPictAttr; rtfKMinor:    rtfPicScaled; rtfKstr : 'picscaled'; rtfkHash : 0),
( rtfKMajor: rtfPictAttr; rtfKMinor:    rtfPicCropTop; rtfKstr : 'piccropt'; rtfkHash : 0),
( rtfKMajor: rtfPictAttr; rtfKMinor:    rtfPicCropBottom; rtfKstr : 'piccropb'; rtfkHash : 0),
( rtfKMajor: rtfPictAttr; rtfKMinor:    rtfPicCropLeft; rtfKstr : 'piccropl'; rtfkHash : 0),
( rtfKMajor: rtfPictAttr; rtfKMinor:    rtfPicCropRight; rtfKstr : 'piccropr'; rtfkHash : 0),
( rtfKMajor: rtfPictAttr; rtfKMinor:    rtfPixelBits; rtfKstr : 'wbmbitspixel'; rtfkHash : 0),
( rtfKMajor: rtfPictAttr; rtfKMinor:    rtfBitmapPlanes; rtfKstr : 'wbmplanes'; rtfkHash : 0),
( rtfKMajor: rtfPictAttr; rtfKMinor:    rtfBitmapWid; rtfKstr : 'wbmwidthbytes'; rtfkHash : 0),
( rtfKMajor: rtfPictAttr; rtfKMinor:    rtfPicBinary; rtfKstr : 'bin'; rtfkHash : 0),

( rtfKMajor: rtfNeXTGrAttr; rtfKMinor:  rtfNeXTGWidth; rtfKstr : 'width'; rtfkHash : 0),
( rtfKMajor: rtfNeXTGrAttr; rtfKMinor:  rtfNeXTGHeight; rtfKstr : 'height'; rtfkHash : 0),

( rtfKMajor: rtfDestination; rtfKMinor:         rtfPict; rtfKstr : 'pict'; rtfkHash : 0),
( rtfKMajor: rtfDestination; rtfKMinor:         rtfNeXTGraphic; rtfKstr : 'NeXTGraphic'; rtfkHash : 0),
( rtfKMajor: rtfDestination; rtfKMinor:         rtfFootnote; rtfKstr : 'footnote'; rtfkHash : 0),
( rtfKMajor: rtfDestination; rtfKMinor:         rtfHeader; rtfKstr : 'header'; rtfkHash : 0),
( rtfKMajor: rtfDestination; rtfKMinor:         rtfHeaderLeft; rtfKstr : 'headerl'; rtfkHash : 0),
( rtfKMajor: rtfDestination; rtfKMinor:         rtfHeaderRight; rtfKstr : 'headerr'; rtfkHash : 0),
( rtfKMajor: rtfDestination; rtfKMinor:         rtfHeaderFirst; rtfKstr : 'headerf'; rtfkHash : 0),
( rtfKMajor: rtfDestination; rtfKMinor:         rtfFooter; rtfKstr : 'footer'; rtfkHash : 0),
( rtfKMajor: rtfDestination; rtfKMinor:         rtfFooterLeft; rtfKstr : 'footerl'; rtfkHash : 0),
( rtfKMajor: rtfDestination; rtfKMinor:         rtfFooterRight; rtfKstr : 'footerr'; rtfkHash : 0),
( rtfKMajor: rtfDestination; rtfKMinor:         rtfFooterFirst; rtfKstr : 'footerf'; rtfkHash : 0),
( rtfKMajor: rtfDestination; rtfKMinor:         rtfFNSep; rtfKstr : 'ftnsep'; rtfkHash : 0),
( rtfKMajor: rtfDestination; rtfKMinor:         rtfFNContSep; rtfKstr : 'ftnsepc'; rtfkHash : 0),
( rtfKMajor: rtfDestination; rtfKMinor:         rtfFNContNotice; rtfKstr : 'ftncn'; rtfkHash : 0),
( rtfKMajor: rtfDestination; rtfKMinor:         rtfInfo; rtfKstr : 'info'; rtfkHash : 0),
( rtfKMajor: rtfDestination; rtfKMinor:         rtfStyleSheet; rtfKstr : 'stylesheet'; rtfkHash : 0),
( rtfKMajor: rtfDestination; rtfKMinor:         rtfFontTbl; rtfKstr : 'fonttbl'; rtfkHash : 0),
( rtfKMajor: rtfDestination; rtfKMinor:         rtfColorTbl; rtfKstr : 'colortbl'; rtfkHash : 0),
(*
( rtfKMajor: rtfDestination; rtfKMinor:         rtfAnnotation; rtfKstr : 'annotation'; rtfkHash : 0),
( rtfKMajor: rtfDestination; rtfKMinor:         rtfAnnotID; rtfKstr : 'atnid'; rtfkHash : 0),
*)
( rtfKMajor: rtfDestination; rtfKMinor:         rtfField; rtfKstr : 'field'; rtfkHash : 0),
( rtfKMajor: rtfDestination; rtfKMinor:         rtfFieldInst; rtfKstr : 'fldinst'; rtfkHash : 0),
( rtfKMajor: rtfDestination; rtfKMinor:         rtfFieldResult; rtfKstr : 'fldrslt'; rtfkHash : 0),
( rtfKMajor: rtfDestination; rtfKMinor:         rtfIndex; rtfKstr : 'xe'; rtfkHash : 0),
( rtfKMajor: rtfDestination; rtfKMinor:         rtfIndexBold; rtfKstr : 'bxe'; rtfkHash : 0),
( rtfKMajor: rtfDestination; rtfKMinor:         rtfIndexItalic; rtfKstr : 'ixe'; rtfkHash : 0),
( rtfKMajor: rtfDestination; rtfKMinor:         rtfIndexText; rtfKstr : 'txe'; rtfkHash : 0),
( rtfKMajor: rtfDestination; rtfKMinor:         rtfIndexRange; rtfKstr : 'rxe'; rtfkHash : 0),
( rtfKMajor: rtfDestination; rtfKMinor:         rtfTOC; rtfKstr : 'tc'; rtfkHash : 0),
( rtfKMajor: rtfDestination; rtfKMinor:         rtfBookmarkStart; rtfKstr : 'bkmkstart'; rtfkHash : 0),
( rtfKMajor: rtfDestination; rtfKMinor:         rtfBookmarkEnd; rtfKstr : 'bkmkend'; rtfkHash : 0),
( rtfKMajor: rtfDestination; rtfKMinor:         rtfITitle; rtfKstr : 'title'; rtfkHash : 0),
( rtfKMajor: rtfDestination; rtfKMinor:         rtfISubject; rtfKstr : 'subject'; rtfkHash : 0),
( rtfKMajor: rtfDestination; rtfKMinor:         rtfIAuthor; rtfKstr : 'author'; rtfkHash : 0),
( rtfKMajor: rtfDestination; rtfKMinor:         rtfIOperator; rtfKstr : 'operator'; rtfkHash : 0),
( rtfKMajor: rtfDestination; rtfKMinor:         rtfIKeywords; rtfKstr : 'keywords'; rtfkHash : 0),
( rtfKMajor: rtfDestination; rtfKMinor:         rtfIComment; rtfKstr : 'comment'; rtfkHash : 0),
( rtfKMajor: rtfDestination; rtfKMinor:         rtfIVersion; rtfKstr : 'version'; rtfkHash : 0),
( rtfKMajor: rtfDestination; rtfKMinor:         rtfIDoccomm; rtfKstr : 'doccomm'; rtfkHash : 0),
( rtfKMajor: rtfDestination; rtfKMinor:         rtfICompany; rtfKstr : 'company'; rtfkHash : 0),  (* Added *)

( rtfKMajor: rtfTOCAttr; rtfKMinor:     rtfTOCType; rtfKstr : 'tcf'; rtfkHash : 0),
( rtfKMajor: rtfTOCAttr; rtfKMinor:     rtfTOCLevel; rtfKstr : 'tcl'; rtfkHash : 0),

( rtfKMajor: rtfFontFamily; rtfKMinor:  rtfFFNil; rtfKstr : 'fnil'; rtfkHash : 0),
( rtfKMajor: rtfFontFamily; rtfKMinor:  rtfFFRoman; rtfKstr : 'froman'; rtfkHash : 0),
( rtfKMajor: rtfFontFamily; rtfKMinor:  rtfFFSwiss; rtfKstr : 'fswiss'; rtfkHash : 0),
( rtfKMajor: rtfFontFamily; rtfKMinor:  rtfFFModern; rtfKstr : 'fmodern'; rtfkHash : 0),
( rtfKMajor: rtfFontFamily; rtfKMinor:  rtfFFScript; rtfKstr : 'fscript'; rtfkHash : 0),
( rtfKMajor: rtfFontFamily; rtfKMinor:  rtfFFDecor; rtfKstr : 'fdecor'; rtfkHash : 0),
( rtfKMajor: rtfFontFamily; rtfKMinor:  rtfFFTech; rtfKstr : 'ftech'; rtfkHash : 0),

( rtfKMajor: rtfColorName; rtfKMinor:   rtfRed; rtfKstr : 'red'; rtfkHash : 0),
( rtfKMajor: rtfColorName; rtfKMinor:   rtfGreen; rtfKstr : 'green'; rtfkHash : 0),
( rtfKMajor: rtfColorName; rtfKMinor:   rtfBlue; rtfKstr : 'blue'; rtfkHash : 0),

( rtfKMajor: rtfCharSet; rtfKMinor:     rtfMacCharSet; rtfKstr : 'mac'; rtfkHash : 0),
( rtfKMajor: rtfCharSet; rtfKMinor:     rtfAnsiCharSet; rtfKstr : 'ansi'; rtfkHash : 0),
( rtfKMajor: rtfCharSet; rtfKMinor:     rtfPcCharSet; rtfKstr : 'pc'; rtfkHash : 0),
( rtfKMajor: rtfCharSet; rtfKMinor:     rtfPcaCharSet; rtfKstr : 'pca'; rtfkHash : 0),

( rtfKMajor: rtfTblAttr; rtfKMinor:     rtfCellBordBottom; rtfKstr : 'clbrdrb'; rtfkHash : 0),
( rtfKMajor: rtfTblAttr; rtfKMinor:     rtfCellBordTop; rtfKstr : 'clbrdrt'; rtfkHash : 0),
( rtfKMajor: rtfTblAttr; rtfKMinor:     rtfCellBordLeft; rtfKstr : 'clbrdrl'; rtfkHash : 0),
( rtfKMajor: rtfTblAttr; rtfKMinor:     rtfCellBordRight; rtfKstr : 'clbrdrr'; rtfkHash : 0),
( rtfKMajor: rtfTblAttr; rtfKMinor:     rtfRowDef; rtfKstr : 'trowd'; rtfkHash : 0),
( rtfKMajor: rtfTblAttr; rtfKMinor:     rtfRowLeft; rtfKstr : 'trql'; rtfkHash : 0),
( rtfKMajor: rtfTblAttr; rtfKMinor:     rtfRowRight; rtfKstr : 'trqr'; rtfkHash : 0),
( rtfKMajor: rtfTblAttr; rtfKMinor:     rtfRowCenter; rtfKstr : 'trqc'; rtfkHash : 0),
( rtfKMajor: rtfTblAttr; rtfKMinor:     rtfRowGapH; rtfKstr : 'trgaph'; rtfkHash : 0),
( rtfKMajor: rtfTblAttr; rtfKMinor:     rtfRowHt; rtfKstr : 'trrh'; rtfkHash : 0),
( rtfKMajor: rtfTblAttr; rtfKMinor:     rtfRowLeftEdge; rtfKstr : 'trleft'; rtfkHash : 0),
( rtfKMajor: rtfTblAttr; rtfKMinor:     rtfRowHeader; rtfKstr : 'trhdr'; rtfkHash : 0),  (* Added *)
( rtfKMajor: rtfTblAttr; rtfKMinor:     rtfCellPos; rtfKstr : 'cellx'; rtfkHash : 0),
( rtfKMajor: rtfTblAttr; rtfKMinor:     rtfCellBackColor; rtfKstr : 'clcbpat'; rtfkHash : 0),  (* Added *)
( rtfKMajor: rtfTblAttr; rtfKMinor:     rtfMergeRngFirst; rtfKstr : 'clmgf'; rtfkHash : 0),
( rtfKMajor: rtfTblAttr; rtfKMinor:     rtfMergePrevious; rtfKstr : 'clmrg'; rtfkHash : 0),

( rtfKMajor: rtfFieldAttr; rtfKMinor:   rtfFieldDirty; rtfKstr : 'flddirty'; rtfkHash : 0),
( rtfKMajor: rtfFieldAttr; rtfKMinor:   rtfFieldEdited; rtfKstr : 'fldedit'; rtfkHash : 0),
( rtfKMajor: rtfFieldAttr; rtfKMinor:   rtfFieldLocked; rtfKstr : 'fldlock'; rtfkHash : 0),
( rtfKMajor: rtfFieldAttr; rtfKMinor:   rtfFieldPrivate; rtfKstr : 'fldpriv'; rtfkHash : 0),

( rtfKMajor: rtfPosAttr; rtfKMinor:     rtfPosX; rtfKstr : 'posx'; rtfkHash : 0),
( rtfKMajor: rtfPosAttr; rtfKMinor:     rtfPosXCenter; rtfKstr : 'posxc'; rtfkHash : 0),
( rtfKMajor: rtfPosAttr; rtfKMinor:     rtfPosXInside; rtfKstr : 'posxi'; rtfkHash : 0),
( rtfKMajor: rtfPosAttr; rtfKMinor:     rtfPosXLeft; rtfKstr : 'posxl'; rtfkHash : 0),
( rtfKMajor: rtfPosAttr; rtfKMinor:     rtfPosXOutSide; rtfKstr : 'posxo'; rtfkHash : 0),
( rtfKMajor: rtfPosAttr; rtfKMinor:     rtfPosXRight; rtfKstr : 'posxr'; rtfkHash : 0),
( rtfKMajor: rtfPosAttr; rtfKMinor:     rtfPosY; rtfKstr : 'posy'; rtfkHash : 0),
( rtfKMajor: rtfPosAttr; rtfKMinor:     rtfPosYInline; rtfKstr : 'posyil'; rtfkHash : 0),
( rtfKMajor: rtfPosAttr; rtfKMinor:     rtfPosYTop; rtfKstr : 'posyt'; rtfkHash : 0),
( rtfKMajor: rtfPosAttr; rtfKMinor:     rtfPosYCenter; rtfKstr : 'posyc'; rtfkHash : 0),
( rtfKMajor: rtfPosAttr; rtfKMinor:     rtfPosYBottom; rtfKstr : 'posyb'; rtfkHash : 0),
( rtfKMajor: rtfPosAttr; rtfKMinor:     rtfAbsWid; rtfKstr : 'absw'; rtfkHash : 0),
( rtfKMajor: rtfPosAttr; rtfKMinor:     rtfTextDist; rtfKstr : 'dxfrtext'; rtfkHash : 0),
( rtfKMajor: rtfPosAttr; rtfKMinor:     rtfRPosMargV; rtfKstr : 'pvmrg'; rtfkHash : 0),
( rtfKMajor: rtfPosAttr; rtfKMinor:     rtfRPosPageV; rtfKstr : 'pvpg'; rtfkHash : 0),
( rtfKMajor: rtfPosAttr; rtfKMinor:     rtfRPosMargH; rtfKstr : 'phmrg'; rtfkHash : 0),
( rtfKMajor: rtfPosAttr; rtfKMinor:     rtfRPosPageH; rtfKstr : 'phpg'; rtfkHash : 0),
( rtfKMajor: rtfPosAttr; rtfKMinor:     rtfRPosColH; rtfKstr : 'phcol'; rtfkHash : 0),

( rtfKMajor: rtfVersion; rtfKMinor:     -1; rtfKstr : 'rtf'; rtfkHash : 0),
( rtfKMajor: rtfDefFont; rtfKMinor:     -1; rtfKstr : 'deff'; rtfkHash : 0),

( rtfKMajor: 0; rtfKMinor:              -1; rtfKstr : ''; rtfkHash : 0)
);

