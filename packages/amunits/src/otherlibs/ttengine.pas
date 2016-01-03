{
  This file is part of the Free Pascal run time library.

  A file in Amiga system run time library.
  Copyright (c) 2003 by Nils Sjöholm.
  member of the Amiga RTL development team.

  This is a unit for ttengine.library

  See the file COPYING.FPC, included in this distribution,
  for details about the copyright.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

**********************************************************************}
{
  History:

  First version of this unit.
  16 Jan 2003.

  Changed cardinal > longword.
  Changed startcode for unit.
  12 Feb 2003.

  nils.sjoholm@mailbox.swipnet.se Nils Sjoholm
}

{$mode objfpc}
{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
   {$smartlink on}
{$endif use_amiga_smartlink}

UNIT TTENGINE;

INTERFACE
USES Exec,utility,graphics;

VAR TTEngineBase : pLibrary;

const
    TTENGINENAME : PChar = 'ttengine.library';


  { $VER: ttengine.h 6.0 (3.1.2003) (c) by Grzegorz Kraszewski 2002.  }

  const

     TTENGINEVERSION = 6;
     TTENGINEMINVERSION = 3;
  { Tags  }
  { Tags applicability legend:  }
  { O - TT_OpenFont()  }
  { G - TT_GetAttrs()  }
  { S - TT_SetAttrs()  }
  { P - TT_GetPixmap()  }
  { ---- name -------------------- value ----- applicability  }
  { OG..  }
     TT_FontFile = $6EDA0000;
  { OG..  }
     TT_FontStyle = $6EDA0001;
     TT_FontStyle_Regular = 0;
     TT_FontStyle_Italic = 1;
  { O...  }
     TT_FamilyTable = $6EDA0002;
  { OG..  }
     TT_FontSize = $6EDA0003;
  { OG..  }
     TT_FontWeight = $6EDA0004;
     TT_FontWeight_Normal = 400;
     TT_FontWeight_Bold = 700;
  { O...  }
     TT_ColorMap = $6EDA0005;
  { O...  }
     TT_Screen = $6EDA0006;
  { O...  }
     TT_Window = $6EDA0007;
  { .G..  }
     TT_FontAscender = $6EDA0008;
  { .G..  }
     TT_FontDescender = $6EDA0009;
  { .GSP  }
     TT_Antialias = $6EDA000F;
     TT_Antialias_Auto = 0;
     TT_Antialias_Off = 1;
     TT_Antialias_On = 2;
  { .GSP  }
     TT_Encoding = $6EDA0010;
  { supported  }
  { use ENV:ttfcodepage or ISO-8859-1 if not found  }
     TT_Encoding_Default = 0;
  { Western Europe and US  }
     TT_Encoding_ISO8859_1 = 4;
  { Eastern Europe  }
     TT_Encoding_ISO8859_2 = 5;
     TT_Encoding_ISO8859_3 = 6;
     TT_Encoding_ISO8859_4 = 7;
     TT_Encoding_ISO8859_5 = 8;
     TT_Encoding_ISO8859_6 = 9;
     TT_Encoding_ISO8859_7 = 10;
     TT_Encoding_ISO8859_8 = 11;
     TT_Encoding_ISO8859_9 = 12;
     TT_Encoding_ISO8859_10 = 13;
     TT_Encoding_ISO8859_11 = 14;
     TT_Encoding_ISO8859_13 = 109;
     TT_Encoding_ISO8859_14 = 110;
     TT_Encoding_ISO8859_15 = 111;
     TT_Encoding_ISO8859_16 = 112;
     TT_Encoding_UTF16_BE = 1013;
     TT_Encoding_UTF32_BE = 1018;
     TT_Encoding_UTF8 = 106;
     TT_Encoding_UTF16_LE = 1014;
     TT_Encoding_UTF32_LE = 1019;
     TT_Encoding_UTF16 = 1015;
     TT_Encoding_UTF32 = 1017;
  { .G..  }
     TT_FontName = $6EDA0011;
  { .G..  }
     TT_FamilyName = $6EDA0012;
  { .G..  }
     TT_SubfamilyName = $6EDA0013;
  { .GS.  from 0 to 255  }
     TT_Transparency = $6EDA0014;
  { O.SP  single precision floating point +- 0.01 to 100  }
     TT_ScaleX = $6EDA0015;
  { O.SP  single precision floating point +- 0.01 to 100  }
     TT_ScaleY = $6EDA0016;
  { ..SP (V5)  }
     TT_SoftStyle = $6EDA0017;
     TT_SoftStyle_None = $0000;
     TT_SoftStyle_Underlined = $0001;
     TT_SoftStyle_DblUnderlined = $0002;
     TT_SoftStyle_Overstriked = $0004;
     TT_SoftStyle_DblOverstriked = $0008;
  { ..S.  foreground RGB value }
     TT_Foreground = $6EDA0018;
     TT_Foreground_UseRastPort = -(1);
  { ..S.  background RGB value }
     TT_Background = $6EDA0019;
     TT_Background_UseRastPort = -(1);
  { .G..  }
     TT_FontMaxTop = $6EDA001E;
  { .G..  }
     TT_FontMaxBottom = $6EDA001F;
  { .G..  }
     TT_FontDesignHeight = $6EDA0020;
  { .G..  }
     TT_FontRealAscender = $6EDA0021;
  { .G..  }
     TT_FontRealDescender = $6EDA0022;
  { .G..  }
     TT_FontAccentedAscender = $6EDA0023;
  { ..SP  }
     TT_CustomEncoding = $6EDA0024;


  { Structure returned by TT_GetPixmap() (V5) }
  type
     PTT_Pixmap = ^tTT_Pixmap;
     tTT_Pixmap = record
          ttp_Size : ULONG;      { size of the structure inculdung this field  }
          ttp_Width : ULONG;     { also equal to bytes per row  }
          ttp_Height : ULONG;    { number of rows  }
          ttp_Data : Pointer;    { grayscale pixmap data  }
       end;

  { font requester attributes (V6)  }

  const
  { struct Window ,   NULL               }
     TTRQ_Window = $6EDA2000;
  { STRPTR,           NULL [Workbench]   }
     TTRQ_PubScreenName = $6EDA2001;
  { struct Screen ,   NULL               }
     TTRQ_Screen = $6EDA2002;
  { BOOL,             FALSE              }
     TTRQ_SleepWindow = $6EDA2003;
  { STRPTR,           "Select TrueType font" or localized  }
     TTRQ_TitleText = $6EDA2004;
  { STRPTR,           "OK" or localized  }
     TTRQ_PositiveText = $6EDA2005;
  { STRPTR,           "Cancel" or localized  }
     TTRQ_NegativeText = $6EDA2006;
  { WORD,             centered on screen  }
     TTRQ_InitialLeftEdge = $6EDA2007;
  { WORD,             centered on screen  }
     TTRQ_InitialTopEdge = $6EDA2008;
  { WORD,             max(200, 25% of sceeen width)  }
     TTRQ_InitialWidth = $6EDA2009;
  { WORD,             max(200, 50% of screen height)  }
     TTRQ_InitialHeight = $6EDA200A;
  { BOOL,             TRUE               }
     TTRQ_DoSizes = $6EDA2000;



FUNCTION TT_AllocRequest : POINTER;
PROCEDURE TT_CloseFont(font : POINTER);
PROCEDURE TT_DoneRastPort(rp : pRastPort);
PROCEDURE TT_FreePixmap(pixmap : pTT_Pixmap);
PROCEDURE TT_FreeRequest(request : POINTER);
FUNCTION TT_GetAttrsA(rp : pRastPort; taglist : pTagItem) : longword;
FUNCTION TT_GetPixmapA(font : POINTER; _string : POINTER; count : longword; taglist : pTagItem) : pTT_Pixmap;
FUNCTION TT_OpenFontA(taglist : pTagItem) : POINTER;
FUNCTION TT_RequestA(request : POINTER; taglist : pTagItem) : pTagItem;
FUNCTION TT_SetAttrsA(rp : pRastPort; taglist : pTagItem) : longword;
FUNCTION TT_SetFont(rp : pRastPort; font : POINTER) : BOOLEAN;
PROCEDURE TT_Text(rp : pRastPort; _string : POINTER; count : longword);
PROCEDURE TT_TextExtent(rp : pRastPort; _string : POINTER; count : LONGINT; te : pTextExtent);
FUNCTION TT_TextFit(rp : pRastPort; _string : POINTER; count : longword; te : pTextExtent; tec : pTextExtent; dir : LONGINT; cwidth : longword; cheight : longword) : longword;
FUNCTION TT_TextLength(rp : pRastPort; _string : POINTER; count : longword) : longword;
{
 Functions and procedures with array of const go here
}
FUNCTION TT_GetAttrs(rp : pRastPort; const taglist : Array Of Const) : longword;
FUNCTION TT_GetPixmap(font : POINTER; _string : POINTER; count : longword; const taglist : Array Of Const) : pTT_Pixmap;
FUNCTION TT_OpenFont(const taglist : Array Of Const) : POINTER;
FUNCTION TT_Request(request : POINTER; const taglist : Array Of Const) : pTagItem;
FUNCTION TT_SetAttrs(rp : pRastPort; const taglist : Array Of Const) : longword;

{You can remove this include and use a define instead}
{$I useautoopenlib.inc}
{$ifdef use_init_openlib}
procedure InitTTENGINELibrary;
{$endif use_init_openlib}

{This is a variable that knows how the unit is compiled}
var
    TTENGINEIsCompiledHow : longint;

IMPLEMENTATION

uses
{$ifndef dont_use_openlib}
msgbox,
{$endif dont_use_openlib}
tagsarray;

FUNCTION TT_AllocRequest : POINTER;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L TTEngineBase,A6
        JSR     -102(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE TT_CloseFont(font : POINTER);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L font,A0
        MOVEA.L TTEngineBase,A6
        JSR     -042(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE TT_DoneRastPort(rp : pRastPort);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L rp,A1
        MOVEA.L TTEngineBase,A6
        JSR     -096(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE TT_FreePixmap(pixmap : pTT_Pixmap);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L pixmap,A0
        MOVEA.L TTEngineBase,A6
        JSR     -090(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE TT_FreeRequest(request : POINTER);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L request,A0
        MOVEA.L TTEngineBase,A6
        JSR     -114(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION TT_GetAttrsA(rp : pRastPort; taglist : pTagItem) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L rp,A1
        MOVEA.L taglist,A0
        MOVEA.L TTEngineBase,A6
        JSR     -060(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION TT_GetPixmapA(font : POINTER; _string : POINTER; count : longword; taglist : pTagItem) : pTT_Pixmap;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L font,A1
        MOVEA.L _string,A2
        MOVE.L  count,D0
        MOVEA.L taglist,A0
        MOVEA.L TTEngineBase,A6
        JSR     -084(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION TT_OpenFontA(taglist : pTagItem) : POINTER;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L taglist,A0
        MOVEA.L TTEngineBase,A6
        JSR     -030(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION TT_RequestA(request : POINTER; taglist : pTagItem) : pTagItem;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L request,A0
        MOVEA.L taglist,A1
        MOVEA.L TTEngineBase,A6
        JSR     -108(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION TT_SetAttrsA(rp : pRastPort; taglist : pTagItem) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L rp,A1
        MOVEA.L taglist,A0
        MOVEA.L TTEngineBase,A6
        JSR     -054(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION TT_SetFont(rp : pRastPort; font : POINTER) : BOOLEAN;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L rp,A1
        MOVEA.L font,A0
        MOVEA.L TTEngineBase,A6
        JSR     -036(A6)
        MOVEA.L (A7)+,A6
        TST.W   D0
        BEQ.B   @end
        MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

PROCEDURE TT_Text(rp : pRastPort; _string : POINTER; count : longword);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L rp,A1
        MOVEA.L _string,A0
        MOVE.L  count,D0
        MOVEA.L TTEngineBase,A6
        JSR     -048(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE TT_TextExtent(rp : pRastPort; _string : POINTER; count : LONGINT; te : pTextExtent);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L rp,A1
        MOVEA.L _string,A0
        MOVE.L  count,D0
        MOVEA.L te,A2
        MOVEA.L TTEngineBase,A6
        JSR     -072(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION TT_TextFit(rp : pRastPort; _string : POINTER; count : longword; te : pTextExtent; tec : pTextExtent; dir : LONGINT; cwidth : longword; cheight : longword) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L rp,A1
        MOVEA.L _string,A0
        MOVE.L  count,D0
        MOVEA.L te,A2
        MOVEA.L tec,A3
        MOVE.L  dir,D1
        MOVE.L  cwidth,D2
        MOVE.L  cheight,D3
        MOVEA.L TTEngineBase,A6
        JSR     -078(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION TT_TextLength(rp : pRastPort; _string : POINTER; count : longword) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L rp,A1
        MOVEA.L _string,A0
        MOVE.L  count,D0
        MOVEA.L TTEngineBase,A6
        JSR     -066(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

{
 Functions and procedures with array of const go here
}
FUNCTION TT_GetAttrs(rp : pRastPort; const taglist : Array Of Const) : longword;
begin
    TT_GetAttrs := TT_GetAttrsA(rp , readintags(taglist));
end;

FUNCTION TT_GetPixmap(font : POINTER; _string : POINTER; count : longword; const taglist : Array Of Const) : pTT_Pixmap;
begin
    TT_GetPixmap := TT_GetPixmapA(font , _string , count , readintags(taglist));
end;

FUNCTION TT_OpenFont(const taglist : Array Of Const) : POINTER;
begin
    TT_OpenFont := TT_OpenFontA(readintags(taglist));
end;

FUNCTION TT_Request(request : POINTER; const taglist : Array Of Const) : pTagItem;
begin
    TT_Request := TT_RequestA(request , readintags(taglist));
end;

FUNCTION TT_SetAttrs(rp : pRastPort; const taglist : Array Of Const) : longword;
begin
    TT_SetAttrs := TT_SetAttrsA(rp , readintags(taglist));
end;

const
    { Change VERSION and LIBVERSION to proper values }

    VERSION : string[2] = '0';
    LIBVERSION : longword = 0;

{$ifdef use_init_openlib}
  {$Info Compiling initopening of ttengine.library}
  {$Info don't forget to use InitTTENGINELibrary in the beginning of your program}

var
    ttengine_exit : Pointer;

procedure ClosettengineLibrary;
begin
    ExitProc := ttengine_exit;
    if TTEngineBase <> nil then begin
        CloseLibrary(TTEngineBase);
        TTEngineBase := nil;
    end;
end;

procedure InitTTENGINELibrary;
begin
    TTEngineBase := nil;
    TTEngineBase := OpenLibrary(TTENGINENAME,LIBVERSION);
    if TTEngineBase <> nil then begin
        ttengine_exit := ExitProc;
        ExitProc := @ClosettengineLibrary;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open ttengine.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;
end;

begin
    TTENGINEIsCompiledHow := 2;
{$endif use_init_openlib}

{$ifdef use_auto_openlib}
  {$Info Compiling autoopening of ttengine.library}

var
    ttengine_exit : Pointer;

procedure ClosettengineLibrary;
begin
    ExitProc := ttengine_exit;
    if TTEngineBase <> nil then begin
        CloseLibrary(TTEngineBase);
        TTEngineBase := nil;
    end;
end;

begin
    TTEngineBase := nil;
    TTEngineBase := OpenLibrary(TTENGINENAME,LIBVERSION);
    if TTEngineBase <> nil then begin
        ttengine_exit := ExitProc;
        ExitProc := @ClosettengineLibrary;
        TTENGINEIsCompiledHow := 1;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open ttengine.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;

{$endif use_auto_openlib}

{$ifdef dont_use_openlib}
begin
    TTENGINEIsCompiledHow := 3;
   {$Warning No autoopening of ttengine.library compiled}
   {$Warning Make sure you open ttengine.library yourself}
{$endif dont_use_openlib}


END. (* UNIT TTENGINE *)



