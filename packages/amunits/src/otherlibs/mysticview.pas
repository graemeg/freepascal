{
  This file is part of the Free Pascal run time library.

  A file in Amiga system run time library.
  Copyright (c) 2003 by Nils Sjöholm.
  member of the Amiga RTL development team.

  This is a unit for mysticview.library

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

  Changed startcode for unit.
  12 Feb 2003.

  nils.sjoholm@mailbox.swipnet.se Nils Sjoholm
}

{$mode objfpc}
{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
   {$smartlink on}
{$endif use_amiga_smartlink}

UNIT MYSTICVIEW;

INTERFACE
USES Exec,graphics,utility,intuition;

VAR MysticBase : pLibrary;

const
    MYSTICVIEWNAME : PChar = 'mysticview.library';

  {
        $VER: mysticview.h 4.2 (1.6.99)

        mysticview.library definitions

        © 1997-99 TEK neoscientists
   }

  {
        Tags
    }

  const
     MVIEW_Dummy = TAG_USER + 765432;
  { left edge in rastport }
     MVIEW_DestX = MVIEW_Dummy + 1;
  { top edge in rastport }
     MVIEW_DestY = MVIEW_Dummy + 2;
  { width in rastport }
     MVIEW_DestWidth = MVIEW_Dummy + 3;
  { height in rastport }
     MVIEW_DestHeight = MVIEW_Dummy + 4;
  { background color }
     MVIEW_BackColor = MVIEW_Dummy + 5;
  { OBP_Precision (view.h) }
     MVIEW_Precision = MVIEW_Dummy + 6;
  { aspect mode - see definitions below }
     MVIEW_DisplayMode = MVIEW_Dummy + 7;
  { preview mode - see definitions below }
     MVIEW_PreviewMode = MVIEW_Dummy + 8;
  { a guigfx.library picture }
     MVIEW_Picture = MVIEW_Dummy + 9;
  { simple text line }
     MVIEW_Text = MVIEW_Dummy + 10;
  { static palette }
     MVIEW_StaticPalette = MVIEW_Dummy + 11;
  { dither activation mode (see below) }
     MVIEW_Dither = MVIEW_Dummy + 12;
  { histogram type }
     MVIEW_HSType = MVIEW_Dummy + 13;
  { screen aspect horizontal }
     MVIEW_ScreenAspectX = MVIEW_Dummy + 14;
  { screen aspect vertical }
     MVIEW_ScreenAspectY = MVIEW_Dummy + 15;
  { dither mode }
     MVIEW_DitherMode = MVIEW_Dummy + 16;
  { display cursor }
     MVIEW_ShowCursor = MVIEW_Dummy + 17;
  { zoom factor (0.1 ... 10) }
     MVIEW_Zoom = MVIEW_Dummy + 18;
  { x position (0 ... 1) }
     MVIEW_XPos = MVIEW_Dummy + 19;
  { y position (0 ... 1) }
     MVIEW_YPos = MVIEW_Dummy + 20;
  { rotation (0 ... 1) }
     MVIEW_Rotation = MVIEW_Dummy + 21;
  { do not use }
     MVIEW_AutoDither = MVIEW_Dummy + 22;
  { picture fully drawn }
     MVIEW_ReadySignal = MVIEW_Dummy + 23;
  { picture X inside the rastport }
     MVIEW_PictureX = MVIEW_Dummy + 24;
  { picture Y inside the rastport }
     MVIEW_PictureY = MVIEW_Dummy + 25;
  { picture Width inside the rastport }
     MVIEW_PictureWidth = MVIEW_Dummy + 26;
  { picture Height inside the rastport }
     MVIEW_PictureHeight = MVIEW_Dummy + 27;
  { indicate scrollability }
     MVIEW_DrawArrows = MVIEW_Dummy + 28;
  { show PIP layer }
     MVIEW_ShowPip = MVIEW_Dummy + 29;
  { text/grid color }
     MVIEW_TextColor = MVIEW_Dummy + 30;
  { color for pip-border, cursor, arrows... }
     MVIEW_MarkColor = MVIEW_Dummy + 31;
  { rastport semaphore (MV_Create() only) }
     MVIEW_RPSemaphore = MVIEW_Dummy + 32;
  { set task priority (MV_Create() only) }
     MVIEW_Priority = MVIEW_Dummy + 33;
  {
        Types
    }
  { image fits exactly into view }
     MVDISPMODE_FIT = 0;
  { image is fully visible }
     MVDISPMODE_KEEPASPECT_MIN = 1;
  { width or height is fully visible }
     MVDISPMODE_KEEPASPECT_MAX = 2;
  { the image aspect is ignored }
     MVDISPMODE_ONEPIXEL = 3;
  { aspect ratios are ignored }
     MVDISPMODE_IGNOREASPECT = 4;
  { no realtime refresh }
     MVPREVMODE_NONE = 0;
  { grid realtime refresh }
     MVPREVMODE_GRID = 1;
  { opaque realtime refresh }
     MVPREVMODE_OPAQUE = 2;
  { dithering on }
     MVDITHERMODE_ON = 0;
  { dithering off }
     MVDITHERMODE_OFF = 1;
  { auto dithering  }
     MVDITHERMODE_AUTO = 2;





FUNCTION MV_CreateA(screen : pScreen; a1arg : pRastPort; tags : pTagItem) : POINTER;
PROCEDURE MV_Delete(mview : POINTER);
PROCEDURE MV_DrawOff(mview : POINTER);
FUNCTION MV_DrawOn(mview : POINTER) : BOOLEAN;
PROCEDURE MV_GetAttrsA(mview : POINTER; tags : pTagItem);
PROCEDURE MV_Refresh(mview : POINTER);
PROCEDURE MV_SetAttrsA(mview : POINTER; tags : pTagItem);
PROCEDURE MV_SetViewRelative(mview : POINTER; x : LONGINT; y : LONGINT);
PROCEDURE MV_SetViewStart(mview : POINTER; x : LONGINT; y : LONGINT);
{
 Functions and procedures with array of const go here
}
FUNCTION MV_Create(screen : pScreen; a1arg : pRastPort; const tags : Array Of Const) : POINTER;
PROCEDURE MV_GetAttrs(mview : POINTER; const tags : Array Of Const);
PROCEDURE MV_SetAttrs(mview : POINTER; const tags : Array Of Const);

{You can remove this include and use a define instead}
{$I useautoopenlib.inc}
{$ifdef use_init_openlib}
procedure InitMYSTICVIEWLibrary;
{$endif use_init_openlib}

{This is a variable that knows how the unit is compiled}
var
    MYSTICVIEWIsCompiledHow : longint;

IMPLEMENTATION

uses
{$ifndef dont_use_openlib}
msgbox,
{$endif dont_use_openlib}
tagsarray;

FUNCTION MV_CreateA(screen : pScreen; a1arg : pRastPort; tags : pTagItem) : POINTER;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L screen,A0
        MOVEA.L a1arg,A1
        MOVEA.L tags,A2
        MOVEA.L MysticBase,A6
        JSR     -030(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE MV_Delete(mview : POINTER);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L mview,A0
        MOVEA.L MysticBase,A6
        JSR     -036(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE MV_DrawOff(mview : POINTER);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L mview,A0
        MOVEA.L MysticBase,A6
        JSR     -054(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION MV_DrawOn(mview : POINTER) : BOOLEAN;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L mview,A0
        MOVEA.L MysticBase,A6
        JSR     -048(A6)
        MOVEA.L (A7)+,A6
        TST.W   D0
        BEQ.B   @end
        MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

PROCEDURE MV_GetAttrsA(mview : POINTER; tags : pTagItem);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L mview,A0
        MOVEA.L tags,A1
        MOVEA.L MysticBase,A6
        JSR     -066(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE MV_Refresh(mview : POINTER);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L mview,A0
        MOVEA.L MysticBase,A6
        JSR     -060(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE MV_SetAttrsA(mview : POINTER; tags : pTagItem);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L mview,A0
        MOVEA.L tags,A1
        MOVEA.L MysticBase,A6
        JSR     -042(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE MV_SetViewRelative(mview : POINTER; x : LONGINT; y : LONGINT);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L mview,A0
        MOVE.L  x,D0
        MOVE.L  y,D1
        MOVEA.L MysticBase,A6
        JSR     -078(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE MV_SetViewStart(mview : POINTER; x : LONGINT; y : LONGINT);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L mview,A0
        MOVE.L  x,D0
        MOVE.L  y,D1
        MOVEA.L MysticBase,A6
        JSR     -072(A6)
        MOVEA.L (A7)+,A6
  END;
END;

{
 Functions and procedures with array of const go here
}
FUNCTION MV_Create(screen : pScreen; a1arg : pRastPort; const tags : Array Of Const) : POINTER;
begin
    MV_Create := MV_CreateA(screen , a1arg , readintags(tags));
end;

PROCEDURE MV_GetAttrs(mview : POINTER; const tags : Array Of Const);
begin
    MV_GetAttrsA(mview , readintags(tags));
end;

PROCEDURE MV_SetAttrs(mview : POINTER; const tags : Array Of Const);
begin
    MV_SetAttrsA(mview , readintags(tags));
end;

const
    { Change VERSION and LIBVERSION to proper values }

    VERSION : string[2] = '0';
    LIBVERSION : longword = 0;

{$ifdef use_init_openlib}
  {$Info Compiling initopening of mysticview.library}
  {$Info don't forget to use InitMYSTICVIEWLibrary in the beginning of your program}

var
    mysticview_exit : Pointer;

procedure ClosemysticviewLibrary;
begin
    ExitProc := mysticview_exit;
    if MysticBase <> nil then begin
        CloseLibrary(MysticBase);
        MysticBase := nil;
    end;
end;

procedure InitMYSTICVIEWLibrary;
begin
    MysticBase := nil;
    MysticBase := OpenLibrary(MYSTICVIEWNAME,LIBVERSION);
    if MysticBase <> nil then begin
        mysticview_exit := ExitProc;
        ExitProc := @ClosemysticviewLibrary;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open mysticview.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;
end;

begin
    MYSTICVIEWIsCompiledHow := 2;
{$endif use_init_openlib}

{$ifdef use_auto_openlib}
  {$Info Compiling autoopening of mysticview.library}

var
    mysticview_exit : Pointer;

procedure ClosemysticviewLibrary;
begin
    ExitProc := mysticview_exit;
    if MysticBase <> nil then begin
        CloseLibrary(MysticBase);
        MysticBase := nil;
    end;
end;

begin
    MysticBase := nil;
    MysticBase := OpenLibrary(MYSTICVIEWNAME,LIBVERSION);
    if MysticBase <> nil then begin
        mysticview_exit := ExitProc;
        ExitProc := @ClosemysticviewLibrary;
        MYSTICVIEWIsCompiledHow := 1;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open mysticview.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;

{$endif use_auto_openlib}

{$ifdef dont_use_openlib}
begin
    MYSTICVIEWIsCompiledHow := 3;
   {$Warning No autoopening of mysticview.library compiled}
   {$Warning Make sure you open mysticview.library yourself}
{$endif dont_use_openlib}

END. (* UNIT MYSTICVIEW *)



