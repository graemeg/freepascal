{
  This file is part of the Free Pascal run time library.

  A file in Amiga system run time library.
  Copyright (c) 2003 by Nils Sjöholm.
  member of the Amiga RTL development team.

  This is a unit for guigfx.library

  See the file COPYING.FPC, included in this distribution,
  for details about the copyright.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

**********************************************************************}
{
  History:

  First version of this unit.
  15 Jan 2003.

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

UNIT GUIGFX;

INTERFACE
USES Exec,utility,graphics;

VAR GuiGFXBase : pLibrary;

const
    GUIGFXNAME : PChar = 'guigfx.library';


  {
        $VER: guigfx.h 17.2 (9.2.2000)

        guigfx.library definitions

        © 1997-2000 TEK neoscientists
   }

  {
        Tags
    }

  const
     GGFX_Dummy = 4567 + TAG_USER;
  { strictly private  }
     GGFX_Owner = GGFX_Dummy + 0;
     GGFX_HSType = GGFX_Dummy + 1;
     GGFX_DitherMode = GGFX_Dummy + 2;
     GGFX_DitherAmount = GGFX_Dummy + 3;
     GGFX_AutoDither = GGFX_Dummy + 4;
     GGFX_DitherThreshold = GGFX_Dummy + 5;
     GGFX_AspectX = GGFX_Dummy + 6;
     GGFX_AspectY = GGFX_Dummy + 7;
     GGFX_PixelFormat = GGFX_Dummy + 8;
     GGFX_Palette = GGFX_Dummy + 9;
     GGFX_PaletteFormat = GGFX_Dummy + 10;
     GGFX_NumColors = GGFX_Dummy + 11;
     GGFX_Precision = GGFX_Dummy + 12;
     GGFX_Weight = GGFX_Dummy + 13;
     GGFX_Ratio = GGFX_Dummy + 14;
     GGFX_SourceWidth = GGFX_Dummy + 15;
     GGFX_SourceHeight = GGFX_Dummy + 16;
     GGFX_SourceX = GGFX_Dummy + 17;
     GGFX_SourceY = GGFX_Dummy + 18;
     GGFX_DestWidth = GGFX_Dummy + 19;
     GGFX_DestHeight = GGFX_Dummy + 20;
     GGFX_DestX = GGFX_Dummy + 21;
     GGFX_DestY = GGFX_Dummy + 22;
     GGFX_CallBackHook = GGFX_Dummy + 23;
     GGFX_ErrorCode = GGFX_Dummy + 24;
     GGFX_MaxAllocPens = GGFX_Dummy + 25;
     GGFX_BufferSize = GGFX_Dummy + 26;
     GGFX_AlphaPresent = GGFX_Dummy + 27;
     GGFX_Independent = GGFX_Dummy + 28;
     GGFX_ModeID = GGFX_Dummy + 29;
     GGFX_PenTable = GGFX_Dummy + 30;
  { obsolete  }
     GGFX_License = GGFX_Dummy + 31;
     GGFX_BGColor = GGFX_Dummy + 32;
     GGFX_UseMask = GGFX_Dummy + 33;
     GGFX_RastLock = GGFX_Dummy + 34;
     GGFX_FormatName = GGFX_Dummy + 35;
  {
        Picture Attributes
    }
     PICATTR_Dummy = 123 + TAG_USER;
     PICATTR_Width = PICATTR_Dummy + 0;
     PICATTR_Height = PICATTR_Dummy + 1;
     PICATTR_RawData = PICATTR_Dummy + 2;
     PICATTR_PixelFormat = PICATTR_Dummy + 3;
     PICATTR_AspectX = PICATTR_Dummy + 4;
     PICATTR_AspectY = PICATTR_Dummy + 5;
     PICATTR_AlphaPresent = PICATTR_Dummy + 6;
  {
        Picture Methods
    }
     PICMTHD_CROP = 1;
     PICMTHD_RENDER = 2;
     PICMTHD_SCALE = 3;
     PICMTHD_MIX = 4;
     PICMTHD_SETALPHA = 5;
     PICMTHD_MIXALPHA = 6;
     PICMTHD_MAPDRAWHANDLE = 7;
     PICMTHD_CREATEALPHAMASK = 8;
     PICMTHD_TINT = 9;
     PICMTHD_TEXTURE = 10;
     PICMTHD_SET = 11;
     PICMTHD_TINTALPHA = 12;
     PICMTHD_INSERT = 13;
     PICMTHD_FLIPX = 14;
     PICMTHD_FLIPY = 15;
     PICMTHD_CHECKAUTODITHER = 16;
     PICMTHD_NEGATIVE = 17;
     PICMTHD_AUTOCROP = 18;
     PICMTHD_CONVOLVE = 19;
  {
        hook message types
    }
     GGFX_MSGTYPE_LINEDRAWN = 1;
  {
        picture locking
    }
     LOCKMODE_DRAWHANDLE = 1;
     LOCKMODE_FORCE = 1 shl 8;
     LOCKMODE_MASK = $ff;

  {
        bitmap attributes
        (strictly internal)
    }

  const
     BMAPATTR_Width = 0 + TAG_USER;
     BMAPATTR_Height = 1 + TAG_USER;
     BMAPATTR_Depth = 2 + TAG_USER;
     BMAPATTR_CyberGFX = 3 + TAG_USER;
     BMAPATTR_BitMapFormat = 4 + TAG_USER;
     BMAPATTR_PixelFormat = 5 + TAG_USER;
     BMAPATTR_Flags = 6 + TAG_USER;


FUNCTION AddPaletteA(psm : POINTER; palette : POINTER; tags : pTagItem) : POINTER;
FUNCTION AddPictureA(psm : POINTER; pic : POINTER; tags : pTagItem) : POINTER;
FUNCTION AddPixelArrayA(psm : POINTER; _array : POINTER; width : WORD; height : WORD; tags : pTagItem) : POINTER;
FUNCTION ClonePictureA(pic : POINTER; tags : pTagItem) : POINTER;
FUNCTION CreateDirectDrawHandleA(drawhandle : POINTER; sw : WORD; sh : WORD; dw : WORD; dh : WORD; tags : pTagItem) : POINTER;
FUNCTION CreatePenShareMapA(tags : pTagItem) : POINTER;
FUNCTION CreatePictureBitMapA(drawhandle : POINTER; pic : POINTER; tags : pTagItem) : pBitMap;
FUNCTION CreatePictureMaskA(pic : POINTER; mask : pCHAR; maskwidth : WORD; tags : pTagItem) : BOOLEAN;
PROCEDURE DeleteDirectDrawHandle(ddh : POINTER);
PROCEDURE DeletePenShareMap(psm : POINTER);
PROCEDURE DeletePicture(pic : POINTER);
FUNCTION DirectDrawTrueColorA(ddh : POINTER; _array : pULONG; x : WORD; y : WORD; tags : pTagItem) : BOOLEAN;
FUNCTION DoPictureMethodA(pic : POINTER; method : longword; arguments : pULONG) : longword;
FUNCTION DrawPictureA(drawhandle : POINTER; pic : POINTER; x : WORD; y : WORD; tags : pTagItem) : BOOLEAN;
FUNCTION GetPictureAttrsA(pic : POINTER; tags : pTagItem) : longword;
FUNCTION IsPictureA(filename : pCHAR; tags : pTagItem) : BOOLEAN;
FUNCTION LoadPictureA(filename : pCHAR; tags : pTagItem) : POINTER;
FUNCTION LockPictureA(pic : POINTER; mode : longword; args : pULONG) : longword;
FUNCTION MakePictureA(_array : POINTER; width : WORD; height : WORD; tags : pTagItem) : POINTER;
FUNCTION MapPaletteA(drawhandle : POINTER; palette : POINTER; pentab : pCHAR; tags : pTagItem) : BOOLEAN;
FUNCTION MapPenA(drawhandle : POINTER; rgb : longword; tags : pTagItem) : LONGINT;
FUNCTION ObtainDrawHandleA(psm : POINTER; a1arg : pRastPort; cm : pColorMap; tags : pTagItem) : POINTER;
FUNCTION ReadPictureA(a0arg : pRastPort; colormap : pColorMap; x : WORD; y : WORD; width : WORD; height : WORD; tags : pTagItem) : POINTER;
PROCEDURE ReleaseDrawHandle(drawhandle : POINTER);
PROCEDURE RemColorHandle(colorhandle : POINTER);
PROCEDURE UnLockPicture(pic : POINTER; mode : longword);
{
 Functions and procedures with array of const go here
}
FUNCTION AddPalette(psm : POINTER; palette : POINTER; const tags : Array Of Const) : POINTER;
FUNCTION AddPicture(psm : POINTER; pic : POINTER; const tags : Array Of Const) : POINTER;
FUNCTION AddPixelArray(psm : POINTER; _array : POINTER; width : WORD; height : WORD; const tags : Array Of Const) : POINTER;
FUNCTION ClonePicture(pic : POINTER; const tags : Array Of Const) : POINTER;
FUNCTION CreateDirectDrawHandle(drawhandle : POINTER; sw : WORD; sh : WORD; dw : WORD; dh : WORD; const tags : Array Of Const) : POINTER;
FUNCTION CreatePenShareMap(const tags : Array Of Const) : POINTER;
FUNCTION CreatePictureBitMap(drawhandle : POINTER; pic : POINTER; const tags : Array Of Const) : pBitMap;
FUNCTION CreatePictureMask(pic : POINTER; mask : pCHAR; maskwidth : WORD; const tags : Array Of Const) : BOOLEAN;
FUNCTION DirectDrawTrueColor(ddh : POINTER; _array : pULONG; x : WORD; y : WORD; const tags : Array Of Const) : BOOLEAN;
FUNCTION DoPictureMethod(pic : POINTER; method : longword; const arguments : Array Of Const) : longword;
FUNCTION DrawPicture(drawhandle : POINTER; pic : POINTER; x : WORD; y : WORD; const tags : Array Of Const) : BOOLEAN;
FUNCTION GetPictureAttrs(pic : POINTER; const tags : Array Of Const) : longword;
FUNCTION IsPicture(filename : pCHAR; const tags : Array Of Const) : BOOLEAN;
FUNCTION LoadPicture(filename : pCHAR; const tags : Array Of Const) : POINTER;
FUNCTION LockPicture(pic : POINTER; mode : longword; const args : Array Of Const) : longword;
FUNCTION MakePicture(_array : POINTER; width : WORD; height : WORD; const tags : Array Of Const) : POINTER;
FUNCTION MapPalette(drawhandle : POINTER; palette : POINTER; pentab : pCHAR; const tags : Array Of Const) : BOOLEAN;
FUNCTION MapPen(drawhandle : POINTER; rgb : longword; const tags : Array Of Const) : LONGINT;
FUNCTION ObtainDrawHandle(psm : POINTER; a1arg : pRastPort; cm : pColorMap; const tags : Array Of Const) : POINTER;
FUNCTION ReadPicture(a0arg : pRastPort; colormap : pColorMap; x : WORD; y : WORD; width : WORD; height : WORD; const tags : Array Of Const) : POINTER;

{You can remove this include and use a define instead}
{$I useautoopenlib.inc}
{$ifdef use_init_openlib}
procedure InitGUIGFXLibrary;
{$endif use_init_openlib}

{This is a variable that knows how the unit is compiled}
var
    GUIGFXIsCompiledHow : longint;

IMPLEMENTATION

uses
{$ifndef dont_use_openlib}
msgbox,
{$endif dont_use_openlib}
tagsarray,longarray;

FUNCTION AddPaletteA(psm : POINTER; palette : POINTER; tags : pTagItem) : POINTER;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L psm,A0
        MOVEA.L palette,A1
        MOVEA.L tags,A2
        MOVEA.L GuiGFXBase,A6
        JSR     -072(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AddPictureA(psm : POINTER; pic : POINTER; tags : pTagItem) : POINTER;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L psm,A0
        MOVEA.L pic,A1
        MOVEA.L tags,A2
        MOVEA.L GuiGFXBase,A6
        JSR     -066(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AddPixelArrayA(psm : POINTER; _array : POINTER; width : WORD; height : WORD; tags : pTagItem) : POINTER;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L psm,A0
        MOVEA.L _array,A1
        MOVE.L  width,D0
        MOVE.L  height,D1
        MOVEA.L tags,A2
        MOVEA.L GuiGFXBase,A6
        JSR     -078(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION ClonePictureA(pic : POINTER; tags : pTagItem) : POINTER;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L pic,A0
        MOVEA.L tags,A1
        MOVEA.L GuiGFXBase,A6
        JSR     -048(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION CreateDirectDrawHandleA(drawhandle : POINTER; sw : WORD; sh : WORD; dw : WORD; dh : WORD; tags : pTagItem) : POINTER;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L drawhandle,A0
        MOVE.L  sw,D0
        MOVE.L  sh,D1
        MOVE.L  dw,D2
        MOVE.L  dh,D3
        MOVEA.L tags,A1
        MOVEA.L GuiGFXBase,A6
        JSR     -168(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION CreatePenShareMapA(tags : pTagItem) : POINTER;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L tags,A0
        MOVEA.L GuiGFXBase,A6
        JSR     -090(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION CreatePictureBitMapA(drawhandle : POINTER; pic : POINTER; tags : pTagItem) : pBitMap;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L drawhandle,A0
        MOVEA.L pic,A1
        MOVEA.L tags,A2
        MOVEA.L GuiGFXBase,A6
        JSR     -132(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION CreatePictureMaskA(pic : POINTER; mask : pCHAR; maskwidth : WORD; tags : pTagItem) : BOOLEAN;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L pic,A0
        MOVEA.L mask,A1
        MOVE.L  maskwidth,D0
        MOVEA.L tags,A2
        MOVEA.L GuiGFXBase,A6
        JSR     -186(A6)
        MOVEA.L (A7)+,A6
        TST.W   D0
        BEQ.B   @end
        MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

PROCEDURE DeleteDirectDrawHandle(ddh : POINTER);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L ddh,A0
        MOVEA.L GuiGFXBase,A6
        JSR     -174(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE DeletePenShareMap(psm : POINTER);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L psm,A0
        MOVEA.L GuiGFXBase,A6
        JSR     -096(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE DeletePicture(pic : POINTER);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L pic,A0
        MOVEA.L GuiGFXBase,A6
        JSR     -054(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION DirectDrawTrueColorA(ddh : POINTER; _array : pULONG; x : WORD; y : WORD; tags : pTagItem) : BOOLEAN;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L ddh,A0
        MOVEA.L _array,A1
        MOVE.L  x,D0
        MOVE.L  y,D1
        MOVEA.L tags,A2
        MOVEA.L GuiGFXBase,A6
        JSR     -180(A6)
        MOVEA.L (A7)+,A6
        TST.W   D0
        BEQ.B   @end
        MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION DoPictureMethodA(pic : POINTER; method : longword; arguments : pULONG) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L pic,A0
        MOVE.L  method,D0
        MOVEA.L arguments,A1
        MOVEA.L GuiGFXBase,A6
        JSR     -138(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION DrawPictureA(drawhandle : POINTER; pic : POINTER; x : WORD; y : WORD; tags : pTagItem) : BOOLEAN;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L drawhandle,A0
        MOVEA.L pic,A1
        MOVE.L  x,D0
        MOVE.L  y,D1
        MOVEA.L tags,A2
        MOVEA.L GuiGFXBase,A6
        JSR     -114(A6)
        MOVEA.L (A7)+,A6
        TST.W   D0
        BEQ.B   @end
        MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION GetPictureAttrsA(pic : POINTER; tags : pTagItem) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L pic,A0
        MOVEA.L tags,A1
        MOVEA.L GuiGFXBase,A6
        JSR     -144(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION IsPictureA(filename : pCHAR; tags : pTagItem) : BOOLEAN;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L filename,A0
        MOVEA.L tags,A1
        MOVEA.L GuiGFXBase,A6
        JSR     -162(A6)
        MOVEA.L (A7)+,A6
        TST.W   D0
        BEQ.B   @end
        MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION LoadPictureA(filename : pCHAR; tags : pTagItem) : POINTER;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L filename,A0
        MOVEA.L tags,A1
        MOVEA.L GuiGFXBase,A6
        JSR     -036(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION LockPictureA(pic : POINTER; mode : longword; args : pULONG) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L pic,A0
        MOVE.L  mode,D0
        MOVEA.L args,A1
        MOVEA.L GuiGFXBase,A6
        JSR     -150(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION MakePictureA(_array : POINTER; width : WORD; height : WORD; tags : pTagItem) : POINTER;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L _array,A0
        MOVE.L  width,D0
        MOVE.L  height,D1
        MOVEA.L tags,A1
        MOVEA.L GuiGFXBase,A6
        JSR     -030(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION MapPaletteA(drawhandle : POINTER; palette : POINTER; pentab : pCHAR; tags : pTagItem) : BOOLEAN;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L drawhandle,A0
        MOVEA.L palette,A1
        MOVEA.L pentab,A2
        MOVEA.L tags,A3
        MOVEA.L GuiGFXBase,A6
        JSR     -120(A6)
        MOVEA.L (A7)+,A6
        TST.W   D0
        BEQ.B   @end
        MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION MapPenA(drawhandle : POINTER; rgb : longword; tags : pTagItem) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L drawhandle,A0
        MOVEA.L rgb,A1
        MOVEA.L tags,A2
        MOVEA.L GuiGFXBase,A6
        JSR     -126(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION ObtainDrawHandleA(psm : POINTER; a1arg : pRastPort; cm : pColorMap; tags : pTagItem) : POINTER;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L psm,A0
        MOVEA.L a1arg,A1
        MOVEA.L cm,A2
        MOVEA.L tags,A3
        MOVEA.L GuiGFXBase,A6
        JSR     -102(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION ReadPictureA(a0arg : pRastPort; colormap : pColorMap; x : WORD; y : WORD; width : WORD; height : WORD; tags : pTagItem) : POINTER;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L a0arg,A0
        MOVEA.L colormap,A1
        MOVE.L  x,D0
        MOVE.L  y,D1
        MOVE.L  width,D2
        MOVE.L  height,D3
        MOVEA.L tags,A2
        MOVEA.L GuiGFXBase,A6
        JSR     -042(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE ReleaseDrawHandle(drawhandle : POINTER);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L drawhandle,A0
        MOVEA.L GuiGFXBase,A6
        JSR     -108(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE RemColorHandle(colorhandle : POINTER);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L colorhandle,A0
        MOVEA.L GuiGFXBase,A6
        JSR     -084(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE UnLockPicture(pic : POINTER; mode : longword);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L pic,A0
        MOVE.L  mode,D0
        MOVEA.L GuiGFXBase,A6
        JSR     -156(A6)
        MOVEA.L (A7)+,A6
  END;
END;

{
 Functions and procedures with array of const go here
}
FUNCTION AddPalette(psm : POINTER; palette : POINTER; const tags : Array Of Const) : POINTER;
begin
    AddPalette := AddPaletteA(psm , palette , readintags(tags));
end;

FUNCTION AddPicture(psm : POINTER; pic : POINTER; const tags : Array Of Const) : POINTER;
begin
    AddPicture := AddPictureA(psm , pic , readintags(tags));
end;

FUNCTION AddPixelArray(psm : POINTER; _array : POINTER; width : WORD; height : WORD; const tags : Array Of Const) : POINTER;
begin
    AddPixelArray := AddPixelArrayA(psm , _array , width , height , readintags(tags));
end;

FUNCTION ClonePicture(pic : POINTER; const tags : Array Of Const) : POINTER;
begin
    ClonePicture := ClonePictureA(pic , readintags(tags));
end;

FUNCTION CreateDirectDrawHandle(drawhandle : POINTER; sw : WORD; sh : WORD; dw : WORD; dh : WORD; const tags : Array Of Const) : POINTER;
begin
    CreateDirectDrawHandle := CreateDirectDrawHandleA(drawhandle , sw , sh , dw , dh , readintags(tags));
end;

FUNCTION CreatePenShareMap(const tags : Array Of Const) : POINTER;
begin
    CreatePenShareMap := CreatePenShareMapA(readintags(tags));
end;

FUNCTION CreatePictureBitMap(drawhandle : POINTER; pic : POINTER; const tags : Array Of Const) : pBitMap;
begin
    CreatePictureBitMap := CreatePictureBitMapA(drawhandle , pic , readintags(tags));
end;

FUNCTION CreatePictureMask(pic : POINTER; mask : pCHAR; maskwidth : WORD; const tags : Array Of Const) : BOOLEAN;
begin
    CreatePictureMask := CreatePictureMaskA(pic , mask , maskwidth , readintags(tags));
end;

FUNCTION DirectDrawTrueColor(ddh : POINTER; _array : pULONG; x : WORD; y : WORD; const tags : Array Of Const) : BOOLEAN;
begin
    DirectDrawTrueColor := DirectDrawTrueColorA(ddh , _array , x , y , readintags(tags));
end;

FUNCTION DoPictureMethod(pic : POINTER; method : longword; const arguments : Array Of Const) : longword;
begin
    DoPictureMethod := DoPictureMethodA(pic , method , readinlongs(arguments));
end;

FUNCTION DrawPicture(drawhandle : POINTER; pic : POINTER; x : WORD; y : WORD; const tags : Array Of Const) : BOOLEAN;
begin
    DrawPicture := DrawPictureA(drawhandle , pic , x , y , readintags(tags));
end;

FUNCTION GetPictureAttrs(pic : POINTER; const tags : Array Of Const) : longword;
begin
    GetPictureAttrs := GetPictureAttrsA(pic , readintags(tags));
end;

FUNCTION IsPicture(filename : pCHAR; const tags : Array Of Const) : BOOLEAN;
begin
    IsPicture := IsPictureA(filename , readintags(tags));
end;

FUNCTION LoadPicture(filename : pCHAR; const tags : Array Of Const) : POINTER;
begin
    LoadPicture := LoadPictureA(filename , readintags(tags));
end;

FUNCTION LockPicture(pic : POINTER; mode : longword; const args : Array Of Const) : longword;
begin
    LockPicture := LockPictureA(pic , mode , readinlongs(args));
end;

FUNCTION MakePicture(_array : POINTER; width : WORD; height : WORD; const tags : Array Of Const) : POINTER;
begin
    MakePicture := MakePictureA(_array , width , height , readintags(tags));
end;

FUNCTION MapPalette(drawhandle : POINTER; palette : POINTER; pentab : pCHAR; const tags : Array Of Const) : BOOLEAN;
begin
    MapPalette := MapPaletteA(drawhandle , palette , pentab , readintags(tags));
end;

FUNCTION MapPen(drawhandle : POINTER; rgb : longword; const tags : Array Of Const) : LONGINT;
begin
    MapPen := MapPenA(drawhandle , rgb , readintags(tags));
end;

FUNCTION ObtainDrawHandle(psm : POINTER; a1arg : pRastPort; cm : pColorMap; const tags : Array Of Const) : POINTER;
begin
    ObtainDrawHandle := ObtainDrawHandleA(psm , a1arg , cm , readintags(tags));
end;

FUNCTION ReadPicture(a0arg : pRastPort; colormap : pColorMap; x : WORD; y : WORD; width : WORD; height : WORD; const tags : Array Of Const) : POINTER;
begin
    ReadPicture := ReadPictureA(a0arg , colormap , x , y , width , height , readintags(tags));
end;

const
    { Change VERSION and LIBVERSION to proper values }

    VERSION : string[2] = '0';
    LIBVERSION : longword = 0;

{$ifdef use_init_openlib}
  {$Info Compiling initopening of guigfx.library}
  {$Info don't forget to use InitGUIGFXLibrary in the beginning of your program}

var
    guigfx_exit : Pointer;

procedure CloseguigfxLibrary;
begin
    ExitProc := guigfx_exit;
    if GuiGFXBase <> nil then begin
        CloseLibrary(GuiGFXBase);
        GuiGFXBase := nil;
    end;
end;

procedure InitGUIGFXLibrary;
begin
    GuiGFXBase := nil;
    GuiGFXBase := OpenLibrary(GUIGFXNAME,LIBVERSION);
    if GuiGFXBase <> nil then begin
        guigfx_exit := ExitProc;
        ExitProc := @CloseguigfxLibrary;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open guigfx.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;
end;

begin
    GUIGFXIsCompiledHow := 2;
{$endif use_init_openlib}

{$ifdef use_auto_openlib}
  {$Info Compiling autoopening of guigfx.library}

var
    guigfx_exit : Pointer;

procedure CloseguigfxLibrary;
begin
    ExitProc := guigfx_exit;
    if GuiGFXBase <> nil then begin
        CloseLibrary(GuiGFXBase);
        GuiGFXBase := nil;
    end;
end;

begin
    GuiGFXBase := nil;
    GuiGFXBase := OpenLibrary(GUIGFXNAME,LIBVERSION);
    if GuiGFXBase <> nil then begin
        guigfx_exit := ExitProc;
        ExitProc := @CloseguigfxLibrary;
        GUIGFXIsCompiledHow := 1;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open guigfx.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;

{$endif use_auto_openlib}

{$ifdef dont_use_openlib}
begin
    GUIGFXIsCompiledHow := 3;
   {$Warning No autoopening of guigfx.library compiled}
   {$Warning Make sure you open guigfx.library yourself}
{$endif dont_use_openlib}

END. (* UNIT GUIGFX *)



