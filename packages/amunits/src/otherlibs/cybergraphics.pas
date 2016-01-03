{
  This file is part of the Free Pascal run time library.

  A file in Amiga system run time library.
  Copyright (c) 2003 by Nils Sjöholm.
  member of the Amiga RTL development team.

  This is a unit for cybergraphics.library

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

UNIT CYBERGRAPHICS;

INTERFACE
USES Exec,graphics,utility;

VAR CyberGfxBase : pLibrary;

const
    CYBERGRAPHICSNAME : PChar = 'cybergraphics.library';

{
        $VER: cybergraphics.h 41.18 (21.02.1998)

        include file for cybergraphics.library

        Copyright © 1996-1998 by phase5 digital products
          All Rights reserved.

   }

  const
     CYBERGFXNAME = 'cybergraphics.library';
     CYBERGFX_INCLUDE_VERSION = 41;
  {
      Definition of CyberModeNode (Returned in AllocModeList)
                                                               }
  type
     PCyberModeNode = ^tCyberModeNode;
     tCyberModeNode = record
          Node : tNode;
          ModeText : array[0..(DISPLAYNAMELEN)-1] of char; { name for this mode  }
          DisplayID : ULONG;                               { display id associated with the node  }
          Width : UWORD;                                   { visible width  }
          Height : UWORD;                                  { visible height  }
          Depth : UWORD;                                   { display depth  }
          DisplayTagList : PTagItem;                       { taglist with extended ModeID information  }
       end;

  {
     Parameters for GetCyberMapAttr()
                                       }
  const
  { function returns BytesPerRow if its called with this parameter  }
     CYBRMATTR_XMOD = $80000001;
  { BytesPerPixel shall be returned  }
     CYBRMATTR_BPPIX = $80000002;
  { do not use this ! private tag  }
     CYBRMATTR_DISPADR = $80000003;
  { the pixel format is returned  }
     CYBRMATTR_PIXFMT = $80000004;
  { returns width in pixels  }
     CYBRMATTR_WIDTH = $80000005;
  { returns height in lines  }
     CYBRMATTR_HEIGHT = $80000006;
  { returns bits per pixel  }
     CYBRMATTR_DEPTH = $80000007;
  { returns -1 if supplied bitmap is a cybergfx one  }
     CYBRMATTR_ISCYBERGFX = $80000008;
  { returns -1 if supplied bitmap is linear accessable  }
     CYBRMATTR_ISLINEARMEM = $80000009;
  {
     Parameters for GetCyberIDAttr()
                                      }
  { the pixel format is returned  }
     CYBRIDATTR_PIXFMT = $80000001;
  { returns visible width in pixels  }
     CYBRIDATTR_WIDTH = $80000002;
  { returns visible height in lines  }
     CYBRIDATTR_HEIGHT = $80000003;
  { returns bits per pixel  }
     CYBRIDATTR_DEPTH = $80000004;
  { BytesPerPixel shall be returned  }
     CYBRIDATTR_BPPIX = $80000005;
  {
     Tags for CyberModeRequest()
                                   }
     CYBRMREQ_TB = TAG_USER + $40000;
  {
     FilterTags
                 }
  { Minimum depth for displayed screenmode  }
     CYBRMREQ_MinDepth = CYBRMREQ_TB + 0;
  { Maximum depth  "       "        "  }
     CYBRMREQ_MaxDepth = CYBRMREQ_TB + 1;
  { Minumum width  "       "        "  }
     CYBRMREQ_MinWidth = CYBRMREQ_TB + 2;
  { Maximum width  "       "        "  }
     CYBRMREQ_MaxWidth = CYBRMREQ_TB + 3;
  { Minumum height "       "        "  }
     CYBRMREQ_MinHeight = CYBRMREQ_TB + 4;
  { Minumum height "       "        "  }
     CYBRMREQ_MaxHeight = CYBRMREQ_TB + 5;
     CYBRMREQ_CModelArray = CYBRMREQ_TB + 6;
     CYBRMREQ_WinTitle = CYBRMREQ_TB + 20;
     CYBRMREQ_OKText = CYBRMREQ_TB + 21;
     CYBRMREQ_CancelText = CYBRMREQ_TB + 22;
  { Screen you wish the Requester to open on  }
     CYBRMREQ_Screen = CYBRMREQ_TB + 30;
  {
     Tags for BestCyberModeID()
                                 }
     CYBRBIDTG_TB = TAG_USER + $50000;
  { FilterTags  }
     CYBRBIDTG_Depth = CYBRBIDTG_TB + 0;
     CYBRBIDTG_NominalWidth = CYBRBIDTG_TB + 1;
     CYBRBIDTG_NominalHeight = CYBRBIDTG_TB + 2;
     CYBRBIDTG_MonitorID = CYBRBIDTG_TB + 3;
     CYBRBIDTG_BoardName = CYBRBIDTG_TB + 5;
  {
     definition of divers pixel formats
                                         }
     PIXFMT_LUT8 = 0;
     PIXFMT_RGB15 = 1;
     PIXFMT_BGR15 = 2;
     PIXFMT_RGB15PC = 3;
     PIXFMT_BGR15PC = 4;
     PIXFMT_RGB16 = 5;
     PIXFMT_BGR16 = 6;
     PIXFMT_RGB16PC = 7;
     PIXFMT_BGR16PC = 8;
     PIXFMT_RGB24 = 9;
     PIXFMT_BGR24 = 10;
     PIXFMT_ARGB32 = 11;
     PIXFMT_BGRA32 = 12;
     PIXFMT_RGBA32 = 13;
  {
     SrcRectangle formats defines for xxxPixelArray calls()
                                                             }
     RECTFMT_RGB = 0;
     RECTFMT_RGBA = 1;
     RECTFMT_ARGB = 2;
     RECTFMT_LUT8 = 3;
     RECTFMT_GREY8 = 4;
  {
     Parameters for CVideoCtrlTagList()
                                         }
     SETVC_DPMSLevel = $88002001;
  { Full operation                              }
     DPMS_ON = 0;
  { Optional state of minimal power reduction   }
     DPMS_STANDBY = 1;
  { Significant reduction of power consumption  }
     DPMS_SUSPEND = 2;
  { Lowest level of power consumption           }
     DPMS_OFF = 3;
  {
     Tags for LockBitMapTagList()
                                   }
     LBMI_WIDTH = $84001001;
     LBMI_HEIGHT = $84001002;
     LBMI_DEPTH = $84001003;
     LBMI_PIXFMT = $84001004;
     LBMI_BYTESPERPIX = $84001005;
     LBMI_BYTESPERROW = $84001006;
     LBMI_BASEADDRESS = $84001007;
  {
     Tags for UnLockBitMapTagList()
                                     }
     UBMI_UPDATERECTS = $85001001;
     UBMI_REALLYUNLOCK = $85001002;
  {
     Message passed to the DoCDrawMethodTagList() hook function
                                                                 }

  type
     PCDrawMsg = ^tCDrawMsg;
     tCDrawMsg = record
          cdm_MemPtr : APTR;
          cdm_offx : ULONG;
          cdm_offy : ULONG;
          cdm_xsize : ULONG;
          cdm_ysize : ULONG;
          cdm_BytesPerRow : UWORD;
          cdm_BytesPerPix : UWORD;
          cdm_ColorModel : UWORD;
       end;

  {
     Colour Table source formats for WriteLUTPixelArray()
                                                         }
  { ULONG [] table  }

  const
     CTABFMT_XRGB8 = 0;
  {
        graphics.library/AllocBitMap() extended flags
                                                         }
     BMB_SPECIALFMT = 7;
     BMF_SPECIALFMT = 1 shl BMB_SPECIALFMT;

FUNCTION AllocCModeListTagList(ModeListTags : pTagItem) : pList;
FUNCTION BestCModeIDTagList(BestModeIDTags : pTagItem) : longword;
FUNCTION CModeRequestTagList(ModeRequest : POINTER; ModeRequestTags : pTagItem) : longword;
PROCEDURE CVideoCtrlTagList(ViewPort : pViewPort; TagList : pTagItem);
PROCEDURE DoCDrawMethodTagList(Hook : pHook; a1arg : pRastPort; TagList : pTagItem);
FUNCTION ExtractColor(a0arg : pRastPort; BitMap : pBitMap; Colour : longword; SrcX : longword; SrcY : longword; Width : longword; Height : longword) : longword;
FUNCTION FillPixelArray(a1arg : pRastPort; DestX : WORD; DestY : WORD; SizeX : WORD; SizeY : WORD; ARGB : longword) : longword;
PROCEDURE FreeCModeList(ModeList : pList);
FUNCTION GetCyberIDAttr(CyberIDAttr : longword; CyberDisplayModeID : longword) : longword;
FUNCTION GetCyberMapAttr(CyberGfxBitmap : pBitMap; CyberAttrTag : longword) : longword;
FUNCTION InvertPixelArray(a1arg : pRastPort; DestX : WORD; DestY : WORD; SizeX : WORD; SizeY : WORD) : longword;
FUNCTION IsCyberModeID(displayID : longword) : BOOLEAN;
FUNCTION LockBitMapTagList(BitMap : POINTER; TagList : pTagItem) : POINTER;
FUNCTION MovePixelArray(SrcX : WORD; SrcY : WORD; a1arg : pRastPort; DestX : WORD; DestY : WORD; SizeX : WORD; SizeY : WORD) : longword;
FUNCTION ReadPixelArray(destRect : POINTER; destX : WORD; destY : WORD; destMod : WORD; a1arg : pRastPort; SrcX : WORD; SrcY : WORD; SizeX : WORD; SizeY : WORD; DestFormat : byte) : longword;
FUNCTION ReadRGBPixel(a1arg : pRastPort; x : WORD; y : WORD) : longword;
FUNCTION ScalePixelArray(srcRect : POINTER; SrcW : WORD; SrcH : WORD; SrcMod : WORD; a1arg : pRastPort; DestX : WORD; DestY : WORD; DestW : WORD; DestH : WORD; SrcFormat : byte) : LONGINT;
PROCEDURE UnLockBitMap(Handle : POINTER);
PROCEDURE UnLockBitMapTagList(Handle : POINTER; TagList : pTagItem);
FUNCTION WriteLUTPixelArray(srcRect : POINTER; SrcX : WORD; SrcY : WORD; SrcMod : WORD; a1arg : pRastPort; ColorTab : POINTER; DestX : WORD; DestY : WORD; SizeX : WORD; SizeY : WORD; CTFormat : byte) : longword;
FUNCTION WritePixelArray(srcRect : POINTER; SrcX : WORD; SrcY : WORD; SrcMod : WORD; a1arg : pRastPort; DestX : WORD; DestY : WORD; SizeX : WORD; SizeY : WORD; SrcFormat : byte) : longword;
FUNCTION WriteRGBPixel(a1arg : pRastPort; x : WORD; y : WORD; argb : longword) : LONGINT;
{
 Functions and procedures with array of const go here
}
FUNCTION AllocCModeListTags(const ModeListTags : Array Of Const) : pList;
FUNCTION BestCModeIDTags(const BestModeIDTags : Array Of Const) : longword;
FUNCTION CModeRequestTags(ModeRequest : POINTER; const ModeRequestTags : Array Of Const) : longword;
PROCEDURE CVideoCtrlTags(ViewPort : pViewPort; const TagList : Array Of Const);
PROCEDURE DoCDrawMethodTags(Hook : pHook; a1arg : pRastPort; const TagList : Array Of Const);
FUNCTION LockBitMapTags(BitMap : POINTER; const TagList : Array Of Const) : POINTER;
PROCEDURE UnLockBitMapTags(Handle : POINTER; const TagList : Array Of Const);

function SHIFT_PIXFMT(fmt : longint) : longint;

{You can remove this include and use a define instead}
{$I useautoopenlib.inc}
{$ifdef use_init_openlib}
procedure InitCYBERGRAPHICSLibrary;
{$endif use_init_openlib}

{This is a variable that knows how the unit is compiled}
var
    CYBERGRAPHICSIsCompiledHow : longint;

IMPLEMENTATION

uses
{$ifndef dont_use_openlib}
msgbox,
{$endif dont_use_openlib}
tagsarray;

FUNCTION AllocCModeListTagList(ModeListTags : pTagItem) : pList;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L ModeListTags,A1
        MOVEA.L CyberGfxBase,A6
        JSR     -072(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION BestCModeIDTagList(BestModeIDTags : pTagItem) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L BestModeIDTags,A0
        MOVEA.L CyberGfxBase,A6
        JSR     -060(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION CModeRequestTagList(ModeRequest : POINTER; ModeRequestTags : pTagItem) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L ModeRequest,A0
        MOVEA.L ModeRequestTags,A1
        MOVEA.L CyberGfxBase,A6
        JSR     -066(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE CVideoCtrlTagList(ViewPort : pViewPort; TagList : pTagItem);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L ViewPort,A0
        MOVEA.L TagList,A1
        MOVEA.L CyberGfxBase,A6
        JSR     -162(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE DoCDrawMethodTagList(Hook : pHook; a1arg : pRastPort; TagList : pTagItem);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L Hook,A0
        MOVEA.L a1arg,A1
        MOVEA.L TagList,A2
        MOVEA.L CyberGfxBase,A6
        JSR     -156(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION ExtractColor(a0arg : pRastPort; BitMap : pBitMap; Colour : longword; SrcX : longword; SrcY : longword; Width : longword; Height : longword) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L a0arg,A0
        MOVEA.L BitMap,A1
        MOVE.L  Colour,D0
        MOVE.L  SrcX,D1
        MOVE.L  SrcY,D2
        MOVE.L  Width,D3
        MOVE.L  Height,D4
        MOVEA.L CyberGfxBase,A6
        JSR     -186(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION FillPixelArray(a1arg : pRastPort; DestX : WORD; DestY : WORD; SizeX : WORD; SizeY : WORD; ARGB : longword) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L a1arg,A1
        MOVE.L  DestX,D0
        MOVE.L  DestY,D1
        MOVE.L  SizeX,D2
        MOVE.L  SizeY,D3
        MOVE.L  ARGB,D4
        MOVEA.L CyberGfxBase,A6
        JSR     -150(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE FreeCModeList(ModeList : pList);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L ModeList,A0
        MOVEA.L CyberGfxBase,A6
        JSR     -078(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION GetCyberIDAttr(CyberIDAttr : longword; CyberDisplayModeID : longword) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  CyberIDAttr,D0
        MOVE.L  CyberDisplayModeID,D1
        MOVEA.L CyberGfxBase,A6
        JSR     -102(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION GetCyberMapAttr(CyberGfxBitmap : pBitMap; CyberAttrTag : longword) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L CyberGfxBitmap,A0
        MOVE.L  CyberAttrTag,D0
        MOVEA.L CyberGfxBase,A6
        JSR     -096(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION InvertPixelArray(a1arg : pRastPort; DestX : WORD; DestY : WORD; SizeX : WORD; SizeY : WORD) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L a1arg,A1
        MOVE.L  DestX,D0
        MOVE.L  DestY,D1
        MOVE.L  SizeX,D2
        MOVE.L  SizeY,D3
        MOVEA.L CyberGfxBase,A6
        JSR     -144(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION IsCyberModeID(displayID : longword) : BOOLEAN;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  displayID,D0
        MOVEA.L CyberGfxBase,A6
        JSR     -054(A6)
        MOVEA.L (A7)+,A6
        TST.W   D0
        BEQ.B   @end
        MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION LockBitMapTagList(BitMap : POINTER; TagList : pTagItem) : POINTER;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L BitMap,A0
        MOVEA.L TagList,A1
        MOVEA.L CyberGfxBase,A6
        JSR     -168(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION MovePixelArray(SrcX : WORD; SrcY : WORD; a1arg : pRastPort; DestX : WORD; DestY : WORD; SizeX : WORD; SizeY : WORD) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  SrcX,D0
        MOVE.L  SrcY,D1
        MOVEA.L a1arg,A1
        MOVE.L  DestX,D2
        MOVE.L  DestY,D3
        MOVE.L  SizeX,D4
        MOVE.L  SizeY,D5
        MOVEA.L CyberGfxBase,A6
        JSR     -132(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION ReadPixelArray(destRect : POINTER; destX : WORD; destY : WORD; destMod : WORD; a1arg : pRastPort; SrcX : WORD; SrcY : WORD; SizeX : WORD; SizeY : WORD; DestFormat : byte) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L destRect,A0
        MOVE.L  destX,D0
        MOVE.L  destY,D1
        MOVE.L  destMod,D2
        MOVEA.L a1arg,A1
        MOVE.L  SrcX,D3
        MOVE.L  SrcY,D4
        MOVE.L  SizeX,D5
        MOVE.L  SizeY,D6
        MOVE.L  DestFormat,D7
        MOVEA.L CyberGfxBase,A6
        JSR     -120(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION ReadRGBPixel(a1arg : pRastPort; x : WORD; y : WORD) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L a1arg,A1
        MOVE.L  x,D0
        MOVE.L  y,D1
        MOVEA.L CyberGfxBase,A6
        JSR     -108(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION ScalePixelArray(srcRect : POINTER; SrcW : WORD; SrcH : WORD; SrcMod : WORD; a1arg : pRastPort; DestX : WORD; DestY : WORD; DestW : WORD; DestH : WORD; SrcFormat : byte) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L srcRect,A0
        MOVE.L  SrcW,D0
        MOVE.L  SrcH,D1
        MOVE.L  SrcMod,D2
        MOVEA.L a1arg,A1
        MOVE.L  DestX,D3
        MOVE.L  DestY,D4
        MOVE.L  DestW,D5
        MOVE.L  DestH,D6
        MOVE.L  SrcFormat,D7
        MOVEA.L CyberGfxBase,A6
        JSR     -090(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE UnLockBitMap(Handle : POINTER);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L Handle,A0
        MOVEA.L CyberGfxBase,A6
        JSR     -174(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE UnLockBitMapTagList(Handle : POINTER; TagList : pTagItem);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L Handle,A0
        MOVEA.L TagList,A1
        MOVEA.L CyberGfxBase,A6
        JSR     -180(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION WriteLUTPixelArray(srcRect : POINTER; SrcX : WORD; SrcY : WORD; SrcMod : WORD; a1arg : pRastPort; ColorTab : POINTER; DestX : WORD; DestY : WORD; SizeX : WORD; SizeY : WORD; CTFormat : byte) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L srcRect,A0
        MOVE.L  SrcX,D0
        MOVE.L  SrcY,D1
        MOVE.L  SrcMod,D2
        MOVEA.L a1arg,A1
        MOVEA.L ColorTab,A2
        MOVE.L  DestX,D3
        MOVE.L  DestY,D4
        MOVE.L  SizeX,D5
        MOVE.L  SizeY,D6
        MOVE.L  CTFormat,D7
        MOVEA.L CyberGfxBase,A6
        JSR     -198(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION WritePixelArray(srcRect : POINTER; SrcX : WORD; SrcY : WORD; SrcMod : WORD; a1arg : pRastPort; DestX : WORD; DestY : WORD; SizeX : WORD; SizeY : WORD; SrcFormat : byte) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L srcRect,A0
        MOVE.L  SrcX,D0
        MOVE.L  SrcY,D1
        MOVE.L  SrcMod,D2
        MOVEA.L a1arg,A1
        MOVE.L  DestX,D3
        MOVE.L  DestY,D4
        MOVE.L  SizeX,D5
        MOVE.L  SizeY,D6
        MOVE.L  SrcFormat,D7
        MOVEA.L CyberGfxBase,A6
        JSR     -126(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION WriteRGBPixel(a1arg : pRastPort; x : WORD; y : WORD; argb : longword) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L a1arg,A1
        MOVE.L  x,D0
        MOVE.L  y,D1
        MOVE.L  argb,D2
        MOVEA.L CyberGfxBase,A6
        JSR     -114(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

{
 Functions and procedures with array of const go here
}
FUNCTION AllocCModeListTags(const ModeListTags : Array Of Const) : pList;
begin
    AllocCModeListTags := AllocCModeListTagList(readintags(ModeListTags));
end;

FUNCTION BestCModeIDTags(const BestModeIDTags : Array Of Const) : longword;
begin
    BestCModeIDTags := BestCModeIDTagList(readintags(BestModeIDTags));
end;

FUNCTION CModeRequestTags(ModeRequest : POINTER; const ModeRequestTags : Array Of Const) : longword;
begin
    CModeRequestTags := CModeRequestTagList(ModeRequest , readintags(ModeRequestTags));
end;

PROCEDURE CVideoCtrlTags(ViewPort : pViewPort; const TagList : Array Of Const);
begin
    CVideoCtrlTagList(ViewPort , readintags(TagList));
end;

PROCEDURE DoCDrawMethodTags(Hook : pHook; a1arg : pRastPort; const TagList : Array Of Const);
begin
    DoCDrawMethodTagList(Hook , a1arg , readintags(TagList));
end;

FUNCTION LockBitMapTags(BitMap : POINTER; const TagList : Array Of Const) : POINTER;
begin
    LockBitMapTags := LockBitMapTagList(BitMap , readintags(TagList));
end;

PROCEDURE UnLockBitMapTags(Handle : POINTER; const TagList : Array Of Const);
begin
    UnLockBitMapTagList(Handle , readintags(TagList));
end;

function SHIFT_PIXFMT(fmt : longint) : longint;
begin
    SHIFT_PIXFMT:=(ULONG(fmt)) shl 24;
end;


const
    { Change VERSION and LIBVERSION to proper values }

    VERSION : string[2] = '0';
    LIBVERSION : longword = 0;

{$ifdef use_init_openlib}
  {$Info Compiling initopening of cybergraphics.library}
  {$Info don't forget to use InitCYBERGRAPHICSLibrary in the beginning of your program}

var
    cybergraphics_exit : Pointer;

procedure ClosecybergraphicsLibrary;
begin
    ExitProc := cybergraphics_exit;
    if CyberGfxBase <> nil then begin
        CloseLibrary(CyberGfxBase);
        CyberGfxBase := nil;
    end;
end;

procedure InitCYBERGRAPHICSLibrary;
begin
    CyberGfxBase := nil;
    CyberGfxBase := OpenLibrary(CYBERGRAPHICSNAME,LIBVERSION);
    if CyberGfxBase <> nil then begin
        cybergraphics_exit := ExitProc;
        ExitProc := @ClosecybergraphicsLibrary;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open cybergraphics.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;
end;

begin
    CYBERGRAPHICSIsCompiledHow := 2;
{$endif use_init_openlib}

{$ifdef use_auto_openlib}
  {$Info Compiling autoopening of cybergraphics.library}

var
    cybergraphics_exit : Pointer;

procedure ClosecybergraphicsLibrary;
begin
    ExitProc := cybergraphics_exit;
    if CyberGfxBase <> nil then begin
        CloseLibrary(CyberGfxBase);
        CyberGfxBase := nil;
    end;
end;

begin
    CyberGfxBase := nil;
    CyberGfxBase := OpenLibrary(CYBERGRAPHICSNAME,LIBVERSION);
    if CyberGfxBase <> nil then begin
        cybergraphics_exit := ExitProc;
        ExitProc := @ClosecybergraphicsLibrary;
        CYBERGRAPHICSIsCompiledHow := 1;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open cybergraphics.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;

{$endif use_auto_openlib}

{$ifdef dont_use_openlib}
begin
    CYBERGRAPHICSIsCompiledHow := 3;
   {$Warning No autoopening of cybergraphics.library compiled}
   {$Warning Make sure you open cybergraphics.library yourself}
{$endif dont_use_openlib}


END. (* UNIT CYBERGRAPHICS *)



