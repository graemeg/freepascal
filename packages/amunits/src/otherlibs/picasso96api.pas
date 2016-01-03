{
    This file is part of the Free Pascal run time library.

    A file in Amiga system run time library.
    Copyright (c) 2001-2003 by Nils Sjoholm
    member of the Amiga RTL development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

 {
    History:

    First version of unit Picasso96Api
    27 Feb. 2001.

    Updated to fpc 1.0.7
    08 Jan 2003

    Added the defines use_amiga_smartlink and
    use_auto_openlib.
    12 Jan 2003.

    Changed cardinal > longword.
    Changed startcode for unit.
    11 Feb 2003.

    nils.sjoholm@mailbox.swipnet.se

}
{$mode objfpc}
{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
    {$smartlink on}
{$endif use_amiga_smartlink}

UNIT PICASSO96API;

INTERFACE
USES Exec, utility, graphics, intuition;

  {  Picasso96.h -- include File
      (C) Copyright 1996-98 Alexander Kneer & Tobias Abt
      All Rights Reserved.
    }

 const
       PICASSO96APINAME  : PChar = 'Picasso96API.library';
{************************************************************************}
{ Types for RGBFormat used
 }

  type

     RGBFTYPE = (
       RGBFB_NONE,
       RGBFB_CLUT,
       RGBFB_R8G8B8,
       RGBFB_B8G8R8,
       RGBFB_R5G6B5PC,
       RGBFB_R5G5B5PC,
       RGBFB_A8R8G8B8,
       RGBFB_A8B8G8R8,
       RGBFB_R8G8B8A8,
       RGBFB_B8G8R8A8,
       RGBFB_R5G6B5,
       RGBFB_R5G5B5,
       RGBFB_B5G6R5PC,
       RGBFB_B5G5R5PC,
       RGBFB_Y4U2V2,
       RGBFB_Y4U1V1,
       RGBFB_MaxFormats);



  const
     RGBFF_NONE = 1 shl 0;
     RGBFF_CLUT = 1 shl 1;
     RGBFF_R8G8B8 = 1 shl 2;
     RGBFF_B8G8R8 = 1 shl 3;
     RGBFF_R5G6B5PC = 1 shl 4;
     RGBFF_R5G5B5PC = 1 shl 5;
     RGBFF_A8R8G8B8 = 1 shl 6;
     RGBFF_A8B8G8R8 = 1 shl 7;
     RGBFF_R8G8B8A8 = 1 shl 8;
     RGBFF_B8G8R8A8 = 1 shl 9;
     RGBFF_R5G6B5 = 1 shl 10;
     RGBFF_R5G5B5 = 1 shl 11;
     RGBFF_B5G6R5PC = 1 shl 12;
     RGBFF_B5G5R5PC = 1 shl 13;
     RGBFF_Y4U2V2 = 1 shl 14;
     RGBFF_Y4U1V1 = 1 shl 15;
     RGBFF_HICOLOR = ((((RGBFF_R5G6B5PC or RGBFF_R5G5B5PC) or RGBFF_R5G6B5) or RGBFF_R5G5B5) or RGBFF_B5G6R5PC) or RGBFF_B5G5R5PC;
     RGBFF_TRUECOLOR = RGBFF_R8G8B8 or RGBFF_B8G8R8;
     RGBFF_TRUEALPHA = ((RGBFF_A8R8G8B8 or RGBFF_A8B8G8R8) or RGBFF_R8G8B8A8) or RGBFF_B8G8R8A8;
    {                                                                       }
    { Flags for p96AllocBitMap
    }
    BMF_USERPRIVATE = $8000;
    { private user bitmap that will never
        be put to a board, but may be used as a temporary render buffer and accessed
        with OS blit functions, too. Bitmaps allocated with this flag do not need to
        be locked.  }

  {                                                                       }
  { Attributes for p96GetBitMapAttr
    }
     P96BMA_WIDTH = 0;
      P96BMA_HEIGHT = 1;
      P96BMA_DEPTH = 2;
      P96BMA_MEMORY = 3;
      P96BMA_BYTESPERROW = 4;
      P96BMA_BYTESPERPIXEL = 5;
      P96BMA_BITSPERPIXEL = 6;
      P96BMA_RGBFORMAT = 7;
      P96BMA_ISP96 = 8;
      P96BMA_ISONBOARD = 9;
      P96BMA_BOARDMEMBASE = 10;
      P96BMA_BOARDIOBASE = 11;
      P96BMA_BOARDMEMIOBASE = 12;

  {                                                                       }
  { Attributes for p96GetModeIDAttr
    }

       P96IDA_WIDTH = 0;
       P96IDA_HEIGHT = 1;
       P96IDA_DEPTH = 2;
       P96IDA_BYTESPERPIXEL = 3;
       P96IDA_BITSPERPIXEL = 4;
       P96IDA_RGBFORMAT = 5;
       P96IDA_ISP96 = 6;
       P96IDA_BOARDNUMBER = 7;
       P96IDA_STDBYTESPERROW = 8;
       P96IDA_BOARDNAME = 9;
       P96IDA_COMPATIBLEFORMATS = 10;
       P96IDA_VIDEOCOMPATIBLE = 11;
       P96IDA_PABLOIVCOMPATIBLE = 12;
       P96IDA_PALOMAIVCOMPATIBLE = 13;

     {                                                                       }
     { Tags for p96BestModeIDTagList
     }
     P96BIDTAG_Dummy = TAG_USER + 96;
     P96BIDTAG_FormatsAllowed = P96BIDTAG_Dummy + $0001;
     P96BIDTAG_FormatsForbidden = P96BIDTAG_Dummy + $0002;
     P96BIDTAG_NominalWidth = P96BIDTAG_Dummy + $0003;
     P96BIDTAG_NominalHeight = P96BIDTAG_Dummy + $0004;
     P96BIDTAG_Depth = P96BIDTAG_Dummy + $0005;
     P96BIDTAG_VideoCompatible = P96BIDTAG_Dummy + $0006;
     P96BIDTAG_PabloIVCompatible = P96BIDTAG_Dummy + $0007;
     P96BIDTAG_PalomaIVCompatible = P96BIDTAG_Dummy + $0008;
     {                                                                       }
     { Tags for p96RequestModeIDTagList
     }
     P96MA_Dummy = (TAG_USER + $10000) + 96;
     P96MA_MinWidth = P96MA_Dummy + $0001;
     P96MA_MinHeight = P96MA_Dummy + $0002;
     P96MA_MinDepth = P96MA_Dummy + $0003;
     P96MA_MaxWidth = P96MA_Dummy + $0004;
     P96MA_MaxHeight = P96MA_Dummy + $0005;
     P96MA_MaxDepth = P96MA_Dummy + $0006;
     P96MA_DisplayID = P96MA_Dummy + $0007;
     P96MA_FormatsAllowed = P96MA_Dummy + $0008;
     P96MA_FormatsForbidden = P96MA_Dummy + $0009;
     P96MA_WindowTitle = P96MA_Dummy + $000a;
     P96MA_OKText = P96MA_Dummy + $000b;
     P96MA_CancelText = P96MA_Dummy + $000c;
     P96MA_Window = P96MA_Dummy + $000d;
     P96MA_PubScreenName = P96MA_Dummy + $000e;
     P96MA_Screen = P96MA_Dummy + $000f;
     P96MA_VideoCompatible = P96MA_Dummy + $0010;
     P96MA_PabloIVCompatible = P96MA_Dummy + $0011;
     P96MA_PalomaIVCompatible = P96MA_Dummy + $0012;
     {                                                                       }
     { Tags for p96OpenScreenTagList
     }
     P96SA_Dummy = (TAG_USER + $20000) + 96;
     P96SA_Left = P96SA_Dummy + $0001;
     P96SA_Top = P96SA_Dummy + $0002;
     P96SA_Width = P96SA_Dummy + $0003;
     P96SA_Height = P96SA_Dummy + $0004;
     P96SA_Depth = P96SA_Dummy + $0005;
     P96SA_DetailPen = P96SA_Dummy + $0006;
     P96SA_BlockPen = P96SA_Dummy + $0007;
     P96SA_Title = P96SA_Dummy + $0008;
     P96SA_Colors = P96SA_Dummy + $0009;
     P96SA_ErrorCode = P96SA_Dummy + $000a;
     P96SA_Font = P96SA_Dummy + $000b;
     P96SA_SysFont = P96SA_Dummy + $000c;
     P96SA_Type = P96SA_Dummy + $000d;
     P96SA_BitMap = P96SA_Dummy + $000e;
     P96SA_PubName = P96SA_Dummy + $000f;
     P96SA_PubSig = P96SA_Dummy + $0010;
     P96SA_PubTask = P96SA_Dummy + $0011;
     P96SA_DisplayID = P96SA_Dummy + $0012;
     P96SA_DClip = P96SA_Dummy + $0013;
     P96SA_ShowTitle = P96SA_Dummy + $0014;
     P96SA_Behind = P96SA_Dummy + $0015;
     P96SA_Quiet = P96SA_Dummy + $0016;
     P96SA_AutoScroll = P96SA_Dummy + $0017;
     P96SA_Pens = P96SA_Dummy + $0018;
     P96SA_SharePens = P96SA_Dummy + $0019;
     P96SA_BackFill = P96SA_Dummy + $001a;
     P96SA_Colors32 = P96SA_Dummy + $001b;
     P96SA_VideoControl = P96SA_Dummy + $001c;
     P96SA_RGBFormat = P96SA_Dummy + $001d;
     P96SA_NoSprite = P96SA_Dummy + $001e;
     P96SA_NoMemory = P96SA_Dummy + $001f;
     P96SA_RenderFunc = P96SA_Dummy + $0020;
     P96SA_SaveFunc = P96SA_Dummy + $0021;
     P96SA_UserData = P96SA_Dummy + $0022;
     P96SA_Alignment = P96SA_Dummy + $0023;
     P96SA_FixedScreen = P96SA_Dummy + $0024;
     P96SA_Exclusive = P96SA_Dummy + $0025;
     P96SA_ConstantBytesPerRow = P96SA_Dummy + $0026;
     {                                                                       }
     {
     }
     MODENAMELENGTH = 48;

  type

     pubyte = ^ubyte;

     PP96Mode = ^TP96Mode;
     TP96Mode = record
          Node : tNode;
          Description : array[0..(MODENAMELENGTH)-1] of char;
          Width : UWORD;
          Height : UWORD;
          Depth : UWORD;
          DisplayID : ULONG;
       end;

     {                                                                       }
     { Structure to describe graphics data

       short description of the entries:
       Memory:        pointer to graphics data
       BytesPerRow:   distance in bytes between one pixel and its neighbour up
                      or down.
       pad:           private, not used.
       RGBFormat:     RGBFormat of the data.
    }
     PRenderInfo = ^TRenderInfo;
     TRenderInfo = record
          Memory : APTR;
          BytesPerRow : WORD;
          pad : WORD;
          RGBFormat : RGBFTYPE;
       end;

     {                                                                       }
     { Structure for p96WriteTrueColorData() and p96ReadTrueColorData()

       short description of the entries:
       PixelDistance: distance in bytes between the red (must be the same as
                      for the green or blue) component of one pixel and its
                      next neighbour to the left or right.
       BytesPerRow:   distance in bytes between the red (must be the same as
                      for the green or blue) component of one pixel and its
                      next neighbour up or down.
       RedData:       pointer to the red component of the upper left pixel.
       GreenData, BlueData: the same as above.

       examples (for an array width of 640 pixels):
       a) separate arrays for each color:
           (1, 640, red, green, blue );
       b) plain 24 bit RGB data:
           (3, 640 3, array, array+1, array+2 );
       c) 24 bit data, arranged as ARGB:
           (4, 640 4, array+1, array+2, array+3 );
     }
     PTrueColorInfo = ^TTrueColorInfo;
     TTrueColorInfo = record
          PixelDistance : ULONG;
          BytesPerRow : ULONG;
          RedData : PUBYTE;
          GreenData : PUBYTE;
          BlueData : PUBYTE;
       end;

     {                                                                       }
     { Tags for PIPs
     }

  const
     P96PIP_Dummy = (TAG_USER + $30000) + 96;
     { RGBFTYPE (I)  }
     P96PIP_SourceFormat = P96PIP_Dummy + 1;
     { struct BitMap   (G)  }
     P96PIP_SourceBitMap = P96PIP_Dummy + 2;
     { struct RastPort   (G)  }
     P96PIP_SourceRPort = P96PIP_Dummy + 3;
     { ULONG (I)  }
     P96PIP_SourceWidth = P96PIP_Dummy + 4;
     { ULONG (I)  }
     P96PIP_SourceHeight = P96PIP_Dummy + 5;
     { ULONG (I) default: PIPT_MemoryWindow  }
     P96PIP_Type = P96PIP_Dummy + 6;
     { LONG  (I)  }
     P96PIP_ErrorCode = P96PIP_Dummy + 7;
     { ULONG (IGS) default: 0  }
     P96PIP_Brightness = P96PIP_Dummy + 8;
     { ULONG (I) default: 0  }
     P96PIP_Left = P96PIP_Dummy + 9;
     { ULONG (I) default: 0  }
     P96PIP_Top = P96PIP_Dummy + 10;
     { ULONG (I) default: inner width of window  }
     P96PIP_Width = P96PIP_Dummy + 11;
     { ULONG (I) default: inner height of window  }
     P96PIP_Height = P96PIP_Dummy + 12;
     { ULONG (I) default: PIPRel_Width|PIPRel_Height  }
     P96PIP_Relativity = P96PIP_Dummy + 13;
     { struct ColorSpec   (IS)
       ti_Data is an array of struct ColorSpec,
       terminated by ColorIndex = -1.  Specifies
       initial screen palette colors.
       Also see P96PIP_Colors32.
       This only works with CLUT PIPs on non-CLUT
       screens. For CLUT PIPs on CLUT screens the
       PIP colors share the screen palette.
      }
     P96PIP_Colors = P96PIP_Dummy + 14;
     { ULONG  (IS)
       Tag to set the palette colors at 32 bits-per-gun.
       ti_Data is a pointer   to a table to be passed to
       the graphics.library/LoadRGB32() function.
       This format supports both runs of color
       registers and sparse registers.  See the
       autodoc for that function for full details.
       Any color set here has precedence over
       the same register set by P96PIP_Colors.
       This only works with CLUT PIPs on non-CLUT
       screens. For CLUT PIPs on CLUT screens the
       PIP colors share the screen palette.
     }
     P96PIP_Colors32 = P96PIP_Dummy + 15;
     P96PIP_NoMemory = P96PIP_Dummy + 16;
     P96PIP_RenderFunc = P96PIP_Dummy + 17;
     P96PIP_SaveFunc = P96PIP_Dummy + 18;
     P96PIP_UserData = P96PIP_Dummy + 19;
     P96PIP_Alignment = P96PIP_Dummy + 20;
     P96PIP_ConstantBytesPerRow = P96PIP_Dummy + 21;
     P96PIP_AllowCropping = P96PIP_Dummy + 22;
     P96PIP_InitialIntScaling = P96PIP_Dummy + 23;

     PIPT_MemoryWindow = 0;
     PIPT_VideoWindow = 1;
     PIPT_NUMTYPES = 2;

     P96PIPT_MemoryWindow = PIPT_MemoryWindow;
     P96PIPT_VideoWindow = PIPT_VideoWindow;
     { P96PIP_Left is relative to the right side (negative value)  }
     PIPRel_Right = 1;
     { P96PIP_Top is relative to the bottom (negative value)  }
     PIPRel_Bottom = 2;
     { P96PIP_Width is amount of pixels not used by PIP at the
       right side of the window (negative value)  }
     PIPRel_Width = 4;
     { P96PIP_Height is amount of pixels not used by PIP at the
       window bottom (negative value)  }
     PIPRel_Height = 8;
     { couldn't get normal memory  }
     PIPERR_NOMEMORY = 1;
     { Failed to attach to a screen  }
     PIPERR_ATTACHFAIL = 2;
     { PIP not available for other reason        }
     PIPERR_NOTAVAILABLE = 3;
     { couldn't get a free pen for occlusion  }
     PIPERR_OUTOFPENS = 4;
     { type, width, height or format invalid  }
     PIPERR_BADDIMENSIONS = 5;
     { couldn't open window  }
     PIPERR_NOWINDOW = 6;
     { specified alignment is not ok  }
     PIPERR_BADALIGNMENT = 7;
     { pip would be cropped, but isn't allowed to  }
     PIPERR_CROPPED = 8;
     {                                                                       }
     { Tags for P96GetRTGDataTagList
     }
     P96RD_Dummy = (TAG_USER + $40000) + 96;
     P96RD_NumberOfBoards = P96RD_Dummy + 1;
     {                                                                       }
     { Tags for P96GetBoardDataTagList
     }
     P96BD_Dummy = (TAG_USER + $50000) + 96;
     P96BD_BoardName = P96BD_Dummy + 1;
     P96BD_ChipName = P96BD_Dummy + 2;
     P96BD_TotalMemory = P96BD_Dummy + 4;
     P96BD_FreeMemory = P96BD_Dummy + 5;
     P96BD_LargestFreeMemory = P96BD_Dummy + 6;
     P96BD_MonitorSwitch = P96BD_Dummy + 7;
     P96BD_RGBFormats = P96BD_Dummy + 8;
     P96BD_MemoryClock = P96BD_Dummy + 9;
     {                                                                       }
     {                                                                       }


VAR P96Base : pLibrary;

FUNCTION p96AllocBitMap(SizeX : Ulong; SizeY : Ulong; Depth : Ulong; Flags : Ulong; Friend : pBitMap; RGBFormat : RGBFTYPE) : pBitMap;
PROCEDURE p96FreeBitMap(BitMap : pBitMap);
FUNCTION p96GetBitMapAttr(BitMap : pBitMap; Attribute : Ulong) : Ulong;
FUNCTION p96LockBitMap(BitMap : pBitMap; Buffer : pCHAR; Size : Ulong) : LONGINT;
PROCEDURE p96UnlockBitMap(BitMap : pBitMap; Lock : LONGINT);
FUNCTION p96BestModeIDTagList(Tags : pTagItem) : Ulong;
FUNCTION p96RequestModeIDTagList(Tags : pTagItem) : Ulong;
FUNCTION p96AllocModeListTagList(Tags : pTagItem) : pList;
PROCEDURE p96FreeModeList(List : pList);
FUNCTION p96GetModeIDAttr(Mode : Ulong; Attribute : Ulong) : Ulong;
FUNCTION p96OpenScreenTagList(Tags : pTagItem) : pScreen;
FUNCTION p96CloseScreen(Screen : pScreen) : BOOLEAN;
PROCEDURE p96WritePixelArray(ri : pRenderInfo; SrcX : WORD; SrcY : WORD; rp : pRastPort; DestX : WORD; DestY : WORD; SizeX : WORD; SizeY : WORD);
PROCEDURE p96ReadPixelArray(ri : pRenderInfo; DestX : WORD; DestY : WORD; rp : pRastPort; SrcX : WORD; SrcY : WORD; SizeX : WORD; SizeY : WORD);
FUNCTION p96WritePixel(rp : pRastPort; x : WORD; y : WORD; color : Ulong) : Ulong;
FUNCTION p96ReadPixel(rp : pRastPort; x : WORD; y : WORD) : Ulong;
PROCEDURE p96RectFill(rp : pRastPort; MinX : WORD; MinY : WORD; MaxX : WORD; MaxY : WORD; color : Ulong);
PROCEDURE p96WriteTrueColorData(tci : pTrueColorInfo; SrcX : WORD; SrcY : WORD; rp : pRastPort; DestX : WORD; DestY : WORD; SizeX : WORD; SizeY : WORD);
PROCEDURE p96ReadTrueColorData(tci : pTrueColorInfo; DestX : WORD; DestY : WORD; rp : pRastPort; SrcX : WORD; SrcY : WORD; SizeX : WORD; SizeY : WORD);
FUNCTION p96PIP_OpenTagList(Tags : pTagItem) : pWindow;
FUNCTION p96PIP_Close(Window : pWindow) : BOOLEAN;
FUNCTION p96PIP_SetTagList(Window : pWindow; Tags : pTagItem) : LONGINT;
FUNCTION p96PIP_GetTagList(Window : pWindow; Tags : pTagItem) : LONGINT;
FUNCTION p96GetRTGDataTagList(Tags : pTagItem) : LONGINT;
FUNCTION p96GetBoardDataTagList(Board : Ulong; Tags : pTagItem) : LONGINT;
FUNCTION p96EncodeColor(RGBFormat : RGBFTYPE; Color : Ulong) : Ulong;

{
 Functions and procedures with array of const go here
}
FUNCTION p96BestModeIDTags(const Tags : Array Of Const) : longword;
FUNCTION p96RequestModeIDTags(const Tags : Array Of Const) : longword;
FUNCTION p96AllocModeListTags(const Tags : Array Of Const) : pList;
FUNCTION p96OpenScreenTags(const Tags : Array Of Const) : pScreen;
FUNCTION p96PIP_OpenTags(const Tags : Array Of Const) : pWindow;
FUNCTION p96PIP_SetTags(Window : pWindow; const Tags : Array Of Const) : LONGINT;
FUNCTION p96PIP_GetTags(Window : pWindow; const Tags : Array Of Const) : LONGINT;
FUNCTION p96GetRTGDataTags(const Tags : Array Of Const) : LONGINT;
FUNCTION p96GetBoardDataTags(Board : longword; const Tags : Array Of Const) : LONGINT;

{You can remove this include and use a define instead}
{$I useautoopenlib.inc}
{$ifdef use_init_openlib}
procedure InitPICASSO96APILibrary;
{$endif use_init_openlib}

{This is a variable that knows how the unit is compiled}
var
    PICASSO96APIIsCompiledHow : longint;

IMPLEMENTATION

uses
{$ifndef dont_use_openlib}
msgbox,
{$endif dont_use_openlib}
tagsarray;


FUNCTION p96AllocBitMap(SizeX : Ulong; SizeY : Ulong; Depth : Ulong; Flags : Ulong; Friend : pBitMap; RGBFormat : RGBFTYPE) : pBitMap;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  SizeX,D0
        MOVE.L  SizeY,D1
        MOVE.L  Depth,D2
        MOVE.L  Flags,D3
        MOVEA.L Friend,A0
        MOVE.L  RGBFormat,D7
        MOVEA.L P96Base,A6
        JSR     -030(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE p96FreeBitMap(BitMap : pBitMap);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L BitMap,A0
        MOVEA.L P96Base,A6
        JSR     -036(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION p96GetBitMapAttr(BitMap : pBitMap; Attribute : Ulong) : Ulong;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L BitMap,A0
        MOVE.L  Attribute,D0
        MOVEA.L P96Base,A6
        JSR     -042(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION p96LockBitMap(BitMap : pBitMap; Buffer : pCHAR; Size : Ulong) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L BitMap,A0
        MOVEA.L Buffer,A1
        MOVE.L  Size,D0
        MOVEA.L P96Base,A6
        JSR     -048(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE p96UnlockBitMap(BitMap : pBitMap; Lock : LONGINT);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L BitMap,A0
        MOVE.L  Lock,D0
        MOVEA.L P96Base,A6
        JSR     -054(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION p96BestModeIDTagList(Tags : pTagItem) : Ulong;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L Tags,A0
        MOVEA.L P96Base,A6
        JSR     -060(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION p96RequestModeIDTagList(Tags : pTagItem) : Ulong;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L Tags,A0
        MOVEA.L P96Base,A6
        JSR     -066(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION p96AllocModeListTagList(Tags : pTagItem) : pList;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L Tags,A0
        MOVEA.L P96Base,A6
        JSR     -072(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE p96FreeModeList(List : pList);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L List,A0
        MOVEA.L P96Base,A6
        JSR     -078(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION p96GetModeIDAttr(Mode : Ulong; Attribute : Ulong) : Ulong;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  Mode,D0
        MOVE.L  Attribute,D1
        MOVEA.L P96Base,A6
        JSR     -084(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION p96OpenScreenTagList(Tags : pTagItem) : pScreen;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L Tags,A0
        MOVEA.L P96Base,A6
        JSR     -090(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION p96CloseScreen(Screen : pScreen) : BOOLEAN;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L Screen,A0
        MOVEA.L P96Base,A6
        JSR     -096(A6)
        MOVEA.L (A7)+,A6
        TST.W   D0
        BEQ.B   @end
        MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

PROCEDURE p96WritePixelArray(ri : pRenderInfo; SrcX : WORD; SrcY : WORD; rp : pRastPort; DestX : WORD; DestY : WORD; SizeX : WORD; SizeY : WORD);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L ri,A0
        MOVE.L  SrcX,D0
        MOVE.L  SrcY,D1
        MOVEA.L rp,A1
        MOVE.L  DestX,D2
        MOVE.L  DestY,D3
        MOVE.L  SizeX,D4
        MOVE.L  SizeY,D5
        MOVEA.L P96Base,A6
        JSR     -102(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE p96ReadPixelArray(ri : pRenderInfo; DestX : WORD; DestY : WORD; rp : pRastPort; SrcX : WORD; SrcY : WORD; SizeX : WORD; SizeY : WORD);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L ri,A0
        MOVE.L  DestX,D0
        MOVE.L  DestY,D1
        MOVEA.L rp,A1
        MOVE.L  SrcX,D2
        MOVE.L  SrcY,D3
        MOVE.L  SizeX,D4
        MOVE.L  SizeY,D5
        MOVEA.L P96Base,A6
        JSR     -108(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION p96WritePixel(rp : pRastPort; x : WORD; y : WORD; color : Ulong) : Ulong;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L rp,A1
        MOVE.L  x,D0
        MOVE.L  y,D1
        MOVE.L  color,D2
        MOVEA.L P96Base,A6
        JSR     -114(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION p96ReadPixel(rp : pRastPort; x : WORD; y : WORD) : Ulong;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L rp,A1
        MOVE.L  x,D0
        MOVE.L  y,D1
        MOVEA.L P96Base,A6
        JSR     -120(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE p96RectFill(rp : pRastPort; MinX : WORD; MinY : WORD; MaxX : WORD; MaxY : WORD; color : Ulong);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L rp,A1
        MOVE.L  MinX,D0
        MOVE.L  MinY,D1
        MOVE.L  MaxX,D2
        MOVE.L  MaxY,D3
        MOVE.L  color,D4
        MOVEA.L P96Base,A6
        JSR     -126(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE p96WriteTrueColorData(tci : pTrueColorInfo; SrcX : WORD; SrcY : WORD; rp : pRastPort; DestX : WORD; DestY : WORD; SizeX : WORD; SizeY : WORD);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L tci,A0
        MOVE.L  SrcX,D0
        MOVE.L  SrcY,D1
        MOVEA.L rp,A1
        MOVE.L  DestX,D2
        MOVE.L  DestY,D3
        MOVE.L  SizeX,D4
        MOVE.L  SizeY,D5
        MOVEA.L P96Base,A6
        JSR     -132(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE p96ReadTrueColorData(tci : pTrueColorInfo; DestX : WORD; DestY : WORD; rp : pRastPort; SrcX : WORD; SrcY : WORD; SizeX : WORD; SizeY : WORD);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L tci,A0
        MOVE.L  DestX,D0
        MOVE.L  DestY,D1
        MOVEA.L rp,A1
        MOVE.L  SrcX,D2
        MOVE.L  SrcY,D3
        MOVE.L  SizeX,D4
        MOVE.L  SizeY,D5
        MOVEA.L P96Base,A6
        JSR     -138(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION p96PIP_OpenTagList(Tags : pTagItem) : pWindow;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L Tags,A0
        MOVEA.L P96Base,A6
        JSR     -144(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION p96PIP_Close(Window : pWindow) : BOOLEAN;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L Window,A0
        MOVEA.L P96Base,A6
        JSR     -150(A6)
        MOVEA.L (A7)+,A6
        TST.W   D0
        BEQ.B   @end
        MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION p96PIP_SetTagList(Window : pWindow; Tags : pTagItem) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L Window,A0
        MOVEA.L Tags,A1
        MOVEA.L P96Base,A6
        JSR     -156(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION p96PIP_GetTagList(Window : pWindow; Tags : pTagItem) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L Window,A0
        MOVEA.L Tags,A1
        MOVEA.L P96Base,A6
        JSR     -162(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION p96GetRTGDataTagList(Tags : pTagItem) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L Tags,A0
        MOVEA.L P96Base,A6
        JSR     -180(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION p96GetBoardDataTagList(Board : Ulong; Tags : pTagItem) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  Board,D0
        MOVEA.L Tags,A0
        MOVEA.L P96Base,A6
        JSR     -186(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION p96EncodeColor(RGBFormat : RGBFTYPE; Color : Ulong) : Ulong;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  RGBFormat,D0
        MOVE.L  Color,D1
        MOVEA.L P96Base,A6
        JSR     -192(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;


{
 Functions and procedures with array of const go here
}
FUNCTION p96BestModeIDTags(const Tags : Array Of Const) : longword;
begin
    p96BestModeIDTags := p96BestModeIDTagList(readintags(Tags));
end;

FUNCTION p96RequestModeIDTags(const Tags : Array Of Const) : longword;
begin
    p96RequestModeIDTags := p96RequestModeIDTagList(readintags(Tags));
end;

FUNCTION p96AllocModeListTags(const Tags : Array Of Const) : pList;
begin
    p96AllocModeListTags := p96AllocModeListTagList(readintags(Tags));
end;

FUNCTION p96OpenScreenTags(const Tags : Array Of Const) : pScreen;
begin
    p96OpenScreenTags := p96OpenScreenTagList(readintags(Tags));
end;

FUNCTION p96PIP_OpenTags(const Tags : Array Of Const) : pWindow;
begin
    p96PIP_OpenTags := p96PIP_OpenTagList(readintags(Tags));
end;

FUNCTION p96PIP_SetTags(Window : pWindow; const Tags : Array Of Const) : LONGINT;
begin
    p96PIP_SetTags := p96PIP_SetTagList(Window , readintags(Tags));
end;

FUNCTION p96PIP_GetTags(Window : pWindow; const Tags : Array Of Const) : LONGINT;
begin
    p96PIP_GetTags := p96PIP_GetTagList(Window , readintags(Tags));
end;

FUNCTION p96GetRTGDataTags(const Tags : Array Of Const) : LONGINT;
begin
    p96GetRTGDataTags := p96GetRTGDataTagList(readintags(Tags));
end;

FUNCTION p96GetBoardDataTags(Board : longword; const Tags : Array Of Const) : LONGINT;
begin
    p96GetBoardDataTags := p96GetBoardDataTagList(Board , readintags(Tags));
end;

const
    { Change VERSION and LIBVERSION to proper values }

    VERSION : string[2] = '0';
    LIBVERSION : longword = 0;

{$ifdef use_init_openlib}
  {$Info Compiling initopening of picasso96api.library}
  {$Info don't forget to use InitPICASSO96APILibrary in the beginning of your program}

var
    picasso96api_exit : Pointer;

procedure Closepicasso96apiLibrary;
begin
    ExitProc := picasso96api_exit;
    if P96Base <> nil then begin
        CloseLibrary(P96Base);
        P96Base := nil;
    end;
end;

procedure InitPICASSO96APILibrary;
begin
    P96Base := nil;
    P96Base := OpenLibrary(PICASSO96APINAME,LIBVERSION);
    if P96Base <> nil then begin
        picasso96api_exit := ExitProc;
        ExitProc := @Closepicasso96apiLibrary;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open picasso96api.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;
end;

begin
    PICASSO96APIIsCompiledHow := 2;
{$endif use_init_openlib}

{$ifdef use_auto_openlib}
  {$Info Compiling autoopening of picasso96api.library}

var
    picasso96api_exit : Pointer;

procedure Closepicasso96apiLibrary;
begin
    ExitProc := picasso96api_exit;
    if P96Base <> nil then begin
        CloseLibrary(P96Base);
        P96Base := nil;
    end;
end;

begin
    P96Base := nil;
    P96Base := OpenLibrary(PICASSO96APINAME,LIBVERSION);
    if P96Base <> nil then begin
        picasso96api_exit := ExitProc;
        ExitProc := @Closepicasso96apiLibrary;
        PICASSO96APIIsCompiledHow := 1;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open picasso96api.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;

{$endif use_auto_openlib}

{$ifdef dont_use_openlib}
begin
    PICASSO96APIIsCompiledHow := 3;
   {$Warning No autoopening of picasso96api.library compiled}
   {$Warning Make sure you open picasso96api.library yourself}
{$endif dont_use_openlib}


END. (* UNIT PICASSO96API *)


