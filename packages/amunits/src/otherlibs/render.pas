{
  This file is part of the Free Pascal run time library.

  A file in Amiga system run time library.
  Copyright (c) 2003 by Nils Sjöholm.
  member of the Amiga RTL development team.

  This is a unit for render.library

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
  12 Feb 2003

  nils.sjoholm@mailbox.swipnet.se Nils Sjoholm
}

{$mode objfpc}
{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
   {$smartlink on}
{$endif use_amiga_smartlink}

UNIT RENDER;

INTERFACE
USES Exec,utility,graphics;

VAR RenderBase : pLibrary;

type
    pPLANEPTR = ^PLANEPTR;

const
    RENDERNAME : PChar = 'render.library';

{
        $VER: render.h v40 (19.12.2002)
        render.library definitions
   }

  const
     RND_TAGBASE = TAG_USER + $1000;
  {

        memhandler

                                                                          }
  { type of memhandler, see below  }
     RND_MemType = RND_TAGBASE + 1;
  { ptr to block of memory  }
     RND_MemBlock = RND_TAGBASE + 2;
  { size of memblock [bytes]  }
     RND_MemSize = RND_TAGBASE + 3;
  { memflags (exec/memory.h)  }
     RND_MemFlags = RND_TAGBASE + 18;
  { to pass a memhandler as an argument  }
     RND_RMHandler = RND_TAGBASE + 12;
  {
        memhandler types
    }
  { v39 exec dynamic pool  }
     RMHTYPE_POOL = 1;
  { private memory pool  }
     RMHTYPE_PRIVATE = 2;
  { common public memory  }
     RMHTYPE_PUBLIC = 3;
  {

        palette

                                                                          }
  { palette import/export format  }
     RND_PaletteFormat = RND_TAGBASE + 19;
  { tag to indicate a palette is EHB  }
     RND_EHBPalette = RND_TAGBASE + 22;
  { first palette entry  }
     RND_FirstColor = RND_TAGBASE + 23;
  { dispose the old palette and load a new one  }
     RND_NewPalette = RND_TAGBASE + 24;
  { quantization factors  }
     RND_RGBWeight = RND_TAGBASE + 11;
  {
        palette format types
    }
  { ULONG red,green,blue  }
     PALFMT_RGB32 = 1;
  { ULONG 0x00rrggbb  }
     PALFMT_RGB8 = 2;
  { UWORD 0xrgb  }
     PALFMT_RGB4 = 3;
  { render.library palette  }
     PALFMT_PALETTE = 4;
  {
        palette sort mode types
        for the use with SortPalette()
    }
  { no particular order  }
     PALMODE_NONE = $0000;
  { sort palette entries by brightness  }
     PALMODE_BRIGHTNESS = $0001;
  { sort palette entries by the number of pixels that they represent.
           You must supply the RND_Histogram taglist argument.  }
     PALMODE_POPULARITY = $0002;
  { sort palette entries by the number of histogram entries that they
           represent. You must supply the RND_Histogram taglist argument.  }
     PALMODE_REPRESENTATION = $0003;
  { sort palette entries by their optical significance for the human
           eye. Implementation is unknown to you and may change.
           You must supply the RND_Histogram taglist argument.  }
     PALMODE_SIGNIFICANCE = $0004;
  { sort palette entries by color intensity  }
     PALMODE_SATURATION = $0005;
  { By default, sort direction is descending, i.e. the precedence is
           more-to-less. Combine with this flag to invert the sort direction.  }
     PALMODE_ASCENDING = $0008;
  {

        histogram related

                                                                          }
  { histogram type, see below  }
     RND_HSType = RND_TAGBASE + 4;
  { a histogram as an argument  }
     RND_Histogram = RND_TAGBASE + 9;
  {
        Histogram / Palette types
        to be specified with RND_HSType
    }
  { 12bit dynamic histogram  }
     HSTYPE_12BIT = 4;
  { 15bit dynamic histogram  }
     HSTYPE_15BIT = 5;
  { 18bit dynamic histogram  }
     HSTYPE_18BIT = 6;
  { 21bit dynamic histogram  }
     HSTYPE_21BIT = 7;
  { 24bit dynamic histogram  }
     HSTYPE_24BIT = 8;
  { 12bit tabular histogram  }
     HSTYPE_12BIT_TURBO = 20;
  { 15bit tabular histogram  }
     HSTYPE_15BIT_TURBO = 21;
  { 18bit tabular histogram  }
     HSTYPE_18BIT_TURBO = 22;
  {
        tags that can be queried via QueryHistogram()
    }
  { # pixels in a histogram  }
     RND_NumPixels = RND_TAGBASE + 5;
  { # colors in a histogram  }
     RND_NumColors = RND_TAGBASE + 6;
  {

        rendering and conversions

                                                                          }
  { color mode, see below  }
     RND_ColorMode = RND_TAGBASE + 7;
  { dither mode, see below  }
     RND_DitherMode = RND_TAGBASE + 8;
  { dither amount  }
     RND_DitherAmount = RND_TAGBASE + 26;
  { first color index to be output  }
     RND_OffsetColorZero = RND_TAGBASE + 10;
  {
        color mode types
        to be specified with RND_ColorMode
    }
  { normal palette lookup  }
     COLORMODE_CLUT = $0000;
  { HAM8 mode  }
     COLORMODE_HAM8 = $0001;
  { HAM6 mode  }
     COLORMODE_HAM6 = $0002;
  { mask to determine COLORMODE  }
     COLORMODE_MASK = $0003;
  {
        dither mode types
        to be specified with RND_DitherMode
    }
  { no dither  }
     DITHERMODE_NONE = $0000;
  { Floyd-Steinberg dither  }
     DITHERMODE_FS = $0001;
  { random dither. amount required.  }
     DITHERMODE_RANDOM = $0002;
  { EDD dither  }
     DITHERMODE_EDD = $0003;
  {

        miscellaneous

                                                                          }
  { progress callback hook  }
     RND_ProgressHook = RND_TAGBASE + 13;
  { total input width [pixels]  }
     RND_SourceWidth = RND_TAGBASE + 14;
  { total output width [pixels]  }
     RND_DestWidth = RND_TAGBASE + 15;
  { ptr to a chunky conversion table  }
     RND_PenTable = RND_TAGBASE + 16;
  { chunky data left edge [pixels]  }
     RND_LeftEdge = RND_TAGBASE + 17;
  { line callback hook  }
     RND_LineHook = RND_TAGBASE + 20;
  { Mapping-Engine  }
     RND_MapEngine = RND_TAGBASE + 27;
  { Interleave  }
     RND_Interleave = RND_TAGBASE + 28;
  { Palette  }
     RND_Palette = RND_TAGBASE + 29;
  { Weight factor  }
     RND_Weight = RND_TAGBASE + 30;
  { ScaleEngine  }
     RND_ScaleEngine = RND_TAGBASE + 31;
  { Texture coordinates  }
     RND_DestCoordinates = RND_TAGBASE + 42;
  { backcolor for filling  }
     RND_BGColor = RND_TAGBASE + 43;
  { backpen for filling  }
     RND_BGPen = RND_TAGBASE + 44;
  {

        alpha-channel and masking

                                                                          }
  { custom alpha-channel  }
     RND_AlphaChannel = RND_TAGBASE + 32;
  { bytes between alpha-channel pixels  }
     RND_AlphaModulo = RND_TAGBASE + 33;
  { width of alpha-channel array  }
     RND_AlphaWidth = RND_TAGBASE + 34;
  { masking RGB for CreateAlphaArray  }
     RND_MaskRGB = RND_TAGBASE + 35;
  { mask value for outside color range  }
     RND_MaskFalse = RND_TAGBASE + 36;
  { mask value for inside color range  }
     RND_MaskTrue = RND_TAGBASE + 37;
  { total source width for 3channel operations  }
     RND_SourceWidth2 = RND_TAGBASE + 38;
  { second custom alpha-channel  }
     RND_AlphaChannel2 = RND_TAGBASE + 39;
  { pixel modulo for a second alpha-channel  }
     RND_AlphaModulo2 = RND_TAGBASE + 40;
  { width of a second alpha-channel array  }
     RND_AlphaWidth2 = RND_TAGBASE + 41;
  {

        PixelFormat

                                                                          }
  { pixel format, see below  }
     RND_PixelFormat = RND_TAGBASE + 25;
     PIXFMTB_CHUNKY = 3;
     PIXFMTB_BITMAP = 4;
     PIXFMTB_RGB = 5;
     PIXFMT_CHUNKY_CLUT = (1 shl PIXFMTB_CHUNKY) + COLORMODE_CLUT;
     PIXFMT_0RGB_32 = (1 shl PIXFMTB_RGB) + 0;
  {
        these types are currently not used by render.library, but
        some of them are applicable for guigfx.library functions:
    }
     PIXFMT_CHUNKY_HAM8 = (1 shl PIXFMTB_CHUNKY) + COLORMODE_HAM8;
     PIXFMT_CHUNKY_HAM6 = (1 shl PIXFMTB_CHUNKY) + COLORMODE_HAM6;
     PIXFMT_BITMAP_CLUT = (1 shl PIXFMTB_BITMAP) + COLORMODE_CLUT;
     PIXFMT_BITMAP_HAM8 = (1 shl PIXFMTB_BITMAP) + COLORMODE_HAM8;
     PIXFMT_BITMAP_HAM6 = (1 shl PIXFMTB_BITMAP) + COLORMODE_HAM6;
     PIXFMT_RGB_24 = (1 shl PIXFMTB_RGB) + 1;
  {
        strictly internal:
    }
     PIXFMT_BITMAP_RGB = (1 shl PIXFMTB_BITMAP) + (1 shl PIXFMTB_RGB);
  {

        ExtractPalette return codes

        You must at least check for EXTP_SUCCESS.
        EXTP_NO_DATA indicates that there were no colors
        in the histogram.

                                                                          }
     EXTP_SUCCESS = 0;
     EXTP_NOT_ENOUGH_MEMORY = 1;
     EXTP_CALLBACK_ABORTED = 2;
     EXTP_NO_DATA = 3;
  {

        AddRGB, AddRGBImage and AddChunkyImage return codes

        You must at least check for ADDH_SUCCESS.
        If not delivered, the histogram might be
        inaccurate.

                                                                          }
     ADDH_SUCCESS = 0;
     ADDH_NOT_ENOUGH_MEMORY = 1;
     ADDH_CALLBACK_ABORTED = 2;
     ADDH_NO_DATA = 3;
  {

        Render return codes

        You must at least check for REND_SUCCESS.
        If not delivered, the image has not been
        rendered completely.

                                                                          }
     REND_SUCCESS = 0;
     REND_NOT_ENOUGH_MEMORY = 1;
     REND_CALLBACK_ABORTED = 2;
     REND_NO_VALID_PALETTE = 3;
     REND_NO_DATA = 3;
  {

        SortPalette return codes

        You must at least check for SORTP_SUCCESS.
        SORTP_NO_DATA indicates that there were data missing,
        e.g. you specified no histogram or the histogram was empty.

                                                                          }
     SORTP_SUCCESS = 0;
     SORTP_NO_DATA = 1;
     SORTP_NOT_ENOUGH_MEMORY = 2;
     SORTP_NOT_IMPLEMENTED = 3;
  {

        conversion return codes

        These return codes apply to conversion functions
        such as Chunky2RGB and ConvertChunky.

                                                                          }
     CONV_SUCCESS = 0;
     CONV_CALLBACK_ABORTED = 1;
     CONV_NOT_ENOUGH_MEMORY = 2;
     CONV_NO_DATA = 3;





FUNCTION AddChunkyImageA(histogram : POINTER; chunky : pByte; width : WORD; height : WORD; palette : POINTER; taglist : pTagItem) : longword;
FUNCTION AddHistogramA(histogram1 : POINTER; histogram2 : POINTER; taglist : pTagItem) : longword;
FUNCTION AddRGB(histogram : POINTER; RGB : longword; count : longword) : longword;
FUNCTION AddRGBImageA(histogram : POINTER; rgb : pULONG; width : WORD; height : WORD; taglist : pTagItem) : longword;
FUNCTION AllocRenderMem(rendermemhandler : POINTER; size : longword) : POINTER;
FUNCTION AllocRenderVec(rendermemhandler : POINTER; size : longword) : POINTER;
FUNCTION AllocRenderVecClear(rendermemhandler : POINTER; size : longword) : POINTER;
PROCEDURE ApplyAlphaChannelA(sourcearray : pULONG; width : WORD; height : WORD; destarray : pULONG; taglist : pTagItem);
FUNCTION BestPen(palette : POINTER; rgb : longword) : LONGINT;
PROCEDURE Chunky2BitMapA(chunky : pByte; sx : WORD; sy : WORD; width : WORD; height : WORD; bitmap : pBitMap; dx : WORD; dy : WORD; taglist : pTagItem);
FUNCTION Chunky2RGBA(chunky : pByte; width : WORD; height : WORD; rgb : pULONG; palette : POINTER; taglist : pTagItem) : longword;
FUNCTION ChunkyArrayDiversityA(chunky : pByte; palette : POINTER; width : WORD; height : WORD; taglist : pTagItem) : LONGINT;
FUNCTION ConvertChunkyA(source : pByte; oldpalette : POINTER; width : WORD; height : WORD; dest : pByte; newpalette : POINTER; taglist : pTagItem) : longword;
FUNCTION CountRGB(histogram : POINTER; rgb : longword) : longword;
PROCEDURE CreateAlphaArrayA(rgbarray : pULONG; width : WORD; height : WORD; taglist : pTagItem);
FUNCTION CreateHistogramA(taglist : pTagItem) : POINTER;
FUNCTION CreateMapEngineA(palette : POINTER; taglist : pTagItem) : POINTER;
FUNCTION CreatePaletteA(taglist : pTagItem) : POINTER;
PROCEDURE CreatePenTableA(chunky : pByte; oldpalette : POINTER; width : WORD; height : WORD; newpalette : POINTER; convtab : pByte; taglist : pTagItem);
FUNCTION CreateRMHandlerA(taglist : pTagItem) : POINTER;
FUNCTION CreateScaleEngineA(sourcewidth : WORD; sourceheight : WORD; destwidth : WORD; destheight : WORD; taglist : pTagItem) : POINTER;
PROCEDURE DeleteHistogram(histogram : POINTER);
PROCEDURE DeleteMapEngine(engine : POINTER);
PROCEDURE DeletePalette(palette : POINTER);
PROCEDURE DeleteRMHandler(rmh : POINTER);
PROCEDURE DeleteScaleEngine(engine : POINTER);
PROCEDURE ExportPaletteA(palette : POINTER; coltab : POINTER; taglist : pTagItem);
PROCEDURE ExtractAlphaChannelA(rgbarray : pULONG; width : WORD; height : WORD; chunkyarray : pByte; taglist : pTagItem);
FUNCTION ExtractPaletteA(histogram : POINTER; palette : pULONG; numcolors : WORD; taglist : pTagItem) : longword;
PROCEDURE FlushPalette(palette : POINTER);
PROCEDURE FreeRenderMem(rendermemhandler : POINTER; mem : POINTER; size : longword);
PROCEDURE FreeRenderVec(mem : POINTER);
PROCEDURE ImportPaletteA(palette : POINTER; coltab : POINTER; numcols : WORD; taglist : pTagItem);
PROCEDURE InsertAlphaChannelA(maskarray : pByte; width : WORD; height : WORD; rgbarray : pULONG; taglist : pTagItem);
FUNCTION MapChunkyArrayA(engine : POINTER; source : pByte; palette : POINTER; width : WORD; height : WORD; dest : pByte; taglist : pTagItem) : longword;
FUNCTION MapRGBArrayA(engine : POINTER; rgb : pULONG; width : WORD; height : WORD; chunky : pByte; taglist : pTagItem) : longword;
PROCEDURE MixAlphaChannelA(source1 : pULONG; source2 : pULONG; width : WORD; height : WORD; dest : pULONG; taglist : pTagItem);
PROCEDURE MixRGBArrayA(sourcearray : pULONG; width : WORD; height : WORD; destarray : pULONG; ratio : WORD; taglist : pTagItem);
PROCEDURE Planar2ChunkyA(planetab : pPLANEPTR; bytewidth : WORD; height : WORD; depth : WORD; bytesperrow : WORD; chunky : pByte; taglist : pTagItem);
FUNCTION QueryHistogram(histogram : POINTER; d0arg : longword) : longword;
FUNCTION RenderA(rgb : pULONG; width : WORD; height : WORD; chunky : pByte; palette : POINTER; taglist : pTagItem) : longword;
FUNCTION RGBArrayDiversityA(rgb : pULONG; width : WORD; height : WORD; taglist : pTagItem) : LONGINT;
FUNCTION ScaleA(engine : POINTER; source : POINTER; dest : POINTER; taglist : pTagItem) : longword;
FUNCTION ScaleOrdinate(source : WORD; dest : WORD; ordinate : WORD) : WORD;
FUNCTION SortPaletteA(palette : POINTER; mode : longword; taglist : pTagItem) : longword;
PROCEDURE TintRGBArrayA(source : pULONG; width : WORD; height : WORD; RGB : longword; ratio : WORD; dest : pULONG; taglist : pTagItem);
{
 Functions and procedures with array of const go here
}
FUNCTION AddChunkyImage(histogram : POINTER; chunky : pByte; width : WORD; height : WORD; palette : POINTER; const taglist : Array Of Const) : longword;
FUNCTION AddHistogram(histogram1 : POINTER; histogram2 : POINTER; const taglist : Array Of Const) : longword;
FUNCTION AddRGBImage(histogram : POINTER; rgb : pULONG; width : WORD; height : WORD; const taglist : Array Of Const) : longword;
PROCEDURE ApplyAlphaChannel(sourcearray : pULONG; width : WORD; height : WORD; destarray : pULONG; const taglist : Array Of Const);
PROCEDURE Chunky2BitMap(chunky : pByte; sx : WORD; sy : WORD; width : WORD; height : WORD; bitmap : pBitMap; dx : WORD; dy : WORD; const taglist : Array Of Const);
FUNCTION Chunky2RGB(chunky : pByte; width : WORD; height : WORD; rgb : pULONG; palette : POINTER; const taglist : Array Of Const) : longword;
FUNCTION ChunkyArrayDiversity(chunky : pByte; palette : POINTER; width : WORD; height : WORD; const taglist : Array Of Const) : LONGINT;
FUNCTION ConvertChunky(source : pByte; oldpalette : POINTER; width : WORD; height : WORD; dest : pByte; newpalette : POINTER; const taglist : Array Of Const) : longword;
PROCEDURE CreateAlphaArray(rgbarray : pULONG; width : WORD; height : WORD; const taglist : Array Of Const);
FUNCTION CreateHistogram(const taglist : Array Of Const) : POINTER;
FUNCTION CreateMapEngine(palette : POINTER; const taglist : Array Of Const) : POINTER;
FUNCTION CreatePalette(const taglist : Array Of Const) : POINTER;
PROCEDURE CreatePenTable(chunky : pByte; oldpalette : POINTER; width : WORD; height : WORD; newpalette : POINTER; convtab : pByte; const taglist : Array Of Const);
FUNCTION CreateRMHandler(const taglist : Array Of Const) : POINTER;
FUNCTION CreateScaleEngine(sourcewidth : WORD; sourceheight : WORD; destwidth : WORD; destheight : WORD; const taglist : Array Of Const) : POINTER;
PROCEDURE ExportPalette(palette : POINTER; coltab : POINTER; const taglist : Array Of Const);
PROCEDURE ExtractAlphaChannel(rgbarray : pULONG; width : WORD; height : WORD; chunkyarray : pByte; const taglist : Array Of Const);
FUNCTION ExtractPalette(histogram : POINTER; palette : pULONG; numcolors : WORD; const taglist : Array Of Const) : longword;
PROCEDURE ImportPalette(palette : POINTER; coltab : POINTER; numcols : WORD; const taglist : Array Of Const);
PROCEDURE InsertAlphaChannel(maskarray : pByte; width : WORD; height : WORD; rgbarray : pULONG; const taglist : Array Of Const);
FUNCTION MapChunkyArray(engine : POINTER; source : pByte; palette : POINTER; width : WORD; height : WORD; dest : pByte; const taglist : Array Of Const) : longword;
FUNCTION MapRGBArray(engine : POINTER; rgb : pULONG; width : WORD; height : WORD; chunky : pByte; const taglist : Array Of Const) : longword;
PROCEDURE MixAlphaChannel(source1 : pULONG; source2 : pULONG; width : WORD; height : WORD; dest : pULONG; const taglist : Array Of Const);
PROCEDURE MixRGBArray(sourcearray : pULONG; width : WORD; height : WORD; destarray : pULONG; ratio : WORD; const taglist : Array Of Const);
PROCEDURE Planar2Chunky(planetab : pPLANEPTR; bytewidth : WORD; height : WORD; depth : WORD; bytesperrow : WORD; chunky : pByte; const taglist : Array Of Const);
FUNCTION RenderTags(rgb : pULONG; width : WORD; height : WORD; chunky : pByte; palette : POINTER; const taglist : Array Of Const) : longword;
FUNCTION RGBArrayDiversity(rgb : pULONG; width : WORD; height : WORD; const taglist : Array Of Const) : LONGINT;
FUNCTION Scale(engine : POINTER; source : POINTER; dest : POINTER; const taglist : Array Of Const) : longword;
FUNCTION SortPalette(palette : POINTER; mode : longword; const taglist : Array Of Const) : longword;
PROCEDURE TintRGBArray(source : pULONG; width : WORD; height : WORD; RGB : longword; ratio : WORD; dest : pULONG; const taglist : Array Of Const);

{You can remove this include and use a define instead}
{$I useautoopenlib.inc}
{$ifdef use_init_openlib}
procedure InitRENDERLibrary;
{$endif use_init_openlib}

{This is a variable that knows how the unit is compiled}
var
    RENDERIsCompiledHow : longint;

IMPLEMENTATION

uses
{$ifndef dont_use_openlib}
msgbox,
{$endif dont_use_openlib}
tagsarray;

FUNCTION AddChunkyImageA(histogram : POINTER; chunky : pByte; width : WORD; height : WORD; palette : POINTER; taglist : pTagItem) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L histogram,A0
        MOVEA.L chunky,A1
        MOVE.L  width,D0
        MOVE.L  height,D1
        MOVEA.L palette,A2
        MOVEA.L taglist,A3
        MOVEA.L RenderBase,A6
        JSR     -108(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AddHistogramA(histogram1 : POINTER; histogram2 : POINTER; taglist : pTagItem) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L histogram1,A0
        MOVEA.L histogram2,A1
        MOVEA.L taglist,A2
        MOVEA.L RenderBase,A6
        JSR     -222(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AddRGB(histogram : POINTER; RGB : longword; count : longword) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L histogram,A0
        MOVE.L  RGB,D0
        MOVE.L  count,D1
        MOVEA.L RenderBase,A6
        JSR     -096(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AddRGBImageA(histogram : POINTER; rgb : pULONG; width : WORD; height : WORD; taglist : pTagItem) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L histogram,A0
        MOVEA.L rgb,A1
        MOVE.L  width,D0
        MOVE.L  height,D1
        MOVEA.L taglist,A2
        MOVEA.L RenderBase,A6
        JSR     -102(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AllocRenderMem(rendermemhandler : POINTER; size : longword) : POINTER;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L rendermemhandler,A0
        MOVE.L  size,D0
        MOVEA.L RenderBase,A6
        JSR     -054(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AllocRenderVec(rendermemhandler : POINTER; size : longword) : POINTER;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L rendermemhandler,A0
        MOVE.L  size,D0
        MOVEA.L RenderBase,A6
        JSR     -066(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AllocRenderVecClear(rendermemhandler : POINTER; size : longword) : POINTER;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L rendermemhandler,A0
        MOVE.L  size,D0
        MOVEA.L RenderBase,A6
        JSR     -306(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE ApplyAlphaChannelA(sourcearray : pULONG; width : WORD; height : WORD; destarray : pULONG; taglist : pTagItem);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L sourcearray,A0
        MOVE.L  width,D0
        MOVE.L  height,D1
        MOVEA.L destarray,A1
        MOVEA.L taglist,A2
        MOVEA.L RenderBase,A6
        JSR     -294(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION BestPen(palette : POINTER; rgb : longword) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L palette,A0
        MOVE.L  rgb,D0
        MOVEA.L RenderBase,A6
        JSR     -204(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE Chunky2BitMapA(chunky : pByte; sx : WORD; sy : WORD; width : WORD; height : WORD; bitmap : pBitMap; dx : WORD; dy : WORD; taglist : pTagItem);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L chunky,A0
        MOVE.L  sx,D0
        MOVE.L  sy,D1
        MOVE.L  width,D2
        MOVE.L  height,D3
        MOVEA.L bitmap,A1
        MOVE.L  dx,D4
        MOVE.L  dy,D5
        MOVEA.L taglist,A2
        MOVEA.L RenderBase,A6
        JSR     -138(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION Chunky2RGBA(chunky : pByte; width : WORD; height : WORD; rgb : pULONG; palette : POINTER; taglist : pTagItem) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L chunky,A0
        MOVE.L  width,D0
        MOVE.L  height,D1
        MOVEA.L rgb,A1
        MOVEA.L palette,A2
        MOVEA.L taglist,A3
        MOVEA.L RenderBase,A6
        JSR     -132(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION ChunkyArrayDiversityA(chunky : pByte; palette : POINTER; width : WORD; height : WORD; taglist : pTagItem) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L chunky,A0
        MOVEA.L palette,A1
        MOVE.L  width,D0
        MOVE.L  height,D1
        MOVEA.L taglist,A2
        MOVEA.L RenderBase,A6
        JSR     -270(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION ConvertChunkyA(source : pByte; oldpalette : POINTER; width : WORD; height : WORD; dest : pByte; newpalette : POINTER; taglist : pTagItem) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L source,A0
        MOVEA.L oldpalette,A1
        MOVE.L  width,D0
        MOVE.L  height,D1
        MOVEA.L dest,A2
        MOVEA.L newpalette,A3
        MOVEA.L taglist,A4
        MOVEA.L RenderBase,A6
        JSR     -162(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION CountRGB(histogram : POINTER; rgb : longword) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L histogram,A0
        MOVE.L  rgb,D0
        MOVEA.L RenderBase,A6
        JSR     -198(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE CreateAlphaArrayA(rgbarray : pULONG; width : WORD; height : WORD; taglist : pTagItem);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L rgbarray,A0
        MOVE.L  width,D0
        MOVE.L  height,D1
        MOVEA.L taglist,A1
        MOVEA.L RenderBase,A6
        JSR     -312(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION CreateHistogramA(taglist : pTagItem) : POINTER;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L taglist,A1
        MOVEA.L RenderBase,A6
        JSR     -078(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION CreateMapEngineA(palette : POINTER; taglist : pTagItem) : POINTER;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L palette,A0
        MOVEA.L taglist,A1
        MOVEA.L RenderBase,A6
        JSR     -246(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION CreatePaletteA(taglist : pTagItem) : POINTER;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L taglist,A1
        MOVEA.L RenderBase,A6
        JSR     -174(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE CreatePenTableA(chunky : pByte; oldpalette : POINTER; width : WORD; height : WORD; newpalette : POINTER; convtab : pByte; taglist : pTagItem);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L chunky,A0
        MOVEA.L oldpalette,A1
        MOVE.L  width,D0
        MOVE.L  height,D1
        MOVEA.L newpalette,A2
        MOVEA.L convtab,A3
        MOVEA.L taglist,A4
        MOVEA.L RenderBase,A6
        JSR     -168(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION CreateRMHandlerA(taglist : pTagItem) : POINTER;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L taglist,A1
        MOVEA.L RenderBase,A6
        JSR     -042(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION CreateScaleEngineA(sourcewidth : WORD; sourceheight : WORD; destwidth : WORD; destheight : WORD; taglist : pTagItem) : POINTER;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  sourcewidth,D0
        MOVE.L  sourceheight,D1
        MOVE.L  destwidth,D2
        MOVE.L  destheight,D3
        MOVEA.L taglist,A1
        MOVEA.L RenderBase,A6
        JSR     -144(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE DeleteHistogram(histogram : POINTER);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L histogram,A0
        MOVEA.L RenderBase,A6
        JSR     -084(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE DeleteMapEngine(engine : POINTER);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L engine,A0
        MOVEA.L RenderBase,A6
        JSR     -252(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE DeletePalette(palette : POINTER);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L palette,A0
        MOVEA.L RenderBase,A6
        JSR     -180(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE DeleteRMHandler(rmh : POINTER);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L rmh,A0
        MOVEA.L RenderBase,A6
        JSR     -048(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE DeleteScaleEngine(engine : POINTER);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L engine,A0
        MOVEA.L RenderBase,A6
        JSR     -150(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE ExportPaletteA(palette : POINTER; coltab : POINTER; taglist : pTagItem);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L palette,A0
        MOVEA.L coltab,A1
        MOVEA.L taglist,A2
        MOVEA.L RenderBase,A6
        JSR     -192(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE ExtractAlphaChannelA(rgbarray : pULONG; width : WORD; height : WORD; chunkyarray : pByte; taglist : pTagItem);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L rgbarray,A0
        MOVE.L  width,D0
        MOVE.L  height,D1
        MOVEA.L chunkyarray,A1
        MOVEA.L taglist,A2
        MOVEA.L RenderBase,A6
        JSR     -288(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION ExtractPaletteA(histogram : POINTER; palette : pULONG; numcolors : WORD; taglist : pTagItem) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L histogram,A0
        MOVEA.L palette,A1
        MOVE.L  numcolors,D0
        MOVEA.L taglist,A2
        MOVEA.L RenderBase,A6
        JSR     -114(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE FlushPalette(palette : POINTER);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L palette,A0
        MOVEA.L RenderBase,A6
        JSR     -210(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE FreeRenderMem(rendermemhandler : POINTER; mem : POINTER; size : longword);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L rendermemhandler,A0
        MOVEA.L mem,A1
        MOVE.L  size,D0
        MOVEA.L RenderBase,A6
        JSR     -060(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE FreeRenderVec(mem : POINTER);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L mem,A0
        MOVEA.L RenderBase,A6
        JSR     -072(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE ImportPaletteA(palette : POINTER; coltab : POINTER; numcols : WORD; taglist : pTagItem);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L palette,A0
        MOVEA.L coltab,A1
        MOVE.L  numcols,D0
        MOVEA.L taglist,A2
        MOVEA.L RenderBase,A6
        JSR     -186(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE InsertAlphaChannelA(maskarray : pByte; width : WORD; height : WORD; rgbarray : pULONG; taglist : pTagItem);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L maskarray,A0
        MOVE.L  width,D0
        MOVE.L  height,D1
        MOVEA.L rgbarray,A1
        MOVEA.L taglist,A2
        MOVEA.L RenderBase,A6
        JSR     -282(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION MapChunkyArrayA(engine : POINTER; source : pByte; palette : POINTER; width : WORD; height : WORD; dest : pByte; taglist : pTagItem) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L engine,A0
        MOVEA.L source,A1
        MOVEA.L palette,A2
        MOVE.L  width,D0
        MOVE.L  height,D1
        MOVEA.L dest,A3
        MOVEA.L taglist,A4
        MOVEA.L RenderBase,A6
        JSR     -276(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION MapRGBArrayA(engine : POINTER; rgb : pULONG; width : WORD; height : WORD; chunky : pByte; taglist : pTagItem) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L engine,A0
        MOVEA.L rgb,A1
        MOVE.L  width,D0
        MOVE.L  height,D1
        MOVEA.L chunky,A2
        MOVEA.L taglist,A3
        MOVEA.L RenderBase,A6
        JSR     -258(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE MixAlphaChannelA(source1 : pULONG; source2 : pULONG; width : WORD; height : WORD; dest : pULONG; taglist : pTagItem);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L source1,A0
        MOVEA.L source2,A1
        MOVE.L  width,D0
        MOVE.L  height,D1
        MOVEA.L dest,A2
        MOVEA.L taglist,A3
        MOVEA.L RenderBase,A6
        JSR     -318(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE MixRGBArrayA(sourcearray : pULONG; width : WORD; height : WORD; destarray : pULONG; ratio : WORD; taglist : pTagItem);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L sourcearray,A0
        MOVE.L  width,D0
        MOVE.L  height,D1
        MOVEA.L destarray,A1
        MOVE.L  ratio,D2
        MOVEA.L taglist,A2
        MOVEA.L RenderBase,A6
        JSR     -300(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE Planar2ChunkyA(planetab : pPLANEPTR; bytewidth : WORD; height : WORD; depth : WORD; bytesperrow : WORD; chunky : pByte; taglist : pTagItem);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L planetab,A0
        MOVE.L  bytewidth,D0
        MOVE.L  height,D1
        MOVE.L  depth,D2
        MOVE.L  bytesperrow,D3
        MOVEA.L chunky,A1
        MOVEA.L taglist,A2
        MOVEA.L RenderBase,A6
        JSR     -126(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION QueryHistogram(histogram : POINTER; d0arg : longword) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L histogram,A0
        MOVE.L  d0arg,D0
        MOVEA.L RenderBase,A6
        JSR     -090(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION RenderA(rgb : pULONG; width : WORD; height : WORD; chunky : pByte; palette : POINTER; taglist : pTagItem) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L rgb,A0
        MOVE.L  width,D0
        MOVE.L  height,D1
        MOVEA.L chunky,A1
        MOVEA.L palette,A2
        MOVEA.L taglist,A3
        MOVEA.L RenderBase,A6
        JSR     -120(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION RGBArrayDiversityA(rgb : pULONG; width : WORD; height : WORD; taglist : pTagItem) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L rgb,A0
        MOVE.L  width,D0
        MOVE.L  height,D1
        MOVEA.L taglist,A1
        MOVEA.L RenderBase,A6
        JSR     -264(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION ScaleA(engine : POINTER; source : POINTER; dest : POINTER; taglist : pTagItem) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L engine,A0
        MOVEA.L source,A1
        MOVEA.L dest,A2
        MOVEA.L taglist,A3
        MOVEA.L RenderBase,A6
        JSR     -156(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION ScaleOrdinate(source : WORD; dest : WORD; ordinate : WORD) : WORD;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  source,D0
        MOVE.L  dest,D1
        MOVE.L  ordinate,D2
        MOVEA.L RenderBase,A6
        JSR     -228(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION SortPaletteA(palette : POINTER; mode : longword; taglist : pTagItem) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L palette,A0
        MOVE.L  mode,D0
        MOVEA.L taglist,A1
        MOVEA.L RenderBase,A6
        JSR     -216(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE TintRGBArrayA(source : pULONG; width : WORD; height : WORD; RGB : longword; ratio : WORD; dest : pULONG; taglist : pTagItem);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L source,A0
        MOVE.L  width,D0
        MOVE.L  height,D1
        MOVE.L  RGB,D2
        MOVE.L  ratio,D3
        MOVEA.L dest,A1
        MOVEA.L taglist,A2
        MOVEA.L RenderBase,A6
        JSR     -324(A6)
        MOVEA.L (A7)+,A6
  END;
END;

{
 Functions and procedures with array of const go here
}
FUNCTION AddChunkyImage(histogram : POINTER; chunky : pByte; width : WORD; height : WORD; palette : POINTER; const taglist : Array Of Const) : longword;
begin
    AddChunkyImage := AddChunkyImageA(histogram , chunky , width , height , palette , readintags(taglist));
end;

FUNCTION AddHistogram(histogram1 : POINTER; histogram2 : POINTER; const taglist : Array Of Const) : longword;
begin
    AddHistogram := AddHistogramA(histogram1 , histogram2 , readintags(taglist));
end;

FUNCTION AddRGBImage(histogram : POINTER; rgb : pULONG; width : WORD; height : WORD; const taglist : Array Of Const) : longword;
begin
    AddRGBImage := AddRGBImageA(histogram , rgb , width , height , readintags(taglist));
end;

PROCEDURE ApplyAlphaChannel(sourcearray : pULONG; width : WORD; height : WORD; destarray : pULONG; const taglist : Array Of Const);
begin
    ApplyAlphaChannelA(sourcearray , width , height , destarray , readintags(taglist));
end;

PROCEDURE Chunky2BitMap(chunky : pByte; sx : WORD; sy : WORD; width : WORD; height : WORD; bitmap : pBitMap; dx : WORD; dy : WORD; const taglist : Array Of Const);
begin
    Chunky2BitMapA(chunky , sx , sy , width , height , bitmap , dx , dy , readintags(taglist));
end;

FUNCTION Chunky2RGB(chunky : pByte; width : WORD; height : WORD; rgb : pULONG; palette : POINTER; const taglist : Array Of Const) : longword;
begin
    Chunky2RGB := Chunky2RGBA(chunky , width , height , rgb , palette , readintags(taglist));
end;

FUNCTION ChunkyArrayDiversity(chunky : pByte; palette : POINTER; width : WORD; height : WORD; const taglist : Array Of Const) : LONGINT;
begin
    ChunkyArrayDiversity := ChunkyArrayDiversityA(chunky , palette , width , height , readintags(taglist));
end;

FUNCTION ConvertChunky(source : pByte; oldpalette : POINTER; width : WORD; height : WORD; dest : pByte; newpalette : POINTER; const taglist : Array Of Const) : longword;
begin
    ConvertChunky := ConvertChunkyA(source , oldpalette , width , height , dest , newpalette , readintags(taglist));
end;

PROCEDURE CreateAlphaArray(rgbarray : pULONG; width : WORD; height : WORD; const taglist : Array Of Const);
begin
    CreateAlphaArrayA(rgbarray , width , height , readintags(taglist));
end;

FUNCTION CreateHistogram(const taglist : Array Of Const) : POINTER;
begin
    CreateHistogram := CreateHistogramA(readintags(taglist));
end;

FUNCTION CreateMapEngine(palette : POINTER; const taglist : Array Of Const) : POINTER;
begin
    CreateMapEngine := CreateMapEngineA(palette , readintags(taglist));
end;

FUNCTION CreatePalette(const taglist : Array Of Const) : POINTER;
begin
    CreatePalette := CreatePaletteA(readintags(taglist));
end;

PROCEDURE CreatePenTable(chunky : pByte; oldpalette : POINTER; width : WORD; height : WORD; newpalette : POINTER; convtab : pByte; const taglist : Array Of Const);
begin
    CreatePenTableA(chunky , oldpalette , width , height , newpalette , convtab , readintags(taglist));
end;

FUNCTION CreateRMHandler(const taglist : Array Of Const) : POINTER;
begin
    CreateRMHandler := CreateRMHandlerA(readintags(taglist));
end;

FUNCTION CreateScaleEngine(sourcewidth : WORD; sourceheight : WORD; destwidth : WORD; destheight : WORD; const taglist : Array Of Const) : POINTER;
begin
    CreateScaleEngine := CreateScaleEngineA(sourcewidth , sourceheight , destwidth , destheight , readintags(taglist));
end;

PROCEDURE ExportPalette(palette : POINTER; coltab : POINTER; const taglist : Array Of Const);
begin
    ExportPaletteA(palette , coltab , readintags(taglist));
end;

PROCEDURE ExtractAlphaChannel(rgbarray : pULONG; width : WORD; height : WORD; chunkyarray : pByte; const taglist : Array Of Const);
begin
    ExtractAlphaChannelA(rgbarray , width , height , chunkyarray , readintags(taglist));
end;

FUNCTION ExtractPalette(histogram : POINTER; palette : pULONG; numcolors : WORD; const taglist : Array Of Const) : longword;
begin
    ExtractPalette := ExtractPaletteA(histogram , palette , numcolors , readintags(taglist));
end;

PROCEDURE ImportPalette(palette : POINTER; coltab : POINTER; numcols : WORD; const taglist : Array Of Const);
begin
    ImportPaletteA(palette , coltab , numcols , readintags(taglist));
end;

PROCEDURE InsertAlphaChannel(maskarray : pByte; width : WORD; height : WORD; rgbarray : pULONG; const taglist : Array Of Const);
begin
    InsertAlphaChannelA(maskarray , width , height , rgbarray , readintags(taglist));
end;

FUNCTION MapChunkyArray(engine : POINTER; source : pByte; palette : POINTER; width : WORD; height : WORD; dest : pByte; const taglist : Array Of Const) : longword;
begin
    MapChunkyArray := MapChunkyArrayA(engine , source , palette , width , height , dest , readintags(taglist));
end;

FUNCTION MapRGBArray(engine : POINTER; rgb : pULONG; width : WORD; height : WORD; chunky : pByte; const taglist : Array Of Const) : longword;
begin
    MapRGBArray := MapRGBArrayA(engine , rgb , width , height , chunky , readintags(taglist));
end;

PROCEDURE MixAlphaChannel(source1 : pULONG; source2 : pULONG; width : WORD; height : WORD; dest : pULONG; const taglist : Array Of Const);
begin
    MixAlphaChannelA(source1 , source2 , width , height , dest , readintags(taglist));
end;

PROCEDURE MixRGBArray(sourcearray : pULONG; width : WORD; height : WORD; destarray : pULONG; ratio : WORD; const taglist : Array Of Const);
begin
    MixRGBArrayA(sourcearray , width , height , destarray , ratio , readintags(taglist));
end;

PROCEDURE Planar2Chunky(planetab : pPLANEPTR; bytewidth : WORD; height : WORD; depth : WORD; bytesperrow : WORD; chunky : pByte; const taglist : Array Of Const);
begin
    Planar2ChunkyA(planetab , bytewidth , height , depth , bytesperrow , chunky , readintags(taglist));
end;

FUNCTION RenderTags(rgb : pULONG; width : WORD; height : WORD; chunky : pByte; palette : POINTER; const taglist : Array Of Const) : longword;
begin
    RenderTags := RenderA(rgb , width , height , chunky , palette , readintags(taglist));
end;

FUNCTION RGBArrayDiversity(rgb : pULONG; width : WORD; height : WORD; const taglist : Array Of Const) : LONGINT;
begin
    RGBArrayDiversity := RGBArrayDiversityA(rgb , width , height , readintags(taglist));
end;

FUNCTION Scale(engine : POINTER; source : POINTER; dest : POINTER; const taglist : Array Of Const) : longword;
begin
    Scale := ScaleA(engine , source , dest , readintags(taglist));
end;

FUNCTION SortPalette(palette : POINTER; mode : longword; const taglist : Array Of Const) : longword;
begin
    SortPalette := SortPaletteA(palette , mode , readintags(taglist));
end;

PROCEDURE TintRGBArray(source : pULONG; width : WORD; height : WORD; RGB : longword; ratio : WORD; dest : pULONG; const taglist : Array Of Const);
begin
    TintRGBArrayA(source , width , height , RGB , ratio , dest , readintags(taglist));
end;

const
    { Change VERSION and LIBVERSION to proper values }

    VERSION : string[2] = '0';
    LIBVERSION : longword = 0;

{$ifdef use_init_openlib}
  {$Info Compiling initopening of render.library}
  {$Info don't forget to use InitRENDERLibrary in the beginning of your program}

var
    render_exit : Pointer;

procedure CloserenderLibrary;
begin
    ExitProc := render_exit;
    if RenderBase <> nil then begin
        CloseLibrary(RenderBase);
        RenderBase := nil;
    end;
end;

procedure InitRENDERLibrary;
begin
    RenderBase := nil;
    RenderBase := OpenLibrary(RENDERNAME,LIBVERSION);
    if RenderBase <> nil then begin
        render_exit := ExitProc;
        ExitProc := @CloserenderLibrary;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open render.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;
end;

begin
    RENDERIsCompiledHow := 2;
{$endif use_init_openlib}

{$ifdef use_auto_openlib}
  {$Info Compiling autoopening of render.library}

var
    render_exit : Pointer;

procedure CloserenderLibrary;
begin
    ExitProc := render_exit;
    if RenderBase <> nil then begin
        CloseLibrary(RenderBase);
        RenderBase := nil;
    end;
end;

begin
    RenderBase := nil;
    RenderBase := OpenLibrary(RENDERNAME,LIBVERSION);
    if RenderBase <> nil then begin
        render_exit := ExitProc;
        ExitProc := @CloserenderLibrary;
        RENDERIsCompiledHow := 1;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open render.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;

{$endif use_auto_openlib}

{$ifdef dont_use_openlib}
begin
    RENDERIsCompiledHow := 3;
   {$Warning No autoopening of render.library compiled}
   {$Warning Make sure you open render.library yourself}
{$endif dont_use_openlib}


END. (* UNIT RENDER *)



