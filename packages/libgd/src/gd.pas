{
  Translation of the libgd headers for FreePascal
  Copyright(C) 2009 by Ivo Steinmann
}

unit gd;

{$MODE objfpc}
{$MACRO on}
{$H+}
{$MINENUMSIZE 4}


{$DEFINE FPC_TARGET_SUPPORTS_DYNLIBS}

{$IFDEF GO32V2}
  {$UNDEF FPC_TARGET_SUPPORTS_DYNLIBS}
{$ENDIF GO32V2}
{$IFDEF AMIGA}
  {$UNDEF FPC_TARGET_SUPPORTS_DYNLIBS}
{$ENDIF AMIGA}
{$IFDEF MORPHOS}
  {$UNDEF FPC_TARGET_SUPPORTS_DYNLIBS}
{$ENDIF MORPHOS}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$IFDEF AROS}
  {$UNDEF FPC_TARGET_SUPPORTS_DYNLIBS}
{$ENDIF AROS}
{$IFDEF GBA}
  {$UNDEF FPC_TARGET_SUPPORTS_DYNLIBS}
{$ENDIF GBA}
{$IFDEF NDS}
  {$UNDEF FPC_TARGET_SUPPORTS_DYNLIBS}
{$ENDIF NDS}
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew

interface

uses
  Classes,
{$IFDEF FPC_TARGET_SUPPORTS_DYNLIBS}
  dynlibs,
{$ENDIF FPC_TARGET_SUPPORTS_DYNLIBS}
  ctypes;

(* cdecl as default unless defined differently below *)
{$DEFINE EXTDECL := cdecl}

{$IFDEF UNIX}
  {$DEFINE EXTDECL := cdecl}
  const
    gdlib = 'libgd.'+sharedsuffix;
    clib = 'libc.'+sharedsuffix;
{$ENDIF}
{$IFDEF NETWARE}
  {$DEFINE EXTDECL := cdecl}
  const
    gdlib = 'libgd.nlm';
    clib = 'clib.nlm';
{$ENDIF}
{$IFDEF NETWLIBC}
  {$DEFINE EXTDECL := cdecl}
  const
    gdlib = 'libgd.nlm';
    clib = 'libc.nlm';
{$ENDIF}
{$IFDEF WINDOWS}
  {$DEFINE EXTDECL := stdcall}
  const
    gdlib = 'bgd.dll';
    clib = 'msvcrt.dll';
{$ENDIF}
{$IFDEF GO32V2}
  {$DEFINE EXTDECL := cdecl}
    {$DEFINE gdlib := }
    {$DEFINE clib := }
    {$linklib gd}
    {$linklib c}
   {$UNDEF LOAD_DYNAMICALLY}
{$ENDIF GO32V2}
{$IFDEF OS2}
(* Force static linking under OS/2 for now to avoid     *)
(* dependency on dll for a one particular libc version. *)
  {$UNDEF LOAD_DYNAMICALLY}
  {$DEFINE gdlib := }
  {$DEFINE clib := }
{$ENDIF OS2}
{$IFDEF AMIGA}
  {$UNDEF LOAD_DYNAMICALLY}
  {$DEFINE gdlib := }
  {$DEFINE clib := }
{$ENDIF AMIGA}
{$IFDEF MORPHOS}
  {$UNDEF LOAD_DYNAMICALLY}
  {$DEFINE gdlib := }
  {$DEFINE clib := }
{$ENDIF MORPHOS}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$IFDEF AROS}
  {$UNDEF LOAD_DYNAMICALLY}
  {$DEFINE gdlib := }
  {$DEFINE clib := }
{$ENDIF AROS}
{$IFDEF GBA}
  {$UNDEF LOAD_DYNAMICALLY}
  {$DEFINE gdlib := }
  {$DEFINE clib := }
{$ENDIF GBA}
{$IFDEF NDS}
  {$UNDEF LOAD_DYNAMICALLY}
  {$DEFINE gdlib := }
  {$DEFINE clib := }
{$ENDIF NDS}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{$IFNDEF LOAD_DYNAMICALLY}
  {$IFDEF darwin}
    {$linklib c}
    {$linklib gd}
  {$ENDIF}
{$ENDIF}

type
  ppcint = ^pcint;
  PFILE = pointer;

const
  GD_MAJOR_VERSION = 2;
  GD_MINOR_VERSION = 0;
  GD_RELEASE_VERSION = 35;
  GD_EXTRA_VERSION = '';
  GD_VERSION_STRING = '2.0.35';


(* gd.h: declarations file for the graphic-draw module.
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose and without fee is hereby granted, provided
 * that the above copyright notice appear in all copies and that both that
 * copyright notice and this permission notice appear in supporting
 * documentation.  This software is provided "AS IS." Thomas Boutell and
 * Boutell.Com, Inc. disclaim all warranties, either express or implied,
 * including but not limited to implied warranties of merchantability and
 * fitness for a particular purpose, with respect to this code and accompanying
 * documentation. *)

(* stdio is needed for file I/O. *)
//#include <stdio.h>
//#include "gd_io.h"
type
  gdIOCtxPtr = ^gdIOCtx;
  gdIOCtx = record
    getC    : function(ctx: gdIOCtxPtr): cint; EXTDECL;
    getBuf  : function(ctx: gdIOCtxPtr; buf: pointer; len: cint): cint; EXTDECL;
    putC    : procedure(ctx: gdIOCtxPtr; len: cint); EXTDECL;
    putBuf  : procedure(ctx: gdIOCtxPtr; buf: pointer; len: cint); EXTDECL;
    (* seek must return 1 on SUCCESS, 0 on FAILURE. Unlike fseek! *)
    seek    : function(ctx: gdIOCtxPtr; pos: cint): cint; EXTDECL;
    tell    : function(ctx: gdIOCtxPtr): clong; EXTDECL;
    gd_free : procedure(ctx: gdIOCtxPtr); EXTDECL;
  end;

function fopen(filename, rights: pchar): PFile; EXTDECL; external clib;
procedure fclose(f: PFile); EXTDECL; external clib;

(* The maximum number of palette entries in palette-based images.
  In the wonderful new world of gd 2.0, you can of course have
  many more colors when using truecolor mode. *)

const
  gdMaxColors = 256;

(* Image type. See functions below; you will not need to change
  the elements directly. Use the provided macros to
  access sx, sy, the color table, and colorsTotal for
  read-only purposes. *)

(* If 'truecolor' is set true, the image is truecolor;
  pixels are represented by integers, which
  must be 32 bits wide or more.

  True colors are repsented as follows:

  ARGB

  Where 'A'(alpha channel) occupies only the
  LOWER 7 BITS of the MSB. This very small
  loss of alpha channel resolution allows gd 2.x
  to keep backwards compatibility by allowing
  signed integers to be used to represent colors,
  and negative numbers to represent special cases,
  just as in gd 1.x. *)

const
  gdAlphaMax = 127;
  gdAlphaOpaque = 0;
  gdAlphaTransparent = 127;
  gdRedMax = 255;
  gdGreenMax = 255;
  gdBlueMax = 255;

function gdTrueColorGetAlpha(c: cint): cint; inline;
function gdTrueColorGetRed(c: cint): cint; inline;
function gdTrueColorGetGreen(c: cint): cint; inline;
function gdTrueColorGetBlue(c: cint): cint; inline;

(* This function accepts truecolor pixel values only. The
  source color is composited with the destination color
  based on the alpha channel value of the source color.
  The resulting color is opaque. *)

function gdAlphaBlend(dest: cint; src: cint): cint; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdAlphaBlend@8'{$ENDIF};

type
  gdImagePtr = ^gdImage;
  gdImage = record

    (* Palette-based image pixels *)
    pixels: ppbyte;
    sx: cint;
    sy: cint;
    (* These are valid in palette images only. See also
       'alpha', which appears later in the structure to
       preserve binary backwards compatibility *)
    colorsTotal: cint;
    red: array[0..gdMaxColors-1] of cint;
    green: array[0..gdMaxColors-1] of cint;
    blue: array[0..gdMaxColors-1] of cint;
    open: array[0..gdMaxColors-1] of cint;
    (* For backwards compatibility, this is set to the
       first palette entry with 100% transparency,
       and is also set and reset by the
       gdImageColorTransparent function. Newer
       applications can allocate palette entries
       with any desired level of transparency; however,
       bear in mind that many viewers, notably
       many web browsers, fail to implement
       full alpha channel for PNG and provide
       support for full opacity or transparency only. *)
    transparent: cint;
    polyInts: pcint;
    polyAllocated: cint;
    brush: gdImagePtr;
    tile: gdImagePtr;
    brushColorMap: array[0..gdMaxColors-1] of cint;
    tileColorMap: array[0..gdMaxColors-1] of cint;
    styleLength: cint;
    stylePos: cint;
    style: pcint;
    interlace: cint;
    (* New in 2.0: thickness of line. Initialized to 1. *)
    thick: cint;
    (* New in 2.0: alpha channel for palettes. Note that only
       Macintosh Internet Explorer and(possibly) Netscape 6
       really support multiple levels of transparency in
       palettes, to my knowledge, as of 2/15/01. Most
       common browsers will display 100% opaque and
       100% transparent correctly, and do something
       unpredictable and/or undesirable for levels
       in between. TBB *)
    alpha: array[0..gdMaxColors-1] of cint;
    (* Truecolor flag and pixels. New 2.0 fields appear here at the
       end to minimize breakage of existing object code. *)
    trueColor: cint;
    tpixels: ppcint;
    (* Should alpha channel be copied, or applied, each time a
       pixel is drawn? This applies to truecolor images only.
       No attempt is made to alpha-blend in palette images,
       even if semitransparent palette entries exist.
       To do that, build your image as a truecolor image,
       then quantize down to 8 bits. *)
    alphaBlendingFlag: cint;
    (* Should the alpha channel of the image be saved? This affects
       PNG at the moment; other future formats may also
       have that capability. JPEG doesn't. *)
    saveAlphaFlag: cint;

    (* There should NEVER BE ACCESSOR MACROS FOR ITEMS BELOW HERE, so this
       part of the structure can be safely changed in new releases. *)

    (* 2.0.12: anti-aliased globals. 2.0.26: just a few vestiges after
      switching to the fast, memory-cheap implementation from PHP-gd. *)
    AA: cint;
    AA_color: cint;
    AA_dont_blend: cint;

   (* 2.0.12: simple clipping rectangle. These values
      must be checked for safety when set; please use
      gdImageSetClip *)
    cx1: cint;
    cy1: cint;
    cx2: cint;
    cy2: cint;
  end;

  gdFontPtr = ^gdFont;
  gdFont = record
    (* # of characters in font *)
    nchars: cint;
    (* First character is numbered...(usually 32 = space) *)
    offset: cint;
    (* Character width and height *)
    w: cint;
    h: cint;
    (* Font data; array of characters, one row after another.
       Easily included in code, also easily loaded from
       data files. *)
    data: pchar;
  end;


(* For backwards compatibility only. Use gdImageSetStyle()
  for MUCH more flexible line drawing. Also see
  gdImageSetBrush(). *)
const
  gdDashSize = 4;

(* Special colors. *)

  gdStyled =(-2);
  gdBrushed =(-3);
  gdStyledBrushed =(-4);
  gdTiled =(-5);

(* NOT the same as the transparent color index.
  This is used in line styles only. *)
  gdTransparent =(-6);

  gdAntiAliased =(-7);

(* Functions to manipulate images. *)

(* Creates a palette-based image(up to 256 colors). *)
function gdImageCreate(sx: cint; sy: cint): gdImagePtr; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageCreate@8'{$ENDIF};

(* An alternate name for the above(2.0). *)
function gdImageCreatePalette(sx: cint; sy: cint): gdImagePtr;

(* Creates a truecolor image(millions of colors). *)
function gdImageCreateTrueColor(sx: cint; sy: cint): gdImagePtr; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageCreateTrueColor@8'{$ENDIF};

(* Creates an image from various file types. These functions
  return a palette or truecolor image based on the
  nature of the file being loaded. Truecolor PNG
  stays truecolor; palette PNG stays palette-based;
  JPEG is always truecolor. *)
function gdImageCreateFromPng(fd: PFILE): gdImagePtr; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageCreateFromPng@4'{$ENDIF};
function gdImageCreateFromPngCtx(_in: gdIOCtxPtr): gdImagePtr; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageCreateFromPngCtx@4'{$ENDIF};
function gdImageCreateFromPngPtr(size: cint; data: pointer): gdImagePtr; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageCreateFromPngPtr@8'{$ENDIF};

(* These read the first frame only *)
function gdImageCreateFromGif(fd: PFILE): gdImagePtr; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageCreateFromGif@4'{$ENDIF};
function gdImageCreateFromGifCtx(_in: gdIOCtxPtr): gdImagePtr; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageCreateFromGifCtx@4'{$ENDIF};
function gdImageCreateFromGifPtr(size: cint; data: pointer): gdImagePtr; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageCreateFromGifPtr@8'{$ENDIF};
function gdImageCreateFromWBMP(fd: PFILE): gdImagePtr; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageCreateFromWBMP@4'{$ENDIF};
function gdImageCreateFromWBMPCtx(_in: gdIOCtxPtr): gdImagePtr; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageCreateFromWBMPCtx@4'{$ENDIF};
function gdImageCreateFromWBMPPtr(size: cint; data: pointer): gdImagePtr; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageCreateFromWBMPPtr@8'{$ENDIF};
function gdImageCreateFromJpeg(fd: PFILE): gdImagePtr; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageCreateFromJpeg@4'{$ENDIF};
function gdImageCreateFromJpegCtx(_in: gdIOCtxPtr): gdImagePtr; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageCreateFromJpegCtx@4'{$ENDIF};
function gdImageCreateFromJpegPtr(size: cint; data: pointer): gdImagePtr; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageCreateFromJpegPtr@8'{$ENDIF};

(* A custom data source. *)
(* The source function must return -1 on error, otherwise the number
        of bytes fetched. 0 is EOF, not an error! *)
(* context will be passed to your source function. *)

type
  gdSourcePtr = ^gdSource;
  gdSource = record
    source  : function(context: pointer; buffer: pchar; len: cint): cint; EXTDECL;
    context : pointer;
  end;

  (* Deprecated in favor of gdImageCreateFromPngCtx *)
function gdImageCreateFromPngSource(_in: gdSourcePtr): gdImagePtr; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageCreateFromPngSource@4'{$ENDIF};

function gdImageCreateFromGd(_in: PFILE): gdImagePtr; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageCreateFromGd@4'{$ENDIF};
function gdImageCreateFromGdCtx(_in: gdIOCtxPtr): gdImagePtr; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageCreateFromGdCtx@4'{$ENDIF};
function gdImageCreateFromGdPtr(size: cint; data: pointer): gdImagePtr; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageCreateFromGdPtr@8'{$ENDIF};

function gdImageCreateFromGd2(_in: PFILE): gdImagePtr; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageCreateFromGd2@4'{$ENDIF};
function gdImageCreateFromGd2Ctx(_in: gdIOCtxPtr): gdImagePtr; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageCreateFromGd2Ctx@4'{$ENDIF};
function gdImageCreateFromGd2Ptr(size: cint; data: pointer): gdImagePtr; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageCreateFromGd2Ptr@8'{$ENDIF};

function gdImageCreateFromGd2Part(_in: PFILE; srcx: cint; srcy: cint; w: cint; h: cint): gdImagePtr; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageCreateFromGd2Part@20'{$ENDIF};
function gdImageCreateFromGd2PartCtx(_in: gdIOCtxPtr; srcx: cint; srcy: cint; w: cint; h: cint): gdImagePtr; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageCreateFromGd2PartCtx@20'{$ENDIF};
function gdImageCreateFromGd2PartPtr(size: cint; data: pointer; srcx: cint; srcy: cint; w: cint; h: cint): gdImagePtr; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageCreateFromGd2PartCtx@24'{$ENDIF};
 (* 2.0.10: prototype was missing *)
function gdImageCreateFromXbm(_in: PFILE): gdImagePtr; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageCreateFromXbm@4'{$ENDIF};

 (* NOTE: filename, not FILE *)
function gdImageCreateFromXpm(filename: pchar): gdImagePtr; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageCreateFromXpm@4'{$ENDIF};

procedure gdImageDestroy(im: gdImagePtr); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageDestroy@4'{$ENDIF};

(* Replaces or blends with the background depending on the
  most recent call to gdImageAlphaBlending and the
  alpha channel value of 'color'; default is to overwrite.
  Tiling and line styling are also implemented
  here. All other gd drawing functions pass through this call,
  allowing for many useful effects. *)

procedure gdImageSetPixel(im: gdImagePtr; x: cint; y: cint; color: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageSetPixel@16'{$ENDIF};
(* FreeType 2 text output with hook to extra flags *)

function gdImageGetPixel(im: gdImagePtr; x: cint; y: cint): cint; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageGetPixel@12'{$ENDIF};
function gdImageGetTrueColorPixel(im: gdImagePtr; x: cint; y: cint): cint; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageGetTrueColorPixel@12'{$ENDIF};

procedure gdImageAABlend(im: gdImagePtr); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageAABlend@4'{$ENDIF};

procedure gdImageLine(im: gdImagePtr; x1: cint; y1: cint; x2: cint; y2: cint; color: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageLine@24'{$ENDIF};

(* For backwards compatibility only. Use gdImageSetStyle()
  for much more flexible line drawing. *)
procedure gdImageDashedLine(im: gdImagePtr; x1: cint; y1: cint; x2: cint; y2: cint; color: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageDashedLine@24'{$ENDIF};
(* Corners specified(not width and height). Upper left first, lower right
  second. *)
procedure gdImageRectangle(im: gdImagePtr; x1: cint; y1: cint; x2: cint; y2: cint; color: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageRectangle@24'{$ENDIF};
(* Solid bar. Upper left corner first, lower right corner second. *)
procedure gdImageFilledRectangle(im: gdImagePtr; x1: cint; y1: cint; x2: cint; y2: cint; color: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageFilledRectangle@24'{$ENDIF};
procedure gdImageSetClip(im: gdImagePtr; x1: cint; y1: cint; x2: cint; y2: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageSetClip@20'{$ENDIF};
procedure gdImageGetClip(im: gdImagePtr; var x1: cint; var y1: cint; var x2: cint; var y2: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageGetClip@20'{$ENDIF};
function gdImageBoundsSafe(im: gdImagePtr; x: cint; y: cint): cint; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageBoundsSafe@12'{$ENDIF};
procedure gdImageChar(im: gdImagePtr; f: gdFontPtr; x: cint; y: cint; c: cint; color: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageChar@24'{$ENDIF};
procedure gdImageCharUp(im: gdImagePtr; f: gdFontPtr; x: cint; y: cint; c: cint; color: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageCharUp@24'{$ENDIF};
procedure gdImageString(im: gdImagePtr; f: gdFontPtr; x: cint; y: cint; s: pchar; color: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageString@24'{$ENDIF};
procedure gdImageStringUp(im: gdImagePtr; f: gdFontPtr; x: cint; y: cint; s: pchar; color: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageStringUp@24'{$ENDIF};
procedure gdImageString16(im: gdImagePtr; f: gdFontPtr; x: cint; y: cint; s: pwidechar; color: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageString16@24'{$ENDIF};
procedure gdImageStringUp16(im: gdImagePtr; f: gdFontPtr; x: cint; y: cint; s: pwidechar; color: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageStringUp16@24'{$ENDIF};

(* 2.0.16: for thread-safe use of gdImageStringFT and friends,
  call this before allowing any thread to call gdImageStringFT.
  Otherwise it is invoked by the first thread to invoke
  gdImageStringFT, with a very small but real risk of a race condition.
  Return 0 on success, nonzero on failure to initialize freetype. *)
function gdFontCacheSetup(): cint; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdFontCacheSetup@0'{$ENDIF};

(* Optional: clean up after application is done using fonts in
BGD_DECLARE( ) gdImageStringFT(). *)
procedure gdFontCacheShutdown(); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdFontCacheShutdown@0'{$ENDIF};
(* 2.0.20: for backwards compatibility. A few applications did start calling
 this function when it first appeared although it was never documented.
 Simply invokes gdFontCacheShutdown. *)
procedure gdFreeFontCache(); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdFreeFontCache@0'{$ENDIF};

(* Calls gdImageStringFT. Provided for backwards compatibility only. *)
function gdImageStringTTF(im: gdImagePtr; brect: pcint; fg: cint; fontlist: pchar; ptsize: double; angle: double; x: cint; y: cint; str: pchar): pchar; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageStringTTF@36'{$ENDIF};

(* FreeType 2 text output *)
function gdImageStringFT(im: gdImagePtr; brect: pcint; fg: cint; fontlist: pchar; ptsize: double; angle: double; x: cint; y: cint; str: pchar): pchar; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageStringFT@36'{$ENDIF};

(* 2.0.5: provides an extensible way to pass additional parameters.
  Thanks to Wez Furlong, sorry for the delay. *)

type
  gdFTStringExtraPtr = ^gdFTStringExtra;
  gdFTStringExtra = record
    flags: cint;     (* Logical OR of gdFTEX_ values *)
    linespacing: double;  (* fine tune line spacing for '\n' *)
    charmap: cint;
        (* TBB: 2.0.12: may be gdFTEX_Unicode,
           gdFTEX_Shift_JIS, gdFTEX_Big5,
           or gdFTEX_Adobe_Custom;
           when not specified, maps are searched
           for in the above order. *)
    hdpi: cint;                  (* if(flags & gdFTEX_RESOLUTION) *)
    vdpi: cint;    (* if(flags & gdFTEX_RESOLUTION) *)
    xshow: pchar;
      (* if(flags & gdFTEX_XSHOW)
         then, on return, xshow is a malloc'ed
         string contining xshow position data for
         the last string.

         NB. The caller is responsible for gdFree'ing
         the xshow string.
       *)
    fontpath: pchar;    (* if(flags & gdFTEX_RETURNFONTPATHNAME)
                           then, on return, fontpath is a malloc'ed
                           string containing the actual font file path name
                           used, which can be interesting when fontconfig
                           is in use.

                           The caller is responsible for gdFree'ing the
                           fontpath string.
      *)

  end;

const
  gdFTEX_LINESPACE = 1;
  gdFTEX_CHARMAP = 2;
  gdFTEX_RESOLUTION = 4;
  gdFTEX_DISABLE_KERNING = 8;
  gdFTEX_XSHOW = 16;
(* The default unless gdFTUseFontConfig(1); has been called:
  fontlist is a full or partial font file pathname or list thereof
 (i.e. just like before 2.0.29) *)
  gdFTEX_FONTPATHNAME = 32;
(* Necessary to use fontconfig patterns instead of font pathnames
  as the fontlist argument, unless gdFTUseFontConfig(1); has
  been called. New in 2.0.29 *)
  gdFTEX_FONTCONFIG = 64;
(* Sometimes interesting when fontconfig is used: the fontpath
  element of the structure above will contain a gdMalloc'd string
  copy of the actual font file pathname used, if this flag is set
   when the call is made *)
  gdFTEX_RETURNFONTPATHNAME = 128;

(* If flag is nonzero, the fontlist parameter to gdImageStringFT
  and gdImageStringFTEx shall be assumed to be a fontconfig font pattern
  if fontconfig was compiled into gd. This function returns zero
  if fontconfig is not available, nonzero otherwise. *)
function gdFTUseFontConfig(flag: cint): cint; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdFTUseFontConfig@4'{$ENDIF};

(* These are NOT flags; set one in 'charmap' if you set the
  gdFTEX_CHARMAP bit in 'flags'. *)
const
  gdFTEX_Unicode = 0;
  gdFTEX_Shift_JIS = 1;
  gdFTEX_Big5 = 2;
  gdFTEX_Adobe_Custom = 3;

function gdImageStringFTEx(im: gdImagePtr; brect: pcint; fg: cint; fontlist: pchar; ptsize: double; angle: double; x: cint; y: cint; str: pchar; strex: gdFTStringExtraPtr): pchar; EXTDECL;
           external gdlib {$IFDEF WINDOWS}name '_gdImageStringFTEx@40'{$ENDIF};

(* Point type for use in polygon drawing. *)
type
  gdPointPtr = ^gdPoint;
  gdPoint = record
    x, y: cint;
  end;

procedure gdImagePolygon(im: gdImagePtr; p: gdPointPtr; n: cint; c: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImagePolygon@16'{$ENDIF};
procedure gdImageOpenPolygon(im: gdImagePtr; p: gdPointPtr; n: cint; c: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageOpenPolygon@16'{$ENDIF};
procedure gdImageFilledPolygon(im: gdImagePtr; p: gdPointPtr; n: cint; c: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageFilledPolygon@16'{$ENDIF};

(* These functions still work with truecolor images,
  for which they never return error. *)
function gdImageColorAllocate(im: gdImagePtr; r: cint; g: cint; b: cint): cint; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageColorAllocate@16'{$ENDIF};
(* gd 2.0: palette entries with non-opaque transparency are permitted. *)
function gdImageColorAllocateAlpha(im: gdImagePtr; r: cint; g: cint; b: cint; a: cint): cint; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageColorAllocateAlpha@20'{$ENDIF};
(* Assumes opaque is the preferred alpha channel value *)
function gdImageColorClosest(im: gdImagePtr; r: cint; g: cint; b: cint): cint; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageColorClosest@16'{$ENDIF};
(* Closest match taking all four parameters into account.
  A slightly different color with the same transparency
  beats the exact same color with radically different
  transparency *)
function gdImageColorClosestAlpha(im: gdImagePtr; r: cint; g: cint; b: cint; a: cint): cint; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageColorClosestAlpha@20'{$ENDIF};
(* An alternate method *)
function gdImageColorClosestHWB(im: gdImagePtr; r: cint; g: cint; b: cint): cint; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageColorClosestHWB@16'{$ENDIF};
(* Returns exact, 100% opaque matches only *)
function gdImageColorExact(im: gdImagePtr; r: cint; g: cint; b: cint): cint; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageColorExact@16'{$ENDIF};
(* Returns an exact match only, including alpha *)
function gdImageColorExactAlpha(im: gdImagePtr; r: cint; g: cint; b: cint; a: cint): cint; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageColorExactAlpha@20'{$ENDIF};
(* Opaque only *)
function gdImageColorResolve(im: gdImagePtr; r: cint; g: cint; b: cint): cint; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageColorResolve@16'{$ENDIF};
(* Based on gdImageColorExactAlpha and gdImageColorClosestAlpha *)
function gdImageColorResolveAlpha(im: gdImagePtr; r: cint; g: cint; b: cint; a: cint): cint; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageColorResolveAlpha@20'{$ENDIF};

(* A simpler way to obtain an opaque truecolor value for drawing on a
  truecolor image. Not for use with palette images! *)
function gdTrueColor(r: cint; g: cint; b: cint): cint;

(* Returns a truecolor value with an alpha channel component.
  gdAlphaMax(127, **NOT 255** ) is transparent, 0 is completely
  opaque. *)
function gdTrueColorAlpha(r: cint; g: cint; b: cint; a: cint): cint;

procedure gdImageColorDeallocate(im: gdImagePtr; color: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageColorDeallocate@8'{$ENDIF};

(* Converts a truecolor image to a palette-based image,
  using a high-quality two-pass quantization routine
  which attempts to preserve alpha channel information
  as well as R/G/B color information when creating
  a palette. If ditherFlag is set, the image will be
  dithered to approximate colors better, at the expense
  of some obvious "speckling." colorsWanted can be
  anything up to 256. If the original source image
  includes photographic information or anything that
  came out of a JPEG, 256 is strongly recommended.

  Better yet, don't use these function -- write real
  truecolor PNGs and JPEGs. The disk space gain of
        conversion to palette is not great(for small images
        it can be negative) and the quality loss is ugly.

  DIFFERENCES: gdImageCreatePaletteFromTrueColor creates and
  returns a new image. gdImageTrueColorToPalette modifies
  an existing image, and the truecolor pixels are discarded. *)

function gdImageCreatePaletteFromTrueColor(im: gdImagePtr; ditherFlag: cint; colorsWanted: cint): gdImagePtr; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageCreatePaletteFromTrueColor@16'{$ENDIF};

procedure gdImageTrueColorToPalette(im: gdImagePtr; ditherFlag: cint; colorsWanted: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageTrueColorToPalette@16'{$ENDIF};

(* Specifies a color index(if a palette image) or an
  RGB color(if a truecolor image) which should be
  considered 100% transparent. FOR TRUECOLOR IMAGES,
  THIS IS IGNORED IF AN ALPHA CHANNEL IS BEING
  SAVED. Use gdImageSaveAlpha(im, 0); to
  turn off the saving of a full alpha channel in
  a truecolor image. Note that gdImageColorTransparent
  is usually compatible with older browsers that
  do not understand full alpha channels well. TBB *)
procedure gdImageColorTransparent(im: gdImagePtr; color: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageColorTransparent@8'{$ENDIF};

procedure gdImagePaletteCopy(dst: gdImagePtr; src: gdImagePtr); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImagePaletteCopy@8'{$ENDIF};
procedure gdImageGif(im: gdImagePtr; _out: PFILE); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageGif@8'{$ENDIF};
procedure gdImagePng(im: gdImagePtr; _out: PFILE); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImagePng@8'{$ENDIF};
procedure gdImagePngCtx(im: gdImagePtr; _out: gdIOCtxPtr); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImagePngCtx@8'{$ENDIF};
procedure gdImageGifCtx(im: gdImagePtr; _out: gdIOCtxPtr); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageGifCtx@8'{$ENDIF};

(* 2.0.12: Compression level: 0-9 or -1, where 0 is NO COMPRESSION at all,
  1 is FASTEST but produces larger files, 9 provides the best
  compression(smallest files) but takes a long time to compress, and
  -1 selects the default compiled into the zlib library. *)
procedure gdImagePngEx(im: gdImagePtr; _out: PFILE; level: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImagePngEx@12'{$ENDIF};
procedure gdImagePngCtxEx(im: gdImagePtr; _out: gdIOCtxPtr; level: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImagePngCtxEx@12'{$ENDIF};

procedure gdImageWBMP(image: gdImagePtr; fg: cint; _out: PFILE); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageWBMP@12'{$ENDIF};
procedure gdImageWBMPCtx(image: gdImagePtr; fg: cint; _out: gdIOCtxPtr); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageWBMPCtx@12'{$ENDIF};

(* Guaranteed to correctly free memory returned
  by the gdImage*Ptr functions *)
procedure gdFree(m: pointer); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdFree@4'{$ENDIF};

(* Best to free this memory with gdFree(), not free() *)
function gdImageWBMPPtr(im: gdImagePtr; size: pcint; fg: cint): pointer; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageWBMPPtr@12'{$ENDIF};

(* 100 is highest quality(there is always a little loss with JPEG).
  0 is lowest. 10 is about the lowest useful setting. *)
procedure gdImageJpeg(im: gdImagePtr; _out: PFILE; quality: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageJpeg@12'{$ENDIF};
procedure gdImageJpegCtx(im: gdImagePtr; _out: gdIOCtxPtr; quality: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageJpegCtx@12'{$ENDIF};

(* Best to free this memory with gdFree(), not free() *)
function gdImageJpegPtr(im: gdImagePtr; size: pcint; quality: cint): pointer; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageJpegPtr@12'{$ENDIF};

(* Legal values for Disposal. gdDisposalNone is always used by
  the built-in optimizer if previm is passed. *)

(*
enum {
  gdDisposalUnknown,
  gdDisposalNone,
  gdDisposalRestoreBackground,
  gdDisposalRestorePrevious
};

procedure gdImageGifAnimBegin(im: gdImagePtr; _outFile: PFILE; int GlobalCM, int Loops); EXTDECL; BGD_DECLARE;
procedure gdImageGifAnimAdd(im: gdImagePtr; _outFile: PFILE; int LocalCM, int LeftOfs, int TopOfs, int Delay, int Disposal, gdImagePtr previm); EXTDECL; BGD_DECLARE;
procedure gdImageGifAnimEnd(_outFile: PFILE); EXTDECL; BGD_DECLARE;
procedure gdImageGifAnimBeginCtx(im: gdImagePtr; gdIOCtx *out, int GlobalCM, int Loops); EXTDECL; BGD_DECLARE;
procedure gdImageGifAnimAddCtx(im: gdImagePtr; gdIOCtx *out, int LocalCM, int LeftOfs, int TopOfs, int Delay, int Disposal, gdImagePtr previm); EXTDECL; BGD_DECLARE;
procedure gdImageGifAnimEndCtx(gdIOCtx *out); EXTDECL; BGD_DECLARE;
function gdImageGifAnimBeginPtr(im: gdImagePtr; int *size, int GlobalCM, int Loops): pointer; EXTDECL; BGD_DECLARE;
function gdImageGifAnimAddPtr(im: gdImagePtr; int *size, int LocalCM, int LeftOfs, int TopOfs, int Delay, int Disposal, gdImagePtr previm): pointer; EXTDECL; BGD_DECLARE;
function gdImageGifAnimEndPtr(int *size): pointer; EXTDECL; BGD_DECLARE;
*)
{$warning TODO}

(* A custom data sink. For backwards compatibility. Use
  gdIOCtx instead. *)
(* The sink function must return -1 on error, otherwise the number
        of bytes written, which must be equal to len. *)
(* context will be passed to your sink function. *)
type
  gdSinkPtr = ^gdSink;
  gdSink = record
    sink    : function(context: pointer; buffer: pchar; len: cint): cint; EXTDECL;
    context : pointer;
  end;

procedure gdImagePngToSink(im: gdImagePtr; _out: gdSinkPtr); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImagePngToSink@8'{$ENDIF};

procedure gdImageGd(im: gdImagePtr; _out: PFILE); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageGd@8'{$ENDIF};
procedure gdImageGd2(im: gdImagePtr; _out: PFILE; cs: cint; fmt: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageGd2@16'{$ENDIF};

(* Best to free this memory with gdFree(), not free() *)
function gdImageGifPtr(im: gdImagePtr; var size: cint): pointer; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageGifPtr@8'{$ENDIF};

(* Best to free this memory with gdFree(), not free() *)
function gdImagePngPtr(im: gdImagePtr; var size: cint): pointer; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImagePngPtr@8'{$ENDIF};
function gdImagePngPtrEx(im: gdImagePtr; var size: cint; level: cint): pointer; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImagePngPtrEx@12'{$ENDIF};

(* Best to free this memory with gdFree(), not free() *)
function gdImageGdPtr(im: gdImagePtr; var size: cint): pointer; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageGdPtr@8'{$ENDIF};

(* Best to free this memory with gdFree(), not free() *)
function gdImageGd2Ptr(im: gdImagePtr; cs: cint; fmt: cint; var size: cint): pointer; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageGd2Ptr@16'{$ENDIF};


(* Style is a bitwise OR( | operator ) of these.
  gdArc and gdChord are mutually exclusive;
  gdChord just connects the starting and ending
  angles with a straight line, while gdArc produces
  a rounded edge. gdPie is a synonym for gdArc.
  gdNoFill indicates that the arc or chord should be
  outlined, not filled. gdEdged, used together with
  gdNoFill, indicates that the beginning and ending
  angles should be connected to the center; this is
  a good way to outline(rather than fill) a
  'pie slice'. *)
const
  gdArc    = 0;
  gdPie    = gdArc;
  gdChord  = 1;
  gdNoFill = 2;
  gdEdged  = 4;

procedure gdImageFilledArc(im: gdImagePtr; cx: cint; cy: cint; w: cint; h: cint; s: cint; e: cint; color: cint; style: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageFilledArc@36'{$ENDIF};
procedure gdImageArc(im: gdImagePtr; cx: cint; cy: cint; w: cint; h: cint; s: cint; e: cint; color: cint; style: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageArc@36'{$ENDIF};
procedure gdImageEllipse(im: gdImagePtr; cx: cint; cy: cint; w: cint; h: cint; color: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageEllipse@24'{$ENDIF};
procedure gdImageFilledEllipse(im: gdImagePtr; cx: cint; cy: cint; w: cint; h: cint; color: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageFilledEllipse@24'{$ENDIF};
procedure gdImageFillToBorder(im: gdImagePtr; cx: cint; cy: cint; border: cint; color: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageFillToBorder@20'{$ENDIF};
procedure gdImageFill(im: gdImagePtr; x: cint; y: cint; color: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageFill@16'{$ENDIF};
procedure gdImageCopy(dst: gdImagePtr; src: gdImagePtr; dstX: cint; dstY: cint; srcX: cint; srcY: cint; w: cint; h: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageCopy@32'{$ENDIF};
procedure gdImageCopyMerge(dst: gdImagePtr; src: gdImagePtr; dstX: cint; dstY: cint; srcX: cint; srcY: cint; w: cint; h: cint; pct: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageCopyMerge@36'{$ENDIF};
procedure gdImageCopyMergeGray(dst: gdImagePtr; src: gdImagePtr; dstX: cint; dstY: cint; srcX: cint; srcY: cint; w: cint; h: cint; pct: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageCopyMergeGray@36'{$ENDIF};

(* Stretches or shrinks to fit, as needed. Does NOT attempt
  to average the entire set of source pixels that scale down onto the
  destination pixel. *)
procedure gdImageCopyResized(dst: gdImagePtr; src: gdImagePtr; dstX: cint; dstY: cint; srcX: cint; srcY: cint; dstW: cint; dstH: cint; srcW: cint; srcH: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageCopyResized@40'{$ENDIF};

(* gd 2.0: stretches or shrinks to fit, as needed. When called with a
  truecolor destination image, this function averages the
  entire set of source pixels that scale down onto the
  destination pixel, taking into account what portion of the
  destination pixel each source pixel represents. This is a
  floating point operation, but this is not a performance issue
  on modern hardware, except for some embedded devices. If the
  destination is a palette image, gdImageCopyResized is
  substituted automatically. *)
procedure gdImageCopyResampled(dst: gdImagePtr; src: gdImagePtr; dstX: cint; dstY: cint; srcX: cint; srcY: cint; dstW: cint; dstH: cint; srcW: cint; srcH: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageCopyResampled@40'{$ENDIF};

(* gd 2.0.8: gdImageCopyRotated is added. Source
        is a rectangle, with its upper left corner at
        srcX and srcY. Destination is the *center* of
        the rotated copy. Angle is in degrees, same as
        gdImageArc. Floating point destination center
        coordinates allow accurate rotation of
        objects of odd-numbered width or height. *)
procedure gdImageCopyRotated(dst: gdImagePtr; src: gdImagePtr; dstX: double; dstY: double; srcX: cint; srcY: cint; srcWidth: cint; srcHeight: cint; angle: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageCopyRotated@36'{$ENDIF};

procedure gdImageSetBrush(im: gdImagePtr; brush: gdImagePtr); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageSetBrush@8'{$ENDIF};
procedure gdImageSetTile(im: gdImagePtr; tile: gdImagePtr); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageSetTile@8'{$ENDIF};
procedure gdImageSetAntiAliased(im: gdImagePtr; c: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageSetAntiAliased@8'{$ENDIF};
procedure gdImageSetAntiAliasedDontBlend(im: gdImagePtr; c: cint; dont_blend: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageSetAntiAliasedDontBlend@12'{$ENDIF};
procedure gdImageSetStyle(im: gdImagePtr; style: pcint; noOfPixels: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageSetStyle@12'{$ENDIF};
(* Line thickness(defaults to 1). Affects lines, ellipses,
  rectangles, polygons and so forth. *)
procedure gdImageSetThickness(im: gdImagePtr; thickness: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageSetThickness@8'{$ENDIF};
(* On or off(1 or 0) for all three of these. *)
procedure gdImageInterlace(im: gdImagePtr; interlaceArg: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageInterlace@8'{$ENDIF};
procedure gdImageAlphaBlending(im: gdImagePtr; alphaBlendingArg: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageAlphaBlending@8'{$ENDIF};
procedure gdImageSaveAlpha(im: gdImagePtr; saveAlphaArg: cint); EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageSaveAlpha@8'{$ENDIF};

(* Macros to access information about images. *)

(* Returns nonzero if the image is a truecolor image,
  zero for a palette image. *)

function gdImageTrueColor(im: gdImagePtr): cint; inline;
function gdImageSX(im: gdImagePtr): cint; inline;
function gdImageSY(im: gdImagePtr): cint; inline;
function gdImageColorsTotal(im: gdImagePtr): cint; inline;
function gdImageRed(im: gdImagePtr; c: cint): cint; inline;
function gdImageGreen(im: gdImagePtr; c: cint): cint; inline;
function gdImageBlue(im: gdImagePtr; c: cint): cint; inline;
function gdImageAlpha(im: gdImagePtr; c: cint): cint; inline;
function gdImageGetTransparent(im: gdImagePtr): cint; inline;
function gdImageGetInterlaced(im: gdImagePtr): cint; inline;

(* These macros provide direct access to pixels in
  palette-based and truecolor images, respectively.
  If you use these macros, you must perform your own
  bounds checking. Use of the macro for the correct type
  of image is also your responsibility. *)
function gdImagePalettePixel(im: gdImagePtr; x, y: cint): cint; inline;
function gdImageTrueColorPixel(im: gdImagePtr; x, y: cint): cint; inline;

(* I/O Support routines. *)

function gdNewFileCtx(p: PFILE): gdIOCtxPtr; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdNewFileCtx@4'{$ENDIF};
 (* If data is null, size is ignored and an initial data buffer is
    allocated automatically. NOTE: this function assumes gd has the right
    to free or reallocate "data" at will! Also note that gd will free
    "data" when the IO context is freed. If data is not null, it must point
    to memory allocated with gdMalloc, or by a call to gdImage[something]Ptr.
    If not, see gdNewDynamicCtxEx for an alternative. *)
function gdNewDynamicCtx(size: cint; data: pointer): gdIOCtxPtr; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdNewDynamicCtx@8'{$ENDIF};
 (* 2.0.21: if freeFlag is nonzero, gd will free and/or reallocate "data" as
    needed as described above. If freeFlag is zero, gd will never free
    or reallocate "data," which means that the context should only be used
    for *reading* an image from a memory buffer, or writing an image to a
    memory buffer which is already large enough. If the memory buffer is
    not large enough and an image write is attempted, the write operation
    will fail. Those wishing to write an image to a buffer in memory have
    a much simpler alternative in the gdImage[something]Ptr functions. *)
function gdNewDynamicCtxEx(size: cint; data: pointer; freeFlag: cint): gdIOCtxPtr; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdNewDynamicCtxEx@12'{$ENDIF};
function gdNewSSCtx(_in: gdSourcePtr; _out: gdSinkPtr): gdIOCtxPtr; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdNewSSCtx@8'{$ENDIF};
function gdDPExtractData(ctx: gdIOCtxPtr; size: pcint): pointer; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdDPExtractData@8'{$ENDIF};

function gdNewStreamCtx(Stream: TStream; free: boolean): gdIOCtxPtr;
// NOTE: don't forget to call ctx^.gd_free(ctx)

const
  GD2_CHUNKSIZE           = 128;
  GD2_CHUNKSIZE_MIN       = 64;
  GD2_CHUNKSIZE_MAX       = 4096;

  GD2_VERS                = 2;
  GD2_ID                  = 'gd2';

  GD2_FMT_RAW             = 1;
  GD2_FMT_COMPRESSED      = 2;

(* Image comparison definitions *)
function gdImageCompare(im1: gdImagePtr; im2: gdImagePtr): cint; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdImageCompare@8'{$ENDIF};

const
  GD_CMP_IMAGE        = 1; (* Actual image IS different *)
  GD_CMP_NUM_COLORS   = 2; (* Number of Colours in pallette differ *)
  GD_CMP_COLOR        = 4; (* Image colours differ *)
  GD_CMP_SIZE_X       = 8; (* Image width differs *)
  GD_CMP_SIZE_Y       = 16; (* Image heights differ *)
  GD_CMP_TRANSPARENT  = 32;  (* Transparent colour *)
  GD_CMP_BACKGROUND   = 64;  (* Background colour *)
  GD_CMP_INTERLACE    = 128; (* Interlaced setting *)
  GD_CMP_TRUECOLOR    = 256; (* Truecolor vs palette differs *)

(* resolution affects ttf font rendering, particularly hinting *)
  GD_RESOLUTION       = 96; (* pixels per inch *)

(* newfangled special effects *)
//#include "gdfx.h"

function gdFontGetLarge(): gdFontPtr; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdFontGetLarge@0'{$ENDIF};
function gdFontGetSmall(): gdFontPtr; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdFontGetSmall@0'{$ENDIF};
function gdFontGetGiant(): gdFontPtr; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdFontGetGiant@0'{$ENDIF};
function gdFontGetMediumBold(): gdFontPtr; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdFontGetMediumBold@0'{$ENDIF};
function gdFontGetTiny(): gdFontPtr; EXTDECL; external gdlib {$IFDEF WINDOWS}name '_gdFontGetTiny@0'{$ENDIF};

{$ifdef windows}
function gdFontLarge(): gdFontPtr; inline;
function gdFontSmall(): gdFontPtr; inline;
function gdFontGiant(): gdFontPtr; inline;
function gdFontMediumBold(): gdFontPtr; inline;
function gdFontTiny(): gdFontPtr; inline;
{$else}
var
{$ifndef darwin}
  gdFontLarge      : gdFontPtr; cvar; external gdlib;
  gdFontSmall      : gdFontPtr; cvar; external gdlib;
  gdFontGiant      : gdFontPtr; cvar; external gdlib;
  gdFontMediumBold : gdFontPtr; cvar; external gdlib;
  gdFontTiny       : gdFontPtr; cvar; external gdlib;
{$else darwin}
  gdFontLarge      : gdFontPtr; external gdlib name 'gdFontLarge';
  gdFontSmall      : gdFontPtr; external gdlib name 'gdFontSmall';
  gdFontGiant      : gdFontPtr; external gdlib name 'gdFontGiant';
  gdFontMediumBold : gdFontPtr; external gdlib name 'gdFontMediumBold';
  gdFontTiny       : gdFontPtr; external gdlib name 'gdFontTiny';
{$endif darwin}
{$endif}


{overloaded pascal functions}
function fopen(filename, rights: String): PFile;
procedure gdImageChar(im: gdImagePtr; f: gdFontPtr; x: cint; y: cint; c: char; color: cint);
procedure gdImageCharUp(im: gdImagePtr; f: gdFontPtr; x: cint; y: cint; c: char; color: cint);
procedure gdImageString(im: gdImagePtr; f: gdFontPtr; x: cint; y: cint; s: AnsiString; color: cint);
procedure gdImageStringUp(im: gdImagePtr; f: gdFontPtr; x: cint; y: cint; s: AnsiString; color: cint);
procedure gdImageString16(im: gdImagePtr; f: gdFontPtr; x: cint; y: cint; s: WideString; color: cint);
procedure gdImageStringUp16(im: gdImagePtr; f: gdFontPtr; x: cint; y: cint; s: WideString; color: cint);

implementation

{$ifdef windows}
function gdFontLarge(): gdFontPtr; inline;
begin
  Result := gdFontGetLarge();
end;

function gdFontSmall(): gdFontPtr; inline;
begin
  Result := gdFontGetSmall();
end;

function gdFontGiant(): gdFontPtr; inline;
begin
  Result := gdFontGetGiant();
end;

function gdFontMediumBold(): gdFontPtr; inline;
begin
  Result := gdFontGetMediumBold();
end;

function gdFontTiny(): gdFontPtr; inline;
begin
  Result := gdFontGetTiny();
end;
{$endif}

function gdTrueColorGetAlpha(c: cint): cint;
begin
  Result :=(c and $7F000000) shr 24;
end;

function gdTrueColorGetRed(c: cint): cint;
begin
  Result :=(c and $FF0000) shr 16;
end;

function gdTrueColorGetGreen(c: cint): cint;
begin
  Result :=(c and $00FF00) shr 8;
end;

function gdTrueColorGetBlue(c: cint): cint;
begin
  Result := c and $0000FF;
end;

function gdImageCreatePalette(sx: cint; sy: cint): gdImagePtr;
begin
  Result := gdImageCreate(sx, sy);
end;

function gdTrueColor(r: cint; g: cint; b: cint): cint;
begin
  result := (r shl 16) or (g shl 8) or (b);
end;

function gdTrueColorAlpha(r: cint; g: cint; b: cint; a: cint): cint;
begin
  result := (a shl 24) or (r shl 16) or (g shl 8) or (b);
end;

function gdImageTrueColor(im: gdImagePtr): cint;
begin
  Result := im^.trueColor;
end;

function gdImageSX(im: gdImagePtr): cint;
begin
  Result := im^.sx;
end;

function gdImageSY(im: gdImagePtr): cint;
begin
  Result := im^.sy;
end;

function gdImageColorsTotal(im: gdImagePtr): cint;
begin
  Result := im^.colorsTotal;
end;

function gdImageRed(im: gdImagePtr; c: cint): cint;
begin
  if im^.trueColor <> 0 then
    Result := gdTrueColorGetRed(c)
  else
    Result := im^.red[c];
end;

function gdImageGreen(im: gdImagePtr; c: cint): cint;
begin
  if im^.trueColor <> 0 then
    Result := gdTrueColorGetGreen(c)
  else
    Result := im^.green[c];
end;

function gdImageBlue(im: gdImagePtr; c: cint): cint;
begin
  if im^.trueColor <> 0 then
    Result := gdTrueColorGetBlue(c)
  else
    Result := im^.blue[c];
end;

function gdImageAlpha(im: gdImagePtr; c: cint): cint;
begin
  if im^.trueColor <> 0 then
    Result := gdTrueColorGetAlpha(c)
  else
    Result := im^.alpha[c];
end;

function gdImageGetTransparent(im: gdImagePtr): cint;
begin
  Result := im^.transparent;
end;

function gdImageGetInterlaced(im: gdImagePtr): cint;
begin
  Result := im^.interlace;
end;

function gdImagePalettePixel(im: gdImagePtr; x, y: cint): cint;
begin
  Result := im^.pixels[y][x];
end;

function gdImageTrueColorPixel(im: gdImagePtr; x, y: cint): cint;
begin
  Result := im^.tpixels[y][x];
end;

function fopen(filename, rights: String): PFile;
begin
  Result := fopen(PChar(filename), PChar(rights));
end;

procedure gdImageChar(im: gdImagePtr; f: gdFontPtr; x: cint; y: cint; c: char; color: cint);
begin
  gdImageChar(im,f,x,y,ord(c),color);
end;

procedure gdImageCharUp(im: gdImagePtr; f: gdFontPtr; x: cint; y: cint; c: char; color: cint);
begin
  gdImageCharUp(im,f,x,y,ord(c),color);
end;

procedure gdImageString(im: gdImagePtr; f: gdFontPtr; x: cint; y: cint; s: AnsiString; color: cint);
begin
  gdImageString(im,f,x,y,PAnsiChar(s),color);
end;

procedure gdImageStringUp(im: gdImagePtr; f: gdFontPtr; x: cint; y: cint; s: AnsiString; color: cint);
begin
  gdImageStringUp(im,f,x,y,PAnsiChar(s),color);
end;

procedure gdImageString16(im: gdImagePtr; f: gdFontPtr; x: cint; y: cint; s: WideString; color: cint);
begin
  gdImageString16(im,f,x,y,PWideChar(s),color);
end;

procedure gdImageStringUp16(im: gdImagePtr; f: gdFontPtr; x: cint; y: cint; s: WideString; color: cint);
begin
  gdImageStringUp16(im,f,x,y,PWideChar(s),color);
end;

type
  stream_gdIOCtxPtr = ^stream_gdIOCtx;
  stream_gdIOCtx = record
    ctx: gdIOCtx;
    strm: TStream;
    free: Boolean;
  end;

function stream_getC(ctx: gdIOCtxPtr): cint; EXTDECL;
var
  s: stream_gdIOCtxPtr absolute ctx;
begin
  result := 255;
  s^.strm.read(result, 1);
end;

function stream_getBuf(ctx: gdIOCtxPtr; buf: pointer; len: cint): cint; EXTDECL;
var
  s: stream_gdIOCtxPtr absolute ctx;
begin
  result := s^.strm.read(buf^, len);
end;

procedure stream_putC(ctx: gdIOCtxPtr; len: cint); EXTDECL;
var
  s: stream_gdIOCtxPtr absolute ctx;
begin
  s^.strm.write(len, 1);
end;

procedure stream_putBuf(ctx: gdIOCtxPtr; buf: pointer; len: cint); EXTDECL;
var
  s: stream_gdIOCtxPtr absolute ctx;
begin
  s^.strm.write(buf^, len);
end;

function stream_seek(ctx: gdIOCtxPtr; pos: cint): cint; EXTDECL;
var
  s: stream_gdIOCtxPtr absolute ctx;
begin
  result := s^.strm.seek(soFromBeginning, pos);
end;

function stream_tell(ctx: gdIOCtxPtr): clong; EXTDECL;
var
  s: stream_gdIOCtxPtr absolute ctx;
begin
  result := s^.strm.position;
end;

procedure stream_gd_free(ctx: gdIOCtxPtr); EXTDECL;
var
  s: stream_gdIOCtxPtr absolute ctx;
begin
  if s^.free then
    s^.strm.Free;
  FreeMem(s);
end;

function gdNewStreamCtx(Stream: TStream; free: boolean): gdIOCtxPtr;
var
  res: stream_gdIOCtxPtr;
begin
  GetMem(res, Sizeof(stream_gdIOCtx));
  res^.ctx.getC := @stream_getC;
  res^.ctx.getBuf := @stream_getBuf;
  res^.ctx.putC := @stream_putC;
  res^.ctx.putBuf := @stream_putBuf;
  res^.ctx.seek := @stream_seek;
  res^.ctx.tell := @stream_tell;
  res^.ctx.gd_free := @stream_gd_free;
  res^.strm := Stream;
  res^.free := free;
  Result := @res^.ctx;
end;

end.
