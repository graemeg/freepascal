unit xrender;
interface
uses
  x,xlib;

{$ifndef os2}
  {$LinkLib c}
  {$LinkLib X11}
const
  libX11='X11';
{$else}
const
  libX11='X11';
{$endif}

{
  Automatically converted by H2Pas 0.99.15 from xrender.h
  The following command line parameters were used:
    -p
    -T
    -S
    -d
    -c
    xrender.h
}

{$PACKRECORDS C}

type

   PGlyph = ^TGlyph;
   TGlyph = dword;

   PGlyphSet = ^TGlyphSet;
   TGlyphSet = dword;

   PPicture = ^TPicture;
   TPicture = dword;

   PPictFormat = ^TPictFormat;
   TPictFormat = dword;

const
   RENDER_NAME = 'RENDER';
   RENDER_MAJOR = 0;
   RENDER_MINOR = 0;
   X_RenderQueryVersion = 0;
   X_RenderQueryPictFormats = 1;
   X_RenderQueryPictIndexValues = 2;
   X_RenderQueryDithers = 3;
   X_RenderCreatePicture = 4;
   X_RenderChangePicture = 5;
   X_RenderSetPictureClipRectangles = 6;
   X_RenderFreePicture = 7;
   X_RenderComposite = 8;
   X_RenderScale = 9;
   X_RenderTrapezoids = 10;
   X_RenderTriangles = 11;
   X_RenderTriStrip = 12;
   X_RenderTriFan = 13;
   X_RenderColorTrapezoids = 14;
   X_RenderColorTriangles = 15;
   X_RenderTransform = 16;
   X_RenderCreateGlyphSet = 17;
   X_RenderReferenceGlyphSet = 18;
   X_RenderFreeGlyphSet = 19;
   X_RenderAddGlyphs = 20;
   X_RenderAddGlyphsFromPicture = 21;
   X_RenderFreeGlyphs = 22;
   X_RenderCompositeGlyphs8 = 23;
   X_RenderCompositeGlyphs16 = 24;
   X_RenderCompositeGlyphs32 = 25;
   BadPictFormat = 0;
   BadPicture = 1;
   BadPictOp = 2;
   BadGlyphSet = 3;
   BadGlyph = 4;
   RenderNumberErrors = BadGlyph + 1;
   PictTypeIndexed = 0;
   PictTypeDirect = 1;
   PictOpClear = 0;
   PictOpSrc = 1;
   PictOpDst = 2;
   PictOpOver = 3;
   PictOpOverReverse = 4;
   PictOpIn = 5;
   PictOpInReverse = 6;
   PictOpOut = 7;
   PictOpOutReverse = 8;
   PictOpAtop = 9;
   PictOpAtopReverse = 10;
   PictOpXor = 11;
   PictOpAdd = 12;
   PictOpSaturate = 13;
   PictOpMaximum = 13;
   PolyEdgeSharp = 0;
   PolyEdgeSmooth = 1;
   PolyModePrecise = 0;
   PolyModeImprecise = 1;
   CPRepeat = 1 shl 0;
   CPAlphaMap = 1 shl 1;
   CPAlphaXOrigin = 1 shl 2;
   CPAlphaYOrigin = 1 shl 3;
   CPClipXOrigin = 1 shl 4;
   CPClipYOrigin = 1 shl 5;
   CPClipMask = 1 shl 6;
   CPGraphicsExposure = 1 shl 7;
   CPSubwindowMode = 1 shl 8;
   CPPolyEdge = 1 shl 9;
   CPPolyMode = 1 shl 10;
   CPDither = 1 shl 11;
   CPLastBit = 11;
type

   PXRenderDirectFormat = ^TXRenderDirectFormat;
   TXRenderDirectFormat = record
        red : smallint;
        redMask : smallint;
        green : smallint;
        greenMask : smallint;
        blue : smallint;
        blueMask : smallint;
        alpha : smallint;
        alphaMask : smallint;
     end;

   PXRenderPictFormat = ^TXRenderPictFormat;
   TXRenderPictFormat = record
        id : TPictFormat;
        _type : longint;
        depth : longint;
        direct : TXRenderDirectFormat;
        colormap : TColormap;
     end;

const
   PictFormatID = 1 shl 0;
   PictFormatType = 1 shl 1;
   PictFormatDepth = 1 shl 2;
   PictFormatRed = 1 shl 3;
   PictFormatRedMask = 1 shl 4;
   PictFormatGreen = 1 shl 5;
   PictFormatGreenMask = 1 shl 6;
   PictFormatBlue = 1 shl 7;
   PictFormatBlueMask = 1 shl 8;
   PictFormatAlpha = 1 shl 9;
   PictFormatAlphaMask = 1 shl 10;
   PictFormatColormap = 1 shl 11;
type

   PXRenderVisual = ^TXRenderVisual;
   TXRenderVisual = record
        visual : PVisual;
        format : PXRenderPictFormat;
     end;

   PXRenderDepth = ^TXRenderDepth;
   TXRenderDepth = record
        depth : longint;
        nvisuals : longint;
        visuals : PXRenderVisual;
     end;

   PXRenderScreen = ^TXRenderScreen;
   TXRenderScreen = record
        depths : PXRenderDepth;
        ndepths : longint;
        fallback : PXRenderPictFormat;
     end;

   PXRenderInfo = ^TXRenderInfo;
   TXRenderInfo = record
        format : PXRenderPictFormat;
        nformat : longint;
        screen : PXRenderScreen;
        nscreen : longint;
        depth : PXRenderDepth;
        ndepth : longint;
        visual : PXRenderVisual;
        nvisual : longint;
     end;

   PXRenderPictureAttributes = ^TXRenderPictureAttributes;
   TXRenderPictureAttributes = record
        _repeat : TBool;
        alpha_map : TPicture;
        alpha_x_origin : longint;
        alpha_y_origin : longint;
        clip_x_origin : longint;
        clip_y_origin : longint;
        clip_mask : TPixmap;
        graphics_exposures : TBool;
        subwindow_mode : longint;
        poly_edge : longint;
        poly_mode : longint;
        dither : TAtom;
     end;

   PXGlyphInfo = ^TXGlyphInfo;
   TXGlyphInfo = record
        width : word;
        height : word;
        x : smallint;
        y : smallint;
        xOff : smallint;
        yOff : smallint;
     end;

function XRenderQueryExtension(dpy:PDisplay; event_basep:Plongint; error_basep:Plongint):TBool;cdecl;external libX11;
function XRenderQueryVersion(dpy:PDisplay; major_versionp:Plongint; minor_versionp:Plongint):TStatus;cdecl;external libX11;
function XRenderQueryFormats(dpy:PDisplay):TStatus;cdecl;external libX11;
function XRenderFindVisualFormat(dpy:PDisplay; visual:PVisual):PXRenderPictFormat;cdecl;external libX11;
function XRenderFindFormat(dpy:PDisplay; mask:dword; template:PXRenderPictFormat; count:longint):PXRenderPictFormat;cdecl;external libX11;
function XRenderCreatePicture(dpy:PDisplay; drawable:TDrawable; format:PXRenderPictFormat; valuemask:dword; attributes:PXRenderPictureAttributes):TPicture;cdecl;external libX11;
procedure XRenderChangePicture(dpy:PDisplay; picture:TPicture; valuemask:dword; attributes:PXRenderPictureAttributes);cdecl;external libX11;
procedure XRenderFreePicture(dpy:PDisplay; picture:TPicture);cdecl;external libX11;
procedure XRenderComposite(dpy:PDisplay; op:longint; src:TPicture; mask:TPicture; dst:TPicture;
            src_x:longint; src_y:longint; mask_x:longint; mask_y:longint; dst_x:longint;
            dst_y:longint; width:dword; height:dword);cdecl;external libX11;
function XRenderCreateGlyphSet(dpy:PDisplay; format:PXRenderPictFormat):TGlyphSet;cdecl;external libX11;
function XRenderReferenceGlyphSet(dpy:PDisplay; existing:TGlyphSet):TGlyphSet;cdecl;external libX11;
procedure XRenderFreeGlyphSet(dpy:PDisplay; glyphset:TGlyphSet);cdecl;external libX11;
procedure XRenderAddGlyphs(dpy:PDisplay; glyphset:TGlyphSet; gids:PGlyph; glyphs:PXGlyphInfo; nglyphs:longint;
            images:Pchar; nbyte_images:longint);cdecl;external libX11;
procedure XRenderFreeGlyphs(dpy:PDisplay; glyphset:TGlyphSet; gids:PGlyph; nglyphs:longint);cdecl;external libX11;
procedure XRenderCompositeString8(dpy:PDisplay; op:longint; src:TPicture; dst:TPicture; maskFormat:PXRenderPictFormat;
            glyphset:TGlyphSet; xSrc:longint; ySrc:longint; xDst:longint; yDst:longint;
            _string:Pchar; nchar:longint);cdecl;external libX11;

implementation


end.
