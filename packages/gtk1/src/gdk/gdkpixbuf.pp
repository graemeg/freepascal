{

   GdkPixbuf library

   Copyright (C) 1999 The Free Software Foundation

   Authors: Mark Crichton <crichton@gimp.org>
           Miguel de Icaza <miguel@gnu.org>
           Federico Mena-Quintero <federico@gimp.org>
           Havoc Pennington <hp@redhat.com>

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Library General Public
   License as published by the Free Software Foundation; either
   version 2 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Library General Public License for more details.

   You should have received a copy of the GNU Library General Public
   License along with this library; if not, write to the
   Free Software Foundation, Inc., 59 Temple Place - Suite 330,
   Boston, MA 02111-1307, USA.

 **********************************************************************}
unit gdkpixbuf;

interface
{$ifndef os2}

{$Ifdef FPC}
  {$PACKRECORDS C}
{$EndIf}

{.$Define XLIB_SUPPORT}         (* Use gdk_pixbuf_xlib instead of gdk_pixbuf *)
{$Define GTK_SUPPORT}           (* enable TGdkPixbufLoader support *)
{.$Define GNOME_SUPPORT}        (* enable TGnomeCanvasPixbuf support *)

{$IFDef XLIB_SUPPORT}

  {$Ifdef FPC}
    {$linklib gdk_pixbuf_xlib}
  {$EndIf}

  {$Undef GTK_SUPPORT}          (* Ensure GTK is disabled in XLIB mode *)
  {$Undef GNOME_SUPPORT}        (* Ensure GNOME is disabled in XLIB mode *)

{$Else}

  {$Ifdef FPC}
    {$linklib gdk_pixbuf}
  {$EndIf}

  {$IFDef GNOME_SUPPORT}
    {$Define GTK_SUPPORT}       (* Ensure GTK is enabled if GNOME is *)
  {$EndIf}

{$EndIf}

Uses
  GLIB,
  {$IFDef XLIB_SUPPORT}
  XLib,
  {$Else}
    GDK
    {$IFDef GTK_SUPPORT}
      ,GTK
      {$IfDef GNOME_SUPPORT}
        ,GNOME
      {$ENDIF}
    {$ENDIF}
  {$ENDIF};


{$IFDef XLIB_SUPPORT}

  const
    libgdkpixbuf =
      {$Ifdef FPC}
        'gdk_pixbuf_xlib';              (* Set library to gdk_pixbuf_xlib *)
      {$Else}
        'libgdk_pixbuf_xlib.so';        (* Set library to gdk_pixbuf_xlib *)
      {$EndIf}

{$Else}

  const
    libgdkpixbuf =
      {$Ifdef FPC}
        'gdk_pixbuf';           (* Set library to gdk_pixbuf *)

      {$Else}
        'libgdk_pixbuf.so';     (* Set library to gdk_pixbuf *)
      {$EndIf}

{$EndIf}

{From gdk-pixbuf-features.h}
  const
     GDK_PIXBUF_FEATURES_H = 1;
     GDK_PIXBUF_MAJOR = 0;
     GDK_PIXBUF_MINOR = 16;
     GDK_PIXBUF_MICRO = 0;
     GDK_PIXBUF_VERSION_ = '0.16.0';

{$Ifdef FPC}
  var
{$ifndef darwin}
    gdk_pixbuf_major_version : guint; cvar; external;
    gdk_pixbuf_version : Pchar; cvar; external;
{$else darwin}
    gdk_pixbuf_major_version : guint; external libgdkpixbuf name 'gdk_pixbuf_major_version';
    gdk_pixbuf_version : Pchar; external libgdkpixbuf name 'gdk_pixbuf_version';
{$endif darwin}
{$EndIf}

{From gdk-pixbuf.h}
type
  TGdkColorspace = (GDK_COLORSPACE_RGB);
  { All of these are opaque structures  }
   _GdkPixbuf = Record
   end;
   TGdkPixbuf = _GdkPixbuf;
   PGdkPixbuf = ^TGdkPixbuf;

   _GdkPixbufFrame = Record
   end;
   TGdkPixbufFrame = _GdkPixbufFrame;
   PGdkPixbufFrame = ^TGdkPixbufFrame;

   _GdkPixbufAnimation = Record
   end;
   TGdkPixbufAnimation = _GdkPixbufAnimation;
   PGdkPixbufAnimation = ^TGdkPixbufAnimation;

  { Handler that must free the pixel array  }
   TGdkPixbufDestroyNotify = Function (pixels : Pguchar; data : gpointer) : Pointer;
  { Handler for the last unref operation  }
   TGdkPixbufLastUnref = Function (pixbuf : PGdkPixbuf; data : gpointer) : Pointer;
  { Reference counting  }

  function gdk_pixbuf_ref(pixbuf:PGdkPixbuf):PGdkPixbuf;cdecl; external libgdkpixbuf name 'gdk_pixbuf_ref';
  procedure gdk_pixbuf_unref(pixbuf:PGdkPixbuf);cdecl; external libgdkpixbuf name 'gdk_pixbuf_unref';
  procedure gdk_pixbuf_set_last_unref_handler(pixbuf:PGdkPixbuf; last_unref_fn:TGdkPixbufLastUnref; last_unref_fn_data:gpointer);cdecl; external libgdkpixbuf name 'gdk_pixbuf_set_last_unref_handler';
  procedure gdk_pixbuf_finalize(pixbuf:PGdkPixbuf);cdecl; external libgdkpixbuf name 'gdk_pixbuf_finalize';

  { GdkPixbuf accessors  }
  function gdk_pixbuf_get_colorspace(pixbuf:PGdkPixbuf):TGdkColorspace;cdecl; external libgdkpixbuf name 'gdk_pixbuf_get_colorspace';
  function gdk_pixbuf_get_n_channels(pixbuf:PGdkPixbuf):longint;cdecl; external libgdkpixbuf name 'gdk_pixbuf_get_n_channels';
  function gdk_pixbuf_get_has_alpha(pixbuf:PGdkPixbuf):gboolean;cdecl; external libgdkpixbuf name 'gdk_pixbuf_get_has_alpha';
  function gdk_pixbuf_get_bits_per_sample(pixbuf:PGdkPixbuf):longint;cdecl; external libgdkpixbuf name 'gdk_pixbuf_get_bits_per_sample';
  function gdk_pixbuf_get_pixels(pixbuf:PGdkPixbuf):Pguchar;cdecl; external libgdkpixbuf name 'gdk_pixbuf_get_pixels';
  function gdk_pixbuf_get_width(pixbuf:PGdkPixbuf):longint;cdecl; external libgdkpixbuf name 'gdk_pixbuf_get_width';
  function gdk_pixbuf_get_height(pixbuf:PGdkPixbuf):longint;cdecl; external libgdkpixbuf name 'gdk_pixbuf_get_height';
  function gdk_pixbuf_get_rowstride(pixbuf:PGdkPixbuf):longint;cdecl; external libgdkpixbuf name 'gdk_pixbuf_get_rowstride';

  { Create a blank pixbuf with an optimal rowstride and a new buffer  }
  function gdk_pixbuf_new(colorspace:TGdkColorspace; has_alpha:gboolean; bits_per_sample:longint; width:longint; height:longint):PGdkPixbuf;cdecl; external libgdkpixbuf name 'gdk_pixbuf_new';

  { Copy a pixbuf  }
  function gdk_pixbuf_copy(pixbuf:PGdkPixbuf):PGdkPixbuf;cdecl; external libgdkpixbuf name 'gdk_pixbuf_copy';

  { Simple loading  }
  function gdk_pixbuf_new_from_file(filename:Pchar):PGdkPixbuf;cdecl; external libgdkpixbuf name 'gdk_pixbuf_new_from_file';

  function gdk_pixbuf_new_from_data(data:Pguchar; colorspace:TGdkColorspace; has_alpha:gboolean; bits_per_sample:longint; width:longint;
             height:longint; rowstride:longint; destroy_fn:TGdkPixbufDestroyNotify; destroy_fn_data:gpointer):PGdkPixbuf;cdecl; external libgdkpixbuf name 'gdk_pixbuf_new_from_data';

  function gdk_pixbuf_new_from_xpm_data(data:PPchar):PGdkPixbuf;cdecl; external libgdkpixbuf name 'gdk_pixbuf_new_from_xpm_data';

  { Adding an alpha channel  }
  function gdk_pixbuf_add_alpha(pixbuf:PGdkPixbuf; substitute_color:gboolean; r:guchar; g:guchar; b:guchar):PGdkPixbuf;cdecl; external libgdkpixbuf name 'gdk_pixbuf_add_alpha';

  { Copy an area of a pixbuf onto another one  }
  procedure gdk_pixbuf_copy_area(src_pixbuf:PGdkPixbuf; src_x:longint; src_y:longint; width:longint; height:longint;
              dest_pixbuf:PGdkPixbuf; dest_x:longint; dest_y:longint);cdecl; external libgdkpixbuf name 'gdk_pixbuf_copy_area';

  type
     TGdkPixbufAlphaMode = (GDK_PIXBUF_ALPHA_BILEVEL,GDK_PIXBUF_ALPHA_FULL);

  {$IfNDef XLIB_SUPPORT}

    { Rendering to a TGDKDrawable  }

    procedure gdk_pixbuf_render_threshold_alpha(pixbuf:PGdkPixbuf; bitmap:PGdkBitmap; src_x:longint; src_y:longint; dest_x:longint;
              dest_y:longint; width:longint; height:longint; alpha_threshold:longint);cdecl; external libgdkpixbuf name 'gdk_pixbuf_render_threshold_alpha';
    procedure gdk_pixbuf_render_to_drawable(pixbuf:PGdkPixbuf; Drawable:PGDKDrawable; GC:PGDKGC; src_x:longint; src_y:longint;
              dest_x:longint; dest_y:longint; width:longint; height:longint; dither:TGDKRGBDither;
              x_dither:longint; y_dither:longint);cdecl; external libgdkpixbuf name 'gdk_pixbuf_render_to_drawable';
    procedure gdk_pixbuf_render_to_drawable_alpha(pixbuf:PGdkPixbuf; Drawable:PGDKDrawable; src_x:longint; src_y:longint; dest_x:longint;
              dest_y:longint; width:longint; height:longint; alpha_mode:TGdkPixbufAlphaMode; alpha_threshold:longint;
              dither:TGDKRGBDither; x_dither:longint; y_dither:longint);cdecl; external libgdkpixbuf name 'gdk_pixbuf_render_to_drawable_alpha';
    procedure gdk_pixbuf_render_pixmap_and_mask(pixbuf:PGdkPixbuf; Pixmap_return:PPGDKPixmap; mask_return:PPGdkBitmap; alpha_threshold:longint);cdecl; external libgdkpixbuf name 'gdk_pixbuf_render_pixmap_and_mask';

    { Fetching a region from a TDrawable  }
    function gdk_pixbuf_get_from_drawable(dest:PGdkPixbuf; src:PGDKDrawable; cmap:PGDKColormap; src_x:longint; src_y:longint;
               dest_x:longint; dest_y:longint; width:longint; height:longint):PGdkPixbuf;cdecl; external libgdkpixbuf name 'gdk_pixbuf_get_from_drawable';

  {$EndIf}

  { Scaling  }

  type
     TGdkInterpType = (GDK_INTERP_NEAREST,GDK_INTERP_TILES,
       GDK_INTERP_BILINEAR,GDK_INTERP_HYPER);

  procedure gdk_pixbuf_scale(src:PGdkPixbuf; dest:PGdkPixbuf; dest_x:longint; dest_y:longint; dest_width:longint;
              dest_height:longint; offset_x:double; offset_y:double; scale_x:double; scale_y:double;
              interp_type:TGdkInterpType);cdecl; external libgdkpixbuf name 'gdk_pixbuf_scale';
  procedure gdk_pixbuf_composite(src:PGdkPixbuf; dest:PGdkPixbuf; dest_x:longint; dest_y:longint; dest_width:longint;
              dest_height:longint; offset_x:double; offset_y:double; scale_x:double; scale_y:double;
              interp_type:TGdkInterpType; overall_alpha:longint);cdecl; external libgdkpixbuf name 'gdk_pixbuf_composite';
  procedure gdk_pixbuf_composite_color(src:PGdkPixbuf; dest:PGdkPixbuf; dest_x:longint; dest_y:longint; dest_width:longint;
              dest_height:longint; offset_x:double; offset_y:double; scale_x:double; scale_y:double;
              interp_type:TGdkInterpType; overall_alpha:longint; check_x:longint; check_y:longint; check_size:longint;
              color1:guint32; color2:guint32);cdecl; external libgdkpixbuf name 'gdk_pixbuf_composite_color';
  function gdk_pixbuf_scale_simple(src:PGdkPixbuf; dest_width:longint; dest_height:longint; interp_type:TGdkInterpType):PGdkPixbuf;cdecl; external libgdkpixbuf name 'gdk_pixbuf_scale_simple';
  function gdk_pixbuf_composite_color_simple(src:PGdkPixbuf; dest_width:longint; dest_height:longint; interp_type:TGdkInterpType; overall_alpha:longint;
             check_size:longint; color1:guint32; color2:guint32):PGdkPixbuf;cdecl; external libgdkpixbuf name 'gdk_pixbuf_composite_color_simple';

  { Animation support  }
  { GIF-like animation overlay modes for frames  }

  type
     TGdkPixbufFrameAction = (GDK_PIXBUF_FRAME_RETAIN,GDK_PIXBUF_FRAME_DISPOSE,
       GDK_PIXBUF_FRAME_REVERT);

  function gdk_pixbuf_animation_new_from_file(filename:Pchar):PGdkPixbufAnimation;cdecl; external libgdkpixbuf name 'gdk_pixbuf_animation_new_from_file';
  function gdk_pixbuf_animation_ref(animation:PGdkPixbufAnimation):PGdkPixbufAnimation;cdecl; external libgdkpixbuf name 'gdk_pixbuf_animation_ref';
  procedure gdk_pixbuf_animation_unref(animation:PGdkPixbufAnimation);cdecl; external libgdkpixbuf name 'gdk_pixbuf_animation_unref';
  function gdk_pixbuf_animation_get_width(animation:PGdkPixbufAnimation):longint;cdecl; external libgdkpixbuf name 'gdk_pixbuf_animation_get_width';
  function gdk_pixbuf_animation_get_height(animation:PGdkPixbufAnimation):longint;cdecl; external libgdkpixbuf name 'gdk_pixbuf_animation_get_height';
  function gdk_pixbuf_animation_get_frames(animation:PGdkPixbufAnimation):PGList;cdecl; external libgdkpixbuf name 'gdk_pixbuf_animation_get_frames';
  function gdk_pixbuf_animation_get_num_frames(animation:PGdkPixbufAnimation):longint;cdecl; external libgdkpixbuf name 'gdk_pixbuf_animation_get_num_frames';

  { Frame accessors  }
  function gdk_pixbuf_frame_get_pixbuf(frame:PGdkPixbufFrame):PGdkPixbuf;cdecl; external libgdkpixbuf name 'gdk_pixbuf_frame_get_pixbuf';
  function gdk_pixbuf_frame_get_x_offset(frame:PGdkPixbufFrame):longint;cdecl; external libgdkpixbuf name 'gdk_pixbuf_frame_get_x_offset';
  function gdk_pixbuf_frame_get_y_offset(frame:PGdkPixbufFrame):longint;cdecl; external libgdkpixbuf name 'gdk_pixbuf_frame_get_y_offset';
  function gdk_pixbuf_frame_get_delay_time(frame:PGdkPixbufFrame):longint;cdecl; external libgdkpixbuf name 'gdk_pixbuf_frame_get_delay_time';
  function gdk_pixbuf_frame_get_action(frame:PGdkPixbufFrame):TGdkPixbufFrameAction;cdecl; external libgdkpixbuf name 'gdk_pixbuf_frame_get_action';

  { General (presently empty) initialization hooks, primarily for gnome-libs  }
  procedure gdk_pixbuf_preinit(app:gpointer; modinfo:gpointer);cdecl; external libgdkpixbuf name 'gdk_pixbuf_preinit';
  procedure gdk_pixbuf_postinit(app:gpointer; modinfo:gpointer);cdecl; external libgdkpixbuf name 'gdk_pixbuf_postinit';

{From gdk-pixbuf-loader.h}

{$IfDef GTK_SUPPORT}

Type
  _GdkPixbufLoaderPrivate = record
  end;
  TGdkPixbufLoaderPrivate = _GdkPixbufLoaderPrivate;
  PGdkPixbufLoaderPrivate = ^TGdkPixbufLoaderPrivate;

  _GdkPixbufLoader = record
    theobject : PGtkObject;
    { Private data }
    priv : PGdkPixbufLoaderPrivate;
  end;
  TGdkPixbufLoader = _GdkPixbufLoader;
  PGdkPixbufLoader = ^TGdkPixbufLoader;

  _GdkPixbufLoaderClass = record
    parent_class : PGtkObjectClass;

    area_prepared :  Function (loader : PGdkPixbufLoader) : Pointer; cdecl;
    area_updated  :  Function (loader : PGdkPixbufLoader; x, y, width, height : guint) : Pointer; cdecl;
    frame_done  :  Function (loader : PGdkPixbufLoader;  frame : PGdkPixbufFrame) : Pointer; cdecl;
    animation_done  :  Function (loader : PGdkPixbufLoader) : Pointer; cdecl;
    closed  :  Function (loader : PGdkPixbufLoader) : Pointer; cdecl;
  end;
  TGdkPixbufLoaderClass = _GdkPixbufLoaderClass;
  PGdkPixbufLoaderClass = ^TGdkPixbufLoaderClass;

type
  GDK_PIXBUF_LOADER=PGDKPIXBUFLOADER;
  GDK_PIXBUF_LOADER_CLASS=PGDKPIXBUFLOADERClass;

  Function gdk_pixbuf_loader_get_type : TGtkType; cdecl; external libgdkpixbuf name 'gdk_pixbuf_loader_get_type';
  Function gdk_pixbuf_loader_new : PGdkPixbufLoader; cdecl; external libgdkpixbuf name 'gdk_pixbuf_loader_new';
  Function gdk_pixbuf_loader_write(loader : PGdkPixbufLoader;
    const buf : Pguchar; count : longint) : gBoolean; cdecl; external libgdkpixbuf name 'gdk_pixbuf_loader_write';
  Function gdk_pixbuf_loader_get_pixbuf(loader : PGdkPixbufLoader) : PGdkPixbuf; cdecl; external libgdkpixbuf name 'gdk_pixbuf_loader_get_pixbuf';
  Function gdk_pixbuf_loader_get_animation (loader : PGdkPixbufLoader) : PGdkPixbufAnimation; cdecl; external libgdkpixbuf name 'gdk_pixbuf_loader_get_animation';
  Procedure gdk_pixbuf_loader_close(loader : PGdkPixbufLoader); cdecl; external libgdkpixbuf name 'gdk_pixbuf_loader_close';

  Function GDK_TYPE_PIXBUF_LOADER : TGtkType;
  Function GDK_IS_PIXBUF_LOADER(obj:pointer) : boolean;
  Function GDK_IS_PIXBUF_LOADER_CLASS(klass:pointer) : boolean;
{$EndIf}

{From gdk-pixbuf-xlibrgb.h}

{$IfDef XLIB_SUPPORT}
  type
     _XlibRgbCmap = record
          colors : array[0..255] of dword;
          lut : array[0..255] of byte;
       end;
     TXlibRgbCmap = _XlibRgbCmap;
     PXlibRgbCmap = ^TXlibRgbCmap;

  procedure xlib_rgb_init(display:PDisplay; screen:PScreen);cdecl; external libgdkpixbuf name 'xlib_rgb_init';
  procedure xlib_rgb_init_with_depth(display:PDisplay; screen:PScreen; prefDepth:longint);cdecl; external libgdkpixbuf name 'xlib_rgb_init_with_depth';
  function xlib_rgb_xpixel_from_rgb(rgb:guint32):dword;cdecl; external libgdkpixbuf name 'xlib_rgb_xpixel_from_rgb';
  procedure xlib_rgb_gc_set_foreground(GC:TGC; rgb:guint32);cdecl; external libgdkpixbuf name 'xlib_rgb_gc_set_foreground';
  procedure xlib_rgb_gc_set_background(GC:TGC; rgb:guint32);cdecl; external libgdkpixbuf name 'xlib_rgb_gc_set_background';


  type
     TXlibRgbDither = (XLIB_RGB_DITHER_NONE,XLIB_RGB_DITHER_NORMAL,
       XLIB_RGB_DITHER_MAX);

  procedure xlib_draw_rgb_image(Drawable:TDrawable; GC:TGC; x:longint; y:longint; width:longint;
              height:longint; dith:TXlibRgbDither; rgb_buf:Pbyte; rowstride:longint);cdecl; external libgdkpixbuf name 'xlib_draw_rgb_image';
  procedure xlib_draw_rgb_image_dithalign(Drawable:TDrawable; GC:TGC; x:longint; y:longint; width:longint;
              height:longint; dith:TXlibRgbDither; rgb_buf:Pbyte; rowstride:longint; xdith:longint;
              ydith:longint);cdecl; external libgdkpixbuf name 'xlib_draw_rgb_image_dithalign';
  procedure xlib_draw_rgb_32_image(Drawable:TDrawable; GC:TGC; x:longint; y:longint; width:longint;
              height:longint; dith:TXlibRgbDither; buf:Pbyte; rowstride:longint);cdecl; external libgdkpixbuf name 'xlib_draw_rgb_32_image';
  procedure xlib_draw_gray_image(Drawable:TDrawable; GC:TGC; x:longint; y:longint; width:longint;
              height:longint; dith:TXlibRgbDither; buf:Pbyte; rowstride:longint);cdecl; external libgdkpixbuf name 'xlib_draw_gray_image';
  function xlib_rgb_cmap_new(colors:Pguint32; n_colors:longint):PXlibRgbCmap;cdecl; external libgdkpixbuf name 'xlib_rgb_cmap_new';
  procedure xlib_rgb_cmap_free(cmap:PXlibRgbCmap);cdecl; external libgdkpixbuf name 'xlib_rgb_cmap_free';
  procedure xlib_draw_indexed_image(Drawable:TDrawable; GC:TGC; x:longint; y:longint; width:longint;
              height:longint; dith:TXlibRgbDither; buf:Pbyte; rowstride:longint; cmap:PXlibRgbCmap);cdecl; external libgdkpixbuf name 'xlib_draw_indexed_image';

  { Below are some functions which are primarily useful for debugging
     and experimentation.  }
  function xlib_rgb_ditherable:LongBool;cdecl; external libgdkpixbuf name 'xlib_rgb_ditherable';
  procedure xlib_rgb_set_verbose(verbose:LongBool);cdecl; external libgdkpixbuf name 'xlib_rgb_set_verbose';

  { experimental TColormap stuff  }
  procedure xlib_rgb_set_install(install:LongBool);cdecl; external libgdkpixbuf name 'xlib_rgb_set_install';
  procedure xlib_rgb_set_min_colors(min_colors:longint);cdecl; external libgdkpixbuf name 'xlib_rgb_set_min_colors';
  function xlib_rgb_get_cmap:TColormap;cdecl; external libgdkpixbuf name 'xlib_rgb_get_cmap';
  function xlib_rgb_get_visual:PVisual;cdecl; external libgdkpixbuf name 'xlib_rgb_get_visual';
  function xlib_rgb_get_visual_info:PXVisualInfo;cdecl; external libgdkpixbuf name 'xlib_rgb_get_visual_info';
  function xlib_rgb_get_depth:longint;cdecl; external libgdkpixbuf name 'xlib_rgb_get_depth';
  function xlib_rgb_get_display:PDisplay;cdecl; external libgdkpixbuf name 'xlib_rgb_get_display';
  function xlib_rgb_get_screen:PScreen;cdecl; external libgdkpixbuf name 'xlib_rgb_get_screen';
{$EndIf}

{From gdk-pixbuf-xlib.h}

{$IfDef XLIB_SUPPORT}
  procedure gdk_pixbuf_xlib_init(display:PDisplay; screen_num:longint);cdecl; external libgdkpixbuf name 'gdk_pixbuf_xlib_init';
  procedure gdk_pixbuf_xlib_init_with_depth(display:PDisplay; screen_num:longint; prefDepth:longint);cdecl; external libgdkpixbuf name 'gdk_pixbuf_xlib_init_with_depth';
  procedure gdk_pixbuf_xlib_render_threshold_alpha(pixbuf:PGdkPixbuf; bitmap:TPixmap; src_x:longint; src_y:longint; dest_x:longint;
              dest_y:longint; width:longint; height:longint; alpha_threshold:longint);cdecl; external libgdkpixbuf name 'gdk_pixbuf_xlib_render_threshold_alpha';
  procedure gdk_pixbuf_xlib_render_to_drawable(pixbuf:PGdkPixbuf; Drawable:TDrawable; GC:TGC; src_x:longint; src_y:longint;
              dest_x:longint; dest_y:longint; width:longint; height:longint; dither:TXlibRgbDither;
              x_dither:longint; y_dither:longint);cdecl; external libgdkpixbuf name 'gdk_pixbuf_xlib_render_to_drawable';
  procedure gdk_pixbuf_xlib_render_to_drawable_alpha(pixbuf:PGdkPixbuf; Drawable:TDrawable; src_x:longint; src_y:longint; dest_x:longint;
              dest_y:longint; width:longint; height:longint; alpha_mode:TGdkPixbufAlphaMode; alpha_threshold:longint;
              dither:TXlibRgbDither; x_dither:longint; y_dither:longint);cdecl; external libgdkpixbuf name 'gdk_pixbuf_xlib_render_to_drawable_alpha';
  procedure gdk_pixbuf_xlib_render_Pixmap_and_mask(pixbuf:PGdkPixbuf; Pixmap_return:PPixmap; mask_return:PPixmap; alpha_threshold:longint);cdecl; external libgdkpixbuf name 'gdk_pixbuf_xlib_render_Pixmap_and_mask';
  function gdk_pixbuf_xlib_get_from_drawable(dest:PGdkPixbuf; src:TDrawable; cmap:TColormap; visual:PVisual; src_x:longint;
             src_y:longint; dest_x:longint; dest_y:longint; width:longint; height:longint):PGdkPixbuf;cdecl; external libgdkpixbuf name 'gdk_pixbuf_xlib_get_from_drawable';
{$EndIf}

{From gnome-canvas-pixbuf.h}

{$IfDef GNOME_SUPPORT}
Type
  _GnomeCanvasPixbuf = record
    item : TGnomeCanvasItem;

    { Private data }
    priv :gpointer;
  end;
  TGnomeCanvasPixbuf = _GnomeCanvasPixbuf;
  PGnomeCanvasPixbuf = ^TGnomeCanvasPixbuf;

  _GnomeCanvasPixbufClass = record
    parent_class : TGnomeCanvasItemClass;
  end;

  TGnomeCanvasPixbufClass = _GnomeCanvasPixbufClass;
  PGnomeCanvasPixbufClass = ^TGnomeCanvasPixbufClass;

  GNOME_CANVAS_PIXBUF = PGNOMECANVASPIXBUF;
  GNOME_CANVAS_PIXBUF_CLASS = PGnomeCanvasPixbufClass

  Function GNOME_TYPE_CANVAS_PIXBUF : TGTKType; cdcel; external libgdkpixbuf name 'gnome_canvas_pixbuf_get_type';

  Function GNOME_IS_CANVAS_PIXBUF(obj : Pointer);
  Function GNOME_IS_CANVAS_PIXBUF_CLASS(klass : Pointer);

  Function gnome_canvas_pixbuf_get_type : PGtkType; cdecl; external libgdkpixbuf name 'gnome_canvas_pixbuf_get_type';
{$EndIf}

{$endif os2}
implementation
{$ifndef os2}

{$Ifdef FPC}
{ There is a bug in the compiler. If an external variable is not used, it will
  create code, that can not be relocated by the linker.
  So, use them in this hidden procedure.
}
procedure CheckUnusedVariable; [Public];
begin
  if (gdk_pixbuf_major_version=0)
  or (gdk_pixbuf_version=nil) then ;
end;
{$EndIf}

{$IfDef GTK_SUPPORT}
  Function GDK_TYPE_PIXBUF_LOADER       : TGtkType;
  begin
    GDK_TYPE_PIXBUF_LOADER := gdk_pixbuf_loader_get_type;
  end;

  Function GDK_IS_PIXBUF_LOADER(obj : pointer) : boolean;
  begin
    GDK_IS_PIXBUF_LOADER :=(obj<>nil) and GDK_IS_PIXBUF_LOADER_CLASS(PGtkTypeObject(obj)^.klass);
  end;

  Function GDK_IS_PIXBUF_LOADER_CLASS(klass:pointer) : boolean;
  begin
    GDK_IS_PIXBUF_LOADER_CLASS:=(klass<>nil) and (PGtkTypeClass(klass)^.thetype=GDK_TYPE_PIXBUF_LOADER);
  end;
{$EndIf}

{$IfDef GNOME_SUPPORT}
  Function GNOME_IS_CANVAS_PIXBUF(obj : Pointer);
  begin
    GNOME_IS_CANVAS_PIXBUF :=(obj<>nil) and GNOME_IS_CANVAS_PIXBUF_CLASS(PGtkTypeObject(obj)^.klass);
  end;

  Function GNOME_IS_CANVAS_PIXBUF_CLASS(klass : Pointer);
  begin
    GNOME_IS_CANVAS_PIXBUF_CLASS:=(klass<>nil) and (PGtkTypeClass(klass)^.thetype=GNOME_TYPE_CANVAS_PIXBUF);
  end;
{$EndIf}

{$endif os2}
end.
