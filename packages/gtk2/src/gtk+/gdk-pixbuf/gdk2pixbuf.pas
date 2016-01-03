{ GdkPixbuf library - Main header file

   Copyright (C) 1999 The Free Software Foundation

   Authors: Mark Crichton <crichton@gimp.org>
            Miguel de Icaza <miguel@gnu.org>
            Federico Mena-Quintero <federico@gimp.org>
            Havoc Pennington <hp@redhat.com>

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, write to the
<<<<<<< HEAD
<<<<<<< HEAD
   Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
   Boston, MA 02110-1301, USA.
=======
   Free Software Foundation, Inc., 59 Temple Place - Suite 330,
   Boston, MA 02111-1307, USA.
>>>>>>> graemeg/fixes_2_2
=======
   Free Software Foundation, Inc., 59 Temple Place - Suite 330,
   Boston, MA 02111-1307, USA.
>>>>>>> origin/fixes_2_2
  }
unit gdk2pixbuf; // keep unit name lowercase for kylix

{$IFDEF FPC}
  {$mode objfpc}
{$ENDIF}
{$IFDEF VER140}
  {$DEFINE KYLIX}
{$ENDIF}

interface

uses glib2;

const
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef windows}
=======
{$ifdef win32}
>>>>>>> graemeg/fixes_2_2
=======
{$ifdef win32}
>>>>>>> origin/fixes_2_2
  {$define gdkpixbufwin}
  gdkpixbuflib = 'libgdk_pixbuf-2.0-0.dll';
  {$IFDEF FPC}
    {$ifndef NO_SMART_LINK}
      {$smartlink on}
    {$endif}
  {$ENDIF}
{$else}
  {$ifdef darwin}
    gdkpixbuflib = 'gdk_pixbuf-2.0.0';
    {$linklib gtk-x11-2.0}
    {$linklib gdk-x11-2.0}
    {$linklib pango-1.0.0}
    {$linklib glib-2.0.0}
    {$linklib gobject-2.0.0}
    {$linklib gdk_pixbuf-2.0.0}
    {$linklib atk-1.0.0}
  {$else}
    {$ifdef UseCustomLibs}
    gdkpixbuflib = '';
    {$else}
    gdkpixbuflib = 'libgdk_pixbuf-2.0.so';
    {$endif}
  {$endif}
{$endif}

{$define HasGTK2_4}
{$define HasGTK2_6}

{ Pointers to basic pascal types, inserted by h2pas conversion program.}
{$IFNDEF KYLIX}
  {$PACKRECORDS C}
{$ELSE}
  {$ALIGN 4}
  {$WEAKPACKAGEUNIT}
  {$WARNINGS OFF}
{$ENDIF}

{$IFNDEF Kylix}
var
<<<<<<< HEAD
<<<<<<< HEAD
  {$IFDEF WINDOWS }
=======
  {$IFDEF WIN32 }
>>>>>>> graemeg/fixes_2_2
=======
  {$IFDEF WIN32 }
>>>>>>> origin/fixes_2_2
  gdk_pixbuf_major_version: guint; external gdkpixbuflib name 'gdk_pixbuf_major_version';
  gdk_pixbuf_minor_version: guint; external gdkpixbuflib name 'gdk_pixbuf_minor_version';
  gdk_pixbuf_micro_version: guint; external gdkpixbuflib name 'gdk_pixbuf_micro_version';
  gdk_pixbuf_version: PChar ; external gdkpixbuflib name 'gdk_pixbuf_version';
  {$ELSE }
  gdk_pixbuf_major_version: guint;cvar;external;
  gdk_pixbuf_minor_version: guint;cvar;external;
  gdk_pixbuf_micro_version: guint;cvar;external;
  gdk_pixbuf_version: PChar;cvar;external;
  {$ENDIF }
{$ENDIF}

type
  // internal type
  PGdkPixbuf = pointer;

  // internal type
  PGdkPixbufAnimation = pointer;

  // internal type
  PGdkPixbufAnimationIter = pointer;


{ Alpha compositing mode  }
   PGdkPixbufAlphaMode = ^TGdkPixbufAlphaMode;
   TGdkPixbufAlphaMode = (
     GDK_PIXBUF_ALPHA_BILEVEL,
     GDK_PIXBUF_ALPHA_FULL
   );

{ Color spaces; right now only RGB is supported.
   Note that these values are encoded in inline pixbufs
   as ints, so don't reorder them
  }
   PGdkColorspace = ^TGdkColorspace;
   TGdkColorspace = (GDK_COLORSPACE_RGB);

{ Handler that must free the pixel array  }
   TGdkPixbufDestroyNotify = procedure (pixels:Pguchar; data:gpointer); cdecl;

{ image data hosed  }
{ no mem to load image  }
{ bad option passed to save routine  }
{ unsupported image type (sort of an ENOSYS)  }
{ unsupported operation (load, save) for image type  }
   PGdkPixbufError = ^TGdkPixbufError;
   TGdkPixbufError = (
     GDK_PIXBUF_ERROR_CORRUPT_IMAGE,
     GDK_PIXBUF_ERROR_INSUFFICIENT_MEMORY,
     GDK_PIXBUF_ERROR_BAD_OPTION,
     GDK_PIXBUF_ERROR_UNKNOWN_TYPE,
     GDK_PIXBUF_ERROR_UNSUPPORTED_OPERATION,
     GDK_PIXBUF_ERROR_FAILED
   );

{ Interpolation modes  }
   PGdkInterpType = ^TGdkInterpType;
   TGdkInterpType = (
     GDK_INTERP_NEAREST,
     GDK_INTERP_TILES,
     GDK_INTERP_BILINEAR,
     GDK_INTERP_HYPER
   );

<<<<<<< HEAD
<<<<<<< HEAD
   TGdkPixbufRotation = (
	GDK_PIXBUF_ROTATE_NONE             =   0,
	GDK_PIXBUF_ROTATE_COUNTERCLOCKWISE =  90,
	GDK_PIXBUF_ROTATE_UPSIDEDOWN       = 180,
	GDK_PIXBUF_ROTATE_CLOCKWISE        = 270
   );

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function GDK_TYPE_PIXBUF : GType;
function GDK_PIXBUF(anObject : pointer) : PGdkPixbuf;
function GDK_IS_PIXBUF(anObject : pointer) : boolean;

function GDK_TYPE_PIXBUF_ANIMATION : GType;
function GDK_PIXBUF_ANIMATION(anObject : pointer) : PGdkPixbufAnimation;
function GDK_IS_PIXBUF_ANIMATION(anObject : pointer) : boolean;
function GDK_TYPE_PIXBUF_ANIMATION_ITER : GType;
function GDK_PIXBUF_ANIMATION_ITER(anObject : pointer) : PGdkPixbufAnimationIter;
function GDK_IS_PIXBUF_ANIMATION_ITER(anObject : pointer) : boolean;

function GDK_PIXBUF_ERROR : TGQuark;
function gdk_pixbuf_error_quark:TGQuark; cdecl; external gdkpixbuflib;
function gdk_pixbuf_get_type:GType; cdecl; external gdkpixbuflib;


{ Reference counting  }
{$ifndef GDK_PIXBUF_DISABLE_DEPRECATED}
function gdk_pixbuf_ref(pixbuf:PGdkPixbuf):PGdkPixbuf; cdecl; external gdkpixbuflib;
procedure gdk_pixbuf_unref(pixbuf:PGdkPixbuf); cdecl; external gdkpixbuflib;
{$endif}


{ GdkPixbuf accessors  }
function gdk_pixbuf_get_colorspace(pixbuf:PGdkPixbuf):TGdkColorspace; cdecl; external gdkpixbuflib;
function gdk_pixbuf_get_n_channels(pixbuf:PGdkPixbuf):longint; cdecl; external gdkpixbuflib;
function gdk_pixbuf_get_has_alpha(pixbuf:PGdkPixbuf):gboolean; cdecl; external gdkpixbuflib;
function gdk_pixbuf_get_bits_per_sample(pixbuf:PGdkPixbuf):longint; cdecl; external gdkpixbuflib;
function gdk_pixbuf_get_pixels(pixbuf:PGdkPixbuf):Pguchar; cdecl; external gdkpixbuflib;
function gdk_pixbuf_get_width(pixbuf:PGdkPixbuf):longint; cdecl; external gdkpixbuflib;
function gdk_pixbuf_get_height(pixbuf:PGdkPixbuf):longint; cdecl; external gdkpixbuflib;
function gdk_pixbuf_get_rowstride(pixbuf:PGdkPixbuf):longint; cdecl; external gdkpixbuflib;

{ Create a blank pixbuf with an optimal rowstride and a new buffer  }
function gdk_pixbuf_new(colorspace:TGdkColorspace; has_alpha:gboolean; bits_per_sample:longint; width:longint; height:longint):PGdkPixbuf; cdecl; external gdkpixbuflib;

{ Copy a pixbuf  }
function gdk_pixbuf_copy(pixbuf:PGdkPixbuf):PGdkPixbuf; cdecl; external gdkpixbuflib;

{ Create a pixbuf which points to the pixels of another pixbuf  }
function gdk_pixbuf_new_subpixbuf(src_pixbuf:PGdkPixbuf; src_x:longint; src_y:longint; width:longint; height:longint):PGdkPixbuf; cdecl; external gdkpixbuflib;

{ Simple loading  }
function gdk_pixbuf_new_from_file(filename:Pchar; error:PPGError):PGdkPixbuf; cdecl; external gdkpixbuflib;
function gdk_pixbuf_new_from_data(data:Pguchar; colorspace:TGdkColorspace; has_alpha:gboolean; bits_per_sample:longint; width:longint;
           height:longint; rowstride:longint; destroy_fn:TGdkPixbufDestroyNotify; destroy_fn_data:gpointer):PGdkPixbuf; cdecl; external gdkpixbuflib;
function gdk_pixbuf_new_from_xpm_data(data:PPchar):PGdkPixbuf; cdecl; external gdkpixbuflib;
function gdk_pixbuf_new_from_inline(data_length:gint; var data:guint8; copy_pixels:gboolean; error:PPGError):PGdkPixbuf; cdecl; external gdkpixbuflib;

{$IFDEF HasGTK2_4}
function gdk_pixbuf_new_from_file_at_size(filename:Pchar; width, height: gint;error:PPGError):PGdkPixbuf; cdecl; external gdkpixbuflib;
{$ENDIF HasGTK2_4}
{$IFDEF HasGTK2_6}
function gdk_pixbuf_new_from_file_at_scale(filename:Pchar; width, height: gint; preserve_aspect_ratio: gboolean; error:PPGError):PGdkPixbuf; cdecl; external gdkpixbuflib;
{$ENDIF HasGTK2_6}

{ Mutations  }
procedure gdk_pixbuf_fill(pixbuf:PGdkPixbuf; pixel:guint32); cdecl; external gdkpixbuflib;

{ Saving  }
{$IFNDEF KYLIX}
function gdk_pixbuf_save(pixbuf:PGdkPixbuf; filename:Pchar; _type:Pchar; error:PPGError; args:array of const):gboolean; cdecl; overload; external gdkpixbuflib;
function gdk_pixbuf_save(pixbuf:PGdkPixbuf; filename:Pchar; _type:Pchar; error:PPGError):gboolean; cdecl; overload; external gdkpixbuflib;
{$ELSE}
function gdk_pixbuf_save(pixbuf:PGdkPixbuf; filename:Pchar; _type:Pchar; error:PPGError):gboolean; varargs; cdecl; external gdkpixbuflib;
{$ENDIF}
function gdk_pixbuf_savev(pixbuf:PGdkPixbuf; filename:Pchar; _type:Pchar; option_keys:PPchar; option_values:PPchar;
           error:PPGError):gboolean; cdecl; external gdkpixbuflib;

{ Adding an alpha channel  }
function gdk_pixbuf_add_alpha(pixbuf:PGdkPixbuf; substitute_color:gboolean; r:guchar; g:guchar; b:guchar):PGdkPixbuf; cdecl; external gdkpixbuflib;

{ Copy an area of a pixbuf onto another one  }
procedure gdk_pixbuf_copy_area(src_pixbuf:PGdkPixbuf; src_x:longint; src_y:longint; width:longint; height:longint;
            dest_pixbuf:PGdkPixbuf; dest_x:longint; dest_y:longint); cdecl; external gdkpixbuflib;

{ Brighten/darken and optionally make it pixelated-looking  }
procedure gdk_pixbuf_saturate_and_pixelate(src:PGdkPixbuf; dest:PGdkPixbuf; saturation:gfloat; pixelate:gboolean); cdecl; external gdkpixbuflib;


{ Rendering to a drawable  }
{ Scaling  }
procedure gdk_pixbuf_scale(src:PGdkPixbuf; dest:PGdkPixbuf; dest_x:longint; dest_y:longint; dest_width:longint;
            dest_height:longint; offset_x:double; offset_y:double; scale_x:double; scale_y:double;
            interp_type:TGdkInterpType); cdecl; external gdkpixbuflib;
procedure gdk_pixbuf_composite(src:PGdkPixbuf; dest:PGdkPixbuf; dest_x:longint; dest_y:longint; dest_width:longint;
            dest_height:longint; offset_x:double; offset_y:double; scale_x:double; scale_y:double;
            interp_type:TGdkInterpType; overall_alpha:longint); cdecl; external gdkpixbuflib;
procedure gdk_pixbuf_composite_color(src:PGdkPixbuf; dest:PGdkPixbuf; dest_x:longint; dest_y:longint; dest_width:longint;
            dest_height:longint; offset_x:double; offset_y:double; scale_x:double; scale_y:double;
            interp_type:TGdkInterpType; overall_alpha:longint; check_x:longint; check_y:longint; check_size:longint;
            color1:guint32; color2:guint32); cdecl; external gdkpixbuflib;
function gdk_pixbuf_scale_simple(src:PGdkPixbuf; dest_width:longint; dest_height:longint; interp_type:TGdkInterpType):PGdkPixbuf; cdecl; external gdkpixbuflib;
function gdk_pixbuf_composite_color_simple(src:PGdkPixbuf; dest_width:longint; dest_height:longint; interp_type:TGdkInterpType; overall_alpha:longint;
           check_size:longint; color1:guint32; color2:guint32):PGdkPixbuf; cdecl; external gdkpixbuflib;
<<<<<<< HEAD
<<<<<<< HEAD
{$IFDEF HasGTK2_6}
function gdk_pixbuf_rotate_simple(src: PGdkPixbuf; angle: TGdkPixbufRotation): PGdkPixbuf; cdecl; external gdkpixbuflib;
function gdk_pixbuf_flip(src: PGdkPixbuf; horizontal: gboolean): PGdkPixbuf; cdecl; external gdkpixbuflib;
{$ENDIF}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{ Animation support  }
function gdk_pixbuf_animation_get_type:GType; cdecl; external gdkpixbuflib;
function gdk_pixbuf_animation_new_from_file(filename:Pchar; error:PPGError):PGdkPixbufAnimation; cdecl; external gdkpixbuflib;

{$ifndef GDK_PIXBUF_DISABLE_DEPRECATED}
function gdk_pixbuf_animation_ref(animation:PGdkPixbufAnimation):PGdkPixbufAnimation; cdecl; external gdkpixbuflib;
procedure gdk_pixbuf_animation_unref(animation:PGdkPixbufAnimation); cdecl; external gdkpixbuflib;
{$endif}

function gdk_pixbuf_animation_get_width(animation:PGdkPixbufAnimation):longint; cdecl; external gdkpixbuflib;
function gdk_pixbuf_animation_get_height(animation:PGdkPixbufAnimation):longint; cdecl; external gdkpixbuflib;
function gdk_pixbuf_animation_is_static_image(animation:PGdkPixbufAnimation):gboolean; cdecl; external gdkpixbuflib;
function gdk_pixbuf_animation_get_static_image(animation:PGdkPixbufAnimation):PGdkPixbuf; cdecl; external gdkpixbuflib;
function gdk_pixbuf_animation_get_iter(animation:PGdkPixbufAnimation; var start_time:TGTimeVal):PGdkPixbufAnimationIter; cdecl; external gdkpixbuflib;
function gdk_pixbuf_animation_iter_get_type:GType; cdecl; external gdkpixbuflib;
function gdk_pixbuf_animation_iter_get_delay_time(iter:PGdkPixbufAnimationIter):longint; cdecl; external gdkpixbuflib;
function gdk_pixbuf_animation_iter_get_pixbuf(iter:PGdkPixbufAnimationIter):PGdkPixbuf; cdecl; external gdkpixbuflib;
function gdk_pixbuf_animation_iter_on_currently_loading_frame(iter:PGdkPixbufAnimationIter):gboolean; cdecl; external gdkpixbuflib;
function gdk_pixbuf_animation_iter_advance(iter:PGdkPixbufAnimationIter; var current_time:TGTimeVal):gboolean; cdecl; external gdkpixbuflib;
function gdk_pixbuf_get_option(pixbuf:PGdkPixbuf; key:Pgchar):Pgchar; cdecl; external gdkpixbuflib;

{$DEFINE read_interface}
{$include gdk-pixbuf-loader.inc}
{ $include gdk-pixbuf-enum-types.inc}
{$UNDEF read_interface}


implementation


{$DEFINE read_implementation}
{$include gdk-pixbuf-loader.inc}
{ $include gdk-pixbuf-enum-types.inc}
{$UNDEF read_implementation}

{$IFNDEF KYLIX}
{ There is a bug in the compiler. If an external variable is not used, it will
  create code, that can be relocated by the linker.
  So, use them in this hidden procedure.
}
procedure CheckUnusedVariable; [Public];
begin
  if (gdk_pixbuf_major_version=0)
  and (gdk_pixbuf_minor_version=0)
  and (gdk_pixbuf_micro_version=0)
  and (gdk_pixbuf_version=nil) then ;
end;
{$ENDIF}


function GDK_TYPE_PIXBUF : GType;
begin
  GDK_TYPE_PIXBUF:=gdk_pixbuf_get_type;
end;

function GDK_PIXBUF(anObject : pointer) : PGdkPixbuf;
begin
  GDK_PIXBUF:=PGdkPixbuf(G_TYPE_CHECK_INSTANCE_CAST(anObject,GDK_TYPE_PIXBUF));
end;

function GDK_IS_PIXBUF(anObject : pointer) : boolean;
begin
  GDK_IS_PIXBUF:=G_TYPE_CHECK_INSTANCE_TYPE(anObject,GDK_TYPE_PIXBUF);
end;

function GDK_TYPE_PIXBUF_ANIMATION : GType;
begin
  GDK_TYPE_PIXBUF_ANIMATION:=gdk_pixbuf_animation_get_type;
end;

function GDK_PIXBUF_ANIMATION(anObject : pointer) : PGdkPixbufAnimation;
begin
  GDK_PIXBUF_ANIMATION:=PGdkPixbufAnimation(G_TYPE_CHECK_INSTANCE_CAST(anObject,
                                                    GDK_TYPE_PIXBUF_ANIMATION));
end;

function GDK_IS_PIXBUF_ANIMATION(anObject : pointer) : boolean;
begin
  GDK_IS_PIXBUF_ANIMATION:=G_TYPE_CHECK_INSTANCE_TYPE(anObject,
                                                     GDK_TYPE_PIXBUF_ANIMATION);
end;

function GDK_TYPE_PIXBUF_ANIMATION_ITER : GType;
begin
  GDK_TYPE_PIXBUF_ANIMATION_ITER:=gdk_pixbuf_animation_iter_get_type;
end;

function GDK_PIXBUF_ANIMATION_ITER(anObject : pointer) : PGdkPixbufAnimationIter;
begin
  GDK_PIXBUF_ANIMATION_ITER:=PGdkPixbufAnimationIter(G_TYPE_CHECK_INSTANCE_CAST(
                                      anObject,GDK_TYPE_PIXBUF_ANIMATION_ITER));
end;

function GDK_IS_PIXBUF_ANIMATION_ITER(anObject : pointer) : boolean;
begin
  GDK_IS_PIXBUF_ANIMATION_ITER:=G_TYPE_CHECK_INSTANCE_TYPE(anObject,
                                                GDK_TYPE_PIXBUF_ANIMATION_ITER);
end;

function GDK_PIXBUF_ERROR : TGQuark;
begin
  GDK_PIXBUF_ERROR:=gdk_pixbuf_error_quark;
end;


end.
