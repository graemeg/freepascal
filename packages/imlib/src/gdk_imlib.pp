{

   gdk_imlib library

   Copyright (C) 1998 By The Rasterman (Carsten Haitzler)

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
unit gdk_imlib;

interface

{$PACKRECORDS C}
{$mode objfpc}

Uses glib, gdk, gtk;

const
{$ifndef os2}
  gdk_imlibdll='gdk_imlib';
{$else}
  gdk_imlibdll='gdkimlib';
{$endif}

Type
   PGdkImlibBorder = ^TGdkImlibBorder;
   TGdkImlibBorder = record
        left : gint;
        right : gint;
        top : gint;
        bottom : gint;
     end;

   PGdkImlibColor = ^TGdkImlibColor;
   TGdkImlibColor = record
        r : gint;
        g : gint;
        b : gint;
        pixel : gint;
     end;

   PGdkImlibColorModifier = ^TGdkImlibColorModifier;
   TGdkImlibColorModifier = record
        gamma : gint;
        brightness : gint;
        contrast : gint;
     end;

   PGdkImlibModifierMap = ^TGdkImlibModifierMap;
   TGdkImlibModifierMap = record
        rmap : array[0..255] of byte;
        gmap : array[0..255] of byte;
        bmap : array[0..255] of byte;
     end;

   PGdkImlibImage = ^TGdkImlibImage;
   TGdkImlibImage = record
        rgb_width : gint;
        rgb_height : gint;
        rgb_data : Pbyte;
        alpha_data : Pbyte;
        filename : Pgchar;
        width : gint;
        height : gint;
        shape_color : TGdkImlibColor;
        border : TGdkImlibBorder;
        pixmap : PGdkPixmap;
        shape_mask : PGdkBitmap;
        cache : gchar;
        mods : TGdkImlibColorModifier;
        rmod : TGdkImlibColorModifier;
        gmod : TGdkImlibColorModifier;
        bmod : TGdkImlibColorModifier;
        map : PGdkImlibModifierMap;
        ref_count : longint;
        pixmaps : PGHashTable;
     end;

   PGdkImlibSaveInfo = ^TGdkImlibSaveInfo;
   TGdkImlibSaveInfo = record
        quality : longint;
        scaling : longint;
        xjustification : longint;
        yjustification : longint;
        page_size : longint;
        color : char;
     end;

   PGdkImlibInitParams = ^TGdkImlibInitParams;
   TGdkImlibInitParams = record
        flags : longint;
        visualid : longint;
        palettefile : Pchar;
        sharedmem : char;
        sharedpixmaps : char;
        paletteoverride : char;
        remap : char;
        fastrender : char;
        hiquality : char;
        dither : char;
        imagecachesize : longint;
        pixmapcachesize : longint;
     end;

const
  PARAMS_VISUALID        = 1 shl 0;
  PARAMS_PALETTEFILE     = 1 shl 1;
  PARAMS_SHAREDMEM       = 1 shl 2;
  PARAMS_SHAREDPIXMAPS   = 1 shl 3;
  PARAMS_PALETTEOVERRIDE = 1 shl 4;
  PARAMS_REMAP           = 1 shl 5;
  PARAMS_FASTRENDER      = 1 shl 6;
  PARAMS_HIQUALITY       = 1 shl 7;
  PARAMS_DITHER          = 1 shl 8;
  PARAMS_IMAGECACHESIZE  = 1 shl 9;
  PARAMS_PIXMAPCACHESIZE = 1 shl 10;

  PAGE_SIZE_EXECUTIVE    = 0;
  PAGE_SIZE_LETTER       = 1;
  PAGE_SIZE_LEGAL        = 2;
  PAGE_SIZE_A4           = 3;
  PAGE_SIZE_A3           = 4;
  PAGE_SIZE_A5           = 5;
  PAGE_SIZE_FOLIO        = 6;

  RT_PLAIN_PALETTE       = 0;
  RT_PLAIN_PALETTE_FAST  = 1;
  RT_DITHER_PALETTE      = 2;
  RT_DITHER_PALETTE_FAST = 3;
  RT_PLAIN_TRUECOL       = 4;
  RT_DITHER_TRUECOL      = 5;

procedure gdk_imlib_init;cdecl;external gdk_imlibdll name 'gdk_imlib_init';
procedure gdk_imlib_init_params(p:PGdkImlibInitParams);cdecl;external gdk_imlibdll name 'gdk_imlib_init_params';
function gdk_imlib_get_render_type:gint;cdecl;external gdk_imlibdll name 'gdk_imlib_get_render_type';
procedure gdk_imlib_set_render_type(rend_type:gint);cdecl;external gdk_imlibdll name 'gdk_imlib_set_render_type';
function gdk_imlib_load_colors(afile:Pchar):gint;cdecl;external gdk_imlibdll name 'gdk_imlib_load_colors';
function gdk_imlib_load_image(afile:Pchar):PGdkImlibImage;cdecl;external gdk_imlibdll name 'gdk_imlib_load_image';
function gdk_imlib_load_alpha(afile:Pchar):PGdkImlibImage;cdecl;external gdk_imlibdll name 'gdk_imlib_load_alpha';
function gdk_imlib_best_color_match(r:Pgint; g:Pgint; b:Pgint):gint;cdecl;external gdk_imlibdll name 'gdk_imlib_best_color_match';
procedure gdk_imlib_best_color_get(c:PGdkColor);cdecl;external gdk_imlibdll name 'gdk_imlib_best_color_get';
function gdk_imlib_render(image:PGdkImlibImage; width:gint; height:gint):gint;cdecl;external gdk_imlibdll name 'gdk_imlib_render';
function gdk_imlib_copy_image(image:PGdkImlibImage):PGdkPixmap;cdecl;external gdk_imlibdll name 'gdk_imlib_copy_image';
function gdk_imlib_copy_mask(image:PGdkImlibImage):PGdkBitmap;cdecl;external gdk_imlibdll name 'gdk_imlib_copy_mask';
function gdk_imlib_move_image(image:PGdkImlibImage):PGdkPixmap;cdecl;external gdk_imlibdll name 'gdk_imlib_move_image';
function gdk_imlib_move_mask(image:PGdkImlibImage):PGdkBitmap;cdecl;external gdk_imlibdll name 'gdk_imlib_move_mask';
procedure gdk_imlib_destroy_image(image:PGdkImlibImage);cdecl;external gdk_imlibdll name 'gdk_imlib_destroy_image';
procedure gdk_imlib_kill_image(image:PGdkImlibImage);cdecl;external gdk_imlibdll name 'gdk_imlib_kill_image';
procedure gdk_imlib_free_colors;cdecl;external gdk_imlibdll name 'gdk_imlib_free_colors';
procedure gdk_imlib_free_pixmap(pixmap:PGdkPixmap);cdecl;external gdk_imlibdll name 'gdk_imlib_free_pixmap';
procedure gdk_imlib_free_bitmap(bitmap:PGdkBitmap);cdecl;external gdk_imlibdll name 'gdk_imlib_free_bitmap';
procedure gdk_imlib_get_image_border(image:PGdkImlibImage; border:PGdkImlibBorder);cdecl;external gdk_imlibdll name 'gdk_imlib_get_image_border';
procedure gdk_imlib_set_image_border(image:PGdkImlibImage; border:PGdkImlibBorder);cdecl;external gdk_imlibdll name 'gdk_imlib_set_image_border';
procedure gdk_imlib_get_image_shape(image:PGdkImlibImage; color:PGdkImlibColor);cdecl;external gdk_imlibdll name 'gdk_imlib_get_image_shape';
procedure gdk_imlib_set_image_shape(image:PGdkImlibImage; color:PGdkImlibColor);cdecl;external gdk_imlibdll name 'gdk_imlib_set_image_shape';
function gdk_imlib_save_image_to_eim(image:PGdkImlibImage; afile:Pchar):gint;cdecl;external gdk_imlibdll name 'gdk_imlib_save_image_to_eim';
function gdk_imlib_add_image_to_eim(image:PGdkImlibImage; afile:Pchar):gint;cdecl;external gdk_imlibdll name 'gdk_imlib_add_image_to_eim';
function gdk_imlib_save_image_to_ppm(image:PGdkImlibImage; afile:Pchar):gint;cdecl;external gdk_imlibdll name 'gdk_imlib_save_image_to_ppm';
function gdk_imlib_load_file_to_pixmap(filename:Pchar; pmap:PPGdkPixmap; mask:PPGdkBitmap):gint;cdecl;external gdk_imlibdll name 'gdk_imlib_load_file_to_pixmap';
procedure gdk_imlib_set_image_modifier(image:PGdkImlibImage; mods:PGdkImlibColorModifier);cdecl;external gdk_imlibdll name 'gdk_imlib_set_image_modifier';
procedure gdk_imlib_set_image_red_modifier(image:PGdkImlibImage; mods:PGdkImlibColorModifier);cdecl;external gdk_imlibdll name 'gdk_imlib_set_image_red_modifier';
procedure gdk_imlib_set_image_green_modifier(image:PGdkImlibImage; mods:PGdkImlibColorModifier);cdecl;external gdk_imlibdll name 'gdk_imlib_set_image_green_modifier';
procedure gdk_imlib_set_image_blue_modifier(image:PGdkImlibImage; mods:PGdkImlibColorModifier);cdecl;external gdk_imlibdll name 'gdk_imlib_set_image_blue_modifier';
procedure gdk_imlib_get_image_modifier(image:PGdkImlibImage; mods:PGdkImlibColorModifier);cdecl;external gdk_imlibdll name 'gdk_imlib_get_image_modifier';
procedure gdk_imlib_get_image_red_modifier(image:PGdkImlibImage; mods:PGdkImlibColorModifier);cdecl;external gdk_imlibdll name 'gdk_imlib_get_image_red_modifier';
procedure gdk_imlib_get_image_green_modifier(image:PGdkImlibImage; mods:PGdkImlibColorModifier);cdecl;external gdk_imlibdll name 'gdk_imlib_get_image_green_modifier';
procedure gdk_imlib_get_image_blue_modifier(image:PGdkImlibImage; mods:PGdkImlibColorModifier);cdecl;external gdk_imlibdll name 'gdk_imlib_get_image_blue_modifier';
procedure gdk_imlib_set_image_red_curve(image:PGdkImlibImage; mods:Pbyte);cdecl;external gdk_imlibdll name 'gdk_imlib_set_image_red_curve';
procedure gdk_imlib_set_image_green_curve(image:PGdkImlibImage; mods:Pbyte);cdecl;external gdk_imlibdll name 'gdk_imlib_set_image_green_curve';
procedure gdk_imlib_set_image_blue_curve(image:PGdkImlibImage; mods:Pbyte);cdecl;external gdk_imlibdll name 'gdk_imlib_set_image_blue_curve';
procedure gdk_imlib_get_image_red_curve(image:PGdkImlibImage; mods:Pbyte);cdecl;external gdk_imlibdll name 'gdk_imlib_get_image_red_curve';
procedure gdk_imlib_get_image_green_curve(image:PGdkImlibImage; mods:Pbyte);cdecl;external gdk_imlibdll name 'gdk_imlib_get_image_green_curve';
procedure gdk_imlib_get_image_blue_curve(image:PGdkImlibImage; mods:Pbyte);cdecl;external gdk_imlibdll name 'gdk_imlib_get_image_blue_curve';
procedure gdk_imlib_apply_modifiers_to_rgb(image:PGdkImlibImage);cdecl;external gdk_imlibdll name 'gdk_imlib_apply_modifiers_to_rgb';
procedure gdk_imlib_changed_image(image:PGdkImlibImage);cdecl;external gdk_imlibdll name 'gdk_imlib_changed_image';
procedure gdk_imlib_apply_image(image:PGdkImlibImage; p:PGdkWindow);cdecl;external gdk_imlibdll name 'gdk_imlib_apply_image';
procedure gdk_imlib_paste_image(image:PGdkImlibImage; p:PGdkWindow; x:gint; y:gint; w:gint;
            h:gint);cdecl;external gdk_imlibdll name 'gdk_imlib_paste_image';
procedure gdk_imlib_paste_image_border(image:PGdkImlibImage; p:PGdkWindow; x:gint; y:gint; w:gint;
            h:gint);cdecl;external gdk_imlibdll name 'gdk_imlib_paste_image_border';
procedure gdk_imlib_flip_image_horizontal(image:PGdkImlibImage);cdecl;external gdk_imlibdll name 'gdk_imlib_flip_image_horizontal';
procedure gdk_imlib_flip_image_vertical(image:PGdkImlibImage);cdecl;external gdk_imlibdll name 'gdk_imlib_flip_image_vertical';
procedure gdk_imlib_rotate_image(image:PGdkImlibImage; d:gint);cdecl;external gdk_imlibdll name 'gdk_imlib_rotate_image';
function gdk_imlib_create_image_from_data(data:Pbyte; alpha:Pbyte; w:gint; h:gint):PGdkImlibImage;cdecl;external gdk_imlibdll name 'gdk_imlib_create_image_from_data';
function gdk_imlib_clone_image(image:PGdkImlibImage):PGdkImlibImage;cdecl;external gdk_imlibdll name 'gdk_imlib_clone_image';
function gdk_imlib_clone_scaled_image(image:PGdkImlibImage; w:longint; h:longint):PGdkImlibImage;cdecl;external gdk_imlibdll name 'gdk_imlib_clone_scaled_image';
function gdk_imlib_get_fallback:gint;cdecl;external gdk_imlibdll name 'gdk_imlib_get_fallback';
procedure gdk_imlib_set_fallback(fallback:gint);cdecl;external gdk_imlibdll name 'gdk_imlib_set_fallback';
function gdk_imlib_get_visual:PGdkVisual;cdecl;external gdk_imlibdll name 'gdk_imlib_get_visual';
function gdk_imlib_get_colormap:PGdkColormap;cdecl;external gdk_imlibdll name 'gdk_imlib_get_colormap';
function gdk_imlib_get_sysconfig:Pgchar;cdecl;external gdk_imlibdll name 'gdk_imlib_get_sysconfig';
function gdk_imlib_create_image_from_xpm_data(data:PPchar):PGdkImlibImage;cdecl;external gdk_imlibdll name 'gdk_imlib_create_image_from_xpm_data';
function gdk_imlib_data_to_pixmap(data:PPchar; pmap:PPGdkPixmap; mask:PPGdkBitmap):gint;cdecl;external gdk_imlibdll name 'gdk_imlib_data_to_pixmap';
procedure gdk_imlib_crop_image(image:PGdkImlibImage; x:gint; y:gint; w:gint; h:gint);cdecl;external gdk_imlibdll name 'gdk_imlib_crop_image';
function gdk_imlib_save_image(image:PGdkImlibImage; afile:Pchar; info:PGdkImlibSaveInfo):gint;cdecl;external gdk_imlibdll name 'gdk_imlib_save_image';
function gdk_imlib_crop_and_clone_image(image:PGdkImlibImage; x:longint; y:longint; w:longint; h:longint):PGdkImlibImage;cdecl;external gdk_imlibdll name 'gdk_imlib_crop_and_clone_image';
function gdk_imlib_create_image_from_drawable(gwin:PGdkWindow; gmask:PGdkBitmap; x:longint; y:longint; width:longint;
           height:longint):PGdkImlibImage;cdecl;external gdk_imlibdll name 'gdk_imlib_create_image_from_drawable';
function gdk_imlib_inlined_png_to_image(data:Pbyte; data_size:longint):PGdkImlibImage;cdecl;external gdk_imlibdll name 'gdk_imlib_inlined_png_to_image';
procedure gdk_imlib_get_cache_info(cache_pixmaps:Plongint; cache_images:Plongint);cdecl;external gdk_imlibdll name 'gdk_imlib_get_cache_info';
procedure gdk_imlib_set_cache_info(cache_pixmaps:longint; cache_images:longint);cdecl;external gdk_imlibdll name 'gdk_imlib_set_cache_info';

implementation


end.
