{

   Imlib library

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
unit Imlib;

interface

{$PACKRECORDS C}
{$mode objfpc}

Uses X, Xlib, XUtil;

const
{$ifndef os2}
  Imlibdll='Imlib';
{$else}
  Imlibdll='Imlib195';
{$endif}

Type
   PImlibBorder = ^TImlibBorder;
   TImlibBorder = record
        left : longint;
        right : longint;
        top : longint;
        bottom : longint;
     end;

   PImlibColor = ^TImlibColor;
   TImlibColor = record
        r : longint;
        g : longint;
        b : longint;
        pixel : longint;
     end;

   PImlibColorModifier = ^TImlibColorModifier;
   TImlibColorModifier = record
        gamma : longint;
        brightness : longint;
        contrast : longint;
     end;

   PImlibImage = ^TImlibImage;
   TImlibImage = record
        rgb_width : longint;
        rgb_height : longint;
        rgb_data : Pbyte;
        alpha_data : Pbyte;
        filename : PChar;
        width : longint;
        height : longint;
        shape_color : TImlibColor;
        border : TImlibBorder;
        pixmap : TPixmap;
        shape_mask : TPixmap;
        cache : char;
        mods : TImlibColorModifier;
        rmod : TImlibColorModifier;
        gmod : TImlibColorModifier;
        bmod : TImlibColorModifier;
        rmap : array[0..255] of byte;
        gmap : array[0..255] of byte;
        bmap : array[0..255] of byte;
     end;

   Pxdata = ^Txdata;
   Txdata = record
        disp : PDisplay;
        screen : longint;
        root : TWindow;
        visual : PVisual;
        depth : longint;
        render_depth : longint;
        root_cmap : TColormap;
        shm : char;
        shmp : char;
        shm_event : longint;
        last_xim : PXImage;
        last_sxim : PXImage;
        last_shminfo : Pointer;//XShmSegmentInfo;
        last_sshminfo :Pointer;//XShmSegmentInfo;
        base_window : TWindow;
        byte_order : longint;
        bit_order : longint;
     end;

   PImlibData = ^TImlibData;
   TImlibData = record
        num_colors : longint;
        palette : PImlibColor;
        palette_orig : PImlibColor;
        fast_rgb : Pbyte;
        fast_err : Plongint;
        fast_erg : Plongint;
        fast_erb : Plongint;
        render_type : longint;
        max_shm : longint;
        x : TXdata;
        byte_order : longint;
        cache : record
             on_image : char;
             size_image : longint;
             num_image : longint;
             used_image : longint;
             image : Pointer;//image_cache;
             on_pixmap : char;
             size_pixmap : longint;
             num_pixmap : longint;
             used_pixmap : longint;
             pixmap : Pointer;//pixmap_cache;
          end;
        fastrend : char;
        hiq : char;
        mods : TImlibColorModifier;
        rmod : TImlibColorModifier;
        gmod : TImlibColorModifier;
        bmod : TImlibColorModifier;
        rmap : array[0..255] of byte;
        gmap : array[0..255] of byte;
        bmap : array[0..255] of byte;
        fallback : char;
        ordered_dither : char;
     end;

   PImlibSaveInfo = ^TImlibSaveInfo;
   TImlibSaveInfo = record
        quality : longint;
        scaling : longint;
        xjustification : longint;
        yjustification : longint;
        page_size : longint;
        color : char;
     end;

   PImlibInitParams = ^TImlibInitParams;
   TImlibInitParams = record
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
        cmap : TColormap;
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

function Imlib_init(disp:PDisplay):PImlibData;cdecl;external imlibdll name 'Imlib_init';
function Imlib_init_with_params(disp:PDisplay; p:PImlibInitParams):PImlibData;cdecl;external imlibdll name 'Imlib_init_with_params';
function Imlib_get_render_type(id:PImlibData):longint;cdecl;external imlibdll name 'Imlib_get_render_type';
procedure Imlib_set_render_type(id:PImlibData; rend_type:longint);cdecl;external imlibdll name 'Imlib_set_render_type';
function Imlib_load_colors(id:PImlibData; thefile:Pchar):longint;cdecl;external imlibdll name 'Imlib_load_colors';
function Imlib_load_image(id:PImlibData; thefile:Pchar):PImlibImage;cdecl;external imlibdll name 'Imlib_load_image';
function Imlib_best_color_match(id:PImlibData; r:Plongint; g:Plongint; b:Plongint):longint;cdecl;external imlibdll name 'Imlib_best_color_match';
function Imlib_render(id:PImlibData; image:PImlibImage; width:longint; height:longint):longint;cdecl;external imlibdll name 'Imlib_render';
function Imlib_copy_image(id:PImlibData; image:PImlibImage):TPixmap;cdecl;external imlibdll name 'Imlib_copy_image';
function Imlib_copy_mask(id:PImlibData; image:PImlibImage):TPixmap;cdecl;external imlibdll name 'Imlib_copy_mask';
function Imlib_move_image(id:PImlibData; image:PImlibImage):TPixmap;cdecl;external imlibdll name 'Imlib_move_image';
function Imlib_move_mask(id:PImlibData; image:PImlibImage):TPixmap;cdecl;external imlibdll name 'Imlib_move_mask';
procedure Imlib_destroy_image(id:PImlibData; image:PImlibImage);cdecl;external imlibdll name 'Imlib_destroy_image';
procedure Imlib_kill_image(id:PImlibData; image:PImlibImage);cdecl;external imlibdll name 'Imlib_kill_image';
procedure Imlib_free_colors(id:PImlibData);cdecl;external imlibdll name 'Imlib_free_colors';
procedure Imlib_free_pixmap(id:PImlibData; pixmap:TPixmap);cdecl;external imlibdll name 'Imlib_free_pixmap';
procedure Imlib_get_image_border(id:PImlibData; image:PImlibImage; border:PImlibBorder);cdecl;external imlibdll name 'Imlib_get_image_border';
procedure Imlib_set_image_border(id:PImlibData; image:PImlibImage; border:PImlibBorder);cdecl;external imlibdll name 'Imlib_set_image_border';
procedure Imlib_get_image_shape(id:PImlibData; image:PImlibImage; color:PImlibColor);cdecl;external imlibdll name 'Imlib_get_image_shape';
procedure Imlib_set_image_shape(id:PImlibData; image:PImlibImage; color:PImlibColor);cdecl;external imlibdll name 'Imlib_set_image_shape';
function Imlib_save_image_to_eim(id:PImlibData; image:PImlibImage; thefile:Pchar):longint;cdecl;external imlibdll name 'Imlib_save_image_to_eim';
function Imlib_add_image_to_eim(id:PImlibData; image:PImlibImage; thefile:Pchar):longint;cdecl;external imlibdll name 'Imlib_add_image_to_eim';
function Imlib_save_image_to_ppm(id:PImlibData; image:PImlibImage; thefile:Pchar):longint;cdecl;external imlibdll name 'Imlib_save_image_to_ppm';
function Imlib_load_file_to_pixmap(id:PImlibData; filename:Pchar; pmap:PPixmap; mask:PPixmap):longint;cdecl;external imlibdll name 'Imlib_load_file_to_pixmap';
procedure Imlib_set_image_modifier(id:PImlibData; image:PImlibImage; mods:PImlibColorModifier);cdecl;external imlibdll name 'Imlib_set_image_modifier';
procedure Imlib_set_image_red_modifier(id:PImlibData; image:PImlibImage; mods:PImlibColorModifier);cdecl;external imlibdll name 'Imlib_set_image_red_modifier';
procedure Imlib_set_image_green_modifier(id:PImlibData; image:PImlibImage; mods:PImlibColorModifier);cdecl;external imlibdll name 'Imlib_set_image_green_modifier';
procedure Imlib_set_image_blue_modifier(id:PImlibData; image:PImlibImage; mods:PImlibColorModifier);cdecl;external imlibdll name 'Imlib_set_image_blue_modifier';
procedure Imlib_get_image_modifier(id:PImlibData; image:PImlibImage; mods:PImlibColorModifier);cdecl;external imlibdll name 'Imlib_get_image_modifier';
procedure Imlib_get_image_red_modifier(id:PImlibData; image:PImlibImage; mods:PImlibColorModifier);cdecl;external imlibdll name 'Imlib_get_image_red_modifier';
procedure Imlib_get_image_green_modifier(id:PImlibData; image:PImlibImage; mods:PImlibColorModifier);cdecl;external imlibdll name 'Imlib_get_image_green_modifier';
procedure Imlib_get_image_blue_modifier(id:PImlibData; image:PImlibImage; mods:PImlibColorModifier);cdecl;external imlibdll name 'Imlib_get_image_blue_modifier';
procedure Imlib_set_image_red_curve(id:PImlibData; image:PImlibImage; mods:Pbyte);cdecl;external imlibdll name 'Imlib_set_image_red_curve';
procedure Imlib_set_image_green_curve(id:PImlibData; image:PImlibImage; mods:Pbyte);cdecl;external imlibdll name 'Imlib_set_image_green_curve';
procedure Imlib_set_image_blue_curve(id:PImlibData; image:PImlibImage; mods:Pbyte);cdecl;external imlibdll name 'Imlib_set_image_blue_curve';
procedure Imlib_get_image_red_curve(id:PImlibData; image:PImlibImage; mods:Pbyte);cdecl;external imlibdll name 'Imlib_get_image_red_curve';
procedure Imlib_get_image_green_curve(id:PImlibData; image:PImlibImage; mods:Pbyte);cdecl;external imlibdll name 'Imlib_get_image_green_curve';
procedure Imlib_get_image_blue_curve(id:PImlibData; image:PImlibImage; mods:Pbyte);cdecl;external imlibdll name 'Imlib_get_image_blue_curve';
procedure Imlib_apply_modifiers_to_rgb(id:PImlibData; image:PImlibImage);cdecl;external imlibdll name 'Imlib_apply_modifiers_to_rgb';
procedure Imlib_changed_image(id:PImlibData; image:PImlibImage);cdecl;external imlibdll name 'Imlib_changed_image';
procedure Imlib_apply_image(id:PImlibData; image:PImlibImage; p:TWindow);cdecl;external imlibdll name 'Imlib_apply_image';
procedure Imlib_paste_image(id:PImlibData; image:PImlibImage; p:TWindow; x:longint; y:longint;
            w:longint; h:longint);cdecl;external imlibdll name 'Imlib_paste_image';
procedure Imlib_paste_image_border(id:PImlibData; image:PImlibImage; p:TWindow; x:longint; y:longint;
            w:longint; h:longint);cdecl;external imlibdll name 'Imlib_paste_image_border';
procedure Imlib_bevel_image(id:PImlibData; image:PImlibImage; bord:PImlibBorder; up:byte);cdecl;external imlibdll name 'Imlib_bevel_image';
procedure Imlib_bevel_pixmap(id:PImlibData; p:TPixmap; w:longint; h:longint; bord:PImlibBorder;
            up:byte);cdecl;external imlibdll name 'Imlib_bevel_pixmap';
procedure Imlib_flip_image_horizontal(id:PImlibData; image:PImlibImage);cdecl;external imlibdll name 'Imlib_flip_image_horizontal';
procedure Imlib_flip_image_vertical(id:PImlibData; image:PImlibImage);cdecl;external imlibdll name 'Imlib_flip_image_vertical';
procedure Imlib_rotate_image(id:PImlibData; image:PImlibImage; d:longint);cdecl;external imlibdll name 'Imlib_rotate_image';
function Imlib_create_image_from_data(id:PImlibData; data:Pbyte; alpha:Pbyte; w:longint; h:longint):PImlibimage;cdecl;external imlibdll name 'Imlib_create_image_from_data';
function Imlib_clone_image(id:PImlibData; image:PImlibImage):PImlibImage;cdecl;external imlibdll name 'Imlib_clone_image';
function Imlib_clone_scaled_image(id:PImlibData; image:PImlibImage; w:longint; h:longint):PImlibImage;cdecl;external imlibdll name 'Imlib_clone_scaled_image';
function Imlib_get_fallback(id:PImlibData):longint;cdecl;external imlibdll name 'Imlib_get_fallback';
procedure Imlib_set_fallback(id:PImlibData; fallback:longint);cdecl;external imlibdll name 'Imlib_set_fallback';
function Imlib_get_visual(id:PImlibData):PVisual;cdecl;external imlibdll name 'Imlib_get_visual';
function Imlib_get_colormap(id:PImlibData):TColormap;cdecl;external imlibdll name 'Imlib_get_colormap';
function Imlib_get_sysconfig(id:PImlibData):Pchar;cdecl;external imlibdll name 'Imlib_get_sysconfig';
function Imlib_create_image_from_xpm_data(id:PImlibData; data:PPchar):PImlibImage;cdecl;external imlibdll name 'Imlib_create_image_from_xpm_data';
function Imlib_data_to_pixmap(id:PImlibData; data:PPchar; pmap:PPixmap; mask:PPixmap):longint;cdecl;external imlibdll name 'Imlib_data_to_pixmap';
procedure Imlib_crop_image(id:PImlibData; image:PImlibImage; x:longint; y:longint; w:longint;
            h:longint);cdecl;external imlibdll name 'Imlib_crop_image';
function Imlib_save_image(id:PImlibData; image:PImlibImage; thefile:Pchar; info:PImlibSaveInfo):longint;cdecl;external imlibdll name 'Imlib_save_image';
function Imlib_crop_and_clone_image(id:PImlibData; image:PImlibImage; x:longint; y:longint; w:longint;
           h:longint):PImlibImage;cdecl;external imlibdll name 'Imlib_crop_and_clone_image';
function Imlib_create_image_from_drawable(id:PImlibData; win:TDrawable; mask:TPixmap; x:longint; y:longint;
           width:longint; height:longint):PImlibImage;cdecl;external imlibdll name 'Imlib_create_image_from_drawable';
function Imlib_inlined_png_to_image(id:PImlibData; data:Pbyte; data_size:longint):PImlibImage;cdecl;external imlibdll name 'Imlib_inlined_png_to_image';

implementation


end.
