unit CairoXlib;
{
    This file is part of the Free Pascal libraries.
    Copyright (c) 2003-2008 by the Free Pascal development team

    Translation of cairo-ft.h 

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************

 Translation of cairo-xlib.h version 1.4
 by Jeffrey Pohlmeyer 
 updated to version 1.4 by Luiz Américo Pereira Câmara 2007
<<<<<<< HEAD
<<<<<<< HEAD
 
 - Translation and addition of cairo-xlib-xrender.h
 - updated to version 1.12
 by Valdinilson Lourenço da Cunha 2012
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

 As per original authors wish, this file is dual licensed LGPL-MPL see the original file
  cairo.pp for the full license.
}

{$mode ObjFpc}

interface

uses
  Cairo, x, xlib, xrender;
  
function  cairo_xlib_surface_create(dpy: PDisplay; drawable: TDrawable; visual: PVisual; width, height: LongInt): Pcairo_surface_t; cdecl; external LIB_CAIRO;
function  cairo_xlib_surface_create_for_bitmap(dpy: PDisplay; bitmap: TPixmap; screen: PScreen; width, height: LongInt): Pcairo_surface_t; cdecl; external LIB_CAIRO;
<<<<<<< HEAD
<<<<<<< HEAD
procedure cairo_xlib_surface_set_size(surface: Pcairo_surface_t; width, height: LongInt); cdecl; external LIB_CAIRO;
procedure cairo_xlib_surface_set_drawable(surface: Pcairo_surface_t; drawable: TDrawable; width, height: LongInt); cdecl; external LIB_CAIRO;
function  cairo_xlib_surface_get_display(surface: Pcairo_surface_t): PDisplay; cdecl; external LIB_CAIRO;
function  cairo_xlib_surface_get_drawable(surface: Pcairo_surface_t): TDrawable; cdecl; external LIB_CAIRO;
function  cairo_xlib_surface_get_screen(surface: Pcairo_surface_t): PScreen; cdecl; external LIB_CAIRO;
function  cairo_xlib_surface_get_visual(surface: Pcairo_surface_t): PVisual; cdecl; external LIB_CAIRO;
function  cairo_xlib_surface_get_depth(surface: Pcairo_surface_t): LongInt; cdecl; external LIB_CAIRO;
function  cairo_xlib_surface_get_width(surface: Pcairo_surface_t): LongInt; cdecl; external LIB_CAIRO;
function  cairo_xlib_surface_get_height(surface: Pcairo_surface_t): LongInt; cdecl; external LIB_CAIRO;

(* debug interface *)

procedure cairo_xlib_device_debug_cap_xrender_version(device: Pcairo_device_t; major_version, minor_version: LongInt); cdecl; external LIB_CAIRO;
procedure cairo_xlib_device_debug_set_precision(device: Pcairo_device_t; precision: LongInt); cdecl; external LIB_CAIRO;
function  cairo_xlib_device_debug_get_precision(device: Pcairo_device_t): LongInt; cdecl; external LIB_CAIRO;

(* xlib render *)

function  cairo_xlib_surface_create_with_xrender_format (dpy: PDisplay; drawable: TDrawable; screen: PScreen; format: PXRenderPictFormat; width, height: LongInt): Pcairo_surface_t; cdecl; external LIB_CAIRO;
function  cairo_xlib_surface_get_xrender_format(surface: Pcairo_surface_t): PXRenderPictFormat; cdecl; external LIB_CAIRO;
=======
=======
>>>>>>> origin/fixes_2_2
function  cairo_xlib_surface_create_with_xrender_format(dpy: PDisplay; drawable: TDrawable; screen: PScreen; format: PXRenderPictFormat; width, height:LongInt): Pcairo_surface_t; cdecl; external LIB_CAIRO;
function  cairo_xlib_surface_get_depth(surface: Pcairo_surface_t): LongInt; cdecl; external LIB_CAIRO;
function  cairo_xlib_surface_get_display(surface: Pcairo_surface_t): PDisplay; cdecl; external LIB_CAIRO;
function  cairo_xlib_surface_get_drawable(surface: Pcairo_surface_t): TDrawable; cdecl; external LIB_CAIRO;
function  cairo_xlib_surface_get_height(surface: Pcairo_surface_t): LongInt; cdecl; external LIB_CAIRO;
function  cairo_xlib_surface_get_screen(surface: Pcairo_surface_t): PScreen; cdecl; external LIB_CAIRO;
function  cairo_xlib_surface_get_visual(surface: Pcairo_surface_t): PVisual; cdecl; external LIB_CAIRO;
function  cairo_xlib_surface_get_width(surface: Pcairo_surface_t): LongInt; cdecl; external LIB_CAIRO;
procedure cairo_xlib_surface_set_size(surface: Pcairo_surface_t; width, height: LongInt); cdecl; external LIB_CAIRO;
procedure cairo_xlib_surface_set_drawable(surface: Pcairo_surface_t; drawable: TDrawable; width, height: LongInt); cdecl; external LIB_CAIRO;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

implementation

end.
