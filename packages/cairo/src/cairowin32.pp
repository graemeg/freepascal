unit CairoWin32;
{
    This file is part of the Free Pascal libraries.
    Copyright (c) 2003-2008 by the Free Pascal development team

    Translation of cairo-win32.h 

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************

Translation of cairo-win32.h version 1.4 by Luiz Américo Pereira Câmara 2007

<<<<<<< HEAD
<<<<<<< HEAD
Updated to cairo for windows version 1.10 by Valdinilson Lourenço da Cunha 2012

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
For consistency with the other cairo header files, this file is dual
licensed LGPL-MPL see the file cairo.pp for the full license.
}

{$mode ObjFpc}

interface

uses
  Cairo, windows;

<<<<<<< HEAD
<<<<<<< HEAD
function  cairo_win32_surface_create(dc: HDC): Pcairo_surface_t; cdecl; external LIB_CAIRO;
function  cairo_win32_printing_surface_create(dc: HDC): Pcairo_surface_t; cdecl; external LIB_CAIRO;
function  cairo_win32_surface_create_with_ddb(dc: HDC; format: cairo_format_t; width, height: longint): Pcairo_surface_t; cdecl; external LIB_CAIRO;
function  cairo_win32_surface_create_with_dib(format: cairo_format_t; width, height: longint): Pcairo_surface_t; cdecl; external LIB_CAIRO;
function  cairo_win32_surface_get_dc(surface: pcairo_surface_t): HDC; cdecl; external LIB_CAIRO;
function  cairo_win32_surface_get_image(surface: pcairo_surface_t): Pcairo_surface_t; cdecl; external LIB_CAIRO;

(* Win32 font support *)

function  cairo_win32_font_face_create_for_logfontw(logfont: pLOGFONTW): Pcairo_font_face_t; cdecl; external LIB_CAIRO;
function  cairo_win32_font_face_create_for_hfont(font: HFONT): Pcairo_font_face_t; cdecl; external LIB_CAIRO;
function  cairo_win32_font_face_create_for_logfontw_hfont(logfont: PLOGFONTW; font: HFONT): Pcairo_font_face_t; cdecl; external LIB_CAIRO;
function  cairo_win32_scaled_font_select_font(scaled_font: pcairo_scaled_font_t; hdc: HDC): cairo_status_t; cdecl; external LIB_CAIRO;
procedure cairo_win32_scaled_font_done_font(scaled_font: pcairo_scaled_font_t); cdecl; external LIB_CAIRO;
function  cairo_win32_scaled_font_get_metrics_factor(scaled_font: pcairo_scaled_font_t): double; cdecl; external LIB_CAIRO;
=======
=======
>>>>>>> origin/fixes_2_2
function cairo_win32_surface_create(hdc: HDC): Pcairo_surface_t; cdecl; external LIB_CAIRO;
function cairo_win32_surface_create_with_ddb(hdc: HDC; format: cairo_format_t; width, height: longint): Pcairo_surface_t; cdecl; external LIB_CAIRO;
function cairo_win32_surface_create_with_dib(format: cairo_format_t; width, height: longint): Pcairo_surface_t; cdecl; external LIB_CAIRO;
function cairo_win32_surface_get_dc(surface: pcairo_surface_t): HDC; cdecl; external LIB_CAIRO;
function cairo_win32_surface_get_image(surface: pcairo_surface_t): Pcairo_surface_t; cdecl; external LIB_CAIRO;
function cairo_win32_font_face_create_for_logfontw(logfont: pLOGFONTW): Pcairo_font_face_t; cdecl; external LIB_CAIRO;
function cairo_win32_font_face_create_for_hfont(font: HFONT): Pcairo_font_face_t; cdecl; external LIB_CAIRO;
function cairo_win32_scaled_font_select_font(scaled_font: pcairo_scaled_font_t; hdc: HDC): cairo_status_t; cdecl; external LIB_CAIRO;
procedure cairo_win32_scaled_font_done_font(scaled_font: pcairo_scaled_font_t); cdecl; external LIB_CAIRO;
function cairo_win32_scaled_font_get_metrics_factor(scaled_font: pcairo_scaled_font_t): double; cdecl; external LIB_CAIRO;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
procedure cairo_win32_scaled_font_get_logical_to_device(scaled_font: pcairo_scaled_font_t; logical_to_device: pcairo_matrix_t); cdecl; external LIB_CAIRO;
procedure cairo_win32_scaled_font_get_device_to_logical(scaled_font: pcairo_scaled_font_t; device_to_logical: pcairo_matrix_t); cdecl; external LIB_CAIRO;

implementation

<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
end.
