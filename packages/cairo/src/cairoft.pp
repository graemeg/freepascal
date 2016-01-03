unit CairoFT; 
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

Based on:

 Original translation of cairo-ft.h by Jeffrey Pohlmeyer 
 updated to version 1.4 by Luiz Américo Pereira Câmara 2007
<<<<<<< HEAD
<<<<<<< HEAD
 updated to version 1.12 by Valdinilson Lourenço da Cunha 2012
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

As per original authors wish, this file is dual licensed LGPL-MPL see the original file
cairo.pp for the full license.
}

{$mode objfpc}

interface

uses
  Cairo, freetypeh;

type
  //todo: properly define FcPattern:
  //It will require translate FontConfig header
  (*
  typedef struct _XftPattern {
    int		    num;
    int		    size;
    XftPatternElt   *elts;
   } XftPattern;
   typedef FcPattern XftPattern;
  *)
  FcPattern = Pointer;
  PFcPattern = ^FcPattern;
<<<<<<< HEAD
<<<<<<< HEAD

  cairo_ft_synthesize_t = (
    CAIRO_FT_SYNTHESIZE_BOLD = 1 shl 0,
    CAIRO_FT_SYNTHESIZE_OBLIQUE = 1 shl 1
  );

function  cairo_ft_font_face_create_for_ft_face(face:TFT_Face; load_flags:longint):Pcairo_font_face_t; cdecl; external LIB_CAIRO;
procedure cairo_ft_font_face_set_synthesize(font_face: Pcairo_font_face_t; synth_flags: LongWord); cdecl; external LIB_CAIRO;
procedure cairo_ft_font_face_unset_synthesize(font_face: Pcairo_font_face_t; synth_flags: LongWord);  cdecl; external LIB_CAIRO;
function  cairo_ft_font_face_get_synthesize(font_face: Pcairo_font_face_t): LongWord;  cdecl; external LIB_CAIRO;
function  cairo_ft_scaled_font_lock_face(scaled_font:Pcairo_scaled_font_t):TFT_Face; cdecl; external LIB_CAIRO;
procedure cairo_ft_scaled_font_unlock_face(scaled_font:Pcairo_scaled_font_t); cdecl; external LIB_CAIRO;
function  cairo_ft_font_face_create_for_pattern(pattern: PFcPattern): Pcairo_font_face_t; cdecl; external LIB_CAIRO;
procedure cairo_ft_font_options_substitute(options: Pcairo_font_options_t; pattern: PFcPattern); cdecl; external LIB_CAIRO;
=======
=======
>>>>>>> origin/fixes_2_2
    
function  cairo_ft_font_face_create_for_pattern(pattern:PFcPattern):Pcairo_font_face_t; cdecl; external LIB_CAIRO;
procedure cairo_ft_font_options_substitute(options:Pcairo_font_options_t; pattern:PFcPattern); cdecl; external LIB_CAIRO;
function  cairo_ft_font_face_create_for_ft_face(face:TFT_Face; load_flags:longint):Pcairo_font_face_t; cdecl; external LIB_CAIRO;
function  cairo_ft_scaled_font_lock_face(scaled_font:Pcairo_scaled_font_t):TFT_Face; cdecl; external LIB_CAIRO;
procedure cairo_ft_scaled_font_unlock_face(scaled_font:Pcairo_scaled_font_t); cdecl; external LIB_CAIRO;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

implementation

end.

