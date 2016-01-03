{
    Free Pascal port of the Hermes C library.
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
    Original C version by Christian Nentwich (c.nentwich@cs.ucl.ac.uk)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
}

{
   Generic format conversion routines for the HERMES library
   Copyright (c) 1998 Christian Nentwich (c.nentwich@cs.ucl.ac.uk)
   This source code is licensed under the GNU LGPL

   Please refer to the file COPYING.LIB contained in the distribution for
   licensing conditions
}

Procedure ConvertP_Generic32_Generic32(iface : PHermesConverterInterface); CDecl;

Var
  s_pixel, r, g, b : int32;
  count : DWord;
  source, dest : Pchar8;

Begin
  source := iface^.s_pixels; dest := iface^.d_pixels;
  Repeat
    count := iface^.s_width;
    Repeat
      s_pixel := Pint32(source)^;

      r := ((s_pixel Shr iface^.info.r_right) Shl iface^.info.r_left) And
        iface^.mask_r;
      g := ((s_pixel Shr iface^.info.g_right) Shl iface^.info.g_left) And
        iface^.mask_g;
      b := ((s_pixel Shr iface^.info.b_right) Shl iface^.info.b_left) And
        iface^.mask_b;

      Pint32(dest)^ := r Or g Or b;

      Inc(source, 4);
      Inc(dest, 4);
      Dec(count);
    Until count = 0;

    Inc(source, iface^.s_add);
    Inc(dest, iface^.d_add);
    Dec(iface^.s_height);
  Until iface^.s_height = 0;
End;

Procedure ConvertP_Generic32_Generic24(iface : PHermesConverterInterface); CDecl;

Var
  s_pixel, r, g, b : int32;
  d_ptr : Pchar8;
  count : DWord;
  source, dest : Pchar8;

Begin
  d_ptr := @s_pixel;
  source := iface^.s_pixels; dest := iface^.d_pixels;
  Repeat
    count := iface^.s_width;
    Repeat
      s_pixel := Pint32(source)^;

      r := ((s_pixel Shr iface^.info.r_right) Shl iface^.info.r_left) And
        iface^.mask_r;
      g := ((s_pixel Shr iface^.info.g_right) Shl iface^.info.g_left) And
        iface^.mask_g;
      b := ((s_pixel Shr iface^.info.b_right) Shl iface^.info.b_left) And
        iface^.mask_b;

      s_pixel := r Or g Or b;

      (dest+R_24)^ := (d_ptr+R_32)^;
      (dest+G_24)^ := (d_ptr+G_32)^;
      (dest+B_24)^ := (d_ptr+B_32)^;

      Inc(source, 4);
      Inc(dest, 3);
      Dec(count);
    Until count = 0;

    Inc(source, iface^.s_add);
    Inc(dest, iface^.d_add);
    Dec(iface^.s_height);
  Until iface^.s_height = 0;
End;

Procedure ConvertP_Generic32_Generic16(iface : PHermesConverterInterface); CDecl;

Var
  s_pixel, r, g, b : int32;
  count : DWord;
  source, dest : Pchar8;

Begin
  source := iface^.s_pixels; dest := iface^.d_pixels;
  Repeat
    count := iface^.s_width Shr 1;

    If count <> 0 Then
      Repeat
        r := (((Pint32(source)^) Shr iface^.info.r_right) Shl iface^.info.r_left) And
              iface^.mask_r;
        g := (((Pint32(source)^) Shr iface^.info.g_right) Shl iface^.info.g_left) And
              iface^.mask_g;
        b := (((Pint32(source)^) Shr iface^.info.b_right) Shl iface^.info.b_left) And
              iface^.mask_b;

        s_pixel := (r Or g Or b) And $FFFF;

        r := ((((Pint32(source)+1)^) Shr iface^.info.r_right) Shl iface^.info.r_left) And
              iface^.mask_r;
        g := ((((Pint32(source)+1)^) Shr iface^.info.g_right) Shl iface^.info.g_left) And
              iface^.mask_g;
        b := ((((Pint32(source)+1)^) Shr iface^.info.b_right) Shl iface^.info.b_left) And
              iface^.mask_b;

        s_pixel := s_pixel Or ((r Or g Or b) Shl 16);

        Pint32(dest)^ := s_pixel;

        Inc(source, 8);
        Inc(dest, 4);
        Dec(count);
      Until count = 0;


    { Trailing pixel }

    If (iface^.s_width And 1) <> 0 Then
    Begin
      r := (((Pint32(source)^) Shr iface^.info.r_right) Shl iface^.info.r_left) And
            iface^.mask_r;
      g := (((Pint32(source)^) Shr iface^.info.g_right) Shl iface^.info.g_left) And
            iface^.mask_g;
      b := (((Pint32(source)^) Shr iface^.info.b_right) Shl iface^.info.b_left) And
            iface^.mask_b;

      Pshort16(dest)^ := r Or g Or b;
      Inc(dest, 2);
      Inc(source, 4);
    End;

    Inc(source, iface^.s_add);
    Inc(dest, iface^.d_add);
    Dec(iface^.s_height);
  Until iface^.s_height = 0;
End;

Procedure ConvertP_Generic32_Generic8(iface : PHermesConverterInterface); CDecl;

Var
  s_pixel, r, g, b : int32;
  count : DWord;
  source, dest : Pchar8;

Begin
  source := iface^.s_pixels; dest := iface^.d_pixels;
  Repeat
    count := iface^.s_width;
    Repeat
      s_pixel := Pint32(source)^;

      r := ((s_pixel Shr iface^.info.r_right) Shl iface^.info.r_left) And
        iface^.mask_r;
      g := ((s_pixel Shr iface^.info.g_right) Shl iface^.info.g_left) And
        iface^.mask_g;
      b := ((s_pixel Shr iface^.info.b_right) Shl iface^.info.b_left) And
        iface^.mask_b;

      dest^ := r Or g Or b;

      Inc(source, 4);
      Inc(dest);
      Dec(count);
    Until count = 0;

    Inc(source, iface^.s_add);
    Inc(dest, iface^.d_add);
    Dec(iface^.s_height);
  Until iface^.s_height = 0;
End;

Procedure ConvertP_Generic24_Generic32(iface : PHermesConverterInterface); CDecl;

Var
  s_pixel, r, g, b : int32;
  count : DWord;
  source, dest : Pchar8;

Begin
  source := iface^.s_pixels; dest := iface^.d_pixels;
  Repeat
    count := iface^.s_width;
    Repeat
      s_pixel := (((source+R_24)^) Shl 16) Or
                 (((source+G_24)^) Shl 8) Or
                 ((source+B_24)^);

      r := ((s_pixel Shr iface^.info.r_right) Shl iface^.info.r_left) And
        iface^.mask_r;
      g := ((s_pixel Shr iface^.info.g_right) Shl iface^.info.g_left) And
        iface^.mask_g;
      b := ((s_pixel Shr iface^.info.b_right) Shl iface^.info.b_left) And
        iface^.mask_b;

      Pint32(dest)^ := r Or g Or b;

      Inc(source, 3);
      Inc(dest, 4);
      Dec(count);
    Until count = 0;

    Inc(source, iface^.s_add);
    Inc(dest, iface^.d_add);
    Dec(iface^.s_height);
  Until iface^.s_height = 0;
End;

Procedure ConvertP_Generic24_Generic24(iface : PHermesConverterInterface); CDecl;

Var
  s_pixel, r, g, b : int32;
  d_ptr : Pchar8;
  count : DWord;
  source, dest : PChar8;

Begin
  d_ptr := @s_pixel;
  source := iface^.s_pixels; dest := iface^.d_pixels;
  Repeat
    count := iface^.s_width;
    Repeat
      s_pixel := (((source+R_24)^) Shl 16) Or
                 (((source+G_24)^) Shl 8) Or
                 ((source+B_24)^);

      r := ((s_pixel Shl iface^.info.r_left) Shr iface^.info.r_right) And
        iface^.mask_r;
      g := ((s_pixel Shl iface^.info.g_left) Shr iface^.info.g_right) And
        iface^.mask_g;
      b := ((s_pixel Shl iface^.info.b_left) Shr iface^.info.b_right) And
        iface^.mask_b;

      s_pixel := r Or g Or b;

      (dest + R_24)^ := (d_ptr + R_32)^;
      (dest + G_24)^ := (d_ptr + G_32)^;
      (dest + B_24)^ := (d_ptr + B_32)^;

      Inc(source, 3);
      Inc(dest, 3);
      Dec(count);
    Until count = 0;

    Inc(source, iface^.s_add);
    Inc(dest, iface^.d_add);
    Dec(iface^.s_height);
  Until iface^.s_height = 0;
End;

Procedure ConvertP_Generic24_Generic16(iface : PHermesConverterInterface); CDecl;

Var
  s_pixel, r, g, b : int32;
  count : DWord;
  source, dest : Pchar8;

Begin
  source := iface^.s_pixels; dest := iface^.d_pixels;
  Repeat
    count := iface^.s_width;
    Repeat
      s_pixel := (((source+R_24)^) Shl 16) Or
                 (((source+G_24)^) Shl 8) Or
                 ((source+B_24)^);

      r := ((s_pixel Shr iface^.info.r_right) Shl iface^.info.r_left) And
        iface^.mask_r;
      g := ((s_pixel Shr iface^.info.g_right) Shl iface^.info.g_left) And
        iface^.mask_g;
      b := ((s_pixel Shr iface^.info.b_right) Shl iface^.info.b_left) And
        iface^.mask_b;

      Pshort16(dest)^ := r Or g Or b;

      Inc(source, 3);
      Inc(dest, 2);
      Dec(count);
    Until count = 0;

    Inc(source, iface^.s_add);
    Inc(dest, iface^.d_add);
    Dec(iface^.s_height);
  Until iface^.s_height = 0;
End;

Procedure ConvertP_Generic24_Generic8(iface : PHermesConverterInterface); CDecl;

Var
  s_pixel, r, g, b : int32;
  count : DWord;
  source, dest : Pchar8;

Begin
  source := iface^.s_pixels; dest := iface^.d_pixels;
  Repeat
    count := iface^.s_width;
    Repeat
      s_pixel := (((source+R_24)^) Shl 16) Or
                 (((source+G_24)^) Shl 8) Or
                 ((source+B_24)^);

      r := ((s_pixel Shr iface^.info.r_right) Shl iface^.info.r_left) And
        iface^.mask_r;
      g := ((s_pixel Shr iface^.info.g_right) Shl iface^.info.g_left) And
        iface^.mask_g;
      b := ((s_pixel Shr iface^.info.b_right) Shl iface^.info.b_left) And
        iface^.mask_b;

      dest^ := r Or g Or b;

      Inc(source, 3);
      Inc(dest);
      Dec(count);
    Until count = 0;

    Inc(source, iface^.s_add);
    Inc(dest, iface^.d_add);
    Dec(iface^.s_height);
  Until iface^.s_height = 0;
End;

Procedure ConvertP_Generic16_Generic32(iface : PHermesConverterInterface); CDecl;

Var
  s_pixel, r, g, b : int32;
  count : DWord;
  source, dest : Pchar8;

Begin
  source := iface^.s_pixels; dest := iface^.d_pixels;
  Repeat
    count := iface^.s_width;
    Repeat
      s_pixel := Pshort16(source)^;

      r := ((s_pixel Shr iface^.info.r_right) Shl iface^.info.r_left) And
        iface^.mask_r;
      g := ((s_pixel Shr iface^.info.g_right) Shl iface^.info.g_left) And
        iface^.mask_g;
      b := ((s_pixel Shr iface^.info.b_right) Shl iface^.info.b_left) And
        iface^.mask_b;

      Pint32(dest)^ := r Or g Or b;

      Inc(source, 2);
      Inc(dest, 4);
      Dec(count);
    Until count = 0;

    Inc(source, iface^.s_add);
    Inc(dest, iface^.d_add);
    Dec(iface^.s_height);
  Until iface^.s_height = 0;
End;

Procedure ConvertP_Generic16_Generic24(iface : PHermesConverterInterface); CDecl;

Var
  s_pixel, r, g, b : int32;
  d_ptr : Pchar8;
  count : DWord;
  source, dest : Pchar8;

Begin
  d_ptr := @s_pixel;
  source := iface^.s_pixels; dest := iface^.d_pixels;
  Repeat
    count := iface^.s_width;
    Repeat
      s_pixel := Pshort16(source)^;

      r := ((s_pixel Shr iface^.info.r_right) Shl iface^.info.r_left) And
        iface^.mask_r;
      g := ((s_pixel Shr iface^.info.g_right) Shl iface^.info.g_left) And
        iface^.mask_g;
      b := ((s_pixel Shr iface^.info.b_right) Shl iface^.info.b_left) And
        iface^.mask_b;

      s_pixel := r Or g Or b;

      (dest + R_24)^ := (d_ptr + R_32)^;
      (dest + G_24)^ := (d_ptr + G_32)^;
      (dest + B_24)^ := (d_ptr + B_32)^;

      Inc(source, 2);
      Inc(dest, 3);
      Dec(count);
    Until count = 0;

    Inc(source, iface^.s_add);
    Inc(dest, iface^.d_add);
    Dec(iface^.s_height);
  Until iface^.s_height = 0;
End;

Procedure ConvertP_Generic16_Generic16(iface : PHermesConverterInterface); CDecl;

Var
  s_pixel, r, g, b : int32;
  count : DWord;
  source, dest : Pchar8;

Begin
  source := iface^.s_pixels; dest := iface^.d_pixels;
  Repeat
    count := iface^.s_width;
    Repeat
      s_pixel := Pshort16(source)^;

      r := ((s_pixel Shr iface^.info.r_right) Shl iface^.info.r_left) And
        iface^.mask_r;
      g := ((s_pixel Shr iface^.info.g_right) Shl iface^.info.g_left) And
        iface^.mask_g;
      b := ((s_pixel Shr iface^.info.b_right) Shl iface^.info.b_left) And
        iface^.mask_b;

      Pshort16(dest)^ := r Or g Or b;

      Inc(source, 2);
      Inc(dest, 2);
      Dec(count);
    Until count = 0;

    Inc(source, iface^.s_add);
    Inc(dest, iface^.d_add);
    Dec(iface^.s_height);
  Until iface^.s_height = 0;
End;

Procedure ConvertP_Generic16_Generic8(iface : PHermesConverterInterface); CDecl;

Var
  s_pixel, r, g, b : int32;
  count : DWord;
  source, dest : Pchar8;

Begin
  source := iface^.s_pixels; dest := iface^.d_pixels;
  Repeat
    count := iface^.s_width;
    Repeat
      s_pixel := Pshort16(source)^;

      r := ((s_pixel Shr iface^.info.r_right) Shl iface^.info.r_left) And
        iface^.mask_r;
      g := ((s_pixel Shr iface^.info.g_right) Shl iface^.info.g_left) And
        iface^.mask_g;
      b := ((s_pixel Shr iface^.info.b_right) Shl iface^.info.b_left) And
        iface^.mask_b;

      dest^ := r Or g Or b;

      Inc(source, 2);
      Inc(dest);
      Dec(count);
    Until count = 0;

    Inc(source, iface^.s_add);
    Inc(dest, iface^.d_add);
    Dec(iface^.s_height);
  Until iface^.s_height = 0;
End;

Procedure ConvertP_Generic8_Generic32(iface : PHermesConverterInterface); CDecl;

Begin
  {todo}
End;

Procedure ConvertP_Generic8_Generic24(iface : PHermesConverterInterface); CDecl;

Begin
  {todo}
End;

Procedure ConvertP_Generic8_Generic16(iface : PHermesConverterInterface); CDecl;

Begin
  {todo}
End;

Procedure ConvertP_Generic8_Generic8(iface : PHermesConverterInterface); CDecl;

Begin
  {todo}
End;

{ -------------------------------------------------------------------------

                              STRETCH CONVERTERS

  ------------------------------------------------------------------------- }

Procedure ConvertP_Generic32_Generic32_S(iface : PHermesConverterInterface); CDecl;

Var
  s_pixel, r, g, b : int32;
  count : DWord;
  source, dest : Pchar8;
  dx, dy, x, y : DWord;

Begin
  source := iface^.s_pixels;
  dest := iface^.d_pixels;
  dy := (iface^.s_height Shl 16) Div iface^.d_height;
  dx := (iface^.s_width Shl 16) Div iface^.d_width;
  y := 0;
  Repeat
    count := iface^.d_width;
    x := 0;
    Repeat
      s_pixel := (Pint32(source)+(x Shr 16))^;

      r := ((s_pixel Shr iface^.info.r_right) Shl iface^.info.r_left) And
        iface^.mask_r;
      g := ((s_pixel Shr iface^.info.g_right) Shl iface^.info.g_left) And
        iface^.mask_g;
      b := ((s_pixel Shr iface^.info.b_right) Shl iface^.info.b_left) And
        iface^.mask_b;

      Pint32(dest)^ := r Or g Or b;

      Inc(x, dx);
      Inc(dest, 4);
      Dec(count);
    Until count = 0;

    Inc(dest, iface^.d_add);

    Inc(y, dy);
    Inc(source, (y Shr 16)*DWord(iface^.s_pitch));
    y := y And $ffff;

    Dec(iface^.d_height);
  Until iface^.d_height = 0;
End;

Procedure ConvertP_Generic32_Generic24_S(iface : PHermesConverterInterface); CDecl;

Var
  s_pixel, r, g, b : int32;
  d_ptr : Pchar8;
  count : DWord;
  source, dest : Pchar8;
  dx, dy, x, y : DWord;

Begin
  d_ptr := @s_pixel;
  source := iface^.s_pixels;
  dest := iface^.d_pixels;
  dy := (iface^.s_height Shl 16) Div iface^.d_height;
  dx := (iface^.s_width Shl 16) Div iface^.d_width;
  y := 0;
  Repeat
    count := iface^.d_width;
    x := 0;
    Repeat
      s_pixel := (Pint32(source)+(x Shr 16))^;

      r := ((s_pixel Shr iface^.info.r_right) Shl iface^.info.r_left) And
        iface^.mask_r;
      g := ((s_pixel Shr iface^.info.g_right) Shl iface^.info.g_left) And
        iface^.mask_g;
      b := ((s_pixel Shr iface^.info.b_right) Shl iface^.info.b_left) And
        iface^.mask_b;

      s_pixel := r Or g Or b;

      (dest + R_24)^ := (d_ptr + R_32)^;
      (dest + G_24)^ := (d_ptr + G_32)^;
      (dest + B_24)^ := (d_ptr + B_32)^;

      Inc(x, dx);
      Inc(dest, 3);
      Dec(count);
    Until count = 0;

    Inc(dest, iface^.d_add);

    Inc(y, dy);
    Inc(source, (y Shr 16)*DWord(iface^.s_pitch));
    y := y And $ffff;

    Dec(iface^.d_height);
  Until iface^.d_height = 0;
End;

Procedure ConvertP_Generic32_Generic16_S(iface : PHermesConverterInterface); CDecl;

Var
  s_pixel, r, g, b : int32;
  count : DWord;
  source, dest : Pchar8;
  dx, dy, x, y : DWord;

Begin
  source := iface^.s_pixels;
  dest := iface^.d_pixels;
  dy := (iface^.s_height Shl 16) Div iface^.d_height;
  dx := (iface^.s_width Shl 16) Div iface^.d_width;
  y := 0;
  Repeat
    count := iface^.d_width;
    x := 0;
    Repeat
      s_pixel := (Pint32(source)+(x Shr 16))^;

      r := ((s_pixel Shr iface^.info.r_right) Shl iface^.info.r_left) And
        iface^.mask_r;
      g := ((s_pixel Shr iface^.info.g_right) Shl iface^.info.g_left) And
        iface^.mask_g;
      b := ((s_pixel Shr iface^.info.b_right) Shl iface^.info.b_left) And
        iface^.mask_b;

      Pshort16(dest)^ := r Or g Or b;

      Inc(x, dx);
      Inc(dest, 2);
      Dec(count);
    Until count = 0;

    Inc(dest, iface^.d_add);

    Inc(y, dy);
    Inc(source, (y Shr 16)*DWord(iface^.s_pitch));
    y := y And $ffff;

    Dec(iface^.d_height);
  Until iface^.d_height = 0;
End;

Procedure ConvertP_Generic32_Generic8_S(iface : PHermesConverterInterface); CDecl;

Var
  s_pixel, r, g, b : int32;
  count : DWord;
  source, dest : Pchar8;
  dx, dy, x, y : DWord;

Begin
  source := iface^.s_pixels;
  dest := iface^.d_pixels;
  dy := (iface^.s_height Shl 16) Div iface^.d_height;
  dx := (iface^.s_width Shl 16) Div iface^.d_width;
  y := 0;
  Repeat
    count := iface^.d_width;
    x := 0;
    Repeat
      s_pixel := (Pint32(source)+(x Shr 16))^;

      r := ((s_pixel Shr iface^.info.r_right) Shl iface^.info.r_left) And
        iface^.mask_r;
      g := ((s_pixel Shr iface^.info.g_right) Shl iface^.info.g_left) And
        iface^.mask_g;
      b := ((s_pixel Shr iface^.info.b_right) Shl iface^.info.b_left) And
        iface^.mask_b;

      dest^ := r Or g Or b;

      Inc(x, dx);
      Inc(dest);
      Dec(count);
    Until count = 0;

    Inc(dest, iface^.d_add);

    Inc(y, dy);
    Inc(source, (y Shr 16)*DWord(iface^.s_pitch));
    y := y And $ffff;

    Dec(iface^.d_height);
  Until iface^.d_height = 0;
End;

Procedure ConvertP_Generic24_Generic32_S(iface : PHermesConverterInterface); CDecl;

Var
  s_pixel, r, g, b : int32;
  count : DWord;
  source, src, dest : Pchar8;
  dx, dy, x, y : DWord;

Begin
  source := iface^.s_pixels;
  dest := iface^.d_pixels;
  dy := (iface^.s_height Shl 16) Div iface^.d_height;
  dx := (iface^.s_width Shl 16) Div iface^.d_width;
  y := 0;
  Repeat
    count := iface^.d_width;
    x := 0;
    src := source;
    Repeat
      s_pixel := (((src+R_24)^) Shl 16) Or
                 (((src+G_24)^) Shl 8) Or
                 ((src+B_24)^);

      r := ((s_pixel Shr iface^.info.r_right) Shl iface^.info.r_left) And
        iface^.mask_r;
      g := ((s_pixel Shr iface^.info.g_right) Shl iface^.info.g_left) And
        iface^.mask_g;
      b := ((s_pixel Shr iface^.info.b_right) Shl iface^.info.b_left) And
        iface^.mask_b;

      Pint32(dest)^ := r Or g Or b;

      Inc(x, dx);
      Inc(src, x Shr 16);
      Inc(src, x Shr 16);
      Inc(src, x Shr 16);
      x := x And $FFFF;
      Inc(dest, 4);
      Dec(count);
    Until count = 0;

    Inc(dest, iface^.d_add);

    Inc(y, dy);
    Inc(source, (y Shr 16)*DWord(iface^.s_pitch));
    y := y And $ffff;

    Dec(iface^.d_height);
  Until iface^.d_height = 0;
End;

Procedure ConvertP_Generic24_Generic24_S(iface : PHermesConverterInterface); CDecl;

Var
  s_pixel, r, g, b : int32;
  d_ptr : Pchar8;
  count : DWord;
  source, src, dest : Pchar8;
  dx, dy, x, y : DWord;

Begin
  d_ptr := @s_pixel;
  source := iface^.s_pixels;
  dest := iface^.d_pixels;
  dy := (iface^.s_height Shl 16) Div iface^.d_height;
  dx := (iface^.s_width Shl 16) Div iface^.d_width;
  y := 0;
  Repeat
    count := iface^.d_width;
    x := 0;
    src := source;
    Repeat
      s_pixel := (((src+R_24)^) Shl 16) Or
                 (((src+G_24)^) Shl 8) Or
                 ((src+B_24)^);

      r := ((s_pixel Shr iface^.info.r_right) Shl iface^.info.r_left) And
        iface^.mask_r;
      g := ((s_pixel Shr iface^.info.g_right) Shl iface^.info.g_left) And
        iface^.mask_g;
      b := ((s_pixel Shr iface^.info.b_right) Shl iface^.info.b_left) And
        iface^.mask_b;

      s_pixel := r Or g Or b;

      (dest + R_24)^ := (d_ptr + R_32)^;
      (dest + G_24)^ := (d_ptr + G_32)^;
      (dest + B_24)^ := (d_ptr + B_32)^;

      Inc(x, dx);
      Inc(src, x Shr 16);
      Inc(src, x Shr 16);
      Inc(src, x Shr 16);
      x := x And $FFFF;
      Inc(dest, 3);
      Dec(count);
    Until count = 0;

    Inc(dest, iface^.d_add);

    Inc(y, dy);
    Inc(source, (y Shr 16)*DWord(iface^.s_pitch));
    y := y And $ffff;

    Dec(iface^.d_height);
  Until iface^.d_height = 0;
End;

Procedure ConvertP_Generic24_Generic16_S(iface : PHermesConverterInterface); CDecl;

Var
  s_pixel, r, g, b : int32;
  count : DWord;
  source, src, dest : Pchar8;
  dx, dy, x, y : DWord;

Begin
  source := iface^.s_pixels;
  dest := iface^.d_pixels;
  dy := (iface^.s_height Shl 16) Div iface^.d_height;
  dx := (iface^.s_width Shl 16) Div iface^.d_width;
  y := 0;
  Repeat
    count := iface^.d_width;
    x := 0;
    src := source;
    Repeat
      s_pixel := (((src+R_24)^) Shl 16) Or
                 (((src+G_24)^) Shl 8) Or
                 ((src+B_24)^);

      r := ((s_pixel Shr iface^.info.r_right) Shl iface^.info.r_left) And
        iface^.mask_r;
      g := ((s_pixel Shr iface^.info.g_right) Shl iface^.info.g_left) And
        iface^.mask_g;
      b := ((s_pixel Shr iface^.info.b_right) Shl iface^.info.b_left) And
        iface^.mask_b;

      Pshort16(dest)^ := r Or g Or b;

      Inc(x, dx);
      Inc(src, x Shr 16);
      Inc(src, x Shr 16);
      Inc(src, x Shr 16);
      x := x And $FFFF;
      Inc(dest, 2);
      Dec(count);
    Until count = 0;

    Inc(dest, iface^.d_add);

    Inc(y, dy);
    Inc(source, (y Shr 16)*DWord(iface^.s_pitch));
    y := y And $ffff;

    Dec(iface^.d_height);
  Until iface^.d_height = 0;
End;

Procedure ConvertP_Generic24_Generic8_S(iface : PHermesConverterInterface); CDecl;

Var
  s_pixel, r, g, b : int32;
  count : DWord;
  source, src, dest : Pchar8;
  dx, dy, x, y : DWord;

Begin
  source := iface^.s_pixels;
  dest := iface^.d_pixels;
  dy := (iface^.s_height Shl 16) Div iface^.d_height;
  dx := (iface^.s_width Shl 16) Div iface^.d_width;
  y := 0;
  Repeat
    count := iface^.d_width;
    x := 0;
    src := source;
    Repeat
      s_pixel := (((src+R_24)^) Shl 16) Or
                 (((src+G_24)^) Shl 8) Or
                 ((src+B_24)^);

      r := ((s_pixel Shr iface^.info.r_right) Shl iface^.info.r_left) And
        iface^.mask_r;
      g := ((s_pixel Shr iface^.info.g_right) Shl iface^.info.g_left) And
        iface^.mask_g;
      b := ((s_pixel Shr iface^.info.b_right) Shl iface^.info.b_left) And
        iface^.mask_b;

      dest^ := r Or g Or b;

      Inc(x, dx);
      Inc(src, x Shr 16);
      Inc(src, x Shr 16);
      Inc(src, x Shr 16);
      x := x And $FFFF;
      Inc(dest);
      Dec(count);
    Until count = 0;

    Inc(dest, iface^.d_add);

    Inc(y, dy);
    Inc(source, (y Shr 16)*DWord(iface^.s_pitch));
    y := y And $ffff;

    Dec(iface^.d_height);
  Until iface^.d_height = 0;
End;

Procedure ConvertP_Generic16_Generic32_S(iface : PHermesConverterInterface); CDecl;

Var
  s_pixel, r, g, b : int32;
  count : DWord;
  source, dest : Pchar8;
  dx, dy, x, y : DWord;

Begin
  source := iface^.s_pixels;
  dest := iface^.d_pixels;
  dy := (iface^.s_height Shl 16) Div iface^.d_height;
  dx := (iface^.s_width Shl 16) Div iface^.d_width;
  y := 0;
  Repeat
    count := iface^.d_width;
    x := 0;
    Repeat
      s_pixel := (Pshort16(source)+(x Shr 16))^;

      r := ((s_pixel Shr iface^.info.r_right) Shl iface^.info.r_left) And
        iface^.mask_r;
      g := ((s_pixel Shr iface^.info.g_right) Shl iface^.info.g_left) And
        iface^.mask_g;
      b := ((s_pixel Shr iface^.info.b_right) Shl iface^.info.b_left) And
        iface^.mask_b;

      Pint32(dest)^ := r Or g Or b;

      Inc(x, dx);
      Inc(dest, 4);
      Dec(count);
    Until count = 0;

    Inc(dest, iface^.d_add);

    Inc(y, dy);
    Inc(source, (y Shr 16)*DWord(iface^.s_pitch));
    y := y And $ffff;

    Dec(iface^.d_height);
  Until iface^.d_height = 0;
End;

Procedure ConvertP_Generic16_Generic24_S(iface : PHermesConverterInterface); CDecl;

Var
  s_pixel, r, g, b : int32;
  d_ptr : Pchar8;
  count : DWord;
  source, dest : Pchar8;
  dx, dy, x, y : DWord;

Begin
  d_ptr := @s_pixel;
  source := iface^.s_pixels;
  dest := iface^.d_pixels;
  dy := (iface^.s_height Shl 16) Div iface^.d_height;
  dx := (iface^.s_width Shl 16) Div iface^.d_width;
  y := 0;
  Repeat
    count := iface^.d_width;
    x := 0;
    Repeat
      s_pixel := (Pshort16(source)+(x Shr 16))^;

      r := ((s_pixel Shr iface^.info.r_right) Shl iface^.info.r_left) And
        iface^.mask_r;
      g := ((s_pixel Shr iface^.info.g_right) Shl iface^.info.g_left) And
        iface^.mask_g;
      b := ((s_pixel Shr iface^.info.b_right) Shl iface^.info.b_left) And
        iface^.mask_b;

      s_pixel := r Or g Or b;

      (dest + R_24)^ := (d_ptr + R_32)^;
      (dest + G_24)^ := (d_ptr + G_32)^;
      (dest + B_24)^ := (d_ptr + B_32)^;

      Inc(x, dx);
      Inc(dest, 3);
      Dec(count);
    Until count = 0;

    Inc(dest, iface^.d_add);

    Inc(y, dy);
    Inc(source, (y Shr 16)*DWord(iface^.s_pitch));
    y := y And $ffff;

    Dec(iface^.d_height);
  Until iface^.d_height = 0;
End;

Procedure ConvertP_Generic16_Generic16_S(iface : PHermesConverterInterface); CDecl;

Var
  s_pixel, r, g, b : int32;
  count : DWord;
  source, dest : Pchar8;
  dx, dy, x, y : DWord;

Begin
  source := iface^.s_pixels;
  dest := iface^.d_pixels;
  dy := (iface^.s_height Shl 16) Div iface^.d_height;
  dx := (iface^.s_width Shl 16) Div iface^.d_width;
  y := 0;
  Repeat
    count := iface^.d_width;
    x := 0;
    Repeat
      s_pixel := (Pshort16(source)+(x Shr 16))^;

      r := ((s_pixel Shr iface^.info.r_right) Shl iface^.info.r_left) And
        iface^.mask_r;
      g := ((s_pixel Shr iface^.info.g_right) Shl iface^.info.g_left) And
        iface^.mask_g;
      b := ((s_pixel Shr iface^.info.b_right) Shl iface^.info.b_left) And
        iface^.mask_b;

      Pshort16(dest)^ := r Or g Or b;

      Inc(x, dx);
      Inc(dest, 2);
      Dec(count);
    Until count = 0;

    Inc(dest, iface^.d_add);

    Inc(y, dy);
    Inc(source, (y Shr 16)*DWord(iface^.s_pitch));
    y := y And $ffff;

    Dec(iface^.d_height);
  Until iface^.d_height = 0;
End;

Procedure ConvertP_Generic16_Generic8_S(iface : PHermesConverterInterface); CDecl;

Var
  s_pixel, r, g, b : int32;
  count : DWord;
  source, dest : Pchar8;
  dx, dy, x, y : DWord;

Begin
  source := iface^.s_pixels;
  dest := iface^.d_pixels;
  dy := (iface^.s_height Shl 16) Div iface^.d_height;
  dx := (iface^.s_width Shl 16) Div iface^.d_width;
  y := 0;
  Repeat
    count := iface^.d_width;
    x := 0;
    Repeat
      s_pixel := (Pshort16(source)+(x Shr 16))^;

      r := ((s_pixel Shr iface^.info.r_right) Shl iface^.info.r_left) And
        iface^.mask_r;
      g := ((s_pixel Shr iface^.info.g_right) Shl iface^.info.g_left) And
        iface^.mask_g;
      b := ((s_pixel Shr iface^.info.b_right) Shl iface^.info.b_left) And
        iface^.mask_b;

      dest^ := r Or g Or b;

      Inc(x, dx);
      Inc(dest);
      Dec(count);
    Until count = 0;

    Inc(dest, iface^.d_add);

    Inc(y, dy);
    Inc(source, (y Shr 16)*DWord(iface^.s_pitch));
    y := y And $ffff;

    Dec(iface^.d_height);
  Until iface^.d_height = 0;
End;

Procedure ConvertP_Generic8_Generic32_S(iface : PHermesConverterInterface); CDecl;

Begin
  {todo}
End;

Procedure ConvertP_Generic8_Generic24_S(iface : PHermesConverterInterface); CDecl;

Begin
  {todo}
End;

Procedure ConvertP_Generic8_Generic16_S(iface : PHermesConverterInterface); CDecl;

Begin
  {todo}
End;

Procedure ConvertP_Generic8_Generic8_S(iface : PHermesConverterInterface); CDecl;

Begin
  {todo}
End;
