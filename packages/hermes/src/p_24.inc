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
   Generic C converter (from 24 bit) for the HERMES library
   Copyright (c) 1998 Christian Nentwich (c.nentwich@cs.ucl.ac.uk)
   This source code is licensed under the GNU LGPL

   Please refer to the file COPYING.LIB contained in the distribution for
   licensing conditions
}

{ -------------------------------------------------------------------------

                             NORMAL CONVERTERS

  ------------------------------------------------------------------------- }
{ FROM 24 BIT ROUTINES }
Procedure ConvertP_24rgb888_32rgb888(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  d_block : int32;

Begin
  Repeat
    d_block := ((source + R_24)^ Shl 16) Or
               ((source + G_24)^ Shl 8) Or
               (source + B_24)^;
    Pint32(dest)^ := d_block;
    Inc(dest, 4);
    Inc(source, 3);
    Dec(count);
  Until count = 0;
End;

Procedure ConvertP_24rgb888_32bgr888(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  d_block : int32;

Begin
  Repeat
    d_block := (source + R_24)^ Or
               ((source + G_24)^ Shl 8) Or
               ((source + B_24)^ Shl 16);
    Pint32(dest)^ := d_block;
    Inc(dest, 4);
    Inc(source, 3);
    Dec(count);
  Until count = 0;
End;

Procedure ConvertP_24rgb888_32rgba888(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  d_block : int32;

Begin
  Repeat
    d_block := ((source + R_24)^ Shl 24) Or
               ((source + G_24)^ Shl 16) Or
               ((source + B_24)^ Shl 8) Or $ff;
    Pint32(dest)^ := d_block;
    Inc(dest, 4);
    Inc(source, 3);
    Dec(count);
  Until count = 0;
End;

Procedure ConvertP_24rgb888_32bgra888(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  d_block : int32;

Begin
  Repeat
    d_block := ((source + R_24)^ Shl 8) Or
               ((source + G_24)^ Shl 16) Or
               ((source + B_24)^ Shl 24) Or $ff;
    Pint32(dest)^ := d_block;
    Inc(dest, 4);
    Inc(source, 3);
    Dec(count);
  Until count = 0;
End;

Procedure ConvertP_24rgb888_24bgr888(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  d_pix1, d_pix2, d_pix3 : int32;
  c_pix1, c_pix2, c_pix3 : Pchar8;
  c : DWord;
  tmp : char8;

Begin
  While ((PtrUInt(dest) And 3) <> 0) And (count > 0) Do
  Begin
    (dest + 0)^ := (source + 2)^;
    (dest + 1)^ := (source + 1)^;
    (dest + 2)^ := (source + 0)^;
    Dec(count);
    Inc(dest, 3);
    Inc(source, 3);
  End;
  
  c_pix1 := @d_pix1;
  c_pix2 := @d_pix2;
  c_pix3 := @d_pix3;

  c := count Shr 2;
  While c > 0 Do
  Begin
    d_pix1 := Pint32(source)^;
    d_pix2 := Pint32(source + 4)^;
    d_pix3 := Pint32(source + 8)^;

    { Swap R and B in all three pixels }
    tmp := (c_pix1 + 0)^; (c_pix1 + 0)^ := (c_pix1 + 2)^; (c_pix1 + 2)^ := tmp;
    tmp := (c_pix1 + 3)^; (c_pix1 + 3)^ := (c_pix2 + 1)^; (c_pix2 + 1)^ := tmp;
    tmp := (c_pix2 + 2)^; (c_pix2 + 2)^ := (c_pix3 + 0)^; (c_pix3 + 0)^ := tmp;
    tmp := (c_pix3 + 1)^; (c_pix3 + 1)^ := (c_pix3 + 3)^; (c_pix3 + 3)^ := tmp;

    Pint32(dest)^ := d_pix1;
    Pint32(dest + 4)^ := d_pix2;
    Pint32(dest + 8)^ := d_pix3;
    Inc(dest, 12);
    Inc(source, 12);
    Dec(c);
  End;

  count := count And $3;
  While count > 0 Do
  Begin
    (dest + 0)^ := (source + 2)^;
    (dest + 1)^ := (source + 1)^;
    (dest + 2)^ := (source + 0)^;
    Dec(count);
    Inc(dest, 3);
    Inc(source, 3);
  End;
End;

Procedure ConvertP_24rgb888_16rgb565(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  d_block : int32;
  i : DWord;

Begin
  For i := 0 To (count Shr 1) - 1 Do
  Begin
    d_block := (((source + R_24)^ Shl 8) And $f800) Or
               (((source + G_24)^ Shl 3) And $7e0) Or
               (((source + B_24)^ Shr 3) And $1f);

    d_block := d_block Or
             (((((source + R_24 + 3)^ Shl 8) And $f800) Or
               (((source + G_24 + 3)^ Shl 3) And $7e0) Or
               (((source + B_24 + 3)^ Shr 3) And $1f)) Shl 16);

    Pint32(dest)^ := d_block;
    Inc(source, 6);
    Inc(dest, 4);
  End;

  If (count And 1) <> 0 Then
  Begin
    d_block := (((source + R_24)^ Shl 8) And $f800) Or
               (((source + G_24)^ Shl 3) And $7e0) Or
               (((source + B_24)^ Shr 3) And $1f);

    Pshort16(dest)^ := d_block;
  End;
End;

Procedure ConvertP_24rgb888_16bgr565(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  d_block : int32;
  i : DWord;

Begin
  For i := 0 To (count Shr 1) - 1 Do
  Begin
    d_block := (((source + R_24)^ Shr 3) And $1f) Or
               (((source + G_24)^ Shl 3) And $7e0) Or
               (((source + B_24)^ Shl 8) And $f800);

    d_block := d_block Or
             (((((source + R_24 + 3)^ Shr 3) And $1f) Or
               (((source + G_24 + 3)^ Shl 3) And $7e0) Or
               (((source + B_24 + 3)^ Shl 8) And $f800)) Shl 16);

    Pint32(dest)^ := d_block;
    Inc(source, 6);
    Inc(dest, 4);
  End;

  If (count And 1) <> 0 Then
  Begin
    d_block := (((source + R_24)^ Shr 3) And $1f) Or
               (((source + G_24)^ Shl 3) And $7e0) Or
               (((source + B_24)^ Shl 8) And $f800);

    Pshort16(dest)^ := d_block;
  End;
End;

Procedure ConvertP_24rgb888_16rgb555(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  d_block : int32;
  i : DWord;

Begin
  For i := 0 To (count Shr 1) - 1 Do
  Begin
    d_block := (((source + R_24)^ Shl 7) And $7c00) Or
               (((source + G_24)^ Shl 2) And $3e0) Or
               (((source + B_24)^ Shr 3) And $1f);

    d_block := d_block Or
             (((((source + R_24 + 3)^ Shl 7) And $7c00) Or
               (((source + G_24 + 3)^ Shl 2) And $3e0) Or
               (((source + B_24 + 3)^ Shr 3) And $1f)) Shl 16);

    Pint32(dest)^ := d_block;
    Inc(source, 6);
    Inc(dest, 4);
  End;

  If (count And 1) <> 0 Then
  Begin
    d_block := (((source + R_24)^ Shl 7) And $7c00) Or
               (((source + G_24)^ Shl 2) And $3e0) Or
               (((source + B_24)^ Shr 3) And $1f);

    Pshort16(dest)^ := d_block;
  End;
End;

Procedure ConvertP_24rgb888_16bgr555(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  d_block : int32;
  i : DWord;

Begin
  For i := 0 To (count Shr 1) - 1 Do
  Begin
    d_block := (((source + R_24)^ Shr 3) And $1f) Or
               (((source + G_24)^ Shl 2) And $3e0) Or
               (((source + B_24)^ Shl 7) And $7c00);

    d_block := d_block Or
             (((((source + R_24 + 3)^ Shr 3) And $1f) Or
               (((source + G_24 + 3)^ Shl 2) And $3e0) Or
               (((source + B_24 + 3)^ Shl 7) And $7c00)) Shl 16);

    Pint32(dest)^ := d_block;
    Inc(source, 6);
    Inc(dest, 4);
  End;

  If (count And 1) <> 0 Then
  Begin
    d_block := (((source + R_24)^ Shr 3) And $1f) Or
               (((source + G_24)^ Shl 2) And $3e0) Or
               (((source + B_24)^ Shl 7) And $7c00);

    Pshort16(dest)^ := d_block;
  End;
End;

{ optimise me !! }

Procedure ConvertP_24rgb888_8rgb332(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  r, g, b : int32;
  i : DWord;

Begin
  For i := 0 To count - 1 Do
  Begin
    r := (source + R_24)^ And $e0;
    g := ((source + G_24)^ Shr 3) And $1c;
    b := ((source + B_24)^ Shr 6) And $3;

    dest^ := r Or g Or b;
    Inc(source, 3);
    Inc(dest);
  End;
End;

{ -------------------------------------------------------------------------

                             STRETCH CONVERTERS

  ------------------------------------------------------------------------- }
Procedure ConvertP_24rgb888_32rgb888_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  x : DWord;

Begin
  x := 0;
  Repeat
    Pint32(dest)^ := ((source + R_24)^ Shl 16) Or
                     ((source + G_24)^ Shl 8) Or
                      (source + B_24)^;
    Inc(x, inc_source);
    Inc(source, x Shr 16);
    Inc(source, x Shr 16);
    Inc(source, x Shr 16);
    x := x And $FFFF;
    Inc(dest, 4);
    Dec(count);
  Until count = 0;
End;

Procedure ConvertP_24rgb888_32bgr888_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  x : DWord;

Begin
  x := 0;
  Repeat
    Pint32(dest)^ :=  (source + R_24)^ Or
                     ((source + G_24)^ Shl 8) Or
                     ((source + B_24)^ Shl 16);
    Inc(x, inc_source);
    Inc(source, x Shr 16);
    Inc(source, x Shr 16);
    Inc(source, x Shr 16);
    x := x And $FFFF;
    Inc(dest, 4);
    Dec(count);
  Until count = 0;
End;

Procedure ConvertP_24rgb888_32rgba888_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  x : DWord;

Begin
  x := 0;
  Repeat
    Pint32(dest)^ := ((((source + R_24)^ Shl 16) Or
                       ((source + G_24)^ Shl 8) Or
                        (source + B_24)^) Shl 8) Or $FF;
    Inc(x, inc_source);
    Inc(source, x Shr 16);
    Inc(source, x Shr 16);
    Inc(source, x Shr 16);
    x := x And $FFFF;
    Inc(dest, 4);
    Dec(count);
  Until count = 0;
End;

Procedure ConvertP_24rgb888_32bgra888_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  x : DWord;

Begin
  x := 0;
  Repeat
    Pint32(dest)^ :=  (((source + R_24)^ Or
                       ((source + G_24)^ Shl 8) Or
                       ((source + B_24)^ Shl 16)) Shl 8) Or $FF;
    Inc(x, inc_source);
    Inc(source, x Shr 16);
    Inc(source, x Shr 16);
    Inc(source, x Shr 16);
    x := x And $FFFF;
    Inc(dest, 4);
    Dec(count);
  Until count = 0;
End;

Procedure ConvertP_24rgb888_24bgr888_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  x : int32;

Begin
  x := 0;
  Repeat
    (dest + 0)^ := (source + 2)^;
    (dest + 1)^ := (source + 1)^;
    (dest + 2)^ := (source + 0)^;

    Inc(x, inc_source);
    Inc(source, x Shr 16);
    Inc(source, x Shr 16);
    Inc(source, x Shr 16);
    x := x And $FFFF;
    Inc(dest, 3);
    Dec(count);
  Until count = 0;
End;

Procedure ConvertP_24rgb888_16rgb565_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  d_block : int32;
  x : DWord;
  c : DWord;

Begin
  x := 0;
  If (PtrUInt(dest) And 3) <> 0 Then
  Begin
    d_block := (((source + R_24)^ Shl 8) And $f800) Or
               (((source + G_24)^ Shl 3) And $7e0) Or
               (((source + B_24)^ Shr 3) And $1f);

    Inc(x, inc_source);
    Inc(source, x Shr 16);
    Inc(source, x Shr 16);
    Inc(source, x Shr 16);
    x := x And $FFFF;

    Pshort16(dest)^ := d_block;
    Inc(dest, 2);
  End;
  c := count Shr 1;
  While c > 0 Do
  Begin
    d_block := (((source + R_24)^ Shl 8) And $f800) Or
               (((source + G_24)^ Shl 3) And $7e0) Or
               (((source + B_24)^ Shr 3) And $1f);
    Inc(x, inc_source);
    Inc(source, x Shr 16);
    Inc(source, x Shr 16);
    Inc(source, x Shr 16);
    x := x And $FFFF;

    d_block := d_block Or
             (((((source + R_24)^ Shl 8) And $f800) Or
               (((source + G_24)^ Shl 3) And $7e0) Or
               (((source + B_24)^ Shr 3) And $1f)) Shl 16);
    Inc(x, inc_source);
    Inc(source, x Shr 16);
    Inc(source, x Shr 16);
    Inc(source, x Shr 16);
    x := x And $FFFF;

    Pint32(dest)^ := d_block;
    Inc(dest, 4);
    Dec(c);
  End;

  If (count And 1) <> 0 Then
  Begin
    d_block := (((source + R_24)^ Shl 8) And $f800) Or
               (((source + G_24)^ Shl 3) And $7e0) Or
               (((source + B_24)^ Shr 3) And $1f);

    Pshort16(dest)^ := d_block;
  End;
End;

Procedure ConvertP_24rgb888_16bgr565_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  d_block : int32;
  x : DWord;
  c : DWord;

Begin
  x := 0;
  If (PtrUInt(dest) And 3) <> 0 Then
  Begin
    d_block := (((source + R_24)^ Shr 3) And $1f) Or
               (((source + G_24)^ Shl 3) And $7e0) Or
               (((source + B_24)^ Shl 8) And $f800);

    Inc(x, inc_source);
    Inc(source, x Shr 16);
    Inc(source, x Shr 16);
    Inc(source, x Shr 16);
    x := x And $FFFF;

    Pshort16(dest)^ := d_block;
    Inc(dest, 2);
  End;
  c := count Shr 1;
  While c > 0 Do
  Begin
    d_block := (((source + R_24)^ Shr 3) And $1f) Or
               (((source + G_24)^ Shl 3) And $7e0) Or
               (((source + B_24)^ Shl 8) And $f800);
    Inc(x, inc_source);
    Inc(source, x Shr 16);
    Inc(source, x Shr 16);
    Inc(source, x Shr 16);
    x := x And $FFFF;

    d_block := d_block Or
              (((((source + R_24)^ Shr 3) And $1f) Or
                (((source + G_24)^ Shl 3) And $7e0) Or
                (((source + B_24)^ Shl 8) And $f800)) Shl 16);
    Inc(x, inc_source);
    Inc(source, x Shr 16);
    Inc(source, x Shr 16);
    Inc(source, x Shr 16);
    x := x And $FFFF;

    Pint32(dest)^ := d_block;
    Inc(dest, 4);
    Dec(c);
  End;

  If (count And 1) <> 0 Then
  Begin
    d_block := (((source + R_24)^ Shr 3) And $1f) Or
               (((source + G_24)^ Shl 3) And $7e0) Or
               (((source + B_24)^ Shl 8) And $f800);

    Pshort16(dest)^ := d_block;
  End;
End;

Procedure ConvertP_24rgb888_16rgb555_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  d_block : int32;
  x : DWord;
  c : DWord;

Begin
  x := 0;
  If (PtrUInt(dest) And 3) <> 0 Then
  Begin
    d_block := (((source + R_24)^ Shl 7) And $7c00) Or
               (((source + G_24)^ Shl 2) And $3e0) Or
               (((source + B_24)^ Shr 3) And $1f);

    Inc(x, inc_source);
    Inc(source, x Shr 16);
    Inc(source, x Shr 16);
    Inc(source, x Shr 16);
    x := x And $FFFF;

    Pshort16(dest)^ := d_block;
    Inc(dest, 2);
  End;
  c := count Shr 1;
  While c > 0 Do
  Begin
    d_block := (((source + R_24)^ Shl 7) And $7c00) Or
               (((source + G_24)^ Shl 2) And $3e0) Or
               (((source + B_24)^ Shr 3) And $1f);
    Inc(x, inc_source);
    Inc(source, x Shr 16);
    Inc(source, x Shr 16);
    Inc(source, x Shr 16);
    x := x And $FFFF;

    d_block := d_block Or
               (((((source + R_24)^ Shl 7) And $7c00) Or
                (((source + G_24)^ Shl 2) And $3e0) Or
                (((source + B_24)^ Shr 3) And $1f)) Shl 16);
    Inc(x, inc_source);
    Inc(source, x Shr 16);
    Inc(source, x Shr 16);
    Inc(source, x Shr 16);
    x := x And $FFFF;

    Pint32(dest)^ := d_block;
    Inc(dest, 4);
    Dec(c);
  End;

  If (count And 1) <> 0 Then
  Begin
    d_block := (((source + R_24)^ Shl 7) And $7c00) Or
               (((source + G_24)^ Shl 2) And $3e0) Or
               (((source + B_24)^ Shr 3) And $1f);

    Pshort16(dest)^ := d_block;
  End;
End;

Procedure ConvertP_24rgb888_16bgr555_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  d_block : int32;
  x : DWord;
  c : DWord;

Begin
  x := 0;
  If (PtrUInt(dest) And 3) <> 0 Then
  Begin
    d_block := (((source + R_24)^ Shr 3) And $1f) Or
               (((source + G_24)^ Shl 2) And $3e0) Or
               (((source + B_24)^ Shl 7) And $7c00);

    Inc(x, inc_source);
    Inc(source, x Shr 16);
    Inc(source, x Shr 16);
    Inc(source, x Shr 16);
    x := x And $FFFF;

    Pshort16(dest)^ := d_block;
    Inc(dest, 2);
  End;
  c := count Shr 1;
  While c > 0 Do
  Begin
    d_block := (((source + R_24)^ Shr 3) And $1f) Or
               (((source + G_24)^ Shl 2) And $3e0) Or
               (((source + B_24)^ Shl 7) And $7c00);
    Inc(x, inc_source);
    Inc(source, x Shr 16);
    Inc(source, x Shr 16);
    Inc(source, x Shr 16);
    x := x And $FFFF;

    d_block := d_block Or
             (((((source + R_24)^ Shr 3) And $1f) Or
               (((source + G_24)^ Shl 2) And $3e0) Or
               (((source + B_24)^ Shl 7) And $7c00)) Shl 16);
    Inc(x, inc_source);
    Inc(source, x Shr 16);
    Inc(source, x Shr 16);
    Inc(source, x Shr 16);
    x := x And $FFFF;

    Pint32(dest)^ := d_block;
    Inc(dest, 4);
    Dec(c);
  End;

  If (count And 1) <> 0 Then
  Begin
    d_block := (((source + R_24)^ Shr 3) And $1f) Or
               (((source + G_24)^ Shl 2) And $3e0) Or
               (((source + B_24)^ Shl 7) And $7c00);

    Pshort16(dest)^ := d_block;
  End;
End;

Procedure ConvertP_24rgb888_8rgb332_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  r, g, b : int32;
  i : DWord;
  x : DWord;

Begin
  x := 0;
  For i := 0 To count - 1 Do
  Begin
    r := (source + R_24)^ And $e0;
    g := ((source + G_24)^ Shr 3) And $1c;
    b := ((source + B_24)^ Shr 6) And $3;

    dest^ := r Or g Or b;
    Inc(x, inc_source);
    Inc(source, x Shr 16);
    Inc(source, x Shr 16);
    Inc(source, x Shr 16);
    x := x And $FFFF;
    Inc(dest);
  End;
End;
