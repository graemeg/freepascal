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
   32 bit to * converters for the HERMES library
   Copyright (c) 1998 Christian Nentwich (c.nentwich@cs.ucl.ac.uk)
   This source code is licensed under the GNU LGPL

   Please refer to the file COPYING.LIB contained in the distribution for
   licensing conditions
}

{ -------------------------------------------------------------------------

                             NORMAL CONVERTERS

  ------------------------------------------------------------------------- }


{ TO 32 BGR }
Procedure ConvertP_32rgb888_32bgr888(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  s_pixel : int32;
  s_ptr : Pchar8;
  tmp : char8;

Begin
  s_ptr := @s_pixel;
  Repeat
    s_pixel := Pint32(source)^;

    tmp := (s_ptr + R_32)^;
    (s_ptr + R_32)^ := (s_ptr + B_32)^;
    (s_ptr + B_32)^ := tmp;

    Pint32(dest)^ := s_pixel;

    Inc(dest, 4);
    Inc(source, 4);
    Dec(count);
  Until count = 0;
End;

{ 32 RGBA }
Procedure ConvertP_32rgb888_32rgba888(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Begin
  Repeat
    Pint32(dest)^ := (Pint32(source)^ Shl 8) Or $ff;

    Inc(dest, 4);
    Inc(source, 4);
    Dec(count);
  Until count = 0;
End;

{ 32 BGRA }
Procedure ConvertP_32rgb888_32bgra888(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  s_pixel : int32;
  s_ptr : Pchar8;
  tmp : char8;

Begin
  s_ptr := @s_pixel;
  Repeat
    s_pixel := Pint32(source)^;

    tmp := (s_ptr + R_32)^;
    (s_ptr + R_32)^ := (s_ptr + B_32)^;
    (s_ptr + B_32)^ := tmp;

    Pint32(dest)^ := (s_pixel Shl 8) Or $ff;

    Inc(dest, 4);
    Inc(source, 4);
    Dec(count);
  Until count = 0;
End;

{ TO 24 RGB }
Procedure ConvertP_32rgb888_24rgb888(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  s_pixel, s_pixel2 : int32;
  s_point : Pchar8;
  c : DWord;

Begin
  s_point := @s_pixel;

  { Align mod 4 (quite important in this case.. ) }

  While (PtrUInt(dest) And $3) <> 0 Do
  Begin
    s_pixel := Pint32(source)^;

    (dest + R_24)^ := (s_point + R_32)^;
    (dest + G_24)^ := (s_point + G_32)^;
    (dest + B_24)^ := (s_point + B_32)^;

    Inc(source, 4);
    Inc(dest, 3);

    Dec(count);
    If count = 0 Then
      Exit;
  End;

  { Main loop. TODO: Big endian check! }

  c := count Shr 2;
  While c <> 0 Do
  Begin
    Dec(c);
    s_pixel := Pint32(source)^;
    s_pixel2 := (Pint32(source) + 1)^;

    s_pixel := (s_pixel And $ffffff) Or (s_pixel2 Shl 24);
    Pint32(dest)^ := s_pixel;

    s_pixel := (Pint32(source) + 2)^;
    s_pixel2 := ((s_pixel2 Shr 8) And $ffff) Or (s_pixel Shl 16);
    (Pint32(dest) + 1)^ := s_pixel2;

    s_pixel2 := (Pint32(source) + 3)^;
    s_pixel := ((s_pixel Shr 16) And $ff) Or (s_pixel2 Shl 8);
    (Pint32(dest) + 2)^ := s_pixel;

    Inc(source, 16);
    Inc(dest, 12);
  End;

  { Convert trailing pixels }

  count := count And $3;
  While count <> 0 Do
  Begin
    Dec(count);
    s_pixel := Pint32(source)^;

    (dest + R_24)^ := (s_point + R_32)^;
    (dest + G_24)^ := (s_point + G_32)^;
    (dest + B_24)^ := (s_point + B_32)^;

    Inc(source, 4);
    Inc(dest, 3);
  End;
End;

{ TO 24 BGR }
Procedure ConvertP_32rgb888_24bgr888(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  s_pixel : int32;
  s_point : Pchar8;

Begin
  s_point := @s_pixel;
  Repeat
    s_pixel := Pint32(source)^;

    { Note that R and B are swapped }
    (dest + B_24)^ := (s_point + R_32)^;
    (dest + G_24)^ := (s_point + G_32)^;
    (dest + R_24)^ := (s_point + B_32)^;

    Inc(source, 4);
    Inc(dest, 3);
    Dec(count);
  Until count = 0;
End;

{ TO 16 RGB 565 }
Procedure ConvertP_32rgb888_16rgb565(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  i : DWord;
  s_pixel : int32;
  source_32, dest_32 : Pint32;

Begin
  dest_32 := Pint32(dest);
  source_32 := Pint32(source);

  { If the current pixel isn't dword aligned, try write one pixel first }
  If (PtrUInt(dest_32) And $3) <> 0 Then
  Begin
    s_pixel := ((source_32^ Shr 8) And $f800) Or
               ((source_32^ Shr 5) And $7e0) Or
               ((source_32^ Shr 3) And $1f);

    Pshort16(dest_32)^ := s_pixel;

    dest_32 := Pint32(dest + 2);
    Inc(source_32);

    Dec(count);
  End;

  { Write blocks of two pixels }

  For i := 1 To count Shr 1 Do
  Begin
    {This horrible construct is actually faster than loading into a variable}
    dest_32^ := ((source_32^ Shr 8) And $f800) Or
                ((source_32^ Shr 5) And $7e0) Or
                ((source_32^ Shr 3) And $1f) Or
                (((source_32 + 1)^ Shl 8) And $f8000000) Or
                (((source_32 + 1)^ Shl 11) And $7e00000) Or
                (((source_32 + 1)^ Shl 13) And $1f0000);

    Inc(dest_32);
    Inc(source_32, 2);
  End;

  { Eventually, write a single odd pixel that might be left }
  If (count And 1) <> 0 Then
  Begin
    s_pixel := source_32^;
    Pshort16(dest_32)^ := ((s_pixel Shr 8) And $f800) Or
                          ((s_pixel Shr 5) And $7e0) Or
                          ((s_pixel Shr 3) And $1f);
  End;
End;

{ TO 16 BGR 565 }
Procedure ConvertP_32rgb888_16bgr565(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  i : DWord;
  r, g, b : int32;
  s_pixel, d_pixelblock : int32;
  d_pixel : short16;

Begin
  { If the current pixel isn't dword aligned, try write one pixel first }
  If (PtrUInt(dest) And $3) <> 0 Then
  Begin
    s_pixel := Pint32(source)^;

    r := (s_pixel Shr 19) And $1f;
    g := (s_pixel Shr 5) And $7e0;
    b := (s_pixel Shl 8) And $f800;

    d_pixel := r Or g Or b;

    Pshort16(dest)^ := d_pixel;

    Inc(source, 4);
    Inc(dest, 2);
    Dec(count);
  End;

  { Write blocks of two pixels }
  For i := 1 To count Shr 1 Do
  Begin
    s_pixel := Pint32(source)^;

    d_pixelblock := ((s_pixel Shr 19) And $1f) Or
                    ((s_pixel Shr 5) And $7e0) Or
                    ((s_pixel Shl 8) And $f800);

    s_pixel := (Pint32(source) + 1)^;

    d_pixelblock := d_pixelblock Or
                  ((((s_pixel Shr 19) And $1f) Or
                    ((s_pixel Shr 5) And $7e0) Or
                    ((s_pixel Shl 8) And $f800)) Shl 16);

    Pint32(dest)^ := d_pixelblock;
    Inc(source, 8);
    Inc(dest, 4);
  End;

  { Eventually, write a single odd pixel that might be left }
  If (count And 1) <> 0 Then
  Begin
    s_pixel := Pint32(source)^;

    r := (s_pixel Shr 19) And $1f;
    g := (s_pixel Shr 5) And $7e0;
    b := (s_pixel Shl 8) And $f800;

    d_pixel := r Or g Or b;

    Pshort16(dest)^ := d_pixel;
  End;
End;

{ TO 16 RGB 555 }
Procedure ConvertP_32rgb888_16rgb555(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  r, g, b : int32;
  s_pixel, d_pixelblock : int32;
  d_pixel : short16;
  i : DWord;

Begin
  If (PtrUInt(dest) And $3) <> 0 Then
  Begin
    s_pixel := Pint32(source)^;

    r := (s_pixel Shr 9) And $7c00;
    g := (s_pixel Shr 6) And $3e0;
    b := (s_pixel Shr 3) And $1f;

    d_pixel := r Or g Or b;

    Pshort16(dest)^ := d_pixel;

    Inc(source, 4);
    Inc(dest, 2);
    Dec(count);
  End;

  For i := 1 To count Shr 1 Do
  Begin
    s_pixel := Pint32(source)^;

    d_pixelblock := ((s_pixel Shr 9) And $7c00) Or
                    ((s_pixel Shr 6) And $3e0) Or
                    ((s_pixel Shr 3) And $1f);

    s_pixel := (Pint32(source) + 1)^;

    d_pixelblock := d_pixelblock Or
                  ((((s_pixel Shr 9) And $7c00) Or
                    ((s_pixel Shr 6) And $3e0) Or
                    ((s_pixel Shr 3) And $1f)) Shl 16);

    Pint32(dest)^ := d_pixelblock;
    Inc(source, 8);
    Inc(dest, 4);
  End;

  If (count And 1) <> 0 Then
  Begin
    s_pixel := Pint32(source)^;

    r := (s_pixel Shr 9) And $7c00;
    g := (s_pixel Shr 6) And $3e0;
    b := (s_pixel Shr 3) And $1f;

    d_pixel := r Or g Or b;

    Pshort16(dest)^ := d_pixel;
  End;
End;

{ TO 16 BGR 555 }
Procedure ConvertP_32rgb888_16bgr555(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  r, g, b : int32;
  s_pixel, d_pixelblock : int32;
  d_pixel : short16;
  i : DWord;

Begin
  If (PtrUInt(dest) And $3) <> 0 Then
  Begin
    s_pixel := Pint32(source)^;

    r := (s_pixel Shr 19) And $1f;
    g := (s_pixel Shr 6) And $3e0;
    b := (s_pixel Shl 7) And $7c00;

    d_pixel := r Or g Or b;

    Pshort16(dest)^ := d_pixel;

    Inc(source, 4);
    Inc(dest, 2);
    Dec(count);
  End;

  For i := 1 To count Shr 1 Do
  Begin
    s_pixel := Pint32(source)^;

    d_pixelblock := ((s_pixel Shr 19) And $1f) Or
                    ((s_pixel Shr 6) And $3e0) Or
                    ((s_pixel Shl 7) And $7c00);

    s_pixel := (Pint32(source) + 1)^;

    d_pixelblock := d_pixelblock Or
                  ((((s_pixel Shr 19) And $1f) Or
                    ((s_pixel Shr 6) And $3e0) Or
                    ((s_pixel Shl 7) And $7c00)) Shl 16);

    Pint32(dest)^ := d_pixelblock;
    Inc(source, 8);
    Inc(dest, 4);
  End;

  If (count And 1) <> 0 Then
  Begin
    s_pixel := Pint32(source)^;

    r := (s_pixel Shr 19) And $1f;
    g := (s_pixel Shr 6) And $3e0;
    b := (s_pixel Shl 7) And $7c00;

    d_pixel := r Or g Or b;

    Pshort16(dest)^ := d_pixel;
  End;
End;

{ TO 8 RGB 332 }
Procedure ConvertP_32rgb888_8rgb332(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  i : DWord;
  s_pixel, d_block : int32;
  d_pixel : char8;

Begin
  { Process single pixels until we are dword aligned }
  While (PtrUInt(dest) And $3) <> 0 Do
  Begin
    s_pixel := Pint32(source)^;

    d_pixel := ((s_pixel Shr 16) And $e0) Or
               ((s_pixel Shr 11) And $1c) Or
               ((s_pixel Shr 6) And $3);

    dest^ := d_pixel;

    Dec(count);
    If count = 0 Then
      Exit;
    Inc(dest);
    Inc(source, 4);
  End;

  { Now process blocks of four pixels }
  For i := 1 To count Shr 2 Do
  Begin
    s_pixel := Pint32(source)^;
    d_block := ((s_pixel Shr 16) And $e0) Or
               ((s_pixel Shr 11) And $1c) Or
               ((s_pixel Shr 6) And $3);

    s_pixel := (Pint32(source) + 1)^;
    d_block := ((((s_pixel Shr 16) And $e0) Or
                 ((s_pixel Shr 11) And $1c) Or
                 ((s_pixel Shr 6) And $3)) Shl 8) Or d_block;

    s_pixel := (Pint32(source) + 2)^;
    d_block := ((((s_pixel Shr 16) And $e0) Or
                 ((s_pixel Shr 11) And $1c) Or
                 ((s_pixel Shr 6) And $3)) Shl 16) Or d_block;

    s_pixel := (Pint32(source) + 3)^;
    d_block := ((((s_pixel Shr 16) And $e0) Or
                 ((s_pixel Shr 11) And $1c) Or
                 ((s_pixel Shr 6) And $3)) Shl 24) Or d_block;

    Pint32(dest)^ := d_block;
    Inc(source, 16);
    Inc(dest, 4);
  End;

  { Write all possibly remaining pixel }
  count := count And $3;
  While count <> 0 Do
  Begin
    Dec(count);
    s_pixel := Pint32(source)^;

    dest^ := ((s_pixel Shr 16) And $e0) Or
             ((s_pixel Shr 11) And $1c) Or
             ((s_pixel Shr 6) And $3);

    Inc(dest);
    Inc(source, 4);
  End;
End;

{ -------------------------------------------------------------------------

                             STRETCH CONVERTERS

  ------------------------------------------------------------------------- }

Procedure ConvertP_32rgb888_32bgr888_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  x : DWord;
  s_pixel : int32;
  s_ptr : Pchar8;
  tmp : char8;

Begin
  x := 0;
  s_ptr := @s_pixel;
  Repeat
    s_pixel := (Pint32(source)+(x Shr 16))^;

    tmp := (s_ptr + R_32)^;
    (s_ptr + R_32)^ := (s_ptr + B_32)^;
    (s_ptr + B_32)^ := tmp;

    Pint32(dest)^ := s_pixel;

    Inc(dest, 4);
    Inc(x, inc_source);
    Dec(count);
  Until count = 0;
End;

Procedure ConvertP_32rgb888_32rgba888_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  x : DWord;

Begin
  x := 0;
  Repeat
    Pint32(dest)^ := ((Pint32(source) + (x Shr 16))^ Shl 8) Or $ff;

    Inc(dest, 4);
    Inc(x, inc_source);
    Dec(count);
  Until count = 0;
End;

Procedure ConvertP_32rgb888_32bgra888_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  x : DWord;
  s_pixel : int32;
  s_ptr : Pchar8;
  tmp : char8;

Begin
  x := 0;
  s_ptr := @s_pixel;
  Repeat
    s_pixel := (Pint32(source)+(x Shr 16))^;

    tmp := (s_ptr + R_32)^;
    (s_ptr + R_32)^ := (s_ptr + B_32)^;
    (s_ptr + B_32)^ := tmp;

    Pint32(dest)^ := (s_pixel Shl 8) Or $ff;

    Inc(dest, 4);
    Inc(x, inc_source);
    Dec(count);
  Until count = 0;
End;

Procedure ConvertP_32rgb888_24rgb888_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  x, c : DWord;
  s1, s2 : DWord;

Begin
  x := 0;
  While (PtrUInt(dest) And 3) <> 0 Do
  Begin
    Pshort16(dest)^ := Pshort16(source)^;
    (dest + 2)^ := (source + 2)^;
    
    Inc(x, inc_source);
    Inc(source, (x Shr 16)*4);
    x := x And $FFFF;
    
    Inc(dest, 3);
    Dec(count);
    If count = 0 Then
      Exit;
  End;
  
  c := count Shr 2;
  While c > 0 Do
  Begin
    s1 := (Pint32(source) + ((x + inc_source) Shr 16))^ And $FFFFFF;
    Pint32(dest)^ := ((Pint32(source) + (x Shr 16))^ And $FFFFFF) Or (s1 Shl 24);
    s2 := (Pint32(source) + ((x + 2*inc_source) Shr 16))^ And $FFFFFF;
    Pint32(dest + 4)^ := (s1 Shr 8) Or (s2 Shl 16);
    Pint32(dest + 8)^ := (s2 Shr 16) Or ((Pint32(source) + ((x + 3*inc_source) Shr 16))^ Shl 8);
    Inc(x, 4*inc_source);
    
    Inc(dest, 12);
    Dec(c);
  End;
  Inc(source, (x Shr 16) * 4);
  x := x And $FFFF;
  
  count := count And $3;
  While count > 0 Do
  Begin
    Pshort16(dest)^ := Pshort16(source)^;
    (dest + 2)^ := (source + 2)^;
    
    Inc(x, inc_source);
    Inc(source, (x Shr 16)*4);
    x := x And $FFFF;
    
    Inc(dest, 3);
    Dec(count);
  End;
End;

Procedure ConvertP_32rgb888_24bgr888_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  x, c : DWord;
  s1, s2, s3, s4 : DWord;

Begin
  x := 0;
  While (PtrUInt(dest) And 3) <> 0 Do
  Begin
    (dest)^ := (source + 2)^;
    (dest + 1)^ := (source + 1)^;
    (dest + 2)^ := (source)^;
    
    Inc(x, inc_source);
    Inc(source, (x Shr 16)*4);
    x := x And $FFFF;
    
    Inc(dest, 3);
    Dec(count);
    If count = 0 Then
      Exit;
  End;
  
  c := count Shr 2;
  While c > 0 Do
  Begin
    s1 := (Pint32(source) + (x Shr 16))^;
    s2 := (Pint32(source) + ((x + inc_source) Shr 16))^;
    s3 := (Pint32(source) + ((x + 2*inc_source) Shr 16))^;
    s4 := (Pint32(source) + ((x + 3*inc_source) Shr 16))^;
    
    Pint32(dest + 0)^ := ((s2 And $FF0000) Shl 8) Or ((s1 And $FF) Shl 16) Or (s1 And $FF00) Or ((s1 Shr 16) And $FF);
    Pint32(dest + 4)^ := ((s3 And $FF00) Shl 16) Or (s3 And $FF0000) Or ((s2 And $FF) Shl 8) Or ((s2 Shr 8) And $FF);
    Pint32(dest + 8)^ := ((s4 And $FF) Shl 24) Or ((s4 And $FF00) Shl 8) Or ((s4 Shr 8) And $FF00) Or (s3 And $FF);
    
    Inc(x, 4*inc_source);
    
    Inc(dest, 12);
    Dec(c);
  End;
  Inc(source, (x Shr 16) * 4);
  x := x And $FFFF;
  
  count := count And $3;
  While count > 0 Do
  Begin
    (dest)^ := (source + 2)^;
    (dest + 1)^ := (source + 1)^;
    (dest + 2)^ := (source)^;
    
    Inc(x, inc_source);
    Inc(source, (x Shr 16)*4);
    x := x And $FFFF;
    
    Inc(dest, 3);
    Dec(count);
  End;
End;

Procedure ConvertP_32rgb888_16rgb565_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  x, c : DWord;
  p : int32;

Begin
  x := 0;
  { Align mod 4 }
  If (PtrUInt(dest) And 3) <> 0 Then
  Begin
    Pshort16(dest)^ := (((Pint32(source) + (x Shr 16))^ Shr 8) And $f800) Or
                       (((Pint32(source) + (x Shr 16))^ Shr 5) And $7e0) Or
                       (((Pint32(source) + (x Shr 16))^ Shr 3) And $1f);
    Inc(x, inc_source);
    Inc(dest, 2);
    Dec(count);
  End;

  { Try to write 2 pixel blocks }
  c := count Shr 1;
  While c <> 0 Do
  Begin
    Dec(c);
    p := (((Pint32(source) + (x Shr 16))^ Shr 8) And $f800) Or
         (((Pint32(source) + (x Shr 16))^ Shr 5) And $7e0) Or
         (((Pint32(source) + (x Shr 16))^ Shr 3) And $1f);
    Inc(x, inc_source);

    p := p Or
       (((((Pint32(source) + (x Shr 16))^ Shr 8) And $f800) Or
         (((Pint32(source) + (x Shr 16))^ Shr 5) And $7e0) Or
         (((Pint32(source) + (x Shr 16))^ Shr 3) And $1f)) Shl 16);
    Inc(x, inc_source);

    Pint32(dest)^ := p;

    Inc(dest, 4);
  End;

  { Write trailing pixel }
  If (count And 1) <> 0 Then
    Pshort16(dest)^ := (((Pint32(source) + (x Shr 16))^ Shr 8) And $f800) Or
                       (((Pint32(source) + (x Shr 16))^ Shr 5) And $7e0) Or
                       (((Pint32(source) + (x Shr 16))^ Shr 3) And $1f);
End;

Procedure ConvertP_32rgb888_16bgr565_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  x, c : DWord;
  p : int32;

Begin
  x := 0;
  { Align mod 4 }
  If (PtrUInt(dest) And 3) <> 0 Then
  Begin
    Pshort16(dest)^ := (((Pint32(source) + (x Shr 16))^ Shr 19) And $1f) Or
                       (((Pint32(source) + (x Shr 16))^ Shr 5) And $7e0) Or
                       (((Pint32(source) + (x Shr 16))^ Shl 8) And $f800);
    Inc(x, inc_source);
    Inc(dest, 2);
    Dec(count);
  End;

  { Try to write 2 pixel blocks }
  c := count Shr 1;
  While c <> 0 Do
  Begin
    Dec(c);
    p := (((Pint32(source) + (x Shr 16))^ Shr 19) And $1f) Or
         (((Pint32(source) + (x Shr 16))^ Shr 5) And $7e0) Or
         (((Pint32(source) + (x Shr 16))^ Shl 8) And $f800);
    Inc(x, inc_source);

    p := p Or
       (((((Pint32(source) + (x Shr 16))^ Shr 19) And $1f) Or
         (((Pint32(source) + (x Shr 16))^ Shr 5) And $7e0) Or
         (((Pint32(source) + (x Shr 16))^ Shl 8) And $f800)) Shl 16);
    Inc(x, inc_source);

    Pint32(dest)^ := p;

    Inc(dest, 4);
  End;

  { Write trailing pixel }
  If (count And 1) <> 0 Then
    Pshort16(dest)^ := (((Pint32(source) + (x Shr 16))^ Shr 19) And $1f) Or
                       (((Pint32(source) + (x Shr 16))^ Shr 5) And $7e0) Or
                       (((Pint32(source) + (x Shr 16))^ Shl 8) And $f800);
End;

Procedure ConvertP_32rgb888_16rgb555_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  x, c : DWord;
  p : int32;

Begin
  x := 0;
  { Align mod 4 }
  If (PtrUInt(dest) And 3) <> 0 Then
  Begin
    Pshort16(dest)^ := (((Pint32(source) + (x Shr 16))^ Shr 9) And $7c00) Or
                       (((Pint32(source) + (x Shr 16))^ Shr 6) And $3e0) Or
                       (((Pint32(source) + (x Shr 16))^ Shr 3) And $1f);
    Inc(x, inc_source);
    Inc(dest, 2);
    Dec(count);
  End;

  { Try to write 2 pixel blocks }
  c := count Shr 1;
  While c <> 0 Do
  Begin
    Dec(c);
    p := (((Pint32(source) + (x Shr 16))^ Shr 9) And $7c00) Or
         (((Pint32(source) + (x Shr 16))^ Shr 6) And $3e0) Or
         (((Pint32(source) + (x Shr 16))^ Shr 3) And $1f);
    Inc(x, inc_source);

    p := p Or
       (((((Pint32(source) + (x Shr 16))^ Shr 9) And $7c00) Or
         (((Pint32(source) + (x Shr 16))^ Shr 6) And $3e0) Or
         (((Pint32(source) + (x Shr 16))^ Shr 3) And $1f)) Shl 16);
    Inc(x, inc_source);

    Pint32(dest)^ := p;

    Inc(dest, 4);
  End;

  { Write trailing pixel }
  If (count And 1) <> 0 Then
    Pshort16(dest)^ := (((Pint32(source) + (x Shr 16))^ Shr 9) And $7c00) Or
                       (((Pint32(source) + (x Shr 16))^ Shr 6) And $3e0) Or
                       (((Pint32(source) + (x Shr 16))^ Shr 3) And $1f);
End;

Procedure ConvertP_32rgb888_16bgr555_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  x, c : DWord;
  p : int32;

Begin
  x := 0;
  { Align mod 4 }
  If (PtrUInt(dest) And 3) <> 0 Then
  Begin
    Pshort16(dest)^ := (((Pint32(source) + (x Shr 16))^ Shr 19) And $1f) Or
                       (((Pint32(source) + (x Shr 16))^ Shr 6) And $3e0) Or
                       (((Pint32(source) + (x Shr 16))^ Shl 7) And $7c00);
    Inc(x, inc_source);
    Inc(dest, 2);
    Dec(count);
  End;

  { Try to write 2 pixel blocks }
  c := count Shr 1;
  While c <> 0 Do
  Begin
    Dec(c);
    p := (((Pint32(source) + (x Shr 16))^ Shr 19) And $1f) Or
         (((Pint32(source) + (x Shr 16))^ Shr 6) And $3e0) Or
         (((Pint32(source) + (x Shr 16))^ Shl 7) And $7c00);
    Inc(x, inc_source);

    p := p Or
       (((((Pint32(source) + (x Shr 16))^ Shr 19) And $1f) Or
         (((Pint32(source) + (x Shr 16))^ Shr 6) And $3e0) Or
         (((Pint32(source) + (x Shr 16))^ Shl 7) And $7c00)) Shl 16);
    Inc(x, inc_source);

    Pint32(dest)^ := p;

    Inc(dest, 4);
  End;

  { Write trailing pixel }
  If (count And 1) <> 0 Then
    Pshort16(dest)^ := (((Pint32(source) + (x Shr 16))^ Shr 19) And $1f) Or
                       (((Pint32(source) + (x Shr 16))^ Shr 6) And $3e0) Or
                       (((Pint32(source) + (x Shr 16))^ Shl 7) And $7c00);
End;

Procedure ConvertP_32rgb888_8rgb332_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  x, c : DWord;
  p : int32;

Begin
  x := 0;
  { Write single pixels until the destination address is aligned mod 4 }
  While (PtrUInt(dest) And $3) <> 0 Do
  Begin
    dest^ := (((Pint32(source) + (x Shr 16))^ Shr 16) And $e0) Or
             (((Pint32(source) + (x Shr 16))^ Shr 11) And $1c) Or
             (((Pint32(source) + (x Shr 16))^ Shr 6) And $3);
    Inc(x, inc_source);
    Inc(dest);
    Dec(count);
    If count = 0 Then
      Exit;
  End;

  { Write blocks of four pixels now }
  c := count Shr 2;
  While c <> 0 Do
  Begin
    Dec(c);
    p := (((Pint32(source) + (x Shr 16))^ Shr 16) And $e0) Or
         (((Pint32(source) + (x Shr 16))^ Shr 11) And $1c) Or
         (((Pint32(source) + (x Shr 16))^ Shr 6) And $3);
    Inc(x, inc_source);

    p := p Or
       (((((Pint32(source) + (x Shr 16))^ Shr 16) And $e0) Or
         (((Pint32(source) + (x Shr 16))^ Shr 11) And $1c) Or
         (((Pint32(source) + (x Shr 16))^ Shr 6) And $3)) Shl 8);
    Inc(x, inc_source);

    p := p Or
       (((((Pint32(source) + (x Shr 16))^ Shr 16) And $e0) Or
         (((Pint32(source) + (x Shr 16))^ Shr 11) And $1c) Or
         (((Pint32(source) + (x Shr 16))^ Shr 6) And $3)) Shl 16);
    Inc(x, inc_source);

    p := p Or
       (((((Pint32(source) + (x Shr 16))^ Shr 16) And $e0) Or
         (((Pint32(source) + (x Shr 16))^ Shr 11) And $1c) Or
         (((Pint32(source) + (x Shr 16))^ Shr 6) And $3)) Shl 24);
    Inc(x, inc_source);

    Pint32(dest)^ := p;
    Inc(dest, 4);
  End;

  { Write up to three trailing pixels }
  c := count And $3;
  While c <> 0 Do
  Begin
    Dec(c);
    dest^ := (((Pint32(source) + (x Shr 16))^ Shr 16) And $e0) Or
             (((Pint32(source) + (x Shr 16))^ Shr 11) And $1c) Or
             (((Pint32(source) + (x Shr 16))^ Shr 6) And $3);
    Inc(x, inc_source);
    Inc(dest);
  End;
End;
