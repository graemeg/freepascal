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
   16 bit to * converters for the HERMES library
   Copyright (c) 1998 Christian Nentwich (c.nentwich@cs.ucl.ac.uk)
   This source code is licensed under the GNU LGPL

   Please refer to the file COPYING.LIB contained in the distribution for
   licensing conditions
}

{ TO 32 BIT RGB }
Procedure ConvertP_16rgb565_32rgb888(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  d_pixel : int32;

Begin
  Repeat
    d_pixel := Pshort16(source)^;

    d_pixel := ((d_pixel And $f800) Shl 8) Or ((d_pixel And $7e0) Shl 5) Or
               ((d_pixel And $1f) Shl 3) Or $030103;

    Pint32(dest)^ := d_pixel;

    Inc(source, 2);
    Inc(dest, 4);
    Dec(count);
  Until count = 0;
End;

{ TO 32 BIT BGR }
Procedure ConvertP_16rgb565_32bgr888(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  d_pixel : int32;

Begin
  Repeat
    d_pixel := Pshort16(source)^;

    d_pixel := ((d_pixel And $f800) Shr 8) Or ((d_pixel And $7e0) Shl 5) Or
               ((d_pixel And $1f) Shl 19) Or $030103;

    Pint32(dest)^ := d_pixel;

    Inc(source, 2);
    Inc(dest, 4);
    Dec(count);
  Until count = 0;
End;

{ TO 32 BIT RGBA }
Procedure ConvertP_16rgb565_32rgba888(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  d_pixel : int32;

Begin
  Repeat
    d_pixel := Pshort16(source)^;

    d_pixel := ((d_pixel And $f800) Shl 8) Or ((d_pixel And $7e0) Shl 5) Or
               ((d_pixel And $1f) Shl 3) Or $030103;

    Pint32(dest)^ := (d_pixel Shl 8) Or $ff;

    Inc(source, 2);
    Inc(dest, 4);
    Dec(count);
  Until count = 0;
End;

{ TO 32 BIT BGRA }
Procedure ConvertP_16rgb565_32bgra888(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  d_pixel : int32;

Begin
  Repeat
    d_pixel := Pshort16(source)^;

    d_pixel := ((d_pixel And $f800) Shr 8) Or ((d_pixel And $7e0) Shl 5) Or
               ((d_pixel And $1f) Shl 19) Or $030103;

    Pint32(dest)^ := (d_pixel Shl 8) Or $ff;

    Inc(source, 2);
    Inc(dest, 4);
    Dec(count);
  Until count = 0;
End;

{ TO 24 BIT RGB }
Procedure ConvertP_16rgb565_24rgb888(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  d_pixel : int32;
  d_ptr : Pchar8;

Begin
  d_ptr := @d_pixel;
  Repeat
    d_pixel := Pshort16(source)^;

    d_pixel := ((d_pixel And $f800) Shl 8) Or ((d_pixel And $7e0) Shl 5) Or
               ((d_pixel And $1f) Shl 3) Or $030103;

    (dest + R_24)^ := (d_ptr + R_32)^;
    (dest + G_24)^ := (d_ptr + G_32)^;
    (dest + B_24)^ := (d_ptr + B_32)^;

    Inc(source, 2);
    Inc(dest, 3);
    Dec(count);
  Until count = 0;
End;

{ TO 24 BIT BGR }
Procedure ConvertP_16rgb565_24bgr888(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  d_pixel : int32;
  d_ptr : Pchar8;

Begin
  d_ptr := @d_pixel;
  Repeat
    d_pixel := Pshort16(source)^;

    d_pixel := ((d_pixel And $f800) Shl 8) Or ((d_pixel And $7e0) Shl 5) Or
               ((d_pixel And $1f) Shl 3) Or $030103;

    { Red and blue are swapped here }
    (dest + R_24)^ := (d_ptr + B_32)^;
    (dest + G_24)^ := (d_ptr + G_32)^;
    (dest + B_24)^ := (d_ptr + R_32)^;

    Inc(source, 2);
    Inc(dest, 3);
    Dec(count);
  Until count = 0;
End;

{ TO 16 BIT BGR 565 }
Procedure ConvertP_16rgb565_16bgr565(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  i : DWord;
  s_pixel : int32;

Begin
  { If we are not aligned to a dword, try and convert a single pixel }
  If (PtrUInt(dest) And $3) <> 0 Then
  Begin
    s_pixel := Pshort16(source)^;

    { Swap around R and B, leave G unchanged }
    s_pixel := (s_pixel Shr 11) Or (s_pixel And $7e0) Or
               ((s_pixel Shl 11) And $f800);

    Pshort16(dest)^ := s_pixel;

    Dec(count);
    Inc(dest, 2); Inc(source, 2);
  End;

  { Now copy blocks of dwords }
  For i := 1 To count Shr 1 Do
  Begin
    s_pixel := Pint32(source)^;

    { Leave G unchanged, shift R to the right and B to the left }
    s_pixel := (s_pixel And $07e007e0) Or ((s_pixel And $f800f800) Shr 11) Or
               ((s_pixel And $001f001f) Shl 11);

    Pint32(dest)^ := s_pixel;
    Inc(source, 4); Inc(dest, 4);
  End;

  If (count And 1) <> 0 Then
  Begin
    s_pixel := Pshort16(source)^;

    { Swap around R and B, leave G unchanged }
    s_pixel := (s_pixel Shr 11) Or (s_pixel And $7e0) Or
               ((s_pixel Shl 11) And $f800);

    Pshort16(dest)^ := s_pixel;
  End;
End;

{ TO 16 BIT RGB 555 }
Procedure ConvertP_16rgb565_16rgb555(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  i : DWord;
  s_pixel : int32;

Begin
  { If we are not aligned to a dword, try and convert a single pixel }
  If (PtrUInt(dest) And $3) <> 0 Then
  Begin
    s_pixel := Pshort16(source)^;

    { Leave blue untouched, mask red and shift by one, mask green and shift
      by one }
    s_pixel := (s_pixel And $1f) Or ((s_pixel And $f800) Shr 1) Or
               ((s_pixel And $7c0) Shr 1);

    Pshort16(dest)^ := s_pixel;

    Dec(count);
    Inc(dest, 2); Inc(source, 2);
  End;

  { Now copy blocks of dwords }
  For i := 1 To count Shr 1 Do
  Begin
    s_pixel := Pint32(source)^;

    { Leave blue untouched, mask red and shift by one, mask green and shift
      by one }
    s_pixel := (s_pixel And $001f001f) Or ((s_pixel And $f800f800) Shr 1) Or
               ((s_pixel And $07c007c0) Shr 1);

    Pint32(dest)^ := s_pixel;
    Inc(source, 4); Inc(dest, 4);
  End;

  If (count And 1) <> 0 Then
  Begin
    s_pixel := Pshort16(source)^;

    { Leave blue untouched, mask red and shift by one, mask green and shift
      by one }
    s_pixel := (s_pixel And $1f) Or ((s_pixel And $f800) Shr 1) Or
               ((s_pixel And $7c0) Shr 1);

    Pshort16(dest)^ := s_pixel;
  End;
End;

{ TO 16 BIT BGR 555 }
Procedure ConvertP_16rgb565_16bgr555(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  i : DWord;
  s_pixel : int32;

Begin
  { If we are not aligned to a dword, try and convert a single pixel }
  If (PtrUInt(dest) And $3) <> 0 Then
  Begin
    s_pixel := Pshort16(source)^;

    { Shift red right by 11, mask green and shift right one, shift blue
      left ten }
    s_pixel := ((s_pixel And $f800) Shr 11) Or ((s_pixel And $7c0) Shr 1) Or
               ((s_pixel And $1f) Shl 10);

    Pshort16(dest)^ := s_pixel;

    Dec(count);
    Inc(dest, 2); Inc(source, 2);
  End;

  { Now copy blocks of dwords }
  For i := 1 To count Shr 1 Do
  Begin
    s_pixel := Pint32(source)^;

    { Shift red right by 11, mask green and shift right one, shift blue
      left ten }
    s_pixel := ((s_pixel And $f800f800) Shr 11) Or
               ((s_pixel And $07c007c0) Shr 1) Or
               ((s_pixel And $001f001f) Shl 10);

    Pint32(dest)^ := s_pixel;
    Inc(source, 4); Inc(dest, 4);
  End;

  If (count And 1) <> 0 Then
  Begin
    s_pixel := Pshort16(source)^;

    { Shift red right by 11, mask green and shift right one, shift blue
      left ten }
    s_pixel := ((s_pixel And $f800) Shr 11) Or ((s_pixel And $7c0) Shr 1) Or
               ((s_pixel And $1f) Shl 10);

    Pshort16(dest)^ := s_pixel;
  End;
End;

{ TO 8 BIT RGB 332 }
Procedure ConvertP_16rgb565_8rgb332(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  s_block, d_block : int32;
  i : DWord;

Begin
  { Align to dword first }
  While (PtrUInt(dest) And $3) <> 0 Do
  Begin
    s_block := Pshort16(source)^;

    s_block := ((s_block And $e000) Shr 8) Or ((s_block And $0700) Shr 6) Or
               ((s_block And $18) Shr 3);

    dest^ := s_block;

    Dec(count);
    If count = 0 Then
      Exit;

    Inc(source, 2);
    Inc(dest);
  End;

  { Write blocks of four pixels }
  For i := 1 To count Shr 2 Do
  Begin
    { Read and process first two pixels }
    s_block := Pint32(source)^;

    d_block := ((s_block And $e000e000) Shr 8) Or
               ((s_block And $07000700) Shr 6) Or
               ((s_block And $00180018) Shr 3);
    d_block := (d_block And $ff) Or ((d_block And $ff0000) Shr 8);

    { And the second two }
    s_block := (Pint32(source)+1)^;

    s_block := ((s_block And $e000e000) Shr 8) Or
               ((s_block And $07000700) Shr 6) Or
               ((s_block And $00180018) Shr 3);
    s_block := (s_block And $ff) Or ((s_block And $ff0000) Shr 8);

    { Put it all in one dword and write it }
    d_block := d_block Or (s_block Shl 16);

    Pint32(dest)^ := d_block;
    Inc(source, 8);
    Inc(dest, 4);
  End;

  { Clean up remaining pixels }
  count := count And 3;
  While count > 0 Do
  Begin
    Dec(count);
    s_block := Pshort16(source)^;

    dest^ := ((s_block And $e000) Shr 8) Or ((s_block And $0700) Shr 6) Or
             ((s_block And $18) Shr 3);
    Inc(dest);
    Inc(source, 2);
  End;
End;

{ -------------------------------------------------------------------------

                          STRETCH CONVERTERS

  ------------------------------------------------------------------------- }


Procedure ConvertP_16rgb565_32rgb888_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  x : DWord;
  p : int32;

Begin
  x := 0;
  Repeat
    p := (((Pshort16(source) + (x Shr 16))^ And $f800) Shl 8) Or
         (((Pshort16(source) + (x Shr 16))^ And $7e0) Shl 5) Or
         (((Pshort16(source) + (x Shr 16))^ And $1f) Shl 3) Or $30103;

    Inc(x, inc_source);

    Pint32(dest)^ := p;

    Inc(dest, 4);
    Dec(count);
  Until count = 0;
End;

Procedure ConvertP_16rgb565_32bgr888_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  d_pixel : int32;
  x : DWord;

Begin
  x := 0;

  Repeat
    d_pixel := (Pshort16(source) + (x Shr 16))^;

    d_pixel := ((d_pixel And $f800) Shr 8) Or ((d_pixel And $7e0) Shl 5) Or
               ((d_pixel And $1f) Shl 19) Or $30103;

    Pint32(dest)^ := d_pixel;

    Inc(dest, 4);
    Inc(x, inc_source);
    Dec(count);
  Until count = 0;
End;

Procedure ConvertP_16rgb565_32rgba888_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  x : DWord;
  p : int32;

Begin
  x := 0;
  Repeat
    p := (((Pshort16(source) + (x Shr 16))^ And $f800) Shl (8+8)) Or
         (((Pshort16(source) + (x Shr 16))^ And $7e0) Shl (5+8)) Or
         (((Pshort16(source) + (x Shr 16))^ And $1f) Shl (3+8)) Or $30103ff;

    Inc(x, inc_source);

    Pint32(dest)^ := p;

    Inc(dest, 4);
    Dec(count);
  Until count = 0;
End;

Procedure ConvertP_16rgb565_32bgra888_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  d_pixel : int32;
  x : DWord;

Begin
  x := 0;

  Repeat
    d_pixel := (Pshort16(source) + (x Shr 16))^;

    d_pixel := ((d_pixel And $f800) {Shr 8}) Or ((d_pixel And $7e0) Shl (5+8)) Or
               ((d_pixel And $1f) Shl (19+8)) Or $30103ff;

    Pint32(dest)^ := d_pixel;

    Inc(dest, 4);
    Inc(x, inc_source);
    Dec(count);
  Until count = 0;
End;

Procedure ConvertP_16rgb565_24rgb888_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  x : DWord;
  p1, p2, p3, p4 : DWord;
  c : DWord;

Begin
  x := 0;
  While (PtrUInt(dest) And 3) <> 0 Do
  Begin
    p1 := (((Pshort16(source) + (x Shr 16))^ And $f800) Shl 8) Or
          (((Pshort16(source) + (x Shr 16))^ And $7e0) Shl 5) Or
          (((Pshort16(source) + (x Shr 16))^ And $1f) Shl 3) Or $30103;
    Pshort16(dest)^ := p1 And $FFFF;
    (dest + 2)^ := p1 Shr 16;
    
    Inc(dest, 3);
    Inc(x, inc_source);
    Dec(count);
    If count = 0 Then
      Exit;
  End;
  
  c := count Shr 2;
  While c > 0 Do
  Begin
    p1 := (Pshort16(source) + (x Shr 16))^;
    p2 := (Pshort16(source) + ((x + inc_source) Shr 16))^;
    p3 := (Pshort16(source) + ((x + 2*inc_source) Shr 16))^;
    p4 := (Pshort16(source) + ((x + 3*inc_source) Shr 16))^;

    Pint32(dest + 0)^ := ((p2 And $001F) Shl 27) Or ((p1 And $F800) Shl 8) Or ((p1 And $07E0) Shl 5) Or ((p1 And $001F) Shl 3) Or $03030103;
    Pint32(dest + 4)^ := ((p3 And $07E0) Shl 21) Or ((p3 And $001F) Shl 19) Or (p2 And $F800) Or ((p2 And $07E0) Shr 3) Or $01030301;
    Pint32(dest + 8)^ := ((p4 And $F800) Shl 16) Or ((p4 And $07E0) Shl 13) Or ((p4 And $001F) Shl 11) Or ((p3 And $F800) Shr 8) Or $03010303;
    
    Inc(x, 4*inc_source);
    Inc(dest, 12);
    Dec(c);
  End;
  
  count := count And 3;
  While count > 0 Do
  Begin
    p1 := (((Pshort16(source) + (x Shr 16))^ And $f800) Shl 8) Or
          (((Pshort16(source) + (x Shr 16))^ And $7e0) Shl 5) Or
          (((Pshort16(source) + (x Shr 16))^ And $1f) Shl 3) Or $30103;
    Pshort16(dest)^ := p1 And $FFFF;
    (dest + 2)^ := p1 Shr 16;
    
    Inc(dest, 3);
    Inc(x, inc_source);
    Dec(count);
  End;
End;

Procedure ConvertP_16rgb565_24bgr888_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  x : DWord;
  p1, p2, p3, p4 : DWord;
  c : DWord;

Begin
  x := 0;
  While (PtrUInt(dest) And 3) <> 0 Do
  Begin
    p1 := (((Pshort16(source) + (x Shr 16))^ And $f800) Shr 8) Or
          (((Pshort16(source) + (x Shr 16))^ And $7e0) Shl 5) Or
          (((Pshort16(source) + (x Shr 16))^ And $1f) Shl 19) Or $30103;
    Pshort16(dest)^ := p1 And $FFFF;
    (dest + 2)^ := p1 Shr 16;
    
    Inc(dest, 3);
    Inc(x, inc_source);
    Dec(count);
    If count = 0 Then
      Exit;
  End;
  
  c := count Shr 2;
  While c > 0 Do
  Begin
    p1 := (Pshort16(source) + (x Shr 16))^;
    p2 := (Pshort16(source) + ((x + inc_source) Shr 16))^;
    p3 := (Pshort16(source) + ((x + 2*inc_source) Shr 16))^;
    p4 := (Pshort16(source) + ((x + 3*inc_source) Shr 16))^;

    Pint32(dest + 0)^ := ((p2 And $F800) Shl 16) Or ((p1 And $001F) Shl 19) Or ((p1 And $07E0) Shl 5) Or ((p1 And $F800) Shr 8) Or $03030103;
    Pint32(dest + 4)^ := ((p3 And $07E0) Shl 21) Or ((p3 And $F800) Shl 8) Or ((p2 And $001F) Shl 11) Or ((p2 And $07E0) Shr 3) Or $01030301;
    Pint32(dest + 8)^ := ((p4 And $001F) Shl 27) Or ((p4 And $07E0) Shl 13) Or (p4 And $F800) Or ((p3 And $001F) Shl 3) Or $03010303;
    
    Inc(x, 4*inc_source);
    Inc(dest, 12);
    Dec(c);
  End;
  
  count := count And 3;
  While count > 0 Do
  Begin
    p1 := (((Pshort16(source) + (x Shr 16))^ And $f800) Shr 8) Or
          (((Pshort16(source) + (x Shr 16))^ And $7e0) Shl 5) Or
          (((Pshort16(source) + (x Shr 16))^ And $1f) Shl 19) Or $30103;
    Pshort16(dest)^ := p1 And $FFFF;
    (dest + 2)^ := p1 Shr 16;
    
    Inc(dest, 3);
    Inc(x, inc_source);
    Dec(count);
  End;
End;

Procedure ConvertP_16rgb565_16bgr565_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  x, c : DWord;
  p : int32;

Begin
  x := 0;
  { If we are not aligned to a dword, try and convert a single pixel }
  If (PtrUInt(dest) And $3) <> 0 Then
  Begin
    { Swap r and b, leave g untouched }
    Pshort16(dest)^ := ((Pshort16(source) + (x Shr 16))^ Shr 11) Or
                       ((Pshort16(source) + (x Shr 16))^ And $7e0) Or
                       (((Pshort16(source) + (x Shr 16))^ Shl 11) And $f800);
    Inc(x, inc_source);
    Inc(dest, 2);
    Dec(count);
  End;

  c := count Shr 1;
  While c <> 0 Do
  Begin
    Dec(c);

    { Swap r and b, leave g untouched }
    p := ((Pshort16(source) + (x Shr 16))^ Shr 11) Or
         ((Pshort16(source) + (x Shr 16))^ And $7e0) Or
         (((Pshort16(source) + (x Shr 16))^ Shl 11) And $f800);
    Inc(x, inc_source);

    p := p Or ((((Pshort16(source) + (x Shr 16))^ Shr 11) Or
                ((Pshort16(source) + (x Shr 16))^ And $7e0) Or
                (((Pshort16(source) + (x Shr 16))^ Shl 11) And $f800)) Shl 16);
    Inc(x, inc_source);

    Pint32(dest)^ := p;
    Inc(dest, 4);
  End;

  If (count And 1) <> 0 Then
    Pshort16(dest)^ := ((Pshort16(source) + (x Shr 16))^ Shr 11) Or
                       ((Pshort16(source) + (x Shr 16))^ And $7e0) Or
                       (((Pshort16(source) + (x Shr 16))^ Shl 11) And $f800);
End;

Procedure ConvertP_16rgb565_16rgb555_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  x, c : DWord;
  p : int32;

Begin
  x := 0;
  { If we are not aligned to a dword, try and convert a single pixel }
  If (PtrUInt(dest) And $3) <> 0 Then
  Begin
    Pshort16(dest)^ := (((Pshort16(source) + (x Shr 16))^ And $f800) Shr 1) Or
                       (((Pshort16(source) + (x Shr 16))^ And $7c0) Shr 1) Or
                       ((Pshort16(source) + (x Shr 16))^ And $1f);
    Inc(dest, 2);
    Inc(x, inc_source);
    Dec(count);
  End;

  c := count Shr 1;
  While c <> 0 Do
  Begin
    Dec(c);

    { Leave blue untouched, mask red and shift by one, mask green and shift
      by one }
    p := (((Pshort16(source) + (x Shr 16))^ And $f800) Shr 1) Or
         (((Pshort16(source) + (x Shr 16))^ And $7c0) Shr 1) Or
         ((Pshort16(source) + (x Shr 16))^ And $1f);
    Inc(x, inc_source);

    p := p Or (((((Pshort16(source) + (x Shr 16))^ And $f800) Shr 1) Or
                (((Pshort16(source) + (x Shr 16))^ And $7c0) Shr 1) Or
                ((Pshort16(source) + (x Shr 16))^ And $1f)) Shl 16);
    Inc(x, inc_source);

    Pint32(dest)^ := p;
    Inc(dest, 4);
  End;

  If (count And 1) <> 0 Then
    Pshort16(dest)^ := (((Pshort16(source) + (x Shr 16))^ And $f800) Shr 1) Or
                       (((Pshort16(source) + (x Shr 16))^ And $7c0) Shr 1) Or
                       ((Pshort16(source) + (x Shr 16))^ And $1f);
End;

Procedure ConvertP_16rgb565_16bgr555_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  x, c : DWord;
  p : int32;

Begin
  x := 0;
  { If we are not aligned to a dword, try and convert a single pixel }
  If (PtrUInt(dest) And $3) <> 0 Then
  Begin
    Pshort16(dest)^ := (((Pshort16(source) + (x Shr 16))^ And $f800) Shr 11) Or
                       (((Pshort16(source) + (x Shr 16))^ And $7c0) Shr 1) Or
                       (((Pshort16(source) + (x Shr 16))^ And $1f) Shl 10);
    Inc(dest, 2);
    Inc(x, inc_source);
    Dec(count);
  End;

  c := count Shr 1;
  While c <> 0 Do
  Begin
    Dec(c);

    p := (((Pshort16(source) + (x Shr 16))^ And $f800) Shr 11) Or
         (((Pshort16(source) + (x Shr 16))^ And $7c0) Shr 1) Or
         (((Pshort16(source) + (x Shr 16))^ And $1f) Shl 10);
    Inc(x, inc_source);

    p := p Or (((((Pshort16(source) + (x Shr 16))^ And $f800) Shr 11) Or
                (((Pshort16(source) + (x Shr 16))^ And $7c0) Shr 1) Or
                (((Pshort16(source) + (x Shr 16))^ And $1f) Shl 10)) Shl 16);
    Inc(x, inc_source);

    Pint32(dest)^ := p;
    Inc(dest, 4);
  End;

  If (count And 1) <> 0 Then
    Pshort16(dest)^ := (((Pshort16(source) + (x Shr 16))^ And $f800) Shr 11) Or
                       (((Pshort16(source) + (x Shr 16))^ And $7c0) Shr 1) Or
                       (((Pshort16(source) + (x Shr 16))^ And $1f) Shl 10);
End;

Procedure ConvertP_16rgb565_8rgb332_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  x, c : DWord;
  p : int32;

Begin
  x := 0;

  { Write single pixels until the destination address is aligned mod 4 }
  While (PtrUInt(dest) And $3) <> 0 Do
  Begin
    dest^ := (((Pshort16(source) + (x Shr 16))^ Shr 8) And $e0) Or
             (((Pshort16(source) + (x Shr 16))^ Shr 6) And $1c) Or
             (((Pshort16(source) + (x Shr 16))^ Shr 3) And $3);
    Inc(x, inc_source);
    Inc(dest);
    Dec(count);
    If count = 0 Then
      Exit;
  End;


  {* Write blocks of four pixels now }
  c := count Shr 2;
  While c <> 0 Do
  Begin
    Dec(c);
    p := (((Pshort16(source) + (x Shr 16))^ Shr 8) And $e0) Or
         (((Pshort16(source) + (x Shr 16))^ Shr 6) And $1c) Or
         (((Pshort16(source) + (x Shr 16))^ Shr 3) And $3);
    Inc(x, inc_source);

    p := p Or
      (((((Pshort16(source) + (x Shr 16))^ Shr 8) And $e0) Or
        (((Pshort16(source) + (x Shr 16))^ Shr 6) And $1c) Or
        (((Pshort16(source) + (x Shr 16))^ Shr 3) And $3)) Shl 8);
    Inc(x, inc_source);

    p := p Or
      (((((Pshort16(source) + (x Shr 16))^ Shr 8) And $e0) Or
        (((Pshort16(source) + (x Shr 16))^ Shr 6) And $1c) Or
        (((Pshort16(source) + (x Shr 16))^ Shr 3) And $3)) Shl 16);
    Inc(x, inc_source);

    p := p Or
      (((((Pshort16(source) + (x Shr 16))^ Shr 8) And $e0) Or
        (((Pshort16(source) + (x Shr 16))^ Shr 6) And $1c) Or
        (((Pshort16(source) + (x Shr 16))^ Shr 3) And $3)) Shl 24);
    Inc(x, inc_source);

    Pint32(dest)^ := p;

    Inc(dest, 4);
  End;

  { Write up to three trailing pixels }
  c := count And $3;
  While c <> 0 Do
  Begin
    Dec(c);
    dest^ := (((Pshort16(source) + (x Shr 16))^ Shr 8) And $e0) Or
             (((Pshort16(source) + (x Shr 16))^ Shr 6) And $1c) Or
             (((Pshort16(source) + (x Shr 16))^ Shr 3) And $3);
    Inc(x, inc_source);
    Inc(dest);
  End;
End;
