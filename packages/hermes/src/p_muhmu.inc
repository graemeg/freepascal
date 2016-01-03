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
   muhmuh converters for the HERMES library
   Copyright (c) 1998 Christian Nentwich (c.nentwich@cs.ucl.ac.uk)
   This source code is licensed under the GNU LGPL
 
   Please refer to the file COPYING.LIB contained in the distribution for
   licensing conditions
}

{ TO 32 RGB }
Procedure ConvertP_muhmu32_32rgb888(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  s_pixel : int32;

Begin
  Repeat
    s_pixel := Pint32(source)^;

    Pint32(dest)^ := (s_pixel And $ff) Or
                    ((s_pixel And ($ff Shl 10)) Shr 2) Or
	            ((s_pixel And ($ff Shl 20)) Shr 4);
    
    Inc(dest, 4);
    Inc(source, 4);
    Dec(count);
  Until count = 0;
End;

{ TO 32 BGR }
Procedure ConvertP_muhmu32_32bgr888(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  s_pixel : int32;

Begin
  Repeat
    s_pixel := Pint32(source)^;

    Pint32(dest)^ := ((s_pixel And $ff) Shl 16) Or
                     ((s_pixel And ($ff Shl 10)) Shr 2) Or
	             ((s_pixel Shr 20) And $FF);

    Inc(dest, 4);
    Inc(source, 4);
    Dec(count);
  Until count = 0;
End;

{ TO 32 RGBA }
Procedure ConvertP_muhmu32_32rgba888(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  s_pixel : int32;

Begin
  Repeat
    s_pixel := Pint32(source)^;

    Pint32(dest)^ := (((s_pixel And $ff) Or
                      ((s_pixel And ($ff Shl 10)) Shr 2) Or
	              ((s_pixel And ($ff Shl 20)) Shr 4)) Shl 8) Or $FF;
    
    Inc(dest, 4);
    Inc(source, 4);
    Dec(count);
  Until count = 0;
End;

{ TO 32 BGRA }
Procedure ConvertP_muhmu32_32bgra888(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  s_pixel : int32;

Begin
  Repeat
    s_pixel := Pint32(source)^;

    Pint32(dest)^ := (((s_pixel And $ff) Shl 24) Or
                      ((s_pixel And ($ff Shl 10)) Shl 6) Or
	              ((s_pixel Shr 12) And $FF00)) Or $FF;

    Inc(dest, 4);
    Inc(source, 4);
    Dec(count);
  Until count = 0;
End;

{ TO 24 RGB }
Procedure ConvertP_muhmu32_24rgb888(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  s_pixel : int32;
  s_point : Pchar8;

Begin
  s_point := @s_pixel;
  Repeat
    s_pixel := Pint32(source)^;
    s_pixel := (s_pixel And $ff) Or
               ((s_pixel And ($ff Shl 10)) Shr 2) Or
	       ((s_pixel And ($ff Shl 20)) Shr 4);

    (dest+R_24)^ := (s_point+R_32)^;
    (dest+G_24)^ := (s_point+G_32)^;
    (dest+B_24)^ := (s_point+B_32)^;

    Inc(source, 4);
    Inc(dest, 3);
    Dec(count);
  Until count = 0;
End;

{ TO 24 BGR }
Procedure ConvertP_muhmu32_24bgr888(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  s_pixel : int32;
  s_point : Pchar8;

Begin
  s_point := @s_pixel;
  Repeat
    s_pixel := Pint32(source)^;
    s_pixel := (s_pixel And $ff) Or
               ((s_pixel And ($ff Shl 10)) Shr 2) Or
	       ((s_pixel And ($ff Shl 20)) Shr 4);

    { Note that R and B are swapped }
    (dest+B_24)^ := (s_point+R_32)^;
    (dest+G_24)^ := (s_point+G_32)^;
    (dest+R_24)^ := (s_point+B_32)^;

    Inc(source, 4);
    Inc(dest, 3);
    Dec(count);
  Until count = 0;
End;

{ TO 16 RGB 565 }
Procedure ConvertP_muhmu32_16rgb565(source, dest : Pchar8; count, inc_source : DWord); CDecl;

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
  
    r := (s_pixel Shr 12) And $f800;
    g := (s_pixel Shr 7) And $7e0;
    b := (s_pixel Shr 3) And $1f;

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

    d_pixelblock := ((s_pixel Shr 12) And $f800) Or
                    ((s_pixel Shr 7) And $7e0) Or
		    ((s_pixel Shr 3) And $1f);

    s_pixel := (Pint32(source) + 1)^;

    d_pixelblock := d_pixelblock Or
      ((((s_pixel Shr 12) And $f800) Or
        ((s_pixel Shr 7) And $7e0) Or
	((s_pixel Shr 3) And $1f)) Shl 16);

    Pint32(dest)^ := d_pixelblock;
    Inc(source, 8); Inc(dest, 4);
  End;

  { Eventually, write a single odd pixel that might be left }
  If (count And 1) <> 0 Then
  Begin
    s_pixel := Pint32(source)^;

    r := (s_pixel Shr 12) And $f800;
    g := (s_pixel Shr 7) And $7e0;
    b := (s_pixel Shr 3) And $1f;

    d_pixel := r Or g Or b;

    Pshort16(dest)^ := d_pixel;
  End;
End;

{ TO 16 BGR 565 }
Procedure ConvertP_muhmu32_16bgr565(source, dest : Pchar8; count, inc_source : DWord); CDecl;

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
  
    r := (s_pixel Shr 23) And $1f;
    g := (s_pixel Shr 7) And $7e0;
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

    d_pixelblock := ((s_pixel Shr 23) And $1f) Or
                    ((s_pixel Shr 7) And $7e0) Or
		    ((s_pixel Shl 8) And $f800);

    s_pixel := (Pint32(source) + 1)^;

    d_pixelblock := d_pixelblock Or
      ((((s_pixel Shr 23) And $1f) Or
        ((s_pixel Shr 7) And $7e0) Or
	((s_pixel Shl 8) And $f800)) Shl 16);

    Pint32(dest)^ := d_pixelblock;
    Inc(source, 8); Inc(dest, 4);
  End;

  { Eventually, write a single odd pixel that might be left }
  If (count And 1) <> 0 Then
  Begin
    s_pixel := Pint32(source)^;

    r := (s_pixel Shr 23) And $1f;
    g := (s_pixel Shr 7) And $7e0;
    b := (s_pixel Shl 8) And $f800;

    d_pixel := r Or g Or b;

    Pshort16(dest)^ := d_pixel;
  End;
End;

{ TO 16 RGB 555 }
Procedure ConvertP_muhmu32_16rgb555(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  i : DWord;
  r, g, b : int32;
  s_pixel, d_pixelblock : int32;
  d_pixel : short16;

Begin
  If (PtrUInt(dest) And $3) <> 0 Then
  Begin
    s_pixel := Pint32(source)^;
  
    r := (s_pixel Shr 13) And $7c00;
    g := (s_pixel Shr 8) And $3e0;
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

    d_pixelblock := ((s_pixel Shr 13) And $7c00) Or
                    ((s_pixel Shr 8) And $3e0) Or
		    ((s_pixel Shr 3) And $1f);

    s_pixel := (Pint32(source) + 1)^;

    d_pixelblock := d_pixelblock Or
      ((((s_pixel Shr 13) And $7c00) Or
        ((s_pixel Shr 8) And $3e0) Or
	((s_pixel Shr 3) And $1f)) Shl 16);

    Pint32(dest)^ := d_pixelblock;
    Inc(source, 8); Inc(dest, 4);
  End;

  If (count And 1) <> 0 Then
  Begin
    s_pixel := Pint32(source)^;

    r := (s_pixel Shr 13) And $7c00;
    g := (s_pixel Shr 8) And $3e0;
    b := (s_pixel Shr 3) And $1f;

    d_pixel := r Or g Or b;

    Pshort16(dest)^ := d_pixel;
  End;
End;

{ TO 16 BGR 555 }
Procedure ConvertP_muhmu32_16bgr555(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  i : DWord;
  r, g, b : int32;
  s_pixel, d_pixelblock : int32;
  d_pixel : short16;

Begin
  If (PtrUInt(dest) And $3) <> 0 Then
  Begin
    s_pixel := Pint32(source)^;
  
    r := (s_pixel Shr 23) And $1f;
    g := (s_pixel Shr 8) And $3e0;
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

    d_pixelblock := ((s_pixel Shr 23) And $1f) Or
                    ((s_pixel Shr 8) And $3e0) Or
		    ((s_pixel Shl 7) And $7c00);

    s_pixel := (Pint32(source) + 1)^;

    d_pixelblock := d_pixelblock Or
      ((((s_pixel Shr 23) And $1f) Or
        ((s_pixel Shr 8) And $3e0) Or
	((s_pixel Shl 7) And $7c00)) Shl 16);

    Pint32(dest)^ := d_pixelblock;
    Inc(source, 8); Inc(dest, 4);
  End;

  If (count And 1) <> 0 Then
  Begin
    s_pixel := Pint32(source)^;

    r := (s_pixel Shr 23) And $1f;
    g := (s_pixel Shr 8) And $3e0;
    b := (s_pixel Shl 7) And $7c00;

    d_pixel := r Or g Or b;

    Pshort16(dest)^ := d_pixel;
  End;
End;

{ TO 8 RGB 332 }
Procedure ConvertP_muhmu32_8rgb332(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  i : DWord;
  s_pixel, d_block : int32;
  d_pixel : char8;

Begin
  { Process single pixels until we are dword aligned }
  While (PtrUInt(dest) And $3) <> 0 Do
  Begin
    s_pixel := Pint32(source)^;

    d_pixel := ((s_pixel Shr 20) And $e0) Or
               ((s_pixel Shr 13) And $1c) Or
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
    d_block := ((s_pixel Shr 20) And $e0) Or
               ((s_pixel Shr 13) And $1c) Or
	       ((s_pixel Shr 6) And $3);

    s_pixel := (Pint32(source) + 1)^;
    d_block := d_block Or
               ((((s_pixel Shr 20) And $e0) Or
                 ((s_pixel Shr 13) And $1c) Or
	         ((s_pixel Shr 6) And $3)) Shl 8);

    s_pixel := (Pint32(source) + 2)^;
    d_block := d_block Or
               ((((s_pixel Shr 20) And $e0) Or
                 ((s_pixel Shr 13) And $1c) Or
	         ((s_pixel Shr 6) And $3)) Shl 16);

    s_pixel := (Pint32(source) + 3)^;
    d_block := d_block Or
               ((((s_pixel Shr 20) And $e0) Or
                 ((s_pixel Shr 13) And $1c) Or
	         ((s_pixel Shr 6) And $3)) Shl 24);

    Pint32(dest)^ := d_block;
    Inc(source, 16);
    Inc(dest, 4);
  End;

  { Write all possibly remaining pixel }
  count := count And 3;
  While count <> 0 Do
  Begin
    Dec(count);
    dest^ := ((s_pixel Shr 20) And $e0) Or
             ((s_pixel Shr 13) And $1c) Or
	     ((s_pixel Shr 6) And $3);

    Inc(dest);
    Inc(source, 4);
  End;
End;

{ -------------------------------------------------------------------------

                             STRETCH CONVERTERS
   
  ------------------------------------------------------------------------- }

Procedure ConvertP_muhmu32_32rgb888_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  x : DWord;
  s_pixel : DWord;

Begin
  x := 0;
  While count > 0 Do
  Begin
    s_pixel := Pint32(source)^;
    Pint32(dest)^ := (s_pixel And $ff) Or
                    ((s_pixel And ($ff Shl 10)) Shr 2) Or
	            ((s_pixel And ($ff Shl 20)) Shr 4);
    Inc(x, inc_source);
    Inc(source, (x Shr 16)*4);
    x := x And $FFFF;
    Inc(dest, 4);
    Dec(count);
  End;
End;

Procedure ConvertP_muhmu32_32bgr888_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  x : DWord;
  s_pixel : DWord;

Begin
  x := 0;
  While count > 0 Do
  Begin
    s_pixel := Pint32(source)^;
    Pint32(dest)^ := ((s_pixel And $ff) Shl 16) Or
                     ((s_pixel And ($ff Shl 10)) Shr 2) Or
	             ((s_pixel Shr 20) And $FF);
    Inc(x, inc_source);
    Inc(source, (x Shr 16)*4);
    x := x And $FFFF;
    Inc(dest, 4);
    Dec(count);
  End;
End;

Procedure ConvertP_muhmu32_32rgba888_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  x : DWord;
  s_pixel : DWord;

Begin
  x := 0;
  While count > 0 Do
  Begin
    s_pixel := Pint32(source)^;
    Pint32(dest)^ := (((s_pixel And $ff) Or
                      ((s_pixel And ($ff Shl 10)) Shr 2) Or
	              ((s_pixel And ($ff Shl 20)) Shr 4)) Shl 8) Or $FF;
    Inc(x, inc_source);
    Inc(source, (x Shr 16)*4);
    x := x And $FFFF;
    Inc(dest, 4);
    Dec(count);
  End;
End;

Procedure ConvertP_muhmu32_32bgra888_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  x : DWord;
  s_pixel : DWord;

Begin
  x := 0;
  While count > 0 Do
  Begin
    s_pixel := Pint32(source)^;
    Pint32(dest)^ := (((s_pixel And $ff) Shl 24) Or
                      ((s_pixel And ($ff Shl 10)) Shl 6) Or
	              ((s_pixel Shr 12) And $FF00)) Or $FF;
    Inc(x, inc_source);
    Inc(source, (x Shr 16)*4);
    x := x And $FFFF;
    Inc(dest, 4);
    Dec(count);
  End;
End;

Procedure ConvertP_muhmu32_24rgb888_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  x : DWord;
  p1, p2, p3, p4 : DWord;
  c : DWord;

Begin
  x := 0;
  While (PtrUInt(dest) And 3) <> 0 Do
  Begin
    p1 := (Pint32(source) + (x Shr 16))^;
    p1 := (p1 And $ff) Or
         ((p1 And ($ff Shl 10)) Shr 2) Or
         ((p1 And ($ff Shl 20)) Shr 4);
    Pshort16(dest)^ := p1 And $FFFF;
    Pchar8(dest + 2)^ := p1 Shr 16;
    
    Inc(x, inc_source);
    Inc(dest, 3);
    Dec(count);
    If count = 0 Then
      Exit;
  End;
  
  c := count Shr 2;
  While c > 0 Do
  Begin
    p1 := (Pint32(source) + (x Shr 16))^;
    p2 := (Pint32(source) + ((x + inc_source) Shr 16))^;
    p3 := (Pint32(source) + ((x + 2*inc_source) Shr 16))^;
    p4 := (Pint32(source) + ((x + 3*inc_source) Shr 16))^;
    
    Pint32(dest + 0)^ := ((p2 And $FF) Shl 24) Or ((p1 And $FF00000) Shr 4) Or ((p1 And $3FC00) Shr 2) Or (p1 And $FF);
    Pint32(dest + 4)^ := ((p3 And $3FC00) Shl 14) Or ((p3 And $FF) Shl 16) Or ((p2 And $FF00000) Shr 12) Or ((p2 And $3FC00) Shr 10);
    Pint32(dest + 8)^ := ((p4 And $FF00000) Shl 4) Or ((p4 And $3FC00) Shl 6) Or ((p4 And $FF) Shl 8) Or ((p3 And $FF00000) Shr 20);
    
    Dec(c);
    Inc(x, inc_source*4);
    Inc(dest, 12);
  End;
  
  count := count And 3;
  While count > 0 Do
  Begin
    p1 := (Pint32(source) + (x Shr 16))^;
    p1 := (p1 And $ff) Or
         ((p1 And ($ff Shl 10)) Shr 2) Or
         ((p1 And ($ff Shl 20)) Shr 4);
    Pshort16(dest)^ := p1 And $FFFF;
    Pchar8(dest + 2)^ := p1 Shr 16;
    
    Inc(x, inc_source);
    Inc(dest, 3);
    Dec(count);
  End;
End;

Procedure ConvertP_muhmu32_24bgr888_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  x : DWord;
  p1, p2, p3, p4 : DWord;
  c : DWord;

Begin
  x := 0;
  While (PtrUInt(dest) And 3) <> 0 Do
  Begin
    p1 := (Pint32(source) + (x Shr 16))^;
    p1 := ((p1 And $ff) Shl 16) Or
          ((p1 And ($ff Shl 10)) Shr 2) Or
          ((p1 And ($ff Shl 20)) Shr 20);
    Pshort16(dest)^ := p1 And $FFFF;
    Pchar8(dest + 2)^ := p1 Shr 16;
    
    Inc(x, inc_source);
    Inc(dest, 3);
    Dec(count);
    If count = 0 Then
      Exit;
  End;
  
  c := count Shr 2;
  While c > 0 Do
  Begin
    p1 := (Pint32(source) + (x Shr 16))^;
    p2 := (Pint32(source) + ((x + inc_source) Shr 16))^;
    p3 := (Pint32(source) + ((x + 2*inc_source) Shr 16))^;
    p4 := (Pint32(source) + ((x + 3*inc_source) Shr 16))^;
    
    Pint32(dest + 0)^ := ((p2 And $FF00000) Shl 4) Or ((p1 And $FF) Shl 16) Or ((p1 And $3FC00) Shr 2) Or ((p1 And $FF00000) Shr 20);
    Pint32(dest + 4)^ := ((p3 And $3FC00) Shl 14) Or ((p3 And $FF00000) Shr 4) Or ((p2 And $FF) Shl 8) Or ((p2 And $3FC00) Shr 10);
    Pint32(dest + 8)^ := ((p4 And $FF) Shl 24) Or ((p4 And $3FC00) Shl 6) Or ((p4 And $FF00000) Shr 12) Or (p3 And $FF);
    
    Dec(c);
    Inc(x, inc_source*4);
    Inc(dest, 12);
  End;
  
  count := count And 3;
  While count > 0 Do
  Begin
    p1 := (Pint32(source) + (x Shr 16))^;
    p1 := ((p1 And $ff) Shl 16) Or
          ((p1 And ($ff Shl 10)) Shr 2) Or
          ((p1 And ($ff Shl 20)) Shr 20);
    Pshort16(dest)^ := p1 And $FFFF;
    Pchar8(dest + 2)^ := p1 Shr 16;
    
    Inc(x, inc_source);
    Inc(dest, 3);
    Dec(count);
  End;
End;

Procedure ConvertP_muhmu32_16rgb565_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  x, c : DWord;
  p : int32;

Begin
  x := 0;
  { Try to write 2 pixel blocks }
  c := count Shr 1; 
  While c <> 0 Do
  Begin
    Dec(c);
    p := (((Pint32(source) + (x Shr 16))^ Shr 12) And $f800) Or
         (((Pint32(source) + (x Shr 16))^ Shr 7) And $7e0) Or
	 (((Pint32(source) + (x Shr 16))^ Shr 3) And $1f);
    Inc(x, inc_source);

    p := p Or
       (((((Pint32(source) + (x Shr 16))^ Shr 12) And $f800) Or
         (((Pint32(source) + (x Shr 16))^ Shr 7) And $7e0) Or
	 (((Pint32(source) + (x Shr 16))^ Shr 3) And $1f)) Shl 16);
    Inc(x, inc_source);

    Pint32(dest)^ := p;

    Inc(dest, 4);
  End;

  { Write trailing pixel }
  If (count And 1) <> 0 Then
    Pshort16(dest)^ := (((Pint32(source) + (x Shr 16))^ Shr 12) And $f800) Or
                       (((Pint32(source) + (x Shr 16))^ Shr 7) And $7e0) Or
	               (((Pint32(source) + (x Shr 16))^ Shr 3) And $1f);
End;

Procedure ConvertP_muhmu32_16bgr565_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  x, c : DWord;
  p : int32;

Begin
  x := 0;
  { Try to write 2 pixel blocks }
  c := count Shr 1; 
  While c <> 0 Do
  Begin
    Dec(c);
    p := (((Pint32(source) + (x Shr 16))^ Shr 23) And $1f) Or
         (((Pint32(source) + (x Shr 16))^ Shr 7) And $7e0) Or
	 (((Pint32(source) + (x Shr 16))^ Shl 8) And $f800);
    Inc(x, inc_source);

    p := p Or
       (((((Pint32(source) + (x Shr 16))^ Shr 23) And $1f) Or
         (((Pint32(source) + (x Shr 16))^ Shr 7) And $7e0) Or
	 (((Pint32(source) + (x Shr 16))^ Shl 8) And $f800)) Shl 16);
    Inc(x, inc_source);

    Pint32(dest)^ := p;

    Inc(dest, 4);
  End;

  { Write trailing pixel }
  If (count And 1) <> 0 Then
    Pshort16(dest)^ := (((Pint32(source) + (x Shr 16))^ Shr 23) And $1f) Or
                       (((Pint32(source) + (x Shr 16))^ Shr 7) And $7e0) Or
	               (((Pint32(source) + (x Shr 16))^ Shl 8) And $f800);
End;

Procedure ConvertP_muhmu32_16rgb555_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  x, c : DWord;
  p : int32;

Begin
  x := 0;
  { Try to write 2 pixel blocks }
  c := count Shr 1; 
  While c <> 0 Do
  Begin
    Dec(c);
    p := (((Pint32(source) + (x Shr 16))^ Shr 13) And $7c00) Or
         (((Pint32(source) + (x Shr 16))^ Shr 8) And $3e0) Or
	 (((Pint32(source) + (x Shr 16))^ Shr 3) And $1f);
    Inc(x, inc_source);

    p := p Or
       (((((Pint32(source) + (x Shr 16))^ Shr 13) And $7c00) Or
         (((Pint32(source) + (x Shr 16))^ Shr 8) And $3e0) Or
	 (((Pint32(source) + (x Shr 16))^ Shr 3) And $1f)) Shl 16);
    Inc(x, inc_source);

    Pint32(dest)^ := p;

    Inc(dest, 4);
  End;

  { Write trailing pixel }
  If (count And 1) <> 0 Then
    Pshort16(dest)^ := (((Pint32(source) + (x Shr 16))^ Shr 13) And $7c00) Or
                       (((Pint32(source) + (x Shr 16))^ Shr 8) And $3e0) Or
	               (((Pint32(source) + (x Shr 16))^ Shr 3) And $1f);
End;

Procedure ConvertP_muhmu32_16bgr555_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  x, c : DWord;
  p : int32;

Begin
  x := 0;
  { Try to write 2 pixel blocks }
  c := count Shr 1; 
  While c <> 0 Do
  Begin
    Dec(c);
    p := (((Pint32(source) + (x Shr 16))^ Shr 23) And $1f) Or
         (((Pint32(source) + (x Shr 16))^ Shr 8) And $3e0) Or
	 (((Pint32(source) + (x Shr 16))^ Shl 7) And $7c00);
    Inc(x, inc_source);

    p := p Or
       (((((Pint32(source) + (x Shr 16))^ Shr 23) And $1f) Or
         (((Pint32(source) + (x Shr 16))^ Shr 8) And $3e0) Or
	 (((Pint32(source) + (x Shr 16))^ Shl 7) And $7c00)) Shl 16);
    Inc(x, inc_source);

    Pint32(dest)^ := p;

    Inc(dest, 4);
  End;

  { Write trailing pixel }
  If (count And 1) <> 0 Then
    Pshort16(dest)^ := (((Pint32(source) + (x Shr 16))^ Shr 23) And $1f) Or
                       (((Pint32(source) + (x Shr 16))^ Shr 8) And $3e0) Or
	               (((Pint32(source) + (x Shr 16))^ Shl 7) And $7c00);
End;

Procedure ConvertP_muhmu32_8rgb332_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  x, c : DWord;
  p : int32;

Begin
  x := 0;

  { Write single pixels until the destination address is aligned mod 4 }
  While (PtrUInt(dest) And $3) <> 0 Do
  Begin
    dest^ := (((Pint32(source) + (x Shr 16))^ Shr 20) And $e0) Or
             (((Pint32(source) + (x Shr 16))^ Shr 13) And $1c) Or
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
    p := (((Pint32(source) + (x Shr 16))^ Shr 20) And $e0) Or
         (((Pint32(source) + (x Shr 16))^ Shr 13) And $1c) Or
	 (((Pint32(source) + (x Shr 16))^ Shr 6) And $3);
    Inc(x, inc_source);

    p := p Or
       (((((Pint32(source) + (x Shr 16))^ Shr 20) And $e0) Or
         (((Pint32(source) + (x Shr 16))^ Shr 13) And $1c) Or
	 (((Pint32(source) + (x Shr 16))^ Shr 6) And $3)) Shl 8);
    Inc(x, inc_source);

    p := p Or
       (((((Pint32(source) + (x Shr 16))^ Shr 20) And $e0) Or
         (((Pint32(source) + (x Shr 16))^ Shr 13) And $1c) Or
	 (((Pint32(source) + (x Shr 16))^ Shr 6) And $3)) Shl 16);
    Inc(x, inc_source);

    p := p Or
       (((((Pint32(source) + (x Shr 16))^ Shr 20) And $e0) Or
         (((Pint32(source) + (x Shr 16))^ Shr 13) And $1c) Or
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
    dest^ := (((Pint32(source) + (x Shr 16))^ Shr 20) And $e0) Or
             (((Pint32(source) + (x Shr 16))^ Shr 13) And $1c) Or
	     (((Pint32(source) + (x Shr 16))^ Shr 6) And $3);
    Inc(x, inc_source);
    Inc(dest);
  End;
End;
