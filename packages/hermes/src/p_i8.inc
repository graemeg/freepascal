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

   Generic C converter (from 8 bit indexed) for the HERMES library
   Copyright (c) 1998 Christian Nentwich (c.nentwich@cs.ucl.ac.uk)
   This source code is licensed under the GNU LGPL

   Please refer to the file COPYING.LIB contained in the distribution for
   licensing conditions
}

{ -------------------------------------------------------------------------

                             NORMAL CONVERTERS

  ------------------------------------------------------------------------- }

Procedure ConvertP_index8_32(iface : PHermesConverterInterface); CDecl;

Var
  i : Integer;
  s_pixel : char8;
  d_pixel : int32;
  source, dest : Pchar8;

Begin
  source := iface^.s_pixels;
  dest := iface^.d_pixels;
  Repeat
    For i := 0 To iface^.s_width - 1 Do
    Begin
      s_pixel := source^;
      d_pixel := iface^.lookup[s_pixel];
      Pint32(dest)^ := d_pixel;
      Inc(source);
      Inc(dest, 4);
    End;
    Inc(source, iface^.s_add);
    Inc(dest, iface^.d_add);
    Dec(iface^.s_height);
  Until iface^.s_height = 0;
End;

Procedure ConvertP_index8_24(iface : PHermesConverterInterface); CDecl;

Var
  count : Integer;
  s_pixel, s_pixel2, d_pixel : int32;
  d_ptr, source, dest : Pchar8;

Begin
  d_ptr := @d_pixel;
  source := iface^.s_pixels;
  dest := iface^.d_pixels;
  Repeat
    count := iface^.d_width Shr 2;
    While count <> 0 Do
    Begin
      Dec(count);

      s_pixel := iface^.lookup[source^]; Inc(source);
      s_pixel2 := iface^.lookup[source^]; Inc(source);

      s_pixel := (s_pixel And $ffffff) Or (s_pixel2 Shl 24);
      Pint32(dest)^ := s_pixel;

      s_pixel := iface^.lookup[source^]; Inc(source);
      s_pixel2 := ((s_pixel2 Shr 8) And $ffff) Or (s_pixel Shl 16);
      Pint32(dest + 4)^ := s_pixel2;

      s_pixel2 := iface^.lookup[source^]; Inc(source);
      s_pixel := ((s_pixel Shr 16) And $ff) Or (s_pixel2 Shl 8);
      Pint32(dest + 8)^ := s_pixel;

      Inc(dest, 12);
    End;

    count := iface^.d_width And $3;
    While count <> 0 Do
    Begin
      Dec(count);
      d_pixel := iface^.lookup[source^]; Inc(source);

      (dest + R_24)^ := (d_ptr + R_32)^;
      (dest + G_24)^ := (d_ptr + G_32)^;
      (dest + B_24)^ := (d_ptr + B_32)^;

      Inc(dest, 3);
    End;
    Inc(source, iface^.s_add);
    Inc(dest, iface^.d_add);
    Dec(iface^.d_height);
  Until iface^.d_height = 0;
End;

Procedure ConvertP_index8_16(iface : PHermesConverterInterface); CDecl;

Var
  source, dest : Pchar8;
  count, c : DWord;

Begin
  source := iface^.s_pixels;
  dest := iface^.d_pixels;
  Repeat
    count := iface^.s_width;
    If (PtrUInt(dest) And $3) <> 0 Then
    Begin
      Pshort16(dest)^ := iface^.lookup[source^]; Inc(source);
      Inc(dest, 2);
      Dec(count);
    End;
    c := count Shr 1;
    If c <> 0 Then
      Repeat
        Pint32(dest)^ := iface^.lookup[source^] Or
                        (iface^.lookup[(source + 1)^] Shl 16);
        Inc(dest, 4);
        Inc(source, 2);
        Dec(c);
      Until c = 0;
    If (count And 1) <> 0 Then
    Begin
      Pshort16(dest)^ := iface^.lookup[source^];
      Inc(source);
      Inc(dest, 2);
    End;
    Inc(source, iface^.s_add);
    Inc(dest, iface^.d_add);
    Dec(iface^.s_height);
  Until iface^.s_height = 0;
End;

Procedure ConvertP_index8_8(iface : PHermesConverterInterface); CDecl;

Var
  source, dest : Pchar8;
  count, c : DWord;

Begin
  source := iface^.s_pixels;
  dest := iface^.d_pixels;
  Repeat
    count := iface^.s_width;
    If (PtrUInt(dest) And $3) <> 0 Then
    Begin
      dest^ := iface^.lookup[source^]; Inc(source);
      Inc(dest);
      Dec(count);
    End;
    c := count Shr 2;
    If c <> 0 Then
      Repeat
        Pint32(dest)^ := iface^.lookup[source^] Or
                        (iface^.lookup[(source + 1)^] Shl 8) Or
                        (iface^.lookup[(source + 2)^] Shl 16) Or
                        (iface^.lookup[(source + 3)^] Shl 24);
        Inc(dest, 4);
        Inc(source, 4);
        Dec(c);
      Until c = 0;
    count := count And $03;
    While count > 0 Do
    Begin
      dest^ := iface^.lookup[source^]; Inc(source);
      Inc(dest);
      Dec(count);
    End;
    Inc(source, iface^.s_add);
    Inc(dest, iface^.d_add);
    Dec(iface^.s_height);
  Until iface^.s_height = 0;
End;

{ -------------------------------------------------------------------------

                             STRETCH CONVERTERS

  ------------------------------------------------------------------------- }

Procedure ConvertP_index8_32_S(iface : PHermesConverterInterface); CDecl;

Var
  x, y, count : DWord;
  dx, dy : DWord;
  source : Pchar8;

Begin
  y := 0;
  dy := (iface^.s_height Shl 16) Div iface^.d_height;
  dx := (iface^.s_width Shl 16) Div iface^.d_width;
  source := iface^.s_pixels;
  Repeat
    count := iface^.d_width;
    x := 0;

    Repeat
      Pint32(iface^.d_pixels)^ := iface^.lookup[(source + (x Shr 16))^];
      Inc(x, dx);
      Inc(iface^.d_pixels, 4);
      Dec(count);
    Until count = 0;

    { Go to next destination row }
    Inc(iface^.d_pixels, iface^.d_add);

    { Calculate amount of rows to move in source surface }
    Inc(y, dy);

    Inc(source, (y Shr 16) * DWord(iface^.s_pitch));
    y := y And $ffff;
    Dec(iface^.d_height);
  Until iface^.d_height = 0;
End;

{by me!}
Procedure ConvertP_index8_24_S(iface : PHermesConverterInterface); CDecl;

Var
  x, y, count : DWord;
  dx, dy : DWord;
  source, dest : Pchar8;
  s_pixel, s_pixel2, d_pixel : int32;

Begin
  y := 0;
  dy := (iface^.s_height Shl 16) Div iface^.d_height;
  dx := (iface^.s_width Shl 16) Div iface^.d_width;
  source := iface^.s_pixels;
  dest := iface^.d_pixels;
  Repeat
    x := 0;
    count := iface^.d_width Shr 2;
    While count <> 0 Do
    Begin
      Dec(count);

      s_pixel := iface^.lookup[(source + (x Shr 16))^]; Inc(x, dx);
      s_pixel2 := iface^.lookup[(source + (x Shr 16))^]; Inc(x, dx);

      s_pixel := (s_pixel And $ffffff) Or (s_pixel2 Shl 24);
      Pint32(dest)^ := s_pixel;

      s_pixel := iface^.lookup[(source + (x Shr 16))^]; Inc(x, dx);
      s_pixel2 := ((s_pixel2 Shr 8) And $ffff) Or (s_pixel Shl 16);
      Pint32(dest + 4)^ := s_pixel2;

      s_pixel2 := iface^.lookup[(source + (x Shr 16))^]; Inc(x, dx);
      s_pixel := ((s_pixel Shr 16) And $ff) Or (s_pixel2 Shl 8);
      Pint32(dest + 8)^ := s_pixel;

      Inc(dest, 12);
    End;

    count := iface^.d_width And $3;
    While count <> 0 Do
    Begin
      Dec(count);
      d_pixel := iface^.lookup[(source + (x Shr 16))^]; Inc(x, dx);

      Pshort16(dest)^ := d_pixel;
      Pchar8(dest + 2)^ := d_pixel Shr 16;

      Inc(dest, 3);
    End;

    { Go to next destination row }
{    Inc(iface^.d_pixels, iface^.d_add);}
    Inc(dest, iface^.d_add);

    { Calculate amount of rows to move in source surface }
    Inc(y, dy);

    Inc(source, (y Shr 16) * DWord(iface^.s_pitch));
    y := y And $ffff;
    Dec(iface^.d_height);
  Until iface^.d_height = 0;
End;

{ Quick hack of a index 8 to 16 stretch converter }
Procedure ConvertP_index8_16_S(iface : PHermesConverterInterface); CDecl;

Var
  x, y, count : DWord;
  dx, dy : DWord;
  source, dest : Pchar8;

Begin
  y := 0;
  dy := (iface^.s_height Shl 16) Div iface^.d_height;
  dx := (iface^.s_width Shl 16) Div iface^.d_width;
  source := iface^.s_pixels;
  dest := iface^.d_pixels;
  Repeat
    { Do a two pixel at a time loop }

    count := iface^.d_width Shr 1;
    x := 0;

    While count <> 0 Do
    Begin
      Dec(count);
      Pint32(dest)^ := iface^.lookup[(source + (x Shr 16))^] Or
                      (iface^.lookup[(source + ((x + dx) Shr 16))^] Shl 16);
      Inc(x, dx); Inc(x, dx);
      Inc(dest, 4);
    End;

    { Clean up remaining pixel if odd width }
    If (iface^.d_width And 1) <> 0 Then
    Begin
      Pshort16(dest)^ := iface^.lookup[(source + (x Shr 16))^];
      Inc(dest, 2);
    End;

    { Go to next destination row }
    Inc(dest, iface^.d_add);

    { Calculate amount of rows to move in source surface }
    Inc(y, dy);

    Inc(source, (y Shr 16) * DWord(iface^.s_pitch));
    y := y And $ffff;
    Dec(iface^.d_height);
  Until iface^.d_height = 0;
End;

Procedure ConvertP_index8_8_S(iface : PHermesConverterInterface); CDecl;

Var
  x, y, count, c : DWord;
  dx, dy : DWord;
  source, dest : Pchar8;

Begin
  y := 0;
  dy := (iface^.s_height Shl 16) Div iface^.d_height;
  dx := (iface^.s_width Shl 16) Div iface^.d_width;
  source := iface^.s_pixels;
  dest := iface^.d_pixels;
  Repeat
    { Do a four pixel at a time loop }

    count := iface^.d_width;
    x := 0;

    While ((PtrUInt(dest) And 3) <> 0) And (count > 0) Do
    Begin
      Dec(count);
      dest^ := iface^.lookup[(source + (x Shr 16))^];
      Inc(x, dx);
      Inc(dest);
    End;

    c := count Shr 2;
    count := count And 3;

    While c <> 0 Do
    Begin
      Dec(c);
      Pint32(dest)^ := iface^.lookup[(source + (x Shr 16))^] Or
                      (iface^.lookup[(source + ((x + dx) Shr 16))^] Shl 8) Or
                      (iface^.lookup[(source + ((x + (dx Shl 1)) Shr 16))^] Shl 16) Or
                      (iface^.lookup[(source + ((x + dx + (dx Shl 1)) Shr 16))^] Shl 24);
      Inc(x, dx Shl 2);
      Inc(dest, 4);
    End;

    While count > 0 Do
    Begin
      Dec(count);
      dest^ := iface^.lookup[(source + (x Shr 16))^];
      Inc(dest);
    End;

    { Go to next destination row }
    Inc(dest, iface^.d_add);

    { Calculate amount of rows to move in source surface }
    Inc(y, dy);

    Inc(source, (y Shr 16) * DWord(iface^.s_pitch));
    y := y And $ffff;
    Dec(iface^.d_height);
  Until iface^.d_height = 0;
End;
