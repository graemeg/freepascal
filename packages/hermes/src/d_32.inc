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
   32 bit to * dithered converters for the HERMES library
   Copyright (c) 1998 Christian Nentwich (c.nentwich@cs.ucl.ac.uk)
   This source code is licensed under the GNU LGPL
  
   Please refer to the file COPYING.LIB contained in the distribution for
   licensing conditions
}

Procedure ConvertP_32rgb888_16rgb565_dither(iface : PHermesConverterInterface); CDecl;

Var
  source, dest : Pchar8;
  d_pixel : int32;
  y, count : LongInt;

Begin
  y := 0;
  source := iface^.s_pixels;
  dest := iface^.d_pixels;
  While y < iface^.d_height Do
  Begin
    { Get counter for this scanline }
    count := iface^.d_width;

    { Check first pixel alignment, correct if necessary }
    If (PtrUInt(iface^.d_pixels) And 3) <> 0 Then
    Begin
      Pshort16(dest)^ := 
        DitherTab_r565_44[count And 3, y And 3, (Pint32(source)^ Shr 16) And $ff] Or
	DitherTab_g565_44[count And 3, y And 3, (Pint32(source)^ Shr 8) And $ff] Or
	DitherTab_b565_44[count And 3, y And 3, Pint32(source)^ And $ff];

      Inc(source, 4);
      Inc(dest, 2);
      Dec(count);
    End;

    { Two pixels at a time loop }
    While count > 1 Do
    Begin
      d_pixel :=
        DitherTab_r565_44[count And 3, y And 3, (Pint32(source)^ Shr 16) And $ff] Or
	DitherTab_g565_44[count And 3, y And 3, (Pint32(source)^ Shr 8) And $ff] Or
	DitherTab_b565_44[count And 3, y And 3, Pint32(source)^ And $ff];

      Inc(source, 4);
      Dec(count);

      d_pixel := d_pixel Or ((
        DitherTab_r565_44[count And 3, y And 3, (Pint32(source)^ Shr 16) And $ff] Or
	DitherTab_g565_44[count And 3, y And 3, (Pint32(source)^ Shr 8) And $ff] Or
	DitherTab_b565_44[count And 3, y And 3, Pint32(source)^ And $ff]) Shl 16);

      Dec(count);
      Inc(source, 4);

      Pint32(dest)^ := d_pixel;

      Inc(dest, 4);
    End;

    { Convert the odd trailing pixel }
    If (iface^.d_width And 1) <> 0 Then
    Begin
      Pshort16(dest)^ :=
        DitherTab_r565_44[count And 3, y And 3, (Pint32(source)^ Shr 16) And $ff] Or
	DitherTab_g565_44[count And 3, y And 3, (Pint32(source)^ Shr 8) And $ff] Or
	DitherTab_b565_44[count And 3, y And 3, Pint32(source)^ And $ff];

      Inc(source, 4);
      Inc(dest, 2);
    End;

    Inc(source, iface^.s_add);
    Inc(dest, iface^.d_add);

    Inc(y);
  End;
End;

Procedure ConvertP_32rgb888_8rgb332_dither(iface : PHermesConverterInterface); CDecl;

Var
  source, dest : Pchar8;
  d_pixel : int32;
  y, count : LongInt;

Begin
  y := 0;
  source := iface^.s_pixels;
  dest := iface^.d_pixels;
 
  While y < iface^.d_height Do
  Begin
    { Get counter for this scanline }
    count := iface^.d_width;


    { TODO: alignment loop }


    { Convert 4 pixels at a time }
    While count > 3 Do
    Begin
      d_pixel :=
        DitherTab_r332_44[count And 3, y And 3, (Pint32(source)^ Shr 16) And $ff] Or
	DitherTab_g332_44[count And 3, y And 3, (Pint32(source)^ Shr 8) And $ff] Or
	DitherTab_b332_44[count And 3, y And 3, Pint32(source)^ And $ff];
      Dec(count);
      Inc(source, 4);

      d_pixel := d_pixel Or ((
        DitherTab_r332_44[count And 3, y And 3, (Pint32(source)^ Shr 16) And $ff] Or
	DitherTab_g332_44[count And 3, y And 3, (Pint32(source)^ Shr 8) And $ff] Or
	DitherTab_b332_44[count And 3, y And 3, Pint32(source)^ And $ff]) Shl 8);
      Dec(count);
      Inc(source, 4);

      d_pixel := d_pixel Or ((
        DitherTab_r332_44[count And 3, y And 3, (Pint32(source)^ Shr 16) And $ff] Or
	DitherTab_g332_44[count And 3, y And 3, (Pint32(source)^ Shr 8) And $ff] Or
	DitherTab_b332_44[count And 3, y And 3, Pint32(source)^ And $ff]) Shl 16);
      Dec(count);
      Inc(source, 4);

      d_pixel := d_pixel Or ((
        DitherTab_r332_44[count And 3, y And 3, (Pint32(source)^ Shr 16) And $ff] Or
	DitherTab_g332_44[count And 3, y And 3, (Pint32(source)^ Shr 8) And $ff] Or
	DitherTab_b332_44[count And 3, y And 3, Pint32(source)^ And $ff]) Shl 24);
      Dec(count);
      Inc(source, 4);

      Pint32(dest)^ := d_pixel;
      Inc(dest, 4);
    End;

    { Write trailing pixels }
    While count <> 0 Do
    Begin
      Dec(count);
      dest^ := 
        DitherTab_r332_44[count And 3, y And 3, (Pint32(source)^ Shr 16) And $ff] Or
	DitherTab_g332_44[count And 3, y And 3, (Pint32(source)^ Shr 8) And $ff] Or
	DitherTab_b332_44[count And 3, y And 3, Pint32(source)^ And $ff];

      Inc(source, 4);
      Inc(dest);
    End;

    Inc(source, iface^.s_add);
    Inc(dest, iface^.d_add);

    Inc(y);
  End;
End;
