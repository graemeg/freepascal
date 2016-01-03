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

{ Everything in here (C)1998 The Rasterman }

{ Rasterman's dither matrix }

Const
  DitherMatrix_44 : Array[0..3, 0..3] Of char8 = (
    (0, 4, 1, 5),
    (6, 2, 7, 3),
    (1, 5, 0, 4),
    (7, 3, 6, 2));

Var
  DitherTab_r565_44 : Array[0..3, 0..3, 0..255] Of short16;
  DitherTab_g565_44 : Array[0..3, 0..3, 0..255] Of short16;
  DitherTab_b565_44 : Array[0..3, 0..3, 0..255] Of short16;

  DitherTab_r332_44 : Array[0..3, 0..3, 0..255] Of char8;
  DitherTab_g332_44 : Array[0..3, 0..3, 0..255] Of char8;
  DitherTab_b332_44 : Array[0..3, 0..3, 0..255] Of char8;

Procedure Dither_SetupMatrices;

Var
  i, x, y : LongInt;

Begin  
  For y := 0 To 3 Do
    For x := 0 To 3 Do
      For i := 0 To 255 Do
      Begin
        If (DitherMatrix_44[x, y] < (i And $7)) And (i < (256 - 8)) Then
	Begin
	  DitherTab_r565_44[x, y, i] := ((i + 8) And $f8) Shl 8;
	  DitherTab_r332_44[x, y, i] := ((i + 8) And $e0);
	End
	Else
	Begin
	  DitherTab_r565_44[x, y, i] := (i And $f8) Shl 8;
	  DitherTab_r332_44[x, y, i] := i And $e0;
	End;
        If (DitherMatrix_44[x, y] < ((i And $3) Shl 1)) And (i < (256 - 4)) Then
	Begin
	  DitherTab_g565_44[x, y, i] := (((i + 4) And $fc) Shl 8) Shr 5;
	  DitherTab_g332_44[x, y, i] := ((i + 4) And $e0) Shr 3;
	End
	Else
	Begin
	  DitherTab_g565_44[x, y, i] := ((i And $fc) Shl 8) Shr 5;
	  DitherTab_g332_44[x, y, i] := (i And $e0) Shr 3;
	End;
        If (DitherMatrix_44[x, y] < (i And $7)) And (i < (256 - 8)) Then
	Begin
	  DitherTab_b565_44[x, y, i] := (((i + 8) And $f8) Shl 16) Shr 19;
	  DitherTab_b332_44[x, y, i] := ((i + 8) Shr 6) And $3;
	End
	Else
	Begin
	  DitherTab_b565_44[x, y, i] := ((i And $f8) Shl 16) Shr 19;
	  DitherTab_b332_44[x, y, i] := (i Shr 6) And $3;
	End;
      End;
End;
