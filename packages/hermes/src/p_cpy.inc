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
   C straight copy routines for the HERMES library
   Copyright (c) 1998 Christian Nentwich (c.nentwich@cs.ucl.ac.uk)
   This source code is licensed under the GNU LGPL

   Please refer to the file COPYING.LIB contained in the distribution for
   licensing conditions
}

Procedure CopyP_4byte(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Begin
  Move(source^, dest^, count Shl 2);
End;

Procedure CopyP_3byte(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Begin
  Move(source^, dest^, count * 3);
End;

Procedure CopyP_2byte(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Begin
  Move(source^, dest^, count Shl 1);
End;

Procedure CopyP_1byte(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Begin
  Move(source^, dest^, count);
End;

Procedure CopyP_4byte_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  x : DWord;

Begin
  x := 0;
  Repeat
    Pint32(dest)^ := (Pint32(source)+(x Shr 16))^;

    Inc(x, inc_source);
    Inc(dest, 4);
    Dec(count);
  Until count = 0;
End;

{ TODO: Optimise }
Procedure CopyP_3byte_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  x : DWord;

Begin
  x := 0;
  Repeat
    dest[R_24] := source[R_24];
    dest[G_24] := source[G_24];
    dest[B_24] := source[B_24];

    Inc(x, inc_source);
    Inc(source, 3*(x Shr 16));
    x := x And $FFFF;
    Inc(dest, 3);
    Dec(count);
  Until count = 0;
End;

Procedure CopyP_2byte_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  x, c : DWord;
  p : int32;

Begin
  x := 0;
  { Alignment mod 4 }
  If (PtrUInt(dest) And 3) <> 0 Then
  Begin
    Pshort16(dest)^ := (Pshort16(source) + (x Shr 16))^;
    Inc(x, inc_source);
    Inc(dest, 2);
    Dec(count);
  End;

  c := count Shr 1;

  While c <> 0 Do
  Begin
    Dec(c);
    { TODO: make fast :) }
    p := (Pshort16(source) + (x Shr 16))^; Inc(x, inc_source);
    p := p Or ((Pshort16(source) + (x Shr 16))^ Shl 16);
    Inc(x, inc_source);

    Pint32(dest)^ := p;
    Inc(dest, 4);
  End;

  If (count And 1) <> 0 Then
    Pshort16(dest)^ := (Pshort16(source) + (x Shr 16))^;
End;

Procedure CopyP_1byte_S(source, dest : Pchar8; count, inc_source : DWord); CDecl;

Var
  x, c : DWord;
  p : int32;

Begin
  x := 0;
  { Alignment mod 4 }
  While (PtrUInt(dest) And 3) <> 0 Do
  Begin
    dest^ := (source + (x Shr 16))^;
    Inc(x, inc_source);
    Inc(dest); Dec(count);
    If count = 0 Then
      Exit;
  End;

  { Write blocks of four pixels }
  c := count Shr 2;
  While c <> 0 Do
  Begin
    Dec(c);
    p := (source + (x Shr 16))^; Inc(x, inc_source);
    p := p Or ((source + (x Shr 16))^ Shl 8); Inc(x, inc_source);
    p := p Or ((source + (x Shr 16))^ Shl 16); Inc(x, inc_source);
    p := p Or ((source + (x Shr 16))^ Shl 24); Inc(x, inc_source);

    Pint32(dest)^ := p;
    Inc(dest, 4);
  End;

  { Write up to three trailing pixels }
  c := count And $3;
  While c <> 0 Do
  Begin
    Dec(c);
    dest^ := (source + (x Shr 16))^;
    Inc(x, inc_source);
    Inc(dest);
  End;
End;
