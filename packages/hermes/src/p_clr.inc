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
   C surface clearing routines for the HERMES library
   Copyright (c) 1998 Christian Nentwich (c.nentwich@cs.ucl.ac.uk)
   This source code is licensed under the GNU LGPL

   Please refer to the file COPYING.LIB contained in the distribution for
   licensing conditions
}

Procedure ClearP_32(iface : PHermesClearInterface); CDecl;

Var
  count : DWord;
  value : int32;
  dest : Pchar8;

Begin
  value := iface^.value;
  dest := iface^.dest;
  Repeat
    count := iface^.width;
    Repeat
      Pint32(dest)^ := value;
      Inc(dest, 4);
      Dec(count);
    Until count = 0;
    Inc(dest, iface^.add);
    Dec(iface^.height);
  Until iface^.height = 0;
End;

Procedure ClearP_24(iface : PHermesClearInterface); CDecl;

Var
  p_value : Pchar8;
  count : DWord;
  dest : Pchar8;

Begin
  p_value := @iface^.value;
  dest := iface^.dest;
  Repeat
    count := iface^.width;
    Repeat
      (dest + R_24)^ := (p_value + B_32)^;
      (dest + G_24)^ := (p_value + G_32)^;
      (dest + B_24)^ := (p_value + B_32)^;

      Inc(dest, 3);
      Dec(count);
    Until count = 0;

    Inc(dest, iface^.add);
    Dec(iface^.height);
  Until iface^.height = 0;
End;

Procedure ClearP_16(iface : PHermesClearInterface); CDecl;

Var
  value32 : DWord;
  countshifted, count : DWord;
  dest : Pchar8;

Begin
  value32 := (iface^.value Shl 16) Or (iface^.value And $ffff);
  dest := iface^.dest;
  Repeat
    count := iface^.width;

    { Align destination }
    If (PtrUInt(dest) And $3) <> 0 Then
    Begin
      Pshort16(dest)^ := iface^.value;
      Inc(dest, 2);
      Dec(count);
    End;

    countshifted := count Shr 1;

    While countshifted <> 0 Do
    Begin
      Dec(countshifted);
      Pint32(dest)^ := value32;
      Inc(dest, 4);
    End;

    If (count And 1) <> 0 Then
    Begin
      Pshort16(dest)^ := iface^.value;
      Inc(dest, 2);
    End;

    Inc(dest, iface^.add);
    Dec(iface^.height);
  Until iface^.height = 0;
End;

{$GOTO ON}

Procedure ClearP_8(iface : PHermesClearInterface); CDecl;

Label
  yloop;

Var
  count, shiftcount : DWord;
  value32 : int32;
  value : char8;
  dest : Pchar8;

Begin
  dest := iface^.dest;

  value := iface^.value And $ff;
  value32 := (value Shl 24) Or (value Shl 16) Or (value Shl 8) Or value;

  Repeat
    count := iface^.width;

    While (PtrUInt(dest) And $3) <> 0 Do    { Align to dword boundary }
    Begin
      dest^ := value;
      Inc(dest);
      Dec(count);
      If count = 0 Then
        Goto yloop;                { GOTO's are nice ;) }
    End;

    shiftcount := count Shr 2;

    While shiftcount <> 0 Do
    Begin
      Dec(shiftcount);
      Pint32(dest)^ := value32;
      Inc(dest, 4);
    End;

    count := count And $3;
    While count <> 0 Do
    Begin
      Dec(count);
      dest^ := value;
      Inc(dest);
    End;

yloop:
    Inc(dest, iface^.add);
    Dec(iface^.height);
  Until iface^.height = 0;
End;
