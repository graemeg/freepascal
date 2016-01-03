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

{Function Hermes_FormatNewEmpty : PHermesFormat;
Function Hermes_FormatNew(bits : Integer; r, g, b, a : int32;
                          indexed : Boolean) : PHermesFormat;
Procedure Hermes_FormatFree(fmt : PHermesFormat);
Function Hermes_FormatNewEx(bits : Integer; r, g, b, a : int32;
                            indexed, has_colorkey : Boolean;
                            colorkey : int32) : PHermesFormat;
Function Hermes_FormatEquals(op1, op2 : PHermesFormat) : Boolean;
Procedure Hermes_FormatCopy(source, dest : PHermesFormat);}

Function Hermes_FormatNewEmpty : PHermesFormat;

Var
  tmp : PHermesFormat;

Begin
  tmp := malloc(SizeOf(THermesFormat));
  If tmp = Nil Then
  Begin
    Hermes_FormatNewEmpty := Nil;
    Exit;
  End;
  tmp^.bits := 0;
  tmp^.indexed := False;
  tmp^.r := 0;
  tmp^.g := 0;
  tmp^.b := 0;
  tmp^.a := 0;
  tmp^.has_colorkey := False;
  tmp^.colorkey := 0;
  Hermes_FormatNewEmpty := tmp;
End;

Function Hermes_FormatNew(bits : Integer; r, g, b, a : int32;
                          indexed : Boolean) : PHermesFormat;

Var
  tmp : PHermesFormat;

Begin
  If indexed And (bits <> 8) Then
  Begin
    Hermes_FormatNew := Nil;
    Exit;
  End;
  tmp := malloc(SizeOf(THermesFormat));
  If tmp = Nil Then
  Begin
    Hermes_FormatNew := Nil;
    Exit;
  End;
  tmp^.bits := bits;
  tmp^.r := r;
  tmp^.g := g;
  tmp^.b := b;
  tmp^.a := a;
  tmp^.indexed := indexed;
  tmp^.has_colorkey := False;
  tmp^.colorkey := 0;
  Hermes_FormatNew := tmp;
End;

Procedure Hermes_FormatFree(fmt : PHermesFormat);

Begin
  If fmt <> Nil Then
    free(fmt);
End;

Function Hermes_FormatNewEx(bits : Integer; r, g, b, a : int32;
                            indexed, has_colorkey : Boolean;
                            colorkey : int32) : PHermesFormat;

Var
  tmp : PHermesFormat;

Begin
  If indexed And (bits <> 8) Then
  Begin
    Hermes_FormatNewEx := Nil;
    Exit;
  End;
  tmp := malloc(SizeOf(THermesFormat));
  If tmp = Nil Then
  Begin
    Hermes_FormatNewEx := Nil;
    Exit;
  End;
  tmp^.bits := bits;
  tmp^.r := r;
  tmp^.g := g;
  tmp^.b := b;
  tmp^.a := a;
  tmp^.indexed := indexed;
  tmp^.has_colorkey := has_colorkey;
  tmp^.colorkey := colorkey;
  Hermes_FormatNewEx := tmp;
End;

Function Hermes_FormatEquals(op1, op2 : PHermesFormat) : Boolean;

Begin
  Hermes_FormatEquals := ((op1^.indexed = op2^.indexed) And
                          (op1^.bits = op2^.bits) And
                          (op1^.r = op2^.r) And
                          (op1^.g = op2^.g) And
                          (op1^.b = op2^.b) And
                          (op1^.a = op2^.a) And
                          (op1^.has_colorkey = op2^.has_colorkey) And
                          ((op1^.has_colorkey = False) Or
                           (op1^.colorkey = op2^.colorkey)));
End;

Procedure Hermes_FormatCopy(source, dest : PHermesFormat);

Begin
  Move(source^, dest^, SizeOf(THermesFormat));
End;
