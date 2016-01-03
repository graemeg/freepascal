{
    Free Pascal port of the OpenPTC C++ library.
    Copyright (C) 2001-2006  Nikolay Nikolov (nickysn@users.sourceforge.net)
    Original C++ version by Glenn Fiedler (ptc@gaffer.org)

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

Constructor TPTCFormat.Create;

Begin
  { defaults }
  FFormat.r := 0;
  FFormat.g := 0;
  FFormat.b := 0;
  FFormat.a := 0;
  FFormat.bits := 0;
  FFormat.indexed := False;

  { initialize hermes }
  If Not Hermes_Init Then
    Raise TPTCError.Create('could not initialize hermes');
End;

Constructor TPTCFormat.Create(ABits : Integer);

Begin
  { check bits per pixel }
  If ABits <> 8 Then
    Raise TPTCError.Create('unsupported bits per pixel');

  { indexed color }
  FFormat.r := 0;
  FFormat.g := 0;
  FFormat.b := 0;
  FFormat.a := 0;
  FFormat.bits := ABits;
  FFormat.indexed := True;

  { initialize hermes }
  If Not Hermes_Init Then
    Raise TPTCError.Create('could not initialize hermes');
End;

Constructor TPTCFormat.Create(ABits : Integer;
                              ARedMask, AGreenMask, ABlueMask : Uint32;
                              AAlphaMask : Uint32 = 0);

Begin
  { check bits per pixel }
  If ((ABits And 7) <> 0) Or (ABits <= 0) Or (ABits > 32) Then
    Raise TPTCError.Create('unsupported bits per pixel');

  { direct color }
  FFormat.r := ARedMask;
  FFormat.g := AGreenMask;
  FFormat.b := ABlueMask;
  FFormat.a := AAlphaMask;
  FFormat.bits := ABits;
  FFormat.indexed := False;

  { initialize hermes }
  If Not Hermes_Init Then
    Raise TPTCError.Create('could not initialize hermes');
End;

Constructor TPTCFormat.Create(Const format : TPTCFormat);

Begin
  { initialize hermes }
  If Not Hermes_Init Then
    Raise TPTCError.Create('could not initialize hermes');

  Hermes_FormatCopy(@format.FFormat, @FFormat)
End;

{$INFO TODO: check what happens if Hermes_Init blows up in the constructor...}
Destructor TPTCFormat.Destroy;

Begin
  Hermes_Done;
  Inherited Destroy;
End;

Procedure TPTCFormat.Assign(Const format : TPTCFormat);

Begin
  If Self = format Then
    Exit;
  Hermes_FormatCopy(@format.Fformat, @Fformat);
End;

Function TPTCFormat.Equals(Const format : TPTCFormat) : Boolean;

Begin
  Result := Hermes_FormatEquals(@format.FFormat, @FFormat);
End;

Function TPTCFormat.GetDirect : Boolean;

Begin
  Result := Not FFormat.indexed;
End;

Function TPTCFormat.GetBytes : Integer;

Begin
  Result := FFormat.bits Shr 3;
End;
