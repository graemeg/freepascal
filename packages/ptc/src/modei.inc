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

Type
  TPTCModeDynArray = Array Of TPTCMode;

Constructor TPTCMode.Create;

Begin
  FFormat := TPTCFormat.Create;
  FWidth := 0;
  FHeight := 0;
  FValid := False;
End;

Constructor TPTCMode.Create(AWidth, AHeight : Integer; Const AFormat : TPTCFormat);

Begin
  FFormat := TPTCFormat.Create(AFormat);
  FWidth := AWidth;
  FHeight := AHeight;
  FValid := True;
End;

Constructor TPTCMode.Create(Const mode : TPTCMode);

Begin
  FFormat := TPTCFormat.Create(mode.FFormat);
  FWidth := mode.FWidth;
  FHeight := mode.FHeight;
  FValid := mode.FValid;
End;

Destructor TPTCMode.Destroy;

Begin
  FFormat.Free;
  Inherited Destroy;
End;

Procedure TPTCMode.Assign(Const mode : TPTCMode);

Begin
  FFormat.Assign(mode.FFormat);
  FWidth := mode.FWidth;
  FHeight := mode.FHeight;
  FValid := mode.FValid;
End;

Function TPTCMode.Equals(Const mode : TPTCMode) : Boolean;

Begin
  Result := (FValid = mode.FValid) And
            (FWidth = mode.FWidth) And
            (FHeight = mode.FHeight) And
             FFormat.Equals(mode.FFormat);
End;
