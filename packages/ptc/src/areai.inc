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

Constructor TPTCArea.Create(ALeft, ATop, ARight, ABottom : Integer);

Begin
  If ALeft < ARight Then
  Begin
    FLeft := ALeft;
    FRight := ARight;
  End
  Else
  Begin
    FLeft := ARight;
    FRight := ALeft;
  End;
  If ATop < ABottom Then
  Begin
    FTop := ATop;
    FBottom := ABottom;
  End
  Else
  Begin
    FTop := ABottom;
    FBottom := ATop;
  End;
End;

Constructor TPTCArea.Create;

Begin
  FLeft   := 0;
  FRight  := 0;
  FTop    := 0;
  FBottom := 0;
End;

Constructor TPTCArea.Create(Const AArea : TPTCArea);

Begin
  FLeft   := AArea.FLeft;
  FTop    := AArea.FTop;
  FRight  := AArea.FRight;
  FBottom := AArea.FBottom;
End;

Procedure TPTCArea.Assign(Const AArea : TPTCArea);

Begin
  FLeft   := AArea.FLeft;
  FTop    := AArea.FTop;
  FRight  := AArea.FRight;
  FBottom := AArea.FBottom;
End;

Function TPTCArea.Equals(Const AArea : TPTCArea) : Boolean;

Begin
  Result := (FLeft   = AArea.FLeft) And
            (FTop    = AArea.FTop) And
            (FRight  = AArea.FRight) And
            (FBottom = AArea.FBottom);
End;

Function TPTCArea.GetWidth : Integer;

Begin
  Result := FRight - FLeft;
End;

Function TPTCArea.GetHeight : Integer;

Begin
  Result := FBottom - FTop;
End;
