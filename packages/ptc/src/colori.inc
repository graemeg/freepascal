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

Constructor TPTCColor.Create;

Begin
  FIndexed := False;
  FDirect  := False;
  FIndex   := 0;
  FRed     := 0;
  FGreen   := 0;
  FBlue    := 0;
  FAlpha   := 1;
End;

Constructor TPTCColor.Create(AIndex : Integer);

Begin
  FIndexed := True;
  FDirect  := False;
  FIndex   := AIndex;
  FRed     := 0;
  FGreen   := 0;
  FBlue    := 0;
  FAlpha   := 1;
End;

Constructor TPTCColor.Create(ARed, AGreen, ABlue : Single; AAlpha : Single = 1);

Begin
  FIndexed := False;
  FDirect  := True;
  FIndex   := 0;
  FRed     := ARed;
  FGreen   := AGreen;
  FBlue    := ABlue;
  FAlpha   := AAlpha;
End;

Constructor TPTCColor.Create(Const AColor : TPTCColor);

Begin
  FIndex   := AColor.FIndex;
  FRed     := AColor.FRed;
  FGreen   := AColor.FGreen;
  FBlue    := AColor.FBlue;
  FAlpha   := AColor.FAlpha;
  FDirect  := AColor.FDirect;
  FIndexed := AColor.FIndexed;
End;

Procedure TPTCColor.Assign(Const AColor : TPTCColor);

Begin
  FIndex   := AColor.FIndex;
  FRed     := AColor.FRed;
  FGreen   := AColor.FGreen;
  FBlue    := AColor.FBlue;
  FAlpha   := AColor.FAlpha;
  FDirect  := AColor.FDirect;
  FIndexed := AColor.FIndexed;
End;

Function TPTCColor.Equals(Const AColor : TPTCColor) : Boolean;

Begin
  Result := (FIndexed = AColor.FIndexed) And
            (FDirect  = AColor.FDirect) And
            (FIndex   = AColor.FIndex) And
            (FRed     = AColor.FRed) And
            (FGreen   = AColor.FGreen) And
            (FBlue    = AColor.FBlue) And
            (FAlpha   = AColor.FAlpha);
End;
