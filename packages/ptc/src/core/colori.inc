{
    Free Pascal port of the OpenPTC C++ library.
    Copyright (C) 2001-2006  Nikolay Nikolov (nickysn@users.sourceforge.net)
    Original C++ version by Glenn Fiedler (ptc@gaffer.org)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version
    with the following modification:

    As a special exception, the copyright holders of this library give you
    permission to link this library with independent modules to produce an
    executable, regardless of the license terms of these independent modules,and
    to copy and distribute the resulting executable under terms of your choice,
    provided that you also meet, for each linked independent module, the terms
    and conditions of the license of that module. An independent module is a
    module which is not derived from or based on this library. If you modify
    this library, you may extend this exception to your version of the library,
    but you are not obligated to do so. If you do not wish to do so, delete this
    exception statement from your version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
}

constructor TPTCColor.Create;
begin
  FIndexed := False;
  FDirect  := False;
  FIndex   := 0;
  FRed     := 0;
  FGreen   := 0;
  FBlue    := 0;
  FAlpha   := 1;
end;

constructor TPTCColor.Create(AIndex: Integer);
begin
  FIndexed := True;
  FDirect  := False;
  FIndex   := AIndex;
  FRed     := 0;
  FGreen   := 0;
  FBlue    := 0;
  FAlpha   := 1;
end;

constructor TPTCColor.Create(ARed, AGreen, ABlue: Single; AAlpha: Single = 1);
begin
  FIndexed := False;
  FDirect  := True;
  FIndex   := 0;
  FRed     := ARed;
  FGreen   := AGreen;
  FBlue    := ABlue;
  FAlpha   := AAlpha;
end;

constructor TPTCColor.Create(const AColor: TPTCColor);
begin
  FIndex   := AColor.FIndex;
  FRed     := AColor.FRed;
  FGreen   := AColor.FGreen;
  FBlue    := AColor.FBlue;
  FAlpha   := AColor.FAlpha;
  FDirect  := AColor.FDirect;
  FIndexed := AColor.FIndexed;
end;

procedure TPTCColor.Assign(const AColor: TPTCColor);
begin
  FIndex   := AColor.FIndex;
  FRed     := AColor.FRed;
  FGreen   := AColor.FGreen;
  FBlue    := AColor.FBlue;
  FAlpha   := AColor.FAlpha;
  FDirect  := AColor.FDirect;
  FIndexed := AColor.FIndexed;
end;

function TPTCColor.Equals(const AColor: TPTCColor): Boolean;
begin
  Result := (FIndexed = AColor.FIndexed) and
            (FDirect  = AColor.FDirect) and
            (FIndex   = AColor.FIndex) and
            (FRed     = AColor.FRed) and
            (FGreen   = AColor.FGreen) and
            (FBlue    = AColor.FBlue) and
            (FAlpha   = AColor.FAlpha);
end;
