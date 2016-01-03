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

Function TPTCMouseEvent.GetType : TPTCEventType;

Begin
  Result := PTCMouseEvent;
End;

Constructor TPTCMouseEvent.Create(AX, AY, ADeltaX, ADeltaY : Integer; AButtonState : TPTCMouseButtonState);

Begin
  FX := AX;
  FY := AY;
  FDeltaX := ADeltaX;
  FDeltaY := ADeltaY;
  FButtonState := AButtonState;
End;

Constructor TPTCMouseButtonEvent.Create(AX, AY, ADeltaX, ADeltaY : Integer; AButtonState : TPTCMouseButtonState; APress : Boolean; AButton : TPTCMouseButton);

Begin
  If APress Xor (AButton In AButtonState) Then
    Raise TPTCError.Create('Invalid ButtonState');

  Inherited Create(AX, AY, ADeltaX, ADeltaY, AButtonState);

  FPress := APress;
  FButton := AButton;
End;

Function TPTCMouseButtonEvent.GetRelease : Boolean;

Begin
  Result := Not FPress;
End;
