{
    Free Pascal port of the OpenPTC C++ library.
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
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

Function TPTCKeyEvent.GetType : TPTCEventType;

Begin
  Result := PTCKeyEvent;
End;

Constructor TPTCKeyEvent.Create;

Begin
  FCode    := Integer(PTCKEY_UNDEFINED);
  FUnicode := -1;
  FAlt     := False;
  FShift   := False;
  FControl := False;
  FPress   := True;
End;

Constructor TPTCKeyEvent.Create(ACode : Integer);

Begin
  FCode    := ACode;
  FUnicode := -1;
  FAlt     := False;
  FShift   := False;
  FControl := False;
  FPress   := True;
End;

Constructor TPTCKeyEvent.Create(ACode, AUnicode : Integer);

Begin
  FCode    := ACode;
  FUnicode := AUnicode;
  FAlt     := False;
  FShift   := False;
  FControl := False;
  FPress   := True;
End;

Constructor TPTCKeyEvent.Create(ACode, AUnicode : Integer; APress : Boolean);

Begin
  FCode    := ACode;
  FUnicode := AUnicode;
  FAlt     := False;
  FShift   := False;
  FControl := False;
  FPress   := APress;
End;

Constructor TPTCKeyEvent.Create(ACode : Integer; AAlt, AShift, AControl : Boolean);

Begin
  FCode    := ACode;
  FUnicode := -1;
  FAlt     := AAlt;
  FShift   := AShift;
  FControl := AControl;
  FPress   := True;
End;

Constructor TPTCKeyEvent.Create(ACode : Integer; AAlt, AShift, AControl, APress : Boolean);

Begin
  FCode    := ACode;
  FUnicode := -1;
  FAlt     := AAlt;
  FShift   := AShift;
  FControl := AControl;
  FPress   := APress;
End;

Constructor TPTCKeyEvent.Create(ACode, AUnicode : Integer; AAlt, AShift, AControl : Boolean);

Begin
  FCode    := ACode;
  FUnicode := AUnicode;
  FAlt     := AAlt;
  FShift   := AShift;
  FControl := AControl;
  FPress   := True;
End;

Constructor TPTCKeyEvent.Create(ACode, AUnicode : Integer;
                                AAlt, AShift, AControl, APress : Boolean);

Begin
  FCode    := ACode;
  FUnicode := AUnicode;
  FAlt     := AAlt;
  FShift   := AShift;
  FControl := AControl;
  FPress   := APress;
End;

Constructor TPTCKeyEvent.Create(Const AKey : TPTCKeyEvent);

Begin
  FCode    := AKey.Code;
  FUnicode := AKey.Unicode;
  FAlt     := AKey.Alt;
  FShift   := AKey.Shift;
  FControl := AKey.Control;
  FPress   := AKey.Press;
End;

Procedure TPTCKeyEvent.Assign(Const AKey : TPTCKeyEvent);

Begin
  FCode    := AKey.Code;
  FUnicode := AKey.Unicode;
  FAlt     := AKey.Alt;
  FShift   := AKey.Shift;
  FControl := AKey.Control;
  FPress   := AKey.Press;
End;

Function TPTCKeyEvent.Equals(Const AKey : TPTCKeyEvent) : Boolean;

Begin
  Result := (FCode    = AKey.FCode) And
            (FUnicode = AKey.FUnicode) And
            (FAlt     = AKey.FAlt) And
            (FShift   = AKey.FShift) And
            (FControl = AKey.FControl) And
            (FPress   = AKey.FPress);
End;

Function TPTCKeyEvent.GetRelease : Boolean;

Begin
  Result := Not FPress;
End;
