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
  m_code := Integer(PTCKEY_UNDEFINED);
  m_unicode := -1;
  m_alt := False;
  m_shift := False;
  m_control := False;
  m_press := True;
End;

Constructor TPTCKeyEvent.Create(_code : Integer);

Begin
  m_code := _code;
  m_unicode := -1;
  m_alt := False;
  m_shift := False;
  m_control := False;
  m_press := True;
End;

Constructor TPTCKeyEvent.Create(_code, _unicode : Integer);

Begin
  m_code := _code;
  m_unicode := _unicode;
  m_alt := False;
  m_shift := False;
  m_control := False;
  m_press := True;
End;

Constructor TPTCKeyEvent.Create(_code, _unicode : Integer; _press : Boolean);

Begin
  m_code := _code;
  m_unicode := _unicode;
  m_alt := False;
  m_shift := False;
  m_control := False;
  m_press := _press;
End;

Constructor TPTCKeyEvent.Create(_code : Integer; _alt, _shift, _control : Boolean);

Begin
  m_code := _code;
  m_unicode := -1;
  m_alt := _alt;
  m_shift := _shift;
  m_control := _control;
  m_press := True;
End;

Constructor TPTCKeyEvent.Create(_code : Integer; _alt, _shift, _control, _press : Boolean);

Begin
  m_code := _code;
  m_unicode := -1;
  m_alt := _alt;
  m_shift := _shift;
  m_control := _control;
  m_press := _press;
End;

Constructor TPTCKeyEvent.Create(_code, _unicode : Integer; _alt, _shift, _control : Boolean);

Begin
  m_code := _code;
  m_unicode := _unicode;
  m_alt := _alt;
  m_shift := _shift;
  m_control := _control;
  m_press := True;
End;

Constructor TPTCKeyEvent.Create(_code, _unicode : Integer;
                           _alt, _shift, _control, _press : Boolean);

Begin
  m_code := _code;
  m_unicode := _unicode;
  m_alt := _alt;
  m_shift := _shift;
  m_control := _control;
  m_press := _press;
End;

Constructor TPTCKeyEvent.Create(Const key : TPTCKeyEvent);

Begin
  ASSign(key);
End;

Destructor TPTCKeyEvent.Destroy;

Begin
  Inherited Destroy;
End;

Procedure TPTCKeyEvent.Assign(Const key : TPTCKeyEvent);

Begin
  If Self = key Then
    Raise TPTCError.Create('self assignment is not allowed');

  m_code := key.code;
  m_unicode := key.unicode;
  m_alt := key.alt;
  m_shift := key.shift;
  m_control := key.control;
  m_press := key.press;
End;

Function TPTCKeyEvent.Equals(Const key : TPTCKeyEvent) : Boolean;

Begin
  Equals := (m_code = key.m_code) And (m_unicode = key.m_unicode) And
            (m_alt = key.m_alt) And (m_shift = key.m_shift) And
            (m_control = key.m_control) And (m_press = key.m_press);
End;

Function TPTCKeyEvent.GetRelease : Boolean;

Begin
  GetRelease := Not m_press;
End;
