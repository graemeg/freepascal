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

Constructor TPTCBaseConsole.Create;

Begin
  FReleaseEnabled := False;
End;

Function TPTCBaseConsole.KeyPressed : Boolean;

Var
  k, kpeek : TPTCEvent;

Begin
  k := Nil;
  Try
    Repeat
      kpeek := PeekEvent(False, [PTCKeyEvent]);
      If kpeek = Nil Then
        Exit(False);
      If FReleaseEnabled Or (kpeek As TPTCKeyEvent).Press Then
        Exit(True);
      NextEvent(k, False, [PTCKeyEvent]);
    Until False;
  Finally
    k.Free;
  End;
End;

Procedure TPTCBaseConsole.ReadKey(Var AKey : TPTCKeyEvent);

Var
  ev : TPTCEvent;

Begin
  ev := AKey;
  Try
    Repeat
      NextEvent(ev, True, [PTCKeyEvent]);
    Until FReleaseEnabled Or (ev As TPTCKeyEvent).Press;
  Finally
    AKey := ev As TPTCKeyEvent;
  End;
End;

Function TPTCBaseConsole.PeekKey(Var AKey : TPTCKeyEvent) : Boolean;

Begin
  If KeyPressed Then
  Begin
    ReadKey(AKey);
    Result := True;
  End
  Else
    Result := False;
End;

Procedure TPTCBaseConsole.ReadKey;

Var
  k : TPTCKeyEvent;

Begin
  k := TPTCKeyEvent.Create;
  Try
    ReadKey(k);
  Finally
    k.Free;
  End;
End;
