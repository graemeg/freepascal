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

Constructor TPTCError.Create;

Begin
  FMessage := '';
End;

Constructor TPTCError.Create(Const AMessage : String);

Begin
  FMessage := AMessage;
  LOG('error', Self);
End;

Constructor TPTCError.Create(Const AMessage : String; Const AError : TPTCError);

Begin
  FMessage := AMessage + #10 + AError.FMessage;
  LOG('composite error', Self);
End;

Constructor TPTCError.Create(Const AError : TPTCError);

Begin
  FMessage := AError.FMessage;
End;

Destructor TPTCError.Destroy;

Begin
  Inherited Destroy;
End;

Procedure TPTCError.Assign(Const AError : TPTCError);

Begin
  FMessage := AError.FMessage;
End;

Function TPTCError.Equals(Const AError : TPTCError) : Boolean;

Begin
  Equals := (FMessage = AError.FMessage);
End;

Procedure TPTCError.Report;

{$IFDEF Win32}
Var
  txt : AnsiString;
{$ENDIF Win32}

{$IFDEF WinCE}
Var
  txt : WideString;
{$ENDIF WinCE}

Begin
  LOG('error report', Self);
  {$IFDEF GO32V2}
  RestoreTextMode;
  Writeln(stderr, 'error: ', FMessage);
  {$ENDIF GO32V2}

  {$IFDEF Win32}
  Win32Cursor_resurrect;
  txt := FMessage;
  MessageBox(0, PChar(txt), 'Error', MB_OK Or MB_ICONERROR Or MB_SETFOREGROUND Or MB_TOPMOST);
  {$ENDIF Win32}

  {$IFDEF WinCE}
  txt := FMessage;
  MessageBox(0, PWideChar(txt), 'Error', MB_OK Or MB_ICONERROR Or MB_SETFOREGROUND Or MB_TOPMOST);
  {$ENDIF WinCE}

  {$IFDEF UNIX}
  Writeln(stderr, 'error: ', FMessage);
  {$ENDIF UNIX}

  Halt(1);
End;
