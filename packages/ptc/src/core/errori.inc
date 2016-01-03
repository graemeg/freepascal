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

constructor TPTCError.Create;
begin
  FMessage := '';
end;

constructor TPTCError.Create(const AMessage: String);
begin
  FMessage := AMessage;
  LOG('error', Self);
end;

constructor TPTCError.Create(const AMessage: string; const AError: TPTCError);
begin
  FMessage := AMessage + #10 + AError.FMessage;
  LOG('composite error', Self);
end;

constructor TPTCError.Create(const AError: TPTCError);
begin
  FMessage := AError.FMessage;
end;

destructor TPTCError.Destroy;
begin
  inherited Destroy;
end;

procedure TPTCError.Assign(const AError: TPTCError);
begin
  FMessage := AError.FMessage;
end;

function TPTCError.Equals(const AError: TPTCError): Boolean;
begin
  Equals := (FMessage = AError.FMessage);
end;

procedure TPTCError.Report;
{$IFDEF Win32}
var
  txt: AnsiString;
{$ENDIF Win32}
{$IFDEF WinCE}
var
  txt: WideString;
{$ENDIF WinCE}
begin
  LOG('error report', Self);
  {$IFDEF GO32V2}
  RestoreTextMode;
  Writeln(stderr, 'error: ', FMessage);
  {$ENDIF GO32V2}

  {$IFDEF Win32}
//  Win32Cursor_resurrect;
  txt := FMessage;
  MessageBox(0, PChar(txt), 'Error', MB_OK or MB_ICONERROR or MB_SETFOREGROUND or MB_TOPMOST);
  {$ENDIF Win32}

  {$IFDEF WinCE}
  txt := FMessage;
  MessageBox(0, PWideChar(txt), 'Error', MB_OK or MB_ICONERROR or MB_SETFOREGROUND or MB_TOPMOST);
  {$ENDIF WinCE}

  {$IFDEF UNIX}
  Writeln(stderr, 'error: ', FMessage);
  {$ENDIF UNIX}

  Halt(1);
end;
