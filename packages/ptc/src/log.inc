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

{$IFNDEF WinCE}
Const
  LOG_filename = 'ptcpas.log';
{$ELSE WinCE}
Function LOG_filename : WideString;

Var
  RequiredBufferLength : DWord;
  ReturnedPathLength : DWord;
  TempPathBuf : PWideChar;
  dummy : Byte;

Begin
  RequiredBufferLength := GetTempPathW(0, @dummy);
  TempPathBuf := GetMem(RequiredBufferLength * SizeOf(WideChar));
  Try
    ReturnedPathLength := GetTempPathW(RequiredBufferLength, TempPathBuf);

    If ReturnedPathLength > RequiredBufferLength Then
    Begin
      { The temp path length increased between 2 consecutive calls to GetTempPath?! }
      Result := '';
      Exit;
    End;

    Result := TempPathBuf;
    Result := Result + 'ptcpas.log';
  Finally
    FreeMem(TempPathBuf);
  End;
End;
{$ENDIF WinCE}

Var
  LOG_create : Boolean = True;
  LOG_enabled : Boolean =
  {$IFDEF DEBUG}
    True;
  {$ELSE DEBUG}
    False;
  {$ENDIF DEBUG}
  LOG_file : Text;

Procedure LOG_open;

Begin
  AssignFile(LOG_file, LOG_filename);
  If LOG_create Then
  Begin
    Rewrite(LOG_file);
    Writeln(LOG_file, '[log start]');
    LOG_create := False;
  End
  Else
    Append(LOG_file);
End;

Procedure LOG_close;

Begin
  CloseFile(LOG_file);
End;

Procedure LOG(Const message : String);

Begin
  If Not LOG_enabled Then
    Exit;
  LOG_open;
  Writeln(LOG_file, message);
  LOG_close;
End;

Procedure LOG(Const message : String; data : Boolean);

Begin
  If Not LOG_enabled Then
    Exit;
  LOG_open;
  Write(LOG_file, message, ' = ');
  If data Then
    Writeln(LOG_file, 'true')
  Else
    Writeln(LOG_file, 'false');
  LOG_close;
End;

Procedure LOG(Const message : String; data : Integer);

Begin
  If Not LOG_enabled Then
    Exit;
  LOG_open;
  Writeln(LOG_file, message, ' = ', data);
  LOG_close;
End;

Procedure LOG(Const message : String; data : DWord);

Begin
  If Not LOG_enabled Then
    Exit;
  LOG_open;
  Writeln(LOG_file, message, ' = ', data);
  LOG_close;
End;

Procedure LOG(Const message : String; data : Int64);

Begin
  If Not LOG_enabled Then
    Exit;
  LOG_open;
  Writeln(LOG_file, message, ' = ', data);
  LOG_close;
End;

Procedure LOG(Const message : String; data : QWord);

Begin
  If Not LOG_enabled Then
    Exit;
  LOG_open;
  Writeln(LOG_file, message, ' = ', data);
  LOG_close;
End;

Procedure LOG(Const message : String; data : Single);

Begin
  If Not LOG_enabled Then
    Exit;
  LOG_open;
  Writeln(LOG_file, message, ' = ', data);
  LOG_close;
End;

Procedure LOG(Const message : String; data : Double);

Begin
  If Not LOG_enabled Then
    Exit;
  LOG_open;
  Writeln(LOG_file, message, ' = ', data);
  LOG_close;
End;

Procedure LOG(Const message : String; Const data : String);

Begin
  If Not LOG_enabled Then
    Exit;
  LOG_open;
  Writeln(LOG_file, message, ' = ', data);
  LOG_close;
End;

Procedure LOG(Const message : String; data : TPTCFormat);

Begin
  If Not LOG_enabled Then
    Exit;
  LOG_open;
  Write(LOG_file, message, ' = Format(');
  If data = Nil Then
    Write(LOG_file, 'NIL')
  Else
  Begin
    Write(LOG_file, data.bits:2);
    If data.direct Then
    Begin
      Write(LOG_file, ',$', HexStr(data.r, 8), ',$', HexStr(data.g, 8), ',$', HexStr(data.b, 8));
      If data.a <> 0 Then
        Write(LOG_file, ',$', HexStr(data.a, 8));
    End;
  End;
  Writeln(LOG_file, ')');
  LOG_close;
End;

Procedure LOG(Const message : String; data : TPTCError);

Begin
  If Not LOG_enabled Then
    Exit;
  LOG_open;
  Writeln(LOG_file, message, ': ', data.message);
  LOG_close;
End;
