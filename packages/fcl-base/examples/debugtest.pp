{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2003 by the Free Pascal development team

    Interactive test for debugserver.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
program testdebug;

uses dbugintf;

Var
 S : String;

begin
  SendMethodEnter('Program');
  Repeat
    Writeln('Enter message to send to debug server (STOP exits): ');
    Write('> ');
    Readln(S);
    SendDebugEx(S,dlError);
    If (SendError<>'') then
      Writeln('Error : ',SendError);
  Until (S='STOP');
  SendMethodExit('Program');
  SendDebug('Ending');
end.
