{
    Copyright (c) 1998 by Peter Vreman

    Small example program to the GDB

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
program testgdb;
uses gdbcon;
var
  last,s,parafile : string;
  gdb : tgdbcontroller;
begin
  gdb.init;
  if paramcount=1 then
    parafile:=paramstr(1)
  else
    parafile:='test';
  gdb.loadfile(parafile);
  Writeln('Welcome to the pascal GDB...');
  Writeln('Type "q" to exit...');
  last:='';
  repeat
    write('>');
    readln(s);
    if (s='a') then
     gdb.starttrace
    else
     if (s='s') then
      gdb.tracestep
    else
     if (s='n') then
      gdb.tracenext
    else
     if (s='q') then
      break
    else
     begin
       if s='' then
         s:=last;
       GDB.Command(s);
       GDB.WriteErrorBuf;
       GDB.WriteOutputBuf;
       last:=s;
     end;
  until false;
  gdb.done;
  Writeln('End of pascal GDB...');
end.
