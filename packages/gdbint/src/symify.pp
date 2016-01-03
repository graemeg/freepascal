{
    Copyright (c) 1998 by Peter Vreman

    Translate backtrace addresses into file and line info

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
program symify;
uses GDBInt;

var
  gdb : tgdbinterface;


procedure processlog(const fn:string);
var
  t    : text;
  hs,s : string;
  code : word;
  i,
  addr : longint;
  sym  : tsyminfo;
begin
  assign(t,fn);
  {$I-}
   reset(t);
  {$I+}
  if ioresult<>0 then
   exit;
  while not eof(t) do
   begin
     readln(t,s);
     i:=pos('0x',s);
     if i=3 then
      begin
        hs:='$'+Copy(s,5,8);
        Val(hs,addr,code);
        if code=0 then
         begin
           gdb.GetAddrSymInfo(addr,sym);
           Write(Copy(s,1,12));
           if assigned(sym.funcname) then
             write(' in ',sym.funcname,'+',sym.offset);
           if assigned(sym.fname) then
             writeln(' ',sym.fname,':',sym.line)
           else
             writeln;
         end
        else
         writeln(s);
      end
     else
      writeln(s);
   end;
  close(t);
end;


begin
  if paramcount<2 then
   begin
     writeln('usage: symify <log> <file>');
     halt(1);
   end;
  gdb.init;
  writeln('loading ',paramstr(2));
  gdb.gdb_command('file '+paramstr(2));
  writeln('parsing ',paramstr(1));
  processlog(paramstr(1));
  gdb.done;
end.
