{
    Copyright (c) 2000-2002 by Pierre Muller

    This program allows to run the Makefiles
    with the compiler running inside GDB

    GDB only stops if there is something special

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

 ****************************************************************************}

{$mode objfpc}
{ Use ansitrings for long PATH variables }
{$H+}
program fpc_with_gdb;

{
  This program uses several files :

   -- 'gdb4fpc.ini' contains the standard breakpoints (see below)

   -- 'gdb.fpc' is an optional file that can contain any other
      instruction that GDB should do before starting.
      Note that if gdb.fpc is present, no "run" command is
      inserted if gdb4fpc.ini is found
      but it can be inserted in gdb.fpc itself.

  Use EXTDEBUG conditional to get debug information.
}

uses
  sysutils,
  dos;

const
{$ifdef Unix}
  GDBExeName : String = 'gdbpas';
  GDBAltExeName = 'gdb';
  GDBIniName = '.gdbinit';
  DefaultCompilerName = 'ppc386';
  PathSep=':';
  DirSep = '/';
{$else}
  GDBExeName : String = 'gdbpas.exe';
  GDBAltExeName = 'gdb.exe';
  GDBIniName = 'gdb.ini';
  DefaultCompilerName = 'ppc386.exe';
  PathSep=';';
  DirSep = '\';
{$endif not linux}

  { If you add a gdb.fpc file in a given directory }
  { GDB will read it; this allows you to add       }
  { special tests in specific directories   PM     }
  FpcGDBIniName = 'gdb.fpc';
  GDBIniTempName : string = 'gdb4fpc.ini';


{ Dos/Windows GDB still need forward slashes }
procedure AdaptToGDB(var filename : string);
var
  i : longint;
begin
  for i:=1 to length(filename) do
    if filename[i]='\' then
      filename[i]:='/';
end;

var
   fpcgdbini : text;
   CompilerName : String;
   FullCompilerName : String;
{$ifdef linux}
   argv0 : pchar;
{$endif}
   Dir,Name,Ext : ShortString;
   GDBError,GDBExitCode,i : longint;

begin

  fsplit(paramstr(0),Dir,Name,Ext);
{$ifdef linux}
  argv0:=argv[0];
  if (argv0 <> '') then
    fsplit(argv0,Dir,Name,Ext);
{$endif}
  if (length(Name)>3) and (UpCase(Name[1])='G') then
    CompilerName:=Copy(Name,2,255)+Ext
  else
    begin
      if (Name+ext = DefaultCompilerName) then
        begin
          writeln(stderr,'Avoiding infinite recursion with ',Name+Ext,' binary');
          halt(1);
        end;
      CompilerName:=DefaultCompilerName;
    end;

  FullCompilerName:=filesearch(CompilerName,Dir+PathSep+GetEnvironmentVariable('PATH'));

  if FullCompilerName='' then
    begin
      writeln(stderr,'Unable to find ',CompilerName,' binary');
      halt(2);
    end;


  { support for info functions directly : used in makefiles }
  if (paramcount=1) and (pos('-i',Paramstr(1))=1) then
    begin
      Exec(FullCompilerName,Paramstr(1));
      exit;
    end;

  {$ifdef EXTDEBUG}
  writeln(stderr,'Using compiler "',FullCompilerName,'"');
  flush(stderr);
  {$endif}
  if fsearch(GDBIniTempName,'.')<>'' then
    begin
      Assign(fpcgdbini,GDBIniTempName);
      {$ifdef EXTDEBUG}
      writeln(stderr,'Erasing file "',GDBIniTempName,'"');
      flush(stderr);
      {$endif}
      erase(fpcgdbini);
    end;
  GDBIniTempName:=fexpand('.'+DirSep+GDBIniTempName);
  Assign(fpcgdbini,GdbIniTempName);
  {$ifdef EXTDEBUG}
  writeln(stderr,'Creating file "',GDBIniTempName,'"');
  flush(stderr);
  {$endif}
  Rewrite(fpcgdbini);

  Writeln(fpcgdbini,'set language pascal');
  Write(fpcgdbini,'set args');

  { this will not work correctly if there are " or '' inside the command line :( }
  for i:=1 to Paramcount do
    begin
      if pos(' ',Paramstr(i))>0 then
        Write(fpcgdbini,' "'+ParamStr(i)+'"')
      else
        Write(fpcgdbini,' '+ParamStr(i));
    end;
  Writeln(fpcgdbini);
  Writeln(fpcgdbini,'b SYSTEM_EXIT');
  Writeln(fpcgdbini,'cond 1 EXITCODE <> 0');
  Writeln(fpcgdbini,'set $_exitcode := -1');
  { b INTERNALERROR sometimes fails ... Don't know why. PM 2010-08-28 }
  Writeln(fpcgdbini,'info fun INTERNALERROR');
  Writeln(fpcgdbini,'b INTERNALERROR');
  Writeln(fpcgdbini,'b GENERATEERROR');
  Writeln(fpcgdbini,'b HANDLEERRORADDRFRAME');
  { This one will fail unless sysutils unit is also loaded }
  Writeln(fpcgdbini,'b RUNERRORTOEXCEPT');
  if fsearch(FpcGDBIniName,'./')<>'' then
    begin
      Writeln(fpcgdbini,'source '+FpcGDBIniName);
    end
  else
    Writeln(fpcgdbini,'run');
  Writeln(fpcgdbini,'if ($_exitcode = -1)');
  Writeln(fpcgdbini,'  echo Program not completed');
  Writeln(fpcgdbini,'else');
  Writeln(fpcgdbini,'  quit');
  Writeln(fpcgdbini,'end');
  Close(fpcgdbini);
  {$ifdef EXTDEBUG}
  writeln(stderr,'Closing file "',GDBIniTempName,'"');
  flush(stderr);
  {$endif}

  GDBExeName:=filesearch(GDBExeName,Dir+PathSep+GetEnvironmentVariable('PATH'));
  if GDBExeName='' then
    GDBExeName:=filesearch(GDBAltExeName,Dir+PathSep+GetEnvironmentVariable('PATH'));

  if GDBExeName='' then
    begin
      writeln('Unable to find ',GDBExeName,' and ',GDBAltExeName);
      halt(3);
    end;
  AdaptToGDB(CompilerName);
  AdaptToGDB(GDBIniTempName);
  {$ifdef EXTDEBUG}
  Writeln(stderr,'Starting ',GDBExeName,
{$ifdef win32}
    '--nw '+
{$endif win32}
    '--nx --command='+GDBIniTempName+' '+FullCompilerName);
  flush(stderr);
  {$endif}
   DosError:=0;
   Exec(GDBExeName,
{$ifdef win32}
    '--nw '+
{$endif win32}
    '--nx --command='+GDBIniTempName+' '+FullCompilerName);
  GDBError:=DosError;
  GDBExitCode:=DosExitCode;
  if (GDBError<>0) or (GDBExitCode<>0) then
    begin
      Writeln('Error running GDB');
      if (GDBError<>0) then
        Writeln('DosError = ',GDBError);
      if (GDBExitCode<>0) then
        Writeln('DosExitCode = ',GDBExitCode);
      if GDBExitCode<>0 then
        RunError(GDBExitCode)
      else
        RunError(GDBError);
    end
  else
    Erase(fpcgdbini);
end.
