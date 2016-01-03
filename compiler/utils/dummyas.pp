{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2007 by Pierre Muller
    member of the Free Pascal development team.

    Dummy assembler program to be able to easily test
    all FPC targets even without cross tools.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

program dummyas;

var
  assembler_name : string;
  object_name : string;
  ofile : text;

function RemoveSuffix(const st : string) : string;
var
  i,last : longint;
begin
  last:=length(st);
  for i:=length(st) downto 1 do
    begin
      if st[i]='.' then
        begin
          last:=i-1;
          break;
        end;
    end;
  RemoveSuffix:=Copy(st,1,last);
end;

var
  i : longint;
  param : string;
  skipnext : boolean;
begin
  object_name:='';
  skipnext:=false;
  for i:=1 to ParamCount do
    begin
      param:=Paramstr(i);
      if skipnext or (length(Param)=0) then
        begin
          skipnext:=false;
          continue;
        end;
      if Param='-o' then
        begin
          skipnext:=true;
          object_name:=ParamStr(i+1);
        end
      else if (Param[1]='-') then
        begin
          { option Param not handled }
          { Shouldn't be a real problem }
        end
      else
        begin
          if assembler_name='' then
            assembler_name:=ParamStr(i)
          else
            begin
              Writeln(stderr,'two non option param found!');
              Writeln(stderr,'first non option param =',assembler_name);
              Writeln(stderr,'second non option param =',Param);
              Writeln(stderr,'Don''t know how to handle this!');
              halt(1);
            end;
        end;
    end;

  if assembler_name='' then
    begin
      Writeln(stderr,'Dummyas, no source file specified');
      halt(1);
    end;
  Assign(ofile,assembler_name);
<<<<<<< HEAD
<<<<<<< HEAD
{$push}{$I-}
=======
{$I-}
>>>>>>> graemeg/fixes_2_2
=======
{$I-}
>>>>>>> origin/fixes_2_2
  Reset(ofile);
  if IOResult<>0 then
    begin
      Writeln(stderr,'Dummyas, source file not found ',assembler_name);
      halt(1);
    end;
  Close(ofile);
  if object_name='' then
    object_name:=RemoveSuffix(assembler_name)+'.o';
  Assign(ofile,object_name);
  Rewrite(ofile);
  if IOResult<>0 then
    begin
      Writeln(stderr,'Dummyas, object file not writable ',object_name);
      halt(1);
    end;
  Writeln(ofile,'Dummy as called');
  for i:=0 to Paramcount do
    Write(ofile,ParamStr(i),' ');
  Writeln(ofile);
  Writeln(ofile,'assembler file=',assembler_name);
  Writeln(ofile,'object file=',object_name);
  Close(ofile);
<<<<<<< HEAD
<<<<<<< HEAD
{$pop}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end.
