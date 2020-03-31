{
    Copyright (c) 1998-2002 by Peter Vreman and Florian Klaempfl

    Convert xtensareg.dat to several .inc files for usage with
    the Free pascal compiler

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$mode objfpc}

program mkxtensareg;

const Version = '1.00';
      max_regcount = 200;

var s : string;
    i : longint;
    line : longint;
    regcount:byte;
    regcount_bsstart:byte;
    names,
    regtypes,
    subtypes,
    supregs,
    numbers,
    stdnames,
    stabs,dwarf : array[0..max_regcount-1] of string[63];
    regnumber_index,
    std_regname_index : array[0..max_regcount-1] of byte;

function tostr(l : longint) : string;

begin
  str(l,tostr);
end;

function readstr : string;

  begin
     result:='';
     while (s[i]<>',') and (i<=length(s)) do
       begin
          result:=result+s[i];
          inc(i);
       end;
  end;


procedure readcomma;
  begin
     if s[i]<>',' then
       begin
         writeln('Missing "," at line ',line);
         writeln('Line: "',s,'"');
         halt(1);
       end;
     inc(i);
  end;


procedure skipspace;

  begin
     while (s[i] in [' ',#9]) do
       inc(i);
  end;

procedure openinc(out f:text;const fn:string);
begin
  writeln('creating ',fn);
  assign(f,fn);
  rewrite(f);
  writeln(f,'{ don''t edit, this file is generated from xtensareg.dat }');
end;


procedure closeinc(var f:text);
begin
  writeln(f);
  close(f);
end;

procedure build_regnum_index;

var h,i,j,p,t:byte;

begin
  {Build the registernumber2regindex index.
   Step 1: Fill.}
  for i:=0 to regcount-1 do
    regnumber_index[i]:=i;
  {Step 2: Sort. We use a Shell-Metzner sort.}
  p:=regcount_bsstart;
  repeat
    for h:=0 to regcount-p-1 do
      begin
        i:=h;
        repeat
          j:=i+p;
          if numbers[regnumber_index[j]]>=numbers[regnumber_index[i]] then
            break;
          t:=regnumber_index[i];
          regnumber_index[i]:=regnumber_index[j];
          regnumber_index[j]:=t;
          if i<p then
            break;
          dec(i,p);
        until false;
      end;
    p:=p shr 1;
  until p=0;
end;

procedure build_std_regname_index;

var h,i,j,p,t:byte;

begin
  {Build the registernumber2regindex index.
   Step 1: Fill.}
  for i:=0 to regcount-1 do
    std_regname_index[i]:=i;
  {Step 2: Sort. We use a Shell-Metzner sort.}
  p:=regcount_bsstart;
  repeat
    for h:=0 to regcount-p-1 do
      begin
        i:=h;
        repeat
          j:=i+p;
          if stdnames[std_regname_index[j]]>=stdnames[std_regname_index[i]] then
            break;
          t:=std_regname_index[i];
          std_regname_index[i]:=std_regname_index[j];
          std_regname_index[j]:=t;
          if i<p then
            break;
          dec(i,p);
        until false;
      end;
    p:=p shr 1;
  until p=0;
end;


procedure read_spreg_file;

var infile:text;

begin
   { open dat file }
   assign(infile,'xtensareg.dat');
   reset(infile);
   while not(eof(infile)) do
     begin
        { handle comment }
        readln(infile,s);
        inc(line);
        while (s[1]=' ') do
         delete(s,1,1);
        if (s='') or (s[1]=';') then
          continue;

        i:=1;
        names[regcount]:=readstr;
        readcomma;
        regtypes[regcount]:=readstr;
        readcomma;
        subtypes[regcount]:=readstr;
        readcomma;
        supregs[regcount]:=readstr;
        readcomma;
        stdnames[regcount]:=readstr;
        readcomma;
        stabs[regcount]:=readstr;
        readcomma;
        dwarf[regcount]:=readstr;
        { Create register number }
        if supregs[regcount][1]<>'$' then
          begin
            writeln('Missing $ before number, at line ',line);
            writeln('Line: "',s,'"');
            halt(1);
          end;
        numbers[regcount]:=regtypes[regcount]+copy(subtypes[regcount],2,255)+'00'+copy(supregs[regcount],2,255);
        if i<length(s) then
          begin
            writeln('Extra chars at end of line, at line ',line);
            writeln('Line: "',s,'"');
            halt(1);
          end;
        inc(regcount);
        if regcount>max_regcount then
          begin
            writeln('Error: Too much registers, please increase maxregcount in source');
            halt(2);
          end;
     end;
   close(infile);
end;

procedure write_inc_files;

var
    norfile,stdfile,supfile,
    numfile,stabfile,dwarffile,confile,
    rnifile,srifile:text;
    first:boolean;

begin
  { create inc files }
  openinc(confile,'rxtensacon.inc');
  openinc(supfile,'rxtensasup.inc');
  openinc(numfile,'rxtensanum.inc');
  openinc(stdfile,'rxtensastd.inc');
  openinc(stabfile,'rxtensasta.inc');
  openinc(dwarffile,'rxtensadwa.inc');
  openinc(norfile,'rxtensanor.inc');
  openinc(rnifile,'rxtensarni.inc');
  openinc(srifile,'rxtensasri.inc');
  first:=true;
  for i:=0 to regcount-1 do
    begin
      if not first then
        begin
          writeln(numfile,',');
          writeln(stdfile,',');
          writeln(stabfile,',');
          writeln(dwarffile,',');
          writeln(rnifile,',');
          writeln(srifile,',');
        end
      else
        first:=false;
      writeln(supfile,'RS_',names[i],' = ',supregs[i],';');
      writeln(confile,'NR_'+names[i],' = ','tregister(',numbers[i],')',';');
      write(numfile,'tregister(',numbers[i],')');
      write(stdfile,'''',stdnames[i],'''');
      write(stabfile,stabs[i]);
      write(dwarffile,dwarf[i]);
      write(rnifile,regnumber_index[i]);
      write(srifile,std_regname_index[i]);
    end;
  write(norfile,regcount);
  close(confile);
  close(supfile);
  closeinc(numfile);
  closeinc(stdfile);
  closeinc(stabfile);
  closeinc(dwarffile);
  closeinc(norfile);
  closeinc(rnifile);
  closeinc(srifile);
  writeln('Done!');
  writeln(regcount,' registers processed');
end;


begin
   writeln('Register Table Converter Version ',Version);
   line:=0;
   regcount:=0;
   read_spreg_file;
   regcount_bsstart:=1;
   while 2*regcount_bsstart<regcount do
     regcount_bsstart:=regcount_bsstart*2;
   build_regnum_index;
   build_std_regname_index;
   write_inc_files;
end.
