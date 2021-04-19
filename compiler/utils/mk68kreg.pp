{
    Copyright (c) 1998-2002 by Peter Vreman and Florian Klaempfl

    Convert m68kreg.dat to several .inc files for usage with
    the Free pascal compiler

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$mode objfpc}
program mk68kreg;

const Version = '1.00';
      max_regcount = 200;

var s : string;
    i : longint;
    line : longint;
    regcount:byte;
    regcount_bsstart:byte;
    names,
    values,
    stdnames,
    gasnames,
    stdfullnames,
    gasfullnames,
    stabs : array[0..max_regcount-1] of string[63];
    regnumber_index,
    std_regname_index,gas_regname_index : array[0..max_regcount-1] of byte;

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
  writeln(f,'{ don''t edit, this file is generated from m68kreg.dat }');
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
          if values[regnumber_index[j]]>=values[regnumber_index[i]] then
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
          if stdfullnames[std_regname_index[j]]>=stdfullnames[std_regname_index[i]] then
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

procedure build_gas_regname_index;

var h,i,j,p,t:byte;

begin
  {Build the registernumber2regindex index.
   Step 1: Fill.}
  for i:=0 to regcount-1 do
    gas_regname_index[i]:=i;
  {Step 2: Sort. We use a Shell-Metzner sort.}
  p:=regcount_bsstart;
  repeat
    for h:=0 to regcount-p-1 do
      begin
        i:=h;
        repeat
          j:=i+p;
          if gasfullnames[gas_regname_index[j]]>=gasfullnames[gas_regname_index[i]] then
            break;
          t:=gas_regname_index[i];
          gas_regname_index[i]:=gas_regname_index[j];
          gas_regname_index[j]:=t;
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
   assign(infile,'m68kreg.dat');
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
        values[regcount]:=readstr;
        readcomma;
        stdnames[regcount]:=readstr;
        readcomma;
        gasnames[regcount]:=readstr;
        readcomma;
        stdfullnames[regcount]:=readstr;
        readcomma;
        gasfullnames[regcount]:=readstr;
        readcomma;
        stabs[regcount]:=readstr;
        { Create register number }
        if values[regcount][1]<>'$' then
          begin
            writeln('Missing $ before number, at line ',line);
            writeln('Line: "',s,'"');
            halt(1);
          end;
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
    norfile,stdfile,stdffile,gasfile,gasffile,supfile,
    numfile,stabfile,confile,
    rnifile,srifile,grifile,
    bssfile:text;
    first:boolean;

begin
  { create inc files }
  openinc(confile,'r68kcon.inc');
  openinc(supfile,'r68ksup.inc');
  openinc(numfile,'r68knum.inc');
  openinc(stdfile,'r68kstd.inc');
  openinc(gasfile,'r68kgas.inc');
  openinc(stdffile,'r68kstdf.inc');
  openinc(gasffile,'r68kgasf.inc');
  openinc(stabfile,'r68ksta.inc');
  openinc(norfile,'r68knor.inc');
  openinc(rnifile,'r68krni.inc');
  openinc(srifile,'r68ksri.inc');
  openinc(grifile,'r68kgri.inc');
  openinc(bssfile,'r68kbss.inc');
  first:=true;
  for i:=0 to regcount-1 do
    begin
      if not first then
        begin
          writeln(numfile,',');
          writeln(stdfile,',');
          writeln(gasfile,',');
          writeln(stdffile,',');
          writeln(gasffile,',');
          writeln(stabfile,',');
          writeln(rnifile,',');
          writeln(srifile,',');
          writeln(grifile,',');
        end
      else
        first:=false;
      if (copy(values[i],4,2)='00') or (copy(values[i],4,2)='04') then // subd or subnone -> superregister
        writeln(supfile,'RS_',names[i],' = $',copy(values[i],length(values[i])-1,2),'; { ',values[i],' }');
      writeln(confile,'NR_'+names[i],' = ','tregister(',values[i],')',';');
      write(numfile,'tregister(',values[i],')');
      write(stdfile,'''',stdnames[i],'''');
      write(gasfile,'''',gasnames[i],'''');
      write(stdffile,'''',stdfullnames[i],'''');
      write(gasffile,'''',gasfullnames[i],'''');
      write(stabfile,stabs[i]);
      write(rnifile,regnumber_index[i]);
      write(srifile,std_regname_index[i]);
      write(grifile,gas_regname_index[i]);
    end;
  write(norfile,regcount);
  write(bssfile,'regnumber_count_bsstart = ',regcount_bsstart,';');
  close(confile);
  close(supfile);
  closeinc(numfile);
  closeinc(stdfile);
  closeinc(gasfile);
  closeinc(stdffile);
  closeinc(gasffile);
  closeinc(stabfile);
  closeinc(norfile);
  closeinc(rnifile);
  closeinc(srifile);
  closeinc(grifile);
  closeinc(bssfile);
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
   build_gas_regname_index;
   write_inc_files;
end.
