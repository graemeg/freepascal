{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Copyright (c) 1999-2012 by Peter Vreman, Michael Van Canneyt
=======
    Copyright (c) 1999-2010 by Peter Vreman, Michael Van Canneyt
>>>>>>> graemeg/cpstrnew
=======
    Copyright (c) 1999-2010 by Peter Vreman, Michael Van Canneyt
>>>>>>> graemeg/cpstrnew
=======
    Copyright (c) 1999-2010 by Peter Vreman, Michael Van Canneyt
>>>>>>> graemeg/cpstrnew
=======
    Copyright (c) 1999-2010 by Peter Vreman, Michael Van Canneyt
>>>>>>> origin/cpstrnew

    Deletes all files generated for Pascal (*.exe,units,objects,libs)

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
program Delp;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$mode ObjFPC}{$H+}

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
uses
  Sysutils,getopts;

const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Version   = 'Version 1.3';
  Title     = 'DelPascal';
  Copyright = 'Copyright (c) 1999-2012 by the Free Pascal Development Team';
=======
  Version   = 'Version 1.2';
  Title     = 'DelPascal';
=======
  Version   = 'Version 1.2';
  Title     = 'DelPascal';
>>>>>>> graemeg/cpstrnew
=======
  Version   = 'Version 1.2';
  Title     = 'DelPascal';
>>>>>>> graemeg/cpstrnew
=======
  Version   = 'Version 1.2';
  Title     = 'DelPascal';
>>>>>>> origin/cpstrnew
  Copyright = 'Copyright (c) 1999-2010 by the Free Pascal Development Team';

>>>>>>> graemeg/cpstrnew

function DStr(l:longint):string;
var
  TmpStr : string[32];
  i : byte;
begin
  Str(l,tmpstr);
  i:=Length(TmpStr);
  while (i>3) do
   begin
     i:=i-3;
     if TmpStr[i]<>'-' then
      Insert('.',TmpStr,i+1);
   end;
  DStr:=TmpStr;
end;

procedure dosfsplit(path:ansistring;var dir,name,ext:ansistring);
// implementation of dos.fsplit on top of sysutils routines to avoid
// path length issues.
// a lot of old tooling uses fsplit, maybe move this to sysutils?
begin
  dir:=extractfiledir(dir);
  name:=changefileext(extractfilename(path),'');
  ext:=extractfileext(path);
  if (length(ext)>0) and (ext[1]='.') then
    delete(ext,1,1);
end;

function MatchesMask(What, Mask: string): boolean;

  Function CmpStr(const hstr1,hstr2:string):boolean;
  var
    found : boolean;
    i1,i2 : longint;
  begin
    i1:=0;
    i2:=0;
    found:=true;
    while found and (i1<length(hstr1)) and (i2<length(hstr2)) do
     begin
       inc(i2);
       inc(i1);
       case hstr1[i1] of
         '?' :
           found:=true;
         '*' :
           begin
             found:=true;
             if (i1=length(hstr1)) then
              i2:=length(hstr2)
             else
              if (i1<length(hstr1)) and (hstr1[i1+1]<>hstr2[i2]) then
               begin
                 if i2<length(hstr2) then
                  dec(i1)
               end
             else
              if i2>1 then
               dec(i2);
           end;
         else
           found:=(hstr1[i1]=hstr2[i2]) or (hstr2[i2]='?');
       end;
     end;
    if found then
     begin
       { allow 'p*' matching 'p' }
       if (i1<length(hstr1)) and (hstr1[i1+1]='*') then
        inc(i1);
       found:=(i1>=length(hstr1)) and (i2>=length(hstr2));
     end;
    CmpStr:=found;
  end;

var
  D1,D2 : ansistring;
  N1,N2 : ansistring;
  E1,E2 : ansistring;
begin
{$ifdef Unix}
  DosFSplit(What,D1,N1,E1);
  DosFSplit(Mask,D2,N2,E2);
{$else}
  DosFSplit(Uppercase(What),D1,N1,E1);
  DosFSplit(Uppercase(Mask),D2,N2,E2);
{$endif}
  MatchesMask:=CmpStr(N2,N1) and CmpStr(E2,E1);
end;

type
  PMaskItem=^TMaskItem;
  TMaskItem=record
    Mask  : string[15];
    Files : longint;
    Size  : longint;
    Next  : PMaskItem;
  end;

var
  masklist : pmaskitem;

procedure AddMask(s:string);
var
  maskitem : PMaskItem;
  i : longint;
begin
  repeat
    i:=pos(' ',s);
    if i=0 then
     i:=length(s)+1;
    New(maskitem);
    fillchar(maskitem^,sizeof(tmaskitem),0);
    maskitem^.mask:=Copy(s,1,i-1);
    maskitem^.next:=masklist;
    masklist:=maskitem;
    Delete(s,1,i);
  until s='';
end;

Var
  quiet : boolean = False;
  Recurse : Boolean = False;
  Verbose : Boolean = False;
  NoDelete : Boolean = False;
  MaxDepth : Integer = 0;

procedure usage;

begin
  Writeln('Delp [options] <directory> [<directory2> [<directory3> ...]');
  Writeln('Where options is one of:');
  writeln('  -e    Delete executables also (Not on Unix)');
  writeln('  -x ext Add extension to list of extensions to delete (no dot)');
  writeln('  -h    Display (this) help message.');
  writeln('  -r    Recurse into directories.');
  writeln('  -n    Do not actually delete files.');
  writeln('  -m N  Maximum depth to recurse into directories (1 based, zero is no max).');
  writeln('  -q    Quietly perfoms deleting.');
  writeln('  -v    Verbose (print names of deleted files).');
  Halt(1);
end;

procedure processoptions;

Var c : char;

begin
  quiet:=false;
  Repeat
    C:=Getopt('ehvnqrm:x:');
    Case C of
      'e' : AddMAsk('*.exe *.so *.dll');
      'x' : if (optarg<>'') then
              AddMask('*.'+optarg);
      'h' : Usage;
      'r' : Recurse:=True;
      'n' : NoDelete:=True;
      'm' : MaxDepth:=StrToInt(optarg);
      'q' : begin
            Quiet:=True;
            verbose:=False;
            end;
      'v' : Verbose:=True;
      EndOfOptions : ;
    end;
  Until C=EndOfOptions;
end;

Procedure DoDir(basedir : string; ALevel : Integer);

var
  Dir    : TSearchrec;
  hp     : pmaskitem;

begin
  if Verbose and not Quiet then
    Writeln('Cleaning directory "',BaseDir,'".');
  if FindFirst(basedir+'*.*',faanyfile,Dir)=0 then
   begin
     repeat
       hp:=masklist;
       while assigned(hp) do
         begin
           if ((Dir.Attr and faDirectory)=0) and MatchesMask(Dir.Name,hp^.mask) then
             begin
               if Verbose then
                 Writeln('Deleting "',BaseDir+Dir.Name,'"');
               if not NoDelete then
                 DeleteFile(BaseDir+Dir.Name);
               inc(hp^.Files);
               inc(hp^.Size,Dir.Size);
               break;
             end;
           hp:=hp^.next;
         end;
      until FindNext(Dir)<>0;
      FindClose(Dir);
    end;
  if Recurse and ((MaxDepth=0) or (ALevel<=MaxDepth)) then
    if FindFirst(basedir+allfilesmask,faanyfile,Dir)=0 then
      begin
      Repeat
        if ((Dir.Attr and faDirectory)=faDirectory) and Not ((Dir.Name='.') or (Dir.Name='..')) then
           DoDir(IncludeTrailingPathDelimiter(BaseDir+Dir.Name),ALevel+1);
      until FindNext(Dir)<>0;
      FindClose(Dir);
      end;
end;

Procedure PrintResults;

var
  Total  : longint;
  found  : boolean;
  hp     : pmaskitem;

begin
  { Write Results }
  Total:=0;
  found:=false;
  hp:=masklist;
  while assigned(hp) do
    begin
    if hp^.Files>0 then
      begin
      WriteLn(' - Removed ',hp^.Files:2,' ',hp^.Mask,' (',DStr(hp^.Size)+' Bytes)');
      inc(Total,hp^.Size);
      found:=true;
      end;
    hp:=hp^.next;
    end;
  if not found then
    WriteLn(' - No Redundant Files Found!')
  else
    WriteLn(' - Total ',DStr(Total),' Bytes Freed');
end;

var
  basedir : string;
  i : Integer;

begin
  ProcessOptions;
  I:=OptInd;
  if (OptInd=0) or (OptInd>ParamCount) then
    Usage;
  { Win32 target }
  AddMask('*.ppw *.ow *.aw *.sw');
  AddMask('ppas.bat ppas.sh link.res fpcmaked fpcmade fpcmade.*');
  AddMask('*.tpu *.tpp *.tpw *.tr');
  AddMask('*.dcu *.dcp *.bpl');
  AddMask('*.log *.bak *.~pas *.~pp *.*~');
<<<<<<< HEAD
<<<<<<< HEAD
  AddMask('*.ppu *.o *.a *.s *.or *.compiled');
=======
=======
>>>>>>> origin/fixes_2_2
  AddMask('*.ppu *.o *.a *.s');
>>>>>>> graemeg/fixes_2_2
  AddMask('*.pp1 *.o1 *.a1 *.s1');
  AddMask('*.ppo *.oo *.ao *.so');
  AddMask('*.rst *.rsj');
  { OS/2 target }
  AddMask('*.oo2 *.so2 *.ppo');
  { Amiga target }
  AddMask('*.ppa *.asm');
  if not quiet then
    begin
      writeln(Title+' '+Version);
      writeln(Copyright);
      Writeln;
    end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  While (I<=ParamCount) do
    begin
    BaseDir:=IncludeTrailingPathDelimiter(Paramstr(I));
    DoDir(Basedir,1);
    Inc(I);
    end;
  if Not Quiet then
    PrintResults;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  Total:=0;
  While (I<=ParamCount) do
    begin
    BaseDir:=Paramstr(I);
    If BaseDir[Length(BaseDir)]<>DirectorySeparator then
      BaseDir:=BaseDir+DirectorySeparator;
    FindFirst(basedir+'*.*',anyfile,Dir);
    while (doserror=0) do
      begin
      hp:=masklist;
      while assigned(hp) do
        begin
        if MatchesMask(Dir.Name,hp^.mask) then
          begin
          EraseFile(BaseDir+Dir.Name);
          inc(hp^.Files);
          inc(hp^.Size,Dir.Size);
          break;
          end;
        hp:=hp^.next;
        end;
        FindNext(Dir);
      end;
    FindClose(Dir);
    Inc(I);
    end;
  { Write Results }
  found:=false;
  hp:=masklist;
  while assigned(hp) do
    begin
    if hp^.Files>0 then
      begin
      if not quiet then
        WriteLn(' - Removed ',hp^.Files:2,' ',hp^.Mask,' (',DStr(hp^.Size)+' Bytes)');
      inc(Total,hp^.Size);
      found:=true;
      end;
    hp:=hp^.next;
    end;
  if not quiet then
    if not found then
      WriteLn(' - No Redundant Files Found!')
    else
      WriteLn(' - Total ',DStr(Total),' Bytes Freed');
>>>>>>> graemeg/cpstrnew
end.
