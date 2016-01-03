{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Florian Klaempfl and Pavel Ozerski
    member of the Free Pascal development team.

    FPC Pascal system unit for go32v2.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{*****************************************************************************
                           Directory Handling
*****************************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure DosDir(func:byte;s:rawbytestring);
=======
procedure DosDir(func:byte;s:pchar;len:integer);
>>>>>>> graemeg/cpstrnew
=======
procedure DosDir(func:byte;s:pchar;len:integer);
>>>>>>> graemeg/cpstrnew
=======
procedure DosDir(func:byte;s:pchar;len:integer);
>>>>>>> graemeg/cpstrnew
=======
procedure DosDir(func:byte;s:pchar;len:integer);
>>>>>>> origin/cpstrnew
var
  regs   : trealregs;
  len    : longint;
begin
<<<<<<< HEAD
  DoDirSeparators(s);
=======
  move(s[1],buffer,length(s));
  buffer[length(s)]:=#0;
  DoDirSeparators(pchar(@buffer));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  { True DOS does not like backslashes at end
    Win95 DOS accepts this !!
    but "\" and "c:\" should still be kept and accepted hopefully PM }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  len:=length(s);
  if (len>0) and (s[len]='\') and
     Not ((len=1) or ((len=3) and (s[2]=':'))) then
    s[len]:=#0;
  syscopytodos(longint(pointer(s)),len+1);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  if (len>0) and (s[len-1]='\') and
     Not ((len=1) or ((len=3) and (s[1]=':'))) then
    s[len-1]:=#0;
  syscopytodos(longint(s),len+1);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  regs.realedx:=tb_offset;
  regs.realds:=tb_segment;
  if LFNSupport then
   regs.realeax:=$7100+func
  else
   regs.realeax:=func shl 8;
  sysrealintr($21,regs);
  if (regs.realflags and carryflag) <> 0 then
   GetInOutRes(lo(regs.realeax));
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Procedure do_MkDir(const s: rawbytestring);
begin
  DosDir($39,s);
end;

Procedure do_RmDir(const s: rawbytestring);
begin
  if s='.' then
    begin
      InOutRes := 16;
      exit;
    end;
  DosDir($3a,s);
end;

Procedure do_ChDir(const s: rawbytestring);
=======
Procedure MkDir(s: pchar;len:sizeuint);[IOCheck, public, alias : 'FPC_SYS_MKDIR'];
begin
=======
Procedure MkDir(s: pchar;len:sizeuint);[IOCheck, public, alias : 'FPC_SYS_MKDIR'];
begin
>>>>>>> origin/cpstrnew
 If not assigned(s) or (len=0) or (InOutRes <> 0) then
   exit;
  DosDir($39,s,len);
end;

<<<<<<< HEAD
=======
Procedure MkDir(s: pchar;len:sizeuint);[IOCheck, public, alias : 'FPC_SYS_MKDIR'];
begin
 If not assigned(s) or (len=0) or (InOutRes <> 0) then
   exit;
  DosDir($39,s,len);
end;

>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
Procedure RmDir(s: pchar;len:sizeuint);[IOCheck, public, alias : 'FPC_SYS_RMDIR'];
begin
  if (len=1) and (s[0] = '.' ) then
    InOutRes := 16;
  If not assigned(s) or (len=0) or (InOutRes <> 0) then
   exit;
  DosDir($3a,s,len);
end;

Procedure ChDir(s: pchar;len:sizeuint);[IOCheck, public, alias : 'FPC_SYS_CHDIR'];
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
Procedure MkDir(s: pchar;len:sizeuint);[IOCheck, public, alias : 'FPC_SYS_MKDIR'];
begin
 If not assigned(s) or (len=0) or (InOutRes <> 0) then
   exit;
  DosDir($39,s,len);
end;

Procedure RmDir(s: pchar;len:sizeuint);[IOCheck, public, alias : 'FPC_SYS_RMDIR'];
begin
  if (len=1) and (s[0] = '.' ) then
    InOutRes := 16;
  If not assigned(s) or (len=0) or (InOutRes <> 0) then
   exit;
  DosDir($3a,s,len);
end;

Procedure ChDir(s: pchar;len:sizeuint);[IOCheck, public, alias : 'FPC_SYS_CHDIR'];
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
var
  regs : trealregs;
  len  : longint;
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  len:=length(s);
{ First handle Drive changes }
  if (len>=2) and (s[2]=':') then
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  If not assigned(s) or (len=0) or (InOutRes <> 0) then
   exit;
{ First handle Drive changes }
  if (len>=2) and (s[1]=':') then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
   begin
     regs.realedx:=(ord(s[0]) and (not 32))-ord('A');
     regs.realeax:=$0e00;
     sysrealintr($21,regs);
     regs.realeax:=$1900;
     sysrealintr($21,regs);
     if byte(regs.realeax)<>byte(regs.realedx) then
      begin
        Inoutres:=15;
        exit;
      end;
     { DosDir($3b,'c:') give Path not found error on
       pure DOS PM }
     if len=2 then
       exit;
   end;
{ do the normal dos chdir }
  DosDir($3b,s,len);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure do_GetDir (DriveNr: byte; var Dir: RawByteString);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure GetDir (DriveNr: byte; var Dir: ShortString);
>>>>>>> graemeg/cpstrnew
var
  temp : array[0..255] of char;
  i    : longint;
  regs : trealregs;
begin
  regs.realedx:=drivenr;
  regs.realesi:=tb_offset;
  regs.realds:=tb_segment;
  if LFNSupport then
   regs.realeax:=$7147
  else
   regs.realeax:=$4700;
  sysrealintr($21,regs);
  if (regs.realflags and carryflag) <> 0 then
   Begin
     GetInOutRes (lo(regs.realeax));
     Dir := char (DriveNr + 64) + ':\';
     SetCodePage (Dir,DefaultFileSystemCodePage,false);
     exit;
   end
  else
   syscopyfromdos(longint(@temp),251);
{ conversion to Pascal string including slash conversion }
  i:=0;
  SetLength(dir,260);
  while (temp[i]<>#0) do
   begin
     if temp[i] in AllowDirectorySeparators then
       temp[i]:=DirectorySeparator;
     dir[i+4]:=temp[i];
     inc(i);
   end;
  dir[2]:=':';
  dir[3]:='\';
  SetLength(dir,i+3);
  SetCodePage(dir,DefaultFileSystemCodePage,false);
{ upcase the string }
  if not FileNameCasePreserving then
   dir:=upcase(dir);
  if drivenr<>0 then   { Drive was supplied. We know it }
   dir[1]:=char(65+drivenr-1)
  else
   begin
   { We need to get the current drive from DOS function 19H  }
   { because the drive was the default, which can be unknown }
     regs.realeax:=$1900;
     sysrealintr($21,regs);
     i:= (regs.realeax and $ff) + ord('A');
     dir[1]:=chr(i);
   end;
end;



