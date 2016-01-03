<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{*****************************************************************************
                           Directory Handling
*****************************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure DosDir(func:byte;const s:rawbytestring);
=======
procedure DosDir(func:byte;const s:string);
>>>>>>> graemeg/cpstrnew
=======
procedure DosDir(func:byte;const s:string);
>>>>>>> graemeg/cpstrnew
=======
procedure DosDir(func:byte;const s:string);
>>>>>>> graemeg/cpstrnew
=======
procedure DosDir(func:byte;const s:string);
>>>>>>> origin/cpstrnew
=======
procedure DosDir(func:byte;const s:string);
>>>>>>> origin/fixes_2.4
var
  buffer : array[0..255] of char;
  regs   : trealregs;
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if length(s)>255 then
    begin
      inoutres:=3;
      exit;
    end;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
  move(s[1],buffer,length(s));
  buffer[length(s)]:=#0;
  DoDirSeparators(pchar(@buffer));
  { True DOS does not like backslashes at end
    Win95 DOS accepts this !!
    but "\" and "c:\" should still be kept and accepted hopefully PM }
  if (length(s)>0) and (buffer[length(s)-1]='\') and
     Not ((length(s)=1) or ((length(s)=3) and (s[2]=':'))) then
    buffer[length(s)-1]:=#0;
  syscopytodos(longint(@buffer),length(s)+1);
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
<<<<<<< HEAD
procedure do_mkdir(const s : rawbytestring);
begin
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
procedure mkdir(const s : string);[IOCheck];
begin
  If (s='') or (InOutRes <> 0) then
   exit;
<<<<<<< HEAD
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
=======
>>>>>>> origin/fixes_2.4
  DosDir($39,s);
end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure do_rmdir(const s : rawbytestring);
begin
  if s = '.' then
    begin
      InOutRes := 16;
      exit;
    end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
procedure rmdir(const s : string);[IOCheck];
begin
  if (s = '.' ) then
    InOutRes := 16;
  If (s='') or (InOutRes <> 0) then
   exit;
<<<<<<< HEAD
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
=======
>>>>>>> origin/fixes_2.4
  DosDir($3a,s);
end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure do_chdir(const s : rawbytestring);
var
  regs : trealregs;
begin
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
procedure chdir(const s : string);[IOCheck];
var
  regs : trealregs;
begin
  If (s='') or (InOutRes <> 0) then
   exit;
<<<<<<< HEAD
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
=======
>>>>>>> origin/fixes_2.4
{ First handle Drive changes }
  if (length(s)>=2) and (s[2]=':') then
   begin
     regs.realedx:=(ord(s[1]) and (not 32))-ord('A');
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
     if length(s)=2 then
       exit;
   end;
{ do the normal dos chdir }
  DosDir($3b,s);
end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure do_getdir(drivenr : byte;var dir : RawByteString);
=======
procedure getdir(drivenr : byte;var dir : shortstring);
>>>>>>> graemeg/cpstrnew
=======
procedure getdir(drivenr : byte;var dir : shortstring);
>>>>>>> graemeg/cpstrnew
=======
procedure getdir(drivenr : byte;var dir : shortstring);
>>>>>>> graemeg/cpstrnew
=======
procedure getdir(drivenr : byte;var dir : shortstring);
>>>>>>> origin/cpstrnew
=======
procedure getdir(drivenr : byte;var dir : shortstring);
>>>>>>> origin/fixes_2.4
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
     GetInOutRes(lo(regs.realeax));
     Dir := char (DriveNr + 64) + ':\';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     SetCodePage(dir,DefaultFileSystemCodePage,false);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
     exit;
   end
  else
   syscopyfromdos(longint(@temp),251);
{ conversion to Pascal string including slash conversion }
  i:=0;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  SetLength(Dir,255);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
  while (temp[i]<>#0) do
   begin
     if temp[i] in AllowDirectorySeparators then
      temp[i]:=DirectorySeparator;
     dir[i+4]:=temp[i];
     inc(i);
   end;
  dir[2]:=':';
  dir[3]:='\';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  SetLength(Dir,i+3);
  SetCodePage(dir,DefaultFileSystemCodePage,false);
{ upcase the string }
  if not FileNameCasePreserving then
=======
  dir[0]:=char(i+3);
{ upcase the string }
  if not FileNameCaseSensitive then
>>>>>>> graemeg/cpstrnew
=======
  dir[0]:=char(i+3);
{ upcase the string }
  if not FileNameCaseSensitive then
>>>>>>> graemeg/cpstrnew
=======
  dir[0]:=char(i+3);
{ upcase the string }
  if not FileNameCaseSensitive then
>>>>>>> graemeg/cpstrnew
=======
  dir[0]:=char(i+3);
{ upcase the string }
  if not FileNameCaseSensitive then
>>>>>>> origin/cpstrnew
=======
  dir[0]:=char(i+3);
{ upcase the string }
  if not FileNameCaseSensitive then
>>>>>>> origin/fixes_2.4
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

<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
{*****************************************************************************
                           Directory Handling
*****************************************************************************}

procedure DosDir(func:byte;const s:string);
var
  buffer : array[0..255] of char;
  regs   : trealregs;
begin
  move(s[1],buffer,length(s));
  buffer[length(s)]:=#0;
  DoDirSeparators(pchar(@buffer));
  { True DOS does not like backslashes at end
    Win95 DOS accepts this !!
    but "\" and "c:\" should still be kept and accepted hopefully PM }
  if (length(s)>0) and (buffer[length(s)-1]='\') and
     Not ((length(s)=1) or ((length(s)=3) and (s[2]=':'))) then
    buffer[length(s)-1]:=#0;
  syscopytodos(longint(@buffer),length(s)+1);
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


procedure mkdir(const s : string);[IOCheck];
begin
  If (s='') or (InOutRes <> 0) then
   exit;
  DosDir($39,s);
end;


procedure rmdir(const s : string);[IOCheck];
begin
  if (s = '.' ) then
    InOutRes := 16;
  If (s='') or (InOutRes <> 0) then
   exit;
  DosDir($3a,s);
end;


procedure chdir(const s : string);[IOCheck];
var
  regs : trealregs;
begin
  If (s='') or (InOutRes <> 0) then
   exit;
{ First handle Drive changes }
  if (length(s)>=2) and (s[2]=':') then
   begin
     regs.realedx:=(ord(s[1]) and (not 32))-ord('A');
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
     if length(s)=2 then
       exit;
   end;
{ do the normal dos chdir }
  DosDir($3b,s);
end;


procedure getdir(drivenr : byte;var dir : shortstring);
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
     GetInOutRes(lo(regs.realeax));
     Dir := char (DriveNr + 64) + ':\';
     exit;
   end
  else
   syscopyfromdos(longint(@temp),251);
{ conversion to Pascal string including slash conversion }
  i:=0;
  while (temp[i]<>#0) do
   begin
     if temp[i] in AllowDirectorySeparators then
      temp[i]:=DirectorySeparator;
     dir[i+4]:=temp[i];
     inc(i);
   end;
  dir[2]:=':';
  dir[3]:='\';
  dir[0]:=char(i+3);
{ upcase the string }
  if not FileNameCaseSensitive then
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

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
