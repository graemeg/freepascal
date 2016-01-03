{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2014 by Florian Klaempfl
    and other members of the Free Pascal development team.

    FPC Pascal system unit for OS/2.

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
Procedure do_MkDir(s: rawbytestring);
var 
    Rc : word;
begin
<<<<<<< HEAD
  DoDirSeparators(s);
  Rc := DosCreateDir(pchar(s),nil);
=======
Procedure MkDir(s: pchar;len:sizeuint);[IOCheck, public, alias : 'FPC_SYS_MKDIR'];
var 
    Rc : word;
begin
=======
Procedure MkDir(s: pchar;len:sizeuint);[IOCheck, public, alias : 'FPC_SYS_MKDIR'];
var 
    Rc : word;
begin
>>>>>>> graemeg/cpstrnew
=======
Procedure MkDir(s: pchar;len:sizeuint);[IOCheck, public, alias : 'FPC_SYS_MKDIR'];
var 
    Rc : word;
begin
>>>>>>> graemeg/cpstrnew
=======
Procedure MkDir(s: pchar;len:sizeuint);[IOCheck, public, alias : 'FPC_SYS_MKDIR'];
var 
    Rc : word;
begin
>>>>>>> origin/cpstrnew
  If not assigned(s) or (len=0) or (InOutRes <> 0) then
    exit;
  DoDirSeparators(s);
  Rc := DosCreateDir(s,nil);
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
  if Rc <> 0 then
    begin
      InOutRes := Rc;
      Errno2Inoutres;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      OSErrorWatch (RC);
    end;
end;

Procedure do_RmDir(s: rawbytestring);
var 
    Rc : word;
begin
  if s = '.' then
    begin
      InOutRes := 16;
      exit;
    end;
  DoDirSeparators(s);
  Rc := DosDeleteDir(pchar(s));
=======
    end;
end;

=======
    end;
=======
  If (s='') or (InOutRes <> 0) then
   exit;
      move(s[1],buffer,length(s));
      buffer[length(s)]:=#0;
      DoDirSeparators(Pchar(@buffer));
      Rc := DosCreateDir(buffer,nil);
      if Rc <> 0 then
       begin
         InOutRes := Rc;
         Errno2Inoutres;
       end;
>>>>>>> graemeg/fixes_2_2
end;

>>>>>>> graemeg/cpstrnew
=======
    end;
end;

>>>>>>> graemeg/cpstrnew
=======
    end;
end;

>>>>>>> origin/cpstrnew
Procedure RmDir(s: pchar;len:sizeuint);[IOCheck, public, alias : 'FPC_SYS_RMDIR'];
var 
    Rc : word;
begin
  if (len=1) and (s^ = '.' ) then
    InOutRes := 16;
<<<<<<< HEAD
  If not assigned(s) or (len=0) or (InOutRes <> 0) then
    exit;
  DoDirSeparators(s);
  Rc := DosDeleteDir(s);
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
  if Rc <> 0 then
    begin
      InOutRes := Rc;
      Errno2Inoutres;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      OSErrorWatch (RC);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    end;
=======
  If (s='') or (InOutRes <> 0) then
   exit;
      move(s[1],buffer,length(s));
      buffer[length(s)]:=#0;
      DoDirSeparators(Pchar(@buffer));
      Rc := DosDeleteDir(buffer);
      if Rc <> 0 then
       begin
         InOutRes := Rc;
         Errno2Inoutres;
       end;
>>>>>>> graemeg/fixes_2_2
end;

{$ASMMODE INTEL}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Procedure do_ChDir(s: rawbytestring);

var RC: cardinal;
    Len: Longint;

begin
  Len := Length (S);
  if (Len >= 2) and (S[2] = ':') then
=======
=======
>>>>>>> graemeg/cpstrnew
Procedure ChDir(s: pchar;len:sizeuint);[IOCheck, public, alias : 'FPC_SYS_CHDIR'];

var RC: cardinal;

begin
  If not assigned(s) or (len=0) or (InOutRes <> 0) then
    exit;
  if (Len >= 2) and (S[1] = ':') then
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
Procedure ChDir(s: pchar;len:sizeuint);[IOCheck, public, alias : 'FPC_SYS_CHDIR'];

var RC: cardinal;

begin
  If not assigned(s) or (len=0) or (InOutRes <> 0) then
    exit;
  if (Len >= 2) and (S[1] = ':') then
>>>>>>> graemeg/cpstrnew
=======
Procedure ChDir(s: pchar;len:sizeuint);[IOCheck, public, alias : 'FPC_SYS_CHDIR'];

var RC: cardinal;

begin
  If not assigned(s) or (len=0) or (InOutRes <> 0) then
    exit;
  if (Len >= 2) and (S[1] = ':') then
>>>>>>> origin/cpstrnew
  begin
    RC := DosSetDefaultDisk ((Ord (S [0]) and not ($20)) - $40);
    if RC <> 0 then
     begin
      InOutRes := RC;
      OSErrorWatch (RC);
     end
    else
      if Len > 2 then
      begin
<<<<<<< HEAD
        DoDirSeparators (s);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        if (S [Len] = DirectorySeparator) and (Len <> 3) then
         S [Len] := #0;
        RC := DosSetCurrentDir (pchar (s));
=======
        RC := DosSetCurrentDir (s);
>>>>>>> graemeg/cpstrnew
=======
        RC := DosSetCurrentDir (s);
>>>>>>> graemeg/cpstrnew
=======
        RC := DosSetCurrentDir (s);
>>>>>>> graemeg/cpstrnew
=======
        RC := DosSetCurrentDir (s);
>>>>>>> origin/cpstrnew
=======
        Move (S [1], Buffer, Length (S));
        Buffer [Length (S)] := #0;
        DoDirSeparators (PChar (@Buffer));
        RC := DosSetCurrentDir (@Buffer);
>>>>>>> graemeg/fixes_2_2
        if RC <> 0 then
        begin
          InOutRes := RC;
          Errno2InOutRes;
          OSErrorWatch (RC);
        end;
      end;
  end else begin
<<<<<<< HEAD
    DoDirSeparators (s);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    if (Len > 1) and (S [Len] = DirectorySeparator) then
     S [Len] := #0;
    RC := DosSetCurrentDir (pchar (s));
=======
    RC := DosSetCurrentDir (s);
>>>>>>> graemeg/cpstrnew
=======
    RC := DosSetCurrentDir (s);
>>>>>>> graemeg/cpstrnew
=======
    RC := DosSetCurrentDir (s);
>>>>>>> graemeg/cpstrnew
=======
    RC := DosSetCurrentDir (s);
>>>>>>> origin/cpstrnew
=======
    Move (S [1], Buffer, Length (S));
    Buffer [Length (S)] := #0;
    DoDirSeparators (PChar (@Buffer));
    RC := DosSetCurrentDir (@Buffer);
>>>>>>> graemeg/fixes_2_2
    if RC <> 0 then
    begin
      InOutRes:= RC;
      Errno2InOutRes;
      OSErrorWatch (RC);
    end;
  end;
end;

{$ASMMODE ATT}

procedure do_GetDir (DriveNr: byte; var Dir: RawByteString);
{Written by Michael Van Canneyt.}
var sof: Pchar;
    i:byte;
    l,l2:cardinal;
    RC: cardinal;
begin
    setlength(Dir,255);
    Dir [4] := #0;
    { Used in case the specified drive isn't available }
    sof:=pchar(@dir[4]);
    { dir[1..3] will contain '[drivenr]:\', but is not }
    { supplied by DOS, so we let dos string start at   }
    { dir[4]                                           }
    { Get dir from drivenr : 0=default, 1=A etc... }
    { TODO: if max path length is > 255, increase the setlength parameter above and
      the 255 below }
    l:=255-3;
    RC := DosQueryCurrentDir(DriveNr, sof^, l);
    if RC <> 0 then
     begin
      InOutRes := longint (RC);
      Errno2Inoutres;
      OSErrorWatch (RC);
     end;
{$WARNING Result code should be translated in some cases!}
    { Now Dir should be filled with directory in ASCIIZ, }
    { starting from dir[4]                               }
    dir[2]:=':';
    dir[3]:='\';
    i:=4;
    {Conversion to Pascal string }
    while (dir[i]<>#0) do
        begin
            { convert path name to DOS }
            if dir[i] in AllowDirectorySeparators then
              dir[i]:=DirectorySeparator;
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
            dir[0]:=char(i);
>>>>>>> graemeg/fixes_2_2
            inc(i);
        end;
    setlength(dir,i-1);
    { upcase the string (FPC function) }
    if drivenr<>0 then   { Drive was supplied. We know it }
        dir[1]:=chr(64+drivenr)
    else
        begin
            { We need to get the current drive from DOS function 19H  }
            { because the drive was the default, which can be unknown }
            DosQueryCurrentDisk(l, l2);
            dir[1]:=chr(64+l);
        end;
    SetCodePage(dir,DefaultFileSystemCodePage,false);
    if not (FileNameCasePreserving) then dir:=upcase(dir);
end;



