{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Florian Klaempfl and Pavel Ozerski
    member of the Free Pascal development team.

    FPC Pascal system unit for the Win32 API.

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
Procedure do_MkDir(const s: rawbytestring);
var Res: LONGINT;
BEGIN
  Res := FpMkdir (pchar(s),S_IRWXU);
=======
Procedure MkDir(s: pchar;len:sizeuint);[IOCheck, public, alias : 'FPC_SYS_MKDIR'];
var Res: LONGINT;
BEGIN
  Res := FpMkdir (s,S_IRWXU);
>>>>>>> graemeg/cpstrnew
=======
Procedure MkDir(s: pchar;len:sizeuint);[IOCheck, public, alias : 'FPC_SYS_MKDIR'];
var Res: LONGINT;
BEGIN
  Res := FpMkdir (s,S_IRWXU);
>>>>>>> graemeg/cpstrnew
=======
Procedure MkDir(s: pchar;len:sizeuint);[IOCheck, public, alias : 'FPC_SYS_MKDIR'];
var Res: LONGINT;
BEGIN
  Res := FpMkdir (s,S_IRWXU);
>>>>>>> graemeg/cpstrnew
=======
Procedure MkDir(s: pchar;len:sizeuint);[IOCheck, public, alias : 'FPC_SYS_MKDIR'];
var Res: LONGINT;
BEGIN
  Res := FpMkdir (s,S_IRWXU);
>>>>>>> origin/cpstrnew
  if Res = 0 then
    InOutRes:=0
  else
    SetFileError (Res);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure do_RmDir(const s: rawbytestring);
var Res: longint;
begin
  Res := FpRmdir (pchar(s));
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure RmDir(s: pchar;len:sizeuint);[IOCheck, public, alias : 'FPC_SYS_RMDIR'];
var Res: longint;
begin
  Res := FpRmdir (s);
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
  if Res = 0 then
    InOutRes:=0
  else
    SetFileError (Res);
end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure do_ChDir(const s: rawbytestring);
=======
procedure ChDir(s: pchar;len:sizeuint);[IOCheck, public, alias : 'FPC_SYS_CHDIR'];
>>>>>>> graemeg/cpstrnew
=======
procedure ChDir(s: pchar;len:sizeuint);[IOCheck, public, alias : 'FPC_SYS_CHDIR'];
>>>>>>> graemeg/cpstrnew
=======
procedure ChDir(s: pchar;len:sizeuint);[IOCheck, public, alias : 'FPC_SYS_CHDIR'];
>>>>>>> graemeg/cpstrnew
=======
procedure ChDir(s: pchar;len:sizeuint);[IOCheck, public, alias : 'FPC_SYS_CHDIR'];
>>>>>>> origin/cpstrnew
var Res: longint;
begin
  Res := FpChdir (s);
  if Res = 0 then
    InOutRes:=0
  else
    SetFileError (Res);
end;


procedure do_getdir(drivenr : byte;var dir : rawbytestring);
var P : array [0..255] of CHAR;
    i : LONGINT;
begin
  P[0] := #0;
  getcwdpath(@P,nil,0);   // getcwd does not return volume, getcwdpath does
  i := libc_strlen (P);
  if i > 0 then
  begin
    SetLength (dir, i);
    Move (P, dir[1], i);
<<<<<<< HEAD
=======
    BYTE(dir[0]) := i;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    DoDirSeparators(dir);
    // fix / after volume, the compiler needs that
    // normaly root of a volumes is SERVERNAME/SYS:, change that
    // to SERVERNAME/SYS:/
    i := pos (':',dir);
    if (i > 0) then
      if i = Length (dir) then dir := dir + '/' else
      if dir [i+1] <> '/' then insert ('/',dir,i+1);
    SetCodePage (dir,DefaultFileSystemCodePage,false);
  end else
    InOutRes := 1;
end;



