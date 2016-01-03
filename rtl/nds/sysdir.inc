{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2006 by Free Pascal development team

    Low level directory functions
    Nintendo DS does not have any drive, so no directory handling is needed.
    Copyright (c) 2006 by Francesco Lombardi
    
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
procedure do_mkdir(const s: rawbytestring);
begin

end;

procedure do_rmdir(const s: rawbytestring);
begin

end;

procedure do_chdir(const s: rawbytestring);
begin

=======
procedure mkdir(s: pchar; len: sizeuint);[IOCheck, public, alias : 'FPC_SYS_MKDIR'];
begin
=======
procedure mkdir(s: pchar; len: sizeuint);[IOCheck, public, alias : 'FPC_SYS_MKDIR'];
begin
>>>>>>> graemeg/cpstrnew
=======
procedure mkdir(s: pchar; len: sizeuint);[IOCheck, public, alias : 'FPC_SYS_MKDIR'];
begin
>>>>>>> graemeg/cpstrnew
=======
procedure mkdir(s: pchar; len: sizeuint);[IOCheck, public, alias : 'FPC_SYS_MKDIR'];
begin
>>>>>>> origin/cpstrnew
  if not assigned(s) or (len=0) or (InOutRes<>0) then exit;
  if FileIODevice.DirIO.DoMkdir <> nil then
    FileIODevice.DirIO.DoMkdir(strpas(s));
end;

procedure rmdir(s: pchar; len: sizeuint);[IOCheck, public, alias : 'FPC_SYS_RMDIR'];
begin
  if not assigned(s) or (len=0) then exit;
  if FileIODevice.DirIO.DoRmdir <> nil then
    FileIODevice.DirIO.DoRmdir(strpas(s));
end;

procedure chdir(s: pchar; len: sizeuint);[IOCheck, public, alias : 'FPC_SYS_CHDIR'];
begin
  if not assigned(s) or (len=0) then exit;
  if FileIODevice.DirIO.DoChdir <> nil then
    FileIODevice.DirIO.DoChdir(strpas(s));
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
end;

procedure do_GetDir(DriveNr: byte; var Dir: RawByteString);
begin

end;



