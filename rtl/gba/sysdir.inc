{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2006 by Free Pascal development team

    Low level directory functions
    GBA does not have any drive, so no directory handling is needed.
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
=======
procedure mkdir(s: pchar; len: sizeuint);[IOCheck, public, alias : 'FPC_SYS_MKDIR'];
>>>>>>> graemeg/cpstrnew
=======
procedure mkdir(s: pchar; len: sizeuint);[IOCheck, public, alias : 'FPC_SYS_MKDIR'];
>>>>>>> graemeg/cpstrnew
=======
procedure mkdir(s: pchar; len: sizeuint);[IOCheck, public, alias : 'FPC_SYS_MKDIR'];
>>>>>>> graemeg/cpstrnew
=======
procedure mkdir(s: pchar; len: sizeuint);[IOCheck, public, alias : 'FPC_SYS_MKDIR'];
>>>>>>> origin/cpstrnew
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure do_rmdir(const s: rawbytestring);
=======
procedure rmdir(s: pchar; len: sizeuint);[IOCheck, public, alias : 'FPC_SYS_RMDIR'];
>>>>>>> graemeg/cpstrnew
=======
procedure rmdir(s: pchar; len: sizeuint);[IOCheck, public, alias : 'FPC_SYS_RMDIR'];
>>>>>>> graemeg/cpstrnew
=======
procedure rmdir(s: pchar; len: sizeuint);[IOCheck, public, alias : 'FPC_SYS_RMDIR'];
>>>>>>> graemeg/cpstrnew
=======
procedure rmdir(s: pchar; len: sizeuint);[IOCheck, public, alias : 'FPC_SYS_RMDIR'];
>>>>>>> origin/cpstrnew
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure do_chdir(const s: rawbytestring);
=======
procedure chdir(s: pchar; len: sizeuint);[IOCheck, public, alias : 'FPC_SYS_CHDIR'];
>>>>>>> graemeg/cpstrnew
=======
procedure chdir(s: pchar; len: sizeuint);[IOCheck, public, alias : 'FPC_SYS_CHDIR'];
>>>>>>> graemeg/cpstrnew
=======
procedure chdir(s: pchar; len: sizeuint);[IOCheck, public, alias : 'FPC_SYS_CHDIR'];
>>>>>>> graemeg/cpstrnew
=======
procedure chdir(s: pchar; len: sizeuint);[IOCheck, public, alias : 'FPC_SYS_CHDIR'];
>>>>>>> origin/cpstrnew
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure do_GetDir(DriveNr: byte; var Dir: RawByteString);
=======
procedure GetDir(DriveNr: byte; var Dir: ShortString);
>>>>>>> graemeg/cpstrnew
=======
procedure GetDir(DriveNr: byte; var Dir: ShortString);
>>>>>>> graemeg/cpstrnew
=======
procedure GetDir(DriveNr: byte; var Dir: ShortString);
>>>>>>> graemeg/cpstrnew
=======
procedure GetDir(DriveNr: byte; var Dir: ShortString);
>>>>>>> origin/cpstrnew
begin

end;

