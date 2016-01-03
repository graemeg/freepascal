{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 1999-2002 by the Free Pascal development team

    BIOS functions unit for Nintendo DS
    Copyright (c) 2006 by Francesco Lombardi

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 *****************************************************************************}

{
  NDS CPU detecting function
  --------------------------
   ARM946E-S processor can handle dsp extensions, but ARM7TDMI does not. FPC can 
   detect dsp by catching a SIGILL that fires when ARM7 cpu tries to use a dsp 
   command. Unfortunately, NDS' rtl does not have any error catching mechanism.
   This function takes care to check if the code is running on an ARM9 or on an 
   ARM7 CPU, by checking the IRQ vector address ($0B003FFC for ARM9, 0380fff8 
   for ARM7), declared in the linker script. This function is cleaner than the
   older one, because does not raise any memory writing error.  
   It works on Nintendo DS only, I guess :)
}
function IsARM9(): boolean;
begin
  IsARM9 := integer(@irq_vector) = $0B003FFC;
end;
<<<<<<< HEAD
=======

procedure AssignDevice(const FIOD: TFileIODevice);
begin
  FileIODevice := FIOD;
end;
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
