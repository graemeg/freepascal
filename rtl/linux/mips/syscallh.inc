{
    Copyright (c) 2002 by Marco van de Voort

    Header for syscall in system unit for mips *nix.

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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
    MA 02110-1301, USA.
=======
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
>>>>>>> graemeg/cpstrnew
=======
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
>>>>>>> graemeg/cpstrnew
=======
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
>>>>>>> graemeg/cpstrnew
=======
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
>>>>>>> origin/cpstrnew

 ****************************************************************************

}

Type
  TSysResult = longint; // all platforms, cint=32-bit.
                        // On platforms with off_t =64-bit, people should
                        // use int64, and typecast all calls that don't
                        // return off_t to cint.

// I don't think this is going to work on several platforms
// 64-bit machines don't have only 64-bit params.

  TSysParam  = Longint;

function Do_SysCall(sysnr:TSysParam):TSysResult;  external name 'FPC_SYSCALL0';
function Do_SysCall(sysnr,param1:TSysParam):TSysResult; external name 'FPC_SYSCALL1';
function Do_SysCall(sysnr,param1,param2:TSysParam):TSysResult;  external name 'FPC_SYSCALL2';
function Do_SysCall(sysnr,param1,param2,param3:TSysParam):TSysResult; external name 'FPC_SYSCALL3';
function Do_SysCall(sysnr,param1,param2,param3,param4:TSysParam):TSysResult; external name 'FPC_SYSCALL4';
function Do_SysCall(sysnr,param1,param2,param3,param4,param5:TSysParam):TSysResult;  external name 'FPC_SYSCALL5';
function Do_SysCall(sysnr,param1,param2,param3,param4,param5,param6:TSysParam):TSysResult;  external name 'FPC_SYSCALL6';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
