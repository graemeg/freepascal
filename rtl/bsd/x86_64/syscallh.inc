{
    Copyright (c) 2002 by Marco van de Voort

    Header for syscall in system unit for i386 *BSD.

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
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
    MA 02111-1301, USA.

 ****************************************************************************

}

{$ifdef FPC_USE_SYSCALL}

Type

  TSysResult = int64;   // all platforms, cint=32-bit.
                        // On platforms with off_t =64-bit, people should
                        // use int64, and typecast all calls that don't
                        // return off_t to cint.

  TSysParam  = int64;

function do_sysCall(sysnr:TSysParam):TSysResult; external name 'FPC_DOSYS0';
function do_sysCall(sysnr,param1:TSysParam):TSysResult; external name 'FPC_DOSYS1';
function do_sysCall(sysnr,param1,param2:TSysParam):TSysResult;  external name 'FPC_DOSYS2';
function do_sysCall(sysnr,param1,param2,param3:TSysParam):TSysResult; external name 'FPC_DOSYS3';
function do_sysCall(sysnr,param1,param2,param3,param4:TSysParam):TSysResult; external name 'FPC_DOSYS4';
function do_sysCall(sysnr,param1,param2,param3,param4,param5:TSysParam):TSysResult; external name 'FPC_DOSYS5';
function do_sysCall(sysnr,param1,param2,param3,param4,param5,param6:TSysParam):int64; external name 'FPC_DOSYS6';
function do_sysCall(sysnr,param1,param2,param3,param4,param5,param6,param7:TSysParam):int64; external name 'FPC_DOSYS7';

<<<<<<< HEAD
<<<<<<< HEAD
=======
// special
function do__sysCall(sysnr,param1,param2,param3,param4,param5,param6,param7,Param8:TSysParam):TSysResult; external name 'FPC__DOSYS';
>>>>>>> graemeg/fixes_2_2
=======
// special
function do__sysCall(sysnr,param1,param2,param3,param4,param5,param6,param7,Param8:TSysParam):TSysResult; external name 'FPC__DOSYS';
>>>>>>> origin/fixes_2_2
{$endif}

