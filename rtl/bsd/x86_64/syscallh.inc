{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2002 Marco van de Voort
    member of the Free Pascal development team.

    x86_64 syscall headers for *BSD

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

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

{$endif}

