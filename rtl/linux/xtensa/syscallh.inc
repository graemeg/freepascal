{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2002 Marco van de Voort
    member of the Free Pascal development team.

    Xtensa syscall headers for Linux

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

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
