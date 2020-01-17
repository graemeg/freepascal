{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2002 Marco van de Voort
    member of the Free Pascal development team.

    m68k syscall headers for Linux 

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


Type
  TSysResult = longint;
  TSysParam  = Longint;

function Do_SysCall(sysnr:TSysParam):TSysResult; external name 'FPC_SYSCALL0';
function Do_SysCall(sysnr,param1:TSysParam):TSysResult; external name 'FPC_SYSCALL1';
function Do_SysCall(sysnr,param1,param2:TSysParam):TSysResult; external name 'FPC_SYSCALL2';
function Do_SysCall(sysnr,param1,param2,param3:TSysParam):TSysResult; external name 'FPC_SYSCALL3';
function Do_SysCall(sysnr,param1,param2,param3,param4:TSysParam):TSysResult; external name 'FPC_SYSCALL4';
function Do_SysCall(sysnr,param1,param2,param3,param4,param5:TSysParam):TSysResult; external name 'FPC_SYSCALL5';
function Do_SysCall(sysnr,param1,param2,param3,param4,param5,param6:TSysParam):TSysResult; external name 'FPC_SYSCALL6';

