{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2020 by Florian Klaempfl,
    member of the Free Pascal development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{No debugging for syslinux include !}
{$IFDEF SYS_LINUX}
  {$UNDEF SYSCALL_DEBUG}
{$ENDIF SYS_LINUX}


{*****************************************************************************
                     --- Main:The System Call Self ---
*****************************************************************************}

function FpSysCall(sysnr:TSysParam):TSysResult; assembler; [public,alias:'FPC_SYSCALL0'];
{
  Perform syscall and set errno variable if needed.
}
asm
  syscall
  movi  a4,-4095
  bgeu  a2,a4,.Lnoexit
  retw
.Lnoexit:
  neg a2,a2
  call4 seterrno
end;


function FpSysCall(sysnr,param1:TSysParam):TSysResult; assembler; [public,alias:'FPC_SYSCALL1'];
{
  Perform syscall and set errno variable if needed.
}
asm
  mov a6,param1
  syscall
  movi  a4,-4095
  bgeu  a2,a4,.Lnoexit
  retw
.Lnoexit:
  neg a2,a2
  call4 seterrno
end;


function FpSysCall(sysnr,param1,param2:TSysParam):TSysResult; assembler; [public,alias:'FPC_SYSCALL2'];
{
  Perform syscall and set errno variable if needed.
}
asm
  mov a6,param1
  mov a3,param2
  syscall
  movi  a4,-4095
  bgeu  a2,a4,.Lnoexit
  retw
.Lnoexit:
  neg a2,a2
  call4 seterrno
end;


function FpSysCall(sysnr,param1,param2,param3:TSysParam):TSysResult; assembler; [public,alias:'FPC_SYSCALL3'];
{
  Perform syscall and set errno variable if needed.
}
asm
  mov a6,param1
  mov a3,param2
  mov a4,param3
  syscall
  movi  a4,-4095
  bgeu  a2,a4,.Lnoexit
  retw
.Lnoexit:
  neg a2,a2
  call4 seterrno
end;


function FpSysCall(sysnr,param1,param2,param3,param4:TSysParam):TSysResult; assembler; [public,alias:'FPC_SYSCALL4'];
{
  Perform syscall and set errno variable if needed.
}
asm
  mov a9,param1
  mov a3,param2
  mov a4,param3
  mov a5,param4
  mov a6,a9
  syscall
  movi  a4,-4095
  bgeu  a2,a4,.Lnoexit
  retw
.Lnoexit:
  neg a2,a2
  call4 seterrno
end;

function FpSysCall(sysnr,param1,param2,param3,param4,param5:TSysParam):TSysResult; assembler; [public,alias:'FPC_SYSCALL5'];
{
  Perform syscall and set errno variable if needed.
}
asm
  mov a9,param1
  mov a3,param2
  mov a4,param3
  mov a5,param4
  mov a8,param5
  mov a6,a9
  syscall
  movi  a4,-4095
  bgeu  a2,a4,.Lnoexit
  retw
.Lnoexit:
  neg a2,a2
  call4 seterrno
end;


function FpSysCall(sysnr,param1,param2,param3,param4,param5,param6:TSysParam):TSysResult; assembler; [public,alias:'FPC_SYSCALL6'];
{
  Perform syscall and set errno variable if needed.
}
asm
  mov a9,param1
  mov a3,param2
  mov a4,param3
  mov a5,param4
  mov a8,param5
  mov a6,a9
  l32i a9,param6
  syscall
  movi  a4,-4095
  bgeu  a2,a4,.Lnoexit
  retw
.Lnoexit:
  neg a2,a2
  call4 seterrno
end;
