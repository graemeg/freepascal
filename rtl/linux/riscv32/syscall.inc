{
  This file is part of the Free Pascal run time library.

  Perform syscall with 0..6 arguments.
  If syscall return value is negative, negate it, set errno, and return -1.

  Written by Edmund Grimley Evans in 2015 and released into the public domain.
}

function FpSysCall(sysnr:TSysParam):TSysResult;
assembler; nostackframe; [public,alias:'FPC_SYSCALL0'];
asm
  addi x17, sysnr, 0
  ecall
  bge x10,x0,.Ldone
  sw x1, -4(x2)
  addi x2, x2, -4
  sub x10, x0, x10
  jal x1, seterrno
  addi x2, x2, 4
  lw x1, -4(x2)
  addi x10,x0, -1
.Ldone:
end;

function FpSysCall(sysnr,param1:TSysParam):TSysResult;
assembler; nostackframe; [public,alias:'FPC_SYSCALL1'];
asm
  addi x17, sysnr, 0
  addi x10, x11, 0
  ecall
  bge x10,x0,.Ldone
  sw x1, -4(x2)
  addi x2, x2, -4
  sub x10, x0, x10
  jal x1, seterrno
  addi x2, x2, 4
  lw x1, -4(x2)
  addi x10,x0, -1
.Ldone:
end;

function FpSysCall(sysnr,param1,param2:TSysParam):TSysResult;
assembler; nostackframe; [public,alias:'FPC_SYSCALL2'];
asm
  addi x17, sysnr, 0
  addi x10, x11, 0
  addi x11, x12, 0
  ecall
  bge x10,x0,.Ldone
  sw x1, -4(x2)
  addi x2, x2, -4
  sub x10, x0, x10
  jal x1, seterrno
  addi x2, x2, 4
  lw x1, -4(x2)
  addi x10,x0, -1
.Ldone:
end;

function FpSysCall(sysnr,param1,param2,param3:TSysParam):TSysResult;
assembler; nostackframe; [public,alias:'FPC_SYSCALL3'];
asm
  addi x17, sysnr, 0
  addi x10, x11, 0
  addi x11, x12, 0
  addi x12, x13, 0
  ecall
  bge x10,x0,.Ldone
  sw x1, -4(x2)
  addi x2, x2, -4
  sub x10, x0, x10
  jal x1, seterrno
  addi x2, x2, 4
  lw x1, -4(x2)
  addi x10,x0, -1
.Ldone:
end;

function FpSysCall(sysnr,param1,param2,param3,param4:TSysParam):TSysResult;
assembler; nostackframe; [public,alias:'FPC_SYSCALL4'];
asm
  addi x17, sysnr, 0
  addi x10, x11, 0
  addi x11, x12, 0
  addi x12, x13, 0
  addi x13, x14, 0
  ecall
  bge x10,x0,.Ldone
  sw x1, -4(x2)
  addi x2, x2, -4
  sub x10, x0, x10
  jal x1, seterrno
  addi x2, x2, 4
  lw x1, -4(x2)
  addi x10,x0, -1
.Ldone:
end;

function FpSysCall(sysnr,param1,param2,param3,param4,param5:TSysParam):TSysResult;
assembler; nostackframe; [public,alias:'FPC_SYSCALL5'];
asm
  addi x17, sysnr, 0
  addi x10, x11, 0
  addi x11, x12, 0
  addi x12, x13, 0
  addi x13, x14, 0
  addi x14, x15, 0
  ecall
  bge x10,x0,.Ldone
  sw x1, -4(x2)
  addi x2, x2, -4
  sub x10, x0, x10
  jal x1, seterrno
  addi x2, x2, 4
  lw x1, -4(x2)
  addi x10,x0, -1
.Ldone:
end;

function FpSysCall(sysnr,param1,param2,param3,param4,param5,param6:TSysParam):TSysResult;
assembler; nostackframe; [public,alias:'FPC_SYSCALL6'];
asm
  addi x17, sysnr, 0
  addi x10, x11, 0
  addi x11, x12, 0
  addi x12, x13, 0
  addi x13, x14, 0
  addi x14, x15, 0
  addi x15, x16, 0
  ecall
  bge x10,x0,.Ldone
  sw x1, -4(x2)
  addi x2, x2, -4
  sub x10, x0, x10
  jal x1, seterrno
  addi x2, x2, 4
  lw x1, -4(x2)
  addi x10,x0, -1
.Ldone:
end;
