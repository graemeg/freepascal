{
  This file is part of the Free Pascal run time library.

  Perform syscall with 0..6 arguments.
  If syscall return value is negative, negate it, set errno, and return -1.

  Written by Edmund Grimley Evans in 2015 and released into the public domain.
}

function FpSysCall(sysnr:TSysParam):TSysResult;
assembler; nostackframe; [public,alias:'FPC_DOSYS0'];
asm
  mov w8,w0
  svc #0
  b.cc .Ldone
  str x30,[sp,#-16]!
  bl seterrno
  ldr x30,[sp],#16
  mov x0,#-1
.Ldone:
end;

function FpSysCall(sysnr,param1:TSysParam):TSysResult;
assembler; nostackframe; [public,alias:'FPC_DOSYS1'];
asm
  mov w8,w0
  mov x0,x1
  svc #0
  b.cc .Ldone
  str x30,[sp,#-16]!
  bl seterrno
  ldr x30,[sp],#16
  mov x0,#-1
.Ldone:
end;

function FpSysCall(sysnr,param1,param2:TSysParam):TSysResult;
assembler; nostackframe; [public,alias:'FPC_DOSYS2'];
asm
  mov w8,w0
  mov x0,x1
  mov x1,x2
  svc #0
  b.cc .Ldone
  str x30,[sp,#-16]!
  bl seterrno
  ldr x30,[sp],#16
  mov x0,#-1
.Ldone:
end;

function FpSysCall(sysnr,param1,param2,param3:TSysParam):TSysResult;
assembler; nostackframe; [public,alias:'FPC_DOSYS3'];
asm
  mov w8,w0
  mov x0,x1
  mov x1,x2
  mov x2,x3
  svc #0
  b.cc .Ldone
  str x30,[sp,#-16]!
  bl seterrno
  ldr x30,[sp],#16
  mov x0,#-1
.Ldone:
end;

function FpSysCall(sysnr,param1,param2,param3,param4:TSysParam):TSysResult;
assembler; nostackframe; [public,alias:'FPC_DOSYS4'];
asm
  mov w8,w0
  mov x0,x1
  mov x1,x2
  mov x2,x3
  mov x3,x4
  svc #0
  b.cc .Ldone
  str x30,[sp,#-16]!
  bl seterrno
  ldr x30,[sp],#16
  mov x0,#-1
.Ldone:
end;

function FpSysCall(sysnr,param1,param2,param3,param4,param5:TSysParam):TSysResult;
assembler; nostackframe; [public,alias:'FPC_DOSYS5'];
asm
  mov w8,w0
  mov x0,x1
  mov x1,x2
  mov x2,x3
  mov x3,x4
  mov x4,x5
  svc #0
  b.cc .Ldone
  str x30,[sp,#-16]!
  bl seterrno
  ldr x30,[sp],#16
  mov x0,#-1
.Ldone:
end;

function FpSysCall(sysnr,param1,param2,param3,param4,param5,param6:TSysParam):TSysResult;
assembler; nostackframe; [public,alias:'FPC_DOSYS6'];
asm
  mov w8,w0
  mov x0,x1
  mov x1,x2
  mov x2,x3
  mov x3,x4
  mov x4,x5
  mov x5,x6
  svc #0
  b.cc .Ldone
  str x30,[sp,#-16]!
  bl seterrno
  ldr x30,[sp],#16
  mov x0,#-1
.Ldone:
end;

function FpSysCall(sysnr,param1,param2,param3,param4,param5,param6,param7:TSysParam):TSysResult;
assembler; nostackframe; [public,alias:'FPC_DOSYS7'];
asm
  mov w8,w0
  mov x0,x1
  mov x1,x2
  mov x2,x3
  mov x3,x4
  mov x4,x5
  mov x5,x6
  mov x6,x7
  svc #0
  b.cc .Ldone
  str x30,[sp,#-16]!
  bl seterrno
  ldr x30,[sp],#16
  mov x0,#-1
.Ldone:
end;
