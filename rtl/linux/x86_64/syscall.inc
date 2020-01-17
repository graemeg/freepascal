{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Michael Van Canneyt,
    member of the Free Pascal development team.

    The syscalls for the new RTL, moved to platform dependant dir.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$ASMMODE GAS}

{$ifdef FPC_PROFILE}
procedure mcount; external name 'mcount';
{$endif FPC_PROFILE}

function FpSysCall(sysnr:TSysParam):TSysResult; assembler;[public,alias:'FPC_SYSCALL0'];

asm
{$ifdef FPC_PROFILE}
  pushq sysnr
  call mcount
  popq sysnr
{$endif FPC_PROFILE}
  movq sysnr, %rax        { Syscall number -> rax.  }
  syscall                 { Do the system call. }
  cmpq $-4095, %rax       { Check %rax for error.  }
  jnae .LSyscOK           { Jump to error handler if error.  }
  negq  %rax
  movq  %rax,%rdi
  call  seterrno@PLT
  movq  $-1,%rax
.LSyscOK:
end;

function FpSysCall(sysnr,param1 : TSysParam):TSysResult; assembler;[public,alias:'FPC_SYSCALL1'];

asm
{$ifdef FPC_PROFILE}
  pushq sysnr
  pushq param1
  call mcount
  popq param1
  popq sysnr
{$endif FPC_PROFILE}
  movq sysnr, %rax        { Syscall number -> rax.  }
  movq param1, %rdi         { shift arg1 - arg5. }
  syscall                 { Do the system call. }
  cmpq $-4095, %rax       { Check %rax for error.  }
  jnae .LSyscOK           { Jump to error handler if error.  }
  negq  %rax
  movq  %rax,%rdi
  call  seterrno@PLT
  movq  $-1,%rax
.LSyscOK:
end;

function FpSysCall(sysnr,param1,param2 : TSysParam):TSysResult; assembler;[public,alias:'FPC_SYSCALL2'];

asm
{$ifdef FPC_PROFILE}
  pushq sysnr
  pushq param1
  pushq param2
  call mcount
  popq param2
  popq param1
  popq sysnr
{$endif FPC_PROFILE}
  movq sysnr, %rax        { Syscall number -> rax.  }
  movq param1, %rdi         { shift arg1 - arg5. }
  movq param2, %rsi
  syscall                 { Do the system call. }
  cmpq $-4095, %rax       { Check %rax for error.  }
  jnae .LSyscOK           { Jump to error handler if error.  }
  negq  %rax
  movq  %rax,%rdi
  call  seterrno@PLT
  movq  $-1,%rax
.LSyscOK:
end;

function FpSysCall(sysnr,param1,param2,param3:TSysParam):TSysResult; assembler;[public,alias:'FPC_SYSCALL3'];

asm
{$ifdef FPC_PROFILE}
  pushq sysnr
  pushq param1
  pushq param2
  pushq param3
  call mcount
  popq param3
  popq param2
  popq param1
  popq sysnr
{$endif FPC_PROFILE}
  movq sysnr, %rax        { Syscall number -> rax.  }
  movq param1, %rdi         { shift arg1 - arg5. }
  movq param2, %rsi
  movq param3, %rdx
  syscall                 { Do the system call. }
  cmpq $-4095, %rax       { Check %rax for error.  }
  jnae .LSyscOK           { Jump to error handler if error.  }
  negq  %rax
  movq  %rax,%rdi
  call  seterrno@PLT
  movq  $-1,%rax
.LSyscOK:
end;

function FpSysCall(sysnr,param1,param2,param3,param4:TSysParam):TSysResult; assembler;[public,alias:'FPC_SYSCALL4'];

asm
{$ifdef FPC_PROFILE}
  pushq sysnr
  pushq param1
  pushq param2
  pushq param3
  pushq param4
  call mcount
  popq param4
  popq param3
  popq param2
  popq param1
  popq sysnr
{$endif FPC_PROFILE}
  movq sysnr, %rax        { Syscall number -> rax.  }
  movq param1, %rdi         { shift arg1 - arg5. }
  movq param2, %rsi
  movq param3, %rdx
  movq param4, %r10
  syscall                 { Do the system call. }
  cmpq $-4095, %rax       { Check %rax for error.  }
  jnae .LSyscOK           { Jump to error handler if error.  }
  negq  %rax
  movq  %rax,%rdi
  call  seterrno@PLT
  movq  $-1,%rax
.LSyscOK:
end;

function FpSysCall(sysnr,param1,param2,param3,param4,param5 : TSysParam):TSysResult; assembler;[public,alias:'FPC_SYSCALL5'];

asm
{$ifdef FPC_PROFILE}
  pushq sysnr
  pushq param1
  pushq param2
  pushq param3
  pushq param4
  pushq param5
  call mcount
  popq param5
  popq param4
  popq param3
  popq param2
  popq param1
  popq sysnr
{$endif FPC_PROFILE}
  movq sysnr, %rax        { Syscall number -> rax.  }
  movq param1, %rdi         { shift arg1 - arg5. }
  movq param2, %rsi
  movq param3, %rdx
  movq param4, %r10
  movq param5, %r8
  syscall                 { Do the system call. }
  cmpq $-4095, %rax       { Check %rax for error.  }
  jnae .LSyscOK           { Jump to error handler if error.  }
  negq  %rax
  movq  %rax,%rdi
  call  seterrno@PLT
  movq  $-1,%rax
.LSyscOK:
end;


function FpSysCall(sysnr,param1,param2,param3,param4,param5,param6 : TSysParam):TSysResult; assembler;[public,alias:'FPC_SYSCALL6'];

asm
{$ifdef FPC_PROFILE}
  pushq sysnr
  pushq param1
  pushq param2
  pushq param3
  pushq param4
  pushq param5
  pushq param6
  call mcount
  popq param6
  popq param5
  popq param4
  popq param3
  popq param2
  popq param1
  popq sysnr
{$endif FPC_PROFILE}
  movq sysnr, %rax        { Syscall number -> rax.  }
  movq param1, %rdi         { shift arg1 - arg5. }
  movq param2, %rsi
  movq param3, %rdx
  movq param4, %r10
  movq param5, %r8
  movq param6, %r9
  syscall                 { Do the system call. }
  cmpq $-4095, %rax       { Check %rax for error.  }
  jnae .LSyscOK           { Jump to error handler if error.  }
  negq  %rax
  movq  %rax,%rdi
  call  seterrno@PLT
  movq  $-1,%rax
.LSyscOK:
end;


{No debugging for syslinux include !}
{$IFDEF SYS_LINUX}
  {$UNDEF SYSCALL_DEBUG}
{$ENDIF SYS_LINUX}

