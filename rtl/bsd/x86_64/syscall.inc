{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2004 Marco van de Voort
    member of the Free Pascal development team.

    The syscalls for the *BSD AMD64 rtl

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{
 Origin of this file: copied from linux/x86_64 dir, blended with the
        freebsd x86 changes and checked against objdump of a
        x86_64/freebsdprog
- jge directly behind the syscall to branch on non-error
- rcx is used as scratch reg (fpc/Linux-x86_64 uses edx)
- More 6 and 7 param dosyscall because of the __syscall problem
}

{$ASMMODE GAS}

function fpsysCall(sysnr:TSysParam):TSysResult; assembler;[public,alias:'FPC_DOSYS0'];

asm
  movq sysnr,%rax         { Syscall number -> rax.  }
  syscall                 { Do the system call. }
<<<<<<< HEAD
  jnb .LSyscOK            { branch to exit if ok, errorhandler otherwise}
  movq  %rax,%rdi
  call  SetErrno
=======
  jge .LSyscOK            { branch to exit if ok, errorhandler otherwise}
 movq  %rax,%rdx
{$ifdef FPC_PIC}
  movq  fpc_threadvar_relocate_proc@GOTPCREL(%rip),%rax
  movq  (%rax),%rax
  movq  Errno@GOTPCREL(%rip),%r11
{$else FPC_PIC}
  movq  fpc_threadvar_relocate_proc,%rax
  leaq  Errno,%r11
{$endif FPC_PIC}
  testq %rax,%rax
  jne   .LThread
  movl  %edx,8(%r11)
  jmp   .LNoThread
.LThread:
  pushq %rdx
  movq  (%r11),%rdi
  call  *%rax
  popq  %rdx
  movl  %edx,(%rax)
.LNoThread:
>>>>>>> graemeg/fixes_2_2
  movq  $-1,%rax
.LSyscOK:
end;

function fpsysCall(sysnr,param1 : TSysParam):TSysResult; assembler;[public,alias:'FPC_DOSYS1'];

asm
  movq sysnr,%rax         { Syscall number -> rax.  }
  movq param1,%rdi        { shift arg1 - arg1. }
  syscall                 { Do the system call. }
<<<<<<< HEAD
  jnb .LSyscOK            { branch to exit if ok, errorhandler otherwise}
  movq  %rax,%rdi
  call  SetErrno
=======
  jge .LSyscOK            { branch to exit if ok, errorhandler otherwise}
 movq  %rax,%rdx
{$ifdef FPC_PIC}
  movq  fpc_threadvar_relocate_proc@GOTPCREL(%rip),%rax
  movq  (%rax),%rax
  movq  Errno@GOTPCREL(%rip),%r11
{$else FPC_PIC}
  movq  fpc_threadvar_relocate_proc,%rax
  leaq  Errno,%r11
{$endif FPC_PIC}
  testq %rax,%rax
  jne   .LThread
  movl  %edx,8(%r11)
  jmp   .LNoThread
.LThread:
  pushq %rdx
  movq  (%r11),%rdi
  call  *%rax
  popq  %rdx
  movl  %edx,(%rax)
.LNoThread:
>>>>>>> graemeg/fixes_2_2
  movq  $-1,%rax
.LSyscOK:
end;

function fpsysCall(sysnr,param1,param2 : TSysParam):TSysResult; assembler;[public,alias:'FPC_DOSYS2'];

asm
<<<<<<< HEAD
  movq sysnr,%rax         { Syscall number -> rax.  }
  movq param1,%rdi        { shift arg1 - arg2. }
  movq param2,%rsi
  syscall                 { Do the system call. }
  jnb .LSyscOK            { branch to exit if ok, errorhandler otherwise}
  movq  %rax,%rdi
  call  SetErrno
=======
  movq sysnr, %rax        { Syscall number -> rax.  }
  movq param1, %rdi         { shift arg1 - arg2. }
  movq param2, %rsi
  syscall                 { Do the system call. }
  jge .LSyscOK            { branch to exit if ok, errorhandler otherwise}
 movq  %rax,%rdx
{$ifdef FPC_PIC}
  movq  fpc_threadvar_relocate_proc@GOTPCREL(%rip),%rax
  movq  (%rax),%rax
  movq  Errno@GOTPCREL(%rip),%r11
{$else FPC_PIC}
  movq  fpc_threadvar_relocate_proc,%rax
  leaq  Errno,%r11
{$endif FPC_PIC}
  testq %rax,%rax
  jne   .LThread
  movl  %edx,8(%r11)
  jmp   .LNoThread
.LThread:
  pushq %rdx
  movq  (%r11),%rdi
  call  *%rax
  popq  %rdx
  movl  %edx,(%rax)
.LNoThread:
>>>>>>> graemeg/fixes_2_2
  movq  $-1,%rax
.LSyscOK:
end;

function fpsysCall(sysnr,param1,param2,param3:TSysParam):TSysResult; assembler;[public,alias:'FPC_DOSYS3'];

asm
<<<<<<< HEAD
  movq sysnr,%rax         { Syscall number -> rax.  }
  movq param1,%rdi        { shift arg1 - arg3. }
  movq param2,%rsi
  movq param3,%rdx
  syscall                 { Do the system call. }
  jnb .LSyscOK            { branch to exit if ok, errorhandler otherwise}
  movq  %rax,%rdi
  call  SetErrno
=======
  movq sysnr, %rax        { Syscall number -> rax.  }
  movq param1, %rdi         { shift arg1 - arg3. }
  movq param2, %rsi
  movq param3, %rdx
  syscall                 { Do the system call. }
  jge .LSyscOK            { branch to exit if ok, errorhandler otherwise}
 movq  %rax,%rdx
{$ifdef FPC_PIC}
  movq  fpc_threadvar_relocate_proc@GOTPCREL(%rip),%rax
  movq  (%rax),%rax
  movq  Errno@GOTPCREL(%rip),%r11
{$else FPC_PIC}
  movq  fpc_threadvar_relocate_proc,%rax
  leaq  Errno,%r11
{$endif FPC_PIC}
  testq %rax,%rax
  jne   .LThread
  movl  %edx,8(%r11)
  jmp   .LNoThread
.LThread:
  pushq %rdx
  movq  (%r11),%rdi
  call  *%rax
  popq  %rdx
  movl  %edx,(%rax)
.LNoThread:
>>>>>>> graemeg/fixes_2_2
  movq  $-1,%rax
.LSyscOK:
end;

function fpsysCall(sysnr,param1,param2,param3,param4:TSysParam):TSysResult; assembler;[public,alias:'FPC_DOSYS4'];

asm
<<<<<<< HEAD
  movq sysnr,%rax         { Syscall number -> rax.  }
  movq param1,%rdi        { shift arg1 - arg5. }
  movq param2,%rsi
  movq param3,%rdx
  movq param4,%r10
  syscall                 { Do the system call. }
  jnb .LSyscOK            { branch to exit if ok, errorhandler otherwise}
  movq  %rax,%rdi
  call  SetErrno
=======
  movq sysnr, %rax        { Syscall number -> rax.  }
  movq param1, %rdi         { shift arg1 - arg5. }
  movq param2, %rsi
  movq param3, %rdx
  movq param4, %r10
  syscall                 { Do the system call. }
  jge .LSyscOK            { branch to exit if ok, errorhandler otherwise}
 movq  %rax,%rdx
{$ifdef FPC_PIC}
  movq  fpc_threadvar_relocate_proc@GOTPCREL(%rip),%rax
  movq  (%rax),%rax
  movq  Errno@GOTPCREL(%rip),%r11
{$else FPC_PIC}
  movq  fpc_threadvar_relocate_proc,%rax
  leaq  Errno,%r11
{$endif FPC_PIC}
  testq %rax,%rax
  jne   .LThread
  movl  %edx,8(%r11)
  jmp   .LNoThread
.LThread:
  pushq %rdx
  movq  (%r11),%rdi
  call  *%rax
  popq  %rdx
  movl  %edx,(%rax)
.LNoThread:
>>>>>>> graemeg/fixes_2_2
  movq  $-1,%rax
.LSyscOK:
end;

function fpsysCall(sysnr,param1,param2,param3,param4,param5 : TSysParam):TSysResult; assembler;[public,alias:'FPC_DOSYS5'];

asm
<<<<<<< HEAD
  movq sysnr,%rax         { Syscall number -> rax.  }
  movq param1,%rdi        { shift arg1 - arg5. }
  movq param2,%rsi
  movq param3,%rdx
  movq param4,%r10
  movq param5,%r8
  syscall                 { Do the system call. }
  jnb .LSyscOK            { branch to exit if ok, errorhandler otherwise}
  movq  %rax,%rdi
  call  SetErrno
=======
  movq sysnr, %rax        { Syscall number -> rax.  }
  movq param1, %rdi         { shift arg1 - arg5. }
  movq param2, %rsi
  movq param3, %rdx
  movq param4, %r10
  movq param5, %r8
  syscall                 { Do the system call. }
  jge .LSyscOK            { branch to exit if ok, errorhandler otherwise}
 movq  %rax,%rdx
{$ifdef FPC_PIC}
  movq  fpc_threadvar_relocate_proc@GOTPCREL(%rip),%rax
  movq  (%rax),%rax
  movq  Errno@GOTPCREL(%rip),%r11
{$else FPC_PIC}
  movq  fpc_threadvar_relocate_proc,%rax
  leaq  Errno,%r11
{$endif FPC_PIC}
  testq %rax,%rax
  jne   .LThread
  movl  %edx,8(%r11)
  jmp   .LNoThread
.LThread:
  pushq %rdx
  movq  (%r11),%rdi
  call  *%rax
  popq  %rdx
  movl  %edx,(%rax)
.LNoThread:
>>>>>>> graemeg/fixes_2_2
  movq  $-1,%rax
.LSyscOK:
end;

<<<<<<< HEAD
function fpsysCall(sysnr,param1,param2,param3,param4,param5,param6 : TSysParam):TSysResult; assembler;[public,alias:'FPC_DOSYS6'];

asm
  movq sysnr,%rax         { Syscall number -> rax.  }
  movq param1,%rdi        { shift arg1 - arg6. }
  movq param2,%rsi
  movq param3,%rdx
  movq param4,%r10
  movq param5,%r8
  movq param6,%r9
  syscall                 { Do the system call. }
  jnb .LSyscOK            { branch to exit if ok, errorhandler otherwise}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  movq  %rax,%rdi
  call  SetErrno
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  movq  %rax,%rdx
=======

function fpsysCall(sysnr,param1,param2,param3,param4,param5,param6 : TSysParam):TSysResult; assembler;[public,alias:'FPC_DOSYS6'];

asm
  movq sysnr, %rax        { Syscall number -> rax.  }
  movq param1, %rdi         { shift arg1 - arg6. }
  movq param2, %rsi
  movq param3, %rdx
  movq param4, %r10
  movq param5, %r8
  movq param6, %r9
  syscall                 { Do the system call. }
  jge .LSyscOK            { branch to exit if ok, errorhandler otherwise}

 movq  %rax,%rdx
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ifdef FPC_PIC}
  movq  fpc_threadvar_relocate_proc@GOTPCREL(%rip),%rax
  movq  (%rax),%rax
  movq  Errno@GOTPCREL(%rip),%r11
{$else FPC_PIC}
  movq  fpc_threadvar_relocate_proc,%rax
  leaq  Errno,%r11
{$endif FPC_PIC}
  testq %rax,%rax
  jne   .LThread
  movl  %edx,8(%r11)
  jmp   .LNoThread
.LThread:
  pushq %rdx
  movq  (%r11),%rdi
  call  *%rax
  popq  %rdx
  movl  %edx,(%rax)
.LNoThread:
>>>>>>> graemeg/cpstrnew
  movq  $-1,%rax
.LSyscOK:

end;
<<<<<<< HEAD
<<<<<<< HEAD

function fp_sysCall(sysnr,param1,param2,param3,param4,param5,param6,param7 : TSysParam):TSysResult; assembler;[public,alias:'FPC_DOSYS7'];

asm
  subq   $0x10,%rsp
  movq   sysnr,%rax       { Syscall number -> rax.  }
  movq   param1,%rdi      { shift arg1 - arg6. }
  movq   param2,%rsi
  movq   param3,%rdx
  movq   param4,%r10
  movq   param5,%r8
  movq   param6,%r9
  movq   param7,%r11
  movq   %r11,8(%rsp)
  syscall                 { Do the system call. }
  jnb .LSyscOK            { branch to exit if ok, errorhandler otherwise}
  movq  %rax,%rdi
  call  SetErrno
  movq  $-1,%rax
.LSyscOK:
  addq   $0x10,%rsp
=======
//                    edi   esi   edx 	 ecx    r8      r9   stack
function fp_sysCall(sysnr,param1,param2,param3,param4,param5,param6 : TSysParam):TSysResult; assembler;[public,alias:'FPC__DOSYS'];
=======
//                    edi   esi   edx 	 ecx    r8      r9   stack
function fp_sysCall(sysnr,param1,param2,param3,param4,param5,param6 : TSysParam):TSysResult; assembler;[public,alias:'FPC__DOSYS'];

asm
  sub    $0x18,%rsp
  movq   param6,%rax	// from caller stack to mine.	
  movq   %rax,0x8(%rsp)
  movl   $0, 0(%rsp)    // dummy or caller frame because ?
  mov    $0xc6,%rax	// __syscall
  mov    %rcx,%r10
  syscall                 { Do the system call. }
  jge .LSyscOK            { branch to exit if ok, errorhandler otherwise}
  movq  %rax,%rdx
{$ifdef FPC_PIC}
  movq  fpc_threadvar_relocate_proc@GOTPCREL(%rip),%rax
  movq  (%rax),%rax
  movq  Errno@GOTPCREL(%rip),%r11
{$else FPC_PIC}
  movq  fpc_threadvar_relocate_proc,%rax
  leaq  Errno,%r11
{$endif FPC_PIC}
  testq %rax,%rax
  jne   .LThread
  movl  %edx,8(%r11)
  jmp   .LNoThread
.LThread:
  pushq %rdx
  movq  (%r11),%rdi
  call  *%rax
  popq  %rdx
  movl  %edx,(%rax)
.LNoThread:
  movq  $-1,%rax
.LSyscOK:
  add    $0x18,%rsp
end;
>>>>>>> origin/fixes_2_2

asm
  sub    $0x18,%rsp
  movq   param6,%rax	// from caller stack to mine.	
  movq   %rax,0x8(%rsp)
  movl   $0, 0(%rsp)    // dummy or caller frame because ?
  mov    $0xc6,%rax	// __syscall
  mov    %rcx,%r10
  syscall                 { Do the system call. }
  jge .LSyscOK            { branch to exit if ok, errorhandler otherwise}
  movq  %rax,%rdx
{$ifdef FPC_PIC}
  movq  fpc_threadvar_relocate_proc@GOTPCREL(%rip),%rax
  movq  (%rax),%rax
  movq  Errno@GOTPCREL(%rip),%r11
{$else FPC_PIC}
  movq  fpc_threadvar_relocate_proc,%rax
  leaq  Errno,%r11
{$endif FPC_PIC}
  testq %rax,%rax
  jne   .LThread
  movl  %edx,8(%r11)
  jmp   .LNoThread
.LThread:
  pushq %rdx
  movq  (%r11),%rdi
  call  *%rax
  popq  %rdx
  movl  %edx,(%rax)
.LNoThread:
  movq  $-1,%rax
.LSyscOK:
  add    $0x18,%rsp
>>>>>>> graemeg/fixes_2_2
end;

