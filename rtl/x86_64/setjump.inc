{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2003 by Florian Klaempfl and other members of the
    Free Pascal development team

    SetJmp and LongJmp implementation for exception handling

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function fpc_setjmp(var S:jmp_buf):longint;assembler;[public,alias:'FPC_SETJMP'];nostackframe;compilerproc;
=======
function fpc_setjmp(var S : jmp_buf) : longint;assembler;[Public, alias : 'FPC_SETJMP'];nostackframe;compilerproc;
>>>>>>> graemeg/cpstrnew
=======
function fpc_setjmp(var S : jmp_buf) : longint;assembler;[Public, alias : 'FPC_SETJMP'];nostackframe;compilerproc;
>>>>>>> graemeg/cpstrnew
=======
function fpc_setjmp(var S : jmp_buf) : longint;assembler;[Public, alias : 'FPC_SETJMP'];nostackframe;compilerproc;
>>>>>>> graemeg/cpstrnew
=======
function fpc_setjmp(var S : jmp_buf) : longint;assembler;[Public, alias : 'FPC_SETJMP'];nostackframe;compilerproc;
>>>>>>> origin/cpstrnew
  asm
{$ifdef win64}
    // xmm6..xmm15, xmm control word and FPU control word are nonvolatile in win64.
    // Using movdqu because 16-byte aligning of local records with XMM members
    // was broken the last time I checked it (Sergei)
    movq     %rbx,jmp_buf.rbx(%rcx)
    movq     %rbp,jmp_buf.rbp(%rcx)
    movq     %r12,jmp_buf.r12(%rcx)
    movq     %r13,jmp_buf.r13(%rcx)
    movq     %r14,jmp_buf.r14(%rcx)
    movq     %r15,jmp_buf.r15(%rcx)
    movq     %rsi,jmp_buf.rsi(%rcx)
    movq     %rdi,jmp_buf.rdi(%rcx)
    leaq     8(%rsp),%rax
    movq     %rax,jmp_buf.rsp(%rcx)
    movq     (%rsp),%rax
    movq     %rax,jmp_buf.rip(%rcx)
    movdqu   %xmm6,jmp_buf.xmm6(%rcx)
    movdqu   %xmm7,jmp_buf.xmm7(%rcx)
    movdqu   %xmm8,jmp_buf.xmm8(%rcx)
    movdqu   %xmm9,jmp_buf.xmm9(%rcx)
    movdqu   %xmm10,jmp_buf.xmm10(%rcx)
    movdqu   %xmm11,jmp_buf.xmm11(%rcx)
    movdqu   %xmm12,jmp_buf.xmm12(%rcx)
    movdqu   %xmm13,jmp_buf.xmm13(%rcx)
    movdqu   %xmm14,jmp_buf.xmm14(%rcx)
    movdqu   %xmm15,jmp_buf.xmm15(%rcx)
    stmxcsr  jmp_buf.mxcsr(%rcx)
    fnstcw   jmp_buf.fpucw(%rcx)
{$else win64}
    movq     %rbx,jmp_buf.rbx(%rdi)
    movq     %rbp,jmp_buf.rbp(%rdi)
    movq     %r12,jmp_buf.r12(%rdi)
    movq     %r13,jmp_buf.r13(%rdi)
    movq     %r14,jmp_buf.r14(%rdi)
    movq     %r15,jmp_buf.r15(%rdi)
    leaq     8(%rsp),%rax
    movq     %rax,jmp_buf.rsp(%rdi)
    movq     (%rsp),%rax
    movq     %rax,jmp_buf.rip(%rdi)
{$endif win64}
    xorl     %eax,%eax
  end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure fpc_longjmp(var S:jmp_buf;value:longint);assembler;[public,alias:'FPC_LONGJMP'];nostackframe;compilerproc;
=======
procedure fpc_longjmp(var S : jmp_buf;value : longint);assembler;[Public, alias : 'FPC_LONGJMP'];compilerproc;
>>>>>>> graemeg/cpstrnew
=======
procedure fpc_longjmp(var S : jmp_buf;value : longint);assembler;[Public, alias : 'FPC_LONGJMP'];compilerproc;
>>>>>>> graemeg/cpstrnew
=======
procedure fpc_longjmp(var S : jmp_buf;value : longint);assembler;[Public, alias : 'FPC_LONGJMP'];compilerproc;
>>>>>>> graemeg/cpstrnew
=======
procedure fpc_longjmp(var S : jmp_buf;value : longint);assembler;[Public, alias : 'FPC_LONGJMP'];compilerproc;
>>>>>>> origin/cpstrnew
  asm
{$ifdef win64}
    cmpl     $1,%edx
    adcl     $0,%edx
    movl     %edx,%eax
    movq     jmp_buf.rbx(%rcx),%rbx
    movq     jmp_buf.rbp(%rcx),%rbp
    movq     jmp_buf.r12(%rcx),%r12
    movq     jmp_buf.r13(%rcx),%r13
    movq     jmp_buf.r14(%rcx),%r14
    movq     jmp_buf.r15(%rcx),%r15
    movq     jmp_buf.rsi(%rcx),%rsi
    movq     jmp_buf.rdi(%rcx),%rdi
    movq     jmp_buf.rsp(%rcx),%rsp
    movdqu   jmp_buf.xmm6(%rcx),%xmm6
    movdqu   jmp_buf.xmm7(%rcx),%xmm7
    movdqu   jmp_buf.xmm8(%rcx),%xmm8
    movdqu   jmp_buf.xmm9(%rcx),%xmm9
    movdqu   jmp_buf.xmm10(%rcx),%xmm10
    movdqu   jmp_buf.xmm11(%rcx),%xmm11
    movdqu   jmp_buf.xmm12(%rcx),%xmm12
    movdqu   jmp_buf.xmm13(%rcx),%xmm13
    movdqu   jmp_buf.xmm14(%rcx),%xmm14
    movdqu   jmp_buf.xmm15(%rcx),%xmm15
    ldmxcsr  jmp_buf.mxcsr(%rcx)
    fnclex
    fldcw    jmp_buf.fpucw(%rcx)
    jmpq     jmp_buf.rip(%rcx)
{$else win64}
    cmpl     $1,%esi
    adcl     $0,%esi
    movl     %esi,%eax
    movq     jmp_buf.rbx(%rdi),%rbx
    movq     jmp_buf.rbp(%rdi),%rbp
    movq     jmp_buf.r12(%rdi),%r12
    movq     jmp_buf.r13(%rdi),%r13
    movq     jmp_buf.r14(%rdi),%r14
    movq     jmp_buf.r15(%rdi),%r15
    movq     jmp_buf.rsp(%rdi),%rsp
    jmpq     jmp_buf.rip(%rdi)
{$endif win64}
  end;

