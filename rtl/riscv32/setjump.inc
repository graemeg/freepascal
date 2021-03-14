{

    This file is part of the Free Pascal run time library.
    Copyright (c) 2008 by the Free Pascal development team.

    SetJmp and LongJmp implementation for exception handling

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

function fpc_setjmp(var S : jmp_buf) : longint;[Public, alias : 'FPC_SETJMP'];compilerproc;assembler;nostackframe;
  asm
    sw ra,   jmp_buf.ra(a0)
    sw s0,   jmp_buf.s0(a0)
    sw s1,   jmp_buf.s1(a0)
    sw s2,   jmp_buf.s2(a0)
    sw s3,   jmp_buf.s3(a0)
    sw s4,   jmp_buf.s4(a0)
    sw s5,   jmp_buf.s5(a0)
    sw s6,   jmp_buf.s6(a0)
    sw s7,   jmp_buf.s7(a0)
    sw s8,   jmp_buf.s8(a0)
    sw s9,   jmp_buf.s9(a0)
    sw s10,  jmp_buf.s10(a0)
    sw s11,  jmp_buf.s11(a0)
    sw sp,   jmp_buf.sp(a0)

{$if defined(FPUFD) or defined(FPUD)}
    frcsr t0

    sw t0,   jmp_buf.fcsr(a0)

    fsd f8,  jmp_buf.f8(a0)
    fsd f9,  jmp_buf.f9(a0)
    fsd f18, jmp_buf.f18(a0)
    fsd f19, jmp_buf.f19(a0)
    fsd f20, jmp_buf.f20(a0)
    fsd f21, jmp_buf.f21(a0)
    fsd f22, jmp_buf.f22(a0)
    fsd f23, jmp_buf.f23(a0)
    fsd f24, jmp_buf.f24(a0)
    fsd f25, jmp_buf.f25(a0)
    fsd f26, jmp_buf.f26(a0)
    fsd f27, jmp_buf.f27(a0)
{$endif FPUFD or FPUD}
{$if defined(FPUF)}
    frcsr t0

    sw t0,   jmp_buf.fcsr(a0)

    fsw f8,  jmp_buf.f8(a0)
    fsw f9,  jmp_buf.f9(a0)
    fsw f18, jmp_buf.f18(a0)
    fsw f19, jmp_buf.f19(a0)
    fsw f20, jmp_buf.f20(a0)
    fsw f21, jmp_buf.f21(a0)
    fsw f22, jmp_buf.f22(a0)
    fsw f23, jmp_buf.f23(a0)
    fsw f24, jmp_buf.f24(a0)
    fsw f25, jmp_buf.f25(a0)
    fsw f26, jmp_buf.f26(a0)
    fsw f27, jmp_buf.f27(a0)
{$endif FPUF}

    addi x10, x0, 0
end;


procedure fpc_longjmp(var S : jmp_buf;value : longint);[Public, alias : 'FPC_LONGJMP'];compilerproc;assembler;nostackframe;
  asm
    lw ra,  jmp_buf.ra(a0)
    lw s0,  jmp_buf.s0(a0)
    lw s1,  jmp_buf.s1(a0)
    lw s2,  jmp_buf.s2(a0)
    lw s3,  jmp_buf.s3(a0)
    lw s4,  jmp_buf.s4(a0)
    lw s5,  jmp_buf.s5(a0)
    lw s6,  jmp_buf.s6(a0)
    lw s7,  jmp_buf.s7(a0)
    lw s8,  jmp_buf.s8(a0)
    lw s9,  jmp_buf.s9(a0)
    lw s10, jmp_buf.s10(a0)
    lw s11, jmp_buf.s11(a0)

{$if defined(FPUFD) or defined(FPUD)}
    lw sp,   jmp_buf.fcsr(a0)

    fld f8,  jmp_buf.f8(a0)
    fld f9,  jmp_buf.f9(a0)
    fld f18, jmp_buf.f18(a0)
    fld f19, jmp_buf.f19(a0)
    fld f20, jmp_buf.f20(a0)
    fld f21, jmp_buf.f21(a0)
    fld f22, jmp_buf.f22(a0)
    fld f23, jmp_buf.f23(a0)
    fld f24, jmp_buf.f24(a0)
    fld f25, jmp_buf.f25(a0)
    fld f26, jmp_buf.f26(a0)
    fld f27, jmp_buf.f27(a0)

    fscsr sp
{$endif FPUFD or FPUD}
{$if defined(FPUF)}
    lw sp,   jmp_buf.fcsr(a0)

    flw f8,  jmp_buf.f8(a0)
    flw f9,  jmp_buf.f9(a0)
    flw f18, jmp_buf.f18(a0)
    flw f19, jmp_buf.f19(a0)
    flw f20, jmp_buf.f20(a0)
    flw f21, jmp_buf.f21(a0)
    flw f22, jmp_buf.f22(a0)
    flw f23, jmp_buf.f23(a0)
    flw f24, jmp_buf.f24(a0)
    flw f25, jmp_buf.f25(a0)
    flw f26, jmp_buf.f26(a0)
    flw f27, jmp_buf.f27(a0)

    fscsr sp
{$endif FPUF}
    lw sp, jmp_buf.sp(a0)

    beq a1, x0, .Lone
    jal x0, .Lzero
  .Lone:
    addi a0, x0, 1
    jal x0, .Lexit
  .Lzero:
    addi a0, a1, 0
  .Lexit:
  end;


