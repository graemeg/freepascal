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
    sd ra,   0*8(a0)
    sd s0,   1*8(a0)
    sd s1,   2*8(a0)
    sd s2,   3*8(a0)
    sd s3,   4*8(a0)
    sd s4,   5*8(a0)
    sd s5,   6*8(a0)
    sd s6,   7*8(a0)
    sd s7,   8*8(a0)
    sd s8,   9*8(a0)
    sd s9,  10*8(a0)
    sd s10, 11*8(a0)
    sd s11, 12*8(a0)
    sd sp,  13*8(a0)

{$if defined(FPUFD) or defined(FPUD)}
    frcsr t0

    sd t0,   14*8(a0)

    fsd f8,  15*8(a0)
    fsd f9,  16*8(a0)
    fsd f18, 17*8(a0)
    fsd f19, 18*8(a0)
    fsd f20, 19*8(a0)
    fsd f21, 20*8(a0)
    fsd f22, 21*8(a0)
    fsd f23, 22*8(a0)
    fsd f24, 23*8(a0)
    fsd f25, 24*8(a0)
    fsd f26, 25*8(a0)
    fsd f27, 26*8(a0)
{$endif FPUFD or FPUD}
{$if defined(FPUF)}
    frcsr t0

    sd t0,   14*8(a0)

    fsw f8,  30*4(a0)
    fsw f9,  31*4(a0)
    fsw f18, 32*4(a0)
    fsw f19, 33*4(a0)
    fsw f20, 34*4(a0)
    fsw f21, 35*4(a0)
    fsw f22, 36*4(a0)
    fsw f23, 37*4(a0)
    fsw f24, 38*4(a0)
    fsw f25, 39*4(a0)
    fsw f26, 40*4(a0)
    fsw f27, 41*4(a0)
{$endif FPUF}

    addi x10, x0, 0
  end;


procedure fpc_longjmp(var S : jmp_buf;value : longint);[Public, alias : 'FPC_LONGJMP'];compilerproc;assembler;nostackframe;
  asm
    ld ra,  0*8(a0)
    ld s0,  1*8(a0)
    ld s1,  2*8(a0)
    ld s2,  3*8(a0)
    ld s3,  4*8(a0)
    ld s4,  5*8(a0)
    ld s5,  6*8(a0)
    ld s6,  7*8(a0)
    ld s7,  8*8(a0)
    ld s8,  9*8(a0)
    ld s9, 10*8(a0)
    ld s10,11*8(a0)
    ld s11,12*8(a0)

{$if defined(FPUFD) or defined(FPUD)}
    ld sp,   14*8(a0)

    fld f8,  15*8(a0)
    fld f9,  16*8(a0)
    fld f18, 17*8(a0)
    fld f19, 18*8(a0)
    fld f20, 19*8(a0)
    fld f21, 20*8(a0)
    fld f22, 21*8(a0)
    fld f23, 22*8(a0)
    fld f24, 23*8(a0)
    fld f25, 24*8(a0)
    fld f26, 25*8(a0)
    fld f27, 26*8(a0)

    fscsr sp
{$endif FPUFD or FPUD}
{$if defined(FPUF)}
    ld sp,   14*8(a0)

    flw f8,  30*4(a0)
    flw f9,  31*4(a0)
    flw f18, 32*4(a0)
    flw f19, 33*4(a0)
    flw f20, 34*4(a0)
    flw f21, 35*4(a0)
    flw f22, 36*4(a0)
    flw f23, 37*4(a0)
    flw f24, 38*4(a0)
    flw f25, 39*4(a0)
    flw f26, 40*4(a0)
    flw f27, 41*4(a0)

    fscsr sp
{$endif FPUF}
    ld sp, 13*8(a0)

    beq a1, x0, .Lone
    jal x0, .Lzero
  .Lone:
    addi a0, x0, 1
    jal x0, .Lexit
  .Lzero:
    addi a0, a1, 0
  .Lexit:
  end;

