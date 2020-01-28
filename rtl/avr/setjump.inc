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

function fpc_setjmp(var S : jmp_buf) : shortint;assembler;[Public, alias : 'FPC_SETJMP'];nostackframe;compilerproc;
  asm
    mov r26,r24
    mov r27,r25
{$ifndef CPUAVR_16_REGS}   
    st x+,r1
    st x+,r2
    st x+,r3
    st x+,r4
    st x+,r5
    st x+,r6
    st x+,r7
    st x+,r8
    st x+,r9
    st x+,r10
    st x+,r11
    st x+,r12
    st x+,r13
    st x+,r14
    st x+,r15
{$endif CPUAVR_16_REGS}       
    st x+,r16
    st x+,r17
{$ifdef CPUAVR_16_REGS}   
    st x+,r18
    st x+,r19
{$endif CPUAVR_16_REGS}   
    st x+,r28
    st x+,r29

    // store stack pointer
    in	r18,61
    in	r19,62
    st x+,r18
    st x+,r19

    // store return address
    pop r18
    pop r19
    st x+,r18
    st x+,r19
{$ifdef CPUAVR_3_BYTE_PC}
    pop r20
    st x+,r20
    push r20
{$endif CPUAVR_3_BYTE_PC}
    push r19
    push r18

    clr r24
  end;


procedure fpc_longjmp(var S : jmp_buf;value : shortint);assembler;[Public, alias : 'FPC_LONGJMP'];compilerproc;
  asm
    mov r26,r24
    mov r27,r25
{$ifndef CPUAVR_16_REGS}   
    ld r1,x+
    ld r2,x+
    ld r3,x+
    ld r4,x+
    ld r5,x+
    ld r6,x+
    ld r7,x+
    ld r8,x+
    ld r9,x+
    ld r10,x+
    ld r11,x+
    ld r12,x+
    ld r13,x+
    ld r14,x+
    ld r15,x+
{$endif CPUAVR_16_REGS}       
    ld r16,x+
    ld r17,x+
{$ifdef CPUAVR_16_REGS}   
    ld r18,x+
    ld r19,x+
{$endif CPUAVR_16_REGS}   
    ld r28,x+
    ld r29,x+

    // restore stack pointer
    ld r18,x+
    ld r19,x+
{$ifdef CPUAVR_16_REGS}   
    in r16,63
{$else CPUAVR_16_REGS}   
    in r0,63
{$endif CPUAVR_16_REGS}   
    cli
    out 62,r19
{$ifdef CPUAVR_16_REGS}   
    out	63,r16
{$else CPUAVR_16_REGS}   
    out	63,r0
{$endif CPUAVR_16_REGS}   
    out	61,r18

    // restore return address
    pop r18
    pop r19
    ld r18,x+
    ld r19,x+
{$ifdef CPUAVR_3_BYTE_PC}
    pop r20
    ld r20,x+
    push r20
{$endif CPUAVR_3_BYTE_PC}
    push r19
    push r18
    mov r24,r22
  end;


