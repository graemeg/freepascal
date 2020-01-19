{

    This file is part of the Free Pascal run time library.
    Copyright (c) 2008 by the Free Pascal development team.

    SetJmp/Longjmp declarations

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

type
   jmp_buf = packed record
{$ifndef CPUAVR_16_REGS}   
     r1,r2,r3,r4,r5,r6,r7,r8,r9,r10,r11,r12,r13,r14,r15,
{$endif CPUAVR_16_REGS}   
     r16,r17,
{$ifdef CPUAVR_16_REGS}   
     r18,r19,
{$endif CPUAVR_16_REGS}   
     r28,r29,splo,sphi,pclo,pchi : byte;
{$ifdef CPUAVR_3_BYTE_PC}
     pchighest : byte
{$endif CPUAVR_3_BYTE_PC}
   end;
   pjmp_buf = ^jmp_buf;

function setjmp(var S : jmp_buf) : shortint;[external name 'FPC_SETJMP'];
procedure longjmp(var S : jmp_buf;value : shortint);[external name 'FPC_LONGJMP'];


