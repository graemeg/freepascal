{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by xxxx
    member of the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{**********************************************************************
           Set_Jmp/Long_jmp
 **********************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$warning Fix register handling in case of nostackframe }

Function fpc_SetJmp (Var S : Jmp_buf) : longint;assembler;nostackframe;[Public, alias : 'FPC_SETJMP'];compilerproc;
=======
Function fpc_SetJmp (Var S : Jmp_buf) : longint;assembler;[Public, alias : 'FPC_SETJMP'];compilerproc;
>>>>>>> graemeg/cpstrnew
=======
Function fpc_SetJmp (Var S : Jmp_buf) : longint;assembler;[Public, alias : 'FPC_SETJMP'];compilerproc;
>>>>>>> graemeg/cpstrnew
=======
Function fpc_SetJmp (Var S : Jmp_buf) : longint;assembler;[Public, alias : 'FPC_SETJMP'];compilerproc;
>>>>>>> graemeg/cpstrnew
=======
Function fpc_SetJmp (Var S : Jmp_buf) : longint;assembler;[Public, alias : 'FPC_SETJMP'];compilerproc;
>>>>>>> origin/cpstrnew
asm
  // Temporarily store a0 into d0
  move.l a0,d0
  // load S to a0
  move.l 4(sp),a0

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  // Save data registers d1..d7
  movem.l d1/d2/d3/d4/d5/d6/d7,12(a0)
  // Save address registers (a0-a5/a6, a0 is in d0 now)
{$if defined(amiga)}
  movem.l d0/a1/a2/a3/a4/a6,40(a0) { amiga uses a5 as fp }
{$else}
  movem.l d0/a1/a2/a3/a4/a5,40(a0)
{$endif}

  // save return address (PC) and pop S off the stack
  move.l (sp)+,d0
  move.l d0,(sp)
  move.l d0,8(a0)
  // save FP
  move.l fp,(a0)
  // save SP
  move.l sp,d0
  // 4 bytes already popped, 4 to go.
  addq.l #4,d0
  move.l d0,4(a0)
  // restore a0
  move.l 40(a0),a0

  // return 0
  clr.l d0
end;

Procedure fpc_longJmp (Var S : Jmp_buf; value : longint); assembler;nostackframe;[Public, alias : 'FPC_LONGJMP'];compilerproc;
asm
  // load S to a0
  move.l 4(sp),a0
  // load 'value' to d0
  move.l 8(sp),d0
  // don't return zero
  tst.l  d0
  seq    d1
  and.l #1,d1
  or.l   d1,d0
  // restore FP
  move.l (a0),fp
  // restore SP
  move.l 4(a0),sp
  // jump to PC
  move.l 8(a0),-(sp)

  // Restore data registers
  movem.l 12(a0),d1/d2/d3/d4/d5/d6/d7

  // Restore address registers
{$if defined(amiga)}
  movem.l 40(a0),a0/a1/a2/a3/a4/a6   { amiga uses a5 as fp }
{$else}
  movem.l 40(a0),a0/a1/a2/a3/a4/a5
{$endif}

  // new return pc is at (sp)
end;
=======
Procedure fpc_longJmp (Var S : Jmp_buf; value : longint); assembler;[Public, alias : 'FPC_LONGJMP'];compilerproc;
asm
end;
>>>>>>> graemeg/cpstrnew
=======
Procedure fpc_longJmp (Var S : Jmp_buf; value : longint); assembler;[Public, alias : 'FPC_LONGJMP'];compilerproc;
asm
end;
>>>>>>> graemeg/cpstrnew
=======
Procedure fpc_longJmp (Var S : Jmp_buf; value : longint); assembler;[Public, alias : 'FPC_LONGJMP'];compilerproc;
asm
end;
>>>>>>> graemeg/cpstrnew
=======
Procedure fpc_longJmp (Var S : Jmp_buf; value : longint); assembler;[Public, alias : 'FPC_LONGJMP'];compilerproc;
asm
end;
>>>>>>> origin/cpstrnew
