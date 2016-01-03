{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by the Free Pascal development team

    This file contains some helper routines for int64 and qword

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$if (not defined(CPUTHUMB)) and defined(CPUARM_HAS_UMULL)}
{$define FPC_SYSTEM_HAS_MUL_QWORD}
function fpc_mul_qword(f1,f2 : qword;checkoverflow : longbool) : qword;assembler;nostackframe;[public,alias: 'FPC_MUL_QWORD']; compilerproc;
asm
  stmfd sp!,{r4,r5,r6,r14}
  mov r6,#0
  // r4 = result lo, r5 = result hi
{$ifdef ENDIAN_LITTLE}
  // lo(f1)*lo(f2)
  umull r4,r5,r0,r2
  // lo(f1)*hi(f2)
  umlal r5,r6,r0,r3
  // overflow?
  // hi(f1)*hi(f2)
  mul r0,r1,r3
  // hi(f1)*lo(f2)
  umlal r5,r6,r1,r2
  // check for overflow
  orrs r6,r6,r0
  mov r0,r4
  mov r1,r5
{$else}
  // lo(f1)*lo(f2)
  umull r4,r5,r1,r3
  // lo(f1)*hi(f2)
  umlal r5,r6,r1,r2
  // overflow?
  // hi(f1)*hi(f2)
  mul r1,r0,r2
  // hi(f1)*lo(f2)
  umlal r5,r6,r0,r3
  // check for overflow
  orrs r6,r6,r1
  mov r1,r4
  mov r0,r5
{$endif}
  // no overflow?
  beq .Lexit
  // checkoverflow?
  ldr r2,[sp,#16]
  cmp r2,#0
  beq .Lexit

  mov r0,#215
  mov r1,fp
  bl HandleErrorFrame
.Lexit:
  ldmfd sp!,{r4,r5,r6,r15}
end;
<<<<<<< HEAD
<<<<<<< HEAD
{$endif (not defined(CPUTHUMB)) and defined(CPUARM_HAS_UMULL)}
=======
=======
>>>>>>> origin/fixes_2_2

>>>>>>> graemeg/fixes_2_2

