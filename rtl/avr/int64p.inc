{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2008 by the Free Pascal development team

    This file contains some helper routines for int64 and qword

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$ifndef CPUAVR_16_REGS}
{$define FPC_SYSTEM_HAS_SHR_QWORD}
// Simplistic version with checking if whole bytes can be shifted
// Doesn't change bitshift portion even if possible because of byteshift
// Shorter code but not shortest execution time version
function fpc_shr_qword(value: qword; shift: ALUUInt): qword; assembler; nostackframe;
  [public, alias: 'FPC_SHR_QWORD']; compilerproc;
label
  byteshift, bitshift, finish;
asm
// value passed in R25...R18
// shift passed in R16
// return value in R25...R18

  push R16

  andi R16, 63    // mask 64 bit relevant value per generic routine
byteshift:
  breq finish     // shift = 0, finished
  cpi R16, 8      // Check if shift is at least a byte
  brlo bitshift
  mov R18, R19    // if so, then shift all bytes right by 1 position
  mov R19, R20
  mov R20, R21
  mov R21, R22
  mov R22, R23
  mov R23, R24
  mov R24, R25
  clr R25         // and clear the high byte
  subi R16, 8     // subtract 8 bits from shift
  rjmp byteshift  // check if another byte can be shifted

bitshift:         // shift all 8 bytes right by 1 bit
  lsr R25
  ror R24
  ror R23
  ror R22
  ror R21
  ror R20
  ror R19
  ror R18

  dec R16
  brne bitshift   // until R16 = 0

finish:
  pop R16
end;
function fpc_shr_qword(value: qword; shift: ALUUInt): qword; external name 'FPC_SHR_QWORD';

{$define FPC_SYSTEM_HAS_SHL_QWORD}
function fpc_shl_qword(value: qword; shift: ALUUInt): qword; assembler; nostackframe;
[public, alias: 'FPC_SHL_QWORD']; compilerproc;
label
  byteshift, bitshift, finish;
asm
// value passed in R25...R18
// shift passed in R16
// return value in R25...R18
  push R16

  andi R16, 63    // mask 64 bit relevant value per generic routine
byteshift:
  breq finish     // shift = 0, finished
  cpi R16, 8      // Check if shift is at least a byte
  brlo bitshift
  mov R25, R24    // if so, then shift all bytes left by 1 position
  mov R24, R23
  mov R23, R22
  mov R22, R21
  mov R21, R20
  mov R20, R19
  mov R19, R18
  clr R18         // and clear the high byte
  subi R16, 8     // subtract 8 bits from shift
  rjmp byteshift  // check if another byte can be shifted

bitshift:         // shift all 8 bytes left by 1 bit
  lsl R18
  rol R19
  rol R20
  rol R21
  rol R22
  rol R23
  rol R24
  rol R25

  dec R16
  brne bitshift   // until R16 = 0

finish:
  pop R16
end;

function fpc_shl_qword(value: qword; shift: ALUUInt): qword; external name 'FPC_SHL_QWORD';

{$define FPC_SYSTEM_HAS_SHL_INT64}
function fpc_shl_int64(value: int64; shift: ALUUInt): int64;
  [public, alias: 'FPC_SHL_INT64']; compilerproc; inline;
begin
  Result := fpc_shl_qword(qword(value), shift);
end;

{$define FPC_SYSTEM_HAS_SHR_INT64}
// shr of signed int is same as shr of unsigned int (logical shift right)
function fpc_shr_int64(value: int64; shift: ALUUInt): int64; [public, alias: 'FPC_SHR_INT64']; compilerproc;
begin
  Result := fpc_shr_qword(qword(value), shift);
end;

{$define FPC_SYSTEM_HAS_DIV_QWORD}
function fpc_div_qword(n,z : qword): qword; nostackframe; assembler; [public,alias: 'FPC_DIV_QWORD']; compilerproc;
label
  start, div1, div2, div3, finish;
asm
// Symbol  Name        Register(s)
// z (A)   dividend    R17, R16, R15, R14, R13, R12, R11, R10
// n (B)   divisor     R25, R24, R23, R22, R21, R20, R19, R18
// r (P)   remainder   R9,  R8,  R7,  R6,  R5,  R4,  R3,  R2
// i       counter     R26
//         1           R27

  cp R25, R1
  cpc R24, R1
  cpc R23, R1
  cpc R22, R1
  cpc R21, R1
  cpc R20, R1
  cpc R19, R1
  cpc R18, R1

  brne .LNonZero
{$ifdef CPUAVR_HAS_JMP_CALL}
  call fpc_divbyzero
{$else  CPUAVR_HAS_JMP_CALL}
  rcall fpc_divbyzero
{$endif CPUAVR_HAS_JMP_CALL}

.LNonZero:

  push R17
  push R16
  push R15
  push R14
  push R13
  push R12
  push R11
  push R10
  push R9
  push R8
  push R7
  push R6
  push R5
  push R4
  push R3
  push R2

  ldi R27, 1      // needed below for OR instruction

start:            // Start of division...
  clr R9          // clear remainder
  clr R8
  clr R7
  clr R6
  clr R5
  clr R4
  clr R3
  clr R2
  ldi R26, 64     // iterate over 64 bits

div1:
  lsl R10         // shift left A_L
  rol R11
  rol R12
  rol R13
  rol R14
  rol R15
  rol R16
  rol R17

  rol R2          // shift left P with carry from A shift
  rol R3
  rol R4
  rol R5
  rol R6
  rol R7
  rol R8
  rol R9

  sub R2, R18     // Subtract B from P, P <= P - B
  sbc R3, R19
  sbc R4, R20
  sbc R5, R21
  sbc R6, R22
  sbc R7, R23
  sbc R8, R24
  sbc R9, R25

  brlo div2
  or R10, R27     // Set A[0] = 1
  rjmp div3
div2:             // negative branch, A[0] = 0 (default after shift), restore P

  add R2, R18     // restore old value of P
  adc R3, R19
  adc R4, R20
  adc R5, R21
  adc R6, R22
  adc R7, R23
  adc R8, R24
  adc R9, R25

div3:
  dec R26
  breq finish
  rjmp div1

finish:
  mov R25, R17    // Move answer from R17..10 to R25..18
  mov R24, R16
  mov R23, R15
  mov R22, R14
  mov R21, R13
  mov R20, R12
  mov R19, R11
  mov R18, R10

  pop R2
  pop R3
  pop R4
  pop R5
  pop R6
  pop R7
  pop R8
  pop R9
  pop R10
  pop R11
  pop R12
  pop R13
  pop R14
  pop R15
  pop R16
  pop R17
end;
function fpc_div_qword(n,z : qword): qword; external name 'FPC_DIV_QWORD';

{$define FPC_SYSTEM_HAS_MOD_QWORD}
function fpc_mod_qword(n,z : qword): qword; nostackframe; assembler; [public,alias: 'FPC_MOD_QWORD']; compilerproc;
label
  start, div1, div2, div3, finish;
asm
// Symbol  Name        Register(s)
// z (A)   dividend    R17, R16, R15, R14, R13, R12, R11, R10
// n (B)   divisor     R25, R24, R23, R22, R21, R20, R19, R18
// r (P)   remainder   R9,  R8,  R7,  R6,  R5,  R4,  R3,  R2
// i	   counter     R26
//         1           R27

  cp R25, R1
  cpc R24, R1
  cpc R23, R1
  cpc R22, R1
  cpc R21, R1
  cpc R20, R1
  cpc R19, R1
  cpc R18, R1

  brne .LNonZero
{$ifdef CPUAVR_HAS_JMP_CALL}
  call fpc_divbyzero
{$else  CPUAVR_HAS_JMP_CALL}
  rcall fpc_divbyzero
{$endif CPUAVR_HAS_JMP_CALL}

.LNonZero:

  push R17
  push R16
  push R15
  push R14
  push R13
  push R12
  push R11
  push R10
  push R9
  push R8
  push R7
  push R6
  push R5
  push R4
  push R3
  push R2

  ldi R27, 1
start:            // Start of division...
  clr R9          // clear remainder
  clr R8
  clr R7
  clr R6
  clr R5
  clr R4
  clr R3
  clr R2
  ldi R26, 64     // iterate over 64 bits

div1:
  lsl R10         // shift left A_L
  rol R11
  rol R12
  rol R13
  rol R14
  rol R15
  rol R16
  rol R17

  rol R2          // shift left P with carry from A shift
  rol R3
  rol R4
  rol R5
  rol R6
  rol R7
  rol R8
  rol R9

  sub R2, R18     // Subtract B from P, P <= P - B
  sbc R3, R19
  sbc R4, R20
  sbc R5, R21
  sbc R6, R22
  sbc R7, R23
  sbc R8, R24
  sbc R9, R25

  brlo div2
  or R10, R27     // Set A[0] = 1
  rjmp div3
div2:             // negative branch, A[0] = 0 (default after shift), restore P

  add R2, R18     // restore old value of P
  adc R3, R19
  adc R4, R20
  adc R5, R21
  adc R6, R22
  adc R7, R23
  adc R8, R24
  adc R9, R25

div3:
  dec R26
  breq finish
  rjmp div1

finish:
  mov R25, R9     // Move answer from R9..2 to R25..18
  mov R24, R8
  mov R23, R7
  mov R22, R6
  mov R21, R5
  mov R20, R4
  mov R19, R3
  mov R18, R2

  pop R2
  pop R3
  pop R4
  pop R5
  pop R6
  pop R7
  pop R8
  pop R9
  pop R10
  pop R11
  pop R12
  pop R13
  pop R14
  pop R15
  pop R16
  pop R17
end;
function fpc_mod_qword(n,z : qword): qword; external name 'FPC_MOD_QWORD';


{$define FPC_SYSTEM_HAS_DIV_INT64}
function fpc_div_int64(n,z : int64) : int64; nostackframe; assembler; [public,alias: 'FPC_DIV_INT64']; compilerproc;
label
  pos1, pos2, fin;
asm
// Convert n, z to unsigned int, then call div_qword,
// Restore sign if high bits of n xor z is negative
// n       divisor     R25, R24, R23, R22, R21, R20, R19, R18
// z       dividend    R17, R16, R15, R14, R13, R12, R11, R10
//         neg_result  R30
//         one         R31

  mov R30, R17    // store hi8(z)
  eor R30, R25  // hi8(z) XOR hi8(n), answer must be negative if MSB set

  // convert n to absolute
  ldi R31, 1      // 1 in R31 used later
  sub R25, r1     // subtract 0, just to check sign flag
  brpl pos1
  com R25
  com R24
  com R23
  com R22
  com R21
  com R20
  com R19
  com R18
  add R18, R31    // add 1
  adc R19, R1     // add carry bit
  adc R20, R1
  adc R21, R1
  adc R22, R1
  adc R23, R1
  adc R24, R1
  adc R25, R1
  pos1:

  sub R17, R1
  brpl pos2
  com R17
  com R16
  com R15
  com R14
  com R13
  com R12
  com R11
  com R10
  add R10, R31
  adc R11, R1
  adc R12, R1
  adc R13, R1
  adc R14, R1
  adc R15, R1
  adc R16, R1
  adc R17, R1
  pos2:

{$ifdef CPUAVR_HAS_JMP_CALL}
  call fpc_div_qword
{$else  CPUAVR_HAS_JMP_CALL}
  rcall fpc_div_qword
{$endif CPUAVR_HAS_JMP_CALL}

  sbrs R30, 7     // skip if bit 7 is cleared (result should be positive)
  rjmp fin
  com R25         // result from FPC_DIV_WORD in R25 ... R22
  com R24
  com R23
  com R22
  com R21
  com R20
  com R19
  com R18

  ldi R31, 1
  add R18, R31    // add 1
  adc R19, R1     // add carry bit
  adc R20, R1
  adc R21, R1
  adc R22, R1
  adc R23, R1
  adc R24, R1
  adc R25, R1
  fin:
end;

{$define FPC_SYSTEM_HAS_MOD_INT64}
function fpc_mod_int64(n,z : int64) : int64; nostackframe; assembler; [public,alias: 'FPC_MOD_INT64']; compilerproc;
label
  pos1, pos2, fin;
asm
// Convert n, z to unsigned int, then call mod_qword,
// Restore sign if high bits of n xor z is negative
// n       divisor     R25, R24, R23, R22, R21, R20, R19, R18
// z       dividend    R17, R16, R15, R14, R13, R12, R11, R10
//         neg_result  R30
//         one         R31

  mov R30, R17  // store hi8(z)

  // convert n to absolute
  ldi R31, 1
  sub R25, r1     // subtract 0, just to check sign flag
  brpl pos1
  com R25
  com R24
  com R23
  com R22
  com R21
  com R20
  com R19
  com R18
  add R18, R31    // add 1
  adc R19, R1     // add carry bit
  adc R20, R1
  adc R21, R1
  adc R22, R1
  adc R23, R1
  adc R24, R1
  adc R25, R1
  pos1:

  sub R17, R1
  brpl pos2
  com R17
  com R16
  com R15
  com R14
  com R13
  com R12
  com R11
  com R10
  add R10, R31
  adc R11, R1
  adc R12, R1
  adc R13, R1
  adc R14, R1
  adc R15, R1
  adc R16, R1
  adc R17, R1
  pos2:

{$ifdef CPUAVR_HAS_JMP_CALL}
  call fpc_mod_qword
{$else  CPUAVR_HAS_JMP_CALL}
  rcall fpc_mod_qword
{$endif CPUAVR_HAS_JMP_CALL}

  sbrs R30, 7     // Not finished if sign bit is set
  rjmp fin
  com R25         // Convert to 2's complement
  com R24         // Complement all bits...
  com R23
  com R22
  com R21
  com R20
  com R19
  com R18
  ldi R31, 1
  add R18, R31    // ...and add 1 to answer
  adc R19, R1
  adc R20, R1
  adc R21, R1
  adc R22, R1
  adc R23, R1
  adc R24, R1
  adc R25, R1
  fin:
end;
{$endif CPUAVR_16_REGS}
