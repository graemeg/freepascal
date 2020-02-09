// Based on restoring division algorithm
// Algorithm source document: Lecture notes by S. Galal and D. Pham, Division algorithms and hardware implementations.
// Link to documentation http://www.seas.ucla.edu/~ingrid/ee213a/lectures/division_presentV2.pdf
// Also refer to description on Wikipedia: https://en.wikipedia.org/wiki/Division_algorithm#Restoring_division

// Note that the algorithm automatically yields the following results for special cases:
// z div 0 = MAX(type)
// 0 div 0 = MAX(type)
// 0 div n = 0
// Checks for z = 0; n = [0,1]; n = z and n > z could shortcut the algorithm for speed-ups
// but would add extra code
// Perhaps add the checks depending on optimization settings?

// z in Ra, n in Rb, 0 in Rp
function fpc_divmod_byte(n, z: byte): byte; assembler; nostackframe;
label
  div1, div2, div3, finish;
asm
// Symbol  Name        Register(s)
// z (A)   dividend    R22
// n (B)   divisor     R24
// p (P)   remainder   R20
// i       counter     R18
  clr R20         // clear remainder
  ldi R18, 8      // iterate over 8 bits

div1:
  lsl R22         // shift left A
  rol R20         // shift left P with carry from A shift
  sub R20, R24    // Subtract B from P, P <= P - B
  brlo div2
  ori R22, 1      // Set A[0] = 1
  rjmp div3
div2:             // negative branch, A[0] = 0 (default after shift), restore P
  add R20, R24    // restore old value of P

div3:
  dec R18
  brne div1

finish:
  mov R24, R22    // Move result from R22 to R24
end;

{$if defined(CPUAVR_16_REGS) or not(defined(CPUAVR_HAS_MOVW))}
function fpc_divmod_word(n, z: word): word; assembler; nostackframe;
label
  div1, div2, div3, finish;
asm
// Symbol  Name        Register(s)
// z (A)   dividend    R23, R22
// n (B)   divisor     R25, R24
// p (P)   remainder   R21, R20
// i       counter     R18

  clr R20         // clear remainder low
  clr R21         // clear remainder hi
  ldi R18, 16     // iterate over 16 bits

div1:
  lsl R22         // shift left A_L
  rol R23
  rol R20         // shift left P with carry from A shift
  rol R21
  sub R20, R24    // Subtract B from P, P <= P - B
  sbc R21, R25
  brlo div2
  ori R22, 1      // Set A[0] = 1
  rjmp div3
div2:             // negative branch, A[0] = 0 (default after shift), restore P
  add R20, R24    // restore old value of P
  adc R21, R25

div3:
  dec R18
  brne div1

finish:
  mov  R24, R22    // Move result from R22:R23 to R24:R25
  mov  R25, R23    // Move result from R22:R23 to R24:R25
end;

function fpc_divmod_dword(n, z: dword): dword; assembler; nostackframe;
label
  div1, div2, div3, finish;
asm
end;
{$else defined(CPUAVR_16_REGS) or not(defined(CPUAVR_HAS_MOVW))}
// z in Ra, n in Rb, 0 in Rp
function fpc_divmod_word(n, z: word): word; assembler; nostackframe;
label
  div1, div2, div3, finish;
asm
// Symbol  Name        Register(s)
// z (A)   dividend    R23, R22
// n (B)   divisor     R25, R24
// p (P)   remainder   R21, R20
// i       counter     R18

  clr R20         // clear remainder low
  clr R21         // clear remainder hi
  ldi R18, 16     // iterate over 16 bits

div1:
  lsl R22         // shift left A_L
  rol R23
  rol R20         // shift left P with carry from A shift
  rol R21
  sub R20, R24    // Subtract B from P, P <= P - B
  sbc R21, R25
  brlo div2
  ori R22, 1      // Set A[0] = 1
  rjmp div3
div2:             // negative branch, A[0] = 0 (default after shift), restore P
  add R20, R24    // restore old value of P
  adc R21, R25

div3:
  dec R18
  brne div1

finish:
  movw R24, R22    // Move result from R22:R23 to R24:R25
end;

// z in Ra, n in Rb, 0 in Rp
function fpc_divmod_dword(n, z: dword): dword; assembler; nostackframe;
label
  div1, div2, div3, finish;
asm
// Symbol  Name        Register(s)
// z (A)   dividend    R21, R20, R19, R18
// n (B)   divisor     R25, R24, R23, R22
// p (P)   remainder   R17, R16, R15, R14 -> Returned in R25, R24, R23, R22
// i       counter     R26
  push R17
  push R16
  push R15
  push R14

  clr R14         // clear remainder
  clr R15         // clear remainder
  clr R16
  clr R17
  ldi R26, 32     // iterate over 32 bits

div1:
  lsl R18         // shift left A_L
  rol R19
  rol R20
  rol R21
  rol R14         // shift left P with carry from A shift
  rol R15
  rol R16
  rol R17
  sub R14, R22    // Subtract B from P, P <= P - B
  sbc R15, R23
  sbc R16, R24
  sbc R17, R25
  brlo div2
  ori R18, 1      // Set A[0] = 1
  rjmp div3
div2:             // negative branch, A[0] = 0 (default after shift), restore P
  add R14, R22    // restore old value of P
  adc R15, R23
  adc R16, R24
  adc R17, R25

div3:
  dec R26
  brne div1

finish:
  movw R22, R14    // Move remainder into reg that is not volatile
  movw R24, R16

  pop R14
  pop R15
  pop R16
  pop R17
end;
{$endif defined(CPUAVR_16_REGS) or not(defined(CPUAVR_HAS_MOVW))}
