{
    Conversion code from various number formats to QL Float format.

    Code ported from the C68/QL-GCC libc implementation available at:
    http://morloch.hd.free.fr/qdos/qdosgcc.html

    The QL wiki claims the original of these sources are by
    Dave Walker, and they are in the Public Domain.
    https://qlwiki.qlforum.co.uk/doku.php?id=qlwiki:c68

 **********************************************************************}
unit qlfloat;

interface

uses
  qdos;

function longint_to_qlfp(qlf: Pqlfloat; val: longint): Pqlfloat;
function double_to_qlfp(qlf: Pqlfloat; val: Pdouble): Pqlfloat;


implementation

function longint_to_qlfp(qlf: Pqlfloat; val: longint): Pqlfloat; assembler; nostackframe;
asm
  { pointer to qlfloat is in a0 }
  { val is in d0 }

  movem.l d2-d4/a0,-(sp)  { save register variables and a0 }
  moveq.l #0,d2           { sign value }
  move.l  d2,d3           { shift value }
  tst.l   d0              { zero or -ve ? }
  beq     @zeroval        { zero }
  bpl     @plusval        { +ve }

{ i is negative here. set the sign value then make i positive }

  moveq   #1,d2           { boolean to say -ve }
  not.l   d0              { i has all bits reversed }
  bne     @plusval        { i was not -1, so can continue }

{ i was -1, so cannot go into following loop, as it now is zero }

  moveq   #0,d2           { pretend i was positive }
  move.l  #$80000000,d1   { set d1 correctly }
  move.w  #31,d3          { shift value }
  bra     @outloop        { continue }

@plusval:
  move.l  d0,d1           { save a copy of the original i }

{ check for shortcuts with shifts }

  and.l   #$ffffff00,d0   { shift by 23 ? }
  bne     @bigger23       { no cheat available }
  move.w  #23,d3          { shift value is 23 }
  lsl.l   d3,d1           { shift copy of i }
  bra     @nbigger        { continue }

{ check for 15 bit shortcut shift }

@bigger23:
  move.l  d1,d0           { restore i }
  and.l   #$ffff0000,d0   { shift by 15 ? }
  bne     @nbigger        { no cheat available }
  move.w  #15,d3          { shift value is 15 }
  lsl.l   d3,d1           { shift copy of i }

{ no shortcuts available }

@nbigger:
  move.l  d1,d0           { restore i }
  and.l   #$40000000,d0   { if(!(i & 0x40000000)) }
  bne     @outloop        { bit is set, no more shifts }
  lsl.l   #1,d1           { shift copy of i }
  addq.l  #1,d3           { increment shift count }
  bra     @nbigger        { ensures i is restored }

{ finished shifts - copy into qlfloat }
{ correct shifted i is in d1, d0 contains i & 0x40000000 }

@outloop:
  move.w  #$81f,d4
  sub.w   d3,d4           { set exponent correctly }
  move.w  d4,(a0)+        { copy into exponent }

{ difference here between positive and negative numbers
; negative should just be shifted until first zero, so as we
; have 2s complemented and shifted until first one, we must now
; re-complement what is left }

  tst.b   d2
  beq     @setmant        { positive value here - just copy it }

{ negative value, xor it with -1 shifted by same amount as in shift (d3)
; to convert it back to -ve representation }

  moveq.l #-1,d2          { set d2 to all $FFs }
  lsl.l   d3,d2           { shift it by shift (d3 ) }
  eor.l   d2,d1           { not the value by xoring }

{ negative value restored by above }

@setmant:
  move.l  d1,(a0)         { copy into mantissa }
@fin:
  movem.l (sp)+,d2-d4/a0  { reset register variables and return value }
  rts

{ quick exit if zero }

@zeroval:
  move.w  d2,(a0)+        { zero exponent }
  move.l  d2,(a0)         { zero mantissa }
  bra     @fin
end;


function double_to_qlfp(qlf: Pqlfloat; val: Pdouble): Pqlfloat; assembler; nostackframe;
asm
{----------------------------- IEEE -----------------------------------
; routine to convert IEEE double precision (8 byte) floating point
; to a QLFLOAT_t.
}
  { pointer to qlfloat is in a0 }
  move.l  (a1),d0        { high long of IEEE double }

{ SNG - avoid loading low part for now so we can treat D1 as temporary }

  add.l   d0,d0          { Put sign bit in carry }
  lsr.l   #1,d0          { put zero where sign was }
  bne     @notzero       { not zero }
  move.l  4(a1),d1       { Test low bits too (probably zero!) }
  bne     @notzero

{ here the double was a signed zero - set the QLFLOAT_t and return }

  move.w  d1,(a0)+       { We know that D1 is 0 at this point }
  bra     @positive

{ was not zero - do manipulations }

@notzero:
  move.l  d0,d1          { set non-signed high part copy }
{                          We are going to lose least significant byte so we
;                          can afford to over-write it.  We can thus take
;                          advantage that the shift size when specified in
;                          a register is modulo 64 }
  move.b  #20,d0         { shift amount for exponent }
  lsr.l   d0,d0          { get exponent - tricky but it works! }
  add.w   #$402,d0       { adjust to QLFLOAT_t exponent }
  move.w  d0,(a0)+       { set QLFLOAT_t exponent }

{ now deal with mantissa }

  and.l   #$fffff,d1     { get top 20 mantissa bits }
  or.l    #$100000,d1    { add implied bit }
  moveq   #10,d0         { shift amount ;; save another 2 code bytes }
  lsl.l   d0,d1          { shift top 21 bits into place }

  move.l  4(a1),d0       { get less significant bits }

{                          We are going to lose least significant byte so we
;                          can afford to over-write it.  We can thus take
;                          advantage that the shift size when specified in
;                          a register is modulo 64 }
  move.b  #22,d0         { amount to shift down low long: not MOVEQ! }
  lsr.l   d0,d0          { position low 10 bits of mantissa }
  or.l    d0,d1          { D1 now positive mantissa }

@lowzer:
  tst.b   (a1)           { Top byte of IEEE argument }
  bpl     @positive      { No need to negate if positive }
  neg.l   d1             { Mantissa in D1 now }
@positive:
  move.l  d1,(a0)        { put mantissa in QLFLOAT_t }
  subq.l  #2,a0          { correct for return address }
  move.l  a0,d0          { set return value as original QLFLOAT_t address }
end;

end.
