{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Jonas Maebe, member of the
    Free Pascal development team

    Include file with set operations called by the compiler

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef FPC_OLD_BIGENDIAN_SETS}
=======
{$ifndef FPC_NEW_BIGENDIAN_SETS}
>>>>>>> graemeg/fixes_2_2
=======
{$ifndef FPC_NEW_BIGENDIAN_SETS}
>>>>>>> origin/fixes_2_2

{$define FPC_SYSTEM_HAS_FPC_SET_LOAD_SMALL}
function fpc_set_load_small(l: fpc_small_set): fpc_normal_set;assembler;[public,alias:'FPC_SET_LOAD_SMALL']; compilerproc;
{
  load a normal set p from a smallset l

  on entry: p in r3, l in r4
}
asm
        stw     r4,0(r3)
        li      r0,0
        stw     r0,4(r3)
        stw     r0,8(r3)
        stw     r0,12(r3)
        stw     r0,16(r3)
        stw     r0,20(r3)
        stw     r0,24(r3)
        stw     r0,28(r3)
end;


{$define FPC_SYSTEM_HAS_FPC_SET_CREATE_ELEMENT}
{ checked 2001/09/28 (JM) }
function fpc_set_create_element(b : byte): fpc_normal_set;assembler;[public,alias:'FPC_SET_CREATE_ELEMENT']; compilerproc;
{
  create a new set in p from an element b

  on entry: pointer to result in r3, b in r4
}
asm
        li      r0,0
        stw     r0,0(r3)
        stw     r0,4(r3)
        stw     r0,8(r3)
        stw     r0,12(r3)
        stw     r0,16(r3)
        stw     r0,20(r3)
        stw     r0,24(r3)
        stw     r0,28(r3)

        // r0 := 1 shl r4[27-31] -> bit index in dword (rotate instructions
        // with count in register only consider lower 5 bits of this register)
        li      r0,1
        rlwnm   r0,r0,r4,0,31

        // get the index of the correct *dword* in the set
        // (((b div 8) div 4)*4= (b div 8) and not(3))
        // r5 := (r4 rotl(32-3)) and (0x01ffffff8)
        rlwinm  r4,r4,31-3+1,3,31-2
        // store the result
        stwx    r0,r3,r4
end;


{$define FPC_SYSTEM_HAS_FPC_SET_SET_BYTE}
function fpc_set_set_byte(const source: fpc_normal_set; b : byte): fpc_normal_set;assembler; compilerproc;
{
  add the element b to the set pointed by p

  on entry: result in r3, source in r4, b in r5
}
asm
       // copy source to result
       lfd      f0,0(r4)
       lfd      f1,8(r4)
       lfd      f2,16(r4)
       lfd      f3,24(r4)
       stfd     f0,0(r3)
       stfd     f1,8(r3)
       stfd     f2,16(r3)
       stfd     f3,24(r3)

       // get the index of the correct *dword* in the set
       // r0 := (r5 rotl(32-3)) and (0x0fffffff8)
       rlwinm   r0,r5,31-3+1,3,31-2
       // load dword in which the bit has to be set (and update r3 to this address)
       lwzux    r4,r3,r0
       li       r0,1
       // generate bit which has to be inserted
       // (can't use rlwimi, since that one only works for constants)
       rlwnm    r5,r0,r5,0,31
       // insert it
       or       r5,r4,r5
       // store result
       stw      r5,0(r3)
end;


{$define FPC_SYSTEM_HAS_FPC_SET_UNSET_BYTE}
function fpc_set_unset_byte(const source: fpc_normal_set; b : byte): fpc_normal_set;assembler; compilerproc;
{
  suppresses the element b to the set pointed by p
  used for exclude(set,element)

  on entry: p in r3, b in r4
}
asm
       // copy source to result
       lfd      f0,0(r4)
       lfd      f1,8(r4)
       lfd      f2,16(r4)
       lfd      f3,24(r4)
       stfd     f0,0(r3)
       stfd     f1,8(r3)
       stfd     f2,16(r3)
       stfd     f3,24(r3)
       // get the index of the correct *dword* in the set
       // r0 := (r4 rotl(32-3)) and (0x0fffffff8)
       rlwinm   r0,r5,31-3+1,3,31-2
       // load dword in which the bit has to be set (and update r3 to this address)
       lwzux    r4,r3,r0
       li       r0,1
       // generate bit which has to be removed
       rlwnm    r5,r0,r5,0,31
       // remove it
       andc     r5,r4,r5
       // store result
       stw      r4,0(r3)
end;


{$define FPC_SYSTEM_HAS_FPC_SET_SET_RANGE}
function fpc_set_set_range(const orgset: fpc_normal_set; l,h : byte): fpc_normal_set;assembler; compilerproc;
{
  on entry: result in r3, l in r4, h in r5

  on entry: result in r3, ptr to orgset in r4, l in r5, h in r6
}
asm
  // copy source to result
  lfd      f0,0(r4)
  lfd      f1,8(r4)
  lfd      f2,16(r4)
  lfd      f3,24(r4)
  stfd     f0,0(r3)
  stfd     f1,8(r3)
  stfd     f2,16(r3)
  stfd     f3,24(r3)

  cmplw  cr0,r5,r6
  bgt    cr0,.Lset_range_exit
  rlwinm r4,r5,31-3+1,3,31-2  // divide by 8 to get starting and ending byte-
  { load the set the data cache }
  dcbtst r3,r4
  rlwinm r9,r6,31-3+1,3,31-2  // address and clear two lowest bits to get
                              //  start/end longint address
  sub.   r9,r9,r4             // are bit lo and hi in the same longint?
  rlwinm r6,r6,0,31-5+1,31    // hi := hi mod 32 (= "hi and 31", but the andi
                              //  instr. only exists in flags modifying form)
  rlwinm r5,r5,0,31-5+1,31    // lo := lo mod 32 (= "lo and 31", but the andi
                              //  instr. only exists in flags modifying form)
  li     r10,-1               // r10 = $0x0ffffffff = bitmask to be inserted
  subfic r6,r6,31             // hi := 31 - (hi mod 32) = shift count for later
  slw    r10,r10,r5           // shift bitmask to clear bits below lo
  lwzux  r5,r3,r4             // go to starting pos in set and load value
                              //  (lo is not necessary anymore)
  beq    .Lset_range_hi       // if bit lo and hi in same longint, keep
                              //  current mask and adjust for hi bit
  subic. r9,r9,4              // bit hi in next longint?
  or     r5,r5,r10            // merge and
  stw    r5,0(r3)             // store current mask
  li     r10,-1               // new mask
  lwzu   r5,4(r3)             // load next longint of set
  beq    .Lset_range_hi       // bit hi in this longint -> go to adjust for hi
  subi   r3,r3,4
.Lset_range_loop:
  subic. r9,r9,4
  stwu   r10,4(r3)            // fill longints in between with full mask
  bne    .Lset_range_loop
  lwzu   r5,4(r3)             // load next value from set
.Lset_range_hi:               // in all cases, r3 here contains the address of
                              //  the longint which contains the hi bit and r4
                              //  contains this longint
  srw    r9,r10,r6            // r9 := bitmask shl (31 - (hi mod 32)) =
                              //  bitmask with bits higher than hi cleared
                              //  (r8 = $0xffffffff unless the first beq was
                              //   taken)
  and    r10,r9,r10           // combine lo and hi bitmasks for this longint
  or     r5,r5,r10            // and combine with existing set
  stw    r5,0(r3)             // store to set
.Lset_range_exit:
end;


{$define FPC_SYSTEM_HAS_FPC_SET_ADD_SETS}
function fpc_set_add_sets(const set1,set2: fpc_normal_set): fpc_normal_set;assembler;[public,alias:'FPC_SET_ADD_SETS']; compilerproc;
{
  adds set1 and set2 into set dest
  on entry: result in r3, set1 in r4, set2 in r5
}
asm
       {  load the begin of the result set in the data cache }
       dcbtst   0,r3
       li       r0,8
       mtctr    r0
       subi     r5,r5,4
       subi     r4,r4,4
       subi     r3,r3,4
   .LMADDSETS1:
      lwzu      r0,4(r4)
      lwzu      r10,4(r5)
      or        r0,r0,r10
      stwu      r0,4(r3)
      bdnz      .LMADDSETS1
end;


{$define FPC_SYSTEM_HAS_FPC_SET_MUL_SETS}
function fpc_set_mul_sets(const set1,set2: fpc_normal_set): fpc_normal_set;assembler;[public,alias:'FPC_SET_MUL_SETS']; compilerproc;
{
  multiplies (takes common elements of) set1 and set2 result put in dest
  on entry: result in r3, set1 in r4, set2 in r5
}
asm
       {  load the begin of the result set in the data cache }
       dcbtst   0,r3
       li       r0,8
       mtctr    r0
       subi     r5,r5,4
       subi     r4,r4,4
       subi     r3,r3,4
   .LMMULSETS1:
      lwzu      r0,4(r4)
      lwzu      r10,4(r5)
      and       r0,r0,r10
      stwu      r0,4(r3)
      bdnz      .LMMULSETS1
end;


{$define FPC_SYSTEM_HAS_FPC_SET_SUB_SETS}
function fpc_set_sub_sets(const set1,set2: fpc_normal_set): fpc_normal_set;assembler;[public,alias:'FPC_SET_SUB_SETS']; compilerproc;
{
  computes the diff from set1 to set2 result in dest
  on entry: result in r3, set1 in r4, set2 in r5
}
asm
       {  load the begin of the result set in the data cache }
       dcbtst   0,r3
       li       r0,8
       mtctr    r0
       subi     r5,r5,4
       subi     r4,r4,4
       subi     r3,r3,4
   .LMSUBSETS1:
      lwzu      r0,4(r4)
      lwzu      r10,4(r5)
      andc      r0,r0,r10
      stwu      r0,4(r3)
      bdnz      .LMSUBSETS1
end;


{$define FPC_SYSTEM_HAS_FPC_SET_SYMDIF_SETS}
function fpc_set_symdif_sets(const set1,set2: fpc_normal_set): fpc_normal_set;assembler;[public,alias:'FPC_SET_SYMDIF_SETS']; compilerproc;
{
   computes the symetric diff from set1 to set2 result in dest
  on entry: result in r3, set1 in r4, set2 in r5
}
asm
       {  load the begin of the result set in the data cache }
       dcbtst   0,r3
       li       r0,8
       mtctr    r0
       subi     r5,r5,4
       subi     r4,r4,4
       subi     r3,r3,4
   .LMSYMDIFSETS1:
      lwzu      r0,4(r4)
      lwzu      r10,4(r5)
      xor       r0,r0,r10
      stwu      r0,4(r3)
      bdnz      .LMSYMDIFSETS1
end;


{$define FPC_SYSTEM_HAS_FPC_SET_COMP_SETS}
function fpc_set_comp_sets(const set1,set2: fpc_normal_set): boolean;assembler;[public,alias:'FPC_SET_COMP_SETS']; compilerproc;
{
  compares set1 and set2 zeroflag is set if they are equal
  on entry: set1 in r3, set2 in r4
}
asm
       li       r0,8
       mtctr    r0
       subi     r3,r3,4
       subi     r4,r4,4
    .LMCOMPSETS1:
       lwzu     r0,4(r3)
       lwzu     r10,4(r4)
       sub.     r0,r0,r10
       bdnzt    cr0*4+eq,.LMCOMPSETS1
       cntlzw   r3,r0
       srwi.    r3,r3,5
end;


{$define FPC_SYSTEM_HAS_FPC_SET_CONTAINS_SET}
function fpc_set_contains_sets(const set1,set2: fpc_normal_set): boolean;assembler;[public,alias:'FPC_SET_CONTAINS_SETS']; compilerproc;
{
  on exit, zero flag is set if set1 <= set2 (set2 contains set1)
  on entry: set1 in r3, set2 in r4
}
asm
       li       r0,8
       mtctr    r0
       subi     r3,r3,4
       subi     r4,r4,4
    .LMCONTAINSSETS1:
       lwzu     r0,4(r3)
       lwzu     r10,4(r4)
       { set1 and not(set2) = 0? }
       andc.    r0,r0,r10
       bdnzt    cr0*4+eq,.LMCONTAINSSETS1
       cntlzw   r3,r0
       srwi.    r3,r3,5
end;

<<<<<<< HEAD
<<<<<<< HEAD
{$endif FPC_OLD_BIGENDIAN_SETS}
=======
{$endif ndef FPC_NEW_BIGENDIAN_SETS}
>>>>>>> graemeg/fixes_2_2
=======
{$endif ndef FPC_NEW_BIGENDIAN_SETS}
>>>>>>> origin/fixes_2_2
