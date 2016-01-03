{

    This file is part of the Free Pascal run time library.
    Copyright (c) 2000-2006 by the Free Pascal development team.

    Portions Copyright (c) 2000 by Casey Duncan (casey.duncan@state.co.us)

    Processor dependent implementation for the system unit for
    PowerPC

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$IFNDEF LINUX}
{$IFNDEF MORPHOS}
    {$DEFINE USE_DCBZ}
{$ENDIF MORPHOS}
{$ENDIF LINUX}

{****************************************************************************
                           PowerPC specific stuff
****************************************************************************}
{

const
  ppc_fpu_overflow     = (1 shl (32-3));
  ppc_fpu_underflow    = (1 shl (32-4));
  ppc_fpu_divbyzero    = (1 shl (32-5));
  ppc_fpu_inexact      = (1 shl (32-6));
  ppc_fpu_invalid_snan = (1 shl (32-7));
}

{$ifndef FPUNONE}

procedure fpc_enable_ppc_fpu_exceptions;
assembler; nostackframe;
asm
  { clear all "exception happened" flags we care about}
  mtfsfi 0,0
  mtfsfi 1,0
  mtfsfi 2,0
  mtfsfi 3,0
  mtfsb0 21
  mtfsb0 22
  mtfsb0 23

  { enable invalid operations and division by zero exceptions. }
  { No overflow/underflow, since those give some spurious      }
  { exceptions                                                 }
  mtfsfi 6,9
end;

procedure fpc_cpuinit;
begin
  { don't let libraries influence the FPU cw set by the host program }
  if not IsLibrary then
    fpc_enable_ppc_fpu_exceptions;
end;


function fpc_get_ppc_fpscr: cardinal;
assembler;
var
  temp: record a,b:longint; end;
asm
  mffs f0
  stfd f0,temp
  lwz  r3,temp.b
  { clear all exception flags }
{
  rlwinm r4,r3,0,16,31
  stw  r4,temp.b
  lfd  f0,temp
  a_mtfsf f0
}
end;

{ This function is never called directly, it's a dummy to hold the register save/
  load subroutines
}
{$ifndef MACOS}
label
  _restfpr_14_x,
  _restfpr_15_x,
  _restfpr_16_x,
  _restfpr_17_x,
  _restfpr_18_x,
  _restfpr_19_x,
  _restfpr_20_x,
  _restfpr_21_x,
  _restfpr_22_x,
  _restfpr_23_x,
  _restfpr_24_x,
  _restfpr_25_x,
  _restfpr_26_x,
  _restfpr_27_x,
  _restfpr_28_x,
  _restfpr_29_x,
  _restfpr_30_x,
  _restfpr_31_x,
  _restfpr_14_l,
  _restfpr_15_l,
  _restfpr_16_l,
  _restfpr_17_l,
  _restfpr_18_l,
  _restfpr_19_l,
  _restfpr_20_l,
  _restfpr_21_l,
  _restfpr_22_l,
  _restfpr_23_l,
  _restfpr_24_l,
  _restfpr_25_l,
  _restfpr_26_l,
  _restfpr_27_l,
  _restfpr_28_l,
  _restfpr_29_l,
  _restfpr_30_l,
  _restfpr_31_l;

procedure saverestorereg;assembler; nostackframe;
asm
{ exit }
.globl _restfpr_14_x
_restfpr_14_x:  lfd     f14, -144(r11)
.globl _restfpr_15_x
_restfpr_15_x:  lfd     f15, -136(r11)
.globl _restfpr_16_x
_restfpr_16_x:  lfd     f16, -128(r11)
.globl _restfpr_17_x
_restfpr_17_x:  lfd     f17, -120(r11)
.globl _restfpr_18_x
_restfpr_18_x:  lfd     f18, -112(r11)
.globl _restfpr_19_x
_restfpr_19_x:  lfd     f19, -104(r11)
.globl _restfpr_20_x
_restfpr_20_x:  lfd     f20, -96(r11)
.globl _restfpr_21_x
_restfpr_21_x:  lfd     f21, -88(r11)
.globl _restfpr_22_x
_restfpr_22_x:  lfd     f22, -80(r11)
.globl _restfpr_23_x
_restfpr_23_x:  lfd     f23, -72(r11)
.globl _restfpr_24_x
_restfpr_24_x:  lfd     f24, -64(r11)
.globl _restfpr_25_x
_restfpr_25_x:  lfd     f25, -56(r11)
.globl _restfpr_26_x
_restfpr_26_x:  lfd     f26, -48(r11)
.globl _restfpr_27_x
_restfpr_27_x:  lfd     f27, -40(r11)
.globl _restfpr_28_x
_restfpr_28_x:  lfd     f28, -32(r11)
.globl _restfpr_29_x
_restfpr_29_x:  lfd     f29, -24(r11)
.globl _restfpr_30_x
_restfpr_30_x:  lfd     f30, -16(r11)
.globl _restfpr_31_x
_restfpr_31_x:  lwz     r0, 4(r11)
                lfd     f31, -8(r11)
                mtlr    r0
                ori     r1, r11, 0
                blr

{ exit with restoring lr }
.globl _restfpr_14_l
_restfpr_14_l:  lfd     f14, -144(r11)
.globl _restfpr_15_l
_restfpr_15_l:  lfd     f15, -136(r11)
.globl _restfpr_16_l
_restfpr_16_l:  lfd     f16, -128(r11)
.globl _restfpr_17_l
_restfpr_17_l:  lfd     f17, -120(r11)
.globl _restfpr_18_l
_restfpr_18_l:  lfd     f18, -112(r11)
.globl _restfpr_19_l
_restfpr_19_l:  lfd     f19, -104(r11)
.globl _restfpr_20_l
_restfpr_20_l:  lfd     f20, -96(r11)
.globl _restfpr_21_l
_restfpr_21_l:  lfd     f21, -88(r11)
.globl _restfpr_22_l
_restfpr_22_l:  lfd     f22, -80(r11)
.globl _restfpr_23_l
_restfpr_23_l:  lfd     f23, -72(r11)
.globl _restfpr_24_l
_restfpr_24_l:  lfd     f24, -64(r11)
.globl _restfpr_25_l
_restfpr_25_l:  lfd     f25, -56(r11)
.globl _restfpr_26_l
_restfpr_26_l:  lfd     f26, -48(r11)
.globl _restfpr_27_l
_restfpr_27_l:  lfd     f27, -40(r11)
.globl _restfpr_28_l
_restfpr_28_l:  lfd     f28, -32(r11)
.globl _restfpr_29_l
_restfpr_29_l:  lfd     f29, -24(r11)
.globl _restfpr_30_l
_restfpr_30_l:  lfd     f30, -16(r11)
.globl _restfpr_31_l
_restfpr_31_l:  lwz     r0, 4(r11)
                lfd     f31, -8(r11)
                mtlr    r0
                ori     r1, r11, 0
                blr
end;
{$endif MACOS}

{$else}

procedure fpc_cpuinit;
begin
end;

{$endif}

{****************************************************************************
                                Move / Fill
****************************************************************************}

{$ifndef FPC_SYSTEM_HAS_MOVE}
{$define FPC_SYSTEM_HAS_MOVE}
procedure Move(const source;var dest;count:longint);[public, alias: 'FPC_MOVE'];assembler; nostackframe;
asm
          {  count <= 0 ?  }
          cmpwi   cr0,r5,0
          {  check if we have to do the move backwards because of overlap  }
          sub     r10,r4,r3
          {  carry := boolean(dest-source < count) = boolean(overlap) }
          subc    r10,r10,r5

          {  count < 15 ? (to decide whether we will move dwords or bytes  }
          cmpwi   cr1,r5,15

          {  if overlap, then r10 := -1 else r10 := 0  }
          subfe   r10,r10,r10

          {  count < 63 ? (32 + max. alignment (31) }
          cmpwi   cr7,r5,63

          {  if count <= 0, stop  }
          ble     cr0,.LMoveDone

          {  load the begin of the source in the data cache }
          dcbt    0,r3
          { and the dest as well }
          dcbtst  0,r4

          {  if overlap, then r0 := count else r0 := 0  }
          and     r0,r5,r10
          {  if overlap, then point source and dest to the end  }
          add     r3,r3,r0
          add     r4,r4,r0
          {  if overlap, then r6 := 0, else r6 := -1  }
          not     r6,r10
          {  if overlap, then r10 := -2, else r10 := 0  }
          slwi    r10,r10,1
          {  if overlap, then r10 := -1, else r10 := 1  }
          addi    r10,r10,1

          {  if count < 15, copy everything byte by byte  }
          blt     cr1,.LMoveBytes

          {  if no overlap, then source/dest += -1, otherwise they stay }
          {  After the next instruction, r3/r4 + r10 = next position to }
          {  load/store from/to                                         }
          add     r3,r3,r6
          add     r4,r4,r6

          {  otherwise, guarantee 4 byte alignment for dest for starters  }
.LMove4ByteAlignLoop:
          lbzux   r0,r3,r10
          stbux   r0,r4,r10
          {  is dest now 4 aligned?  }
          andi.   r0,r4,3
          subi    r5,r5,1
          {  while not aligned, continue  }
          bne     cr0,.LMove4ByteAlignLoop

{$ifndef ppc603}
          { check for 32 byte alignment }
          andi.   r7,r4,31
{$endif non ppc603}
          { we are going to copy one byte again (the one at the newly }
          { aligned address), so increase count byte 1                }
          addi    r5,r5,1
          { count div 4 for number of dwords to copy }
          srwi    r0,r5,2
          {  if 11 <= count < 63, copy using dwords }
          blt     cr7,.LMoveDWords

{$ifndef ppc603}
          { # of dwords to copy to reach 32 byte alignment (*4) }
          { (depends on forward/backward copy)                  }

          { if forward copy, r6 = -1 -> r8 := 32 }
          { if backward copy, r6 = 0 -> r8 := 0  }
          rlwinm  r8,r6,0,31-6+1,31-6+1
          { if forward copy, we have to copy 32 - unaligned count bytes }
          { if backward copy unaligned count bytes                      }
          sub     r7,r8,r7
          { if backward copy, the calculated value is now negate -> }
          { make it positive again                                 }
          not     r8, r6
          add     r7, r7, r8
          xor     r7, r7, r8
{$endif not ppc603}

          { multiply the update count with 4 }
          slwi    r10,r10,2
          slwi    r6,r6,2
          { and adapt the source and dest }
          add     r3,r3,r6
          add     r4,r4,r6

{$ifndef ppc603}
          beq     cr0,.LMove32BytesAligned
.L32BytesAlignMoveLoop:
          {  count >= 39 -> align to 8 byte boundary and then use the FPU  }
          {  since we're already at 4 byte alignment, use dword store      }
          subic.  r7,r7,4
          lwzux   r0,r3,r10
          subi    r5,r5,4
          stwux   r0,r4,r10
          bne     .L32BytesAlignMoveLoop

.LMove32BytesAligned:
          { count div 32 ( >= 1, since count was >=63 }
          srwi    r0,r5,5
          { remainder }
          andi.   r5,r5,31
          { to decide if we will do some dword stores (instead of only }
          { byte stores) afterwards or not                             }
{$else not ppc603}
          srwi    r0,r5,4
          andi.   r5,r5,15
{$endif not ppc603}
          cmpwi   cr1,r5,11
          mtctr   r0

          {  r0 := count div 4, will be moved to ctr when copying dwords  }
          srwi    r0,r5,2

{$if not defined(ppc603) and not defined(FPUNONE)}
          {  adjust the update count: it will now be 8 or -8 depending on overlap  }
          slwi    r10,r10,1

          {  adjust source and dest pointers: because of the above loop, dest is now   }
          {  aligned to 8 bytes. So if we add r6 we will still have an 8 bytes         }
          { aligned address)                                                           }
          add     r3,r3,r6
          add     r4,r4,r6

          slwi    r6,r6,1
{$IFDEF USE_DCBZ}
          { the dcbz offset must give a 32 byte aligned address when added   }
          { to the current dest address and its address must point to the    }
          { bytes that will be overwritten in the current iteration. In case }
          { of a forward loop, the dest address has currently an offset of   }
          { -8 compared to the bytes that will be overwritten (and r6 = -8). }
          { In case of a backward of a loop, the dest address currently has  }
          { an offset of +32 compared to the bytes that will be overwritten  }
          { (and r6 = 0). So the forward dcbz offset must become +8 and the  }
          { backward -32 -> (-r6 * 5) - 32 gives the correct offset          }
          slwi    r7,r6,2
          add     r7,r7,r6
          neg     r7,r7
          subi    r7,r7,32
{$ENDIF USE_DCBZ}
.LMove32ByteDcbz:
          lfdux   f0,r3,r10
          lfdux   f1,r3,r10
          lfdux   f2,r3,r10
          lfdux   f3,r3,r10
{$IFDEF USE_DCBZ}
          { must be done only now, in case source and dest are less than }
          { 32 bytes apart!                                              }
          dcbz    r4,r7
{$ENDIF USE_DCBZ}
          stfdux  f0,r4,r10
          stfdux  f1,r4,r10
          stfdux  f2,r4,r10
          stfdux  f3,r4,r10
          bdnz    .LMove32ByteDcbz
.LMove32ByteLoopDone:
{$else not ppc603}
.LMove16ByteLoop:
          lwzux   r11,r3,r10
          lwzux   r7,r3,r10
          lwzux   r8,r3,r10
          lwzux   r9,r3,r10
          stwux   r11,r4,r10
          stwux   r7,r4,r10
          stwux   r8,r4,r10
          stwux   r9,r4,r10
          bdnz    .LMove16ByteLoop
{$endif not ppc603}

          { cr0*4+eq is true if "count and 31" = 0 }
          beq     cr0,.LMoveDone

          {  make r10 again -1 or 1, but first adjust source/dest pointers }
          sub     r3,r3,r6
          sub     r4,r4,r6
{$ifndef ppc603}
          srawi   r10,r10,3
          srawi   r6,r6,3
{$else not ppc603}
          srawi   r10,r10,2
          srawi   r6,r6,2
{$endif not ppc603}

          { cr1 contains whether count <= 11 }
          ble     cr1,.LMoveBytes

.LMoveDWords:
          mtctr   r0
          andi.   r5,r5,3
          {  r10 * 4  }
          slwi    r10,r10,2
          slwi    r6,r6,2
          add     r3,r3,r6
          add     r4,r4,r6

.LMoveDWordsLoop:
          lwzux   r0,r3,r10
          stwux   r0,r4,r10
          bdnz    .LMoveDWordsLoop

          beq     cr0,.LMoveDone
          {  make r10 again -1 or 1  }
          sub     r3,r3,r6
          sub     r4,r4,r6
          srawi   r10,r10,2
          srawi   r6,r6,2
.LMoveBytes:
          add     r3,r3,r6
          add     r4,r4,r6
          mtctr   r5
.LMoveBytesLoop:
          lbzux   r0,r3,r10
          stbux   r0,r4,r10
          bdnz    .LMoveBytesLoop
.LMoveDone:
end;
{$endif FPC_SYSTEM_HAS_MOVE}


{$ifndef FPC_SYSTEM_HAS_FILLCHAR}
{$define FPC_SYSTEM_HAS_FILLCHAR}

Procedure FillChar(var x;count:longint;value:byte);assembler;
{ input: x in r3, count in r4, value in r5 }

{$ifndef FPC_ABI_AIX}
{ in the AIX ABI, we can use te red zone for temp storage, otherwise we have }
{ to explicitely allocate room                                               }
var
  temp : packed record
    case byte of
      0: (l1,l2: longint);
{$ifndef FPUNONE}
      1: (d: double);
{$endif}
    end;
{$endif FPC_ABI_AIX}
asm
        { no bytes? }
        cmpwi     cr6,r4,0
        { less than 15 bytes? }
        cmpwi     cr7,r4,15
        { less than 64 bytes? }
        cmpwi     cr1,r4,64
        { fill r5 with ValueValueValueValue }
        rlwimi    r5,r5,8,16,23
        { setup for aligning x to multiple of 4}
        rlwinm    r10,r3,0,31-2+1,31
        rlwimi    r5,r5,16,0,15
        ble       cr6,.LFillCharDone
        { get the start of the data in the cache (and mark it as "will be }
        { modified")                                                      }
        dcbtst    0,r3
        subfic    r10,r10,4
        blt       cr7,.LFillCharVerySmall
        { just store 4 bytes instead of using a loop to align (there are }
        { plenty of other instructions now to keep the processor busy    }
        { while it handles the (possibly unaligned) store)               }
        stw       r5,0(r3)
        { r3 := align(r3,4) }
        add       r3,r3,r10
        { decrease count with number of bytes already stored }
        sub       r4,r4,r10
{$IFNDEF FPUNONE}
        blt       cr1,.LFillCharSmall
{$IFDEF USE_DCBZ}
        { if we have to fill with 0 (which happens a lot), we can simply use }
        { dcbz for the most part, which is very fast, so make a special case }
        { for that                                                           }
        cmplwi    cr1,r5,0
{$ENDIF}
        { align to a multiple of 32 (and immediately check whether we aren't }
        { already 32 byte aligned)                                           }
        rlwinm.   r10,r3,0,31-5+1,31
        { setup r3 for using update forms of store instructions }
        subi      r3,r3,4
        { get number of bytes to store }
        subfic    r10,r10,32
        { if already 32byte aligned, skip align loop }
        beq       .L32ByteAlignLoopDone
        { substract from the total count }
        sub       r4,r4,r10
.L32ByteAlignLoop:
        { we were already aligned to 4 byres, so this will count down to }
        { exactly 0                                                      }
        subic.    r10,r10,4
        stwu      r5,4(r3)
        bne       .L32ByteAlignLoop
.L32ByteAlignLoopDone:
        { get the amount of 32 byte blocks }
        srwi      r10,r4,5
        { and keep the rest in r4 (recording whether there is any rest) }
        rlwinm.   r4,r4,0,31-5+1,31
        { move to ctr }
        mtctr     r10
        { check how many rest there is (to decide whether we'll use }
        { FillCharSmall or FillCharVerySmall)                       }
        cmplwi    cr7,r4,11
{$IFDEF USE_DCBZ}
        { if filling with zero, only use dcbz }
        bne       cr1, .LFillCharNoZero
        { make r3 point again to the actual store position }
        addi      r3,r3,4
.LFillCharDCBZLoop:
        dcbz      0,r3
        addi      r3,r3,32
        bdnz      .LFillCharDCBZLoop
        { if there was no rest, we're finished }
        beq       .LFillCharDone
        b         .LFillCharVerySmall
{$ENDIF USE_DCBZ}
.LFillCharNoZero:
{$ifdef FPC_ABI_AIX}
        stw       r5,-4(r1)
        stw       r5,-8(r1)
        lfd       f0,-8(r1)
{$else FPC_ABI_AIX}
        stw       r5,temp
        stw       r5,temp+4
        lfd       f0,temp
{$endif FPC_ABI_AIX}
        { make r3 point to address-8, so we're able to use fp double stores }
        { with update (it's already -4 now)                                 }
        subi      r3,r3,4
{$IFDEF USE_DCBZ}
        { load r10 with 8, so that dcbz uses the correct address }
        li        r10, 8
{$ENDIF}
.LFillChar32ByteLoop:
{$IFDEF USE_DCBZ}
        dcbz      r3,r10
{$ENDIF USE_DCBZ}
        stfdu     f0,8(r3)
        stfdu     f0,8(r3)
        stfdu     f0,8(r3)
        stfdu     f0,8(r3)
        bdnz      .LFillChar32ByteLoop
        { if there was no rest, we're finished }
        beq       .LFillCharDone
        { make r3 point again to the actual next byte that must be written }
        addi      r3,r3,8
        b         .LFillCharVerySmall
.LFillCharSmall:
{$ENDIF FPUNONE}
        { when we arrive here, we're already 4 byte aligned }
        { get count div 4 to store dwords }
        srwi      r10,r4,2
        { get ready for use of update stores }
        subi      r3,r3,4
        mtctr     r10
        rlwinm.   r4,r4,0,31-2+1,31
.LFillCharSmallLoop:
        stwu      r5,4(r3)
        bdnz      .LFillCharSmallLoop
        { if nothing left, stop }
        beq       .LFillCharDone
        { get ready to store bytes }
        addi      r3,r3,4
.LFillCharVerySmall:
        mtctr     r4
        subi      r3,r3,1
.LFillCharVerySmallLoop:
        stbu      r5,1(r3)
        bdnz      .LFillCharVerySmallLoop
.LFillCharDone:
end;
{$endif FPC_SYSTEM_HAS_FILLCHAR}


{$ifndef FPC_SYSTEM_HAS_FILLDWORD}
{$define FPC_SYSTEM_HAS_FILLDWORD}
procedure filldword(var x;count : longint;value : dword);
assembler; nostackframe;
asm
{       registers:
        r3              x
        r4              count
        r5              value
}
                cmpwi   cr0,r4,0
                mtctr   r4
                subi    r3,r3,4
                ble    .LFillDWordEnd    //if count<=0 Then Exit
.LFillDWordLoop:
                stwu    r5,4(r3)
                bdnz    .LFillDWordLoop
.LFillDWordEnd:
end;
{$endif FPC_SYSTEM_HAS_FILLDWORD}


{$ifndef FPC_SYSTEM_HAS_INDEXBYTE}
{$define FPC_SYSTEM_HAS_INDEXBYTE}
function IndexByte(const buf;len:longint;b:byte):longint; assembler; nostackframe;
{ input: r3 = buf, r4 = len, r5 = b                   }
{ output: r3 = position of b in buf (-1 if not found) }
asm
                {  load the begin of the buffer in the data cache }
                dcbt    0,r3
                cmplwi  r4,0
                mtctr   r4
                subi    r10,r3,1
                mr      r0,r3
                { assume not found }
                li      r3,-1
                ble     .LIndexByteDone
.LIndexByteLoop:
                lbzu    r9,1(r10)
                cmplw   r9,r5
                bdnzf   cr0*4+eq,.LIndexByteLoop
                { r3 still contains -1 here }
                bne     .LIndexByteDone
                sub     r3,r10,r0
.LIndexByteDone:
end;
{$endif FPC_SYSTEM_HAS_INDEXBYTE}


{$ifndef FPC_SYSTEM_HAS_INDEXWORD}
{$define FPC_SYSTEM_HAS_INDEXWORD}
function IndexWord(const buf;len:longint;b:word):longint; assembler; nostackframe;
{ input: r3 = buf, r4 = len, r5 = b                   }
{ output: r3 = position of b in buf (-1 if not found) }
asm
                {  load the begin of the buffer in the data cache }
                dcbt    0,r3
                cmplwi  r4,0
                mtctr   r4
                subi    r10,r3,2
                mr      r0,r3
                { assume not found }
                li      r3,-1
                ble     .LIndexWordDone
.LIndexWordLoop:
                lhzu    r9,2(r10)
                cmplw   r9,r5
                bdnzf   cr0*4+eq,.LIndexWordLoop
                { r3 still contains -1 here }
                bne     .LIndexWordDone
                sub     r3,r10,r0
                srawi   r3,r3,1
.LIndexWordDone:
end;
{$endif FPC_SYSTEM_HAS_INDEXWORD}


{$ifndef FPC_SYSTEM_HAS_INDEXDWORD}
{$define FPC_SYSTEM_HAS_INDEXDWORD}
function IndexDWord(const buf;len:longint;b:DWord):longint; assembler; nostackframe;
{ input: r3 = buf, r4 = len, r5 = b                   }
{ output: r3 = position of b in buf (-1 if not found) }
asm
                {  load the begin of the buffer in the data cache }
                dcbt    0,r3
                cmplwi  r4,0
                mtctr   r4
                subi    r10,r3,4
                mr      r0,r3
                { assume not found }
                li      r3,-1
                ble     .LIndexDWordDone
.LIndexDWordLoop:
                lwzu    r9,4(r10)
                cmplw   r9,r5
                bdnzf   cr0*4+eq, .LIndexDWordLoop
                { r3 still contains -1 here }
                bne     .LIndexDWordDone
                sub     r3,r10,r0
                srawi   r3,r3,2
.LIndexDWordDone:
end;
{$endif FPC_SYSTEM_HAS_INDEXDWORD}


{$ifndef FPC_SYSTEM_HAS_COMPAREBYTE}
{$define FPC_SYSTEM_HAS_COMPAREBYTE}
function CompareByte(const buf1,buf2;len:longint):longint; assembler; nostackframe;
{ input: r3 = buf1, r4 = buf2, r5 = len                           }
{ output: r3 = 0 if equal, < 0 if buf1 < str2, > 0 if buf1 > str2 }
{ note: almost direct copy of strlcomp() from strings.inc         }
asm
        {  load the begin of the first buffer in the data cache }
        dcbt    0,r3
        { use r0 instead of r3 for buf1 since r3 contains result }
        cmplwi  r5,0
        mtctr   r5
        subi    r11,r3,1
        subi    r4,r4,1
        li      r3,0
        ble     .LCompByteDone
.LCompByteLoop:
        { load next chars }
        lbzu    r9,1(r11)
        lbzu    r10,1(r4)
        { calculate difference }
        sub.    r3,r9,r10
        { if chars not equal or at the end, we're ready }
        bdnzt   cr0*4+eq, .LCompByteLoop
.LCompByteDone:
end;
{$endif FPC_SYSTEM_HAS_COMPAREBYTE}


{$ifndef FPC_SYSTEM_HAS_COMPAREWORD}
{$define FPC_SYSTEM_HAS_COMPAREWORD}
function CompareWord(const buf1,buf2;len:longint):longint; assembler; nostackframe;
{ input: r3 = buf1, r4 = buf2, r5 = len                           }
{ output: r3 = 0 if equal, < 0 if buf1 < str2, > 0 if buf1 > str2 }
{ note: almost direct copy of strlcomp() from strings.inc         }
asm
        {  load the begin of the first buffer in the data cache }
        dcbt    0,r3
        { use r0 instead of r3 for buf1 since r3 contains result }
        cmplwi  r5,0
        mtctr   r5
        subi    r11,r3,2
        subi    r4,r4,2
        li      r3,0
        ble     .LCompWordDone
.LCompWordLoop:
        { load next chars }
        lhzu    r9,2(r11)
        lhzu    r10,2(r4)
        { calculate difference }
        sub.    r3,r9,r10
        { if chars not equal or at the end, we're ready }
        bdnzt   cr0*4+eq, .LCompWordLoop
.LCompWordDone:
end;
{$endif FPC_SYSTEM_HAS_COMPAREWORD}


{$ifndef FPC_SYSTEM_HAS_COMPAREDWORD}
{$define FPC_SYSTEM_HAS_COMPAREDWORD}
function CompareDWord(const buf1,buf2;len:longint):longint; assembler; nostackframe;
{ input: r3 = buf1, r4 = buf2, r5 = len                           }
{ output: r3 = 0 if equal, < 0 if buf1 < str2, > 0 if buf1 > str2 }
{ note: almost direct copy of strlcomp() from strings.inc         }
asm
        {  load the begin of the first buffer in the data cache }
        dcbt    0,r3
        { use r0 instead of r3 for buf1 since r3 contains result }
        cmplwi  r5,0
        mtctr   r5
        subi    r11,r3,4
        subi    r4,r4,4
        li      r3,0
        ble     .LCompDWordDone
.LCompDWordLoop:
        { load next chars }
        lwzu    r9,4(r11)
        lwzu    r10,4(r4)
        { calculate difference }
        sub.    r0,r9,r10
        { if chars not equal or at the end, we're ready }
        bdnzt   cr0*4+eq, .LCompDWordLoop
.LCompDWordDone:
        cmplw cr1,r9,r10
        beq .Ldone
        { since these were two dwords, we have to perform an additional }
        { unsigned comparison and set the result accordingly            }
        bgt cr1,.Lpos
        li r3,-2
.Lpos:
        addi r3,r3,1
.Ldone:
end;
{$endif FPC_SYSTEM_HAS_COMPAREDWORD}


{$ifndef FPC_SYSTEM_HAS_INDEXCHAR0}
{$define FPC_SYSTEM_HAS_INDEXCHAR0}
function IndexChar0(const buf;len:longint;b:Char):longint; assembler; nostackframe;
{ input: r3 = buf, r4 = len, r5 = b                         }
{ output: r3 = position of found position (-1 if not found) }
asm
        {  load the begin of the buffer in the data cache }
        dcbt    0,r3
        { length = 0? }
        cmplwi  r4,0
        mtctr   r4
        subi    r9,r3,1
        subi    r0,r3,1
        { assume not found }
        li      r3,-1
        { if yes, do nothing }
        ble     .LIndexChar0Done
.LIndexChar0Loop:
        lbzu    r10,1(r9)
        cmplwi  cr1,r10,0
        cmplw   r10,r5
        beq     cr1,.LIndexChar0Done
        bdnzf   cr0*4+eq, .LIndexChar0Loop
        bne     .LIndexChar0Done
        sub     r3,r9,r0
.LIndexChar0Done:
end;
{$endif FPC_SYSTEM_HAS_INDEXCHAR0}


{****************************************************************************
                                 String
****************************************************************************}

{$ifndef STR_CONCAT_PROCS}

(*
{$ifndef FPC_SYSTEM_HAS_FPC_SHORTSTR_CONCAT}
{$define FPC_SYSTEM_HAS_FPC_SHORTSTR_CONCAT}

function fpc_shortstr_concat(const s1, s2: shortstring): shortstring; compilerproc; [public, alias: 'FPC_SHORTSTR_CONCAT'];
{ expects that (r3) contains a pointer to the result r4 to s1, r5 to s2 }
assembler;
asm
      { load length s1 }
      lbz     r6, 0(r4)
      { load length s2 }
      lbz     r10, 0(r5)
      { length 0 for s1? }
      cmplwi  cr7,r6,0
      { length 255 for s1? }
      subfic. r7,r6,255
      { length 0 for s2? }
      cmplwi  cr1,r10,0
      { calculate min(length(s2),255-length(s1)) }
      subc    r8,r7,r10    { r8 := r7 - r10                                }
      cror    4*6+2,4*1+2,4*7+2
      subfe   r7,r7,r7     { if r7 >= r10 then r7' := 0 else r7' := -1     }
      mtctr   r6
      and     r7,r8,r7     { if r7 >= r10 then r7' := 0 else r7' := r7-r10 }
      add     r7,r7,r10    { if r7 >= r10 then r7' := r10 else r7' := r7   }

      mr      r9,r3

      { calculate length of final string }
      add     r8,r7,r6
      stb     r8,0(r3)
      beq     cr7, .Lcopys1loopDone
    .Lcopys1loop:
      lbzu    r0,1(r4)
      stbu    r0,1(r9)
      bdnz    .Lcopys1loop
    .Lcopys1loopDone:
      mtctr   r7
      beq     cr6, .LconcatDone
    .Lcopys2loop:
      lbzu    r0,1(r5)
      stbu    r0,1(r9)
      bdnz    .Lcopys2loop
end;
{$endif FPC_SYSTEM_HAS_FPC_SHORTSTR_CONCAT}
*)

{$ifndef FPC_SYSTEM_HAS_FPC_SHORTSTR_APPEND_SHORTSTR}
{$define FPC_SYSTEM_HAS_FPC_SHORTSTR_APPEND_SHORTSTR}

procedure fpc_shortstr_append_shortstr(var s1: shortstring; const s2: shortstring); compilerproc;
{ expects that results (r3) contains a pointer to the current string s1, r4 }
{ high(s1) and (r5) a pointer to the one that has to be concatenated        }
assembler; nostackframe;
asm
      { load length s1 }
      lbz     r6, 0(r3)
      { load length s2 }
      lbz     r10, 0(r5)
      { length 0? }
      cmplw   cr1,r6,r4
      cmplwi  r10,0

      { calculate min(length(s2),high(result)-length(result)) }
      sub     r9,r4,r6
      subc    r8,r9,r10    { r8 := r9 - r10                                }
      cror    4*7+2,4*0+2,4*1+2
      subfe   r9,r9,r9     { if r9 >= r10 then r9' := 0 else r9' := -1     }
      and     r9,r8,r9     { if r9 >= r10 then r9' := 0 else r9' := r9-r10 }
      add     r9,r9,r10    { if r9 >= r10 then r9' := r10 else r9' := r9   }

      { calculate new length }
      add     r10,r6,r9
      { load value to copy in ctr }
      mtctr   r9
      { store new length }
      stb     r10,0(r3)
      { go to last current character of result }
      add     r3,r6,r3

      { if nothing to do, exit }
      beq    cr7, .LShortStrAppendDone
      { and concatenate }
.LShortStrAppendLoop:
      lbzu    r10,1(r5)
      stbu    r10,1(r3)
      bdnz    .LShortStrAppendLoop
.LShortStrAppendDone:
end;
{$endif FPC_SYSTEM_HAS_FPC_SHORTSTR_APPEND_SHORTSTR}

{$endif STR_CONCAT_PROCS}

(*
{$define FPC_SYSTEM_HAS_FPC_SHORTSTR_COMPARE}
function fpc_shortstr_compare(const dstr,sstr:shortstring): longint; [public,alias:'FPC_SHORTSTR_COMPARE']; compilerproc;
assembler;
asm
      { load length sstr }
      lbz     r9,0(r4)
      { load length dstr }
      lbz     r10,0(r3)
      { save their difference for later and      }
      { calculate min(length(sstr),length(dstr)) }
      subfc    r7,r10,r9    { r0 := r9 - r10                               }
      subfe    r9,r9,r9     { if r9 >= r10 then r9' := 0 else r9' := -1    }
      and      r7,r7,r9     { if r9 >= r10 then r9' := 0 else r9' := r9-r8 }
      add      r9,r10,r7    { if r9 >= r10 then r9' := r10 else r9' := r9  }

      { first compare dwords (length/4) }
      srwi.   r5,r9,2
      { keep length mod 4 for the ends }
      rlwinm  r9,r9,0,30,31
      { already check whether length mod 4 = 0 }
      cmplwi  cr1,r9,0
      { so we can load r3 with 0, in case the strings both have length 0 }
      mr      r8,r3
      li      r3, 0
      { length div 4 in ctr for loop }
      mtctr   r5
      { if length < 3, goto byte comparing }
      beq     LShortStrCompare1
      { setup for use of update forms of load/store with dwords }
      subi    r4,r4,3
      subi    r8,r8,3
LShortStrCompare4Loop:
      lwzu    r3,4(r4)
      lwzu    r10,4(r8)
      sub.    r3,r3,r10
      bdnzt   cr0+eq,LShortStrCompare4Loop
      { r3 contains result if we stopped because of "ne" flag }
      bne     LShortStrCompareDone
      { setup for use of update forms of load/store with bytes }
      addi    r4,r4,3
      addi    r8,r8,3
LShortStrCompare1:
      { if comparelen mod 4 = 0, skip this and return the difference in }
      { lengths                                                         }
      beq     cr1,LShortStrCompareLen
      mtctr   r9
LShortStrCompare1Loop:
      lbzu    r3,1(r4)
      lbzu    r10,1(r8)
      sub.    r3,r3,r10
      bdnzt   cr0+eq,LShortStrCompare1Loop
      bne     LShortStrCompareDone
LShortStrCompareLen:
      { also return result in flags, maybe we can use this in the CG }
      mr.     r3,r3
LShortStrCompareDone:
end;
*)


{$ifndef FPC_SYSTEM_HAS_FPC_PCHAR_TO_SHORTSTR}
{$define FPC_SYSTEM_HAS_FPC_PCHAR_TO_SHORTSTR}
<<<<<<< HEAD
<<<<<<< HEAD

procedure fpc_pchar_to_shortstr(out res : shortstring;p:pchar);assembler;[public,alias:'FPC_PCHAR_TO_SHORTSTR']; compilerproc; nostackframe;
{
  r3: result address
  r4: high(result)
  r5: p (source)
}
asm
      { nil? }
      mr      r8, p
      cmplwi  p, 0
      {  load the begin of the string in the data cache }
      dcbt    0, p
      { maxlength }
      mr      r10,r4
      mtctr   r10
      { at LStrPasDone, we set the length of the result to 255 - r10 - r4 }
      { = 255 - 255 - 0 if the soure = nil -> perfect :)                  }
      beq     .LStrPasDone
      { save address for at the end and use r7 in loop }
      mr      r7,r3
      { no "subi r7,r7,1" because the first byte = length byte }
      subi    r8,r8,1
.LStrPasLoop:
      lbzu    r10,1(r8)
      cmplwi  cr0,r10,0
      stbu    r10,1(r7)
      bdnzf   cr0*4+eq, .LStrPasLoop

      { if we stopped because of a terminating #0, decrease the length by 1 }
      cntlzw  r4,r10
      { get remaining count for length }
      mfctr   r10
      { if r10 was zero (-> stopped because of zero byte), then r4 will be 32 }
      { (32 leading zero bits) -> shr 5 = 1, otherwise this will be zero      }
      srwi    r4,r4,5
.LStrPasDone:
      subfic  r10,r10,255
      sub     r10,r10,r4

      { store length }
      stb     r10,0(r3)
end;
=======
=======
>>>>>>> origin/fixes_2_2
{$ifndef FPC_STRTOSHORTSTRINGPROC}
function fpc_pchar_to_shortstr(p:pchar):shortstring;[public,alias:'FPC_PCHAR_TO_SHORTSTR']; compilerproc; assembler; nostackframe;
{$else FPC_STRTOSHORTSTRINGPROC}
procedure fpc_pchar_to_shortstr(out res : shortstring;p:pchar);assembler;[public,alias:'FPC_PCHAR_TO_SHORTSTR']; compilerproc; nostackframe;
{$define FPC_STRPASPROC}
{$endif FPC_STRTOSHORTSTRINGPROC}
{$include strpas.inc}
{$undef FPC_STRPASPROC}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$endif FPC_SYSTEM_HAS_FPC_PCHAR_TO_SHORTSTR}


{$ifndef FPC_SYSTEM_HAS_FPC_PCHAR_LENGTH}
{$define FPC_SYSTEM_HAS_FPC_PCHAR_LENGTH}
function fpc_pchar_length(p:pchar):sizeint;assembler;[public,alias:'FPC_PCHAR_LENGTH']; compilerproc; nostackframe;
{$include strlen.inc}
{$endif FPC_SYSTEM_HAS_FPC_PCHAR_LENGTH}


{$define FPC_SYSTEM_HAS_GET_FRAME}
function get_frame:pointer;assembler;{$ifdef SYSTEMINLINE}inline;{$endif} nostackframe;
asm
  { all abi's I know use r1 as stack pointer }
  mr r3, r1
end;

{NOTE: On MACOS, 68000 code might call powerpc code, through the MixedMode manager,
(even in the OS in system 9). The pointer to the switching stack frame is then
indicated by the first bit set to 1. This is checked below.}

{Both routines below assumes that framebp is a valid framepointer or nil.}

{$define FPC_SYSTEM_HAS_GET_CALLER_ADDR}
function get_caller_addr(framebp:pointer;addr:pointer=nil):pointer;assembler;{$ifdef SYSTEMINLINE}inline;{$endif} nostackframe;
asm
   cmplwi  r3,0
   beq     .Lcaller_addr_invalid
   lwz r3,0(r3)
   cmplwi  r3,0
   beq     .Lcaller_addr_invalid
{$ifdef MACOS}
   rlwinm  r4,r3,0,31,31
   cmpwi   r4,0
   bne  cr0,.Lcaller_addr_invalid
{$endif MACOS}
{$ifdef FPC_ABI_AIX}
   lwz r3,8(r3)
{$else FPC_ABI_AIX}
   lwz r3,4(r3)
{$endif FPC_ABI_AIX}
   blr
.Lcaller_addr_invalid:
   li r3,0
end;


{$define FPC_SYSTEM_HAS_GET_CALLER_FRAME}
function get_caller_frame(framebp:pointer;addr:pointer=nil):pointer;assembler;{$ifdef SYSTEMINLINE}inline;{$endif} nostackframe;
asm
    cmplwi  r3,0
    beq     .Lcaller_frame_invalid
    lwz  r3,0(r3)
{$ifdef MACOS}
    rlwinm      r4,r3,0,31,31
    cmpwi       r4,0
    bne cr0,.Lcaller_frame_invalid
{$endif MACOS}
    blr
.Lcaller_frame_invalid:
    li r3,0
end;


{$define FPC_SYSTEM_HAS_SPTR}
Function Sptr : Pointer;assembler;{$ifdef SYSTEMINLINE}inline;{$endif} nostackframe;
asm
        mr    r3,r1
end;


{****************************************************************************
                                 Str()
****************************************************************************}

{ int_str: generic implementation is used for now }


{****************************************************************************
                             Multithreading
****************************************************************************}

{ do a thread save inc/dec }

{$define FPC_SYSTEM_HAS_DECLOCKED_LONGINT}
function declocked(var l : longint) : boolean;assembler;nostackframe;
{ input:  address of l in r3                                      }
{ output: boolean indicating whether l is zero after decrementing }
asm
.LDecLockedLoop:
    lwarx   r10,0,r3
    subi    r10,r10,1
    stwcx.  r10,0,r3
    bne-    .LDecLockedLoop
    cntlzw  r3,r10
    srwi    r3,r3,5
end;

{$define FPC_SYSTEM_HAS_INCLOCKED_LONGINT}
procedure inclocked(var l : longint);assembler;nostackframe;
asm
.LIncLockedLoop:
    lwarx   r10,0,r3
    addi    r10,r10,1
    stwcx.  r10,0,r3
    bne-    .LIncLockedLoop
end;


function InterLockedDecrement (var Target: longint) : longint; assembler; nostackframe;
{ input:  address of target in r3 }
{ output: target-1 in r3          }
{ side-effect: target := target-1 }
asm
.LInterLockedDecLoop:
        lwarx   r10,0,r3
        subi    r10,r10,1
        stwcx.  r10,0,r3
        bne     .LInterLockedDecLoop
        mr      r3,r10
end;


function InterLockedIncrement (var Target: longint) : longint; assembler; nostackframe;
{ input:  address of target in r3 }
{ output: target+1 in r3          }
{ side-effect: target := target+1 }
asm
.LInterLockedIncLoop:
        lwarx   r10,0,r3
        addi    r10,r10,1
        stwcx.  r10,0,r3
        bne     .LInterLockedIncLoop
        mr      r3,r10
end;


function InterLockedExchange (var Target: longint;Source : longint) : longint; assembler; nostackframe;
{ input:  address of target in r3, source in r4 }
{ output: target in r3                          }
{ side-effect: target := source                 }
asm
.LInterLockedXchgLoop:
        lwarx   r10,0,r3
        stwcx.  r4,0,r3
        bne     .LInterLockedXchgLoop
        mr      r3,r10
end;


function InterLockedExchangeAdd (var Target: longint;Source : longint) : longint; assembler; nostackframe;
asm
.LInterLockedXchgAddLoop:
        lwarx   r10,0,r3
        add     r10,r10,r4
        stwcx.  r10,0,r3
        bne     .LInterLockedXchgAddLoop
        sub     r3,r10,r4
end;


function InterlockedCompareExchange(var Target: longint; NewValue: longint; Comperand: longint): longint; assembler; nostackframe;
{ input:  address of target in r3, newvalue in r4, comparand in r5 }
{ output: value stored in target before entry of the function      }
{ side-effect: NewValue stored in target if (target = comparand)   }
asm
.LInterlockedCompareExchangeLoop:
  lwarx  r10,0,r3
  sub    r9,r10,r5
  addic  r9,r9,-1
  subfe  r9,r9,r9
  and    r8,r4,r9
  andc   r7,r10,r9
  or     r6,r7,r8
  stwcx. r6,0,r3
  bne .LInterlockedCompareExchangeLoop
  mr     r3, r10
end;

{$IFDEF MORPHOS}
{ this is only required for MorphOS }
{$define FPC_SYSTEM_HAS_SYSINITFPU}
procedure SysInitFPU;{$ifdef SYSTEMINLINE}inline;{$endif}
  var tmp: array[0..1] of dword;
begin
  asm
     { setting fpu to round to nearest mode }
     li r3,0
     stw r3,8(r1)
     stw r3,12(r1)
     lfd f1,8(r1)
     mtfsf 7,f1
  end;
  { powerpc might use softfloat code }
<<<<<<< HEAD
<<<<<<< HEAD
  softfloat_exception_flags:=[];
  softfloat_exception_mask:=[float_flag_underflow, float_flag_inexact, float_flag_denormal];
=======
  softfloat_exception_flags:=0;
  softfloat_exception_mask:=float_flag_underflow or float_flag_inexact or float_flag_denormal;
>>>>>>> graemeg/fixes_2_2
=======
  softfloat_exception_flags:=0;
  softfloat_exception_mask:=float_flag_underflow or float_flag_inexact or float_flag_denormal;
>>>>>>> origin/fixes_2_2
end;

{$define FPC_SYSTEM_HAS_SYSRESETFPU}
procedure SysResetFPU;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
<<<<<<< HEAD
<<<<<<< HEAD
  softfloat_exception_flags:=[];
=======
  softfloat_exception_flags:=0;
>>>>>>> graemeg/fixes_2_2
=======
  softfloat_exception_flags:=0;
>>>>>>> origin/fixes_2_2
end;
{$ENDIF}

{$ifndef FPC_SYSTEM_HAS_MEM_BARRIER}
{$define FPC_SYSTEM_HAS_MEM_BARRIER}

procedure ReadBarrier;assembler;nostackframe;{$ifdef SYSTEMINLINE}inline;{$endif}
asm
<<<<<<< HEAD
<<<<<<< HEAD
  isync
=======
  sync
>>>>>>> graemeg/fixes_2_2
=======
  sync
>>>>>>> origin/fixes_2_2
end;

procedure ReadDependencyBarrier;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  { reads imply barrier on earlier reads depended on }
end;

procedure ReadWriteBarrier;assembler;nostackframe;{$ifdef SYSTEMINLINE}inline;{$endif}
asm
<<<<<<< HEAD
<<<<<<< HEAD
  isync
  eieio
=======
  sync
>>>>>>> graemeg/fixes_2_2
=======
  sync
>>>>>>> origin/fixes_2_2
end;

procedure WriteBarrier;assembler;nostackframe;{$ifdef SYSTEMINLINE}inline;{$endif}
asm
  eieio
end;

{$endif}
