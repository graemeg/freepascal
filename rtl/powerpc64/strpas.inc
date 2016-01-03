{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by the Free Pascal development team

    Processor specific implementation of strpas

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{
 ifndef FPC_STRPASPROC
   r3: result address
   r4: p (source)
 else
  r3: result address
  r4: high(result)
  r5: p (source)
 endif
}
asm
        { nil? }
        mr      r8, p 
        cmpldi  p, 0
        {  load the begin of the string in the data cache }
        dcbt    0, p
        { maxlength }
{$ifdef FPC_STRPASPROC}
        mr      r10,r4
{$else FPC_STRPASPROC}
        li      r10,255
{$endif FPC_STRPASPROC}
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
        cntlzd  r4,r10
        { get remaining count for length }
        mfctr   r10
        { if r10 was zero (-> stopped because of zero byte), then r4 will be 64 }
        { (64 leading zero bits) -> shr 6 = 1, otherwise this will be zero      }
        srdi    r4,r4,6
.LStrPasDone:
        subfic  r10,r10,255
        sub     r10,r10,r4

        { store length }
        stb     r10,0(r3)
end;

