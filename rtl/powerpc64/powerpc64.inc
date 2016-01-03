{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2000-2001 by the Free Pascal development team.

    Portions Copyright (c) 2000 by Casey Duncan (casey.duncan@state.co.us)

    Processor dependent implementation for the system unit for
    PowerPC64

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


{****************************************************************************
                           PowerPC specific stuff
****************************************************************************}

procedure fpc_enable_ppc_fpu_exceptions; assembler; nostackframe;
asm
  { clear all "exception happened" flags we care about}
  mtfsfi 0,0
  mtfsfi 1,0
  mtfsfi 2,0
  mtfsfi 3,0
{$ifdef fpc_mtfsb0_corrected}
  mtfsb0 21
  mtfsb0 22
  mtfsb0 23
{$endif fpc_mtfsb0_corrected}

  { enable invalid operations and division by zero exceptions. }
  { No overflow/underflow, since those give some spurious      }
  { exceptions                                                 }
  mtfsfi 6,9
end;


procedure fpc_cpuinit;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  { don't let libraries influence the FPU cw set by the host program }
  if not IsLibrary then
    fpc_enable_ppc_fpu_exceptions;
=======
  fpc_enable_ppc_fpu_exceptions;
>>>>>>> graemeg/fixes_2_2
=======
  fpc_enable_ppc_fpu_exceptions;
>>>>>>> origin/fixes_2_2
end;

{****************************************************************************
                                Move / Fill
****************************************************************************}

{$ifndef FPC_SYSTEM_HAS_MOVE}
{$define FPC_SYSTEM_HAS_MOVE}
procedure Move(const source;var dest;count:SizeInt);[public, alias: 'FPC_MOVE'];
type
  bytearray    = array [0..high(sizeint)-1] of byte;
var
  i:longint;
begin
  if count <= 0 then exit;
  Dec(count);
  if (@dest > @source) then
    begin
      for i:=count downto 0 do
        bytearray(dest)[i]:=bytearray(source)[i];
    end
  else
    begin
      for i:=0 to count do
        bytearray(dest)[i]:=bytearray(source)[i];
    end;
end;
{$endif FPC_SYSTEM_HAS_MOVE}


{$ifndef FPC_SYSTEM_HAS_FILLCHAR}
{$define FPC_SYSTEM_HAS_FILLCHAR}

Procedure FillChar(var x;count:SizeInt;value:byte);
type
  longintarray = array [0..high(sizeint) div 4-1] of longint;
  bytearray    = array [0..high(sizeint)-1] of byte;
var
  i,v : longint;
begin
  if count <= 0 then exit;
  v := 0;
  { aligned? }
  if (PtrUInt(@x) mod sizeof(PtrUInt))<>0 then
      for i:=0 to count-1 do
        bytearray(x)[i]:=value
  else begin
    v:=(value shl 8) or (value and $FF);
    v:=(v shl 16) or (v and $ffff);
    for i:=0 to (count div 4)-1 do
      longintarray(x)[i]:=v;
    for i:=(count div 4)*4 to count-1 do
      bytearray(x)[i]:=value;
  end;
end;
{$endif FPC_SYSTEM_HAS_FILLCHAR}

{$ifndef FPC_SYSTEM_HAS_FILLDWORD}
{$define FPC_SYSTEM_HAS_FILLDWORD}
procedure filldword(var x;count : SizeInt;value : dword); assembler; nostackframe;
asm
  cmpdi   cr0,r4,0
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
function IndexByte(const buf;len:SizeInt;b:byte):int64; assembler; nostackframe;
{ input: r3 = buf, r4 = len, r5 = b                   }
{ output: r3 = position of b in buf (-1 if not found) }
asm
  {  load the begin of the buffer in the data cache }
  dcbt    0,r3
  cmpldi  r4,0
  mtctr   r4
  subi    r10,r3,1
  mr      r0,r3
  { assume not found }
  li      r3,-1
  ble     .LIndexByteDone
.LIndexByteLoop:
  lbzu    r9,1(r10)
  cmpld   r9,r5
  bdnzf   cr0*4+eq,.LIndexByteLoop
  { r3 still contains -1 here }
  bne     .LIndexByteDone
  sub     r3,r10,r0
.LIndexByteDone:
end;
{$endif FPC_SYSTEM_HAS_INDEXBYTE}


{$ifndef FPC_SYSTEM_HAS_INDEXWORD}
{$define FPC_SYSTEM_HAS_INDEXWORD}
function IndexWord(const buf;len:SizeInt;b:word):int64; assembler; nostackframe;
{ input: r3 = buf, r4 = len, r5 = b                   }
{ output: r3 = position of b in buf (-1 if not found) }
asm
  {  load the begin of the buffer in the data cache }
  dcbt    0,r3
  cmpldi  r4,0
  mtctr   r4
  subi    r10,r3,2
  mr      r0,r3
  { assume not found }
  li      r3,-1
  ble     .LIndexWordDone
.LIndexWordLoop:
  lhzu    r9,2(r10)
  cmpld   r9,r5
  bdnzf   cr0*4+eq,.LIndexWordLoop
  { r3 still contains -1 here }
  bne     .LIndexWordDone
  sub     r3,r10,r0
  sradi   r3,r3,1
.LIndexWordDone:
end;
{$endif FPC_SYSTEM_HAS_INDEXWORD}


{$ifndef FPC_SYSTEM_HAS_INDEXDWORD}
{$define FPC_SYSTEM_HAS_INDEXDWORD}
function IndexDWord(const buf;len:SizeInt;b:DWord):int64; assembler; nostackframe;
{ input: r3 = buf, r4 = len, r5 = b                   }
{ output: r3 = position of b in buf (-1 if not found) }
asm
  {  load the begin of the buffer in the data cache }
  dcbt    0,r3
  cmpldi  r4,0
  mtctr   r4
  subi    r10,r3,4
  mr      r0,r3
  { assume not found }
  li      r3,-1
  ble     .LIndexDWordDone
.LIndexDWordLoop:
  lwzu    r9,4(r10)
  cmpld   r9,r5
  bdnzf   cr0*4+eq, .LIndexDWordLoop
  { r3 still contains -1 here }
  bne     .LIndexDWordDone
  sub     r3,r10,r0
  sradi   r3,r3,2
.LIndexDWordDone:
end;
{$endif FPC_SYSTEM_HAS_INDEXDWORD}

{$ifndef FPC_SYSTEM_HAS_COMPAREBYTE}
{$define FPC_SYSTEM_HAS_COMPAREBYTE}
function CompareByte(const buf1,buf2;len:SizeInt):int64; assembler; nostackframe;
{ input: r3 = buf1, r4 = buf2, r5 = len                           }
{ output: r3 = 0 if equal, < 0 if buf1 < str2, > 0 if buf1 > str2 }
{ note: almost direct copy of strlcomp() from strings.inc         }
asm
  {  load the begin of the first buffer in the data cache }
  dcbt    0,r3
  { use r0 instead of r3 for buf1 since r3 contains result }
  cmpldi  r5,0
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
function CompareWord(const buf1,buf2;len:SizeInt):int64; assembler; nostackframe;
{ input: r3 = buf1, r4 = buf2, r5 = len                           }
{ output: r3 = 0 if equal, < 0 if buf1 < str2, > 0 if buf1 > str2 }
{ note: almost direct copy of strlcomp() from strings.inc         }
asm
  {  load the begin of the first buffer in the data cache }
  dcbt    0,r3
  { use r0 instead of r3 for buf1 since r3 contains result }
  cmpldi  r5,0
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
function CompareDWord(const buf1,buf2;len:SizeInt):int64; assembler; nostackframe;
{ input: r3 = buf1, r4 = buf2, r5 = len                           }
{ output: r3 = 0 if equal, < 0 if buf1 < str2, > 0 if buf1 > str2 }
{ note: almost direct copy of strlcomp() from strings.inc         }
asm
  {  load the begin of the first buffer in the data cache }
  dcbt    0,r3
  { use r0 instead of r3 for buf1 since r3 contains result }
  cmpldi  r5,0
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
  sub.    r3,r9,r10
  { if chars not equal or at the end, we're ready }
  bdnzt   cr0*4+eq, .LCompDWordLoop
.LCompDWordDone:
  cmpld   cr1,r9,r10
  beq     .Ldone
  { since these were two dwords, we have to perform an additional }
  { unsigned comparison and set the result accordingly            }
  bgt       cr1,.Lpos
  li r3,-2
.Lpos:
  addi r3,r3,1
.Ldone:
end;
{$endif FPC_SYSTEM_HAS_COMPAREDWORD}


{$ifndef FPC_SYSTEM_HAS_INDEXCHAR0}
{$define FPC_SYSTEM_HAS_INDEXCHAR0}
function IndexChar0(const buf;len:SizeInt;b:Char):int64; assembler; nostackframe;
{ input: r3 = buf, r4 = len, r5 = b                         }
{ output: r3 = position of found position (-1 if not found) }
asm
  {  load the begin of the buffer in the data cache }
  dcbt    0,r3
  { length = 0? }
  cmpldi  r4,0
  mtctr   r4
  subi    r9,r3,1
  subi    r0,r3,1
  { assume not found }
  li      r3,-1
  { if yes, do nothing }
  ble     .LIndexChar0Done
.LIndexChar0Loop:
  lbzu    r10,1(r9)
  cmpldi  cr1,r10,0
  cmpld   r10,r5
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
  cmpldi  cr7,r6,0
  { length 255 for s1? }
  subfic. r7,r6,255
  { length 0 for s2? }
  cmpldi  cr1,r10,0
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
.LconcatDone:
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
  cmpld   cr1,r6,r4
  cmpldi  r10,0

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
function fpc_shortstr_compare(const dstr, sstr:shortstring): SizeInt; [public,alias:'FPC_SHORTSTR_COMPARE']; compilerproc;
assembler;
{ TODO: improve, because the main compare loop does an unaligned access everytime.. :(
  TODO: needs some additional opcodes not yet known to the compiler :( }
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

  { first compare qwords (length/4) }
  srdi.   r5,r9,3
  { keep length mod 8 for the ends; note that the value in r9 <= 255
   so we can use rlwinm safely }
  rlwinm  r9,r9,0,29,31
  { already check whether length mod 8 = 0 }
  cmpldi  cr1,r9,0
  { so we can load r3 with 0, in case the strings both have length 0 }
  mr      r8,r3
  li      r3, 0
  { length div 8 in ctr for loop }
  mtctr   r5
  { if length < 7, goto byte comparing }
  beq     .LShortStrCompare1
  { setup for use of update forms of load/store with qwords }
  subi    r4,r4,7
  subi    r8,r8,7
.LShortStrCompare4Loop:
  ldu     r3,8(r4)
  ldu     r10,8(r8)
  sub.    r3,r3,r10
  bdnzt   cr0+eq,.LShortStrCompare4Loop
  { r3 contains result if we stopped because of "ne" flag }
  bne     .LShortStrCompareDone
  { setup for use of update forms of load/store with bytes }
  addi    r4,r4,7
  addi    r8,r8,7
.LShortStrCompare1:
  { if comparelen mod 4 = 0, skip this and return the difference in }
  { lengths                                                         }
  beq     cr1,.LShortStrCompareLen
  mtctr   r9
.LShortStrCompare1Loop:
  lbzu    r3,1(r4)
  lbzu    r10,1(r8)
  sub.    r3,r3,r10
  bdnzt   cr0+eq,.LShortStrCompare1Loop
  bne     .LShortStrCompareDone
.LShortStrCompareLen:
  { also return result in flags, maybe we can use this in the CG }
  mr.     r3,r3
.LShortStrCompareDone:
end;
*)

{$ifndef FPC_SYSTEM_HAS_FPC_PCHAR_TO_SHORTSTR}
{$define FPC_SYSTEM_HAS_FPC_PCHAR_TO_SHORTSTR}
<<<<<<< HEAD
<<<<<<< HEAD
procedure fpc_pchar_to_shortstr(out res : shortstring;p:pchar);[public,alias:'FPC_PCHAR_TO_SHORTSTR']; compilerproc;
assembler; nostackframe;
{
  r3: result address
  r4: high(result)
  r5: p (source)
}
asm
        { nil? }
        mr      r8, p
        cmpldi  p, 0
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
=======
=======
>>>>>>> origin/fixes_2_2
{$ifndef FPC_STRTOSHORTSTRINGPROC}
function fpc_pchar_to_shortstr(p:pchar):shortstring;[public,alias:'FPC_PCHAR_TO_SHORTSTR']; compilerproc;
assembler; nostackframe;
{$else FPC_STRTOSHORTSTRINGPROC}
procedure fpc_pchar_to_shortstr(out res : shortstring;p:pchar);[public,alias:'FPC_PCHAR_TO_SHORTSTR']; compilerproc;
assembler; nostackframe;
{$define FPC_STRPASPROC}
{$endif FPC_STRTOSHORTSTRINGPROC}
{$include strpas.inc}
{$undef FPC_STRPASPROC}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$endif FPC_SYSTEM_HAS_FPC_PCHAR_TO_SHORTSTR}

(*
{$ifndef FPC_SYSTEM_HAS_FPC_PCHAR_LENGTH}
{$define FPC_SYSTEM_HAS_FPC_PCHAR_LENGTH}
function fpc_pchar_length(p:pchar):longint;assembler;[public,alias:'FPC_PCHAR_LENGTH']; {$ifdef hascompilerproc} compilerproc; {$endif} nostackframe;
{$include strlen.inc}
{$endif FPC_SYSTEM_HAS_FPC_PCHAR_LENGTH}
*)

{$define FPC_SYSTEM_HAS_GET_FRAME}
function get_frame:pointer;assembler;{$ifdef SYSTEMINLINE}inline;{$endif} nostackframe;
asm
  { all abi's I know use r1 as stack pointer }
  mr r3, r1
end;

{$define FPC_SYSTEM_HAS_GET_CALLER_ADDR}
<<<<<<< HEAD
<<<<<<< HEAD
function get_caller_addr(framebp:pointer;addr:pointer=nil):pointer;assembler;{$ifdef SYSTEMINLINE}inline;{$endif} nostackframe;
=======
function get_caller_addr(framebp:pointer):pointer;assembler;{$ifdef SYSTEMINLINE}inline;{$endif} nostackframe;
>>>>>>> graemeg/fixes_2_2
=======
function get_caller_addr(framebp:pointer):pointer;assembler;{$ifdef SYSTEMINLINE}inline;{$endif} nostackframe;
>>>>>>> origin/fixes_2_2
asm
  cmpldi  r3,0
  beq     .Lcaller_addr_frame_null
  ld  r3, 0(r3)

  cmpldi  r3,0
  beq     .Lcaller_addr_frame_null
  ld r3, 16(r3)
.Lcaller_addr_frame_null:
end;


{$define FPC_SYSTEM_HAS_GET_CALLER_FRAME}
<<<<<<< HEAD
<<<<<<< HEAD
function get_caller_frame(framebp:pointer;addr:pointer=nil):pointer;assembler;{$ifdef SYSTEMINLINE}inline;{$endif} nostackframe;
=======
function get_caller_frame(framebp:pointer):pointer;assembler;{$ifdef SYSTEMINLINE}inline;{$endif} nostackframe;
>>>>>>> graemeg/fixes_2_2
=======
function get_caller_frame(framebp:pointer):pointer;assembler;{$ifdef SYSTEMINLINE}inline;{$endif} nostackframe;
>>>>>>> origin/fixes_2_2
asm
  cmpldi  r3,0
  beq     .Lcaller_frame_null
  ld  r3, 0(r3)
.Lcaller_frame_null:
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
    lwarx    r10,0,r3
    subi    r10,r10,1
    stwcx.  r10,0,r3
    bne-    .LDecLockedLoop
    cntlzd  r3,r10
    srdi    r3,r3,6
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


{$define FPC_SYSTEM_HAS_DECLOCKED_INT64}
function declocked(var l : int64) : boolean;assembler;nostackframe;
{ input:  address of l in r3                                      }
{ output: boolean indicating whether l is zero after decrementing }
asm
.LDecLockedLoop:
    ldarx    r10,0,r3
    subi    r10,r10,1
    stdcx.  r10,0,r3
    bne-    .LDecLockedLoop
    cntlzd  r3,r10
    srdi    r3,r3,6
end;

{$define FPC_SYSTEM_HAS_INCLOCKED_INT64}
procedure inclocked(var l : int64);assembler;nostackframe;
asm
.LIncLockedLoop:

    ldarx   r10,0,r3
    addi    r10,r10,1
    stdcx.  r10,0,r3
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
{ input:  address of target in r3, source in r4 }
{ output: target in r3                          }
{ side-effect: target := target+source          }
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
<<<<<<< HEAD
<<<<<<< HEAD
  // lwarx performs an unsigned load -> sign extend since arguments are
  // longints
  extsw  r10,r10
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  sub    r9,r10,r5
  addic  r9,r9,-1
  subfe  r9,r9,r9
  and    r8,r4,r9
<<<<<<< HEAD
<<<<<<< HEAD
  andc   r7,r10,r9
  or     r6,r7,r8
  stwcx. r6,0,r3
  bne .LInterlockedCompareExchangeLoop
  mr     r3, r10
=======
=======
>>>>>>> origin/fixes_2_2
  andc   r7,r5,r9
  or     r6,r7,r8
  stwcx. r6,0,r3
  bne .LInterlockedCompareExchangeLoop
  mr     r3, r6
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

function InterLockedDecrement64(var Target: Int64) : Int64; assembler; nostackframe;
{ input:  address of target in r3 }
{ output: target-1 in r3          }
{ side-effect: target := target-1 }
asm
.LInterLockedDecLoop:
        ldarx   r10,0,r3
        subi    r10,r10,1
        stdcx.  r10,0,r3
        bne     .LInterLockedDecLoop
        mr      r3,r10
end;


function InterLockedIncrement64(var Target: Int64) : Int64; assembler; nostackframe;
{ input:  address of target in r3 }
{ output: target+1 in r3          }
{ side-effect: target := target+1 }
asm
.LInterLockedIncLoop:
        ldarx   r10,0,r3
        addi    r10,r10,1
        stdcx.  r10,0,r3
        bne     .LInterLockedIncLoop
        mr      r3,r10
end;


function InterLockedExchange64(var Target: Int64; Source : Int64) : Int64; assembler; nostackframe;
{ input:  address of target in r3, source in r4 }
{ output: target in r3                          }
{ side-effect: target := source                 }
asm
.LInterLockedXchgLoop:
        ldarx   r10,0,r3
        stdcx.  r4,0,r3
        bne     .LInterLockedXchgLoop
        mr      r3,r10
end;


function InterLockedExchangeAdd64(var Target: Int64; Source : Int64) : Int64; assembler; nostackframe;
{ input:  address of target in r3, source in r4 }
{ output: target in r3                          }
{ side-effect: target := target+source          }
asm
.LInterLockedXchgAddLoop:
        ldarx   r10,0,r3
        add     r10,r10,r4
        stdcx.  r10,0,r3
        bne     .LInterLockedXchgAddLoop
        sub     r3,r10,r4
end;

function InterlockedCompareExchange64(var Target: Int64; NewValue: Int64; Comperand: Int64): Int64; assembler; nostackframe;
{ input:  address of target in r3, newvalue in r4, comparand in r5 }
{ output: value stored in target before entry of the function      }
{ side-effect: NewValue stored in target if (target = comparand)   }
asm
.LInterlockedCompareExchangeLoop:
  ldarx  r10,0,r3
  sub    r9,r10,r5
  addic  r9,r9,-1
  subfe  r9,r9,r9
  and    r8,r4,r9
<<<<<<< HEAD
<<<<<<< HEAD
  andc   r7,r10,r9
  or     r6,r7,r8
  stdcx. r6,0,r3
  bne .LInterlockedCompareExchangeLoop
  mr     r3, r10
=======
=======
>>>>>>> origin/fixes_2_2
  andc   r7,r5,r9
  or     r6,r7,r8
  stdcx. r6,0,r3
  bne .LInterlockedCompareExchangeLoop
  mr     r3, r6
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

{$ifndef FPC_SYSTEM_HAS_MEM_BARRIER}
{$define FPC_SYSTEM_HAS_MEM_BARRIER}

procedure ReadBarrier;assembler;nostackframe;{$ifdef SYSTEMINLINE}inline;{$endif}
asm
<<<<<<< HEAD
<<<<<<< HEAD
  isync
=======
{$ifdef FPC_HAS_LWSYNC}
  lwsync
{$endif}
>>>>>>> graemeg/fixes_2_2
=======
{$ifdef FPC_HAS_LWSYNC}
  lwsync
{$endif}
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
{$ifdef FPC_HAS_LWSYNC}
  lwsync
{$endif}
=======
  sync
>>>>>>> graemeg/fixes_2_2
=======
  sync
>>>>>>> origin/fixes_2_2
end;

procedure WriteBarrier;assembler;nostackframe;{$ifdef SYSTEMINLINE}inline;{$endif}
asm
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef FPC_HAS_LWSYNC}
  lwsync
{$endif}
=======
  eieio
>>>>>>> graemeg/fixes_2_2
=======
  eieio
>>>>>>> origin/fixes_2_2
end;

{$endif}
