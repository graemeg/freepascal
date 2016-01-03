{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2000 by Jonas Maebe and other members of the
    Free Pascal development team

    Implementation of mathematical Routines (only for real)

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{****************************************************************************
                       EXTENDED data type routines
 ****************************************************************************}

{$define FPC_SYSTEM_HAS_ABS}
function fpc_abs_real(d : valreal) : valreal;compilerproc;
begin
  { Function is handled internal in the compiler }
  runerror(207);
  result:=0;
end;

{$define FPC_SYSTEM_HAS_SQR}
function fpc_sqr_real(d : valreal) : valreal;compilerproc;
begin
  { Function is handled internal in the compiler }
  runerror(207);
  result:=0;
end;

{$define FPC_SYSTEM_HAS_TRUNC}
function fpc_trunc_real(d : valreal) : int64;compilerproc; assembler; nostackframe;
{ input: d in fr1      }
{ output: result in r3 }
asm
  fctidz f1, f1
  stfd f1, -8(r1)
  ld r3, -8(r1)
end;

{$ifndef FPC_SYSTEM_HAS_ROUND}

{$define FPC_SYSTEM_HAS_ROUND}
function fpc_round_real(d : valreal) : int64; compilerproc;assembler; nostackframe;
{ exactly the same as trunc, except that one fctiwz has become fctiw }
{ input: d in fr1      }
{ output: result in r3 }
asm
  fctid f1, f1
  stfd  f1, -8(r1)
  ld r3, -8(r1)
end;
{$endif not FPC_SYSTEM_HAS_ROUND}

{****************************************************************************
                         Int to real helpers
 ****************************************************************************}

{$define FPC_SYSTEM_HAS_INT64_TO_DOUBLE}
function fpc_int64_to_double(i: int64): double; compilerproc;assembler;
{ input: i in r3 }
{ output: double(i) in f0            }
{from "PowerPC Microprocessor Family: Programming Environments Manual for 64 and 32-Bit Microprocessors", v2.0, pg. 698 }
var temp : int64;
asm
  std r3,temp // store dword
  lfd f0,temp // load float
  fcfid f0,f0     // convert to fpu int
end;

{ we wouls have to generate the .localfunc directive for ELFv2, and moreover it
  must appear at the start right after setting up the TOC pointer, but the local
  variables will cause the code generator to already insert the stack allocation
  before that... -> disable this routine for ELFv2 }
{$if not defined(aix) and (not defined(linux) or (defined(_ELF_CALL) and (_ELF_CALL = 1))) }

{$define FPC_SYSTEM_HAS_QWORD_TO_DOUBLE}
function fpc_qword_to_double(q: qword): double; compilerproc;assembler;
const
  longint_to_real_helper: qword = $80000000;
{from "PowerPC Microprocessor Family: Programming Environments Manual for
 64 and 32-Bit Microprocessors", v2.0, pg. 698, *exact version*              }
{ input: q in r3 }
{ output: double(q) in f0            }
var
  temp1, temp2: qword;
asm
{$ifndef darwin}
  // load 2^32 into f4
  lis   r4, longint_to_real_helper@highesta
  ori   r4, r4, longint_to_real_helper@highera
  sldi  r4, r4, 32
  oris  r4, r4, longint_to_real_helper@ha
  lfd   f4, longint_to_real_helper@l(r4)
{$else not darwin}
{$ifdef FPC_PIC}
   mflr   r0
   bcl    20,31,.Lpiclab
.Lpiclab:
   mflr   r5
   mtlr   r0
   addis  r4,r5,(longint_to_real_helper-.Lpiclab)@ha
   lfd    f2,(longint_to_real_helper-.Lpiclab)@l(r4)
{$else FPC_PIC}
  lis   r4, longint_to_real_helper@ha
  lfd   f4, longint_to_real_helper@l(r4)
{$endif FPC_PIC}
{$endif not darwin}
  rldicl r4,r3,32,32  // isolate high half
  rldicl r0,r3,0,32   // isolate low half
  std r4,temp1        // store dword both
  std r0,temp2
  lfd f2,temp1        // load float both
  lfd f0,temp2        // load float both
  fcfid f2,f2         // convert each half to
  fcfid f0,f0         // fpu int (no rnd)
  fmadd f0,f4,f2,f0   // (2**32)*high+low (only add can rnd)
end;

<<<<<<< HEAD
<<<<<<< HEAD
{$endif ndef aix}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
