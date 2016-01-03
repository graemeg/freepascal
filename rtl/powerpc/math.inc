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


const
  longint_to_real_helper: int64 = $4330000080000000;
  cardinal_to_real_helper: int64 = $4330000000000000;
  int_to_real_factor: double = double(high(cardinal))+1.0;


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

      const
        factor: double = double(int64(1) shl 32);
        factor2: double = double(int64(1) shl 31);

{$ifndef FPC_SYSTEM_HAS_TRUNC}
    {$define FPC_SYSTEM_HAS_TRUNC}
    function fpc_trunc_real(d : valreal) : int64;assembler;compilerproc;
      { input: d in fr1      }
      { output: result in r3 }
      assembler;
      var
        temp: packed record
            case byte of
              0: (l1,l2: longint);
              1: (d: double);
          end;
      asm
        // store d in temp
        stfd    f1,temp
        // extract sign bit (record in cr0)
        lwz     r3,temp
        rlwinm. r3,r3,1,31,31
        // make d positive
        fabs    f1,f1
        // load 2^32 in f2
<<<<<<< HEAD
        {$if not defined(macos) and not defined(aix)}
=======
        {$ifndef macos}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        {$ifdef FPC_PIC}
        {$ifdef darwin}
        mflr   r0
        bcl    20,31,.Lpiclab
.Lpiclab:
        mflr   r5
        mtlr   r0
        addis  r4,r5,(factor-.Lpiclab)@ha
        lfd    f2,(factor-.Lpiclab)@l(r4)
        {$else darwin}
        {$error Add pic code for linux/ppc32}
        {$endif darwin}
        {$else FPC_PIC}
        lis    r4,factor@ha
        lfd    f2,factor@l(r4)
        {$endif FPC_PIC}
<<<<<<< HEAD
<<<<<<< HEAD
        {$else not macos/aix}
        lwz    r4,factor(r2)
        lfd    f2,0(r4)
        {$endif not macos/aix}
=======
        {$else not macos}
        lwz    r4,factor(r2)
        lfd    f2,0(r4)
        {$endif not macos}
>>>>>>> graemeg/fixes_2_2
=======
        {$else not macos}
        lwz    r4,factor(r2)
        lfd    f2,0(r4)
        {$endif not macos}
>>>>>>> origin/fixes_2_2
        // check if value is < 0
        // f3 := d / 2^32;
        fdiv     f3,f1,f2
        // round
        fctiwz   f4,f3
        // store
        stfd     f4,temp
        // and load into r4
        lwz      r3,temp+4
        // convert back to float
        lis      r0,0x4330
        stw      r0,temp
        xoris    r0,r3,0x8000
        stw      r0,temp+4
<<<<<<< HEAD
        {$if not defined(macos) and not defined(aix)}
=======
        {$ifndef macos}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        {$ifdef FPC_PIC}
        {$ifdef darwin}
        addis  r4,r5,(longint_to_real_helper-.Lpiclab)@ha
        lfd    f0,(longint_to_real_helper-.Lpiclab)@l(r4)
        {$else darwin}
        {$error Add pic code for linux/ppc32}
        {$endif darwin}
        {$else FPC_PIC}
        lis    r4,longint_to_real_helper@ha
        lfd    f0,longint_to_real_helper@l(r4)
        {$endif FPC_PIC}
<<<<<<< HEAD
<<<<<<< HEAD
        {$else not macos/aix}
        lwz    r4,longint_to_real_helper(r2)
        lfd    f0,0(r4)
        {$endif not macos/aix}
=======
        {$else not macos}
        lwz    r4,longint_to_real_helper(r2)
        lfd    f0,0(r4)
        {$endif not macos}
>>>>>>> graemeg/fixes_2_2
=======
        {$else not macos}
        lwz    r4,longint_to_real_helper(r2)
        lfd    f0,0(r4)
        {$endif not macos}
>>>>>>> origin/fixes_2_2
        lfd    f3,temp
        fsub   f3,f3,f0


        // f4 := d "mod" 2^32 ( = d - ((d / 2^32) * 2^32))
        fnmsub   f4,f3,f2,f1

        // now, convert to unsigned 32 bit

        // load 2^31 in f2
<<<<<<< HEAD
        {$if not defined(macos) and not defined(aix)}
=======
        {$ifndef macos}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        {$ifdef FPC_PIC}
        {$ifdef darwin}
        addis  r4,r5,(factor2-.Lpiclab)@ha
        lfd    f2,(factor2-.Lpiclab)@l(r4)
        {$else darwin}
        {$error Add pic code for linux/ppc32}
        {$endif darwin}
        {$else FPC_PIC}
        lis    r4,factor2@ha
        lfd    f2,factor2@l(r4)
        {$endif FPC_PIC}
<<<<<<< HEAD
<<<<<<< HEAD
        {$else not macos/aix}
        lwz    r4,factor2(r2)
        lfd    f2,0(r4)
        {$endif not macos/aix}
=======
        {$else not macos}
        lwz    r4,factor2(r2)
        lfd    f2,0(r4)
        {$endif not macos}
>>>>>>> graemeg/fixes_2_2
=======
        {$else not macos}
        lwz    r4,factor2(r2)
        lfd    f2,0(r4)
        {$endif not macos}
>>>>>>> origin/fixes_2_2

        // subtract 2^31
        fsub   f3,f4,f2
        // was the value > 2^31?
        fcmpu  cr1,f4,f2
        // use diff if >= 2^31
        fsel   f4,f3,f3,f4

        // next part same as conversion to signed integer word
        fctiwz f4,f4
        stfd   f4,temp
        lwz    r4,temp+4
        // add 2^31 if value was >=2^31
        blt    cr1, .LTruncNoAdd
        xoris  r4,r4,0x8000
.LTruncNoAdd:
        // negate value if it was negative to start with
        beq    cr0,.LTruncPositive
        subfic r4,r4,0
        subfze r3,r3
.LTruncPositive:
      end;
{$endif not FPC_SYSTEM_HAS_TRUNC}


(*
{$ifndef FPC_SYSTEM_HAS_ROUND}
    {$define FPC_SYSTEM_HAS_ROUND}
    function round(d : extended) : int64;

    function fpc_round(d : extended) : int64;assembler;[public, alias:'FPC_ROUND'];compilerproc;
      { exactly the same as trunc, except that one fctiwz has become fctiw }
      { input: d in fr1      }
      { output: result in r3 }
      assembler;
      var
        temp: packed record
            case byte of
              0: (l1,l2: longint);
              1: (d: double);
          end;
      asm
        // store d in temp
        stfd    f1, temp
        // extract sign bit (record in cr0)
        lwz     r4,temp
        rlwinm. r4,r4,1,31,31
        // make d positive
        fabs    f1,f1
        // load 2^32 in f2
        {$if not defined(macos) and not defined(aix)}
        lis    r4,factor@ha
        lfd    f2,factor@l(r4)
        {$else}
        lwz    r4,factor(r2)
        lfd    f2,0(r4)
        {$endif}
        // check if value is < 0
        // f3 := d / 2^32;
        fdiv     f3,f1,f2
        // round
        fctiwz   f4,f3
        // store
        stfd     f4,temp
        // and load into r4
        lwz      r3,temp+4
        // convert back to float
        lis      r0,0x4330
        stw      r0,temp
        xoris    r0,r3,0x8000
        stw      r0,temp+4
        {$if not defined(macos) and not defined(aix)}
        lis    r4,longint_to_real_helper@ha
        lfd    f0,longint_to_real_helper@l(r4)
        {$else}
        lwz    r4,longint_to_real_helper(r2)
        lfd    f0,0(r4)
        {$endif}
        lfd    f3,temp
        fsub   f3,f3,f0


        // f4 := d "mod" 2^32 ( = d - ((d / 2^32) * 2^32))
        fnmsub   f4,f3,f2,f1

        // now, convert to unsigned 32 bit

        // load 2^31 in f2
        {$if not defined(macos) and not defined(aix)}
        lis    r4,factor2@ha
        lfd    f2,factor2@l(r4)
        {$else}
        lwz    r4,factor2(r2)
        lfd    f2,0(r4)
        {$endif}

        // subtract 2^31
        fsub   f3,f4,f2
        // was the value > 2^31?
        fcmpu  cr1,f4,f2
        // use diff if >= 2^31
        fsel   f4,f3,f3,f4

        // next part same as conversion to signed integer word
        fctiw  f4,f4
        stfd   f4,temp
        lwz    r4,temp+4
        // add 2^31 if value was >=2^31
        blt    cr1, .LRoundNoAdd
        xoris  r4,r4,0x8000
.LRoundNoAdd:
        // negate value if it was negative to start with
        beq    cr0,.LRoundPositive
        subfic r4,r4,0
        subfze r3,r3
.LRoundPositive:
      end;
{$endif not FPC_SYSTEM_HAS_ROUND}
*)


{****************************************************************************
                         Int to real helpers
 ****************************************************************************}
{$ifndef aix}
{ these helpers somehow don't seem to work on AIX/Power 7 }

{$define FPC_SYSTEM_HAS_INT64_TO_DOUBLE}
function fpc_int64_to_double(i: int64): double; compilerproc;
assembler;
{ input: high(i) in r4, low(i) in r3 }
{ output: double(i) in f0            }
var
  temp: packed record
      case byte of
        0: (l1,l2: cardinal);
        1: (d: double);
    end;
asm
           lis    r0,0x4330
           stw    r0,temp
           xoris  r3,r3,0x8000
           stw    r3,temp+4
<<<<<<< HEAD
           {$if not defined(macos) and not defined(aix)}
=======
           {$ifndef macos}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
           {$ifdef FPC_PIC}
           {$ifdef darwin}
           mflr   r0
           bcl    20,31,.Lpiclab
 .Lpiclab:
           mflr   r5
           mtlr   r0
           addis  r3,r5,(longint_to_real_helper-.Lpiclab)@ha
           lfd    f1,(longint_to_real_helper-.Lpiclab)@l(r3)
           {$else darwin}
           {$error Add pic code for linux/ppc32}
           {$endif darwin}
           {$else FPC_PIC}
           lis    r3,longint_to_real_helper@ha
           lfd    f1,longint_to_real_helper@l(r3)
           {$endif FPC_PIC}
<<<<<<< HEAD
<<<<<<< HEAD
           {$else not macos/aix}
=======
           {$else not macos}
>>>>>>> graemeg/fixes_2_2
=======
           {$else not macos}
>>>>>>> origin/fixes_2_2
           lwz    r3,longint_to_real_helper(r2)
           lfd    f1,0(r3)
           {$endif not mac os}
           lfd    f0,temp
           stw    r4,temp+4
           fsub   f0,f0,f1
<<<<<<< HEAD
           {$if not defined(macos) and not defined(aix)}
=======
           {$ifndef macos}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
           {$ifdef FPC_PIC}
           {$ifdef darwin}
           addis  r4,r5,(cardinal_to_real_helper-.Lpiclab)@ha
           lfd    f1,(cardinal_to_real_helper-.Lpiclab)@l(r4)
           addis  r4,r5,(int_to_real_factor-.Lpiclab)@ha
           lfd    f3,temp
           lfd    f2,(int_to_real_factor-.Lpiclab)@l(r4)
           {$else darwin}
           {$error Add pic code for linux/ppc32}
           {$endif darwin}
           {$else FPC_PIC}
           lis    r4,cardinal_to_real_helper@ha
           lfd    f1,cardinal_to_real_helper@l(r4)
           lis    r4,int_to_real_factor@ha
           lfd    f3,temp
           lfd    f2,int_to_real_factor@l(r4)
           {$endif FPC_PIC}
<<<<<<< HEAD
<<<<<<< HEAD
           {$else not macos/aix}
=======
           {$else not macos}
>>>>>>> graemeg/fixes_2_2
=======
           {$else not macos}
>>>>>>> origin/fixes_2_2
           lwz    r4,cardinal_to_real_helper(r2)
           lwz    r3,int_to_real_factor(r2)
           lfd    f3,temp
           lfd    f1,0(r4)
           lfd    f2,0(r3)
<<<<<<< HEAD
<<<<<<< HEAD
           {$endif not macos/aix}
=======
           {$endif not macos}
>>>>>>> graemeg/fixes_2_2
=======
           {$endif not macos}
>>>>>>> origin/fixes_2_2
           fsub   f3,f3,f1
           fmadd  f1,f0,f2,f3
end;


{$define FPC_SYSTEM_HAS_QWORD_TO_DOUBLE}
function fpc_qword_to_double(q: qword): double; compilerproc;
assembler;
{ input: high(q) in r4, low(q) in r3 }
{ output: double(q) in f0            }
var
  temp: packed record
      case byte of
        0: (l1,l2: cardinal);
        1: (d: double);
    end;
asm
           lis    r0,0x4330
           stw    r0,temp
           stw    r3,temp+4
           lfd    f0,temp
<<<<<<< HEAD
           {$if not defined(macos) and not defined(aix)}
=======
           {$ifndef macos}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
           {$ifdef FPC_PIC}
           {$ifdef darwin}
           mflr   r0
           bcl    20,31,.Lpiclab
 .Lpiclab:
           mflr   r5
           mtlr   r0
           addis  r3,r5,(cardinal_to_real_helper-.Lpiclab)@ha
           lfd    f1,(cardinal_to_real_helper-.Lpiclab)@l(r3)
           {$else darwin}
           {$error Add pic code for linux/ppc32}
           {$endif darwin}
           {$else FPC_PIC}
           lis    r3,cardinal_to_real_helper@ha
           lfd    f1,cardinal_to_real_helper@l(r3)
           {$endif FPC_PIC}
<<<<<<< HEAD
<<<<<<< HEAD
           {$else not macos/aix}
           lwz    r3,longint_to_real_helper(r2)
           lfd    f1,0(r3)
           {$endif not macos/aix}
           stw    r4,temp+4
           fsub   f0,f0,f1
           lfd    f3,temp
           {$if not defined(macos) and not defined(aix)}
=======
           {$else not macos}
           lwz    r3,longint_to_real_helper(r2)
           lfd    f1,0(r3)
=======
           {$else not macos}
           lwz    r3,longint_to_real_helper(r2)
           lfd    f1,0(r3)
>>>>>>> origin/fixes_2_2
           {$endif not macos}
           stw    r4,temp+4
           fsub   f0,f0,f1
           lfd    f3,temp
           {$ifndef macos}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
           {$ifdef FPC_PIC}
           {$ifdef darwin}
           addis  r4,r5,(int_to_real_factor-.Lpiclab)@ha
           lfd    f2,(int_to_real_factor-.Lpiclab)@l(r4)
           {$else darwin}
           {$error Add pic code for linux/ppc32}
           {$endif darwin}
           {$else FPC_PIC}
           lis    r4,int_to_real_factor@ha
           lfd    f2,int_to_real_factor@l(r4)
           {$endif FPC_PIC}
<<<<<<< HEAD
<<<<<<< HEAD
           {$else not macos/aix}
           lwz    r4,int_to_real_factor(r2)
           lfd    f2,0(r4)
           {$endif not macos/aix}
=======
           {$else not macos}
           lwz    r4,int_to_real_factor(r2)
           lfd    f2,0(r4)
           {$endif not macos}
>>>>>>> graemeg/fixes_2_2
=======
           {$else not macos}
           lwz    r4,int_to_real_factor(r2)
           lfd    f2,0(r4)
           {$endif not macos}
>>>>>>> origin/fixes_2_2
           fsub   f3,f3,f1
           fmadd  f1,f0,f2,f3
end;
{$endif}

