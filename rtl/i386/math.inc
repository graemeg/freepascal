{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2001 by the Free Pascal development team

    Implementation of mathematical routines (for extended type)

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{-------------------------------------------------------------------------
 Using functions from AMath/DAMath libraries, which are covered by the
 following license:

 (C) Copyright 2009-2013 Wolfgang Ehrhardt

 This software is provided 'as-is', without any express or implied warranty.
 In no event will the authors be held liable for any damages arising from
 the use of this software.

 Permission is granted to anyone to use this software for any purpose,
 including commercial applications, and to alter it and redistribute it
 freely, subject to the following restrictions:

 1. The origin of this software must not be misrepresented; you must not
    claim that you wrote the original software. If you use this software in
    a product, an acknowledgment in the product documentation would be
    appreciated but is not required.

 2. Altered source versions must be plainly marked as such, and must not be
    misrepresented as being the original software.

 3. This notice may not be removed or altered from any source distribution.
----------------------------------------------------------------------------}

{****************************************************************************
                            FPU Control word
 ****************************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
{$push}
{$codealign constmin=16}
const
  FPC_ABSMASK_SINGLE: array[0..1] of qword=($7fffffff7fffffff,$7fffffff7fffffff); cvar; public;
  FPC_ABSMASK_DOUBLE: array[0..1] of qword=($7fffffffffffffff,$7fffffffffffffff); cvar; public;
{$pop}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    procedure Set8087CW(cw:word);
      begin
        { pic-safe ; cw will not be a regvar because it's accessed from }
        { assembler                                                     }
        default8087cw:=cw;
        asm
          fnclex
          fldcw cw
        end;
      end;


    function Get8087CW:word;assembler;
      asm
        pushl $0
        fnstcw (%esp)
        popl %eax
      end;

<<<<<<< HEAD

    procedure SetMXCSR(w : dword);
      begin
        defaultmxcsr:=w;
=======
    procedure SetSSECSR(w : dword);
      begin
        mxcsr:=w;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        asm
          ldmxcsr w
        end;
      end;


    function GetMXCSR : dword;
      var
        _w : dword;
      begin
        asm
          stmxcsr _w
        end;
        result:=_w;
      end;


    procedure SetSSECSR(w : dword);
      begin
        SetMXCSR(w);
      end;

    function GetSSECSR: dword;
      begin
        result:=GetMXCSR;
      end;

{****************************************************************************
                       EXTENDED data type routines
 ****************************************************************************}

    {$define FPC_SYSTEM_HAS_ABS}
    function fpc_abs_real(d : ValReal) : ValReal;compilerproc;
    begin
      { Function is handled internal in the compiler }
      runerror(207);
      result:=0;
    end;
    {$define FPC_SYSTEM_HAS_SQR}
    function fpc_sqr_real(d : ValReal) : ValReal;compilerproc;
    begin
      { Function is handled internal in the compiler }
      runerror(207);
      result:=0;
    end;
    {$define FPC_SYSTEM_HAS_SQRT}
    function fpc_sqrt_real(d : ValReal) : ValReal;compilerproc;
    begin
      { Function is handled internal in the compiler }
      runerror(207);
      result:=0;
    end;
    {$define FPC_SYSTEM_HAS_ARCTAN}
    function fpc_arctan_real(d : ValReal) : ValReal;compilerproc;
    begin
      { Function is handled internal in the compiler }
      runerror(207);
      result:=0;
    end;
    {$define FPC_SYSTEM_HAS_LN}
    function fpc_ln_real(d : ValReal) : ValReal;compilerproc;
    begin
      { Function is handled internal in the compiler }
      runerror(207);
      result:=0;
    end;
    {$define FPC_SYSTEM_HAS_SIN}
    function fpc_sin_real(d : ValReal) : ValReal;compilerproc;
    begin
      { Function is handled internal in the compiler }
      runerror(207);
      result:=0;
    end;
    {$define FPC_SYSTEM_HAS_COS}
    function fpc_cos_real(d : ValReal) : ValReal;compilerproc;
    begin
      { Function is handled internal in the compiler }
      runerror(207);
      result:=0;
    end;

  {$define FPC_SYSTEM_HAS_EXP}
    { exp function adapted from AMath library (C) Copyright 2009-2013 Wolfgang Ehrhardt
      * translated into AT&T syntax
      + PIC support
      * return +Inf/0 for +Inf/-Inf input, instead of NaN }
    function fpc_exp_real(d : ValReal) : ValReal;assembler;compilerproc;
      var
        cw1,cw2: word;
      asm
        fldt        d
        fldl2e
<<<<<<< HEAD
        fmul        %st(1),%st        { z = d * log2(e) }
        frndint
      { Calculate frac(z) using modular arithmetic to avoid precision loss.
        Avoid PIC hacks by using immediate operands (it's not the fastest,
        but likely the cleanest solution). }
        pushl       $0x3fe62e42       { ln(2).hi=6.9314718036912382E-001 }
        pushl       $0xfee00000
        fldl        (%esp)
        fmul        %st(1),%st
        fsubrp      %st,%st(2)
        pushl       $0x3dea39ef       { ln(2).lo=1.9082149292705877E-010 }
        pushl       $0x35793c76
        fldl        (%esp)
        fmul        %st(1),%st
        fsubrp      %st,%st(2)
        fxch        %st(1)            { (d-int(z)*ln2_hi)-int(z)*ln2_lo }
        fldl2e
        fmulp       %st,%st(1)        { frac(z) }

      { The above code can result in |frac(z)|>1, particularly when rounding mode
        is not "round to nearest". f2xm1 is undefined in this case, so a check
        is necessary. Furthermore, frac(z) evaluates to NaN for d=+-Inf. }
        fld         %st
        fabs
        fld1
        fcompp
        fstsw       %ax
        sahf
        jp          .L3               { NaN }
        jae         .L1               { frac(z) <= 1 }
        fld         %st(1)
        fabs
        pushl       $0x46c00000       { single(24576.0) }
        fcomps      (%esp)
        fstsw       %ax
        sahf
        jb          .L0               { int(z) < 24576 }
.L3:
        fstp        %st               { zero out frac(z), hard way because }
        fldz                          { "fsub %st,%st" does not work for NaN }
        jmp         .L1
.L0:
        { Calculate 2**frac(z)-1 as N*(N+2), where N=2**(frac(z)/2)-1 }
        pushl       $0x3f000000       { single(0.5) }
        fmuls       (%esp)
        f2xm1
        fld         %st
        pushl       $0x40000000       { single(2.0) }
        fadds       (%esp)
        fmulp       %st,%st(1)
        jmp         .L2
.L1:
=======
        fmulp       %st,%st(1)
        fstcw       CW1
        fstcw       CW2
        fwait
        andw        $0xf3ff,CW2
        orw         $0x0400,CW2
        fldcw       CW2
        fld         %st(0)
        frndint
        fldcw       CW1
        fxch        %st(1)
        fsub        %st(1),%st
>>>>>>> graemeg/fixes_2_2
        f2xm1
.L2:
        fld1
        faddp       %st,%st(1)
        fscale
        fstp        %st(1)
<<<<<<< HEAD
=======
        fclex
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
     end;


    {$define FPC_SYSTEM_HAS_FRAC}
    function fpc_frac_real(d : ValReal) : ValReal;assembler;compilerproc;
      asm
        subl $4,%esp
        fnstcw (%esp)
        fwait
        movw (%esp),%cx
        orw $0x0f00,(%esp)
        fldcw (%esp)
        fldt d
        frndint
        fldt d
        fsub %st(1),%st
        fstp %st(1)
        movw %cx,(%esp)
        fldcw (%esp)
      end;


    {$define FPC_SYSTEM_HAS_INT}
    function fpc_int_real(d : ValReal) : ValReal;assembler;compilerproc;
      asm
        subl $4,%esp
        fnstcw (%esp)
        fwait
        movw (%esp),%cx
        orw $0x0f00,(%esp)
        fldcw (%esp)
        fwait
        fldt d
        frndint
        fwait
        movw %cx,(%esp)
        fldcw (%esp)
      end;


    {$define FPC_SYSTEM_HAS_TRUNC}
    function fpc_trunc_real(d : ValReal) : int64;assembler;compilerproc;
      asm
        subl $12,%esp
        fldt d
        fnstcw (%esp)
        movw (%esp),%cx
        orw $0x0f00,(%esp)
        fldcw (%esp)
        movw %cx,(%esp)
        fistpq 4(%esp)
        fldcw (%esp)
        fwait
        movl 4(%esp),%eax
        movl 8(%esp),%edx
      end;


    {$define FPC_SYSTEM_HAS_ROUND}
    { keep for bootstrapping with 2.0.x }
    function fpc_round_real(d : ValReal) : int64;compilerproc;assembler;
      var
        res   : int64;
      asm
        fldt d
        fistpq res
        fwait
        movl res,%eax
        movl res+4,%edx
      end;
    


