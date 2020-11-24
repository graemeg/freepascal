{
    Implementation of mathematical routines for x86_64

    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2005 by the Free Pascal development team

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

{$push}
{$codealign constmin=16}
const
  FPC_ABSMASK_SINGLE: array[0..1] of qword=($7fffffff7fffffff,$7fffffff7fffffff); cvar; public;
  FPC_ABSMASK_DOUBLE: array[0..1] of qword=($7fffffffffffffff,$7fffffffffffffff); cvar; public;
{$pop}

{****************************************************************************
                            FPU Control word
 ****************************************************************************}

    procedure Set8087CW(cw:word);
      begin
         default8087cw:=cw;
         asm
           fnclex
           fldcw cw
         end;
      end;


    function Get8087CW:word;assembler;
      var
        tmp: word;
      asm
        fnstcw tmp
        movw   tmp,%ax
        andl   $0xffff,%eax  { clears bits 32-63 }
      end;


    procedure SetMXCSR(w : dword);
      begin
        defaultmxcsr:=w;
        asm
          ldmxcsr w
        end;
      end;


    function GetMXCSR : dword;assembler;
      var
        _w : dword;
      asm
        stmxcsr _w
        movl    _w,%eax
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

    {$ifndef FPC_SYSTEM_HAS_ABS}
    {$define FPC_SYSTEM_HAS_ABS}
    function fpc_abs_real(d : ValReal) : ValReal;compilerproc;
{$ifndef cpullvm}
    begin
      { Function is handled internal in the compiler }
      runerror(207);
      result:=0;
{$else not cpullvm}
    assembler;
    asm
      fldt d
      fabs
{$endif not cpullvm}
    end;
    {$endif FPC_SYSTEM_HAS_ABS}
    {$ifndef FPC_SYSTEM_HAS_SQR}
    {$define FPC_SYSTEM_HAS_SQR}
    function fpc_sqr_real(d : ValReal) : ValReal;compilerproc;
{$ifndef cpullvm}
    begin
      { Function is handled internal in the compiler }
      runerror(207);
      result:=0;
{$else not cpullvm}
    begin
      fpc_sqr_real:=d*d;
{$endif not cpullvm}
    end;
    {$endif FPC_SYSTEM_HAS_SQR}
    {$ifndef FPC_SYSTEM_HAS_SQRT}
    {$define FPC_SYSTEM_HAS_SQRT}
    function fpc_sqrt_real(d : ValReal) : ValReal;compilerproc;
{$ifndef cpullvm}
    begin
      { Function is handled internal in the compiler }
      runerror(207);
      result:=0;
{$else not cpullvm}
    assembler;
    asm
      fldt d
      fsqrt
{$endif not cpullvm}
    end;
    {$endif FPC_SYSTEM_HAS_SQRT}

{$ifdef FPC_HAS_TYPE_EXTENDED}

    {$ifndef FPC_SYSTEM_HAS_ARCTAN}
    {$define FPC_SYSTEM_HAS_ARCTAN}
    function fpc_arctan_real(d : ValReal) : ValReal;compilerproc;
{$ifndef cpullvm}
    begin
      { Function is handled internal in the compiler }
      runerror(207);
      result:=0;
{$else not cpullvm}
    assembler;
    asm
      fldt d
      fld1
      fpatan
{$endif not cpullvm}
    end;
    {$endif FPC_SYSTEM_HAS_ARCTAN}
    {$ifndef FPC_SYSTEM_HAS_LN}
    {$define FPC_SYSTEM_HAS_LN}
    function fpc_ln_real(d : ValReal) : ValReal;compilerproc;
{$ifndef cpullvm}
    begin
      { Function is handled internal in the compiler }
      runerror(207);
      result:=0;
{$else not cpullvm}
    assembler;
    asm
      fldln2
      fldt d
      fyl2x
{$endif not cpullvm}
    end;
    {$endif FPC_SYSTEM_HAS_LN}
    {$ifndef FPC_SYSTEM_HAS_SIN}
    {$define FPC_SYSTEM_HAS_SIN}
    function fpc_sin_real(d : ValReal) : ValReal;compilerproc;
{$ifndef cpullvm}
    begin
      { Function is handled internal in the compiler }
      runerror(207);
      result:=0;
{$else not cpullvm}
    assembler;
    asm
      fldt d
      fsin
{$endif not cpullvm}
    end;
    {$endif FPC_SYSTEM_HAS_SIN}
    {$ifndef FPC_SYSTEM_HAS_COS}
    {$define FPC_SYSTEM_HAS_COS}
    function fpc_cos_real(d : ValReal) : ValReal;compilerproc;
{$ifndef cpullvm}
    begin
      { Function is handled internal in the compiler }
      runerror(207);
      result:=0;
{$else not cpullvm}
    assembler;
    asm
      fldt d
      fcos
{$endif not cpullvm}
    end;
    {$endif FPC_SYSTEM_HAS_COS}

    {$ifndef FPC_SYSTEM_HAS_EXP}
    {$define FPC_SYSTEM_HAS_EXP}
    { exp function adapted from AMath library (C) Copyright 2009-2013 Wolfgang Ehrhardt
      * translated into AT&T syntax
      + PIC support
      * return +Inf/0 for +Inf/-Inf input, instead of NaN }
    function fpc_exp_real(d : ValReal) : ValReal;assembler;compilerproc;
      const
        ln2hi: double=6.9314718036912382E-001;
        ln2lo: double=1.9082149292705877E-010;
        large: single=24576.0;
        two:   single=2.0;
        half:  single=0.5;
      asm
            fldt        d
            fldl2e
            fmul        %st(1),%st        { z = d * log2(e) }
            frndint
          { Calculate frac(z) using modular arithmetic to avoid precision loss }
            fldl        ln2hi(%rip)
            fmul        %st(1),%st
            fsubrp      %st,%st(2)
            fldl        ln2lo(%rip)
            fmul        %st(1),%st
            fsubrp      %st,%st(2)
            fxch        %st(1)            { (d-int(z)*ln2_hi)-int(z)*ln2_lo }
            fldl2e
            fmulp       %st,%st(1)        { frac(z) }

          { Above calculation can yield |frac(z)|>1, particularly when rounding mode
            is not "round to nearest". f2xm1 is undefined in that case, so it's
            necessary to check }
            fld         %st
            fabs
            fld1
            fcomip      %st(1),%st(0)
            fstp        %st
            jp          .L3               { NaN }
            jae         .L1               { |frac(z)| <= 1, good }

            fld         %st(1)
            fabs
            flds        large(%rip)
            fcomip      %st(1),%st(0)
            fstp        %st
            jb          .L3               { int(z) >= 24576 }
   .L0:
            { Calculate 2**frac(z)-1 as N*(N+2), where N=2**(frac(z)/2)-1 }
            fmuls       half(%rip)
            f2xm1
            fld         %st
            fadds       two(%rip)
            fmulp       %st,%st(1)
            jmp         .L2
   .L3:
            fstp        %st               { pop frac(z) and load 0 }
            fldz
   .L1:
            f2xm1
   .L2:
            fld1
            faddp       %st,%st(1)
            fscale
            fstp        %st(1)
      end;
    {$endif FPC_SYSTEM_HAS_EXP}


    {$ifndef FPC_SYSTEM_HAS_FRAC}
    {$define FPC_SYSTEM_HAS_FRAC}
    function fpc_frac_real(d : ValReal) : ValReal;assembler;compilerproc;
      var
        oldcw,newcw: word;
      asm
            fnstcw oldcw
            fwait
            movw oldcw,%cx
            orw $0x0c3f,%cx
            movw %cx,newcw
            fldcw newcw
            fwait
            fldt d
            frndint
            fldt d
            fsub %st(1),%st
            fstp %st(1)
            fnclex
            fldcw oldcw
      end;
    {$endif FPC_SYSTEM_HAS_FRAC}


    {$ifndef FPC_SYSTEM_HAS_INT}
    {$define FPC_SYSTEM_HAS_INT}
    function fpc_int_real(d : ValReal) : ValReal;assembler;compilerproc;
      var
        oldcw,newcw: word;
      asm
            fnstcw oldcw
            fwait
            movw oldcw,%cx
            orw $0x0c3f,%cx
            movw %cx,newcw
            fldcw newcw
            fwait
            fldt d
            frndint
            fwait
            fldcw oldcw
      end;
    {$endif FPC_SYSTEM_HAS_INT}


    {$ifndef FPC_SYSTEM_HAS_TRUNC}
    {$define FPC_SYSTEM_HAS_TRUNC}
    function fpc_trunc_real(d : ValReal) : int64;assembler;compilerproc;
      var
        oldcw,
        newcw : word;
        res   : int64;
      asm
        fnstcw oldcw
        fwait
        movw oldcw,%cx
        orw $0x0c3f,%cx
        movw %cx,newcw
        fldcw newcw
        fldt d
        fistpq res
        fwait
        movq res,%rax
        fldcw oldcw
      end;
    {$endif FPC_SYSTEM_HAS_TRUNC}


    {$ifndef FPC_SYSTEM_HAS_ROUND}
    {$define FPC_SYSTEM_HAS_ROUND}
    function fpc_round_real(d : ValReal) : int64;assembler;compilerproc;
      var
        res   : int64;
      asm
        fldt d
        fistpq res
        fwait
        movq res,%rax
      end;
    {$endif FPC_SYSTEM_HAS_ROUND}

{$else FPC_HAS_TYPE_EXTENDED}

    {$ifndef FPC_SYSTEM_HAS_INT}
    {$define FPC_SYSTEM_HAS_INT}
    function fpc_int_real(d : ValReal) : ValReal;compilerproc; assembler; nostackframe;
      asm
        movq      %xmm0,  %rax
        shr       $48,    %rax
        and       $0x7ff0,%ax
        cmp       $0x4330,%ax
        jge       .L0
        cvttsd2si %xmm0,  %rax
        cvtsi2sd  %rax,   %xmm0
    .L0:
      end;
    {$endif FPC_SYSTEM_HAS_INT}

    {$ifndef FPC_SYSTEM_HAS_TRUNC}
    {$define FPC_SYSTEM_HAS_TRUNC}
    function fpc_trunc_real(d : ValReal) : int64;compilerproc; assembler; nostackframe;
      asm
        cvttsd2si   %xmm0,%rax;
      end;
    {$endif FPC_SYSTEM_HAS_TRUNC}

    {$ifndef FPC_SYSTEM_HAS_ROUND}
    {$define FPC_SYSTEM_HAS_ROUND}
    function fpc_round_real(d : ValReal) : int64;compilerproc; assembler; nostackframe;
      asm
        cvtsd2si   %xmm0,%rax;
      end;
    {$endif FPC_SYSTEM_HAS_ROUND}

    {$ifndef FPC_SYSTEM_HAS_FRAC}
    {$define FPC_SYSTEM_HAS_FRAC}
    function fpc_frac_real(d: ValReal) : ValReal;compilerproc; assembler; nostackframe;
      asm
        { Windows defines %xmm4 and %xmm5 as first non-parameter volatile registers;
          on SYSV systems all are considered as such, so use %xmm4 }
        movq      %xmm0,  %rax
        movapd    %xmm0,  %xmm4
        shr       $48,    %rax
        and       $0x7ff0,%ax
        cmp       $0x4330,%ax
        jge       .L0
        cvttsd2si %xmm0,  %rax
        cvtsi2sd  %rax,   %xmm4
  .L0:
        subsd     %xmm4,  %xmm0
      end;
    {$endif FPC_SYSTEM_HAS_FRAC}

{$endif FPC_HAS_TYPE_EXTENDED}
