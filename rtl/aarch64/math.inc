{
    Implementation of mathematical routines for AArch64

    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2005 by the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


{****************************************************************************
                       Floating point type routines
 ****************************************************************************}

    {$ifndef FPC_SYSTEM_HAS_ABS}
    {$define FPC_SYSTEM_HAS_ABS}
    function fpc_abs_real(d : ValReal) : ValReal;compilerproc;
    begin
      { Function is handled internal in the compiler }
      runerror(207);
      result:=0;
    end;
    {$endif FPC_SYSTEM_HAS_ABS}


    {$ifndef FPC_SYSTEM_HAS_SQR}
    {$define FPC_SYSTEM_HAS_SQR}
    function fpc_sqr_real(d : ValReal) : ValReal;compilerproc;
    begin
      { Function is handled internal in the compiler }
      runerror(207);
      result:=0;
    end;
    {$endif FPC_SYSTEM_HAS_SQR}


    {$ifndef FPC_SYSTEM_HAS_SQRT}
    {$define FPC_SYSTEM_HAS_SQRT}
    function fpc_sqrt_real(d : ValReal) : ValReal;compilerproc;
    begin
      { Function is handled internal in the compiler }
      runerror(207);
      result:=0;
    end;
    {$endif FPC_SYSTEM_HAS_SQRT}


{$if not defined(VER3_2) and not defined(CPULLVM)}
    {$ifndef FPC_SYSTEM_HAS_FRAC}
    {$define FPC_SYSTEM_HAS_FRAC}
    function fpc_frac_real(d : ValReal) : ValReal;compilerproc;
      begin
        { Function is handled internal in the compiler }
        runerror(207);
        result:=0;
      end;
    {$endif FPC_SYSTEM_HAS_FRAC}
{$endif not VER3_2 and not CPULLVM }


    {$ifndef FPC_SYSTEM_HAS_INT}
    {$define FPC_SYSTEM_HAS_INT}
    function fpc_int_real(d : ValReal) : ValReal;assembler;nostackframe;compilerproc;
      asm
        // { round as floating point towards zero }
        frintz d0,d0
      end;
    {$endif FPC_SYSTEM_HAS_INT}


    {$ifndef FPC_SYSTEM_HAS_TRUNC}
    {$define FPC_SYSTEM_HAS_TRUNC}
    function fpc_trunc_real(d : ValReal) : int64;assembler;nostackframe;compilerproc;
      asm
        // { round to signed integer towards zero }
        fcvtzs x0,d0
      end;
    {$endif FPC_SYSTEM_HAS_TRUNC}


    {$ifndef FPC_SYSTEM_HAS_ROUND}
    {$define FPC_SYSTEM_HAS_ROUND}
    function fpc_round_real(d : ValReal) : int64;assembler;nostackframe;compilerproc;
      asm
        // { round as floating point using current rounding mode }
        frintx d0,d0
        // { convert to signed integer rounding towards zero (there's no "round to
        //   integer using current rounding mode") }
        fcvtzs x0,d0
      end;
    {$endif FPC_SYSTEM_HAS_ROUND}
