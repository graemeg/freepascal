{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2001 by Several contributors

    Generic mathemtical routines in libc

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{ for 80x86, we can easily write the optimal inline code }
{ Furthermore, the routines below only go up to double   }
{ precision and we need extended precision if supported  }
{$ifndef FPC_HAS_TYPE_EXTENDED}
<<<<<<< HEAD
<<<<<<< HEAD

{$ifdef aix}
    { aix math library routines don't raise exceptions, you have to manually
      check for them }
    function feclearexcept(flags: longint): longint; cdecl; external 'c';
    function fetestexcept(flags: longint): longint; cdecl; external 'c';

    const
      FE_DIVBYZERO  = $04000000;
      FE_INEXACT    = $02000000;
      FE_INVALID    = $20000000;
      FE_OVERFLOW   = $10000000;
      FE_UNDERFLOW  = $08000000;
      FE_ALL_EXCEPT = $3E000000;

    procedure resetexcepts;
      begin
        seterrno(0);
        feclearexcept(FE_ALL_EXCEPT);
      end;

    procedure checkexcepts;
      var
        feres: longint;
        sfexcepts: TFPUExceptionMask;
      begin
        feres:=fetestexcept(FE_ALL_EXCEPT);
        sfexcepts:=[];
        if feres<>0 then
          begin
            if (feres and FE_DIVBYZERO) <> 0 then
              include(sfexcepts,float_flag_divbyzero);
            if (feres and FE_INEXACT) <> 0 then
              include(sfexcepts,float_flag_inexact);
            if (feres and FE_INVALID) <> 0 then
              include(sfexcepts,float_flag_invalid);
            if (feres and FE_OVERFLOW) <> 0 then
              include(sfexcepts,float_flag_overflow);
            if (feres and FE_UNDERFLOW) <> 0 then
              include(sfexcepts,float_flag_underflow);
          end
        { unknown error }
        else if (geterrno<>0) then
          include(sfexcepts,float_flag_invalid);
        if sfexcepts<>[] then
          float_raise(sfexcepts);
      end;

{$else aix}
    procedure resetexcepts; inline;
      begin
      end;


    procedure checkexcepts; inline;
      begin
      end;

{$endif aix}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{$ifndef SOLARIS}

{$ifndef FPC_SYSTEM_HAS_INT}
{$define FPC_SYSTEM_HAS_INT}

{$ifdef SUPPORT_DOUBLE}
    function c_trunc(d: double): double; cdecl; external 'c' name 'trunc';

    function fpc_int_real(d: ValReal): ValReal;compilerproc; {$ifdef MATHINLINE}inline;{$endif}
    begin
      resetexcepts;
      result := c_trunc(d);
      checkexcepts;
    end;


{$else SUPPORT_DOUBLE}

    function c_truncf(d: single): double; cdecl; external 'c' name 'truncf';

    function fpc_int_real(d: ValReal): ValReal;compilerproc; {$ifdef MATHINLINE}inline;{$endif}
      begin
        { this will be correct since real = single in the case of }
        { the motorola version of the compiler...                 }
        resetexcepts;
        int:=c_truncf(d);
        checkexcepts;
      end;
{$endif SUPPORT_DOUBLE}

{$endif FPC_SYSTEM_HAS_INT}
{$endif SOLARIS}

{$ifndef SYSTEM_HAS_FREXP}
{$define SYSTEM_HAS_FREXP}
    function c_frexp(x: double; out e: longint): double; cdecl; external 'c' name 'frexp';

    function frexp(x:ValReal; out e:Integer ):ValReal; {$ifdef MATHINLINE}inline;{$endif}
    var
      l: longint;
    begin
      resetexcepts;
      frexp := c_frexp(x,l);
      e := l;
      checkexcepts;
    end;
{$endif not SYSTEM_HAS_FREXP}


{$ifndef SYSTEM_HAS_LDEXP}
{$define SYSTEM_HAS_LDEXP}
    function c_ldexp(x: double; n: longint): double; cdecl; external 'c' name 'ldexp';

    function ldexp( x: ValReal; N: Integer):ValReal;{$ifdef MATHINLINE}inline;{$endif}
    begin
      resetexcepts;
      ldexp := c_ldexp(x,n);
      checkexcepts;
    end;
{$endif not SYSTEM_HAS_LDEXP}


{$ifndef FPC_SYSTEM_HAS_SQRT}
{$define FPC_SYSTEM_HAS_SQRT}

    function c_sqrt(d: double): double; cdecl; external 'c' name 'sqrt';

    function fpc_sqrt_real(d:ValReal):ValReal;compilerproc; {$ifdef MATHINLINE}inline;{$endif}
    begin
      resetexcepts;
      result := c_sqrt(d);
      checkexcepts;
    end;

{$endif}


{$ifndef FPC_SYSTEM_HAS_EXP}
{$define FPC_SYSTEM_HAS_EXP}
    function c_exp(d: double): double; cdecl; external 'c' name 'exp';


    function fpc_Exp_real(d:ValReal):ValReal;compilerproc; {$ifdef MATHINLINE}inline;{$endif}
      begin
        resetexcepts;
        result := c_exp(d);
        checkexcepts;
    end;
{$endif}


<<<<<<< HEAD
<<<<<<< HEAD
{ buggy on aix, sets DIV_BY_ZERO flag for some valid inputs }
{$ifndef aix}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ifndef FPC_SYSTEM_HAS_LN}
{$define FPC_SYSTEM_HAS_LN}

    function c_log(d: double): double; cdecl; external 'c' name 'log';

    function fpc_Ln_real(d:ValReal):ValReal;compilerproc;{$ifdef MATHINLINE}inline;{$endif}
    begin
      resetexcepts;
      result := c_log(d);
      checkexcepts;
    end;
{$endif}
{$endif}


{$ifndef FPC_SYSTEM_HAS_SIN}
{$define FPC_SYSTEM_HAS_SIN}
    function c_sin(d: double): double; cdecl; external 'c' name 'sin';

    function fpc_Sin_real(d:ValReal):ValReal;compilerproc; {$ifdef MATHINLINE}inline;{$endif}
    begin
      resetexcepts;
      result := c_sin(d);
      checkexcepts;
    end;
{$endif}



{$ifndef FPC_SYSTEM_HAS_COS}
{$define FPC_SYSTEM_HAS_COS}
    function c_cos(d: double): double; cdecl; external 'c' name 'cos';

    function fpc_Cos_real(d:ValReal):ValReal;compilerproc; {$ifdef MATHINLINE}inline;{$endif}
    begin
      resetexcepts;
      result := c_cos(d);
      checkexcepts;
    end;
{$endif}



{$ifndef FPC_SYSTEM_HAS_ARCTAN}
{$define FPC_SYSTEM_HAS_ARCTAN}
    function c_atan(d: double): double; cdecl; external 'c' name 'atan';

    function fpc_ArcTan_real(d:ValReal):ValReal;compilerproc; {$ifdef MATHINLINE}inline;{$endif}
    begin
      resetexcepts;
      result := c_atan(d);
      checkexcepts;
    end;
{$endif}

{$endif not FPC_HAS_TYPE_EXTENDED}

