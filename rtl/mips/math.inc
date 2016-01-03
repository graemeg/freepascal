{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2000 by Jonas Maebe and other members of the
    Free Pascal development team

    Implementation of mathamatical Routines (only for real)

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
{$ifdef INTERNCONSTINTF}

>>>>>>> graemeg/cpstrnew
=======
{$ifdef INTERNCONSTINTF}

>>>>>>> graemeg/cpstrnew
=======
{$ifdef INTERNCONSTINTF}

>>>>>>> graemeg/cpstrnew
=======
{$ifdef INTERNCONSTINTF}

>>>>>>> origin/cpstrnew
{$ifndef FPC_SYSTEM_HAS_ABS}
    {$define FPC_SYSTEM_HAS_ABS}
    function fpc_abs_real(d : valreal) : valreal;compilerproc;
    begin
      { Function is handled internal in the compiler }
      runerror(207);
      result:=0;
    end;
{$endif}

{$ifndef FPC_SYSTEM_HAS_SQR}
    {$define FPC_SYSTEM_HAS_SQR}
    function fpc_sqr_real(d : valreal) : valreal;compilerproc;
    begin
      { Function is handled internal in the compiler }
      runerror(207);
      result:=0;
    end;
{$endif}

{$ifndef FPC_SYSTEM_HAS_SQRT}
    {$define FPC_SYSTEM_HAS_SQRT}
    function fpc_sqrt_real(d : valreal) : valreal;compilerproc;
    begin
      { Function is handled internal in the compiler }
      runerror(207);
      result:=0;
    end;
{$endif}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$else}

    {$define FPC_SYSTEM_HAS_ABS}
    function abs(d : extended) : extended;[internproc:fpc_in_abs_real];

    {$define FPC_SYSTEM_HAS_SQR}
    function sqr(d : extended) : extended;[internproc:fpc_in_sqr_real];

    {$define FPC_SYSTEM_HAS_SQRT}
    function sqrt(d : extended) : extended;[internproc:fpc_in_sqrt_real];

{$endif}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
