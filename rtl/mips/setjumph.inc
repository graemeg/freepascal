{******************************************************************************
    This file is part of the Free Pascal run time library.
    Copyright (c) 2000-2002 by Jonas Maebe and David Zhang

    SetJmp/Longjmp declarations MIPS


    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

******************************************************************************}

type
  jmp_buf=packed record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    ra,sp,s0,s1,s2,s3,s4,s5,s6,s7,fp,gp: longint;
{$if defined(FPUMIPS2) or defined(FPUMIPS3)}
    _fcsr,f20,f21,f22,f23,f24,f25,f26,f27,f28,f29,f30,f31: longint;
{$endif FPUMIPS2 or FPUMIPS3}
=======
    reg:array[1..12] of longint;
>>>>>>> graemeg/cpstrnew
=======
    reg:array[1..12] of longint;
>>>>>>> graemeg/cpstrnew
=======
    reg:array[1..12] of longint;
>>>>>>> graemeg/cpstrnew
=======
    reg:array[1..12] of longint;
>>>>>>> origin/cpstrnew
  end;
  Pjmp_buf=^jmp_buf;

function setjmp(var S:jmp_buf):longint;[external name 'FPC_SETJMP'];
procedure longjmp(var S:jmp_buf;value:longint);[external name 'FPC_LONGJMP'];
