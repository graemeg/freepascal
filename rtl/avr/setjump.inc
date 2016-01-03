{

    This file is part of the Free Pascal run time library.
    Copyright (c) 2008 by the Free Pascal development team.

    SetJmp and LongJmp implementation for exception handling

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
function fpc_setjmp(var S : jmp_buf) : smallint;assembler;[Public, alias : 'FPC_SETJMP'];nostackframe;compilerproc;
=======
function fpc_setjmp(var S : jmp_buf) : longint;assembler;[Public, alias : 'FPC_SETJMP'];nostackframe;compilerproc;
>>>>>>> graemeg/cpstrnew
=======
function fpc_setjmp(var S : jmp_buf) : longint;assembler;[Public, alias : 'FPC_SETJMP'];nostackframe;compilerproc;
>>>>>>> graemeg/cpstrnew
=======
function fpc_setjmp(var S : jmp_buf) : longint;assembler;[Public, alias : 'FPC_SETJMP'];nostackframe;compilerproc;
>>>>>>> graemeg/cpstrnew
=======
function fpc_setjmp(var S : jmp_buf) : longint;assembler;[Public, alias : 'FPC_SETJMP'];nostackframe;compilerproc;
>>>>>>> origin/cpstrnew
  asm
  end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure fpc_longjmp(var S : jmp_buf;value : smallint);assembler;[Public, alias : 'FPC_LONGJMP'];compilerproc;
=======
procedure fpc_longjmp(var S : jmp_buf;value : longint);assembler;[Public, alias : 'FPC_LONGJMP'];compilerproc;
>>>>>>> graemeg/cpstrnew
=======
procedure fpc_longjmp(var S : jmp_buf;value : longint);assembler;[Public, alias : 'FPC_LONGJMP'];compilerproc;
>>>>>>> graemeg/cpstrnew
=======
procedure fpc_longjmp(var S : jmp_buf;value : longint);assembler;[Public, alias : 'FPC_LONGJMP'];compilerproc;
>>>>>>> graemeg/cpstrnew
=======
procedure fpc_longjmp(var S : jmp_buf;value : longint);assembler;[Public, alias : 'FPC_LONGJMP'];compilerproc;
>>>>>>> origin/cpstrnew
  asm
  end;


