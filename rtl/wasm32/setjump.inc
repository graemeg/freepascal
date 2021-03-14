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

function fpc_setjmp(var S : jmp_buf) : longint;[Public, alias : 'FPC_SETJMP'];compilerproc;
  begin
    fpc_setjmp:=0;
  end;


procedure fpc_longjmp(var S : jmp_buf;value : longint);[Public, alias : 'FPC_LONGJMP'];compilerproc;
  begin
{$ifndef embedded}
    DebugWriteLn('LONGJMP not supported in WebAssembly!!!');
{$endif embedded}
    fpc_wasm32_unreachable;
  end;
