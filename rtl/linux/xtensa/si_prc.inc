{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2019 by Jeppe Johansen.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{******************************************************************************
                          Process start/halt
 ******************************************************************************}

var
  dlexitproc : pointer;

var
  BSS_START: record end; external name '__bss_start';
  STACK_PTR: record end; external name '__stkptr';

procedure _FPC_xtensa_enter(sp: pptruint);
  var
    argc: ptruint;
  begin
    argc:=sp[0];

    initialstkptr:=sp;
    operatingsystem_parameter_argc:=argc;
    operatingsystem_parameter_argv:=@sp[1];
    operatingsystem_parameter_envp:=@sp[1+argc];

    PascalMain;
  end;

procedure _FPC_proc_start; assembler; public name '_start';
  asm
    { outermost stack frame }
    movi a0,0
    { pass stack pointer }
    mov a6,a1
    call4 _FPC_xtensa_enter
  end;


procedure _FPC_dynamic_proc_start; assembler; public name '_dynamic_start';
  asm
    ill
  end;


procedure _FPC_xtensa_exit(e:longint); assembler;
  asm
  .L1:
    mov a6,a3
    movi a2,119
    syscall
    j .L1
  end;


procedure _FPC_proc_haltproc(e:longint); cdecl; public name '_haltproc';
  begin
    if assigned(dlexitproc) then
      TProcedure(dlexitproc);
    _FPC_xtensa_exit(e);
  end;
