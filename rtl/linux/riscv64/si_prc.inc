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

procedure _FPC_rv_enter(sp: pptruint);
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

procedure _FPC_proc_start; assembler; nostackframe; public name '_start';
  asm
    { set up GP }
    .option push
    .option norelax
.L1:
    auipc gp, %pcrel_hi(BSS_START+0x800)
    addi  gp, gp, %pcrel_lo(.L1)
    .option pop

    { Initialise FP to zero }
    addi fp, x0, 0

    addi a0, sp, 0
    jal x1, _FPC_rv_enter
  end;


procedure _FPC_dynamic_proc_start; assembler; nostackframe; public name '_dynamic_start';
  asm
    .option push
    .option norelax
.L1:
    auipc t0, %pcrel_hi(dlexitproc)
{$ifdef RISCV64}
    sd    a0, %pcrel_lo(.L1)(t0)
{$else 32-bit code }
    sw    a0, %pcrel_lo(.L1)(t0)
{$endif}
    .option pop

    jal   x0, _FPC_proc_start
  end;


procedure _FPC_rv_exit(e:longint); assembler; nostackframe;
  asm
.L1:
    addi  a7, x0, 94
    ecall
    jal x0, .L1
  end;


procedure _FPC_proc_haltproc(e:longint); cdecl; public name '_haltproc';
  begin
    if assigned(dlexitproc) then
      TProcedure(dlexitproc);
    _FPC_rv_exit(e);
  end;
