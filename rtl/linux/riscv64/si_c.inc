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

  libc_init_proc: TProcedure; weakexternal name '_init';
  libc_fini_proc: TProcedure; weakexternal name '_fini';

procedure libc_start_main(main: TProcedure; argc: ptruint; argv: ppchar; init, fini, rtld_fini: TProcedure; stack_end: pointer); cdecl; external name '__libc_start_main';
procedure libc_exit(code: ptruint); cdecl; external name 'exit';

procedure _FPC_rv_enter(at_exit: TProcedure; sp: pptruint);
  var
    argc: ptruint;
    argv: ppchar;
  begin
    argc:=sp[0];
    argv:=@sp[1];

    initialstkptr:=sp;
    operatingsystem_parameter_argc:=argc;
    operatingsystem_parameter_argv:=argv;
    operatingsystem_parameter_envp:=@sp[argc+2];

    libc_start_main(@PascalMain, argc, argv, libc_init_proc, libc_fini_proc, at_exit, sp);
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

    { atexit is in a0 }
    addi a1, sp, 0
    jal x1, _FPC_rv_enter
  end;


procedure _FPC_rv_exit(e:longint); assembler; nostackframe;
  asm
    addi  a7, x0, 94
    ecall
  end;


procedure _FPC_proc_haltproc(e:longint); cdecl; public name '_haltproc';
  begin
    while true do
      begin
        libc_exit(e);
        _FPC_rv_exit(e);
      end;
  end;
