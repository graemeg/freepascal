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
                        Shared library start/halt
 ******************************************************************************}

procedure _FPC_shared_lib_start(argc : dword;argv,envp : pointer); cdecl; public name 'FPC_SHARED_LIB_START'; public name '_start';
  begin

    operatingsystem_parameter_argc:=argc;    { Copy the argument count      }
    operatingsystem_parameter_argv:=argv;    { Copy the argument pointer    }
    operatingsystem_parameter_envp:=envp;    { Copy the environment pointer }
    initialstkptr:=get_frame;

    PASCALMAIN;
  end;

{ this routine is only called when the halt() routine of the RTL embedded in
  the shared library is called }
procedure _FPC_shared_lib_haltproc(e:longint); cdecl; assembler; public name '_haltproc';
  asm
  .L1:
    mov a6,a3
    movi a2,119
    syscall
    j .L1
  end;
