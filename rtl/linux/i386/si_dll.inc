{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2005 by Michael Van Canneyt, Peter Vreman,
    & Daniel Mantione, members of the Free Pascal development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$asmmode att}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

procedure PASCALMAIN; external name 'PASCALMAIN';

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$goto on}
{
 Linux ELF startup code for Free Pascal


 Stack layout at program start:

         nil
         envn
         ....
         ....           ENVIRONMENT VARIABLES
         env1
         env0
         nil
         argn
         ....
         ....           COMMAND LINE OPTIONS
         arg1
         arg0
         argc <--- esp
}

procedure PASCALMAIN; external name 'PASCALMAIN';

function get1eipasebx : pointer; compilerproc; nostackframe; assembler; 
asm  
  movl (%esp),%ebx  
  ret  
end;
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

{******************************************************************************
                        Shared library start/halt
 ******************************************************************************}
{$asmmode ATT}

<<<<<<< HEAD
<<<<<<< HEAD
procedure _FPC_shared_lib_start(argc : dword;argv,envp : pointer); cdecl; public name 'FPC_SHARED_LIB_START'; public name '_start';
=======
procedure _FPC_shared_lib_start(argc : dword;argv,envp : pointer); cdecl; public name '_FPC_SHARED_LIB_START_LOCAL'; public name '_start';
>>>>>>> graemeg/fixes_2_2
=======
procedure _FPC_shared_lib_start(argc : dword;argv,envp : pointer); cdecl; public name '_FPC_SHARED_LIB_START_LOCAL'; public name '_start';
>>>>>>> origin/fixes_2_2
begin

  operatingsystem_parameter_argc:=argc;    { Copy the argument count      }
  operatingsystem_parameter_argv:=argv;    { Copy the argument pointer    }
  operatingsystem_parameter_envp:=envp;    { Copy the environment pointer }
  initialstkptr:=get_frame;

<<<<<<< HEAD
<<<<<<< HEAD
=======
  asm
    { Save initial stackpointer }
    {$ifdef FPC_PIC}
        call  get1eipasebx
        addl  $_GLOBAL_OFFSET_TABLE_,%ebx
        movl  initialstkptr@GOT(%ebx),%ecx
	movl  %esp,(%ecx)
    {$else FPC_PIC}
	movl    %esp,initialstkptr
    {$endif FPC_PIC}
  end;

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
  asm
    { Save initial stackpointer }
    {$ifdef FPC_PIC}
        call  get1eipasebx
        addl  $_GLOBAL_OFFSET_TABLE_,%ebx
        movl  initialstkptr@GOT(%ebx),%ecx
	movl  %esp,(%ecx)
    {$else FPC_PIC}
	movl    %esp,initialstkptr
    {$endif FPC_PIC}
  end;

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
=======
  IsLibrary:=true;

>>>>>>> origin/cpstrnew
  asm
    { Save initial stackpointer }
    {$ifdef FPC_PIC}
        call  get1eipasebx
        addl  $_GLOBAL_OFFSET_TABLE_,%ebx
        movl  initialstkptr@GOT(%ebx),%ecx
	movl  %esp,(%ecx)
    {$else FPC_PIC}
	movl    %esp,initialstkptr
    {$endif FPC_PIC}
  end;

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  PASCALMAIN;
end;

<<<<<<< HEAD
{ this routine is only called when the halt() routine of the RTL embedded in
  the shared library is called }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure _FPC_shared_lib_haltproc(e:longint); cdecl; assembler; nostackframe; public name '_haltproc';
asm
.Lhaltproc:
  movl    e,%ebx
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
procedure _FPC_shared_lib_haltproc; assembler; nostackframe; public name 'FPC_SHARED_LIB_EXIT'; public name '_haltproc';
=======
{$ifndef VER2_0}

{ this hack is needed so we can make the reference below to _FPC_shared_lib_start }
{ local in compiler/systems/t_linux.pas                                           }
procedure _FPC_SHARED_LIB_START_LOCAL(argc : dword;argv,envp : pointer); cdecl; external;

procedure initdummy; assembler; nostackframe;
label
  FPC_LIB_START;
asm
.init
  .align 16
  .globl FPC_LIB_START
//    .type FPC_LIB_START,@function
FPC_LIB_START:
{$ifdef FPC_PIC}
  jmp	_FPC_SHARED_LIB_START_LOCAL@PLT
{$else FPC_PIC}
  jmp	_FPC_SHARED_LIB_START_LOCAL
{$endif FPC_PIC}
.text
end;
{$endif VER_2_0}

procedure _FPC_shared_lib_haltproc; assembler; nostackframe; public name '_haltproc';
>>>>>>> graemeg/fixes_2_2
asm
.Lhaltproc:
=======
procedure _FPC_shared_lib_haltproc; assembler; nostackframe; public name 'FPC_SHARED_LIB_EXIT'; public name '_haltproc';
asm
.Lhaltproc:
>>>>>>> origin/cpstrnew
 {$ifdef FPC_PIC}
  call    get1eipasebx
  addl    $_GLOBAL_OFFSET_TABLE_,%ebx
  movl    ExitCode@GOT(%ebx),%ebx
 {$if sizeof(ExitCode)=2}
  movzwl  (%ebx),%ebx
 {$else}
  mov     (%ebx),%ebx
 {$endif}
{$else FPC_PIC}
 {$if sizeof(ExitCode)=2}
  movzwl  ExitCode,%ebx
 {$else}
  mov     ExitCode,%ebx
 {$endif}
{$endif FPC_PIC}
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
  xorl    %eax,%eax
  incl    %eax                    { eax=1, exit call }
  int     $0x80
  jmp     .Lhaltproc
end;
