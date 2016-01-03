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

function fpc_geteipasebx : pointer; compilerproc; nostackframe; assembler;
asm  
  movl (%esp),%ebx  
  ret  
end;

{******************************************************************************
                          Process start/halt
 ******************************************************************************}
{$asmmode ATT}

var
  dlexitproc: pointer;
<<<<<<< HEAD

var
  dlexitproc : pointer;

{$ifdef FPC_PIC}
function fpc_geteipasebxlocal : pointer; [external name 'fpc_geteipasebx'];
{$endif}
=======
>>>>>>> origin/fixes_2_2

procedure _FPC_proc_start; assembler; nostackframe; public name '_start';
asm
  { First locate the start of the environment variables }

  popl    %ecx                    { Get argc in ecx }

  movl    %esp,%ebx               { Esp now points to the arguments }
  leal    4(%esp,%ecx,4),%eax     { The start of the environment is: esp+4*eax+4 }
<<<<<<< HEAD
  andl    $0xfffffff0,%esp        { Align stack to 16 bytes }
=======
  andl    $0xfffffff8,%esp        { Align stack }
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew

  {$ifdef FPC_PIC}
        pushl %ebx
        pushl %ecx
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

<<<<<<< HEAD
        call fpc_geteipasebxlocal
        addl  $_GLOBAL_OFFSET_TABLE_,%ebx

	movl  operatingsystem_parameter_envp@GOT(%ebx),%ecx
	movl  %eax,(%ecx)

	movl  operatingsystem_parameter_argc@GOT(%ebx),%edx
	popl  %ecx
	movl  %ecx,(%edx)

	movl  operatingsystem_parameter_argv@GOT(%ebx),%edx
	popl  %ebx
	movl  %ebx,(%edx)
  {$else FPC_PIC}
  	movl  %eax,operatingsystem_parameter_envp
  	movl  %ecx,operatingsystem_parameter_argc
  	movl  %ebx,operatingsystem_parameter_argv
  {$endif FPC_PIC}

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
	
	call fpc_geteipasebx
        addl  $_GLOBAL_OFFSET_TABLE_,%ebx

	movl  dlexitproc@GOT(%ebx),%ecx
	movl  %edx,(%ecx)

	movl  operatingsystem_parameter_envp@GOT(%ebx),%ecx
	movl  %eax,(%ecx)

	movl  operatingsystem_parameter_argc@GOT(%ebx),%edx
	popl  %ecx
	movl  %ecx,(%edx)

	movl  operatingsystem_parameter_argv@GOT(%ebx),%edx
	popl  %ebx
	movl  %ebx,(%edx)
  {$else FPC_PIC}
  	movl  %edx, dlexitproc
  	movl  %eax,operatingsystem_parameter_envp
  	movl  %ecx,operatingsystem_parameter_argc
  	movl  %ebx,operatingsystem_parameter_argv
  {$endif FPC_PIC}

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
  {$ifdef FPC_PIC}
        pushl %ebx
        pushl %ecx
	
	call fpc_geteipasebx
        addl  $_GLOBAL_OFFSET_TABLE_,%ebx

	movl  dlexitproc@GOT(%ebx),%ecx
	movl  %edx,(%ecx)

<<<<<<< HEAD
	movl  operatingsystem_parameter_envp@GOT(%ebx),%ecx
	movl  %eax,(%ecx)
=======
  movl    %edx, dlexitproc

  movl    %edx, dlexitproc

  fninit                           { initialize fpu }
  fwait
  fldcw   Default8087CW
>>>>>>> graemeg/fixes_2_2

	movl  operatingsystem_parameter_argc@GOT(%ebx),%edx
	popl  %ecx
	movl  %ecx,(%edx)

	movl  operatingsystem_parameter_argv@GOT(%ebx),%edx
	popl  %ebx
	movl  %ebx,(%edx)
  {$else FPC_PIC}
  	movl  %edx, dlexitproc
  	movl  %eax,operatingsystem_parameter_envp
  	movl  %ecx,operatingsystem_parameter_argc
  	movl  %ebx,operatingsystem_parameter_argv
  {$endif FPC_PIC}

>>>>>>> origin/cpstrnew
  { Initialize FPU }
  call SysResetFPU

  { Save initial stackpointer }
  {$ifdef FPC_PIC}
        pushl %ebx
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        call  fpc_geteipasebxlocal
=======
        call  fpc_geteipasebx
>>>>>>> graemeg/cpstrnew
=======
        call  fpc_geteipasebx
>>>>>>> graemeg/cpstrnew
=======
        call  fpc_geteipasebx
>>>>>>> graemeg/cpstrnew
=======
        call  fpc_geteipasebx
>>>>>>> origin/cpstrnew
        addl  $_GLOBAL_OFFSET_TABLE_,%ebx
	movl  initialstkptr@GOT(%ebx),%ebx
  	movl  %esp,(%ebx)
	popl  %ebx
  {$else FPC_PIC}
  	movl    %esp,initialstkptr
  {$endif FPC_PIC}

  xorl    %ebp,%ebp
  call    PASCALMAIN
end;

procedure _FPC_dynamic_proc_start; assembler; nostackframe; public name '_dynamic_start';
asm
  {$ifdef FPC_PIC}
  pushl %ebx
  call fpc_geteipasebxlocal
  addl  $_GLOBAL_OFFSET_TABLE_,%ebx

  movl  dlexitproc@GOT(%ebx),%ebx
  movl  %edx,(%ebx)
  popl  %ebx
  {$else}
  movl  %edx, dlexitproc
  {$endif}
  jmp _FPC_proc_start
end;


procedure _FPC_proc_haltproc(e:longint); cdecl; assembler; public name '_haltproc';
asm
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  //addl  $12, %esp  { align stack back to 16 bytes }
  {$ifdef FPC_PIC}
        call  fpc_geteipasebxlocal
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

.Lhaltproc:
<<<<<<< HEAD
<<<<<<< HEAD
  {$ifdef FPC_PIC}
        call  fpc_geteipasebx
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
        addl  $_GLOBAL_OFFSET_TABLE_,%ebx
	movl  dlexitproc@GOT(%ebx),%eax
	movl  (%eax),%eax
  {$else FPC_PIC}
  	movl    dlexitproc,%eax
  {$endif FPC_PIC}
=======
  movl    dlexitproc,%eax
>>>>>>> graemeg/fixes_2_2
=======
  movl    dlexitproc,%eax
>>>>>>> origin/fixes_2_2
  testl   %eax,%eax
  je      .Lnodlexitproc
  call    *%eax
.Lnodlexitproc:
  movl    syscall_nr_exit_group,%eax

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  movl    e,%ebx
  int     $0x80
  movl    syscall_nr_exit,%eax

  movl    e,%ebx
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifdef FPC_PIC}
  call    fpc_geteipasebx
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

  int     $0x80
  movl    syscall_nr_exit,%eax

{$ifdef FPC_PIC}
  call    fpc_geteipasebx
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
  int     $0x80
  jmp     .Lnodlexitproc
end;

