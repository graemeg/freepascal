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

{$asmmode att}

var
  dlexitproc: pointer; { atexit from loader }

<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
procedure libc_exit; external name '__libc_exit';
procedure libc_init; external name '__libc_init';
procedure libc_setfpucw; external name '__setfpucw';
>>>>>>> graemeg/fixes_2_2
procedure libc_start_main; external name '__libc_start_main';
procedure PASCALMAIN; external name 'PASCALMAIN';

{ Some helpers }

procedure _init_fini_dummy; compilerproc; nostackframe; assembler;
asm
  ret
end; 

{******************************************************************************
                       glibc 2.1 lib + profiling start/halt
 ******************************************************************************}
{$asmmode ATT}

procedure _FPC_libc21_start; assembler; nostackframe; public name '_start';
asm
  xorl    %ebp,%ebp
  { First locate the start of the environment variables }

  popl    %ecx                    { Get argc in ecx }

  movl    %esp,%ebx               { Esp now points to the arguments }
  leal    4(%esp,%ecx,4),%eax     { The start of the environment is: esp+4*eax+4 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  andl    $0xfffffff0,%esp        { Align stack to 16 bytes }
=======
=======
>>>>>>> origin/cpstrnew
  andl    $0xfffffff8,%esp        { Align stack }
>>>>>>> graemeg/cpstrnew
=======
  andl    $0xfffffff8,%esp        { Align stack }
>>>>>>> graemeg/cpstrnew
=======
  andl    $0xfffffff8,%esp        { Align stack }
>>>>>>> graemeg/cpstrnew

  {$ifdef FPC_PIC}
        pushl %edx
        pushl %ebx
        pushl %ecx
	
	call .Lpiclab
.Lpiclab:
        popl  %ebx
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        addl  $_GLOBAL_OFFSET_TABLE_+1,%ebx
=======
        addl  $_GLOBAL_OFFSET_TABLE_,%ebx
>>>>>>> graemeg/cpstrnew
=======
        addl  $_GLOBAL_OFFSET_TABLE_,%ebx
>>>>>>> graemeg/cpstrnew
=======
        addl  $_GLOBAL_OFFSET_TABLE_,%ebx
>>>>>>> graemeg/cpstrnew
=======
        addl  $_GLOBAL_OFFSET_TABLE_,%ebx
>>>>>>> origin/cpstrnew

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
	popl  %edx
  {$else FPC_PIC}
  	movl  %edx, dlexitproc
  	movl  %eax,operatingsystem_parameter_envp
  	movl  %ecx,operatingsystem_parameter_argc
  	movl  %ebx,operatingsystem_parameter_argv
  {$endif FPC_PIC}

  { Save initial stackpointer }
  {$ifdef FPC_PIC}
        pushl %ebx
        call  .Lpiclab2
.Lpiclab2:
        popl  %ebx
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        addl  $_GLOBAL_OFFSET_TABLE_+1,%ebx
=======
        addl  $_GLOBAL_OFFSET_TABLE_,%ebx
>>>>>>> graemeg/cpstrnew
=======
        addl  $_GLOBAL_OFFSET_TABLE_,%ebx
>>>>>>> graemeg/cpstrnew
=======
        addl  $_GLOBAL_OFFSET_TABLE_,%ebx
>>>>>>> graemeg/cpstrnew
=======
        addl  $_GLOBAL_OFFSET_TABLE_,%ebx
>>>>>>> origin/cpstrnew
	movl  initialstkptr@GOT(%ebx),%ebx
  	movl  %esp,(%ebx)
	popl  %ebx
  {$else FPC_PIC}
  	movl    %esp,initialstkptr
  {$endif FPC_PIC}

        { int __libc_start_main(
		int *(main) (int, char * *, char * *), 
		int argc, 
		char * * ubp_av, 
		void (*init) (void), 
		void (*fini) (void), 
		void (*rtld_fini) (void), 
		void (* stack_end)); } 

        pushl %ebp   			{ padding }
        pushl %esp   			{ stack_end }
        pushl %edx   			{ function to be registered with
                      			  atexit(), passed by loader }
  	pushl $_init_fini_dummy
  	pushl $_init_fini_dummy
	pushl %ebx             		{ Push second argument: argv.  }
	pushl %ecx             		{ Push first argument: argc.  }

	pushl $PASCALMAIN

  	call  libc_start_main
	hlt
end;

procedure _FPC_libc21_haltproc(e: longint); cdecl; assembler; public name '_haltproc';
asm
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
.Lhaltproc:
>>>>>>> graemeg/cpstrnew
=======
.Lhaltproc:
>>>>>>> graemeg/cpstrnew
=======
.Lhaltproc:
>>>>>>> graemeg/cpstrnew
=======
.Lhaltproc:
>>>>>>> origin/cpstrnew
  {$ifdef FPC_PIC}
        call  .Lpiclab
.Lpiclab:
        popl  %ebx
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        addl  $_GLOBAL_OFFSET_TABLE_+1,%ebx
=======
        addl  $_GLOBAL_OFFSET_TABLE_,%ebx
>>>>>>> graemeg/cpstrnew
=======
        addl  $_GLOBAL_OFFSET_TABLE_,%ebx
>>>>>>> graemeg/cpstrnew
=======
        addl  $_GLOBAL_OFFSET_TABLE_,%ebx
>>>>>>> graemeg/cpstrnew
=======
        addl  $_GLOBAL_OFFSET_TABLE_,%ebx
>>>>>>> origin/cpstrnew
        movl  dlexitproc@GOT(%ebx),%eax
        movl  (%eax),%eax
  {$else FPC_PIC}
        movl    dlexitproc,%eax
  {$endif FPC_PIC}
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

  int     $0x80
  jmp     .Lnodlexitproc
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{$ifdef FPC_PIC}
  call    .Lpiclab2
.Lpiclab2:
  popl    %ebx
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
  call    .Lpiclab3
.Lpiclab3:
  popl    %ebx
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
  jmp     .Lhaltproc
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
end;
