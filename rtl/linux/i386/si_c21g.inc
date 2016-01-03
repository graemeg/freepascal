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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$asmmode att}

{ Some helpers }

<<<<<<< HEAD
{$ifdef FPC_PIC}
function fpc0geteipasebx : pointer; compilerproc; nostackframe; assembler; 
asm  
  movl (%esp),%ebx  
  ret  
end;
{$endif FPC_PIC}

procedure _init_fini_dummy; compilerproc; nostackframe; assembler;
asm
  ret
end;

=======
{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2005 by Michael Van Canneyt, Peter Vreman,
    & Daniel Mantione, members of the Free Pascal development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

=======
{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2005 by Michael Van Canneyt, Peter Vreman,
    & Daniel Mantione, members of the Free Pascal development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

>>>>>>> graemeg/cpstrnew
=======
{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2005 by Michael Van Canneyt, Peter Vreman,
    & Daniel Mantione, members of the Free Pascal development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2005 by Michael Van Canneyt, Peter Vreman,
    & Daniel Mantione, members of the Free Pascal development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
procedure libc_atexit; external name 'atexit';
procedure libc_exit; external name '__libc_exit';
procedure libc_init; external name '__libc_init';
procedure libc_setfpucw; external name '__setfpucw';
procedure libc_start_main; external name '__libc_start_main';
>>>>>>> graemeg/fixes_2_2

{ Some helpers }

{$ifdef FPC_PIC}
function fpc0geteipasebx : pointer; compilerproc; nostackframe; assembler; 
asm  
  movl (%esp),%ebx  
  ret  
end;
{$endif FPC_PIC}

procedure _init_fini_dummy; compilerproc; nostackframe; assembler;
asm
  ret
end;

>>>>>>> graemeg/cpstrnew
=======

{ Some helpers }

=======

{ Some helpers }

>>>>>>> graemeg/cpstrnew
=======

{ Some helpers }

>>>>>>> origin/cpstrnew
{$ifdef FPC_PIC}
function fpc0geteipasebx : pointer; compilerproc; nostackframe; assembler; 
asm  
  movl (%esp),%ebx  
  ret  
end;
{$endif FPC_PIC}

procedure _init_fini_dummy; compilerproc; nostackframe; assembler;
asm
  ret
end;

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure gmon_mcleanup; cdecl; external name '_mcleanup';
procedure gmon_monstartup (main,etext : pointer);cdecl;external name 'monstartup';

procedure libc_start_main; external name '__libc_start_main';
procedure PASCALMAIN; external name 'PASCALMAIN';

var
  dlexitproc: pointer; { atexit from loader }

  gmon_start : record end;external name 'PASCALMAIN';
  gmon_etext : record end;external name '_etext';

  gmon_monstarted: longint = 0;

{******************************************************************************
                          Process start/halt
 ******************************************************************************}
{$asmmode ATT}

procedure _FPC_libc21_gprof_gmon_start; public name '_FPC_libc21_gprof_gmon_start';
begin
  if gmon_monstarted=0 then
  begin
    inc(gmon_monstarted);
    gmon_monstartup(@gmon_start,@gmon_etext);
  end;
  PASCALMAIN;
end;

procedure _FPC_libc21_gprof_start; assembler; nostackframe; public name '_start';
asm
  xorl    %ebp,%ebp
  { First locate the start of the environment variables }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

  popl    %ecx                    { Get argc in ecx }

  movl    %esp,%ebx               { Esp now points to the arguments }
  leal    4(%esp,%ecx,4),%eax     { The start of the environment is: esp+4*eax+4 }
  andl    $0xfffffff0,%esp        { Align stack to 16 bytes }

  {$ifdef FPC_PIC}
        pushl %edx
        pushl %ebx
        pushl %ecx
	
	call fpc0geteipasebx
        addl  $_GLOBAL_OFFSET_TABLE_,%ebx

	movl  dlexitproc@GOT(%ebx),%ecx
	movl  %edx,(%ecx)

	movl  operatingsystem_parameter_envp@GOT(%ebx),%ecx
	movl  %eax,(%ecx)

	movl  operatingsystem_parameter_argc@GOT(%ebx),%edx
	popl  %ecx
	movl  %ecx,(%edx)

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

  popl    %ecx                    { Get argc in ecx }

  movl    %esp,%ebx               { Esp now points to the arguments }
  leal    4(%esp,%ecx,4),%eax     { The start of the environment is: esp+4*eax+4 }
  andl    $0xfffffff8,%esp        { Align stack }

  {$ifdef FPC_PIC}
        pushl %edx
        pushl %ebx
        pushl %ecx
	
	call fpc0geteipasebx
        addl  $_GLOBAL_OFFSET_TABLE_,%ebx

	movl  dlexitproc@GOT(%ebx),%ecx
	movl  %edx,(%ecx)
<<<<<<< HEAD

	movl  operatingsystem_parameter_envp@GOT(%ebx),%ecx
	movl  %eax,(%ecx)

	movl  operatingsystem_parameter_argc@GOT(%ebx),%edx
	popl  %ecx
	movl  %ecx,(%edx)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======

	movl  operatingsystem_parameter_envp@GOT(%ebx),%ecx
	movl  %eax,(%ecx)

	movl  operatingsystem_parameter_argc@GOT(%ebx),%edx
	popl  %ecx
	movl  %ecx,(%edx)

>>>>>>> origin/cpstrnew
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
        call  fpc0geteipasebx
        addl  $_GLOBAL_OFFSET_TABLE_,%ebx
	movl  initialstkptr@GOT(%ebx),%ebx
  	movl  %esp,(%ebx)
	popl  %ebx
  {$else FPC_PIC}
  	movl    %esp,initialstkptr
  {$endif FPC_PIC}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

        { int __libc_start_main(
		int *(main) (int, char * *, char * *), 
		int argc, 
		char * * ubp_av, 
		void (*init) (void), 
		void (*fini) (void), 
		void (*rtld_fini) (void), 
		void (* stack_end)); } 
<<<<<<< HEAD

        pushl %ebp   			{ padding }
        pushl %esp   			{ stack_end }
        pushl %edx   			{ function to be registered with
                      			  atexit(), passed by loader }
        pushl $_init_fini_dummy
        pushl $_init_fini_dummy
        pushl %ebx                      { Push second argument: argv.  }
        pushl %ecx                      { Push first argument: argc.  }

        pushl $_FPC_libc21_gprof_gmon_start

        call  libc_start_main
	hlt
end;

procedure _FPC_libc21_gprof_haltproc(e:longint);cdecl;public name '_haltproc';
begin
  if gmon_monstarted=1 then
  begin
    dec(gmon_monstarted);
    gmon_mcleanup;
  end;
  asm
.Lhaltproc:
  {$ifdef FPC_PIC}
        call  fpc0geteipasebx
        addl  $_GLOBAL_OFFSET_TABLE_,%ebx
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

{$ifdef FPC_PIC}
  call    fpc0geteipasebx
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
  call    fpc0geteipasebx
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
 end;
=======

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
        pushl %ebx                      { Push second argument: argv.  }
        pushl %ecx                      { Push first argument: argc.  }

        pushl $_FPC_libc21_gprof_gmon_start

        call  libc_start_main
	hlt
>>>>>>> origin/cpstrnew
end;
=======

<<<<<<< HEAD
=======

        { int __libc_start_main(
		int *(main) (int, char * *, char * *), 
		int argc, 
		char * * ubp_av, 
		void (*init) (void), 
		void (*fini) (void), 
		void (*rtld_fini) (void), 
		void (* stack_end)); } 

>>>>>>> graemeg/cpstrnew
=======

        { int __libc_start_main(
		int *(main) (int, char * *, char * *), 
		int argc, 
		char * * ubp_av, 
		void (*init) (void), 
		void (*fini) (void), 
		void (*rtld_fini) (void), 
		void (* stack_end)); } 

>>>>>>> graemeg/cpstrnew
        pushl %ebp   			{ padding }
        pushl %esp   			{ stack_end }
        pushl %edx   			{ function to be registered with
                      			  atexit(), passed by loader }
        pushl $_init_fini_dummy
        pushl $_init_fini_dummy
        pushl %ebx                      { Push second argument: argv.  }
        pushl %ecx                      { Push first argument: argc.  }

        pushl $_FPC_libc21_gprof_gmon_start

        call  libc_start_main
	hlt
end;

=======
>>>>>>> origin/cpstrnew
procedure _FPC_libc21_gprof_haltproc(e:longint);cdecl;public name '_haltproc';
begin
  if gmon_monstarted=1 then
  begin
    dec(gmon_monstarted);
    gmon_mcleanup;
  end;
  asm
.Lhaltproc:
  {$ifdef FPC_PIC}
        call  fpc0geteipasebx
        addl  $_GLOBAL_OFFSET_TABLE_,%ebx
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

{$ifdef FPC_PIC}
  call    fpc0geteipasebx
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
  call    fpc0geteipasebx
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
 end;
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
