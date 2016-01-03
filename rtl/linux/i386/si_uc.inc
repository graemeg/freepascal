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
 Linux/uClibc startup code for Free Pascal
    taken from uClibc/sysdeps/linux/i386/crt1.S

 %edx  Contains a function pointer to be registered with `atexit'.
       This is how the dynamic linker arranges to have DT_FINI
       functions called for shared libraries that have been loaded
       before this code runs.
   

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
var
  dlexitproc: pointer; { atexit from loader }

procedure uclibc_init; external name '__uClibc_init';
procedure uclibc_fini; external name '__uClibc_fini';
procedure uclibc_exit; external name 'exit';
procedure uclibc_main; external name '__uClibc_main';
procedure PASCALMAIN; external name 'PASCALMAIN';

{ Some helpers }

{$ifdef FPC_PIC}
function get3eipasebx : pointer; compilerproc; nostackframe; assembler; 
asm  
  movl (%esp),%ebx  
  ret  
end;
{$endif FPC_PIC}

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
  andl    $0xfffffff8,%esp        { Align stack }

  {$ifdef FPC_PIC}
        pushl %edx
        pushl %ebx
        pushl %ecx
	
	call get3eipasebx
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
        call  get3eipasebx
        addl  $_GLOBAL_OFFSET_TABLE_,%ebx
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

  	call  uclibc_main
	hlt
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure _FPC_libc21_haltproc(e: longint); assembler; public name '_haltproc';
asm
.Lhaltproc:
  movl    e,%ebx
=======
procedure _FPC_libc21_haltproc; assembler; nostackframe; public name '_haltproc';
asm
.Lhaltproc:
=======
procedure _FPC_libc21_haltproc; assembler; nostackframe; public name '_haltproc';
asm
.Lhaltproc:
>>>>>>> graemeg/cpstrnew
=======
procedure _FPC_libc21_haltproc; assembler; nostackframe; public name '_haltproc';
asm
.Lhaltproc:
>>>>>>> graemeg/cpstrnew
=======
procedure _FPC_libc21_haltproc; assembler; nostackframe; public name '_haltproc';
asm
.Lhaltproc:
>>>>>>> origin/cpstrnew

  {$ifdef FPC_PIC}
        call    get3eipasebx
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
  pushl   %ebx
  call    uclibc_exit
=======
=======
>>>>>>> origin/fixes_2_2
procedure libc_init; external name '__uClibc_init';
procedure libc_fini; external name '__uClibc_fini';
procedure libc_exit; external name '_exit';
procedure libc_main; external name '__uClibc_main';

procedure PASCALMAIN; external name 'PASCALMAIN';

{******************************************************************************
                          C library start/halt
 ******************************************************************************}
{$asmmode ATT}

procedure _FPC_libc_start; assembler; nostackframe; public name '_start';
asm
  xorl    %ebp,%ebp               { clear outer most frame for backtraces }
  popl    %esi                    { Get argc in ecx }
  movl    %esp,%ecx               { Esp now points to the arguments }
  leal    4(%esp,%esi,4),%eax     { The start of the environment is: esp+4*eax+8 }
  andl    $0xfffffff0,%esp        { Align stack }
  pushl   %eax                    { push garbage, so we push 32 bytes in total }

  movl    %eax,operatingsystem_parameter_envp    { save the environment pointer }
  movl    %esi,operatingsystem_parameter_argc    { save the argument counter    }
  movl    %ecx,operatingsystem_parameter_argv    { save the argument pointer    }
  movl    %esp,initialstkptr                     { save initial stack pointer   }

  pushl   %esp            { provide highest stack address to C library }
  pushl   %edx            { push address of shared library finalization }

{$ifdef PIC}
  call    .L0
.L0:
  pop     %ebx
  addl    $_GLOBAL_OFFSET_TABLE_+[.-.L0],%ebx

  pushl   _fini@GOT(%ebx) { push address of entry points }
  pushl   _init@GOT(%ebx)

  pushl   %ecx            { push argv }                                 
  pushl   %esi            { push argc }
                                                                       
  pushl   $PASCALMAIN     { push fpc main procedure }
  call    libc_main       { let fpc main be called from libc startup }
{$else}
  pushl   $libc_fini      { push address of entry points }
  pushl   $libc_init

  pushl   %ecx            { push argv }
  pushl   %esi            { push argc }

  pushl   $PASCALMAIN     { push fpc main procedure }
  call    libc_main       { let fpc main be called from libc startup }
{$endif}
end;

procedure _FPC_libc_haltproc; assembler; nostackframe; public name '_haltproc';
asm
.Lhaltproc:
{$if sizeof(ExitCode)=2}
  movzwl  ExitCode,%ebx
{$else}
  mov     ExitCode,%ebx
{$endif}
  pushl   %ebx
  call    libc_exit
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  xorl    %eax,%eax
  incl    %eax                    { eax=1, exit call }
  popl    %ebx
  int     $0x80
  jmp     .Lhaltproc
<<<<<<< HEAD
<<<<<<< HEAD

end;
=======
end;

>>>>>>> graemeg/fixes_2_2
=======
end;

>>>>>>> origin/fixes_2_2
