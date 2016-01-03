#
#   This file is part of the Free Pascal run time library.
#   Copyright (c) 2006 by Florian Klaempfl
#   members of the Free Pascal development team.
#
#   See the file COPYING.FPC, included in this distribution,
#   for details about the copyright.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY;without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#
#**********************************************************************}
#
# Linux ELF startup code for Free Pascal
#

/* This is the canonical entry point, usually the first thing in the text
   segment.  The SVR4/i386 ABI (pages 3-31, 3-32) says that when the entry
   point runs, most registers' values are unspecified, except for:

   %rdx		Contains a function pointer to be registered with `atexit'.
		This is how the dynamic linker arranges to have DT_FINI
		functions called for shared libraries that have been loaded
		before this code runs.

   %rsp		The stack contains the arguments and environment:
		0(%rsp)			argc
		8(%rsp)			argv[0]
		...
		(8*argc)(%rsp)		NULL
		(8*(argc+1))(%rsp)	envp[0]
		...
					NULL
*/
.section .init
	.align 16
<<<<<<< HEAD
<<<<<<< HEAD
	.globl FPC_SHARED_LIB_START
	.type FPC_SHARED_LIB_START,@function
FPC_SHARED_LIB_START:
	jmp	_startlib@PLT

        .text
	.globl _startlib
	.type _startlib,@function
_startlib:
=======
=======
>>>>>>> origin/fixes_2_2
	.globl FPC_LIB_START
	.type FPC_LIB_START,@function
FPC_LIB_START:
	jmp	_startlib@PLT

        .text
	.globl _start
	.type _start,@function
_startlib:
#       movq %rdx,%r9                 /* Address of the shared library termination
#               	                 function.  */
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
	pushq	 %rbx
        movq     operatingsystem_parameter_argc@GOTPCREL(%rip),%rbx
        movq     %rdi,(%rbx)
        movq     operatingsystem_parameter_argv@GOTPCREL(%rip),%rbx
	movq     %rsi,(%rbx)          /* argv starts just at the current stack top.  */
        movq     operatingsystem_parameter_envp@GOTPCREL(%rip),%rbx
        movq     %rdx,(%rbx)

<<<<<<< HEAD
<<<<<<< HEAD
        movq    operatingsystem_islibrary@GOTPCREL(%rip),%rbx
=======
        movq    TC_SYSTEM_ISLIBRARY@GOTPCREL(%rip),%rbx
>>>>>>> graemeg/fixes_2_2
=======
        movq    TC_SYSTEM_ISLIBRARY@GOTPCREL(%rip),%rbx
>>>>>>> origin/fixes_2_2
        movb    $1,(%rbx)

        /* Save initial stackpointer */
        movq    __stkptr@GOTPCREL(%rip),%rbx
        movq    %rsp,(%rbx)

        call    PASCALMAIN@PLT
	popq	%rbx
	ret

<<<<<<< HEAD
<<<<<<< HEAD
/* this routine is only called when the halt() routine of the RTL embedded in
  the shared library is called */
        .globl  _haltproc
        .type   _haltproc,@function
_haltproc:
	.globl FPC_SHARED_LIB_EXIT
	.type FPC_SHARED_LIB_EXIT,@function
FPC_SHARED_LIB_EXIT:
=======
        .globl  _haltproc
        .type   _haltproc,@function
_haltproc:
>>>>>>> graemeg/fixes_2_2
=======
        .globl  _haltproc
        .type   _haltproc,@function
_haltproc:
>>>>>>> origin/fixes_2_2
        movl    $231,%eax                 /* exit_group call */
        movq    operatingsystem_result@GOTPCREL(%rip),%rbx
        movzwl  (%rbx),%edi
        syscall
        jmp     _haltproc@PLT

/* Define a symbol for the first piece of initialized data.  */
	.data
	.globl __data_start
__data_start:
	.long 0
	.weak data_start
        data_start = __data_start

.bss
        .comm __stkptr,8

        .comm operatingsystem_parameter_envp,8
        .comm operatingsystem_parameter_argc,8
        .comm operatingsystem_parameter_argv,8


/* We need this stuff to make gdb behave itself, otherwise
   gdb will chokes with SIGILL when trying to debug apps.

Makes ld choke:
        .section ".note.ABI-tag", "a"
        .align 4
        .long 1f - 0f
        .long 3f - 2f
        .long  1
0:      .asciz "GNU"
1:      .align 4
2:      .long 0
        .long 2,4,0
3:      .align 4
*/
	.section	.note.GNU-stack,"",@progbits
