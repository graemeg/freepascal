#
#   This file is part of the Free Pascal run time library.
#   Copyright (c) 2002 by Florian Klaempfl
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
   segment.  The document "System V Application Binary Interface AMD64
   Architecture Processor Supplement Version 0.99.5" pg. 30 defines that
   the entry point runs, most registers' values are unspecified, except
   for:

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

        .text
	.globl _dynamic_start
	.type _dynamic_start,@function
_dynamic_start:
        movq    __dl_fini@GOTPCREL(%rip),%rax
        movq    %rdx,(%rax)
        jmp _start

        .text
	.globl _start
	.type _start,@function
_start:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
#       movq    %rdx,%r9                 /* Address of the shared library termination
#               	                 function.  */
>>>>>>> graemeg/cpstrnew
=======
#       movq    %rdx,%r9                 /* Address of the shared library termination
#               	                 function.  */
>>>>>>> graemeg/cpstrnew
=======
#       movq    %rdx,%r9                 /* Address of the shared library termination
#               	                 function.  */
>>>>>>> graemeg/cpstrnew
=======
#       movq    %rdx,%r9                 /* Address of the shared library termination
#               	                 function.  */
>>>>>>> origin/cpstrnew
	popq    %rsi		      /* Pop the argument count.  */
        movq 	operatingsystem_parameter_argc@GOTPCREL(%rip),%rax
        movq    %rsi,(%rax)
        movq 	operatingsystem_parameter_argv@GOTPCREL(%rip),%rax
	movq    %rsp,(%rax)   /* argv starts just at the current stack top.  */
        leaq    8(,%rsi,8),%rax
        addq    %rsp,%rax
        movq 	operatingsystem_parameter_envp@GOTPCREL(%rip),%rcx
        movq    %rax,(%rcx)
        andq    $~15,%rsp            /* Align the stack to a 16 byte boundary to follow the ABI.  */

        /* Save initial stackpointer */
        movq 	__stkptr@GOTPCREL(%rip),%rax
        movq    %rsp,(%rax)

        xorq    %rbp, %rbp
        call    PASCALMAIN
	jmp	_haltproc

        .globl  _haltproc
        .type   _haltproc,@function
_haltproc:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        movq    __dl_fini@GOTPCREL(%rip),%rax
        movq    (%rax),%rax
        testq   %rax,%rax
        jz .LNoDlFiniCall
        call    *%rax
.LNoDlFiniCall:

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        movq 	operatingsystem_result@GOTPCREL(%rip),%rax
        movzwl  (%rax),%edi
        movl    $231,%eax                 /* exit_group call */
        syscall
        jmp     _haltproc

/* Define a symbol for the first piece of initialized data.  */
	.data
	.globl __data_start
__data_start:
	.long 0
	.weak data_start
        data_start = __data_start

.bss
        .comm __stkptr,8
        .comm __dl_fini,8

        .comm operatingsystem_parameter_envp,8
        .comm operatingsystem_parameter_argc,8
        .comm operatingsystem_parameter_argv,8


/* We need this stuff to make gdb behave itself, otherwise
   gdb will chokes with SIGILL when trying to debug apps.
*/
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

	.section	.note.GNU-stack,"",@progbits
