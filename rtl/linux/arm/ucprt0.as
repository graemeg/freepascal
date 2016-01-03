/* Startup code for ARM & ELF
   Copyright (C) 1995, 1996, 1997, 1998, 2001, 2002, 2005
   Free Software Foundation, Inc.

   The GNU C Library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   In addition to the permissions in the GNU Lesser General Public
   License, the Free Software Foundation gives you unlimited
   permission to link the compiled version of this file with other
   programs, and to distribute those programs without any restriction
   coming from the use of this file. (The GNU Lesser General Public
   License restrictions do apply in other respects; for example, they
   cover modification of the file, and distribution when not linked
   into another program.)

   Note that people who make modified versions of this file are not
   obligated to grant this special exception for their modified
   versions; it is their choice whether to do so. The GNU Lesser
   General Public License gives permission to release a modified
   version without this exception; this exception also makes it
   possible to release a modified version which carries forward this
   exception.

   The GNU C Library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with the GNU C Library; if not, write to the Free
   Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
   02111-1307 USA.  */

/* This is the canonical entry point, usually the first thing in the text
   segment.

	Note that the code in the .init section has already been run.
	This includes _init and _libc_init


	At this entry point, most registers' values are unspecified, except:

   a1		Contains a function pointer to be registered with `atexit'.
		This is how the dynamic linker arranges to have DT_FINI
		functions called for shared libraries that have been loaded
		before this code runs.

   sp		The stack contains the arguments and environment:
		0(sp)			argc
		4(sp)			argv[0]
		...
		(4*argc)(sp)		NULL
		(4*(argc+1))(sp)	envp[0]
		...
					NULL
*/
/*
   For uClinux it looks like this:

        argc            argument counter (integer)
        argv            char *argv[]
        envp            char *envp[]
        argv[0]         program name (pointer)
        argv[1...N]     program args (pointers)
        argv[argc-1]    end of args (integer)
	NULL
        env[0...N]      environment variables (pointers)
        NULL

ARM register quick reference:

    Name    Number       ARM Procedure Calling Standard Role

    a1      r0           argument 1 / integer result / scratch register / argc
    a2      r1           argument 2 / scratch register / argv
    a3      r2           argument 3 / scratch register / envp
    a4      r3           argument 4 / scratch register
    v1      r4           register variable
    v2      r5           register variable
    v3      r6           register variable
    v4      r7           register variable
    v5      r8           register variable
    sb/v6   r9           static base / register variable
    sl/v7   r10          stack limit / stack chunk handle / reg. variable
    fp      r11          frame pointer
    ip      r12          scratch register / new-sb in inter-link-unit calls
    sp      r13          lower end of current stack frame
    lr      r14          link address / scratch register
    pc      r15          program counter
*/

.text
	.globl	_start
	.type	_start,%function
        .type   _init,%function
        .type   _fini,%function
        .weak   _init
        .weak   _fini

_start:
	/* Clear the frame pointer and link register since this is the outermost frame.  */
	mov fp, #0
	mov lr, #0

	/* Pop argc off the stack and save a pointer to argv */
        ldr a2, [sp], #4	
	mov a3, sp

        /* calc envp */
        add a4, a3, a2, lsl #2

        ldr ip,=operatingsystem_parameter_argc
        str a2, [ip]
        ldr ip,=operatingsystem_parameter_argv
        str a3, [ip]
        ldr ip,=operatingsystem_parameter_envp
        str a4, [ip]
        ldr ip,=__stkptr
        str sp, [ip]

<<<<<<< HEAD
<<<<<<< HEAD
	/* keep stack aligned as required by eabi */
        sub sp,sp,#4

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
	/* Push stack limit */
	str a3, [sp, #-4]!

	/* Push rtld_fini */
	str a1, [sp, #-4]!

	/* Fetch address of fini */
	ldr ip, =_fini
	/* Push fini */
	str ip, [sp, #-4]!

	/* Set up the other arguments in registers */
	ldr a1, =PASCALMAIN
	ldr a4, =_init

	/* __uClibc_main (main, argc, argv, init, fini, rtld_fini, stack_end) */

	/* Let the libc call main and exit with its return code.  */
	b __uClibc_main

	/* should never get here....*/
	bl abort

        .globl  _haltproc
    .type   _haltproc,#function
_haltproc:
        swi 0x900001
        b _haltproc

/* Define a symbol for the first piece of initialized data.  */
	.data
	.globl __data_start
__data_start:
	.long 0
	.weak data_start
	data_start = __data_start

.bss
        .comm __stkptr,4

        .comm operatingsystem_parameter_envp,4
        .comm operatingsystem_parameter_argc,4
        .comm operatingsystem_parameter_argv,4

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
        .long 2,0,0
3:      .align 4
<<<<<<< HEAD
<<<<<<< HEAD

.section .note.GNU-stack,"",%progbits
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
