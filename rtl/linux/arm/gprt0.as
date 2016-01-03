<<<<<<< HEAD
<<<<<<< HEAD
/* Startup code for ARM & ELF
   Copyright (C) 1995, 1996, 1997, 1998, 2001, 2002 Free Software Foundation, Inc.
   This file is part of the GNU C Library.

   The GNU C Library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   The GNU C Library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with the GNU C Library; if not, write to the Free
   Software Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
   MA 02110-1301, USA. */

/* This is the canonical entry point, usually the first thing in the text
   segment.

        Note that the code in the .init section has already been run.
        This includes _init and _libc_init


        At this entry point, most registers' values are unspecified, except:

   a1           Contains a function pointer to be registered with `atexit'.
                This is how the dynamic linker arranges to have DT_FINI
                functions called for shared libraries that have been loaded
                before this code runs.

   sp           The stack contains the arguments and environment:
                0(sp)                   argc
                4(sp)                   argv[0]
                ...
                (4*argc)(sp)            NULL
                (4*(argc+1))(sp)        envp[0]
                ...
                                        NULL
*/

        .text
        .globl _start
        .type _start,#function
_start:
        /* Clear the frame pointer since this is the outermost frame.  */
        mov fp, #0
        ldmia   sp!, {a2}

        /* Pop argc off the stack and save a pointer to argv */
        ldr ip,=operatingsystem_parameter_argc
        ldr a3,=operatingsystem_parameter_argv
        str a2,[ip]

        /* calc envp */
        add a4,a2,#1
        add a4,sp,a4,LSL #2
        ldr ip,=operatingsystem_parameter_envp

        str sp,[a3]
        str a4,[ip]

        /* Save initial stackpointer */
        ldr ip,=__stkptr
        str sp,[ip]

        /* Initialize gmon */
        mov     r2,#1
        ldr     r1,=_etext
        ldr     r0,=_start
        bl      __monstartup
        ldr     r0,=_mcleanup
        bl      atexit

        /* argc already loaded to a2*/
        ldr ip, =operatingsystem_parameter_argc
        ldr a2,[ip]

        /* Fetch address of fini */
        ldr ip, =_fini

        /* load argv */
        mov a3, sp

        /* Push stack limit */
        str a3, [sp, #-4]!

        /* Push rtld_fini */
        str a1, [sp, #-4]!

        /* Set up the other arguments in registers */
        ldr a1, =PASCALMAIN
        ldr a4, =_init

        /* Push fini */
        str ip, [sp, #-4]!

        /* __libc_start_main (main, argc, argv, init, fini, rtld_fini, stack_end) */

        /* Let the libc call main and exit with its return code.  */
        bl __libc_start_main

        /* should never get here....*/
        bl abort

        .globl  _haltproc
    .type   _haltproc,#function
_haltproc:
        ldr r0,=operatingsystem_result
        ldrb r0,[r0]
        swi 0x900001
        b _haltproc

        .globl  _haltproc_eabi
        .type   _haltproc_eabi,#function
_haltproc_eabi:
        bl exit  /* libc exit */

        ldr r0,=operatingsystem_result
        ldrb r0,[r0]
        mov r7,#248
        swi 0x0
        b _haltproc_eabi

        /* Define a symbol for the first piece of initialized data.  */
        .data
        .globl __data_start
__data_start:
        .long 0
        .weak data_start
        data_start = __data_start
=======
=======
>>>>>>> origin/fixes_2_2
/*
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

	.text
	.globl _start
	.type _start,#function
_start:
	/* Clear the frame pointer since this is the outermost frame.  */
	mov fp, #0
	ldmia   sp!, {a2}

	/* Pop argc off the stack and save a pointer to argv */
	ldr ip,=operatingsystem_parameter_argc
	ldr a3,=operatingsystem_parameter_argv
	str a2,[ip]

	/* calc envp */
	add a2,a2,#1
	add a2,sp,a2,LSL #2
	ldr ip,=operatingsystem_parameter_envp

	str sp,[a3]
   	str a2,[ip]

        ldr     r1,=_etext                 /* Initialize gmon */
        ldr     r0,=_start
        bl      monstartup
        ldr     r0,=_mcleanup
        bl      atexit

        /* Save initial stackpointer */
	ldr ip,=__stkptr
	str sp,[ip]

	/* Let the libc call main and exit with its return code.  */
	bl PASCALMAIN

	.globl  _haltproc
    .type   _haltproc,#function
_haltproc:
        /* r0 contains exitcode */
	swi 0x900001
	b _haltproc

	/* Define a symbol for the first piece of initialized data.  */
	.data
	.globl __data_start
__data_start:
	.long 0
	.weak data_start
	data_start = __data_start
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

.bss
        .comm __stkptr,4

        .comm operatingsystem_parameter_envp,4
        .comm operatingsystem_parameter_argc,4
        .comm operatingsystem_parameter_argv,4

<<<<<<< HEAD
<<<<<<< HEAD
        .section ".comment"
        .byte 0
        .ascii "generated by FPC http://www.freepascal.org\0"
=======
	.section ".comment"
	.byte 0
	.ascii "generated by FPC http://www.freepascal.org\0"
>>>>>>> graemeg/fixes_2_2
=======
	.section ".comment"
	.byte 0
	.ascii "generated by FPC http://www.freepascal.org\0"
>>>>>>> origin/fixes_2_2

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
