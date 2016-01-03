/*
 * Startup code for programs linked with GNU libc, PowerPC
 * version.
 *
 * Adapted from the glibc-sources (2.3.5) in the file
 *
 *     sysdeps/powerpc/powerpc32/elf/start.S
 *
 * Original header follows.
 */

/* Startup code for programs linked with GNU libc.
   Copyright (C) 1998,1999,2000,2001,2002,2003 Free Software Foundation, Inc.
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
   Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
   02111-1307 USA.
*/

/* These are the various addresses we require.  */
    .section ".rodata"
    .align    2
start_addresses:
    .long     _SDA_BASE_
    .long     main_stub
    .long     __libc_csu_init
    .long     __libc_csu_fini
    .size   start_adresses, .-start_addresses

    .section ".text"
    .globl  _start
_start:
    /* save stack pointer for later use */
    mr     9, 1
 /* Set up an initial stack frame, and clear the LR.  */
    clrrwi 1, 1, 4
    li     0, 0
    stwu   1, -16(1)
    mtlr   0
    stw    0, 0(1)
 /* Set r13 to point at the 'small data area', and put the address of
    start_addresses in r8...  */
    lis    8,  start_addresses@ha
    lwzu   13, start_addresses@l(8)
 /* and continue in libc-start, in glibc.  */
    b      __libc_start_main

    .globl  main_stub
    .type   main_stub, @function
main_stub:
 /* save link register and setup stack frame */
    mflr    0
    stw     0, 0(1)
    stwu    1, -16(1)

    lis     11, operatingsystem_parameter_argc@ha
    stw      3, operatingsystem_parameter_argc@l(11);

    lis     11, operatingsystem_parameter_argv@ha
    stw      4, operatingsystem_parameter_argv@l(11);

    lis     11, operatingsystem_parameter_envp@ha
    stw      5, operatingsystem_parameter_envp@l(11);

<<<<<<< HEAD
<<<<<<< HEAD
    lis     11,__stkptr@ha
    stw      1,__stkptr@l(11);
=======
    lis 	11,__stkptr@ha
	stw 	1,__stkptr@l(11);
>>>>>>> graemeg/fixes_2_2
=======
    lis 	11,__stkptr@ha
	stw 	1,__stkptr@l(11);
>>>>>>> origin/fixes_2_2

    lis     11, ___fpc_ret@ha
    stw     1, ___fpc_ret@l(11)

    lis     11, __stkptr@ha
    stw     1, __stkptr@l(11)

    bl      PASCALMAIN

    .globl  _haltproc
    .type   _haltproc, @function
_haltproc:
    lis     11, ___fpc_ret@ha
    lwz     1, ___fpc_ret@l(11)
    addi    1, 1, 16
    lwz     0, 0(1)
    mtlr    0
    blr

#    li      0, 1       /* exit call */
#    lis     3, operatingsystem_result@h
#    stw     3, operatingsystem_result@l(3)
#    sc
#    b       _haltproc

 /* Define a symbol for the first piece of initialized data.  */
    .section ".data"
    .globl   __data_start
__data_start:
data_start:

___fpc_ret:                            /* return address to libc */
    .long   0

<<<<<<< HEAD
<<<<<<< HEAD
    .section ".bss"
    .type __stkptr, @object
    .size __stkptr, 8
    .global __stkptr
__stkptr:
    .skip 4

    .type operatingsystem_parameters, @object
    .size operatingsystem_parameters, 12
operatingsystem_parameters:
    .skip 3 * 4
    .global operatingsystem_parameter_argc
    .global operatingsystem_parameter_argv
    .global operatingsystem_parameter_envp
    .set operatingsystem_parameter_argc, operatingsystem_parameters+0
    .set operatingsystem_parameter_argv, operatingsystem_parameters+4
    .set operatingsystem_parameter_envp, operatingsystem_parameters+8

.section .note.GNU-stack,"",%progbits
=======
=======
>>>>>>> origin/fixes_2_2
.text
    .comm __stkptr, 4

    .comm operatingsystem_parameter_envp, 4
    .comm operatingsystem_parameter_argc, 4
    .comm operatingsystem_parameter_argv, 4

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
