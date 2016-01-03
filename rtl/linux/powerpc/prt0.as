<<<<<<< HEAD
/*
 * This file is part of the Free Pascal run time library.
 * Copyright (c) 2011 by Thomas Schatzl,
 * member of the Free Pascal development team.
 *
 * Startup code for normal programs, PowerPC version.
 *
 * See the file COPYING.FPC, included in this distribution,
 * for details about the copyright.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 */

/*
 * Main program entry point for dynamic executables.
 */
    .section ".text"
    .globl  _dynamic_start
_dynamic_start:
    lis     11,__dl_fini@ha
    stw      7,__dl_fini@l(11)

    b _start

/*
 * Main program entry point for static executables.
 */
=======
/* Startup code for programs linked with GNU libc.
   Copyright (C) 1998, 1999, 2000, 2001 Free Software Foundation, Inc.
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
   02111-1307 USA.  */

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
    .section ".text"
    .globl  _start
_start:
    mr      26,1
    /* Set up an initial stack frame, and clear the LR.  */
    clrrwi   1,1,4
    li       0,0
    stwu     1,-16(1)
    mtlr     0
    stw      0,0(1)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    lwz      3,0(26)       /* get argc */
    lis     11,operatingsystem_parameter_argc@ha
    stw      3,operatingsystem_parameter_argc@l(11);

    addi     4,26,4        /* get argv */
    lis     11,operatingsystem_parameter_argv@ha
    stw      4,operatingsystem_parameter_argv@l(11);

    addi    27,3,1        /* calculate argc + 1 into r27 */
    slwi    27,27,2       /* calculate (argc + 1) * sizeof(char *) into r27 */
    add      5,4,27       /* get address of env[0] */
    lis     11,operatingsystem_parameter_envp@ha
    stw      5,operatingsystem_parameter_envp@l(11);

    lis     11,__stkptr@ha
    stw      1,__stkptr@l(11);

    bl         PASCALMAIN

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    /* we should not reach here. Crash horribly */
    trap
=======
    b          _haltproc
>>>>>>> graemeg/cpstrnew
=======
    b          _haltproc
>>>>>>> graemeg/cpstrnew
=======
    b          _haltproc
>>>>>>> graemeg/cpstrnew
=======
    b          _haltproc
>>>>>>> origin/cpstrnew

    .globl  _haltproc
    .type   _haltproc,@function
_haltproc:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

    lis     11,__dl_fini@ha
    lwz     11,__dl_fini@l(11)

    cmpwi   11, 0
    beq     .LNoDlFiniCall

    mtctr   11
    bctrl

.LNoDlFiniCall:

    lis     11,operatingsystem_result@ha
    lwz      3,operatingsystem_result@l(11)
    li       0,234        /* exit group call */
    sc

    lis     11,operatingsystem_result@ha
    lwz      3,operatingsystem_result@l(11)
    li       0,1          /* exit call */
    sc
    /* we should not reach here. Crash horribly */
    trap


=======
    li       0,1          /* exit call */
    sc
    b          _haltproc

>>>>>>> graemeg/cpstrnew
=======
    li       0,1          /* exit call */
    sc
    b          _haltproc

>>>>>>> graemeg/cpstrnew
=======
    li       0,1          /* exit call */
    sc
    b          _haltproc

>>>>>>> graemeg/cpstrnew
=======
    li       0,1          /* exit call */
    sc
    b          _haltproc

>>>>>>> origin/cpstrnew
/* Define a symbol for the first piece of initialized data.  */
    .section ".data"
    .globl __data_start
__data_start:
data_start:

    .section ".bss"
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

    .type __dl_fini, @object
    .size __dl_fini, 4
    .global __dl_fini
__dl_fini:
    .skip 4

    .type __stkptr, @object
    .size __stkptr, 4
    .global __stkptr
__stkptr:
    .skip 4

=======

    .type __stkptr, @object
    .size __stkptr, 4
    .global __stkptr
__stkptr:
    .skip 4

>>>>>>> graemeg/cpstrnew
=======

    .type __stkptr, @object
    .size __stkptr, 4
    .global __stkptr
__stkptr:
    .skip 4

>>>>>>> graemeg/cpstrnew
=======

    .type __stkptr, @object
    .size __stkptr, 4
    .global __stkptr
__stkptr:
    .skip 4

>>>>>>> graemeg/cpstrnew
=======

    .type __stkptr, @object
    .size __stkptr, 4
    .global __stkptr
__stkptr:
    .skip 4

>>>>>>> origin/cpstrnew
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
