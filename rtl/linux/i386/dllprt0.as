#
#   This file is part of the Free Pascal run time library.
#   Copyright (c) 2001 by Peter Vreman
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
# Linux ELF shared library startup code for Free Pascal
#

        .file   "dllprt0.as"
        .text
        .globl  _startlib
        .type   _startlib,@function
_startlib:
        .globl  FPC_SHARED_LIB_START
        .type   FPC_SHARED_LIB_START,@function
FPC_SHARED_LIB_START:
        pushl   %ebp
        movl    %esp,%ebp
        subl    $8, %esp         /* align back to 16 bytes if it was before the call */

        movl    8(%ebp),%eax
        movl    12(%ebp),%ecx
        movl    16(%ebp),%edx

        movl    %edx,operatingsystem_parameter_envp    /* Move the environment pointer */
        movl    %eax,operatingsystem_parameter_argc    /* Move the argument counter    */
        movl    %ecx,operatingsystem_parameter_argv    /* Move the argument pointer    */

        movb    $1,operatingsystem_islibrary

        /* Save initial stackpointer */
        movl    %esp,__stkptr

        call    PASCALMAIN

        leave
        ret

        .globl  _haltproc
        .type   _haltproc,@function
_haltproc:
_haltproc2:             # GAS <= 2.15 bug: generates larger jump if a label is exported
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        subl    $12, %esp               /* align back to 16 bytes if it was before the call */
=======
        .globl  FPC_SHARED_LIB_EXIT
        .type   FPC_SHARED_LIB_EXIT,@function
FPC_SHARED_LIB_EXIT:
>>>>>>> graemeg/cpstrnew
=======
        .globl  FPC_SHARED_LIB_EXIT
        .type   FPC_SHARED_LIB_EXIT,@function
FPC_SHARED_LIB_EXIT:
>>>>>>> graemeg/cpstrnew
=======
        .globl  FPC_SHARED_LIB_EXIT
        .type   FPC_SHARED_LIB_EXIT,@function
FPC_SHARED_LIB_EXIT:
>>>>>>> graemeg/cpstrnew
=======
        .globl  FPC_SHARED_LIB_EXIT
        .type   FPC_SHARED_LIB_EXIT,@function
FPC_SHARED_LIB_EXIT:
>>>>>>> origin/cpstrnew
	call	lib_exit
        xorl    %eax,%eax
        incl    %eax                    /* eax=1, exit call */
        movzwl  operatingsystem_result,%ebx
        int     $0x80
        jmp     _haltproc2

.bss
        .type   __stkptr,@object
        .size   __stkptr,4
        .global __stkptr
__stkptr:
        .skip   4

        .type operatingsystem_parameters,@object
        .size operatingsystem_parameters,12
operatingsystem_parameters:
        .skip 3*4

        .global operatingsystem_parameter_envp
        .global operatingsystem_parameter_argc
        .global operatingsystem_parameter_argv
        .set operatingsystem_parameter_envp,operatingsystem_parameters+0
        .set operatingsystem_parameter_argc,operatingsystem_parameters+4
        .set operatingsystem_parameter_argv,operatingsystem_parameters+8

.section .note.GNU-stack,"",%progbits
