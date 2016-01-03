|
|   $Id: dllprt0.as,v 1.1.2.4 2001/08/01 13:26:17 pierre Exp $
|   This file is part of the Free Pascal run time library.
|   Copyright (c) 2001 by Pierre Muller
|
|   See the file COPYING.FPC, included in this distribution,
|   for details about the copyright.
|
|   This program is distributed in the hope that it will be useful,
|   but WITHOUT ANY WARRANTY;without even the implied warranty of
|   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
|
|**********************************************************************}
|
| Linux m68k ELF shared library startup code for Free Pascal
|
        .file "dllprt0.as"
	.text
        .globl  _startlib
        .type   _startlib,@function
_startlib:
<<<<<<< HEAD
<<<<<<< HEAD
        .globl  FPC_SHARED_LIB_START
        .type   FPC_SHARED_LIB_START,@function
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
# This is a normal C function with args (argc,argv,envp)
FPC_SHARED_LIB_START:
        link.w   %a6,#0
        move.l   8(%fp),%d0
        move.l   %d0,operatingsystem_parameter_argc
        move.l   12(%fp),%d0
        move.l   %d0,operatingsystem_parameter_argv
        move.l   16(%fp),%d0
        move.l   %d0,operatingsystem_parameter_envp
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
FPC_SHARED_LIB_START:
=======
        .globl  FPC_LIB_START
        .type   FPC_LIB_START,@function
FPC_LIB_START:
>>>>>>> graemeg/fixes_2_2
=======
        .globl  FPC_LIB_START
        .type   FPC_LIB_START,@function
FPC_LIB_START:
>>>>>>> origin/fixes_2_2
|
|       The args and envs are not tested yet
|
        move.l   (%sp)+, %d0
        lea      4(%sp,%d0*4),%a0
        move.l   %a0, U_SYSLINUX_ENVP
        move.l   %sp,U_SYSLINUX_ARGV
        move.l   %d0,U_SYSLINUX_ARGC
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
        jsr      PASCALMAIN
        unlk     %a6
        rts
.size FPC_SHARED_LIB_START,.-FPC_SHARED_LIB_START

        .globl  _haltproc
        .type   _haltproc,@function
<<<<<<< HEAD
_haltproc:
=======
haltproc:
        .globl  FPC_SHARED_LIB_EXIT
        .type   FPC_SHARED_LIB_EXIT,@function
FPC_SHARED_LIB_EXIT:
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
        moveq.l   #1,%d0
        move.w    operatingsystem_result,%d1
        trap      #0
        bra       _haltproc
.size _haltproc,.-_haltproc
=======
=======
>>>>>>> origin/fixes_2_2
        jsr      PASCALMAIN
        rts

        .globl  _haltproc
        .type   _haltproc,@function
haltproc:
        moveq.l   #1,%d0
        move.w    U_SYSLINUX_EXITCODE,%d1
        trap      #0
        bra       _haltproc

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

        .data
	.align  4
	.globl  ___fpc_brk_addr
___fpc_brk_addr:
        .long   0

<<<<<<< HEAD
<<<<<<< HEAD
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
=======
=======
>>>>>>> origin/fixes_2_2
|
| $Log: dllprt0.as,v $
| Revision 1.1.2.4  2001/08/01 13:26:17  pierre
|  * syntax adapted to GNU as
|
| Revision 1.1.2.3  2001/07/13 15:13:47  pierre
|  + add and fix some comments
|
| Revision 1.1.2.2  2001/07/13 15:04:35  pierre
|  * correct assembler error
|
| Revision 1.1.2.1  2001/07/13 15:03:02  pierre
|  + New file converted from i386 version
|
|
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

