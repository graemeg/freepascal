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
        .globl  FPC_SHARED_LIB_START
        .type   FPC_SHARED_LIB_START,@function
# This is a normal C function with args (argc,argv,envp)
FPC_SHARED_LIB_START:
        link.w   %a6,#0
        move.l   %sp,__stkptr
        move.l   8(%fp),%d0
        move.l   %d0,operatingsystem_parameter_argc
        move.l   12(%fp),%d0
        move.l   %d0,operatingsystem_parameter_argv
        move.l   16(%fp),%d0
        move.l   %d0,operatingsystem_parameter_envp
        jsr      PASCALMAIN
        unlk     %a6
        rts
.size FPC_SHARED_LIB_START,.-FPC_SHARED_LIB_START

        .globl  _haltproc
        .type   _haltproc,@function
_haltproc:
        moveq.l   #1,%d0
        move.w    operatingsystem_result,%d1
        trap      #0
        bra       _haltproc
.size _haltproc,.-_haltproc

        .data
	.align  4
	.globl  ___fpc_brk_addr
___fpc_brk_addr:
        .long   0

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

