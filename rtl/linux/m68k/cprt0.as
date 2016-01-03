<<<<<<< HEAD
<<<<<<< HEAD
#
#   This file is part of the Free Pascal run time library.
#   Copyright (c) 2001 by Free Pascal Core Team
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
# Linux m68k ELF startup code for linking with C lib for Free Pascal
#
        .file   "cprt0.as"
        .text
=======
=======
>>>>>>> origin/fixes_2_2
|
|   $Id: cprt0.as,v 1.1.2.2 2001/08/01 13:26:17 pierre Exp $
|   This file is part of the Free Pascal run time library.
|   Copyright (c) 2001 by Free Pascal Core Team
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
| Linux m68k ELF startup code for linking with C lib for Free Pascal
|
        .file   "cprt0.as"
	.text
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        .globl  _start
        .type   _start,@function
_start:
        .globl  __entry
        .type   __entry,@function
__entry:
<<<<<<< HEAD
<<<<<<< HEAD
######### This order of parameters is for ucLibc.
#       move.l   8(%sp),%d0
#       move.l   %d0,operatingsystem_parameter_envp
#       move.l   %d0,__environ
#       move.l   4(%sp),%d0
#       move.l   %d0,operatingsystem_parameter_argv
#       move.l   (%sp),%d0
#       move.l   %d0,operatingsystem_parameter_argc
#########

        sub.l    %fp,%fp
        move.l   (%sp)+,%d0
        move.l   %d0,operatingsystem_parameter_argc
        move.l   %sp,operatingsystem_parameter_argv
        lea.l    8(%sp,%d0.l*4),%a0
        move.l   %a0,operatingsystem_parameter_envp
        move.l   %sp,%a0       /* argv */

        pea      (%sp)         /* highest available stack address */
        pea      (%a1)         /* termination function provided by kernel */
        pea      __libc_csu_fini
        pea      __libc_csu_init
        pea      (%a0)         /* argv */
        move.l   %d0,-(%sp)    /* argc */
        pea      PASCALMAIN
        jsr      __libc_start_main
        illegal
        .size _start,.-_start

#       Used by System_exit procedure
        .globl  _haltproc
        .type   _haltproc,@function
_haltproc:
#       Call C exit function
        move.l   operatingsystem_result,%d1
=======
=======
>>>>>>> origin/fixes_2_2
        move.l   8(%sp),%d0
        move.l   %d0,U_SYSLINUX_ENVP
        move.l   %d0,__environ
        move.l   4(%sp),%d0
        move.l   %d0,U_SYSLINUX_ARGV
        move.l   (%sp),%d0
        move.l   %d0,U_SYSLINUX_ARGC
|       The arguments should be in correct order for
|       calling __libc_init
|       This code is untested for now PM
        jsr     __libc_init
|       insert _fini in atexit chain
        move.l   _fini,-(%sp)
        jsr      atexit
        addq.l   #4,%sp
|       call _init function
        jsr      _init

        jsr      PASCALMAIN

|       Used by System_exit procedure
        .globl  _haltproc
_haltproc:
|       Call C exit function
        move.w   U_SYSLINUX_EXITCODE,%d1
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        move.l   %d1,-(%sp)
        jsr      exit
        moveq.l  #1,%d0
        move.l   (%sp)+,%d1
        trap     #0
        bra      _haltproc
<<<<<<< HEAD
<<<<<<< HEAD
.size _haltproc,.-_haltproc

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


|       Is this still needed ??
|        .data
|        .align	4
|        .globl	___fpc_brk_addr
|___fpc_brk_addr:
|       .long	0


|
| $Log: cprt0.as,v $
| Revision 1.1.2.2  2001/08/01 13:26:17  pierre
|  * syntax adapted to GNU as
|
| Revision 1.1.2.1  2001/07/13 15:29:32  pierre
|  first version of cprt0.as
|
|

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
