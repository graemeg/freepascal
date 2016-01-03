<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
.include "mips/prt0.as"
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
/*
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2009 by Michael Van Canneyt and David Zhang

    Startup code for elf32-mipsel

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
        .set noat

	.section ".text"
	.align 4
	.global _start
	.type _start,@function
/*  This is the canonical entry point, usually the first thing in the text
    segment.  The SVR4/Mips ABI (pages 3-31, 3-32) says that when the entry
    point runs, most registers' values are unspecified, except for:

    v0 ($2)	Contains a function pointer to be registered with `atexit'.
 		This is how the dynamic linker arranges to have DT_FINI
 		functions called for shared libraries that have been loaded
 		before this code runs.

    sp ($29)	The stack contains the arguments and environment:
 		0(%esp)			argc
 		4(%esp)			argv[0]
 		...
 		(4*argc)(%esp)		NULL
 		(4*(argc+1))(%esp)	envp[0]
 		...
 					NULL
    ra ($31)	The return address register is set to zero so that programs
 		that search backword through stack frames recognize the last
 		stack frame.
_start:
        /* load fp */
        move    $s8,$sp

        /* align stack */
        li      $at,-8
        and     $sp,$sp,$at

        addiu   $sp,$sp,-32

        lui     $s7,0x3d
        addiu   $s7,$s7,2304
        li      $at,-8
        and     $s7,$s7,$at
        addiu   $s7,$s7,-32

        /* store argc */
        lw      $a0,0($s8)
        lui     $a1,%hi(operatingsystem_parameter_argc)
        sw      $a0,%lo(operatingsystem_parameter_argc)($a1)

        /* store argv */
        addiu   $a1,$s8,4
        lui     $a2,%hi(operatingsystem_parameter_argv)
        sw      $a1,%lo(operatingsystem_parameter_argv)($a2)

        /* store envp */
        addiu   $a2,$a0,1
        sll     $a2,$a2,0x2
        addu    $a2,$a2,$a1
        lui     $a3,$hi(operatingsystem_parameter_envp)
        jal     PASCALMAIN
        sw      $a2,%lo(operatingsystem_parameter_envp)($a3)
        nop

.globl  _haltproc
.type   _haltproc,@function
_haltproc:
        li      $v0,4001
        lui     $a0,0x0
        lw      $a0,0($a0)
        syscall
        b       _haltproc
        nop

	.size _start, .-_start

        .comm __stkptr,4

        .comm operatingsystem_parameter_envp,4
        .comm operatingsystem_parameter_argc,4
        .comm operatingsystem_parameter_argv,4
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
