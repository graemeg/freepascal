/*
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2009 by Michael Van Canneyt and David Zhang

    Startup code for elf32-mipsel/elf32-mips

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
        .set noat
        .set noreorder
        .section ".text"

        .align 4
        .global _dynamic_start
        .ent    _dynamic_start
        .type   _dynamic_start,@function
_dynamic_start:
        lui     $a2,%hi(__dl_fini)
        b _start
        sw      $v0,%lo(__dl_fini)($a2)

        .end    _dynamic_start
        .size   _dynamic_start, .-_dynamic_start

        .align 4
        .global _start
        .set    nomips16
        .ent    _start
        .type   _start,@function
/*  This is the canonical entry point, usually the first thing in the text
    segment.  The SVR4/Mips ABI (pages 3-31, 3-32) says that when the entry
    point runs, most registers' values are unspecified, except for:

    v0 ($2)     Function pointer of a function to be executed at exit

    sp ($29)    The stack contains the arguments and environment:
             0(%sp)    argc
             4(%sp)    argv[0]

               ...
      (4*argc)(%sp)    NULL
   4*(argc+1))(%sp)    envp[0]
               ...
                       NULL
    ra ($31)    Return address set to zero.
*/
_start:
        /* load fp */
        move    $s8,$sp
        /* set __stkptr value to $s8 */
        lui     $at,%hi(__stkptr)
        sw      $s8,%lo(__stkptr)($at)

        /* align stack */
        li      $at,-8
        and     $sp,$sp,$at

        addiu   $sp,$sp,-32

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
        lui     $a3,%hi(operatingsystem_parameter_envp)
        sw      $a2,%lo(operatingsystem_parameter_envp)($a3)
        lui     $t9,%hi(PASCALMAIN)
        addiu   $t9,$t9,%lo(PASCALMAIN)
        jalr    $t9
        nop
        b       _haltproc
        nop

        .end    _start
        .size   _start, .-_start

        .globl  _haltproc
        .ent    _haltproc
        .type   _haltproc,@function
_haltproc:
        addiu   $sp,$sp,-24
        sw      $a0,16($sp)              /* $a0 contains the exitcode */
        lui     $at,%hi(__dl_fini)
        lw      $t9,%lo(__dl_fini)($at)
        beqz    $t9,.L1
        nop
        jalr    $t9
        nop
.L1:
        lw      $a0,16($sp)
        li      $v0,4001
        syscall
        nop
        b       .L1
        nop

        .end _haltproc
        .size _haltproc, .-_haltproc

        .comm __stkptr,4
        .comm __dl_fini,4

        .comm operatingsystem_parameter_envp,4
        .comm operatingsystem_parameter_argc,4
        .comm operatingsystem_parameter_argv,4

.section .note.GNU-stack,"",@progbits
