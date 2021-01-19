/* Startup code compliant to the ELF Mips ABI.
   Copyright (C) 1995, 1997, 2000, 2001, 2002, 2003, 2004
	Free Software Foundation, Inc.
   This file is part of the GNU C Library.

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
   Software Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
   02110-1301 USA.  */

/* This is the canonical entry point, usually the first thing in the text
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
*/


/* We need to call:
   __libc_start_main (int (*main) (int, char **, char **), int argc,
		      char **argv, void (*init) (void), void (*fini) (void),
		      void (*rtld_fini) (void), void *stack_end)
*/
 .text
 .globl main
 .globl __start
 .type __start,@function
__start:
.globl _start
 .type _start,@function
_start:
  .ent _start


 .set noreorder
 move $0, $31
 bal 10f
 nop
 10: 
 .cpload $31
 move $31, $0
 .set reorder
	/* Setup GP correctly if we're non-PIC.  */
 la $28,_gp

 lui $4, %hi(main_stub)		/* main */
 addiu $4,$4,%lo(main_stub)

 lw $5, 0($29)		/* argc */
 addiu $6, $29, 4	/* argv */
        /* store argc */
        lw      $t0,0($29)
        lui     $t1,%hi(operatingsystem_parameter_argc)
        sw      $t0,%lo(operatingsystem_parameter_argc)($t1)

        /* store argv */
        addiu   $t1,$29,4
        lui     $t2,%hi(operatingsystem_parameter_argv)
        sw      $t1,%lo(operatingsystem_parameter_argv)($t2)

        /* store envp */
        addiu   $t2,$t0,1
        sll     $t2,$t2,0x2
        addu    $t2,$t2,$t1
        lui     $t3,%hi(operatingsystem_parameter_envp)
        sw      $t2,%lo(operatingsystem_parameter_envp)($t3)
 
	/* Allocate space on the stack for seven arguments (o32 only)
	   and make sure the stack is aligned to double words (8 bytes) 
	   on o32 and quad words (16 bytes) on n32 and n64.  */
 and $29, -2 * 4
 subu $29, 32

 lw $7,%got(__libc_csu_init)($gp) /* init */
 lw $8,%got(__libc_csu_fini)($gp) /* fini */

 sw $8, 16($29)				/* fini */
 sw $2, 20($29)				/* rtld_fini */
 sw $29, 24($29)			/* stack_end */

 lw $t9,%got(__libc_start_main)($gp)
 jalr $t9
 .end _start
 .size _start, . - _start

        .globl  main_stub
        .type   main_stub,@function
main_stub:
        /* load fp */
        move    $s8,$sp
        /* set __stkptr value to $s8 */
        lui     $v0,%hi(__stkptr)
        sw      $s8,%lo(__stkptr)($v0)
        lui     $v0,%hi(__fpc_ret_sp)
        sw      $sp,%lo(__fpc_ret_sp)($v0)
        lui     $v0,%hi(__fpc_ret_ra)
        sw      $ra,%lo(__fpc_ret_ra)($v0)
        lui     $v0,%hi(main)
        addiu   $t9,$v0,%lo(main)
        jr      $t9
        nop
        .size   main_stub,.-main_stub


	.globl  _haltproc
	.ent	_haltproc
	.type   _haltproc,@function
_haltproc:
        lui     $v0,%hi(__fpc_ret_sp)
        lw      $sp,%lo(__fpc_ret_sp)($v0)
        lui     $v0,%hi(__fpc_ret_ra)
        lw      $ra,%lo(__fpc_ret_ra)($v0)
	/* $a0 contains exitcode, we need to move it to $v0,
	   as it would be at return of C main unction.  */
        move	$v0,$a0
        jr      $ra
	nop
hlt:
        b hlt
        .end _haltproc
        .size _haltproc,.-_haltproc

/* Define a symbol for the first piece of initialized data.  */
 .data
 .globl __data_start
__data_start:
 .long 0
 .weak data_start
 data_start = __data_start	

        .comm __stkptr,4
        .comm __dl_fini,4
        .comm __fpc_ret_sp,4
        .comm __fpc_ret_ra,4

        .comm operatingsystem_parameter_envp,4
        .comm operatingsystem_parameter_argc,4
        .comm operatingsystem_parameter_argv,4

