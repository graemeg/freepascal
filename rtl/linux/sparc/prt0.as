/* Startup code for elf32-sparc
   Copyright (C) 1997, 1998 Free Software Foundation, Inc.
   This file is part of the GNU C Library.
   Contributed by Richard Henderson <richard@gnu.ai.mit.edu>, 1997.

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
   MA 02111-1301, USA.  */

	.section ".text"

	.align 4
	.global _dynamic_start
	.type _dynamic_start,#function
_dynamic_start:
.ifdef PIC
        call   1f
        sethi  %hi(_GLOBAL_OFFSET_TABLE_+4),%o0
1:      or     %o0,%lo(_GLOBAL_OFFSET_TABLE_+8),%o0
        add    %o0,%o7,%o0
        sethi  %hi(__dl_fini),%o1
        or     %o0,%lo(__dl_fini),%o1
        ld     [%o0+%o1],%o0
        b      _start
        st     %g1,[%o0]
.else
        sethi  %hi(__dl_fini),%o0
        b      _start
        st     %g1,[%o0+%lo(__dl_fini)]
.endif

	.align 4
	.global _start
	.type _start,#function
_start:
  	/* Terminate the stack frame, and reserve space for functions to
     	   drop their arguments.  */
	mov	%g0, %fp
	sub	%sp, 6*4, %sp

.ifdef PIC
        /* Set %l7 to _GLOBAL_OFFSET_TABLE value */
        sethi %hi(_GLOBAL_OFFSET_TABLE_-8),%l7
        or %l7,%lo(_GLOBAL_OFFSET_TABLE_-4),%l7
        call FPC_GETGOT
        nop
.endif
  	/* Extract the arguments and environment as encoded on the stack.  The
     	   argument info starts after one register window (16 words) past the SP.  */
	ld	[%sp+22*4], %o2
	sethi	%hi(operatingsystem_parameter_argc),%o1
	or	%o1,%lo(operatingsystem_parameter_argc),%o1
.ifdef PIC
        ld      [%o1+%l7],%o1
.endif
	st	%o2, [%o1]

	add	%sp, 23*4, %o0
	sethi	%hi(operatingsystem_parameter_argv),%o1
	or	%o1,%lo(operatingsystem_parameter_argv),%o1
.ifdef PIC
        ld      [%o1+%l7],%o1
.endif
	st	%o0, [%o1]

	/* envp=(argc+1)*4+argv */
	inc     %o2
	sll     %o2, 2, %o2
	add	%o2, %o0, %o2
	sethi	%hi(operatingsystem_parameter_envp),%o1
	or	%o1,%lo(operatingsystem_parameter_envp),%o1
.ifdef PIC
        ld      [%o1+%l7],%o1
.endif
	st	%o2, [%o1]

        /* Save initial stackpointer */
	sethi	%hi(__stkptr),%o1
	or	%o1,%lo(__stkptr),%o1
.ifdef PIC
        ld      [%o1+%l7],%o1
.endif
	st	%sp, [%o1]

  	/* Call the user program entry point.  */
  	call	PASCALMAIN
  	nop
	/* Die very horribly if main returns.  */
	unimp

.size _start, .-_start

.globl  _haltproc
.type   _haltproc,@function
_haltproc:
<<<<<<< HEAD
<<<<<<< HEAD
        save    %o6,-96,%o6
.ifdef PIC
        call   1f
        sethi  %hi(_GLOBAL_OFFSET_TABLE_+4),%l7
1:      or     %l7,%lo(_GLOBAL_OFFSET_TABLE_+8),%l7
        add    %l7,%o7,%l7
.endif
        mov     %i0,%l0  
        sethi   %hi(__dl_fini),%o0
        or      %o0,%lo(__dl_fini),%o0
.ifdef PIC
        ld      [%o0+%l7],%o0
.endif
        ld      [%o0],%o0
        subcc   %o0,%g0,%g0
        beq     2f
        nop
        call    %o0
        nop
2:
        mov     %i0,%o0                 /* i0 contains the exitcode */
        mov     188, %g1                /* "exit_group" system call */
        ta      0x10                    /* do the system call */
        unimp                           /* Die very horribly if exit returns.  */

.size _haltproc,.-_haltproc
=======
=======
>>>>>>> origin/fixes_2_2
	mov	188, %g1			/* "exit_group" system call */
	ta	0x10			/* dot the system call */
	nop				/* delay slot */
	/* Die very horribly if exit returns.  */
	unimp
>>>>>>> graemeg/fixes_2_2


        .comm __stkptr,4
        .comm __dl_fini,4

        .comm operatingsystem_parameter_envp,4
        .comm operatingsystem_parameter_argc,4
        .comm operatingsystem_parameter_argv,4

.section .note.GNU-stack,"",@progbits
