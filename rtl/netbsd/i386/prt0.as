#
#   $Id: prt0.as,v 1.2 2004/01/04 01:13:23 marco Exp $
#   This file is part of the Free Pascal run time library.
#   Copyright (c) 1999-2000 by Marco van de Voort, Michael Van Canneyt
#                                                  and Peter Vreman
#   members of the Free Pascal development team.
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
# NetBSD standard (static) ELF/i386 startup code for Free Pascal
#

http://cvsweb.netbsd.org/bsdweb.cgi/src/lib/csu/i386/crt0.c?rev=1.33&content-type=text/x-cvsweb-markup

	.file	"prt0.s"
	.version	"01.01"
gcc2_compiled.:
.globl __progname

.section	.rodata
.LC0:
	.ascii "\0"
.data
	.align 4
	.type	 __progname,@object
	.size	 __progname,4
__progname:
	.long .LC0
.globl __ps_strings
	.align 4
	.type	 __ps_strings,@object
	.size	 __ps_strings,4
__ps_strings:
	.long 0
	.align 4
<<<<<<< HEAD
<<<<<<< HEAD
.ifdef CPREFIX
___fpucw:
.else
__fpucw:
.endif
=======
___fpucw:
>>>>>>> graemeg/fixes_2_2
=======
___fpucw:
>>>>>>> origin/fixes_2_2
        .long   0x1332

        .globl  ___fpc_brk_addr         /* heap management */
        .type   ___fpc_brk_addr,@object
        .size   ___fpc_brk_addr,4
___fpc_brk_addr:
        .long   0

#APP
<<<<<<< HEAD
<<<<<<< HEAD

=======
	
>>>>>>> graemeg/fixes_2_2
=======
	
>>>>>>> origin/fixes_2_2
	.text
	.align	4
	.globl	__start
	.globl	_start
_start:
__start:
	pushl	%ebx			# ps_strings
	pushl	%ecx			# obj
	pushl	%edx			# cleanup
	movl	12(%esp),%eax
	leal	20(%esp,%eax,4),%ecx
	leal	16(%esp),%edx
	pushl	%ecx
	pushl	%edx
	pushl	%eax
	call	___start

#NO_APP
.text
	.align 4
.globl ___start
	.type	 ___start,@function
___start:
	pushl %ebp
	movl %esp,%ebp
	movl 16(%ebp),%eax
	movl %eax,environ
<<<<<<< HEAD
<<<<<<< HEAD
	movl %eax,operatingsystem_parameter_envp
	movl 8(%ebp),%eax
	movl %eax,operatingsystem_parameter_argc
	movl 12(%ebp),%eax
	movl %eax,operatingsystem_parameter_argv
=======
=======
>>>>>>> origin/fixes_2_2
	movl %eax,U_SYSTEM_ENVP
	movl 8(%ebp),%eax
	movl %eax,U_SYSTEM_ARGC
	movl 12(%ebp),%eax
	movl %eax,U_SYSTEM_ARGV
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
	movl (%eax),%edx
	movl %edx,__progname
	testl %edx,%edx
	je .L2
<<<<<<< HEAD
<<<<<<< HEAD
        movl __progname,%edx
        // Increase until 0 found
        movl $0,%ebx
.LL1:
        movb (%edx,%ebx),%al
        orb  %al,%al
        je .LL2
        incl %ebx
.LL2:
        decl %ebx
        movb (%edx,%ebx),%al
        cmpb $47,%al
        je   .LL3
        cmpl $0,%ebx
        je   .LL4
.LL3:    // slash found
        incl %ebx
        leal (%edx,%ebx),%eax
        movl %eax,__progname
.LL4:
        cmpl $0,__progname
=======
=======
>>>>>>> origin/fixes_2_2
	pushl $47
	movl __progname,%eax
	pushl %eax
	call _strrchr
	addl $8,%esp
	movl %eax,%eax
	movl %eax,__progname
	cmpl $0,__progname
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
	jne .L3
	movl 12(%ebp),%eax
	movl (%eax),%edx
	movl %edx,__progname
	jmp .L2
	.align 4
.L3:
	incl __progname
.L4:
.L2:
	cmpl $0,28(%ebp)
	je .L5
	movl 28(%ebp),%eax
	movl %eax,__ps_strings
.L5:
#	pushl $_fini
#	call atexit
#	addl $4,%esp
#	call _init
# copied from linux

        finit                           /* initialize fpu */
        fwait
<<<<<<< HEAD
<<<<<<< HEAD
 .ifdef CPREFIX
        fldcw   ___fpucw
 .else
        fldcw   __fpucw
 .endif
        xorl    %ebp,%ebp

.ifdef CPREFIX
	call _main
.else
	call main
.endif
pushl %eax
=======
=======
>>>>>>> origin/fixes_2_2
        fldcw   ___fpucw

        xorl    %ebp,%ebp

	call _main
	pushl %eax
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
	jmp  _haltproc

.p2align 2,0x90
.globl _haltproc
.type _haltproc,@function

_haltproc:
           mov $1,%eax
<<<<<<< HEAD
<<<<<<< HEAD
           movzwl operatingsystem_result,%ebx
=======
           movzwl U_SYSTEM_EXITCODE,%ebx
>>>>>>> graemeg/fixes_2_2
=======
           movzwl U_SYSTEM_EXITCODE,%ebx
>>>>>>> origin/fixes_2_2
           pushl %ebx
           call _actualsyscall
           addl  $4,%esp
           jmp   _haltproc

_actualsyscall:
         int $0x80
         jb .LErrorcode
         xor %ebx,%ebx
         ret
.LErrorcode:
         mov %eax,%ebx
         mov $-1,%eax
         ret
        .p2align 2,0x90

# This section is needed for NetBSD to recognize a NetBSD binary as such.
# otherwise it will be startup in Linux emulation mode.

.section ".note.netbsd.ident","a"
.p2align 2

.long 7
.long 4
# ELF NOTE TYPE NETBSD TAG
.long 1
.ascii "NetBSD\0\0"
.long 199905
<<<<<<< HEAD
<<<<<<< HEAD

        .comm environ,4,4
        .comm operatingsystem_parameter_envp,4,4
        .comm operatingsystem_parameter_argc,4,4
        .comm operatingsystem_parameter_argv,4,4

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
