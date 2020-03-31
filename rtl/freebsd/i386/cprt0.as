#
#   $Id: cprt0.as,v 1.3 2000/11/21 19:03:23 marco Exp $
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

# FreeBSD  ELF startup code for Free Pascal for dynamical linking to libc.
#
# To avoid needing a "COMPAT" system, patch the constant following the 
# "FreeBSD" field in the abitag to the relevant ABI number 
# Some versions of the "file" command support
# visualizing this constant. If not use elfdump, or patch 
# automatically using the ../i386/identpatch.sh script
#
# some typical values: 
#
# freebsd 5.4 504000
# freebsd 6.3 (prerelease) : 603100
# freebsd 7.0 700055
# freebsd 8.0 800500  (-stable)
# FreeBSD 9.0 900044
# FreeBSD 11.0  1100122
# FreeBSD 11.1  1101001
# FreeBSD 12.1  1201000

        .file   "cprt0.as"
        .ident  "FreePascal 2.6.x/2.7.x series dynlinked to libc"
.section        .note.ABI-tag,"a",@progbits
        .p2align 2
        .type   abitag, @object
        .size   abitag, 24
abitag:
        .long   8
        .long   4
        .long   1
        .string "FreeBSD"
        .long   900044
        .section	.rodata.str1.1,"aMS",@progbits,1
.LC0:
        .string ""
	.data
        .p2align 2
	.globl __progname
        .type    __progname,@object
        .size    __progname,4
__progname:
        .long .LC0
        .text
        .p2align  2,,3
	.type   __fpucw,@object
        .size   __fpucw,4
        .global __fpucw
___fpucw:
        .long   0x1332
        .globl  ___fpc_brk_addr         /* heap management */
        .type   ___fpc_brk_addr,@object
        .size   ___fpc_brk_addr,4
___fpc_brk_addr:
        .long   0

	.text
        .p2align 4,,15
.globl _start
        .type    _start,@function
_start:
        pushl %ebp
        movl %esp,%ebp
	subl $40,%esp
	call get_rtld_cleanup
	movl %eax,%edx

        pushl %edi
        pushl %esi
        pushl %ebx
#APP
        movl %edx,%edx
#NO_APP
        leal 8(%ebp),%edi
        movl %edi,operatingsystem_parameter_argv
        mov -4(%edi),%eax
        movl %eax,operatingsystem_parameter_argc
        movl 4(%ebp),%ebx
        leal 12(%ebp,%ebx,4),%esi
        movl %esi,operatingsystem_parameter_envp
        movl %esi,environ
        testl %ebx,%ebx
        jle .L2
        movl 8(%ebp),%eax
        testl %eax,%eax
        je .L2
        movl %eax,__progname
        cmpb $0,(%eax)
        je .L2
        .p2align 2,0x90
.L6:
        cmpb $47,(%eax)
        jne .L5
        leal 1(%eax),%ecx
        movl %ecx,__progname
.L5:
        incl %eax
        cmpb $0,(%eax)
        jne .L6
.L2:
        movl $_DYNAMIC,%eax
        testl %eax,%eax
        je .LTLS
        pushl %edx
        call atexit
        addl $4,%esp
.L9:
        pushl $_fini
        call atexit
        call _init
#       pushl %esi
#       pushl %edi
#       pushl %ebx
#       call main
#       pushl %eax
#       call exit


        finit                           /* initialize fpu */
        fwait
        fldcw   ___fpucw

        xorl    %ebp,%ebp

        call main
        pushl %eax
        jmp   _haltproc
.LTLS:  
	call _init_tls
	jmp .L9
        .p2align 2,0x90
       
 
.globl _haltproc
.type _haltproc,@function

_haltproc:
           mov $1,%eax  
           movzwl operatingsystem_result,%ebx
           pushl %ebx
           call .Lactualsyscall
           addl  $4,%esp
           jmp   _haltproc

.Lactualsyscall:
         int $0x80
         jb .LErrorcode
         xor %ebx,%ebx
         ret
.LErrorcode:
         mov %eax,%ebx
         mov $-1,%eax
         ret
        .p2align 2,0x90
.Lfe1:
        .size    _start,.Lfe1-_start
        .comm   environ,4,4
        .p2align 4,,15
        .type   get_rtld_cleanup, @function
get_rtld_cleanup:
        pushl   %ebp
        movl    %esp, %ebp
        subl    $4, %esp
#APP
        movl %edx,-4(%ebp)
#NO_APP
        movl    -4(%ebp), %eax
        leave
        ret

        .weak   _DYNAMIC
        .ident  "GCC: (GNU) 3.4.2 - FPC: 2.0.2"

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

//.section .threadvar,"aw",@nobits
        .comm   ___fpc_threadvar_offset,4
