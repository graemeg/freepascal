/*
   Start-up code for Free Pascal Compiler when linking with C library
   with profiling support.
*/

	.text
	.align 2

	.globl _start
	.type _start,#function
_start:
	/* Initialise FP to zero */
	mov	x29,#0

	/* Get argc, argv, envp */
	ldr	x1,[x0]
	add	x2,x0,#8
	add	x11,x1,#1
	add	x11,x2,x11,lsl #3

	/* Save argc, argv, envp, and initial stack pointer */
	adrp	x10,:got:operatingsystem_parameter_argc
	ldr	x10,[x10,#:got_lo12:operatingsystem_parameter_argc]
	str	x1,[x10]
	adrp	x10,:got:operatingsystem_parameter_argv
	ldr	x10,[x10,#:got_lo12:operatingsystem_parameter_argv]
	str	x2,[x10]
	adrp	x10,:got:operatingsystem_parameter_envp
	ldr	x10,[x10,#:got_lo12:operatingsystem_parameter_envp]
	str	x3,[x10]

	/* save environ */
	adrp	x10,environ
	ldr	x10,[x10,:lo12:environ]
	cbnz	x10,.LBB0_2
	ldr	x10,=environ
	str	x3,[x10]
.LBB0_2:
	/* save __progname */
        ldr     w8,=operatingsystem_parameter_argc
        cmp     w8,#0
        cset    w8,le
        tbnz    w8,#0,.LBB0_9
// %bb.1:
	adrp	x8,operatingsystem_parameter_argv
        ldr     x8,[x8,:lo12:operatingsystem_parameter_argv]
        cbz     x8,.LBB0_9
// %bb.2:
	ldr	x2,[x2]
	adrp	x9,__progname
	adrp	x10,__progname
	add	x10,x10,:lo12:__progname
	str	x2,[x10]
	ldr	x8,[x9,:lo12:__progname]
	adrp	x9,s
	add	x9,x9,:lo12:s
	str	x8,[x9]
.LBB0_3:
	adrp	x8,s
	ldr	x8,[x8,:lo12:s]
	ldrb	w9,[x8]
	cbz	w9,.LBB0_8
// %bb.4:
	adrp	x8,s
	ldr	x8,[x8, :lo12:s]
	ldrb	w9,[x8]
	cmp	w9,#47
	b.ne	.LBB0_6
// %bb.5:
	adrp	x8,s
	ldr	x8,[x8,:lo12:s]
	add	x8,x8,#1
	adrp	x9,__progname
	add	x9,x9,:lo12:__progname
	str	x8,[x9]
.LBB0_6:
// %bb.7:
	adrp	x8,s
	adrp	x9,s
	add	x9,x9,:lo12:s
	ldr	x8,[x8,:lo12:s]
	add	x8,x8,#1
	str	x8,[x9]
	b	.LBB0_3
.LBB0_8:
.LBB0_9:
	adrp	x10,:got:__stkptr
	ldr	x10,[x10,#:got_lo12:__stkptr]
	mov	x6,sp
	str	x6,[x10]

	bl	main

	/* This should never happen */
	b	abort

	.globl	_init_dummy
	.type	_init_dummy,#function
_init_dummy:
	ret

	.globl	_fini_dummy
	.type	_fini_dummy,#function
_fini_dummy:
	ret

	.globl	main_stub
	.type	main_stub,#function
main_stub:
	stp	x29,x30,[sp,#-16]!

	/* Save initial stackpointer */
	mov	x0,sp
	adrp	x1,:got:__stkptr
	ldr	x1,[x1,#:got_lo12:__stkptr]
	str	x0,[x1]

	/* Initialize gmon */
	adrp	x0,:got:_start
	ldr	x0,[x0,#:got_lo12:_start]
	adrp	x1,:got:_etext
	ldr	x1,[x1,#:got_lo12:_etext]
	bl	__monstartup
	adrp	x0,:got:_mcleanup
	ldr	x0,[x0,#:got_lo12:_mcleanup]
	bl	atexit

	/* Start the program */
	bl	 PASCALMAIN
	b	 abort

	.globl	_haltproc
	.type	_haltproc,#function
_haltproc:
	/* Return to libc */
	adrp	x1,:got:__stkptr
	ldr	x1,[x1,#:got_lo12:__stkptr]
	ldr	x1,[x1]
	mov	sp,x1
	ldp	x29,x30,[sp],#16
	ret

	/* Define a symbol for the first piece of initialized data. */
	.data
	.align 3
	.globl __data_start
__data_start:
	.long 0
	.weak data_start
	data_start = __data_start

	.bss
	.align 3

	.comm __stkptr,8

	.comm operatingsystem_parameter_envp,8
	.comm operatingsystem_parameter_argc,8
	.comm operatingsystem_parameter_argv,8
	.comm environ,8,8

s:
        .xword  0
        .size   s, 8

	.section .note.GNU-stack,"",%progbits
