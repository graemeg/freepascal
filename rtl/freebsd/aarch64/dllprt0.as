/*
   Start-up code for Free Pascal Compiler in a shared library,
   not linking with C library.

*/

	.text
	.align 2

	.globl	_startlib
	.type	_startlib,#function
_startlib:
	.globl	FPC_SHARED_LIB_START
	.type	FPC_SHARED_LIB_START,#function
FPC_SHARED_LIB_START:
	stp	x29,x30,[sp,#-16]!

	/* Get argc, argv, envp */
	ldr	x1,[x0]
	add	x2,x0,#8
	add	x3,x1,#1
	add	x3,x2,x3,lsl #3

	/* Save argc, argv and envp */
	adrp	x9,:got:operatingsystem_parameter_argc
	ldr	x9,[x9,#:got_lo12:operatingsystem_parameter_argc]
	str	x1,[x9]
	adrp	x9,:got:operatingsystem_parameter_argv
	ldr	x9,[x9,#:got_lo12:operatingsystem_parameter_argv]
	str	x2,[x9]
	adrp	x9,:got:operatingsystem_parameter_envp
	ldr	x9,[x9,#:got_lo12:operatingsystem_parameter_envp]
	str	x3,[x9]

	/* save environ */
	adrp	x10,environ
	ldr	x10,[x10,:lo12:environ]
	cbnz	x10,.LBB0_2
	adrp	x10,environ
	add	x10,x9,:lo12:environ
	str	x3,[x10]
.LBB0_2:
	/* save __progname */
	adrp	x8,:got:operatingsystem_parameter_argc
	ldr	x8,[x8,#:got_lo12:operatingsystem_parameter_argc]
        cmp     x8,#0
        cset    x8,le
        tbnz    x8,#0,.LBB0_9
// %bb.1:
	adrp	x8,operatingsystem_parameter_argv
        ldr     x8,[x8,:got_lo12:operatingsystem_parameter_argv]
        cbz     x8,.LBB0_9
// %bb.2:
	ldr	x2,[x2]
	adrp	x9,:got:__progname
	adrp	x10,:got:__progname
	add	x10,x10,:lo12:__progname
	str	x2,[x10]
	ldr	x8,[x9,:got_lo12:__progname]
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
	adrp	x9,:got:__progname
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
	/* Save initial stackpointer */
	adrp	x9,:got:__stkptr
	ldr	x9,[x9,#:got_lo12:__stkptr]
	mov	x10,sp
	str	x10,[x9]

	/* Call main */
	bl	PASCALMAIN

	/* Return */
	ldp	x29,x30,[sp],#16
	ret

	.globl	_haltproc
	.type	_haltproc,#function
_haltproc:
	adrp	x0,:got:operatingsystem_result
	ldr	x0,[x0,#:got_lo12:operatingsystem_result]
	ldr	w0,[x0]
	mov	w8,#1 // SYS_exit
	svc	#0
	b	_haltproc

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
