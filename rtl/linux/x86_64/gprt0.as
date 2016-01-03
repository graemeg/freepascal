/* This is the canonical entry point, usually the first thing in the text
   segment.  The SVR4/i386 ABI (pages 3-31, 3-32) says that when the entry
   point runs, most registers' values are unspecified, except for:

   %rdx		Contains a function pointer to be registered with `atexit'.
		This is how the dynamic linker arranges to have DT_FINI
		functions called for shared libraries that have been loaded
		before this code runs.

   %rsp		The stack contains the arguments and environment:
		0(%rsp)			argc
		8(%rsp)			argv[0]
		...
		(8*argc)(%rsp)		NULL
		(8*(argc+1))(%rsp)	envp[0]
		...
					NULL
*/

	.text
	.globl _start
	.type _start,@function
_start:
	/* Clear the frame pointer.  The ABI suggests this be done, to mark
	   the outermost frame obviously.  */
	xorq %rbp, %rbp

	/* Extract the arguments as encoded on the stack and set up
	   the arguments for __libc_start_main (int (*main) (int, char **, char **),
		   int argc, char *argv,
		   void (*init) (void), void (*fini) (void),
		   void (*rtld_fini) (void), void *stack_end).
	   The arguments are passed via registers and on the stack:
	main:		%rdi
	argc:		%rsi
	argv:		%rdx
	init:		%rcx
	fini:		%r8
	rtld_fini:	%r9
	stack_end:	stack.	*/

	movq %rdx, %r9		/* Address of the shared library termination
				   function.  */
	popq %rsi		/* Pop the argument count.  */
	movq %rsp, %rdx		/* argv starts just at the current stack top.  */

        movq    operatingsystem_parameter_argc@GOTPCREL(%rip),%rax
        movq    %rsi,(%rax)
        movq    operatingsystem_parameter_argv@GOTPCREL(%rip),%rax
        movq    %rsp,(%rax)   /* argv starts just at the current stack top.  */
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        leaq    8(,%rsi,8),%rax
        addq    %rsp,%rax
        movq    operatingsystem_parameter_envp@GOTPCREL(%rip),%rcx
        movq    %rax,(%rcx)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        leaq     8(,%rsi,8),%rax
        addq     %rsp,%rax
        movq    operatingsystem_parameter_envp@GOTPCREL(%rip),%rsi
        movq    %rax,(%rsi)
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

	/* Align the stack to a 16 byte boundary to follow the ABI.  */
	andq  $~15, %rsp

	pushq %rax		/* Push garbage because we push 8 more bytes.  */

	/* Provide the highest stack address to the user code (for stacks
	   which grow downwards).  */
	pushq %rsp

	/* Pass address of our own entry points to .fini and .init.  */
        movq _init_dummy@GOTPCREL(%rip), %r8
        movq _fini_dummy@GOTPCREL(%rip), %rcx

        movq main_stub@GOTPCREL(%rip), %rdi

	/* Call the user's main function, and exit with its value.
	   But let the libc call main.	  */
	call __libc_start_main@PLT

	hlt			/* Crash if somehow `exit' does return.	 */

/* fake main routine which will be run from libc */
        .globl main_stub
        .type main_stub,@function
main_stub:
        /* save return address */
        popq    %rax

	// stack alignment
        pushq   %rax

        movq    ___fpc_ret_rbp@GOTPCREL(%rip),%rcx
        movq    %rbp,(%rcx)
        movq    ___fpc_ret@GOTPCREL(%rip),%rcx
        movq    %rax,(%rcx)
        pushq   %rax

	/* Initialize gmon */
        movq    _etext@GOTPCREL(%rip),%rsi
        movq    _start@GOTPCREL(%rip),%rdi
        call    monstartup@PLT

        movq    _mcleanup@GOTPCREL(%rip),%rdi
        call    atexit@PLT

        /* Save initial stackpointer */
        movq    __stkptr@GOTPCREL(%rip),%rax
        movq    %rsp,(%rax)

        /* start the program */
        xorq    %rbp,%rbp
        call    PASCALMAIN@PLT
        hlt
        .size   main_stub,.-main_stub

        .globl _haltproc
        .type _haltproc,@function
_haltproc:
        movq    operatingsystem_result@GOTPCREL(%rip),%rax
        movzwl  (%rax),%eax

       /* return to libc */
        movq    ___fpc_ret_rbp@GOTPCREL(%rip),%rcx
        movq    (%rcx),%rbp
        movq    ___fpc_ret@GOTPCREL(%rip),%rcx
        movq    (%rcx),%rdx
        pushq    %rdx
        ret
        .size   _haltproc,.-_haltproc

        .globl _init_dummy
        .type   _init_dummy, @function
_init_dummy:
        ret
        .size   _init_dummy,.-_init_dummy

        .globl  _fini_dummy
        .type   _fini_dummy, @function
_fini_dummy:
        ret
        .size   _fini_dummy,.-_fini_dummy

/* Define a symbol for the first piece of initialized data.  */
	.data
	.globl __data_start
__data_start:
	.long 0
	.weak data_start
	data_start = __data_start

        .globl  ___fpc_brk_addr         /* heap management */
        .type   ___fpc_brk_addr,@object
        .size   ___fpc_brk_addr,8
___fpc_brk_addr:
        .quad   0

___fpc_ret:                             /* return address to libc */
        .quad   0
___fpc_ret_rbp:
        .quad   0

.bss
        .comm __stkptr,8

        .comm operatingsystem_parameter_envp,8
        .comm operatingsystem_parameter_argc,8
        .comm operatingsystem_parameter_argv,8

/* We need this stuff to make gdb behave itself, otherwise
   gdb will chokes with SIGILL when trying to debug apps.
*/
        .section ".note.ABI-tag", "a"
        .align 4
        .long 1f - 0f
        .long 3f - 2f
        .long  1
0:      .asciz "GNU"
1:      .align 4
2:      .long 0
        .long 2,4,0
3:      .align 4

	.section	.note.GNU-stack,"",@progbits
