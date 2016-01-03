{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2009 by Pierre Muller,
    member of the Free Pascal development team.

    Program startup
    Adapted from source code on opensolaris 2.11

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

(*
/*
 * CDDL HEADER START
 *
 * The contents of this file are subject to the terms of the
 * Common Development and Distribution License (the "License").
 * You may not use this file except in compliance with the License.
 *
 * You can obtain a copy of the license at usr/src/OPENSOLARIS.LICENSE
 * or http://www.opensolaris.org/os/licensing.
 * See the License for the specific language governing permissions
 * and limitations under the License.
 *
 * When distributing Covered Code, include this CDDL HEADER in each
 * file and include the License file at usr/src/OPENSOLARIS.LICENSE.
 * If applicable, add the following below this CDDL HEADER, with the
 * fields enclosed by brackets "[]" replaced with your own identifying
 * information: Portions Copyright [yyyy] [name of copyright owner]
 *
 * CDDL HEADER END
 */

/*
 * Copyright 2007 Sun Microsystems, Inc.  All rights reserved.
 * Use is subject to license terms.
 */
*)

(*
/*
 * This crt1.o module is provided as the bare minimum required to build
 * a 64-bit executable with gcc.  It is installed in /usr/lib/amd64
 * where it will be picked up by gcc, along with crti.o and crtn.o
 */

	.ident	"%Z%%M%	%I%	%E% SMI"

	.file	"crt1.s"

	.globl	_start

/* global entities defined elsewhere but used here */
	.globl	main
	.globl	__fpstart
	.globl	exit
	.globl	_exit
	.weak	_DYNAMIC
*)
type
   TCdeclProcedure = procedure; cdecl;
function atexit(proc:TCdeclProcedure):longint;cdecl;external 'c' name 'atexit'{ @plt };
procedure C_exit;cdecl;external 'c' name 'exit';
procedure _exit;cdecl;external 'c' name '_exit';
//procedure _fini;cdecl;external 'c' name '_fini';
//procedure __fpstart;cdecl;external 'c' name '__fpstart'{ @plt };
//procedure __fsr;cdecl;external 'c' name '__fsr';
//procedure _init;cdecl;external 'c' name '_init';
procedure PascalMain;cdecl;external name 'PASCALMAIN';


{vars are not correctly transformed :(
var
  _DYNAMIC : longint; cvar; external;
  __Argv : pointer; cvar; external;
  environ : pointer; cvar; external;
  __get_exit_frame_monitor_ptr : pointer; cvar; external;
  __do_exit_code_ptr : pointer; cvar; external;
}

(*
	.section	.data

	.weak	environ
	.set	environ,_environ
	.globl	_environ
	.type	_environ,@object
	.size	_environ,8
	.align	8
_environ:
	.8byte	0x0

	.globl	__environ_lock
	.type	__environ_lock,@object
	.size	__environ_lock,24
	.align	8
__environ_lock:
	.zero	24

	.globl	___Argv
	.type	___Argv,@object
	.size	___Argv,8
	.align	8
___Argv:
	.8byte	0x0


	.globl	__longdouble_used
	.section	.data
	.align	8
	.type	__longdouble_used,@object
	.size	__longdouble_used,4
__longdouble_used:
	.4byte	0

*)
var
  _environ : pointer; cvar; public;
  __environ_lock : Array[0..24-1] of byte; cvar; public;
  ___Argv : pointer;cvar; public;
  __longdouble_used : longint; cvar; public;
var
  _DYNAMIC : pointer;cvar;weakexternal;

//procedure _DYNAMIC;cdecl;external 'c' name '_DYNAMIC'; { should be weak }

(*
/*
 *   The SVR4/i386 ABI (pages 3-29) says that when the entry
 *   point runs registers' %rbp, %rsp, %rdx values are specified
 *   the following:
 *
 *	%rbp The content of this register is unspecified at
 *		process initialization time, but the user code should mark
 *		the deepest stack frame by setting the frame pointer to zero.
 *		No other frame's %ebp should have a zero value.
 *
 *	%rsp Performing its usual job, the stack pointer holds the address
 *	of the bottom of the stack, which is guaranteed to be
 *	quadword aligned.
 *
 *		The stack contains the arguments and environment:
 *        ...
 *        envp[0]		(16+(8*argc))(%rsp)
 *        NULL			(8+(8*argc))(%rsp)
 *        ...
 *        argv[0]		8(%rsp)
 *        argc			0(%rsp)
 *
 *	%rdx In a conforming program, this register contains a function
 *		pointer that the application should register with atexit(BA_OS).
 *		This function is used for shared object termination code
 *		[see Dynamic Linking in Chapter 5 of the System V ABI].
 *
 */
*)
procedure _start;assembler;nostackframe;public name '_start';
asm
(*
/*
 * Allocate a NULL return address and a NULL previous %rbp as if
 * there was a genuine call to _start.
 */
*)
   pushq  $0x0
   pushq  $0x0
   movq   %rsp,%rbp
(*
/*
 * The stack now is
 *
 *        envp[0]		(32+(8*argc))(%rsp)      - (A)
 *        NULL			(24+(8*argc))(%rsp)
 *        ...
 *        argv[0]		24(%rbp)		 - (B)
 *        argc			16(%rbp)
 *	  0			8(%rbp)
 *	  0			0(%rbp)
 */
*)
{$ifdef FPC_PIC}
   movq   _DYNAMIC@GOTPCREL(%rip),%rax
{$else FPC_PIC}
   movq   $_DYNAMIC,%rax
{$endif FPC_PIC}
   testq  %rax,%rax
   je     .Label1
   movq   %rdx,%rdi { register rt_do_exit }
   call   atexit{$ifdef FPC_PIC}@PLT{$endif}
.Label1:
(* What should we do about this?
   movq   $_fini,%rdi
   call   atexit *)
(*
/*
 * Calculate the location of the envp array by adding the size of
 * the argv array to the start of the argv array.
 */
*)
   movq   0x10(%rbp),%rax
{$ifdef FPC_PIC}
   movq   argc@GOTPCREL(%rip),%rcx
{$else FPC_PIC}
   movq   $argc,%rcx
{$endif FPC_PIC}
   movl   %eax,(%rcx)
{$ifdef FPC_PIC}
   movq   _environ@GOTPCREL(%rip),%rcx
   movq   (%rcx),%rcx
{$else FPC_PIC}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   movq   _environ(%rip),%rcx
=======
   movq   _environ{$ifdef FPC_HAS_RIP_RELATIVE}(%rip){$endif},%rcx
>>>>>>> graemeg/cpstrnew
=======
   movq   _environ{$ifdef FPC_HAS_RIP_RELATIVE}(%rip){$endif},%rcx
>>>>>>> graemeg/cpstrnew
=======
   movq   _environ{$ifdef FPC_HAS_RIP_RELATIVE}(%rip){$endif},%rcx
>>>>>>> graemeg/cpstrnew
=======
   movq   _environ{$ifdef FPC_HAS_RIP_RELATIVE}(%rip){$endif},%rcx
>>>>>>> origin/cpstrnew
{$endif FPC_PIC}
   testq  %rcx,%rcx
   jne    .Label3
   lea    0x20(%rbp,%rax,8),%rcx
.Label3:
{$ifdef FPC_PIC}
   movq   _environ@GOTPCREL(%rip),%rbx
   movq   %rcx,(%rbx)
{$else FPC_PIC}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   movq   %rcx,_environ(%rip)
=======
   movq   %rcx,_environ{$ifdef FPC_HAS_RIP_RELATIVE}(%rip){$endif}
>>>>>>> graemeg/cpstrnew
=======
   movq   %rcx,_environ{$ifdef FPC_HAS_RIP_RELATIVE}(%rip){$endif}
>>>>>>> graemeg/cpstrnew
=======
   movq   %rcx,_environ{$ifdef FPC_HAS_RIP_RELATIVE}(%rip){$endif}
>>>>>>> graemeg/cpstrnew
=======
   movq   %rcx,_environ{$ifdef FPC_HAS_RIP_RELATIVE}(%rip){$endif}
>>>>>>> origin/cpstrnew
{$endif FPC_PIC}
   // Specific to Free Pascal
{$ifdef FPC_PIC}
   movq   envp@GOTPCREL(%rip),%rbx
   movq   %rcx,(%rbx)
{$else FPC_PIC}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   movq   %rcx,envp(%rip)
=======
   movq   %rcx,envp{$ifdef FPC_HAS_RIP_RELATIVE}(%rip){$endif}
>>>>>>> graemeg/cpstrnew
=======
   movq   %rcx,envp{$ifdef FPC_HAS_RIP_RELATIVE}(%rip){$endif}
>>>>>>> graemeg/cpstrnew
=======
   movq   %rcx,envp{$ifdef FPC_HAS_RIP_RELATIVE}(%rip){$endif}
>>>>>>> graemeg/cpstrnew
=======
   movq   %rcx,envp{$ifdef FPC_HAS_RIP_RELATIVE}(%rip){$endif}
>>>>>>> origin/cpstrnew
{$endif FPC_PIC}

(*
/*
 * Force stack alignment - below here there must have been an even
 * number of un-popped pushq instructions whenever a call is reached
 */
*)
   andq   $-16,%rsp
   pushq  %rdx
   leaq   24(%rbp),%rdx   { argv (B) }
{$ifdef FPC_PIC}
   movq   ___Argv@GOTPCREL(%rip),%rbx
   movq   %rdx,(%rbx)
{$else FPC_PIC}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   movq   %rdx,___Argv(%rip)
=======
   movq   %rdx,___Argv{$ifdef FPC_HAS_RIP_RELATIVE}(%rip){$endif}
>>>>>>> graemeg/cpstrnew
=======
   movq   %rdx,___Argv{$ifdef FPC_HAS_RIP_RELATIVE}(%rip){$endif}
>>>>>>> graemeg/cpstrnew
=======
   movq   %rdx,___Argv{$ifdef FPC_HAS_RIP_RELATIVE}(%rip){$endif}
>>>>>>> graemeg/cpstrnew
=======
   movq   %rdx,___Argv{$ifdef FPC_HAS_RIP_RELATIVE}(%rip){$endif}
>>>>>>> origin/cpstrnew
{$endif FPC_PIC}
{$ifdef FPC_PIC}
   movq   argv@GOTPCREL(%rip),%rbx
{$else FPC_PIC}
   movq   $argv,%rbx
{$endif FPC_PIC}
   movq    %rdx,(%rbx)
   pushq   %rcx
   pushq   %rdx
   pushq   %rax
   (* Should this be done, and where?
   call   __fpstart
   call   _init *)
   popq   %rdi
   popq   %rsi
   popq   %rdx
   popq   %rcx
   call   PASCALMAIN{$ifdef FPC_PIC}@PLT{$endif}      { main(argc,argv,envp) }
   pushq  %rax
   pushq  %rax
   movq   %rax,%rdi	  { and call exit }
   call   C_exit{$ifdef FPC_PIC}@PLT{$endif}
   popq   %rdi
   popq   %rdi
   call   _exit{$ifdef FPC_PIC}@PLT{$endif}	  { if user redefined exit, call _exit }
   hlt
end;

(*
/*
 * The following is here in case any object module compiled with cc -p
 *	was linked into this module.
 */
	.globl	_mcount
	.section	.text
	.align	8
	.type	_mcount,@function
_mcount:
	ret
	.size	_mcount, .-_mcount
*)
