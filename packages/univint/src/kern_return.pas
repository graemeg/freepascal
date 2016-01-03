{
 * Copyright (c) 2000 Apple Computer, Inc. All rights reserved.
 *
 * @APPLE_OSREFERENCE_LICENSE_HEADER_START@
 * 
 * This file contains Original Code and/or Modifications of Original Code
 * as defined in and that are subject to the Apple Public Source License
 * Version 2.0 (the 'License'). You may not use this file except in
 * compliance with the License. The rights granted to you under the License
 * may not be used to create, or enable the creation or redistribution of,
 * unlawful or unlicensed copies of an Apple operating system, or to
 * circumvent, violate, or enable the circumvention or violation of, any
 * terms of an Apple operating system software license agreement.
 * 
 * Please obtain a copy of the License at
 * http://www.opensource.apple.com/apsl/ and read it before using this file.
 * 
 * The Original Code and all software distributed under the License are
 * distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER
 * EXPRESS OR IMPLIED, AND APPLE HEREBY DISCLAIMS ALL SUCH WARRANTIES,
 * INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT.
 * Please see the License for the specific language governing rights and
 * limitations under the License.
 * 
 * @APPLE_OSREFERENCE_LICENSE_HEADER_END@
 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, September 2010 }
{
    Modified for use with Free Pascal
    Version 308
    Please report any bugs to <gpc@microbizz.nl>
}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
{$mode macpas}
{$packenum 1}
{$macro on}
{$inline on}
{$calling mwpascal}

unit kern_return;
interface
{$setc UNIVERSAL_INTERFACES_VERSION := $0400}
{$setc GAP_INTERFACES_VERSION := $0308}

{$ifc not defined USE_CFSTR_CONSTANT_MACROS}
    {$setc USE_CFSTR_CONSTANT_MACROS := TRUE}
{$endc}

{$ifc defined CPUPOWERPC and defined CPUI386}
	{$error Conflicting initial definitions for CPUPOWERPC and CPUI386}
{$endc}
{$ifc defined FPC_BIG_ENDIAN and defined FPC_LITTLE_ENDIAN}
	{$error Conflicting initial definitions for FPC_BIG_ENDIAN and FPC_LITTLE_ENDIAN}
{$endc}

{$ifc not defined __ppc__ and defined CPUPOWERPC32}
	{$setc __ppc__ := 1}
{$elsec}
	{$setc __ppc__ := 0}
{$endc}
{$ifc not defined __ppc64__ and defined CPUPOWERPC64}
	{$setc __ppc64__ := 1}
{$elsec}
	{$setc __ppc64__ := 0}
{$endc}
{$ifc not defined __i386__ and defined CPUI386}
	{$setc __i386__ := 1}
{$elsec}
	{$setc __i386__ := 0}
{$endc}
{$ifc not defined __x86_64__ and defined CPUX86_64}
	{$setc __x86_64__ := 1}
{$elsec}
	{$setc __x86_64__ := 0}
{$endc}
{$ifc not defined __arm__ and defined CPUARM}
	{$setc __arm__ := 1}
{$elsec}
	{$setc __arm__ := 0}
{$endc}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc not defined __arm64__ and defined CPUAARCH64}
  {$setc __arm64__ := 1}
{$elsec}
  {$setc __arm64__ := 0}
{$endc}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{$ifc defined cpu64}
  {$setc __LP64__ := 1}
{$elsec}
  {$setc __LP64__ := 0}
{$endc}


{$ifc defined __ppc__ and __ppc__ and defined __i386__ and __i386__}
	{$error Conflicting definitions for __ppc__ and __i386__}
{$endc}

{$ifc defined __ppc__ and __ppc__}
	{$setc TARGET_CPU_PPC := TRUE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/cpstrnew
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/cpstrnew
{$elifc defined __i386__ and __i386__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := TRUE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifc defined(iphonesim)}
 	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := TRUE}
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
{$ifc defined(iphonesim)}
 	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := TRUE}
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/cpstrnew
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := TRUE}
{$elifc defined __arm64__ and __arm64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_CPU_ARM64 := TRUE}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := TRUE}
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ nor __arm64__ is defined.}
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> origin/cpstrnew
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
  {$setc TARGET_CPU_64 := FALSE}
{$endc}

{$ifc defined FPC_BIG_ENDIAN}
	{$setc TARGET_RT_BIG_ENDIAN := TRUE}
	{$setc TARGET_RT_LITTLE_ENDIAN := FALSE}
{$elifc defined FPC_LITTLE_ENDIAN}
	{$setc TARGET_RT_BIG_ENDIAN := FALSE}
	{$setc TARGET_RT_LITTLE_ENDIAN := TRUE}
{$elsec}
	{$error Neither FPC_BIG_ENDIAN nor FPC_LITTLE_ENDIAN are defined.}
{$endc}
{$setc ACCESSOR_CALLS_ARE_FUNCTIONS := TRUE}
{$setc CALL_NOT_IN_CARBON := FALSE}
{$setc OLDROUTINENAMES := FALSE}
{$setc OPAQUE_TOOLBOX_STRUCTS := TRUE}
{$setc OPAQUE_UPP_TYPES := TRUE}
{$setc OTCARBONAPPLICATION := TRUE}
{$setc OTKERNEL := FALSE}
{$setc PM_USE_SESSION_APIS := TRUE}
{$setc TARGET_API_MAC_CARBON := TRUE}
{$setc TARGET_API_MAC_OS8 := FALSE}
{$setc TARGET_API_MAC_OSX := TRUE}
{$setc TARGET_CARBON := TRUE}
{$setc TARGET_CPU_68K := FALSE}
{$setc TARGET_CPU_MIPS := FALSE}
{$setc TARGET_CPU_SPARC := FALSE}
{$setc TARGET_OS_UNIX := FALSE}
{$setc TARGET_OS_WIN32 := FALSE}
{$setc TARGET_RT_MAC_68881 := FALSE}
{$setc TARGET_RT_MAC_CFM := FALSE}
{$setc TARGET_RT_MAC_MACHO := TRUE}
{$setc TYPED_FUNCTION_POINTERS := TRUE}
{$setc TYPE_BOOL := FALSE}
{$setc TYPE_EXTENDED := FALSE}
{$setc TYPE_LONGLONG := TRUE}
uses MacTypes;
{$endc} {not MACOSALLINCLUDE}


{
 * @OSF_COPYRIGHT@
 }
{ 
 * Mach Operating System
 * Copyright (c) 1991,1990,1989,1988,1987 Carnegie Mellon University
 * All Rights Reserved.
 * 
 * Permission to use, copy, modify and distribute this software and its
 * documentation is hereby granted, provided that both the copyright
 * notice and this permission notice appear in all copies of the
 * software, derivative works or modified versions, and any portions
 * thereof, and that both notices appear in supporting documentation.
 * 
 * CARNEGIE MELLON ALLOWS FREE USE OF THIS SOFTWARE IN ITS "AS IS"
 * CONDITION.  CARNEGIE MELLON DISCLAIMS ANY LIABILITY OF ANY KIND FOR
 * ANY DAMAGES WHATSOEVER RESULTING FROM THE USE OF THIS SOFTWARE.
 * 
 * Carnegie Mellon requests users of this software to return to
 * 
 *  Software Distribution Coordinator  or  Software.Distribution@CS.CMU.EDU
 *  School of Computer Science
 *  Carnegie Mellon University
 *  Pittsburgh PA 15213-3890
 * 
 * any improvements or extensions that they make and grant Carnegie Mellon
 * the rights to redistribute these changes.
 }
{
 *	File:	h/kern_return.h
 *	Author:	Avadis Tevanian, Jr.
 *	Date:	1985
 *
 *	Kernel return codes.
 *
 }


const
	KERN_SUCCESS = 0;

const
	KERN_INVALID_ADDRESS = 1;
		{ Specified address is not currently valid.
		 }

const
	KERN_PROTECTION_FAILURE = 2;
		{ Specified memory is valid, but does not permit the
		 * required forms of access.
		 }

const
	KERN_NO_SPACE = 3;
		{ The address range specified is already in use, or
		 * no address range of the size specified could be
		 * found.
		 }

const
	KERN_INVALID_ARGUMENT = 4;
		{ The function requested was not applicable to this
		 * type of argument, or an argument is invalid
		 }

const
	KERN_FAILURE = 5;
		{ The function could not be performed.  A catch-all.
		 }

const
	KERN_RESOURCE_SHORTAGE = 6;
		{ A system resource could not be allocated to fulfill
		 * this request.  This failure may not be permanent.
		 }

const
	KERN_NOT_RECEIVER = 7;
		{ The task in question does not hold receive rights
		 * for the port argument.
		 }

const
	KERN_NO_ACCESS = 8;
		{ Bogus access restriction.
		 }

const
	KERN_MEMORY_FAILURE = 9;
		{ During a page fault, the target address refers to a
		 * memory object that has been destroyed.  This
		 * failure is permanent.
		 }

const
	KERN_MEMORY_ERROR = 10;
		{ During a page fault, the memory object indicated
		 * that the data could not be returned.  This failure
		 * may be temporary; future attempts to access this
		 * same data may succeed, as defined by the memory
		 * object.
		 }

const
	KERN_ALREADY_IN_SET = 11;
		{ The receive right is already a member of the portset.
		 }

const
	KERN_NOT_IN_SET = 12;
		{ The receive right is not a member of a port set.
		 }

const
	KERN_NAME_EXISTS = 13;
		{ The name already denotes a right in the task.
		 }

const
	KERN_ABORTED = 14;
		{ The operation was aborted.  Ipc code will
		 * catch this and reflect it as a message error.
		 }

const
	KERN_INVALID_NAME = 15;
		{ The name doesn't denote a right in the task.
		 }

const
	KERN_INVALID_TASK = 16;
		{ Target task isn't an active task.
		 }

const
	KERN_INVALID_RIGHT = 17;
		{ The name denotes a right, but not an appropriate right.
		 }

const
	KERN_INVALID_VALUE = 18;
		{ A blatant range error.
		 }

const
	KERN_UREFS_OVERFLOW = 19;
		{ Operation would overflow limit on user-references.
		 }

const
	KERN_INVALID_CAPABILITY = 20;
		{ The supplied (port) capability is improper.
		 }

const
	KERN_RIGHT_EXISTS = 21;
		{ The task already has send or receive rights
		 * for the port under another name.
		 }

const
	KERN_INVALID_HOST = 22;
		{ Target host isn't actually a host.
		 }

const
	KERN_MEMORY_PRESENT = 23;
		{ An attempt was made to supply "precious" data
		 * for memory that is already present in a
		 * memory object.
		 }

const
	KERN_MEMORY_DATA_MOVED = 24;
		{ A page was requested of a memory manager via
		 * memory_object_data_request for an object using
		 * a MEMORY_OBJECT_COPY_CALL strategy, with the
		 * VM_PROT_WANTS_COPY flag being used to specify
		 * that the page desired is for a copy of the
		 * object, and the memory manager has detected
		 * the page was pushed into a copy of the object
		 * while the kernel was walking the shadow chain
		 * from the copy to the object. This error code
		 * is delivered via memory_object_data_error
		 * and is handled by the kernel (it forces the
		 * kernel to restart the fault). It will not be
		 * seen by users.
		 }

const
	KERN_MEMORY_RESTART_COPY = 25;
		{ A strategic copy was attempted of an object
		 * upon which a quicker copy is now possible.
		 * The caller should retry the copy using
		 * vm_object_copy_quickly. This error code
		 * is seen only by the kernel.
		 }

const
	KERN_INVALID_PROCESSOR_SET = 26;
		{ An argument applied to assert processor set privilege
		 * was not a processor set control port.
		 }

const
	KERN_POLICY_LIMIT = 27;
		{ The specified scheduling attributes exceed the thread's
		 * limits.
		 }

const
	KERN_INVALID_POLICY = 28;
		{ The specified scheduling policy is not currently
		 * enabled for the processor set.
		 }

const
	KERN_INVALID_OBJECT = 29;
		{ The external memory manager failed to initialize the
		 * memory object.
		 }

const
	KERN_ALREADY_WAITING = 30;
		{ A thread is attempting to wait for an event for which 
		 * there is already a waiting thread.
		 }

const
	KERN_DEFAULT_SET = 31;
		{ An attempt was made to destroy the default processor
		 * set.
		 }

const
	KERN_EXCEPTION_PROTECTED = 32;
		{ An attempt was made to fetch an exception port that is
		 * protected, or to abort a thread while processing a
		 * protected exception.
		 }

const
	KERN_INVALID_LEDGER = 33;
		{ A ledger was required but not supplied.
		 }

const
	KERN_INVALID_MEMORY_CONTROL = 34;
		{ The port was not a memory cache control port.
		 }

const
	KERN_INVALID_SECURITY = 35;
		{ An argument supplied to assert security privilege 	
		 * was not a host security port.
		 }
		
const
	KERN_NOT_DEPRESSED = 36;
		{ thread_depress_abort was called on a thread which
		 * was not currently depressed.
		 }
		
const
	KERN_TERMINATED = 37;
		{ Object has been terminated and is no longer available
		 }

const
	KERN_LOCK_SET_DESTROYED = 38;
		{ Lock set has been destroyed and is no longer available.
		 }

const
	KERN_LOCK_UNSTABLE = 39;
		{ The thread holding the lock terminated before releasing
		 * the lock
		 }

const
	KERN_LOCK_OWNED = 40;
		{ The lock is already owned by another thread
		 }

const
	KERN_LOCK_OWNED_SELF = 41;
		{ The lock is already owned by the calling thread
		 }

const
	KERN_SEMAPHORE_DESTROYED = 42;
		{ Semaphore has been destroyed and is no longer available.
		 }

const
	KERN_RPC_SERVER_TERMINATED = 43;
		{ Return from RPC indicating the target server was 
		 * terminated before it successfully replied 
		 }

const
	KERN_RPC_TERMINATE_ORPHAN = 44;
		{ Terminate an orphaned activation.
		 }

const
	KERN_RPC_CONTINUE_ORPHAN = 45;
		{ Allow an orphaned activation to continue executing.
		 }

const
	KERN_NOT_SUPPORTED = 46;
		{ Empty thread activation (No thread linked to it)
		 }

const
	KERN_NODE_DOWN = 47;
		{ Remote node down or inaccessible.
		 }

const
	KERN_NOT_WAITING = 48;
		{ A signalled thread was not actually waiting. }

const
	KERN_OPERATION_TIMED_OUT = 49;
		{ Some thread-oriented operation (semaphore_wait) timed out
		 }

const
	KERN_CODESIGN_ERROR = 50;
		{ During a page fault, indicates that the page was rejected
		 * as a result of a signature check.
		 }

const
	KERN_RETURN_MAX = $100;
		{ Maximum return value allowable
		 }

{ cpu-specific, but the same for ppc and x86 at least, both on 32 and 64 bit -> presumably also on ARM }
type
	kern_return_t = SInt32;
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
