{
 * Copyright (c) 1999 Apple Computer, Inc. All rights reserved.
 *
 * @APPLE_LICENSE_HEADER_START@
 * 
 * Copyright (c) 1999-2003 Apple Computer, Inc.  All Rights Reserved.
 * 
 * This file contains Original Code and/or Modifications of Original Code
 * as defined in and that are subject to the Apple Public Source License
 * Version 2.0 (the 'License'). You may not use this file except in
 * compliance with the License. Please obtain a copy of the License at
 * http://www.opensource.apple.com/apsl/ and read it before using this
 * file.
 * 
 * The Original Code and all software distributed under the License are
 * distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER
 * EXPRESS OR IMPLIED, AND APPLE HEREBY DISCLAIMS ALL SUCH WARRANTIES,
 * INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT.
 * Please see the License for the specific language governing rights and
 * limitations under the License.
 * 
 * @APPLE_LICENSE_HEADER_END@
 }

{
	Pascal translation by Adriaan van Os <gpc@microbizz.nl>, April 2008

 *	objc-class.h
 *	Copyright 1988-1996, NeXT Software, Inc.

 *	objc.h
 *	Copyright 1988-1996, NeXT Software, Inc.

 *	objc-auto.h
 *	Copyright 2004 Apple Computer, Inc.

 *  objc_exception.h
		Support for Objective-C language Exceptions	
		Created by Blaine Garst on Fri Nov 01 2002.
 *  Copyright (c) 2002-3 Apple Computer, Inc. All rights reserved.

 *  objc_sync.h
 *  Copyright (c) 2002 Apple Computer, Inc. All rights reserved.

 *	objc-runtime.h
 *	Copyright 1988-1996, NeXT Software, Inc.
}

{
    Modified for use with Free Pascal
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Version 308
    Please report any bugs to <gpc@microbizz.nl>
}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
    Version 210
    Please report any bugs to <gpc@microbizz.nl>
}

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
    Version 308
    Please report any bugs to <gpc@microbizz.nl>
}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4
{$mode macpas}
{$packenum 1}
{$macro on}
{$inline on}
{$calling mwpascal}

unit ObjCRuntime;
interface
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$setc UNIVERSAL_INTERFACES_VERSION := $0400}
{$setc GAP_INTERFACES_VERSION := $0308}
=======
{$setc UNIVERSAL_INTERFACES_VERSION := $0342}
{$setc GAP_INTERFACES_VERSION := $0210}
>>>>>>> graemeg/fixes_2_2
=======
{$setc UNIVERSAL_INTERFACES_VERSION := $0342}
{$setc GAP_INTERFACES_VERSION := $0210}
>>>>>>> origin/fixes_2_2
=======
{$setc UNIVERSAL_INTERFACES_VERSION := $0400}
{$setc GAP_INTERFACES_VERSION := $0308}
>>>>>>> origin/fixes_2.4

{$ifc not defined USE_CFSTR_CONSTANT_MACROS}
    {$setc USE_CFSTR_CONSTANT_MACROS := TRUE}
{$endc}

{$ifc defined CPUPOWERPC and defined CPUI386}
	{$error Conflicting initial definitions for CPUPOWERPC and CPUI386}
{$endc}
{$ifc defined FPC_BIG_ENDIAN and defined FPC_LITTLE_ENDIAN}
	{$error Conflicting initial definitions for FPC_BIG_ENDIAN and FPC_LITTLE_ENDIAN}
{$endc}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc not defined __ppc__ and defined CPUPOWERPC32}
=======
{$ifc not defined __ppc__ and defined CPUPOWERPC}
>>>>>>> graemeg/fixes_2_2
=======
{$ifc not defined __ppc__ and defined CPUPOWERPC}
>>>>>>> origin/fixes_2_2
=======
{$ifc not defined __ppc__ and defined CPUPOWERPC32}
>>>>>>> origin/fixes_2.4
	{$setc __ppc__ := 1}
{$elsec}
	{$setc __ppc__ := 0}
{$endc}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{$ifc not defined __ppc64__ and defined CPUPOWERPC64}
	{$setc __ppc64__ := 1}
{$elsec}
	{$setc __ppc64__ := 0}
{$endc}
<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{$ifc not defined __i386__ and defined CPUI386}
	{$setc __i386__ := 1}
{$elsec}
	{$setc __i386__ := 0}
{$endc}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
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
=======
>>>>>>> origin/fixes_2.4

{$ifc defined cpu64}
  {$setc __LP64__ := 1}
{$elsec}
  {$setc __LP64__ := 0}
{$endc}

<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{$ifc defined __ppc__ and __ppc__ and defined __i386__ and __i386__}
	{$error Conflicting definitions for __ppc__ and __i386__}
{$endc}

{$ifc defined __ppc__ and __ppc__}
	{$setc TARGET_CPU_PPC := TRUE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_PPC64 := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_CPU_ARM64 := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
	{$setc TARGET_OS_EMBEDDED := FALSE}
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_CPU_ARM64 := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
=======
>>>>>>> graemeg/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
=======
>>>>>>> graemeg/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
=======
>>>>>>> origin/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
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
>>>>>>> origin/cpstrnew
{$ifc defined(iphonesim)}
 	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := TRUE}
<<<<<<< HEAD
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
=======
>>>>>>> graemeg/cpstrnew
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
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
=======
>>>>>>> graemeg/cpstrnew
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
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
>>>>>>> graemeg/cpstrnew
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
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
=======
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
>>>>>>> origin/cpstrnew
  {$setc TARGET_CPU_64 := FALSE}
{$endc}
=======
=======
>>>>>>> origin/fixes_2_2
=======
	{$setc TARGET_CPU_PPC64 := FALSE}
>>>>>>> origin/fixes_2.4
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elifc defined __i386__ and __i386__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := TRUE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
{$ifc defined(iphonesim)}
 	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := TRUE}
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
  {$setc TARGET_CPU_64 := FALSE}
{$endc}
<<<<<<< HEAD
{$setc TARGET_CPU_PPC_64 := FALSE}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
{$setc TARGET_OS_MAC := TRUE}
>>>>>>> graemeg/fixes_2_2
=======
{$setc TARGET_OS_MAC := TRUE}
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not MACOSALLINCLUDE}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{$ifc not TARGET_CPU_64 and TARGET_OS_MAC}
=======

{$ifc not TARGET_CPU_64 and TARGET_OS_MAC}

{$ALIGN POWER}
>>>>>>> graemeg/cpstrnew

=======
=======

{$ifc not TARGET_CPU_64 and TARGET_OS_MAC}

{$ALIGN POWER}
>>>>>>> graemeg/cpstrnew

<<<<<<< HEAD
{$ifc not TARGET_CPU_64 and TARGET_OS_MAC}

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
{$ALIGN POWER}


=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======

{$ifc not TARGET_CPU_64 and TARGET_OS_MAC}

{$ALIGN POWER}


>>>>>>> origin/cpstrnew
{$ifc TARGET_CPU_64 }
{$errorc 64-bit not supported}
{$endc}
=======
=======
>>>>>>> origin/fixes_2_2
{$ALIGN POWER}
=======
{$endc} {not MACOSALLINCLUDE}


{$ifc not TARGET_CPU_64 and TARGET_OS_MAC}
>>>>>>> origin/fixes_2.4

{$ALIGN POWER}


{$ifc TARGET_CPU_64 }
{$errorc 64-bit not supported}
{$endc}
<<<<<<< HEAD
{$endc}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

type
  objc_ivar_list_Ptr = ^objc_ivar_list;
  objc_method_list_PtrPtr = ^objc_method_list_Ptr;  
  objc_method_list_Ptr = ^objc_method_list;
  objc_cache_Ptr = ^objc_cache;
  objc_protocol_list_Ptr = ^objc_protocol_list;

{ 
 *	Class Template
 }
	objc_class_Ptr = ^objc_class;
	objc_class = record
		isa: objc_class_Ptr;	
		super_class: objc_class_Ptr;	
		name: CStringPtr;
		version: SInt32;
		info: SInt32;
		instance_size: SInt32;
		ivars: objc_ivar_list_Ptr;
		methodLists: objc_method_list_PtrPtr;
		cache: objc_cache_Ptr;
		protocols: objc_protocol_list_Ptr
	end;

{ 
 *	Category Template
 }
	objc_category_Ptr = ^objc_category;
	objc_category = record
		category_name: CStringPtr;
		class_name: CStringPtr;
		instance_methods: objc_method_list_Ptr;
		class_methods: objc_method_list_Ptr;
		protocols: objc_protocol_list_Ptr
	end;
	Category = objc_category_Ptr;

{ 
 *	Instance Variable Template
 }
	objc_ivar_Ptr = ^objc_ivar;
	objc_ivar = record
		ivar_name: CStringPtr;
		ivar_type: CStringPtr;
		ivar_offset: SInt32;
{$ifc not undefined __alpha__ }
		space: SInt32;
{$endc}
	end;
	Ivar = objc_ivar_Ptr;

	objc_ivar_list = record
		ivar_count: SInt32;
{$ifc not undefined __alpha__ }
		space: SInt32;
{$endc}
		ivar_list: array[ 0..0] of objc_ivar		{ variable length structure }
	end;

	{ use _ObjC suffix to avoid Object Pascal class an self conflicts }
	Class_ObjC_Ptr = ^Class_ObjC;
	Class_ObjC = objc_class_Ptr;

	objc_object_Ptr = ^objc_object;
	objc_object = record
		isa: Class_ObjC
	end;

  objc_id_Ptr = ^objc_id;
  objc_id = objc_object_Ptr;

	id = objc_id;
	id_Ptr = objc_id_Ptr;

  objc_selector_Ptr = UnivPtr;  

  SELPtr = ^SEL;  
  SEL = objc_selector_Ptr;

	IMP = function( self_ObjC: objc_id; param2: SEL; ... ): objc_id; 

{ 
 *	Method Template
 }
	objc_method_Ptr = ^objc_method;
	objc_method = record
		method_name: SEL;
		method_types: CStringPtr;
		method_imp: IMP
	end;
	Method = objc_method_Ptr;

	objc_method_list = record
		obsolete: objc_method_list_Ptr;
		method_count: SInt32;
{$ifc not undefined __alpha__ }
		space: SInt32;
{$endc}
    method_list: array[0..0] of objc_method	{ variable length structure }
	end;

{ Protocol support }

	Protocol = objc_object;
	objc_protocol_list = record
		next: objc_protocol_list_Ptr;
		count: SInt32;
    list: array[0..0] of Protocol
	end;

	Cache = objc_cache_Ptr;
	objc_cache = record
		mask: UInt32;            { total = mask + 1 }
		occupied: UInt32;        
    buckets: array[0..0] of Method
	end;

type
	BOOL = boolean; {SInt8}
{
	BOOL is explicitly signed so @encode(BOOL) == "c" rather than "C" 
  even if -funsigned-char is used.
}

const
  YES            = True; {BOOL(1)}
  NO             = False; {BOOL(0)}

{
# define __strong
}

{$ifc not defined(STRICT_OPENSTEP) }

{
type
	STR = CStringPtr;
}

function sel_isMapped(_sel: SEL): boolean; external name '_sel_isMapped';
function sel_getName(_sel: SEL): CStringPtr; external name '_sel_getName';
function sel_getUid( str: CStringPtr): SEL; external name '_sel_getUid';
function sel_registerName(str: CStringPtr): SEL; external name '_sel_registerName';
function object_getClassName(obj: objc_id): CStringPtr; external name '_object_getClassName';
function object_getIndexedIvars(obj: objc_id): UnivPtr; external name '_object_getIndexedIvars';


{$ifc defined(__osf__) and defined(__alpha__) }
type
	arith_t = SInt32;
	uarith_t = UInt32;
const ARITH_SHIFT = 32;
{$elsec}
type
	arith_t = SInt32;
	uarith_t = UInt32;
const ARITH_SHIFT = 16;
{$endc}
{$endc}	{ not defined(STRICT_OPENSTEP) }


const
	CLS_CLASS							= $01;
	CLS_META							= $02;
	CLS_INITIALIZED				= $04;
	CLS_POSING						= $08;
	CLS_MAPPED						= $010;
	CLS_FLUSH_CACHE				= $020;
	CLS_GROW_CACHE				= $040;
	CLS_NEED_BIND					= $080;
	CLS_METHOD_ARRAY  		= $0100;
{ the JavaBridge constructs classes with these markers }
	CLS_JAVA_HYBRID				= $0200;
	CLS_JAVA_CLASS				= $0400;
{ thread-safe +initialize }
	CLS_INITIALIZING			= $0800;
{ bundle unloading }
	CLS_FROM_BUNDLE				= $01000;
{ C++ ivar support }
	CLS_HAS_CXX_STRUCTORS	= $02000;
{ Lazy method list arrays }
	CLS_NO_METHOD_ARRAY		= $04000;
{ +load implementation }
	CLS_HAS_LOAD_METHOD		= $08000;


function object_setInstanceVariable(param1: objc_id; name: CStringPtr; param3: UnivPtr): Ivar; external name '_object_setInstanceVariable';
function object_getInstanceVariable(param1: objc_id; name: CStringPtr; var param3: UnivPtr): Ivar; external name '_object_getInstanceVariable';

{ Definitions of filer types }

Const
	_C_ID			  = '@';
	_C_CLASS	  = '#';
	_C_SEL		  = ':';
	_C_CHR		  = 'c';
	_C_UCHR		  = 'C';
	_C_SHT		  = 's';
	_C_USHT		  = 'S';
	_C_INT		  = 'i';
	_C_UINT		  = 'I';
	_C_LNG		  = 'l';
	_C_ULNG		  = 'L';
	_C_FLT		  = 'f';
	_C_DBL		  = 'd';
	_C_BFLD		  = 'b';
	_C_VOID		  = 'v';
	_C_UNDEF	  = '?';
	_C_PTR		  = '^';
	_C_CHARPTR	= '*';
	_C_ARY_B	  = '[';
	_C_ARY_E	  = ']';
	_C_UNION_B	= '(';
	_C_UNION_E	= ')';
	_C_STRUCT_B	= '{';
	_C_STRUCT_E	= '}';

{ Structure for method cache - allocated/sized at runtime }
{todo}
{
#define CACHE_BUCKET_NAME(B)  ((B)->method_name)
#define CACHE_BUCKET_IMP(B)   ((B)->method_imp)
#define CACHE_BUCKET_VALID(B) (B)
#define CACHE_HASH(sel, mask) (((uarith_t)(sel)>>2) & (mask))
}

{ operations }
function class_createInstance( param1: Class_ObjC; idxIvars: UInt32 ): objc_id; external name '_class_createInstance';
function class_createInstanceFromZone( param1: Class_ObjC; idxIvars: UInt32; z: UnivPtr): objc_id; external name '_class_createInstanceFromZone';

procedure class_setVersion( param1: Class_ObjC; param2: SInt32); external name '_class_setVersion';
function class_getVersion( param1: Class_ObjC): SInt32; external name '_class_getVersion';

function class_getInstanceVariable( param1: Class_ObjC; param2: CStringPtr): Ivar; external name '_class_getInstanceVariable';
function class_getInstanceMethod( param1: Class_ObjC; param2: SEL): Method; external name '_class_getInstanceMethod';
function class_getClassMethod( param1: Class_ObjC; param2: SEL): Method; external name '_class_getClassMethod';

procedure class_addMethods( param1: Class_ObjC; param2: objc_method_list_Ptr); external name '_class_addMethods';
procedure class_removeMethods( param1: Class_ObjC; param2: objc_method_list_Ptr); external name '_class_removeMethods';

function class_poseAs( imposter: Class_ObjC; original: Class_ObjC): Class_ObjC; external name '_class_poseAs';

function method_getNumberOfArguments( param1: Method): UInt32; external name '_method_getNumberOfArguments';
function method_getSizeOfArguments( param1: Method): UInt32; external name '_method_getSizeOfArguments';
function method_getArgumentInfo( m: Method; arg: SInt32; var argtype: CStringPtr; var offset: SInt32): UInt32; external name '_method_getArgumentInfo';

{
  usage for nextMethodList

 void *iterator = 0;
 struct objc_method_list *mlist;
 while ( mlist = class_nextMethodList( cls, &iterator ) )
}

const
	OBJC_NEXT_METHOD_LIST = 1;

function  class_nextMethodList( param1: Class_ObjC; var param2: UnivPtr): objc_method_list_Ptr; external name '_class_nextMethodList';

type
	marg_list = UnivPtr;

{$ifc TARGET_CPU_PPC}
const
	marg_prearg_size = 128;
{$elsec}
const
	marg_prearg_size = 0;
{$endc}

{todo}
{
#define marg_malloc(margs, method) \
	do ( \
		margs = (marg_list *)malloc (marg_prearg_size + ((7 + method_getSizeOfArguments(method)) & ~7)); \
	) while (0)


#define marg_free(margs) \
	do ( \
		free(margs); \
	) while (0)
	
#define marg_adjustedOffset(method, offset) \
	(marg_prearg_size + offset)


#define marg_getRef(margs, offset, type) \
	( (type *)((char *)margs + marg_adjustedOffset(method,offset) ) )

#define marg_getValue(margs, offset, type) \
	( *marg_getRef(margs, offset, type) )

#define marg_setValue(margs, offset, type, value) \
	( marg_getValue(margs, offset, type) = (value) )
}

{ Collection utilities }

const
	OBJC_GENERATIONAL = 1 shl 0;

procedure objc_collect_if_needed( options: UInt32); external name '_objc_collect_if_needed';
function objc_numberAllocated: UInt32; external name '_objc_numberAllocated';
function objc_collecting_enabled: boolean; external name '_objc_collecting_enabled';

{ Memory management }
function objc_allocate_object(cls: Class_ObjC; extra: SInt32): objc_id; external name '_objc_allocate_object';

{ Write barriers }
function objc_assign_strongCast( val: objc_id; dest: objc_id_Ptr): objc_id; external name '_objc_assign_strongCast';
function objc_assign_global( val: objc_id; dest: objc_id_Ptr): objc_id; external name '_objc_assign_global';
function objc_assign_ivar( value: objc_id; dest: objc_id_ptr; offset: UInt32): objc_id; external name '_objc_assign_ivar';
function objc_memmove_collectable( dst: UnivPtr; src: UnivPtr; size: size_t): UnivPtr; external name '_objc_memmove_collectable';

{ Testing tools }
function objc_is_finalized( ptr: UnivPtr): boolean; external name '_objc_is_finalized';


{ compiler reserves a setjmp buffer + 4 words as localExceptionData}

procedure objc_exception_throw( exception: objc_id); external name '_objc_exception_throw';
procedure objc_exception_try_enter( localExceptionData: UnivPtr); external name '_objc_exception_try_enter';
procedure objc_exception_try_exit( localExceptionData: UnivPtr); external name '_objc_exception_try_exit';
function objc_exception_extract( localExceptionData: UnivPtr): objc_id; external name '_objc_exception_extract';
function objc_exception_match( exceptionClass: Class_ObjC; exception: objc_id): SInt32; external name '_objc_exception_match';

type
  throw_exc_t = procedure( param1: objc_id);
  try_enter_t = procedure( param1: UnivPtr);
  try_exit_t = procedure( param1: UnivPtr);
  extract_t = function( param1: UnivPtr): objc_id;
  match_t = function( param1: Class_ObjC; param2: objc_id): SInt32;

  objc_exception_functions_t_Ptr = ^objc_exception_functions_t;
	objc_exception_functions_t = record
		version: SInt32;
    throw_exc: throw_exc_t;    { version 0 }
    try_enter: try_enter_t;    { version 0 }
    try_exit: try_exit_t;      { version 0 }
    extract: extract_t;        { version 0 }
    match: match_t;	           { version 0 }
	end;

{ get table; version tells how many }
procedure objc_exception_get_functions( table: objc_exception_functions_t_Ptr); external name '_objc_exception_get_functions';

{ set table }
procedure objc_exception_set_functions( table: objc_exception_functions_t_Ptr); external name '_objc_exception_set_functions';




{
   Begin synchronizing on 'obj'.
   Allocates recursive pthread_mutex associated with 'obj' if needed.
   Returns OBJC_SYNC_SUCCESS once lock is acquired.
}
function objc_sync_enter( obj: objc_id): SInt32; external name '_objc_sync_enter';

{
   End synchronizing on 'obj'.
   Returns OBJC_SYNC_SUCCESS or OBJC_SYNC_NOT_OWNING_THREAD_ERROR
}
function objc_sync_exit( obj: objc_id): SInt32; external name '_objc_sync_exit';

{
   Temporarily release lock on 'obj' and wait for another thread to notify on 'obj'
   Return OBJC_SYNC_SUCCESS, OBJC_SYNC_NOT_OWNING_THREAD_ERROR, OBJC_SYNC_TIMED_OUT
}
function objc_sync_wait( obj: objc_id; milliSecondsMaxWait: SInt64): SInt32; external name '_objc_sync_wait';

{
   Wake up another thread waiting on 'obj'
   Return OBJC_SYNC_SUCCESS, OBJC_SYNC_NOT_OWNING_THREAD_ERROR
}
function objc_sync_notify( obj: objc_id): SInt32; external name '_objc_sync_notify';

{ 
   Wake up all threads waiting on 'obj'
   Return OBJC_SYNC_SUCCESS, OBJC_SYNC_NOT_OWNING_THREAD_ERROR
}
function objc_sync_notifyAll( obj: objc_id): SInt32; external name '_objc_sync_notifyAll';

const
	OBJC_SYNC_SUCCESS = 0;
	OBJC_SYNC_NOT_OWNING_THREAD_ERROR = -1;
	OBJC_SYNC_TIMED_OUT = -2;
	OBJC_SYNC_NOT_INITIALIZED = -3;

type
	objc_symtab_Ptr = ^objc_symtab;
	objc_symtab = record
		sel_ref_cnt: UInt32;
		refs: SELPtr;		
		cls_def_cnt: UInt16;
		cat_def_cnt: UInt16;
		defs: array[ 0..0] of UnivPtr
	end;
	Symtab = objc_symtab_Ptr;

type
	objc_module_Ptr = ^objc_module;
	objc_module = record
		version: UInt32;
		size: UInt32;
		name: CStringPtr;
		_symtab: Symtab
	end;
	Module = objc_module_Ptr;

type
	objc_super_Ptr = ^objc_super;
	objc_super = record
		receiver: objc_id;
		super_class: Class_ObjC
	end;

{
 * Messaging Primitives (prototypes)
 }

function objc_getClass( name: cStringPtr): objc_id; external name '_objc_getClass';
function objc_getMetaClass( name: CStringPtr): objc_id; external name '_objc_getMetaClass';
function objc_msgSend( self_ObjC: objc_id; op: SEL; ...): objc_id; external name '_objc_msgSend'; (* attribute ignoreable *)
function objc_msgSendSuper( super: objc_super_Ptr; op: SEL; ...): objc_id; external name '_objc_msgSendSuper'; (* attribute ignoreable *)


{ Floating-point-returning Messaging Primitives (prototypes)
 * 
 * On some platforms, the ABI for functions returning a floating-point 
 * value is incompatible with that for functions returning an integral type. 
 * objc_msgSend_fpret must be used for these. 
 * 
 * ppc: objc_msgSend_fpret not used
 * ppc64: objc_msgSend_fpret not used
 * i386: objc_msgSend_fpret REQUIRED
 *
 * For `float` or `long double` return types, cast the function 
 * to an appropriate function pointer type first.
 }

{$ifc TARGET_CPU_X86}
function objc_msgSend_fpret(self_ObjC: objc_id; op: SEL; ...): double; external name '_objc_msgSend_fpret';
{$endc}


{ Struct-returning Messaging Primitives (prototypes)
 *
 * For historical reasons, the prototypes for the struct-returning 
 * messengers are unusual. The portable, correct way to call these functions 
 * is to cast them to your desired return type first.
 * 
 * For example, `NSRect result = [myNSView frame]` could be written as:
 *   NSRect (*msgSend_stret_fn)(id, SEL, ...) = (NSRect(*)(id, SEL, ...))objc_msgSend_stret;
 *   NSRect result = (*msgSend_stret_fn)(myNSView, @selector(frame));
 * or, without the function pointer:
 *   NSRect result = (*(NSRect(*)(id, SEL, ...))objc_msgSend_stret)(myNSView, @selector(frame));
 * 
 * BE WARNED that these prototypes have changed in the past and will change 
 * in the future. Code that uses a cast like the example above will be 
 * unaffected. 
 }

procedure objc_msgSend_stret( stretAddr: UnivPtr; self_ObjC: objc_id; op: SEL; ...); external name '_objc_msgSend_stret';
procedure objc_msgSendSuper_stret( stretAddr: UnivPtr; super: objc_super_Ptr; op: SEL; ...); external name '_objc_msgSendSuper_stret';


{ Forwarding }

{ Note that objc_msgSendv_stret() does not return a structure type, 
 * and should not be cast to do so. This is unlike objc_msgSend_stret() 
 * and objc_msgSendSuper_stret().
 }

function objc_msgSendv( self_ObjC: objc_id; op: SEL; arg_size: UInt32; arg_frame: marg_list): objc_id; external name '_objc_msgSendv';
procedure objc_msgSendv_stret( stretAddr: UnivPtr; self_ObjC: objc_id; op: SEL; arg_size: UInt32; arg_frame: marg_list); external name '_objc_msgSendv_stret';
{$ifc TARGET_CPU_X86}
function  objc_msgSendv_fpret( seflid: objc_id; op: SEL; arg_size: UInt32; arg_frame: marg_list ): double; external name '_objc_msgSendv_fpret';
{$endc}


{ 
    getting all the classes in the application...
    
    int objc_getClassList(buffer, bufferLen)
	classes is an array of Class values (which are pointers)
		which will be filled by the function; if this
		argument is NULL, no copying is done, only the
		return value is returned
	bufferLen is the number of Class values the given buffer
		can hold; if the buffer is not large enough to
		hold all the classes, the buffer is filled to
		the indicated capacity with some arbitrary subset
		of the known classes, which could be different
		from call to call
	returns the number of classes, which is the number put
		in the buffer if the buffer was large enough,
		or the length the buffer should have been

    int numClasses = 0, newNumClasses = objc_getClassList(NULL, 0);
    Class *classes = NULL;
    while (numClasses < newNumClasses) (
        numClasses = newNumClasses;
        classes = realloc(classes, sizeof(Class) * numClasses);
        newNumClasses = objc_getClassList(classes, numClasses);
    )
    // now, can use the classes list; if NULL, there are no classes
    free(classes);

}
function objc_getClassList( buffer: Class_ObjC_Ptr; bufferLen: SInt32): SInt32; external name '_objc_getClassList';

{$setc OBSOLETE_OBJC_GETCLASSES := 1}
{$ifc OBSOLETE_OBJC_GETCLASSES}
function objc_getClasses: UnivPtr; external name '_objc_getClasses';
{$endc}

function objc_lookUpClass( name: CStringPtr): objc_id; external name '_objc_lookUpClass';
function objc_getRequiredClass( name: CStringPtr): objc_id; external name '_objc_getRequiredClass';
procedure objc_addClass( myClass: Class_ObjC ); external name '_objc_addClass';

{ customizing the error handling for objc_getClass/objc_getMetaClass }
type
  ClassHandlerCallback = function( param1: CStringPtr): SInt32;

procedure objc_setClassHandler( handler: ClassHandlerCallback); external name '_objc_setClassHandler';

{ Making the Objective-C runtime thread safe. }
procedure objc_setMultithreaded ( flag: boolean); external name '_objc_setMultithreaded';

{ overriding the default object allocation and error handling routines }

{todo}
{
OBJC_EXPORT id	(_alloc)(Class, unsigned int);
OBJC_EXPORT id	(_copy)(id, unsigned int);
OBJC_EXPORT id	(_realloc)(id, unsigned int);
OBJC_EXPORT id	(_dealloc)(id);
OBJC_EXPORT id	(_zoneAlloc)(Class, unsigned int, void *);
OBJC_EXPORT id	(_zoneRealloc)(id, unsigned int, void *);
OBJC_EXPORT id	(_zoneCopy)(id, unsigned int, void *);

OBJC_EXPORT void	(_error)(id, const char *, va_list);
}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not TARGET_CPU_64 and TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
end.
>>>>>>> graemeg/fixes_2_2
=======
end.
>>>>>>> origin/fixes_2_2
=======
{$endc} {not TARGET_CPU_64 and TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4
