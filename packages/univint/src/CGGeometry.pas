{ CoreGraphics - CGGeometry.h
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   Copyright (c) 1998-2011 Apple Inc.
   All rights reserved. }
{       Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2015 }
=======
   Copyright (c) 1998-2009 Apple Inc.
   All rights reserved. }
{       Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
   Copyright (c) 1998-2009 Apple Inc.
   All rights reserved. }
{       Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
   Copyright (c) 1998-2009 Apple Inc.
   All rights reserved. }
{       Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
   Copyright (c) 1998-2009 Apple Inc.
   All rights reserved. }
{       Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> origin/cpstrnew
{
    Modified for use with Free Pascal
    Version 308
    Please report any bugs to <gpc@microbizz.nl>
}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
 * Copyright (c) 1998-2003 Apple Computer, Inc.
 * All rights reserved.
 }
=======
   Copyright (c) 1998-2009 Apple Inc.
   All rights reserved. }
>>>>>>> origin/fixes_2.4
{       Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{
    Modified for use with Free Pascal
    Version 308
    Please report any bugs to <gpc@microbizz.nl>
}

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4
{$mode macpas}
{$packenum 1}
{$macro on}
{$inline on}
{$calling mwpascal}

unit CGGeometry;
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
=======
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
>>>>>>> origin/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
>>>>>>> graemeg/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
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
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
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
	{$setc TARGET_OS_EMBEDDED := FALSE}
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
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
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := FALSE}
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
	{$setc TARGET_CPU_ARM64 := FALSE}
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
	{$setc TARGET_OS_EMBEDDED := TRUE}
{$elifc defined __arm64__ and __arm64__}
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
{$elifc defined __arm__ and __arm__}
>>>>>>> graemeg/cpstrnew
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_CPU_ARM64 := TRUE}
=======
	{$setc TARGET_CPU_ARM := TRUE}
>>>>>>> graemeg/cpstrnew
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := TRUE}
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ nor __arm64__ is defined.}
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
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  {$setc TARGET_CPU_64 := FALSE}
{$endc}
=======
=======
>>>>>>> origin/fixes_2_2
=======
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
>>>>>>> origin/fixes_2.4
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
uses MacTypes,CFBase,CFDictionary,CGBase;
{$endc} {not MACOSALLINCLUDE}

=======
uses MacTypes,CFBase,CGBase;
>>>>>>> graemeg/fixes_2_2
=======
uses MacTypes,CFBase,CGBase;
>>>>>>> origin/fixes_2_2
=======
uses MacTypes,CFBase,CFDictionary,CGBase;
{$endc} {not MACOSALLINCLUDE}

>>>>>>> origin/fixes_2.4
{$ALIGN POWER}


{ Points. }

type
	CGPointPtr = ^CGPoint;
	CGPoint = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		x: CGFloat;
		y: CGFloat;
=======
		x: Float32;
		y: Float32;
>>>>>>> graemeg/fixes_2_2
=======
		x: Float32;
		y: Float32;
>>>>>>> origin/fixes_2_2
=======
		x: CGFloat;
		y: CGFloat;
>>>>>>> origin/fixes_2.4
	end;

{ Sizes. }

type
	CGSizePtr = ^CGSize;
	CGSize = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		width: CGFloat;
		height: CGFloat;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	end;

{ Vectors. }

const
	CGVECTOR_DEFINED = 1;

type
  CGVectorPtr = ^CGVector;
	CGVector = record
		dx: CGFloat;
		dy: CGFloat;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
		width: Float32;
		height: Float32;
>>>>>>> graemeg/fixes_2_2
=======
		width: Float32;
		height: Float32;
>>>>>>> origin/fixes_2_2
=======
		width: CGFloat;
		height: CGFloat;
>>>>>>> origin/fixes_2.4
	end;

{ Rectangles. }

type
	CGRectPtr = ^CGRect;
	CGRect = record
		origin: CGPoint;
		size: CGSize;
	end;

{ Rectangle edges. }

type
	CGRectEdge = SInt32;
const
	CGRectMinXEdge = 0;
	CGRectMinYEdge = 1;
	CGRectMaxXEdge = 2;
	CGRectMaxYEdge = 3;

{ The "zero" point -- equivalent to CGPointMake(0, 0). } 

var CGPointZero: CGPoint; external name '_CGPointZero'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ The "zero" size -- equivalent to CGSizeMake(0, 0). } 

var CGSizeZero: CGSize; external name '_CGSizeZero'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ The "zero" rectangle -- equivalent to CGRectMake(0, 0, 0, 0). } 

var CGRectZero: CGRect; external name '_CGRectZero'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ The "empty" rect. This is the rectangle returned when, for example, we
   intersect two disjoint rectangles. Note that the null rect is not the
   same as the zero rect. }

var CGRectNull: CGRect; external name '_CGRectNull'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ The infinite rectangle. }

var CGRectInfinite: CGRect; external name '_CGRectInfinite'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Return the leftmost x-value of `rect'. }

function CGRectGetMinX( rect: CGRect ): CGFloat; external name '_CGRectGetMinX';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the midpoint x-value of `rect'. }

function CGRectGetMidX( rect: CGRect ): CGFloat; external name '_CGRectGetMidX';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the rightmost x-value of `rect'. }

function CGRectGetMaxX( rect: CGRect ): CGFloat; external name '_CGRectGetMaxX';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the bottommost y-value of `rect'. }

function CGRectGetMinY( rect: CGRect ): CGFloat; external name '_CGRectGetMinY';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the midpoint y-value of `rect'. }

function CGRectGetMidY( rect: CGRect ): CGFloat; external name '_CGRectGetMidY';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the topmost y-value of `rect'. }

function CGRectGetMaxY( rect: CGRect ): CGFloat; external name '_CGRectGetMaxY';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the width of `rect'. }

function CGRectGetWidth( rect: CGRect ): CGFloat; external name '_CGRectGetWidth';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the height of `rect'. }

function CGRectGetHeight( rect: CGRect ): CGFloat; external name '_CGRectGetHeight';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD



{ Return true if `rect1' and `rect2' are the same, false otherwise. }

=======



{ Return true if `rect1' and `rect2' are the same, false otherwise. }

>>>>>>> graemeg/cpstrnew
=======



{ Return true if `rect1' and `rect2' are the same, false otherwise. }

>>>>>>> graemeg/cpstrnew
=======



{ Return true if `rect1' and `rect2' are the same, false otherwise. }

>>>>>>> graemeg/cpstrnew
=======



{ Return true if `rect1' and `rect2' are the same, false otherwise. }

>>>>>>> origin/cpstrnew
function CGRectEqualToRect( rect1: CGRect; rect2: CGRect ): CBool; external name '_CGRectEqualToRect';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Standardize `rect' -- i.e., convert it to an equivalent rect which has
   positive width and height. }

function CGRectStandardize( rect: CGRect ): CGRect; external name '_CGRectStandardize';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return true if `rect' is empty (that is, if it has zero width or height),
   false otherwise. A null rect is defined to be empty. }

function CGRectIsEmpty( rect: CGRect ): CBool; external name '_CGRectIsEmpty';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return true if `rect' is the null rectangle, false otherwise. }

function CGRectIsNull( rect: CGRect ): CBool; external name '_CGRectIsNull';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return true if `rect' is the infinite rectangle, false otherwise. }

function CGRectIsInfinite( rect: CGRect ): CBool; external name '_CGRectIsInfinite';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Inset `rect' by `(dx, dy)' -- i.e., offset its origin by `(dx, dy)', and
   decrease its size by `(2*dx, 2*dy)'. }

function CGRectInset( rect: CGRect; dx: CGFloat; dy: CGFloat ): CGRect; external name '_CGRectInset';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Expand `rect' to the smallest rect containing it with integral origin and
   size. }

function CGRectIntegral( rect: CGRect ): CGRect; external name '_CGRectIntegral';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ The "empty" rect. This is the rectangle returned when, for example, we
   intersect two disjoint rectangles. Note that the null rect is not the
   same as the zero rect. }

var CGRectNull: CGRect; external name '_CGRectNull'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ The infinite rectangle. }

var CGRectInfinite: CGRect; external name '_CGRectInfinite'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Return the leftmost x-value of `rect'. }

function CGRectGetMinX( rect: CGRect ): CGFloat; external name '_CGRectGetMinX';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the midpoint x-value of `rect'. }

function CGRectGetMidX( rect: CGRect ): CGFloat; external name '_CGRectGetMidX';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the rightmost x-value of `rect'. }

function CGRectGetMaxX( rect: CGRect ): CGFloat; external name '_CGRectGetMaxX';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the bottommost y-value of `rect'. }

function CGRectGetMinY( rect: CGRect ): CGFloat; external name '_CGRectGetMinY';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the midpoint y-value of `rect'. }

function CGRectGetMidY( rect: CGRect ): CGFloat; external name '_CGRectGetMidY';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the topmost y-value of `rect'. }

function CGRectGetMaxY( rect: CGRect ): CGFloat; external name '_CGRectGetMaxY';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the width of `rect'. }

function CGRectGetWidth( rect: CGRect ): CGFloat; external name '_CGRectGetWidth';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the height of `rect'. }

function CGRectGetHeight( rect: CGRect ): CGFloat; external name '_CGRectGetHeight';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)



{ Return true if `rect1' and `rect2' are the same, false otherwise. }

function CGRectEqualToRect( rect1: CGRect; rect2: CGRect ): CBool; external name '_CGRectEqualToRect';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Standardize `rect' -- i.e., convert it to an equivalent rect which has
   positive width and height. }

function CGRectStandardize( rect: CGRect ): CGRect; external name '_CGRectStandardize';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return true if `rect' is empty (that is, if it has zero width or height),
   false otherwise. A null rect is defined to be empty. }

function CGRectIsEmpty( rect: CGRect ): CBool; external name '_CGRectIsEmpty';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return true if `rect' is the null rectangle, false otherwise. }

function CGRectIsNull( rect: CGRect ): CBool; external name '_CGRectIsNull';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return true if `rect' is the infinite rectangle, false otherwise. }

function CGRectIsInfinite( rect: CGRect ): CBool; external name '_CGRectIsInfinite';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Inset `rect' by `(dx, dy)' -- i.e., offset its origin by `(dx, dy)', and
   decrease its size by `(2*dx, 2*dy)'. }

function CGRectInset( rect: CGRect; dx: CGFloat; dy: CGFloat ): CGRect; external name '_CGRectInset';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Expand `rect' to the smallest rect containing it with integral origin and
   size. }

function CGRectIntegral( rect: CGRect ): CGRect; external name '_CGRectIntegral';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ Return the union of `r1' and `r2'. }

function CGRectUnion( r1: CGRect; r2: CGRect ): CGRect; external name '_CGRectUnion';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the intersection of `r1' and `r2'. This may return a null rect. }

function CGRectIntersection( r1: CGRect; r2: CGRect ): CGRect; external name '_CGRectIntersection';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Offset `rect' by `(dx, dy)'. }

function CGRectOffset( rect: CGRect; dx: CGFloat; dy: CGFloat ): CGRect; external name '_CGRectOffset';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{ Make two new rectangles, `slice' and `remainder', by dividing `rect' with
   a line that's parallel to one of its sides, specified by `edge' -- either
   `CGRectMinXEdge', `CGRectMinYEdge', `CGRectMaxXEdge', or
   `CGRectMaxYEdge'. The size of `slice' is determined by `amount', which
   measures the distance from the specified edge. }

procedure CGRectDivide( rect: CGRect; var slice: CGRect; var remainder: CGRect; amount: CGFloat; edge: CGRectEdge ); external name '_CGRectDivide';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return true if `point' is contained in `rect', false otherwise. }
<<<<<<< HEAD
<<<<<<< HEAD

function CGRectContainsPoint( rect: CGRect; point: CGPoint ): CBool; external name '_CGRectContainsPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
<<<<<<< HEAD

{ Return true if `rect2' is contained in `rect1', false otherwise. `rect2'
   is contained in `rect1' if the union of `rect1' and `rect2' is equal to
   `rect1'. }

function CGRectContainsRect( rect1: CGRect; rect2: CGRect ): CBool; external name '_CGRectContainsRect';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return true if `rect1' intersects `rect2', false otherwise. `rect1'
   intersects `rect2' if the intersection of `rect1' and `rect2' is not the
   null rect. }

function CGRectIntersectsRect( rect1: CGRect; rect2: CGRect ): CBool; external name '_CGRectIntersectsRect';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{** Persistent representations. **}

{ Return a dictionary representation of `point'. }

=======

{ Make two new rectangles, `slice' and `remainder', by dividing `rect' with
   a line that's parallel to one of its sides, specified by `edge' -- either
   `CGRectMinXEdge', `CGRectMinYEdge', `CGRectMaxXEdge', or
   `CGRectMaxYEdge'. The size of `slice' is determined by `amount', which
   measures the distance from the specified edge. }

procedure CGRectDivide( rect: CGRect; var slice: CGRect; var remainder: CGRect; amount: CGFloat; edge: CGRectEdge ); external name '_CGRectDivide';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return true if `point' is contained in `rect', false otherwise. }

function CGRectContainsPoint( rect: CGRect; point: CGPoint ): CBool; external name '_CGRectContainsPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return true if `rect2' is contained in `rect1', false otherwise. `rect2'
   is contained in `rect1' if the union of `rect1' and `rect2' is equal to
   `rect1'. }

function CGRectContainsRect( rect1: CGRect; rect2: CGRect ): CBool; external name '_CGRectContainsRect';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return true if `rect1' intersects `rect2', false otherwise. `rect1'
   intersects `rect2' if the intersection of `rect1' and `rect2' is not the
   null rect. }

function CGRectIntersectsRect( rect1: CGRect; rect2: CGRect ): CBool; external name '_CGRectIntersectsRect';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{** Persistent representations. **}

{ Return a dictionary representation of `point'. }

>>>>>>> graemeg/cpstrnew
=======

{ Return true if `rect2' is contained in `rect1', false otherwise. `rect2'
   is contained in `rect1' if the union of `rect1' and `rect2' is equal to
   `rect1'. }

function CGRectContainsRect( rect1: CGRect; rect2: CGRect ): CBool; external name '_CGRectContainsRect';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return true if `rect1' intersects `rect2', false otherwise. `rect1'
   intersects `rect2' if the intersection of `rect1' and `rect2' is not the
   null rect. }

function CGRectIntersectsRect( rect1: CGRect; rect2: CGRect ): CBool; external name '_CGRectIntersectsRect';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{** Persistent representations. **}

{ Return a dictionary representation of `point'. }

>>>>>>> graemeg/cpstrnew
=======

function CGRectContainsPoint( rect: CGRect; point: CGPoint ): CBool; external name '_CGRectContainsPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return true if `rect2' is contained in `rect1', false otherwise. `rect2'
   is contained in `rect1' if the union of `rect1' and `rect2' is equal to
   `rect1'. }

function CGRectContainsRect( rect1: CGRect; rect2: CGRect ): CBool; external name '_CGRectContainsRect';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return true if `rect1' intersects `rect2', false otherwise. `rect1'
   intersects `rect2' if the intersection of `rect1' and `rect2' is not the
   null rect. }

function CGRectIntersectsRect( rect1: CGRect; rect2: CGRect ): CBool; external name '_CGRectIntersectsRect';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{** Persistent representations. **}

{ Return a dictionary representation of `point'. }

>>>>>>> graemeg/cpstrnew
=======

function CGRectContainsPoint( rect: CGRect; point: CGPoint ): CBool; external name '_CGRectContainsPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return true if `rect2' is contained in `rect1', false otherwise. `rect2'
   is contained in `rect1' if the union of `rect1' and `rect2' is equal to
   `rect1'. }

function CGRectContainsRect( rect1: CGRect; rect2: CGRect ): CBool; external name '_CGRectContainsRect';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return true if `rect1' intersects `rect2', false otherwise. `rect1'
   intersects `rect2' if the intersection of `rect1' and `rect2' is not the
   null rect. }

function CGRectIntersectsRect( rect1: CGRect; rect2: CGRect ): CBool; external name '_CGRectIntersectsRect';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{** Persistent representations. **}

{ Return a dictionary representation of `point'. }

>>>>>>> origin/cpstrnew
function CGPointCreateDictionaryRepresentation( point: CGPoint ): CFDictionaryRef; external name '_CGPointCreateDictionaryRepresentation';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Make a CGPoint from the contents of `dict' (presumably returned earlier
   from `CGPointCreateDictionaryRepresentation') and store the value in
   `point'. Returns true on success; false otherwise. }

function CGPointMakeWithDictionaryRepresentation( dict: CFDictionaryRef; var point: { out } CGPoint ): CBool; external name '_CGPointMakeWithDictionaryRepresentation';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Return a dictionary representation of `size'. }

function CGSizeCreateDictionaryRepresentation( size: CGSize ): CFDictionaryRef; external name '_CGSizeCreateDictionaryRepresentation';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Make a CGSize from the contents of `dict' (presumably returned earlier
   from `CGSizeCreateDictionaryRepresentation') and store the value in
   `size'. Returns true on success; false otherwise. }

function CGSizeMakeWithDictionaryRepresentation( dict: CFDictionaryRef; var size: { out } CGSize ): CBool; external name '_CGSizeMakeWithDictionaryRepresentation';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Return a dictionary representation of `rect'. }

function CGRectCreateDictionaryRepresentation( rect: CGRect ): CFDictionaryRef; external name '_CGRectCreateDictionaryRepresentation';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Make a CGRect from the contents of `dict' (presumably returned earlier
   from `CGRectCreateDictionaryRepresentation') and store the value in
   `rect'. Returns true on success; false otherwise. }

function CGRectMakeWithDictionaryRepresentation( dict: CFDictionaryRef; var rect: { out } CGRect ): CBool; external name '_CGRectMakeWithDictionaryRepresentation';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)


{** Definitions of inline functions. **}
// CG_INLINE CGPoint
function CGPointMake(x: CGFloat; y: CGFloat): CGPoint; inline;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
{
  CGPoint p; p.x = x; p.y = y; return p;
}

// CG_INLINE CGSize
function CGSizeMake(width: CGFloat; height: CGFloat): CGSize; inline;
<<<<<<< HEAD
<<<<<<< HEAD
{
  CGSize size; size.width = width; size.height = height; return size;
}
// CG_INLINE CGVector
function CGVectorMake(dx: CGFloat; dy: CGFloat): CGVector; inline;

=======
{
  CGSize size; size.width = width; size.height = height; return size;
}
>>>>>>> graemeg/cpstrnew

// CG_INLINE CGRect
// seems not useful to inline to me, is fairly big (unless you can reschedule
// all the stores among the rest of the code, but still will probably increase
// code size in all cases)
function CGRectMake(x: CGFloat; y: CGFloat; width: CGFloat; height: CGFloat): CGRect;
<<<<<<< HEAD
{
  CGRect rect;
  rect.origin.x = x; rect.origin.y = y;
  rect.size.width = width; rect.size.height = height;
  return rect;
}

// CG_INLINE bool
function CGPointEqualToPoint(const point1: CGPoint; const point2: CGPoint): boolean; inline;
{
  return point1.x == point2.x && point1.y == point2.y;
}

// CG_INLINE bool
=======
{
  CGPoint p; p.x = x; p.y = y; return p;
}

// CG_INLINE CGSize
function CGSizeMake(width: CGFloat; height: CGFloat): CGSize; inline;
{
  CGSize size; size.width = width; size.height = height; return size;
}

// CG_INLINE CGRect
// seems not useful to inline to me, is fairly big (unless you can reschedule
// all the stores among the rest of the code, but still will probably increase
// code size in all cases)
function CGRectMake(x: CGFloat; y: CGFloat; width: CGFloat; height: CGFloat): CGRect;
{
  CGRect rect;
  rect.origin.x = x; rect.origin.y = y;
  rect.size.width = width; rect.size.height = height;
  return rect;
}

// CG_INLINE bool
function CGPointEqualToPoint(const point1: CGPoint; const point2: CGPoint): boolean; inline;
{
  return point1.x == point2.x && point1.y == point2.y;
}

// CG_INLINE bool
>>>>>>> origin/cpstrnew
function CGSizeEqualToSize(size1: CGSize; size2: CGSize): boolean; inline;
{
  return size1.width == size2.width && size1.height == size2.height;
}

<<<<<<< HEAD
=======
{
  CGSize size; size.width = width; size.height = height; return size;
}

// CG_INLINE CGRect
// seems not useful to inline to me, is fairly big (unless you can reschedule
// all the stores among the rest of the code, but still will probably increase
// code size in all cases)
function CGRectMake(x: CGFloat; y: CGFloat; width: CGFloat; height: CGFloat): CGRect;
{
  CGRect rect;
  rect.origin.x = x; rect.origin.y = y;
  rect.size.width = width; rect.size.height = height;
  return rect;
}

// CG_INLINE bool
function CGPointEqualToPoint(const point1: CGPoint; const point2: CGPoint): boolean; inline;
{
  return point1.x == point2.x && point1.y == point2.y;
}

=======
{
  CGPoint p; p.x = x; p.y = y; return p;
}

// CG_INLINE CGSize
function CGSizeMake(width: CGFloat; height: CGFloat): CGSize; inline;
{
  CGSize size; size.width = width; size.height = height; return size;
}

// CG_INLINE CGRect
// seems not useful to inline to me, is fairly big (unless you can reschedule
// all the stores among the rest of the code, but still will probably increase
// code size in all cases)
function CGRectMake(x: CGFloat; y: CGFloat; width: CGFloat; height: CGFloat): CGRect;
{
  CGRect rect;
  rect.origin.x = x; rect.origin.y = y;
  rect.size.width = width; rect.size.height = height;
  return rect;
}

// CG_INLINE bool
function CGPointEqualToPoint(const point1: CGPoint; const point2: CGPoint): boolean; inline;
{
  return point1.x == point2.x && point1.y == point2.y;
}

>>>>>>> graemeg/cpstrnew
=======
{
  CGRect rect;
  rect.origin.x = x; rect.origin.y = y;
  rect.size.width = width; rect.size.height = height;
  return rect;
}

// CG_INLINE bool
function CGPointEqualToPoint(const point1: CGPoint; const point2: CGPoint): boolean; inline;
{
  return point1.x == point2.x && point1.y == point2.y;
}

>>>>>>> graemeg/cpstrnew
// CG_INLINE bool
function CGSizeEqualToSize(size1: CGSize; size2: CGSize): boolean; inline;
{
  return size1.width == size2.width && size1.height == size2.height;
}

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
implementation

function CGPointMake(x: CGFloat; y: CGFloat): CGPoint; inline;
begin
  CGPointMake.x := x;
  CGPointMake.y := y;
end;


function CGSizeMake(width: CGFloat; height: CGFloat): CGSize; inline;
begin
  CGSizeMake.width := width;
  CGSizeMake.height := height;
end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function CGVectorMake(dx: CGFloat; dy: CGFloat): CGVector; inline;
begin
  CGVectorMake.dx := dx;
  CGVectorMake.dy := dy;
end;


=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function CGRectMake(x: CGFloat; y: CGFloat; width: CGFloat; height: CGFloat): CGRect;
begin
  CGRectMake.origin.x := x;
  CGRectMake.origin.y := y;
  CGRectMake.size.width := width;
  CGRectMake.size.height := height;
end;


function CGPointEqualToPoint(const point1: CGPoint; const point2: CGPoint): boolean; inline;
begin
  CGPointEqualToPoint:=
    (point1.x = point2.x) and
    (point1.y = point2.y);
end;


function CGSizeEqualToSize(size1: CGSize; size2: CGSize): boolean; inline;
begin
  CGSizeEqualToSize:=
    (size1.width = size2.width) and
    (size1.height = size2.height);
end;


end.

{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ Return the intersection of `r1' and `r2'. This may return a null rect. }

function CGRectIntersection( r1: CGRect; r2: CGRect ): CGRect; external name '_CGRectIntersection';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Offset `rect' by `(dx, dy)'. }

function CGRectOffset( rect: CGRect; dx: CGFloat; dy: CGFloat ): CGRect; external name '_CGRectOffset';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Make two new rectangles, `slice' and `remainder', by dividing `rect' with
   a line that's parallel to one of its sides, specified by `edge' -- either
   `CGRectMinXEdge', `CGRectMinYEdge', `CGRectMaxXEdge', or
   `CGRectMaxYEdge'. The size of `slice' is determined by `amount', which
   measures the distance from the specified edge. }

procedure CGRectDivide( rect: CGRect; var slice: CGRect; var remainder: CGRect; amount: CGFloat; edge: CGRectEdge ); external name '_CGRectDivide';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return true if `point' is contained in `rect', false otherwise. }

function CGRectContainsPoint( rect: CGRect; point: CGPoint ): CBool; external name '_CGRectContainsPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return true if `rect2' is contained in `rect1', false otherwise. `rect2'
   is contained in `rect1' if the union of `rect1' and `rect2' is equal to
   `rect1'. }

function CGRectContainsRect( rect1: CGRect; rect2: CGRect ): CBool; external name '_CGRectContainsRect';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return true if `rect1' intersects `rect2', false otherwise. `rect1'
   intersects `rect2' if the intersection of `rect1' and `rect2' is not the
   null rect. }

function CGRectIntersectsRect( rect1: CGRect; rect2: CGRect ): CBool; external name '_CGRectIntersectsRect';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{** Persistent representations. **}

{ Return a dictionary representation of `point'. }

function CGPointCreateDictionaryRepresentation( point: CGPoint ): CFDictionaryRef; external name '_CGPointCreateDictionaryRepresentation';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Make a CGPoint from the contents of `dict' (presumably returned earlier
   from `CGPointCreateDictionaryRepresentation') and store the value in
   `point'. Returns true on success; false otherwise. }

function CGPointMakeWithDictionaryRepresentation( dict: CFDictionaryRef; var point: { out } CGPoint ): CBool; external name '_CGPointMakeWithDictionaryRepresentation';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Return a dictionary representation of `size'. }

function CGSizeCreateDictionaryRepresentation( size: CGSize ): CFDictionaryRef; external name '_CGSizeCreateDictionaryRepresentation';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Make a CGSize from the contents of `dict' (presumably returned earlier
   from `CGSizeCreateDictionaryRepresentation') and store the value in
   `size'. Returns true on success; false otherwise. }

function CGSizeMakeWithDictionaryRepresentation( dict: CFDictionaryRef; var size: { out } CGSize ): CBool; external name '_CGSizeMakeWithDictionaryRepresentation';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Return a dictionary representation of `rect'. }

function CGRectCreateDictionaryRepresentation( rect: CGRect ): CFDictionaryRef; external name '_CGRectCreateDictionaryRepresentation';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Make a CGRect from the contents of `dict' (presumably returned earlier
   from `CGRectCreateDictionaryRepresentation') and store the value in
   `rect'. Returns true on success; false otherwise. }

function CGRectMakeWithDictionaryRepresentation( dict: CFDictionaryRef; var rect: { out } CGRect ): CBool; external name '_CGRectMakeWithDictionaryRepresentation';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)


{** Definitions of inline functions. **}
// CG_INLINE CGPoint
function CGPointMake(x: CGFloat; y: CGFloat): CGPoint; inline;

{
  CGPoint p; p.x = x; p.y = y; return p;
}

// CG_INLINE CGSize
function CGSizeMake(width: CGFloat; height: CGFloat): CGSize; inline;
{
  CGSize size; size.width = width; size.height = height; return size;
}

// CG_INLINE CGRect
// seems not useful to inline to me, is fairly big (unless you can reschedule
// all the stores among the rest of the code, but still will probably increase
// code size in all cases)
function CGRectMake(x: CGFloat; y: CGFloat; width: CGFloat; height: CGFloat): CGRect;
{
  CGRect rect;
  rect.origin.x = x; rect.origin.y = y;
  rect.size.width = width; rect.size.height = height;
  return rect;
}

// CG_INLINE bool
function CGPointEqualToPoint(const point1: CGPoint; const point2: CGPoint): boolean; inline;
{
  return point1.x == point2.x && point1.y == point2.y;
}

// CG_INLINE bool
function CGSizeEqualToSize(size1: CGSize; size2: CGSize): boolean; inline;
{
  return size1.width == size2.width && size1.height == size2.height;
}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
implementation

function CGPointMake(x: CGFloat; y: CGFloat): CGPoint; inline;
begin
  CGPointMake.x := x;
  CGPointMake.y := y;
end;


function CGSizeMake(width: CGFloat; height: CGFloat): CGSize; inline;
begin
  CGSizeMake.width := width;
  CGSizeMake.height := height;
end;


function CGRectMake(x: CGFloat; y: CGFloat; width: CGFloat; height: CGFloat): CGRect;
begin
  CGRectMake.origin.x := x;
  CGRectMake.origin.y := y;
  CGRectMake.size.width := width;
  CGRectMake.size.height := height;
end;


function CGPointEqualToPoint(const point1: CGPoint; const point2: CGPoint): boolean; inline;
begin
  CGPointEqualToPoint:=
    (point1.x = point2.x) and
    (point1.y = point2.y);
end;


function CGSizeEqualToSize(size1: CGSize; size2: CGSize): boolean; inline;
begin
  CGSizeEqualToSize:=
    (size1.width = size2.width) and
    (size1.height = size2.height);
end;


end.
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4
