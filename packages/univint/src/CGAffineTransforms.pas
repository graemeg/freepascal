{ CoreGraphics - CGAffineTransform.h
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   Copyright (c) 1998-2011 Apple Inc.
=======
 * Copyright (c) 1998-2009 Apple Inc.
>>>>>>> graemeg/cpstrnew
=======
 * Copyright (c) 1998-2009 Apple Inc.
>>>>>>> graemeg/cpstrnew
=======
 * Copyright (c) 1998-2009 Apple Inc.
>>>>>>> graemeg/cpstrnew
=======
 * Copyright (c) 1998-2009 Apple Inc.
>>>>>>> origin/cpstrnew
 * All rights reserved.
 }
{       Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, August 2015 }
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
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
 * Copyright (c) 1998-2000 Apple Computer, Inc.
=======
 * Copyright (c) 1998-2009 Apple Inc.
>>>>>>> origin/fixes_2.4
 * All rights reserved.
 }
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

unit CGAffineTransforms;
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
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
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
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
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
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
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
=======
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
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
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
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
=======
>>>>>>> origin/cpstrnew
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
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
=======
=======
>>>>>>> origin/fixes_2_2
=======
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
uses MacTypes,CGBase,CGGeometry;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not MACOSALLINCLUDE}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {not MACOSALLINCLUDE}

>>>>>>> origin/fixes_2.4
{$ALIGN POWER}


type
	CGAffineTransformPtr = ^CGAffineTransform;
	CGAffineTransform = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		a, b, c, d: CGFloat;
		tx, ty: CGFloat;
=======
		a, b, c, d: Float32;
		tx, ty: Float32;
>>>>>>> graemeg/fixes_2_2
=======
		a, b, c, d: Float32;
		tx, ty: Float32;
>>>>>>> origin/fixes_2_2
=======
		a, b, c, d: CGFloat;
		tx, ty: CGFloat;
>>>>>>> origin/fixes_2.4
	end;

{ The identity transform: [ 1 0 0 1 0 0 ]. }

var CGAffineTransformIdentity: CGAffineTransform; external name '_CGAffineTransformIdentity'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the transform [ a b c d tx ty ]. }

function CGAffineTransformMake( a: CGFloat; b: CGFloat; c: CGFloat; d: CGFloat; tx: CGFloat; ty: CGFloat ): CGAffineTransform; external name '_CGAffineTransformMake';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew

{ Return a transform which translates by `(tx, ty)':
     t' = [ 1 0 0 1 tx ty ] }

function CGAffineTransformMakeTranslation( tx: CGFloat; ty: CGFloat ): CGAffineTransform; external name '_CGAffineTransformMakeTranslation';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew

{ Return a transform which scales by `(sx, sy)':
     t' = [ sx 0 0 sy 0 0 ] }

function CGAffineTransformMakeScale( sx: CGFloat; sy: CGFloat ): CGAffineTransform; external name '_CGAffineTransformMakeScale';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew

{ Return a transform which rotates by `angle' radians:
     t' = [ cos(angle) sin(angle) -sin(angle) cos(angle) 0 0 ] }

function CGAffineTransformMakeRotation( angle: CGFloat ): CGAffineTransform; external name '_CGAffineTransformMakeRotation';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
=======
=======
>>>>>>> origin/fixes_2_2

{ Return the transform [ a b c d tx ty ]. }

<<<<<<< HEAD
function CGAffineTransformMake( a: Float32; b: Float32; c: Float32; d: Float32; tx: Float32; ty: Float32 ): CGAffineTransform; external name '_CGAffineTransformMake';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CGAffineTransformMake( a: CGFloat; b: CGFloat; c: CGFloat; d: CGFloat; tx: CGFloat; ty: CGFloat ): CGAffineTransform; external name '_CGAffineTransformMake';
>>>>>>> origin/fixes_2.4

{ Return a transform which translates by `(tx, ty)':
 *   t' = [ 1 0 0 1 tx ty ] }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function CGAffineTransformMakeTranslation( tx: CGFloat; ty: CGFloat ): CGAffineTransform; external name '_CGAffineTransformMakeTranslation';
=======
function CGAffineTransformMakeTranslation( tx: Float32; ty: Float32 ): CGAffineTransform; external name '_CGAffineTransformMakeTranslation';
>>>>>>> graemeg/fixes_2_2
=======
function CGAffineTransformMakeTranslation( tx: Float32; ty: Float32 ): CGAffineTransform; external name '_CGAffineTransformMakeTranslation';
>>>>>>> origin/fixes_2_2
=======
function CGAffineTransformMakeTranslation( tx: CGFloat; ty: CGFloat ): CGAffineTransform; external name '_CGAffineTransformMakeTranslation';
>>>>>>> origin/fixes_2.4

{ Return a transform which scales by `(sx, sy)':
 *   t' = [ sx 0 0 sy 0 0 ] }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function CGAffineTransformMakeScale( sx: CGFloat; sy: CGFloat ): CGAffineTransform; external name '_CGAffineTransformMakeScale';
=======
function CGAffineTransformMakeScale( sx: Float32; sy: Float32 ): CGAffineTransform; external name '_CGAffineTransformMakeScale';
>>>>>>> graemeg/fixes_2_2
=======
function CGAffineTransformMakeScale( sx: Float32; sy: Float32 ): CGAffineTransform; external name '_CGAffineTransformMakeScale';
>>>>>>> origin/fixes_2_2
=======
function CGAffineTransformMakeScale( sx: CGFloat; sy: CGFloat ): CGAffineTransform; external name '_CGAffineTransformMakeScale';
>>>>>>> origin/fixes_2.4

{ Return a transform which rotates by `angle' radians:
 *   t' = [ cos(angle) sin(angle) -sin(angle) cos(angle) 0 0 ] }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function CGAffineTransformMakeRotation( angle: CGFloat ): CGAffineTransform; external name '_CGAffineTransformMakeRotation';
>>>>>>> origin/cpstrnew

{ Return true if `t' is the identity transform, false otherwise. }

function CGAffineTransformIsIdentity( t: CGAffineTransform ): CBool; external name '_CGAffineTransformIsIdentity';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Translate `t' by `(tx, ty)' and return the result:
<<<<<<< HEAD
     t' = [ 1 0 0 1 tx ty ] * t }

function CGAffineTransformTranslate( t: CGAffineTransform; tx: CGFloat; ty: CGFloat ): CGAffineTransform; external name '_CGAffineTransformTranslate';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew

{ Scale `t' by `(sx, sy)' and return the result:
     t' = [ sx 0 0 sy 0 0 ] * t }

function CGAffineTransformScale( t: CGAffineTransform; sx: CGFloat; sy: CGFloat ): CGAffineTransform; external name '_CGAffineTransformScale';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew

{ Rotate `t' by `angle' radians and return the result:
     t' =  [ cos(angle) sin(angle) -sin(angle) cos(angle) 0 0 ] * t }

function CGAffineTransformRotate( t: CGAffineTransform; angle: CGFloat ): CGAffineTransform; external name '_CGAffineTransformRotate';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew

{ Invert `t' and return the result. If `t' has zero determinant, then `t'
   is returned unchanged. }
=======
 *   t' = [ 1 0 0 1 tx ty ] * t }

function CGAffineTransformTranslate( t: CGAffineTransform; tx: CGFloat; ty: CGFloat ): CGAffineTransform; external name '_CGAffineTransformTranslate';
=======
=======
>>>>>>> origin/fixes_2_2
function CGAffineTransformMakeRotation( angle: Float32 ): CGAffineTransform; external name '_CGAffineTransformMakeRotation';
=======
function CGAffineTransformMakeRotation( angle: CGFloat ): CGAffineTransform; external name '_CGAffineTransformMakeRotation';
>>>>>>> origin/fixes_2.4

{ Return true if `t' is the identity transform, false otherwise. }

function CGAffineTransformIsIdentity( t: CGAffineTransform ): CBool; external name '_CGAffineTransformIsIdentity'; (* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{ Translate `t' by `(tx, ty)' and return the result:
 *   t' = [ 1 0 0 1 tx ty ] * t }

<<<<<<< HEAD
function CGAffineTransformTranslate( t: CGAffineTransform; tx: Float32; ty: Float32 ): CGAffineTransform; external name '_CGAffineTransformTranslate';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CGAffineTransformTranslate( t: CGAffineTransform; tx: CGFloat; ty: CGFloat ): CGAffineTransform; external name '_CGAffineTransformTranslate';
>>>>>>> origin/fixes_2.4

{ Scale `t' by `(sx, sy)' and return the result:
 *   t' = [ sx 0 0 sy 0 0 ] * t }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function CGAffineTransformScale( t: CGAffineTransform; sx: CGFloat; sy: CGFloat ): CGAffineTransform; external name '_CGAffineTransformScale';
=======
function CGAffineTransformScale( t: CGAffineTransform; sx: Float32; sy: Float32 ): CGAffineTransform; external name '_CGAffineTransformScale';
>>>>>>> graemeg/fixes_2_2
=======
function CGAffineTransformScale( t: CGAffineTransform; sx: Float32; sy: Float32 ): CGAffineTransform; external name '_CGAffineTransformScale';
>>>>>>> origin/fixes_2_2
=======
function CGAffineTransformScale( t: CGAffineTransform; sx: CGFloat; sy: CGFloat ): CGAffineTransform; external name '_CGAffineTransformScale';
>>>>>>> origin/fixes_2.4

{ Rotate `t' by `angle' radians and return the result:
 *   t' =  [ cos(angle) sin(angle) -sin(angle) cos(angle) 0 0 ] * t }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function CGAffineTransformRotate( t: CGAffineTransform; angle: CGFloat ): CGAffineTransform; external name '_CGAffineTransformRotate';

{ Invert `t' and return the result.  If `t' has zero determinant, then `t'
 * is returned unchanged. }
>>>>>>> origin/cpstrnew

function CGAffineTransformInvert( t: CGAffineTransform ): CGAffineTransform; external name '_CGAffineTransformInvert';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Concatenate `t2' to `t1' and return the result:
     t' = t1 * t2 }

function CGAffineTransformConcat( t1: CGAffineTransform; t2: CGAffineTransform ): CGAffineTransform; external name '_CGAffineTransformConcat';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return true if `t1' and `t2' are equal, false otherwise. }

function CGAffineTransformEqualToTransform( t1: CGAffineTransform; t2: CGAffineTransform ): CBool; external name '_CGAffineTransformEqualToTransform';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Transform `point' by `t' and return the result:
     p' = p * t
   where p = [ x y 1 ]. }

function CGPointApplyAffineTransform( point: CGPoint; t: CGAffineTransform ): CGPoint; external name '_CGPointApplyAffineTransform';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Transform `size' by `t' and return the result:
     s' = s * t
   where s = [ width height 0 ]. }

function CGSizeApplyAffineTransform( size: CGSize; t: CGAffineTransform ): CGSize; external name '_CGSizeApplyAffineTransform';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Transform `rect' by `t' and return the result. Since affine transforms do
   not preserve rectangles in general, this function returns the smallest
   rectangle which contains the transformed corner points of `rect'. If `t'
   consists solely of scales, flips and translations, then the returned
   rectangle coincides with the rectangle constructed from the four
   transformed corners. }

function CGRectApplyAffineTransform( rect: CGRect; t: CGAffineTransform ): CGRect; external name '_CGRectApplyAffineTransform';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
function CGAffineTransformRotate( t: CGAffineTransform; angle: Float32 ): CGAffineTransform; external name '_CGAffineTransformRotate';
=======
function CGAffineTransformRotate( t: CGAffineTransform; angle: CGFloat ): CGAffineTransform; external name '_CGAffineTransformRotate';
>>>>>>> origin/fixes_2.4

{ Invert `t' and return the result.  If `t' has zero determinant, then `t'
 * is returned unchanged. }

function CGAffineTransformInvert( t: CGAffineTransform ): CGAffineTransform; external name '_CGAffineTransformInvert';

{ Concatenate `t2' to `t1' and return the result:
 *   t' = t1 * t2 }

function CGAffineTransformConcat( t1: CGAffineTransform; t2: CGAffineTransform ): CGAffineTransform; external name '_CGAffineTransformConcat';

{ Return true if `t1' and `t2' are equal, false otherwise. }

function CGAffineTransformEqualToTransform( t1: CGAffineTransform; t2: CGAffineTransform ): CBool; external name '_CGAffineTransformEqualToTransform'; (* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{ Transform `point' by `t' and return the result:
 *   p' = p * t
 * where p = [ x y 1 ]. }

function CGPointApplyAffineTransform( point: CGPoint; t: CGAffineTransform ): CGPoint; external name '_CGPointApplyAffineTransform';

{ Transform `size' by `t' and return the result:
 *   s' = s * t
 * where s = [ width height 0 ]. }

function CGSizeApplyAffineTransform( size: CGSize; t: CGAffineTransform ): CGSize; external name '_CGSizeApplyAffineTransform';

{ Transform `rect' by `t' and return the result. Since affine transforms
 * do not preserve rectangles in general, this function returns the
 * smallest rectangle which contains the transformed corner points of
 * `rect'. If `t' consists solely of scales, flips and translations, then
 * the returned rectangle coincides with the rectangle constructed from the
 * four transformed corners. }

function CGRectApplyAffineTransform( rect: CGRect; t: CGAffineTransform ): CGRect; external name '_CGRectApplyAffineTransform'; (* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

(*
{** Definitions of inline functions. **}

CG_INLINE CGAffineTransform
__CGAffineTransformMake(float a, float b, float c, float d, float tx, float ty)
{
    CGAffineTransform t;

    t.a = a; t.b = b; t.c = c; t.d = d; t.tx = tx; t.ty = ty;
    return t;
}

#define CGAffineTransformMake __CGAffineTransformMake

CG_INLINE CGPoint
__CGPointApplyAffineTransform(CGPoint point, CGAffineTransform t)
{
    CGPoint p;

    p.x = t.a * point.x + t.c * point.y + t.tx;
    p.y = t.b * point.x + t.d * point.y + t.ty;
    return p;
}

#define CGPointApplyAffineTransform __CGPointApplyAffineTransform

CG_INLINE CGSize
__CGSizeApplyAffineTransform(CGSize size, CGAffineTransform t)
{
    CGSize s;

    s.width = t.a * size.width + t.c * size.height;
    s.height = t.b * size.width + t.d * size.height;
    return s;
}

#define CGSizeApplyAffineTransform __CGSizeApplyAffineTransform
*)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4
