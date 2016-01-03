{ CoreGraphics - CGPath.h
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   Copyright (c) 2001-2011 Apple Inc.
   All rights reserved. }
{       Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, August 2015 }
=======
 * Copyright (c) 2001-2008 Apple Inc.
 * All rights reserved. }
{       Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
 * Copyright (c) 2001-2008 Apple Inc.
 * All rights reserved. }
{       Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
 * Copyright (c) 2001-2008 Apple Inc.
 * All rights reserved. }
{       Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
 * Copyright (c) 2001-2008 Apple Inc.
 * All rights reserved. }
{       Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
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
 * Copyright (c) 2001-2002 Apple Computer, Inc.
 * All rights reserved.
 }
=======
 * Copyright (c) 2001-2008 Apple Inc.
 * All rights reserved. }
>>>>>>> origin/fixes_2.4
{       Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
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

unit CGPath;
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
>>>>>>> graemeg/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
=======
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
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
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
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
>>>>>>> origin/cpstrnew
{$ifc defined(iphonesim)}
 	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := TRUE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/cpstrnew
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
<<<<<<< HEAD
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
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_CPU_ARM64 := TRUE}
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
	{$setc TARGET_OS_EMBEDDED := TRUE}
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ nor __arm64__ is defined.}
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
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
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
uses MacTypes,CGBase,CGAffineTransforms,CFBase,CGGeometry;
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CGMutablePathRef = ^OpaqueCGMutablePathRef; { an opaque type }
	OpaqueCGMutablePathRef = record end;
type
	CGPathRef = ^OpaqueCGPathRef; { an opaque type }
	OpaqueCGPathRef = record end;

=======
	CGMutablePathRef = ^SInt32; { an opaque type }
type
	CGPathRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew

{ Line join styles. }

type
	CGLineJoin = SInt32;
const
	kCGLineJoinMiter = 0;
	kCGLineJoinRound = 1;
	kCGLineJoinBevel = 2;

{ Line cap styles. }
=======
	CGMutablePathRef = ^SInt32; { an opaque type }
type
	CGPathRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CGMutablePathRef = ^SInt32; { an opaque type }
type
	CGPathRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CGMutablePathRef = ^SInt32; { an opaque type }
type
	CGPathRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew

type
	CGLineCap = SInt32;
const
	kCGLineCapButt = 0;
	kCGLineCapRound = 1;
	kCGLineCapSquare = 2;

{ Return the CFTypeID for CGPathRefs. }

function CGPathGetTypeID: CFTypeID; external name '_CGPathGetTypeID';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Create a mutable path. }

function CGPathCreateMutable: CGMutablePathRef; external name '_CGPathCreateMutable';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Create a copy of `path'. }

function CGPathCreateCopy( path: CGPathRef ): CGPathRef; external name '_CGPathCreateCopy';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{ Create a copy of `path' transformed by `transform'. }

function CGPathCreateCopyByTransformingPath( path: CGPathRef; const (*var*) transform: CGAffineTransform ): CGPathRef; external name '_CGPathCreateCopyByTransformingPath';
(* CG_AVAILABLE_STARTING(__MAC_10_7, __IPHONE_5_0) *)
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{ Create a mutable copy of `path'. }

function CGPathCreateMutableCopy( path: CGPathRef ): CGMutablePathRef; external name '_CGPathCreateMutableCopy';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{ Create a mutable copy of `path' transformed by `transform'. }

function CGPathCreateMutableCopyByTransformingPath( path: CGPathRef; const (*var*) transform: CGAffineTransform ): CGMutablePathRef; external name '_CGPathCreateMutableCopyByTransformingPath';
(* CG_AVAILABLE_STARTING(__MAC_10_7, __IPHONE_5_0) *)

{ Return a path representing a rectangle bounded by `rect'. The rectangle
   forms a complete subpath of the path --- that is, it begins with a "move
   to" and ends with a "close subpath" --- oriented in the clockwise
   direction. If `transform' is non-NULL, then the lines representing the
   rectangle will be transformed by `transform' before they are added to the
   path. }

function CGPathCreateWithRect( rect: CGRect; transform: {const} CGAffineTransformPtr ): CGPathRef; external name '_CGPathCreateWithRect';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_4_0) *)

{ Return a path representing an ellipse bounded by `rect'. The ellipse is
   approximated by a sequence of BÃ©zier curves. The center of the ellipse is
   the midpoint of `rect'. If `rect' is square, then the ellipse will be
   circular with radius equal to one-half the width (equivalently, one-half
   the height) of `rect'. If `rect' is rectangular, then the major- and
   minor-axes will be the `width' and `height' of rect. The ellipse forms a
   complete subpath of the path --- that is, it begins with a "move to" and
   ends with a "close subpath" --- oriented in the clockwise direction. If
   `transform' is non-NULL, then the constructed BÃ©zier curves representing
   the ellipse will be transformed by `transform' before they are added to
   the path. }

function CGPathCreateWithEllipseInRect( rect: CGRect; transform: {const} CGAffineTransformPtr): CGPathRef; external name '_CGPathCreateWithEllipseInRect';
(* CG_AVAILABLE_STARTING(__MAC_10_7, __IPHONE_5_0) *)

{ Return a path representing a rounded rectangle. The rounded rectangle
   coincides with the edges of `rect'. Each corner is consists of
   one-quarter of an ellipse with axes equal to `cornerWidth' and
   `cornerHeight'. The rounded rectangle forms a complete subpath of the
   path --- that is, it begins with a "move to" and ends with a "close
   subpath" --- oriented in the clockwise direction. If `transform' is
   non-NULL, then the path elements representing the rounded rectangle will
   be transformed by `transform' before they are added to the path. }

function CGPathCreateWithRoundedRect( rect: CGRect; cornerWidth: CGFloat; cornerHeight: CGFloat; transform: {const} CGAffineTransformPtr ): CGPathRef; external name '_CGPathCreateWithRoundedRect';
(* CG_AVAILABLE_STARTING(__MAC_10_9, __IPHONE_7_0) *)

{ Add a rounded rectangle to `path'. The rounded rectangle coincides with
   the edges of `rect'. Each corner is consists of one-quarter of an ellipse
   with axes equal to `cornerWidth' and `cornerHeight'. The rounded
   rectangle forms a complete subpath of the path --- that is, it begins
   with a "move to" and ends with a "close subpath" --- oriented in the
   clockwise direction. If `transform' is non-NULL, then the path elements
   representing the rounded rectangle will be transformed by `transform'
   before they are added to the path. }

procedure CGPathAddRoundedRect( path: CGMutablePathRef; transform: {const} CGAffineTransformPtr; rect: CGRect; cornerWidth: CGFloat; cornerHeight: CGFloat ); external name '_CGPathAddRoundedRect';
(* CG_AVAILABLE_STARTING(__MAC_10_9, __IPHONE_7_0) *)

{ Create a dashed path from `path'. The parameters `phase', `lengths', and
   `count' have the same meaning as the corresponding parameters for
   `CGContextSetLineDash'. If `transform' is non-NULL, then the elements of
   the constructed path will be transformed by `transform' before they are
   added to the path. }

function CGPathCreateCopyByDashingPath( path: CGPathRef; transform: {const} CGAffineTransformPtr; phase: CGFloat; {const} lengths: {variable-size-array} CGFloatPtr; count: size_t ): CGPathRef; external name '_CGPathCreateCopyByDashingPath';
(* CG_AVAILABLE_STARTING(__MAC_10_7, __IPHONE_5_0) *)

{ Create a stroked path from `path'. The parameters `lineWidth', `lineCap',
   `lineJoin', and `miterLimit' have the same meaning as the corresponding
   CGContext parameters. If `transform' is non-NULL, then the elements of
   the constructed path will be transformed by `transform' before they are
   added to the path. }

function CGPathCreateCopyByStrokingPath( path: CGPathRef; transform: {const} CGAffineTransformPtr; lineWidth: CGFloat; lineCap: CGLineCap; lineJoin: CGLineJoin; miterLimit: CGFloat ): CGPathRef; external name '_CGPathCreateCopyByStrokingPath';
(* CG_AVAILABLE_STARTING(__MAC_10_7, __IPHONE_5_0) *)
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{ Equivalent to `CFRetain(path)', except it doesn't crash (as CFRetain
   does) if `path' is NULL. }

function CGPathRetain( path: CGPathRef ): CGPathRef; external name '_CGPathRetain';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Equivalent to `CFRelease(path)', except it doesn't crash (as CFRelease
   does) if `path' is NULL. }

procedure CGPathRelease( path: CGPathRef ); external name '_CGPathRelease';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Return true if `path1' is equal to `path2'; false otherwise. }

function CGPathEqualToPath( path1: CGPathRef; path2: CGPathRef ): CBool; external name '_CGPathEqualToPath';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{** Path construction functions. **}

{ Move the current point to `(x, y)' in `path' and begin a new subpath. If
   `m' is non-NULL, then transform `(x, y)' by `m' first. }

procedure CGPathMoveToPoint( path: CGMutablePathRef; m: CGAffineTransformPtr; x: CGFloat; y: CGFloat ); external name '_CGPathMoveToPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Append a straight line segment from the current point to `(x, y)' in
   `path' and move the current point to `(x, y)'. If `m' is non-NULL, then
   transform `(x, y)' by `m' first. }

procedure CGPathAddLineToPoint( path: CGMutablePathRef; m: CGAffineTransformPtr; x: CGFloat; y: CGFloat ); external name '_CGPathAddLineToPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Append a quadratic curve from the current point to `(x, y)' with control
   point `(cpx, cpy)' in `path' and move the current point to `(x, y)'. If
   `m' is non-NULL, then transform all points by `m' first. }

procedure CGPathAddQuadCurveToPoint( path: CGMutablePathRef; m: CGAffineTransformPtr; cpx: CGFloat; cpy: CGFloat; x: CGFloat; y: CGFloat ); external name '_CGPathAddQuadCurveToPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ Append a cubic BÃ©zier curve from the current point to `(x,y)' with
=======
{ Append a cubic BŽzier curve from the current point to `(x,y)' with
>>>>>>> graemeg/cpstrnew
=======
{ Append a cubic BŽzier curve from the current point to `(x,y)' with
>>>>>>> graemeg/cpstrnew
=======
{ Append a cubic BŽzier curve from the current point to `(x,y)' with
>>>>>>> graemeg/cpstrnew
=======
{ Append a cubic BŽzier curve from the current point to `(x,y)' with
>>>>>>> origin/cpstrnew
   control points `(cp1x, cp1y)' and `(cp2x, cp2y)' in `path' and move the
   current point to `(x, y)'. If `m' is non-NULL, then transform all points
   by `m' first. }

procedure CGPathAddCurveToPoint( path: CGMutablePathRef; m: CGAffineTransformPtr; cp1x: CGFloat; cp1y: CGFloat; cp2x: CGFloat; cp2y: CGFloat; x: CGFloat; y: CGFloat ); external name '_CGPathAddCurveToPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Append a line from the current point to the starting point of the current
   subpath of `path' and end the subpath. }

procedure CGPathCloseSubpath( path: CGMutablePathRef ); external name '_CGPathCloseSubpath';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
	CGMutablePathRef = ^SInt32; { an opaque 32-bit type }
=======
	CGMutablePathRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4
type
	CGPathRef = ^SInt32; { an opaque type }


{ Return the CFTypeID for CGPathRefs. }

function CGPathGetTypeID: CFTypeID; external name '_CGPathGetTypeID';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Create a mutable path. }

function CGPathCreateMutable: CGMutablePathRef; external name '_CGPathCreateMutable';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Create a copy of `path'. }

function CGPathCreateCopy( path: CGPathRef ): CGPathRef; external name '_CGPathCreateCopy';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Create a mutable copy of `path'. }

function CGPathCreateMutableCopy( path: CGPathRef ): CGMutablePathRef; external name '_CGPathCreateMutableCopy';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Equivalent to `CFRetain(path)', except it doesn't crash (as CFRetain
   does) if `path' is NULL. }

function CGPathRetain( path: CGPathRef ): CGPathRef; external name '_CGPathRetain';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Equivalent to `CFRelease(path)', except it doesn't crash (as CFRelease
   does) if `path' is NULL. }

procedure CGPathRelease( path: CGPathRef ); external name '_CGPathRelease';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Return true if `path1' is equal to `path2'; false otherwise. }

function CGPathEqualToPath( path1: CGPathRef; path2: CGPathRef ): CBool; external name '_CGPathEqualToPath';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{** Path construction functions. **}

{ Move the current point to `(x, y)' in `path' and begin a new subpath. If
   `m' is non-NULL, then transform `(x, y)' by `m' first. }

procedure CGPathMoveToPoint( path: CGMutablePathRef; const (*var*) m: CGAffineTransform; x: CGFloat; y: CGFloat ); external name '_CGPathMoveToPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Append a straight line segment from the current point to `(x, y)' in
   `path' and move the current point to `(x, y)'. If `m' is non-NULL, then
   transform `(x, y)' by `m' first. }

procedure CGPathAddLineToPoint( path: CGMutablePathRef; const (*var*) m: CGAffineTransform; x: CGFloat; y: CGFloat ); external name '_CGPathAddLineToPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Append a quadratic curve from the current point to `(x, y)' with control
   point `(cpx, cpy)' in `path' and move the current point to `(x, y)'. If
   `m' is non-NULL, then transform all points by `m' first. }

procedure CGPathAddQuadCurveToPoint( path: CGMutablePathRef; const (*var*) m: CGAffineTransform; cpx: CGFloat; cpy: CGFloat; x: CGFloat; y: CGFloat ); external name '_CGPathAddQuadCurveToPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Append a cubic BŽzier curve from the current point to `(x,y)' with
   control points `(cp1x, cp1y)' and `(cp2x, cp2y)' in `path' and move the
   current point to `(x, y)'. If `m' is non-NULL, then transform all points
   by `m' first. }

procedure CGPathAddCurveToPoint( path: CGMutablePathRef; const (*var*) m: CGAffineTransform; cp1x: CGFloat; cp1y: CGFloat; cp2x: CGFloat; cp2y: CGFloat; x: CGFloat; y: CGFloat ); external name '_CGPathAddCurveToPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Append a line from the current point to the starting point of the current
   subpath of `path' and end the subpath. }

<<<<<<< HEAD
procedure CGPathCloseSubpath( path: CGMutablePathRef ); external name '_CGPathCloseSubpath'; (* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure CGPathCloseSubpath( path: CGMutablePathRef ); external name '_CGPathCloseSubpath';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{** Path construction convenience functions. **}

{ Add `rect' to `path'. If `m' is non-NULL, then first transform `rect' by
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   `m' before adding it to `path'. }

procedure CGPathAddRect( path: CGMutablePathRef; m: CGAffineTransformPtr; rect: CGRect ); external name '_CGPathAddRect';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Add each rectangle specified by `rects', an array of `count' CGRects, to
   `path'. If `m' is non-NULL, then first transform each rectangle by `m'
   before adding it to `path'. }

procedure CGPathAddRects( path: CGMutablePathRef; m: CGAffineTransformPtr; {const} rects: {variable-size-array} CGRectPtr; count: size_t ); external name '_CGPathAddRects';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Move to the first element of `points', an array of `count' CGPoints, and
   append a line from each point to the next point in `points'. If `m' is
   non-NULL, then first transform each point by `m'. }

procedure CGPathAddLines( path: CGMutablePathRef; m: CGAffineTransformPtr; {const} points: {variable-size-array} CGPointPtr; count: size_t ); external name '_CGPathAddLines';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Add an ellipse (an oval) inside `rect' to `path'. The ellipse is
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   approximated by a sequence of BÃ©zier curves. The center of the ellipse is
=======
   approximated by a sequence of BŽzier curves. The center of the ellipse is
>>>>>>> graemeg/cpstrnew
=======
   approximated by a sequence of BŽzier curves. The center of the ellipse is
>>>>>>> graemeg/cpstrnew
=======
   approximated by a sequence of BŽzier curves. The center of the ellipse is
>>>>>>> graemeg/cpstrnew
=======
   approximated by a sequence of BŽzier curves. The center of the ellipse is
>>>>>>> origin/cpstrnew
   the midpoint of `rect'. If `rect' is square, then the ellipse will be
   circular with radius equal to one-half the width (equivalently, one-half
   the height) of `rect'. If `rect' is rectangular, then the major- and
   minor-axes will be the `width' and `height' of rect. The ellipse forms a
   complete subpath of `path' --- that is, it begins with a "move to" and
   ends with a "close subpath" --- oriented in the clockwise direction. If
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   `m' is non-NULL, then the constructed BÃ©zier curves representing the
=======
   `m' is non-NULL, then the constructed BŽzier curves representing the
>>>>>>> graemeg/cpstrnew
=======
   `m' is non-NULL, then the constructed BŽzier curves representing the
>>>>>>> graemeg/cpstrnew
=======
   `m' is non-NULL, then the constructed BŽzier curves representing the
>>>>>>> graemeg/cpstrnew
=======
   `m' is non-NULL, then the constructed BŽzier curves representing the
>>>>>>> origin/cpstrnew
   ellipse will be transformed by `m' before they are added to `path'. }

procedure CGPathAddEllipseInRect( path: CGMutablePathRef; m: CGAffineTransformPtr; rect: CGRect ); external name '_CGPathAddEllipseInRect';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)
<<<<<<< HEAD
<<<<<<< HEAD

{ Add an arc of a circle to `path', possibly preceded by a straight line
<<<<<<< HEAD
<<<<<<< HEAD
   segment. The arc is approximated by a sequence of BÃ©zier curves. The
   center of the arc is `(x,y)'; `radius' is its radius. `startAngle' is the
   angle to the first endpoint of the arc, measured counter-clockwise from
   the positive x-axis. `startAngle + delta' is the angle to the second
   endpoint of the arc. If `delta' is positive, then the arc is drawn
   counter-clockwise; if negative, clockwise. `startAngle' and `delta' are
   measured in radians. If `matrix' is non-NULL, then the constructed BÃ©zier
   curves representing the arc will be transformed by `matrix' before they
   are added to the path. }

procedure CGPathAddRelativeArc( path: CGMutablePathRef; matrix: {const} CGAffineTransformPtr; x: CGFloat; y: CGFloat; radius: CGFloat; startAngle: CGFloat; delta: CGFloat ); external name '_CGPathAddRelativeArc';
(* CG_AVAILABLE_STARTING(__MAC_10_7, __IPHONE_5_0) *)

{ Add an arc of a circle to `path', possibly preceded by a straight line
   segment. The arc is approximated by a sequence of BÃ©zier curves. `(x, y)'
=======
   segment. The arc is approximated by a sequence of BŽzier curves. `(x, y)'
>>>>>>> graemeg/cpstrnew
=======

{ Add an arc of a circle to `path', possibly preceded by a straight line
   segment. The arc is approximated by a sequence of BŽzier curves. `(x, y)'
>>>>>>> origin/cpstrnew
   is the center of the arc; `radius' is its radius; `startAngle' is the
   angle to the first endpoint of the arc; `endAngle' is the angle to the
   second endpoint of the arc; and `clockwise' is true if the arc is to be
   drawn clockwise, false otherwise. `startAngle' and `endAngle' are
<<<<<<< HEAD
<<<<<<< HEAD
   measured in radians. If `m' is non-NULL, then the constructed BÃ©zier
   curves representing the arc will be transformed by `m' before they are
   added to `path'.

   Note that using values very near 2Ï€ can be problematic. For example,
   setting `startAngle' to 0, `endAngle' to 2Ï€, and `clockwise' to true will
   draw nothing. (It's easy to see this by considering, instead of 0 and 2Ï€,
   the values Îµ and 2Ï€ - Îµ, where Îµ is very small.) Due to round-off error,
   however, it's possible that passing the value `2 * M_PI' to approximate
   2Ï€ will numerically equal to 2Ï€ + Î´, for some small Î´; this will cause a
   full circle to be drawn.

   If you want a full circle to be drawn clockwise, you should set
   `startAngle' to 2Ï€, `endAngle' to 0, and `clockwise' to true. This avoids
   the instability problems discussed above. }
=======
   measured in radians. If `m' is non-NULL, then the constructed BŽzier
   curves representing the arc will be transformed by `m' before they are
   added to `path'. }
>>>>>>> graemeg/cpstrnew

procedure CGPathAddArc( path: CGMutablePathRef; m: CGAffineTransformPtr; x: CGFloat; y: CGFloat; radius: CGFloat; startAngle: CGFloat; endAngle: CGFloat; clockwise: CBool ); external name '_CGPathAddArc';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Add an arc of a circle to `path', possibly preceded by a straight line
<<<<<<< HEAD
   segment. The arc is approximated by a sequence of BÃ©zier curves. `radius'
   is the radius of the arc. The resulting arc is tangent to the line from
   the current point of `path' to `(x1, y1)', and the line from `(x1, y1)'
   to `(x2, y2)'. If `m' is non-NULL, then the constructed BÃ©zier curves
=======
   segment. The arc is approximated by a sequence of BŽzier curves. `radius'
   is the radius of the arc. The resulting arc is tangent to the line from
   the current point of `path' to `(x1, y1)', and the line from `(x1, y1)'
   to `(x2, y2)'. If `m' is non-NULL, then the constructed BŽzier curves
>>>>>>> graemeg/cpstrnew
   representing the arc will be transformed by `m' before they are added to
   `path'. }

=======
=======

{ Add an arc of a circle to `path', possibly preceded by a straight line
>>>>>>> graemeg/cpstrnew
   segment. The arc is approximated by a sequence of BŽzier curves. `(x, y)'
   is the center of the arc; `radius' is its radius; `startAngle' is the
   angle to the first endpoint of the arc; `endAngle' is the angle to the
   second endpoint of the arc; and `clockwise' is true if the arc is to be
   drawn clockwise, false otherwise. `startAngle' and `endAngle' are
   measured in radians. If `m' is non-NULL, then the constructed BŽzier
   curves representing the arc will be transformed by `m' before they are
   added to `path'. }

procedure CGPathAddArc( path: CGMutablePathRef; m: CGAffineTransformPtr; x: CGFloat; y: CGFloat; radius: CGFloat; startAngle: CGFloat; endAngle: CGFloat; clockwise: CBool ); external name '_CGPathAddArc';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Add an arc of a circle to `path', possibly preceded by a straight line
   segment. The arc is approximated by a sequence of BŽzier curves. `radius'
   is the radius of the arc. The resulting arc is tangent to the line from
   the current point of `path' to `(x1, y1)', and the line from `(x1, y1)'
   to `(x2, y2)'. If `m' is non-NULL, then the constructed BŽzier curves
   representing the arc will be transformed by `m' before they are added to
   `path'. }

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
   measured in radians. If `m' is non-NULL, then the constructed BŽzier
   curves representing the arc will be transformed by `m' before they are
   added to `path'. }

procedure CGPathAddArc( path: CGMutablePathRef; m: CGAffineTransformPtr; x: CGFloat; y: CGFloat; radius: CGFloat; startAngle: CGFloat; endAngle: CGFloat; clockwise: CBool ); external name '_CGPathAddArc';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Add an arc of a circle to `path', possibly preceded by a straight line
   segment. The arc is approximated by a sequence of BŽzier curves. `radius'
   is the radius of the arc. The resulting arc is tangent to the line from
   the current point of `path' to `(x1, y1)', and the line from `(x1, y1)'
   to `(x2, y2)'. If `m' is non-NULL, then the constructed BŽzier curves
   representing the arc will be transformed by `m' before they are added to
   `path'. }

>>>>>>> origin/cpstrnew
procedure CGPathAddArcToPoint( path: CGMutablePathRef; m: CGAffineTransformPtr; x1: CGFloat; y1: CGFloat; x2: CGFloat; y2: CGFloat; radius: CGFloat ); external name '_CGPathAddArcToPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Add `path2' to `path1'. If `m' is non-NULL, then the points in `path2'
   will be transformed by `m' before they are added to `path1'. }

procedure CGPathAddPath( path1: CGMutablePathRef; m: CGAffineTransformPtr; path2: CGPathRef ); external name '_CGPathAddPath';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
 * `m' before adding it to `path'. }
=======
   `m' before adding it to `path'. }
>>>>>>> origin/fixes_2.4

procedure CGPathAddRect( path: CGMutablePathRef; const (*var*) m: CGAffineTransform; rect: CGRect ); external name '_CGPathAddRect';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Add each rectangle specified by `rects', an array of `count' CGRects, to
   `path'. If `m' is non-NULL, then first transform each rectangle by `m'
   before adding it to `path'. }

procedure CGPathAddRects( path: CGMutablePathRef; const (*var*) m: CGAffineTransform; {const} rects: {variable-size-array} CGRectPtr; count: size_t ); external name '_CGPathAddRects';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Move to the first element of `points', an array of `count' CGPoints, and
   append a line from each point to the next point in `points'. If `m' is
   non-NULL, then first transform each point by `m'. }

procedure CGPathAddLines( path: CGMutablePathRef; const (*var*) m: CGAffineTransform; {const} points: {variable-size-array} CGPointPtr; count: size_t ); external name '_CGPathAddLines';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Add an ellipse (an oval) inside `rect' to `path'. The ellipse is
   approximated by a sequence of BŽzier curves. The center of the ellipse is
   the midpoint of `rect'. If `rect' is square, then the ellipse will be
   circular with radius equal to one-half the width (equivalently, one-half
   the height) of `rect'. If `rect' is rectangular, then the major- and
   minor-axes will be the `width' and `height' of rect. The ellipse forms a
   complete subpath of `path' --- that is, it begins with a "move to" and
   ends with a "close subpath" --- oriented in the clockwise direction. If
   `m' is non-NULL, then the constructed BŽzier curves representing the
   ellipse will be transformed by `m' before they are added to `path'. }

procedure CGPathAddEllipseInRect( path: CGMutablePathRef; const (*var*) m: CGAffineTransform; rect: CGRect ); external name '_CGPathAddEllipseInRect';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Add an arc of a circle to `path', possibly preceded by a straight line
   segment. The arc is approximated by a sequence of BŽzier curves. `(x, y)'
   is the center of the arc; `radius' is its radius; `startAngle' is the
   angle to the first endpoint of the arc; `endAngle' is the angle to the
   second endpoint of the arc; and `clockwise' is true if the arc is to be
   drawn clockwise, false otherwise. `startAngle' and `endAngle' are
   measured in radians. If `m' is non-NULL, then the constructed BŽzier
   curves representing the arc will be transformed by `m' before they are
   added to `path'. }

procedure CGPathAddArc( path: CGMutablePathRef; const (*var*) m: CGAffineTransform; x: CGFloat; y: CGFloat; radius: CGFloat; startAngle: CGFloat; endAngle: CGFloat; clockwise: CBool ); external name '_CGPathAddArc';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Add an arc of a circle to `path', possibly preceded by a straight line
   segment. The arc is approximated by a sequence of BŽzier curves. `radius'
   is the radius of the arc. The resulting arc is tangent to the line from
   the current point of `path' to `(x1, y1)', and the line from `(x1, y1)'
   to `(x2, y2)'. If `m' is non-NULL, then the constructed BŽzier curves
   representing the arc will be transformed by `m' before they are added to
   `path'. }

procedure CGPathAddArcToPoint( path: CGMutablePathRef; const (*var*) m: CGAffineTransform; x1: CGFloat; y1: CGFloat; x2: CGFloat; y2: CGFloat; radius: CGFloat ); external name '_CGPathAddArcToPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Add `path2' to `path1'. If `m' is non-NULL, then the points in `path2'
   will be transformed by `m' before they are added to `path1'. }

<<<<<<< HEAD
procedure CGPathAddPath( path1: CGMutablePathRef; const (*var*) m: CGAffineTransform; path2: CGPathRef ); external name '_CGPathAddPath'; (* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure CGPathAddPath( path1: CGMutablePathRef; const (*var*) m: CGAffineTransform; path2: CGPathRef ); external name '_CGPathAddPath';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{** Path information functions. **}

{ Return true if `path' contains no elements, false otherwise. }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function CGPathIsEmpty( path: CGPathRef ): CBool; external name '_CGPathIsEmpty';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Return true if `path' represents a rectangle, false otherwise. }

function CGPathIsRect( path: CGPathRef; var rect: CGRect ): CBool; external name '_CGPathIsRect';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Return the current point of the current subpath of `path'. If there is no
   current point, then return CGPointZero. }
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

function CGPathGetCurrentPoint( path: CGPathRef ): CGPoint; external name '_CGPathGetCurrentPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Return the bounding box of `path'. The bounding box is the smallest
   rectangle completely enclosing all points in the path, including control
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   points for BÃ©zier cubic and quadratic curves. If the path is empty, then
   return `CGRectNull'. }

function CGPathGetBoundingBox( path: CGPathRef ): CGRect; external name '_CGPathGetBoundingBox';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Return the path bounding box of `path'. The path bounding box is the
   smallest rectangle completely enclosing all points in the path, *not*
   including control points for BÃ©zier cubic and quadratic curves. If the
   path is empty, then return `CGRectNull'. }

function CGPathGetPathBoundingBox( path: CGPathRef ): CGRect; external name '_CGPathGetPathBoundingBox';
(* CG_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_4_0) *)

{ Return true if `point' is contained in `path'; false otherwise. A point
   is contained in a path if it is inside the painted region when the path
   is filled; if `eoFill' is true, then the even-odd fill rule is used to
   evaluate the painted region of the path, otherwise, the winding-number
   fill rule is used. If `m' is non-NULL, then the point is transformed by
   `m' before determining whether the path contains it. }

function CGPathContainsPoint( path: CGPathRef; m: CGAffineTransformPtr; point: CGPoint; eoFill: CBool ): CBool; external name '_CGPathContainsPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

=======

function CGPathGetCurrentPoint( path: CGPathRef ): CGPoint; external name '_CGPathGetCurrentPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Return the bounding box of `path'. The bounding box is the smallest
   rectangle completely enclosing all points in the path, including control
   points for BŽzier cubic and quadratic curves. If the path is empty, then
   return `CGRectNull'. }

function CGPathGetBoundingBox( path: CGPathRef ): CGRect; external name '_CGPathGetBoundingBox';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Return the path bounding box of `path'. The path bounding box is the
   smallest rectangle completely enclosing all points in the path, *not*
   including control points for BŽzier cubic and quadratic curves. If the
   path is empty, then return `CGRectNull'. }

function CGPathGetPathBoundingBox( path: CGPathRef ): CGRect; external name '_CGPathGetPathBoundingBox';
(* CG_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ Return true if `point' is contained in `path'; false otherwise. A point
   is contained in a path if it is inside the painted region when the path
   is filled; if `eoFill' is true, then the even-odd fill rule is used to
   evaluate the painted region of the path, otherwise, the winding-number
   fill rule is used. If `m' is non-NULL, then the point is transformed by
   `m' before determining whether the path contains it. }

function CGPathContainsPoint( path: CGPathRef; m: CGAffineTransformPtr; point: CGPoint; eoFill: CBool ): CBool; external name '_CGPathContainsPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

>>>>>>> graemeg/cpstrnew
=======
   points for BŽzier cubic and quadratic curves. If the path is empty, then
   return `CGRectNull'. }

function CGPathGetBoundingBox( path: CGPathRef ): CGRect; external name '_CGPathGetBoundingBox';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Return the path bounding box of `path'. The path bounding box is the
   smallest rectangle completely enclosing all points in the path, *not*
   including control points for BŽzier cubic and quadratic curves. If the
   path is empty, then return `CGRectNull'. }

function CGPathGetPathBoundingBox( path: CGPathRef ): CGRect; external name '_CGPathGetPathBoundingBox';
(* CG_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ Return true if `point' is contained in `path'; false otherwise. A point
   is contained in a path if it is inside the painted region when the path
   is filled; if `eoFill' is true, then the even-odd fill rule is used to
   evaluate the painted region of the path, otherwise, the winding-number
   fill rule is used. If `m' is non-NULL, then the point is transformed by
   `m' before determining whether the path contains it. }

function CGPathContainsPoint( path: CGPathRef; m: CGAffineTransformPtr; point: CGPoint; eoFill: CBool ): CBool; external name '_CGPathContainsPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/cpstrnew
   points for BŽzier cubic and quadratic curves. If the path is empty, then
   return `CGRectNull'. }

function CGPathGetBoundingBox( path: CGPathRef ): CGRect; external name '_CGPathGetBoundingBox';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Return the path bounding box of `path'. The path bounding box is the
   smallest rectangle completely enclosing all points in the path, *not*
   including control points for BŽzier cubic and quadratic curves. If the
   path is empty, then return `CGRectNull'. }

function CGPathGetPathBoundingBox( path: CGPathRef ): CGRect; external name '_CGPathGetPathBoundingBox';
(* CG_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ Return true if `point' is contained in `path'; false otherwise. A point
   is contained in a path if it is inside the painted region when the path
   is filled; if `eoFill' is true, then the even-odd fill rule is used to
   evaluate the painted region of the path, otherwise, the winding-number
   fill rule is used. If `m' is non-NULL, then the point is transformed by
   `m' before determining whether the path contains it. }

function CGPathContainsPoint( path: CGPathRef; m: CGAffineTransformPtr; point: CGPoint; eoFill: CBool ): CBool; external name '_CGPathContainsPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{ The types of path elements returned by `CGPathApply'. }
=======
=======
>>>>>>> origin/fixes_2_2
function CGPathIsEmpty( path: CGPathRef ): CBool; external name '_CGPathIsEmpty'; (* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
=======
function CGPathIsEmpty( path: CGPathRef ): CBool; external name '_CGPathIsEmpty';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ Return true if `path' represents a rectangle, false otherwise. }

function CGPathIsRect( path: CGPathRef; var rect: CGRect ): CBool; external name '_CGPathIsRect';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Return the current point of the current subpath of `path'. If there is no
   current point, then return CGPointZero. }

function CGPathGetCurrentPoint( path: CGPathRef ): CGPoint; external name '_CGPathGetCurrentPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Return the bounding box of `path'. The bounding box is the smallest
   rectangle completely enclosing all points in the path, including control
   points for BŽzier cubic and quadratic curves. If the path is empty, then
   return `CGRectNull'. }

function CGPathGetBoundingBox( path: CGPathRef ): CGRect; external name '_CGPathGetBoundingBox';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Return the path bounding box of `path'. The path bounding box is the
   smallest rectangle completely enclosing all points in the path, *not*
   including control points for BŽzier cubic and quadratic curves. If the
   path is empty, then return `CGRectNull'. }

function CGPathGetPathBoundingBox( path: CGPathRef ): CGRect; external name '_CGPathGetPathBoundingBox';
(* CG_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ Return true if `point' is contained in `path'; false otherwise. A point
   is contained in a path if it is inside the painted region when the path
   is filled; if `eoFill' is true, then the even-odd fill rule is used to
   evaluate the painted region of the path, otherwise, the winding-number
   fill rule is used. If `m' is non-NULL, then the point is transformed by
   `m' before determining whether the path contains it. }

function CGPathContainsPoint( path: CGPathRef; const (*var*) m: CGAffineTransform; point: CGPoint; eoFill: CBool ): CBool; external name '_CGPathContainsPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

<<<<<<< HEAD
function CGPathContainsPoint( path: CGPathRef; const (*var*) m: CGAffineTransform; point: CGPoint; eoFill: CBool ): CBool; external name '_CGPathContainsPoint'; (* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ The types of path elements returned by `CGPathApply'. }
>>>>>>> origin/fixes_2.4

type
	CGPathElementType = SInt32;
const
	kCGPathElementMoveToPoint = 0;
	kCGPathElementAddLineToPoint = 1;
	kCGPathElementAddQuadCurveToPoint = 2;
	kCGPathElementAddCurveToPoint = 3;
	kCGPathElementCloseSubpath = 4;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ An element of a path returned by `CGPathApply'. }

type
	CGPathElement = record
		typ: CGPathElementType;
{$ifc TARGET_CPU_64}
		__alignment_dummy: SInt32;
{$endc}
		points: CGPointPtr;
	end;

{ The prototype for the function called by `CGPathApplyFunction'. }

type
	CGPathApplierFunction = procedure( info: UnivPtr; const (*var*) element: CGPathElement );

{ For element of `path', call `function', passing it the path element and
   `info'. }

procedure CGPathApply( path: CGPathRef; info: UnivPtr; func: CGPathApplierFunction ); external name '_CGPathApply';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
=======
{ An element of a path returned by `CGPathApply'. }

>>>>>>> origin/fixes_2.4
type
	CGPathElement = record
		typ: CGPathElementType;
{$ifc TARGET_CPU_64}
		__alignment_dummy: SInt32;
{$endc}
		points: CGPointPtr;
	end;

{ The prototype for the function called by `CGPathApplyFunction'. }

type
	CGPathApplierFunction = procedure( info: UnivPtr; const (*var*) element: CGPathElement );

{ For element of `path', call `function', passing it the path element and
   `info'. }

procedure CGPathApply( path: CGPathRef; info: UnivPtr; func: CGPathApplierFunction ); external name '_CGPathApply';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4
