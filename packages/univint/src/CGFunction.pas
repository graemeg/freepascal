{ CoreGraphics - CGFunction.h
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   Copyright (c) 1999-2011 Apple Inc.
   All rights reserved. }
{       Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, August 2015 }
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 * Copyright (c) 1999-2008 Apple Inc.
 * All rights reserved. }
{       Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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
{
    Modified for use with Free Pascal
    Version 308
    Please report any bugs to <gpc@microbizz.nl>
}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
 * Copyright (c) 1999-2002 Apple Computer, Inc. (unpublished)
 * All rights reserved.
 }
=======
 * Copyright (c) 1999-2008 Apple Inc.
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

unit CGFunction;
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
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
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
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
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
>>>>>>> origin/cpstrnew
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> graemeg/cpstrnew
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
=======
{$elsec}
=======
{$elsec}
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
>>>>>>> origin/cpstrnew
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
uses MacTypes,CGBase,CFBase;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not MACOSALLINCLUDE}

{$ALIGN POWER}


{ A CGFunction is a general floating-point function evaluator which uses a
   user-specified callback to map an arbitrary number of inputs to an
   arbitrary number of outputs. }

type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CGFunctionRef = ^OpaqueCGFunctionRef; { an opaque type }
	OpaqueCGFunctionRef = record end;
=======
	CGFunctionRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CGFunctionRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CGFunctionRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CGFunctionRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew


{ This callback evaluates a function, using `in' as inputs, and places the
   result in `out'. `info' is the info parameter passed to the CGFunction
   creation functions. }

type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CGFunctionEvaluateCallback = procedure( info: UnivPtr; inp: {const} CGFloatPtr; out: CGFloatPtr );
=======
	CGFunctionEvaluateCallback = procedure( info: UnivPtr; inp: {const} CGFloatPtr; out: Float32Ptr );
>>>>>>> graemeg/cpstrnew
=======
	CGFunctionEvaluateCallback = procedure( info: UnivPtr; inp: {const} CGFloatPtr; out: Float32Ptr );
>>>>>>> graemeg/cpstrnew
=======
	CGFunctionEvaluateCallback = procedure( info: UnivPtr; inp: {const} CGFloatPtr; out: Float32Ptr );
>>>>>>> graemeg/cpstrnew
=======
	CGFunctionEvaluateCallback = procedure( info: UnivPtr; inp: {const} CGFloatPtr; out: Float32Ptr );
>>>>>>> origin/cpstrnew

{ When a function is deallocated, this callback releases `info', the info
   parameter passed to the CGFunction creation functions. }
=======
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {not MACOSALLINCLUDE}

>>>>>>> origin/fixes_2.4
{$ALIGN POWER}


{ A CGFunction is a general floating-point function evaluator which uses a
   user-specified callback to map an arbitrary number of inputs to an
   arbitrary number of outputs. }

type
	CGFunctionRef = ^SInt32; { an opaque type }


{ This callback evaluates a function, using `in' as inputs, and places the
   result in `out'. `info' is the info parameter passed to the CGFunction
   creation functions. }

type
<<<<<<< HEAD
	CGFunctionEvaluateCallback = procedure( info: UnivPtr; inp: {const} Float32Ptr; out: Float32Ptr );

{! @typedef CGFunctionReleaseInfoCallback
 *   This callback releases the info parameter passed to the CGFunction
 *   creation functions when the function is deallocated.
 *
 * @param info
 *   The info parameter passed to CGFunctionCreate.
 }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	CGFunctionEvaluateCallback = procedure( info: UnivPtr; inp: {const} CGFloatPtr; out: Float32Ptr );

{ When a function is deallocated, this callback releases `info', the info
   parameter passed to the CGFunction creation functions. }
>>>>>>> origin/fixes_2.4

type
	CGFunctionReleaseInfoCallback = procedure( info: UnivPtr );

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{ Callbacks for a CGFunction.
     `version' is the version number of this structure. This structure is
       version 0.
     `evaluate' is the callback used to evaluate the function.
     `releaseInfo', if non-NULL, is the callback used to release the info
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
       parameter passed to the CGFunction creation functions when the
       function is deallocated. }
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
       parameter passed to the CGFunction creation functions when the function
       is deallocated.
 }
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
{! @typedef CGFunctionCallbacks
 *   Structure containing the callbacks of a CGFunction.
 *
 * @field version
 *   The version number of the structure passed to the CGFunction creation
 *   functions. This structure is version 0.
 *
 * @field evaluate
 *   The callback used to evaluate the function.
 *
 * @field releaseInfo
 *   If non-NULL, the callback used to release the info parameter passed to
 *   the CGFunction creation functions when the function is deallocated.
=======
       parameter passed to the CGFunction creation functions when the function
       is deallocated.
>>>>>>> origin/fixes_2.4
 }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

type
	CGFunctionCallbacks = record
		version: UInt32;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc TARGET_CPU_64}
 		__alignment_dummy: UInt32;
{$endif}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$ifc TARGET_CPU_64}
 		__alignment_dummy: UInt32;
{$endif}
>>>>>>> origin/fixes_2.4
		evaluate: CGFunctionEvaluateCallback;
		releaseInfo: CGFunctionReleaseInfoCallback;
	end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ Return the CFTypeID for CGFunctionRefs. }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

function CGFunctionGetTypeID: CFTypeID; external name '_CGFunctionGetTypeID';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Create a CGFunction using `callbacks' to evaluate the function. `info' is
   passed to each of the callback functions. `domainDimension' is the number
   of input values to the function; `rangeDimension' is the number of output
   values from the function.

   `domain' is an array of 2M values, where M is the number of input values.
   For each k from 0 to M-1, domain[2*k] must be less than or equal to
   domain[2*k+1]. The k'th input value (in[k]) will be clipped to lie in
   this interval, so that domain[2*k] <= in[k] <= domain[2*k+1]. If `domain'
   is NULL, then the input values will not be clipped. However, it's
   strongly recommended that `domain' be specified; each domain interval
   should specify reasonable values for the minimum and maximum in that
   dimension.

   `range' is an array of 2N values, where N is the number of output values.
   For each k from 0 to N-1, range[2*k] must be less than or equal to
   range[2*k+1]. The k'th output value (out[k]) will be clipped to lie in
   this interval, so that range[2*k] <= out[k] <= range[2*k+1]. If `range'
   is NULL, then the output values will not be clipped. However, it's
   strongly recommended that `range' be specified; each range interval
   should specify reasonable values for the minimum and maximum in that
   dimension.

   The contents of the callbacks structure is copied, so, for example, a
   pointer to a structure on the stack can be passed to this function. }

function CGFunctionCreate( info: UnivPtr; domainDimension: size_t; domain: {const} CGFloatPtr; rangeDimension: size_t; range: {const} CGFloatPtr; const (*var*) callbacks: CGFunctionCallbacks ): CGFunctionRef; external name '_CGFunctionCreate';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Equivalent to `CFRetain(function)', except it doesn't crash (as CFRetain
   does) if `function' is NULL. }

function CGFunctionRetain( func: CGFunctionRef ): CGFunctionRef; external name '_CGFunctionRetain';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Equivalent to `CFRelease(function)', except it doesn't crash (as
   CFRelease does) if `function' is NULL. }

=======

function CGFunctionGetTypeID: CFTypeID; external name '_CGFunctionGetTypeID';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)
 
{ Create a CGFunction using `callbacks' to evaluate the function. `info' is
   passed to each of the callback functions. `domainDimension' is the number
   of input values to the function; `rangeDimension' is the number of output
   values from the function.

   `domain' is an array of 2M values, where M is the number of input values.
   For each k from 0 to M-1, domain[2*k] must be less than or equal to
   domain[2*k+1]. The k'th input value (in[k]) will be clipped to lie in
   this interval, so that domain[2*k] <= in[k] <= domain[2*k+1]. If `domain'
   is NULL, then the input values will not be clipped. However, it's
   strongly recommended that `domain' be specified; each domain interval
   should specify reasonable values for the minimum and maximum in that
   dimension.

   `range' is an array of 2N values, where N is the number of output values.
   For each k from 0 to N-1, range[2*k] must be less than or equal to
   range[2*k+1]. The k'th output value (out[k]) will be clipped to lie in
   this interval, so that range[2*k] <= out[k] <= range[2*k+1]. If `range'
   is NULL, then the output values will not be clipped. However, it's
   strongly recommended that `range' be specified; each range interval
   should specify reasonable values for the minimum and maximum in that
   dimension.

=======

function CGFunctionGetTypeID: CFTypeID; external name '_CGFunctionGetTypeID';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)
 
{ Create a CGFunction using `callbacks' to evaluate the function. `info' is
   passed to each of the callback functions. `domainDimension' is the number
   of input values to the function; `rangeDimension' is the number of output
   values from the function.

   `domain' is an array of 2M values, where M is the number of input values.
   For each k from 0 to M-1, domain[2*k] must be less than or equal to
   domain[2*k+1]. The k'th input value (in[k]) will be clipped to lie in
   this interval, so that domain[2*k] <= in[k] <= domain[2*k+1]. If `domain'
   is NULL, then the input values will not be clipped. However, it's
   strongly recommended that `domain' be specified; each domain interval
   should specify reasonable values for the minimum and maximum in that
   dimension.

   `range' is an array of 2N values, where N is the number of output values.
   For each k from 0 to N-1, range[2*k] must be less than or equal to
   range[2*k+1]. The k'th output value (out[k]) will be clipped to lie in
   this interval, so that range[2*k] <= out[k] <= range[2*k+1]. If `range'
   is NULL, then the output values will not be clipped. However, it's
   strongly recommended that `range' be specified; each range interval
   should specify reasonable values for the minimum and maximum in that
   dimension.

>>>>>>> graemeg/cpstrnew
=======

function CGFunctionGetTypeID: CFTypeID; external name '_CGFunctionGetTypeID';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)
 
{ Create a CGFunction using `callbacks' to evaluate the function. `info' is
   passed to each of the callback functions. `domainDimension' is the number
   of input values to the function; `rangeDimension' is the number of output
   values from the function.

   `domain' is an array of 2M values, where M is the number of input values.
   For each k from 0 to M-1, domain[2*k] must be less than or equal to
   domain[2*k+1]. The k'th input value (in[k]) will be clipped to lie in
   this interval, so that domain[2*k] <= in[k] <= domain[2*k+1]. If `domain'
   is NULL, then the input values will not be clipped. However, it's
   strongly recommended that `domain' be specified; each domain interval
   should specify reasonable values for the minimum and maximum in that
   dimension.

   `range' is an array of 2N values, where N is the number of output values.
   For each k from 0 to N-1, range[2*k] must be less than or equal to
   range[2*k+1]. The k'th output value (out[k]) will be clipped to lie in
   this interval, so that range[2*k] <= out[k] <= range[2*k+1]. If `range'
   is NULL, then the output values will not be clipped. However, it's
   strongly recommended that `range' be specified; each range interval
   should specify reasonable values for the minimum and maximum in that
   dimension.

>>>>>>> graemeg/cpstrnew
=======

function CGFunctionGetTypeID: CFTypeID; external name '_CGFunctionGetTypeID';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)
 
{ Create a CGFunction using `callbacks' to evaluate the function. `info' is
   passed to each of the callback functions. `domainDimension' is the number
   of input values to the function; `rangeDimension' is the number of output
   values from the function.

   `domain' is an array of 2M values, where M is the number of input values.
   For each k from 0 to M-1, domain[2*k] must be less than or equal to
   domain[2*k+1]. The k'th input value (in[k]) will be clipped to lie in
   this interval, so that domain[2*k] <= in[k] <= domain[2*k+1]. If `domain'
   is NULL, then the input values will not be clipped. However, it's
   strongly recommended that `domain' be specified; each domain interval
   should specify reasonable values for the minimum and maximum in that
   dimension.

   `range' is an array of 2N values, where N is the number of output values.
   For each k from 0 to N-1, range[2*k] must be less than or equal to
   range[2*k+1]. The k'th output value (out[k]) will be clipped to lie in
   this interval, so that range[2*k] <= out[k] <= range[2*k+1]. If `range'
   is NULL, then the output values will not be clipped. However, it's
   strongly recommended that `range' be specified; each range interval
   should specify reasonable values for the minimum and maximum in that
   dimension.

>>>>>>> origin/cpstrnew
   The contents of the callbacks structure is copied, so, for example, a
   pointer to a structure on the stack can be passed to this function. }

function CGFunctionCreate( info: UnivPtr; domainDimension: size_t; domain: {const} CGFloatPtr; rangeDimension: size_t; range: {const} Float32Ptr; const (*var*) callbacks: CGFunctionCallbacks ): CGFunctionRef; external name '_CGFunctionCreate';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Equivalent to `CFRetain(function)', except it doesn't crash (as CFRetain
   does) if `function' is NULL. }

function CGFunctionRetain( func: CGFunctionRef ): CGFunctionRef; external name '_CGFunctionRetain';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Equivalent to `CFRelease(function)', except it doesn't crash (as
   CFRelease does) if `function' is NULL. }

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
procedure CGFunctionRelease( func: CGFunctionRef ); external name '_CGFunctionRelease';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
=======
{ Return the CFTypeID for CGFunctionRefs. }
>>>>>>> origin/fixes_2.4

function CGFunctionGetTypeID: CFTypeID; external name '_CGFunctionGetTypeID';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)
 
{ Create a CGFunction using `callbacks' to evaluate the function. `info' is
   passed to each of the callback functions. `domainDimension' is the number
   of input values to the function; `rangeDimension' is the number of output
   values from the function.

   `domain' is an array of 2M values, where M is the number of input values.
   For each k from 0 to M-1, domain[2*k] must be less than or equal to
   domain[2*k+1]. The k'th input value (in[k]) will be clipped to lie in
   this interval, so that domain[2*k] <= in[k] <= domain[2*k+1]. If `domain'
   is NULL, then the input values will not be clipped. However, it's
   strongly recommended that `domain' be specified; each domain interval
   should specify reasonable values for the minimum and maximum in that
   dimension.

   `range' is an array of 2N values, where N is the number of output values.
   For each k from 0 to N-1, range[2*k] must be less than or equal to
   range[2*k+1]. The k'th output value (out[k]) will be clipped to lie in
   this interval, so that range[2*k] <= out[k] <= range[2*k+1]. If `range'
   is NULL, then the output values will not be clipped. However, it's
   strongly recommended that `range' be specified; each range interval
   should specify reasonable values for the minimum and maximum in that
   dimension.

   The contents of the callbacks structure is copied, so, for example, a
   pointer to a structure on the stack can be passed to this function. }

function CGFunctionCreate( info: UnivPtr; domainDimension: size_t; domain: {const} CGFloatPtr; rangeDimension: size_t; range: {const} Float32Ptr; const (*var*) callbacks: CGFunctionCallbacks ): CGFunctionRef; external name '_CGFunctionCreate';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Equivalent to `CFRetain(function)', except it doesn't crash (as CFRetain
   does) if `function' is NULL. }

function CGFunctionRetain( func: CGFunctionRef ): CGFunctionRef; external name '_CGFunctionRetain';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Equivalent to `CFRelease(function)', except it doesn't crash (as
   CFRelease does) if `function' is NULL. }

procedure CGFunctionRelease( func: CGFunctionRef ); external name '_CGFunctionRelease';
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
