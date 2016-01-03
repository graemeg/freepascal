{
 *	CTFrame.h
 *	CoreText
 *
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *	Copyright (c) 2003-2012 Apple Inc. All rights reserved.
 *
 }
{  Initial Pascal Translation:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, August 2015 }
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
 *	Copyright (c) 2003-2008 Apple Inc. All rights reserved.
 *
 }
{       Initial Pascal Translation:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
<<<<<<< HEAD
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
=======
>>>>>>> origin/fixes_2.4
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

unit CTFrame;
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
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/fixes_2.4
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
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/fixes_2.4
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
=======
>>>>>>> origin/fixes_2.4
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
=======
>>>>>>> origin/fixes_2.4
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
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/fixes_2.4
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
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/fixes_2.4
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
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> origin/fixes_2.4
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
uses MacTypes,CFArray,CFDictionary,CFBase,CFNumber,CFString,CGContext,CGGeometry,CGPath;
{$endc} {not MACOSALLINCLUDE}


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
{$ifc TARGET_OS_MAC}

>>>>>>> graemeg/cpstrnew
=======
{$ifc TARGET_OS_MAC}

>>>>>>> graemeg/cpstrnew
=======
{$ifc TARGET_OS_MAC}

>>>>>>> graemeg/cpstrnew
=======
{$ifc TARGET_OS_MAC}

>>>>>>> origin/cpstrnew
=======
{$ifc TARGET_OS_MAC}

>>>>>>> origin/fixes_2.4
{$ALIGN POWER}

 
{!
    @header

    Thread Safety Information

    All functions in this header are thread safe unless otherwise specified.
}


{ --------------------------------------------------------------------------- }
{ Frame Types }
{ --------------------------------------------------------------------------- }

type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CTFrameRef = ^__CTFrame; { an opaque type }
	__CTFrame = record end;
=======
	CTFrameRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CTFrameRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CTFrameRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CTFrameRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	CTFrameRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4


{!
	@function	CTFrameGetTypeID
	@abstract	Returns the CFType of the frame object
}

function CTFrameGetTypeID: CFTypeID; external name '_CTFrameGetTypeID';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{ --------------------------------------------------------------------------- }
{ Frame Values }
{ --------------------------------------------------------------------------- }

{!
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    @enum		CTFrameProgression
    @abstract	These constants specify frame progression types.

    @discussion The lines of text within a frame may be stacked for either
                horizontal or vertical text. Values are enumerated for each
                stacking type supported by CTFrame. Frames created with a
                progression type specifying vertical text will rotate lines
                90 degrees counterclockwise when drawing.

    @constant	kCTFrameProgressionTopToBottom
                Lines are stacked top to bottom for horizontal text.

    @constant	kCTFrameProgressionRightToLeft
                Lines are stacked right to left for vertical text.

    @constant	kCTFrameProgressionLeftToRight
                Lines are stacked left to right for vertical text.
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	@enum		CTFrameProgression
	@abstract	These constants specify frame progression types.
	
	@discussion The lines of text within a frame may be stacked for either
				horizontal or vertical text. Values are enumerated for each
				stacking type supported by CTFrame. Frames created with a
				progression type specifying vertical text will rotate lines
				90 degrees counterclockwise when drawing.
	
	@constant	kCTFrameProgressionTopToBottom
				Lines are stacked top to bottom for horizontal text.
	
	@constant	kCTFrameProgressionRightToLeft
				Lines are stacked right to left for vertical text.
<<<<<<< HEAD
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
=======
>>>>>>> origin/fixes_2.4
}

const
	kCTFrameProgressionTopToBottom = 0;
	kCTFrameProgressionRightToLeft = 1;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        kCTFrameProgressionLeftToRight  = 2;
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
type
	CTFrameProgression = UInt32;


{!
	@const		kCTFrameProgressionAttributeName
	@abstract	Specifies progression for a frame.
	
	@discussion Value must be a CFNumberRef containing a CTFrameProgression.
				Default is kCTFrameProgressionTopToBottom. This value determines
				the line stacking behavior for a frame and does not affect the
				appearance of the glyphs within that frame.

	@seealso	CTFramesetterCreateFrame
}

var kCTFrameProgressionAttributeName: CFStringRef; external name '_kCTFrameProgressionAttributeName'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)

{!
	@enum		CTFramePathFillRule
	@abstract	These constants specify fill rule used by the frame.
 
	@discussion When a path intersects with itself, the client should specify which rule to use for deciding the 
				area of the path.
 
	@constant	kCTFramePathFillEvenOdd
				Text is filled in the area that would be painted if the path were given to CGContextEOFillPath.

	@constant	kCTFramePathFillWindingNumber
				Text is fill in the area that would be painted if the path were given to CGContextFillPath.
 
 
 }

const
	kCTFramePathFillEvenOdd = 0;
	kCTFramePathFillWindingNumber = 1;
type
	CTFramePathFillRule = UInt32;


{!
	@const		kCTFramePathFillRuleAttributeName
	@abstract	Specifies fill rule for a frame if this attribute is used at top level of frameAttributes dictionary, or specify
				fill rule for a clipping path if used in a dictionary contained in an array specified by kCTFrameClippingPathsAttributeName.
				
	@discussion Value must be a CFNumberRef containing kCTFramePathFillEvenOdd or kCTFramePathFillWindingNumber.
				Default is kCTFramePathFillEvenOdd.

	@seealso	CTFramesetterCreateFrame
 }

var kCTFramePathFillRuleAttributeName: CFStringRef; external name '_kCTFramePathFillRuleAttributeName'; (* attribute const *)
(* CT_AVAILABLE_STARTING( __MAC_10_7, __IPHONE_4_2) *)

{!
	@const		kCTFramePathWidthAttributeName
	@abstract	Specifies frame width if this attribute is used at top level of frameAttributes dictionary, or specify
				clipping path width if used in a dictionary contained in an array specified by kCTFrameClippingPathsAttributeName.

	@discussion Value must be a CFNumberRef specifying frame width.
				Default is zero.

	@seealso	CTFramesetterCreateFrame
 }

var kCTFramePathWidthAttributeName: CFStringRef; external name '_kCTFramePathWidthAttributeName'; (* attribute const *)
(* CT_AVAILABLE_STARTING( __MAC_10_7, __IPHONE_4_2) *)

	
{!
	@const		kCTFrameClippingPathsAttributeName
	@abstract	Specifies array of paths to clip frame.
	
	@discussion Value must be a CFArrayRef containing CFDictionaryRefs or CGPathRef.  (CGPathRef is allowed on 10.8 or later.)
				Each dictionary should have a kCTFramePathClippingPathAttributeName key-value pair, and can have a kCTFramePathFillRuleAttributeName key-value pair 
				and kCTFramePathFillRuleAttributeName key-value pair as optional parameters.  In case of CGPathRef, default fill rule (kCTFramePathFillEvenOdd) and width (0.0) are used.

	@seealso	CTFramesetterCreateFrame
}

var kCTFrameClippingPathsAttributeName: CFStringRef; external name '_kCTFrameClippingPathsAttributeName'; (* attribute const *)
(* CT_AVAILABLE_STARTING( __MAC_10_7, __IPHONE_4_3) *)

{!
	@const		kCTFramePathClippingPathAttributeName
	@abstract	Specifies clipping path.  This attribute is valid in a dictionary contained in an array specified by kCTFrameClippingPathsAttributeName.
				On 10.8 or later, This attribute is also valid in frameAttributes dictionary passed to CTFramesetterCreateFrame.

	@discussion Value must be a CGPathRef specifying a clipping path.

	@seealso	kCTFrameClippingPathsAttributeName
 }

var kCTFramePathClippingPathAttributeName: CFStringRef; external name '_kCTFramePathClippingPathAttributeName'; (* attribute const *)
(* CT_AVAILABLE_STARTING( __MAC_10_7, __IPHONE_4_3) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> origin/fixes_2.4

{ --------------------------------------------------------------------------- }
{ Frame Accessors }
{ --------------------------------------------------------------------------- }

{!
	@function	CTFrameGetStringRange
	@abstract	Returns the range of characters that were originally requested
				to fill the frame.

	@param		frame
				The frame that you want to get the character range from.

	@result		This function will return a CFRange containing the backing
				store range of characters that were originally requested
				to fill the frame. If the function call is not successful,
				then an empty range will be returned.
}

function CTFrameGetStringRange( frame: CTFrameRef ): CFRange; external name '_CTFrameGetStringRange';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{!
	@function	CTFrameGetVisibleStringRange
	@abstract	Returns the range of characters that actually fit in the
				frame.

	@discussion This can be used to cascade frames, as it returns the range of
				characters that can be seen in the frame. The next frame would
				start where this frame ends.

	@param		frame
				The frame that you want to get the visible character range
				from.

	@result		This function will return a CFRange containing the backing
				store range of characters that fit into the frame. If the
				function call is not successful, or if no characters fit
				in the frame, then an empty range will be returned.
}

function CTFrameGetVisibleStringRange( frame: CTFrameRef ): CFRange; external name '_CTFrameGetVisibleStringRange';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{!
	@function	CTFrameGetPath
	@abstract	Returns the path used to create the frame.

	@param		frame
				The frame that you want to obtain the path from.
}

function CTFrameGetPath( frame: CTFrameRef ): CGPathRef; external name '_CTFrameGetPath';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{!
	@function	CTFrameGetFrameAttributes
	@abstract	Returns the frame attributes used to create the frame.

	@discussion It is possible to create a frame with an attributes dictionary
				in order to control various aspects of the framing process.
				These attributes are different from the ones that are used to
				create an attributed string.

	@param		frame
				The frame that you want to obtain the frame attributes from.

	@result		This function will return a CFDictionary containing the
				frame attributes that were used to create the frame. If the
				frame was created without any frame attributes, this function
				will return NULL.
}

function CTFrameGetFrameAttributes( frame: CTFrameRef ): CFDictionaryRef; external name '_CTFrameGetFrameAttributes';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{!
	@function	CTFrameGetLines
	@abstract	Returns an array of lines that make up the frame.

	@discussion This function will return an array of CTLine objects that are
				stored in the frame. These line objects can be accessed and
				manipulated in any way that normal line objects can be. It is
				possible that an empty frame exists. That is, a frame in which
				no lines exist. In this case, the returned array will have 0
				entries.

	@param		frame
				The frame that you want to obtain the line array from.

	@result		This function will return a CFArray object containing the
				CTLine objects that make up the frame.
}

function CTFrameGetLines( frame: CTFrameRef ): CFArrayRef; external name '_CTFrameGetLines';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{!
	@function	CTFrameGetLineOrigins
	@abstract	Copies a range of line origins for a frame.

	@discussion	This function will copy a range of CGPoint structures. Each
				CGPoint is the origin of the corresponding line in the array of
				lines returned by CTFrameGetLines, relative to the origin of the
				frame's path. The maximum number of line origins returned by
				this function is the count of the array of lines.

	@param		frame
				The frame that you want to obtain the line origin array from.

	@param		range
				The range of line origins you wish to copy. If the length of the
				range is set to 0, then the copy operation will continue from
				the range's start index to the last line origin.

	@param		origins
				The buffer to which the origins will be copied. The buffer must
				have at least as many elements as specified by range's length.
				When using the origins to calculate measurements for a frame's
				contents, remember that line origins do not always correspond to
				line metrics; paragraph style settings can affect line origins,
				for one. The overall typographic bounds of a frame may generally
				be calculated as the difference between the top of the frame and
				the descent of the last line. This will obviously exclude any
				spacing following the last line, but such spacing has no effect
				on framesetting in the first place.
}

procedure CTFrameGetLineOrigins( frame: CTFrameRef; range: CFRange; origins: {variable-size-array} CGPointPtr ); external name '_CTFrameGetLineOrigins';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{!
	@function	CTFrameDraw
	@abstract	Draws an entire frame to a context.

	@discussion This function will draw an entire frame to the context. Note
				that this call may leave the context in any state and does not
				flush it after the draw operation.

	@param		frame
				The frame that you want to draw.

	@param		context
				The context to draw the frame to.

	@result		If both the frame and the context are valid, the frame will be
				drawn in the context.
}

procedure CTFrameDraw( frame: CTFrameRef; context: CGContextRef ); external name '_CTFrameDraw';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)

=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{$endc} {TARGET_OS_MAC}
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{$endc} {TARGET_OS_MAC}
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{$endc} {TARGET_OS_MAC}
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{$endc} {TARGET_OS_MAC}
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{$endc} {TARGET_OS_MAC}
>>>>>>> origin/fixes_2.4
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
