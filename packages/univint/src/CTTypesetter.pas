{
 *	CTTypesetter.h
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

unit CTTypesetter;
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
>>>>>>> origin/cpstrnew
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
>>>>>>> graemeg/cpstrnew
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
uses MacTypes,CTLine,CFBase,CFDictionary,CFAttributedString;
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
{ Typesetter Types }
{ --------------------------------------------------------------------------- }

type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CTTypesetterRef = ^__CTTypesetter; { an opaque type }
	__CTTypesetter = record end;
=======
	CTTypesetterRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CTTypesetterRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CTTypesetterRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CTTypesetterRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	CTTypesetterRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4


{!
	@function	CTTypesetterGetTypeID
	@abstract	Returns the CFType of the typesetter object
}

function CTTypesetterGetTypeID: CFTypeID; external name '_CTTypesetterGetTypeID';
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
{ Typesetter Values }
{ --------------------------------------------------------------------------- }

{!
	@const		kCTTypesetterOptionDisableBidiProcessing
	@abstract	Disables bidi processing.
	@discussion	Value must be a CFBooleanRef. Default is false.
				Normally, typesetting applies the Unicode Bidirectional
				Algorithm as described in UAX #9. If a typesetter is created
				with this option set to true, no directional reordering is
				performed and any directional control characters are ignored.
}

var kCTTypesetterOptionDisableBidiProcessing: CFStringRef; external name '_kCTTypesetterOptionDisableBidiProcessing'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CT_AVAILABLE_BUT_DEPRECATED( __MAC_10_5, __MAC_10_8, __IPHONE_3_2, __IPHONE_6_0) *)
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
	@const		kCTTypesetterOptionForcedEmbeddingLevel
	@abstract	Specifies the embedding level.
	@discussion	Value must be a CFNumberRef. Default is unset. Normally,
				typesetting applies the Unicode Bidirectional Algorithm as
				described in UAX #9. If present, this specifies the embedding
				level and any directional control characters are ignored.
}

var kCTTypesetterOptionForcedEmbeddingLevel: CFStringRef; external name '_kCTTypesetterOptionForcedEmbeddingLevel'; (* attribute const *)
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
{ Typesetter Creation }
{ --------------------------------------------------------------------------- }

{!
	@function	CTTypesetterCreateWithAttributedString
	@abstract	Creates an immutable typesetter object using an attributed
				string.

	@discussion The resultant typesetter can be used to create lines, perform
				line breaking, and do other contextual analysis based on the
				characters in the string.

	@param		string
				The CFAttributedStringRef that you want to typeset. This
				parameter must be filled in with a valid CFAttributedString.

	@result		This function will return a reference to a CTTypesetter if the
				call was successful. Otherwise, it will return NULL.
}

function CTTypesetterCreateWithAttributedString( strng: CFAttributedStringRef ): CTTypesetterRef; external name '_CTTypesetterCreateWithAttributedString';
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
	@function	CTTypesetterCreateWithAttributedStringAndOptions
	@abstract	Creates an immutable typesetter object using an attributed
				string and a dictionary of options.

	@discussion The resultant typesetter can be used to create lines, perform
				line breaking, and do other contextual analysis based on the
				characters in the string.

	@param		string
				The CFAttributedStringRef that you want to typeset. This
				parameter must be filled in with a valid CFAttributedString.

	@param		options
				A CFDictionary of typesetter options, or NULL if there are none.

	@result		This function will return a reference to a CTTypesetter if the
				call was successful. Otherwise, it will return NULL.
}

function CTTypesetterCreateWithAttributedStringAndOptions( strng: CFAttributedStringRef; options: CFDictionaryRef ): CTTypesetterRef; external name '_CTTypesetterCreateWithAttributedStringAndOptions';
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
{ Typeset Line Creation }
{ --------------------------------------------------------------------------- }

{!
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@function	CTTypesetterCreateLineWithOffset
=======
	@function	CTTypesetterCreateLine
>>>>>>> graemeg/cpstrnew
=======
	@function	CTTypesetterCreateLine
>>>>>>> graemeg/cpstrnew
=======
	@function	CTTypesetterCreateLine
>>>>>>> graemeg/cpstrnew
=======
	@function	CTTypesetterCreateLine
>>>>>>> origin/cpstrnew
=======
	@function	CTTypesetterCreateLine
>>>>>>> origin/fixes_2.4
	@abstract	Creates an immutable line from the typesetter.

	@discussion The resultant line will consist of glyphs in the correct visual
				order, ready to draw.

	@param		typesetter
				The typesetter which the line will come from. This parameter is
				required and cannot be set to NULL.

	@param		stringRange
				The string range which the line will be based on. If the length
				portion of range is set to 0, then the typesetter will continue
				to add glyphs to the line until it runs out of characters in the
				string. The location and length of the range must be within the
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
				bounds of the string, otherwise the call will fail.

	@param		offset
				The line position offset.
=======
				bounds of the string, othewise the call will fail.
>>>>>>> graemeg/cpstrnew
=======
				bounds of the string, othewise the call will fail.
>>>>>>> graemeg/cpstrnew
=======
				bounds of the string, othewise the call will fail.
>>>>>>> graemeg/cpstrnew
=======
				bounds of the string, othewise the call will fail.
>>>>>>> origin/cpstrnew
=======
				bounds of the string, othewise the call will fail.
>>>>>>> origin/fixes_2.4

	@result		This function will return a reference to a CTLine if the call was
				successful. Otherwise, it will return NULL.
}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function CTTypesetterCreateLineWithOffset( typesetter: CTTypesetterRef; stringRange: CFRange; offset: Float64 ): CTLineRef; external name '_CTTypesetterCreateLineWithOffset';
(* CT_AVAILABLE_STARTING( __MAC_10_6, __IPHONE_3_2) *)

{!
	@function	CTTypesetterCreateLine
	@abstract	Equivalent to CTTypesetterCreateLineWithOffset with offset = 0.0.
}

function CTTypesetterCreateLine( typesetter: CTTypesetterRef; stringRange: CFRange ): CTLineRef; external name '_CTTypesetterCreateLine';
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)
=======
function CTTypesetterCreateLine( typesetter: CTTypesetterRef; stringRange: CFRange ): CTLineRef; external name '_CTTypesetterCreateLine';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
function CTTypesetterCreateLine( typesetter: CTTypesetterRef; stringRange: CFRange ): CTLineRef; external name '_CTTypesetterCreateLine';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
function CTTypesetterCreateLine( typesetter: CTTypesetterRef; stringRange: CFRange ): CTLineRef; external name '_CTTypesetterCreateLine';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
function CTTypesetterCreateLine( typesetter: CTTypesetterRef; stringRange: CFRange ): CTLineRef; external name '_CTTypesetterCreateLine';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
function CTTypesetterCreateLine( typesetter: CTTypesetterRef; stringRange: CFRange ): CTLineRef; external name '_CTTypesetterCreateLine';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{ --------------------------------------------------------------------------- }
{ Typeset Line Breaking }
{ --------------------------------------------------------------------------- }

{!
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@function	CTTypesetterSuggestLineBreakWithOffset
=======
	@function	CTTypesetterSuggestLineBreak
>>>>>>> graemeg/cpstrnew
=======
	@function	CTTypesetterSuggestLineBreak
>>>>>>> graemeg/cpstrnew
=======
	@function	CTTypesetterSuggestLineBreak
>>>>>>> graemeg/cpstrnew
=======
	@function	CTTypesetterSuggestLineBreak
>>>>>>> origin/cpstrnew
=======
	@function	CTTypesetterSuggestLineBreak
>>>>>>> origin/fixes_2.4
	@abstract	Suggests a contextual line break point based on the width
				provided.

	@discussion The line break can be triggered either by a hard break character
				in the stream or by filling the specified width with characters.

	@param		typesetter
				The typesetter which the line will come from. This parameter is
				required and cannot be set to NULL.

	@param		startIndex
				The starting point for the line break calculations. The break
				calculations will include the character starting at startIndex.

	@param		width
				The requested line break width.

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@param		offset
				The line position offset.

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
	@result		The value returned is a count of the characters from startIndex
				that would cause the line break. This value returned can be used
				to construct a character range for CTTypesetterCreateLine.
}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function CTTypesetterSuggestLineBreakWithOffset( typesetter: CTTypesetterRef; startIndex: CFIndex; width: Float64; offset: Float64 ): CFIndex; external name '_CTTypesetterSuggestLineBreakWithOffset';
(* CT_AVAILABLE_STARTING( __MAC_10_6, __IPHONE_3_2) *)

{!
	@function	CTTypesetterSuggestLineBreak
	@abstract	Equivalent to CTTypesetterSuggestLineBreakWithOffset with offset = 0.0.
}

function CTTypesetterSuggestLineBreak( typesetter: CTTypesetterRef; startIndex: CFIndex; width: Float64 ): CFIndex; external name '_CTTypesetterSuggestLineBreak';
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)


{!
	@function	CTTypesetterSuggestClusterBreakWithOffset
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
function CTTypesetterSuggestLineBreak( typesetter: CTTypesetterRef; startIndex: CFIndex; width: Float64 ): CFIndex; external name '_CTTypesetterSuggestLineBreak';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{!
	@function	CTTypesetterSuggestClusterBreak
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
	@abstract	Suggests a cluster line break point based on the width provided.

	@discussion Suggests a typographic cluster line break point based on the width
				provided. This cluster break is similar to a character break,
				except that it will not break apart linguistic clusters. No other
				contextual analysis will be done. This can be used by the caller
				to implement a different line breaking scheme, such as
				hyphenation. Note that a typographic cluster break can also be
				triggered by a hard break character in the stream.

	@param		typesetter
				The typesetter which the line will come from. This parameter is
				required and cannot be set to NULL.

	@param		startIndex
				The starting point for the typographic cluster break
				calculations. The break calculations will include the character
				starting at startIndex.

	@param		width
				The requested typographic cluster break width.

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@param		offset
				The line position offset.

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
	@result		The value returned is a count of the characters from startIndex
				that would cause the cluster break. This value returned can be
				used to construct a character range for CTTypesetterCreateLine.
}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function CTTypesetterSuggestClusterBreakWithOffset( typesetter: CTTypesetterRef; startIndex: CFIndex; width: Float64; offset: Float64 ): CFIndex; external name '_CTTypesetterSuggestClusterBreakWithOffset';
(* CT_AVAILABLE_STARTING( __MAC_10_6, __IPHONE_3_2) *)


{!
	@function	CTTypesetterSuggestClusterBreak
	@abstract	Equivalent to CTTypesetterSuggestClusterBreakWithOffset with offset = 0.0.
}

function CTTypesetterSuggestClusterBreak( typesetter: CTTypesetterRef; startIndex: CFIndex; width: Float64 ): CFIndex; external name '_CTTypesetterSuggestClusterBreak';
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
function CTTypesetterSuggestClusterBreak( typesetter: CTTypesetterRef; startIndex: CFIndex; width: Float64 ): CFIndex; external name '_CTTypesetterSuggestClusterBreak';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{$endc} {TARGET_OS_MAC}
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
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
