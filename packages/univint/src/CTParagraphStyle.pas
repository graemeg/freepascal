{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  CTParagraphStyle.h
 *  CoreText
 *
 *  Copyright (c) 2004-2012 Apple Inc. All rights reserved.
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
 *	CTParagraphStyle.h
 *	CoreText
 *
 *	Copyright (c) 2004-2008 Apple Inc. All rights reserved.
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

unit CTParagraphStyle;
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
>>>>>>> graemeg/cpstrnew
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
uses MacTypes,CFArray,CGBase,CFBase;
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
{ Paragraph Style Types }
{ --------------------------------------------------------------------------- }

type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CTParagraphStyleRef = ^__CTParagraphStyle; { an opaque type }
	__CTParagraphStyle = record end;


{!
    @function   CTParagraphStyleGetTypeID
    @abstract   Returns the CFType of the paragraph style object
}

function CTParagraphStyleGetTypeID: CFTypeID; external name '_CTParagraphStyleGetTypeID';
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
	CTParagraphStyleRef = ^SInt32; { an opaque type }


{!
	@function	CTParagraphStyleGetTypeID
	@abstract	Returns the CFType of the paragraph style object
}

function CTParagraphStyleGetTypeID: CFTypeID; external name '_CTParagraphStyleGetTypeID';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
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


{ --------------------------------------------------------------------------- }
{ Paragraph Style Values }
{ --------------------------------------------------------------------------- }

{!
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    @enum       CTTextAlignment
    @abstract   These constants specify text alignment.

    @constant   kCTTextAlignmentLeft
                Text is visually left-aligned.

    @constant   kCTTextAlignmentRight
                Text is visually right-aligned.

    @constant   kCTTextAlignmentCenter
                Text is visually center-aligned.

    @constant   kCTTextAlignmentJustified
                Text is fully justified. The last line in a paragraph is
                naturally aligned.

    @constant   kCTTextAlignmentNatural
                Use the natural alignment of the text's script.
}

const
    kCTTextAlignmentLeft = 0;
    kCTTextAlignmentRight = 1;
    kCTTextAlignmentCenter = 2;
    kCTTextAlignmentJustified = 3;
    kCTTextAlignmentNatural = 4;

    kCTLeftTextAlignment = kCTTextAlignmentLeft;
    kCTRightTextAlignment = kCTTextAlignmentRight;
    kCTCenterTextAlignment = kCTTextAlignmentCenter;
    kCTJustifiedTextAlignment = kCTTextAlignmentJustified;
    kCTNaturalTextAlignment = kCTTextAlignmentNatural;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	@enum		CTTextAlignment
	@abstract	These constants specify text alignment.

	@constant	kCTLeftTextAlignment
				Text is visually left-aligned.

	@constant	kCTRightTextAlignment
				Text is visually right-aligned.

	@constant	kCTCenterTextAlignment
				Text is visually center-aligned.

	@constant	kCTJustifiedTextAlignment
				Text is fully justified. The last line in a paragraph is
				naturally aligned.

	@constant	kCTNaturalTextAlignment
				Use the natural alignment of the text's script.
}

const
	kCTLeftTextAlignment = 0;
	kCTRightTextAlignment = 1;
	kCTCenterTextAlignment = 2;
	kCTJustifiedTextAlignment = 3;
	kCTNaturalTextAlignment = 4;
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
type
	CTTextAlignment = UInt8;


{!
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    @enum       CTLineBreakMode
    @abstract   These constants specify what happens when a line is too long for
                its frame.

    @constant   kCTLineBreakByWordWrapping
                Wrapping occurs at word boundaries, unless the word itself doesn't
                fit on a single line.

    @constant   kCTLineBreakByCharWrapping
                Wrapping occurs before the first character that doesn't fit.

    @constant   kCTLineBreakByClipping
                Lines are simply not drawn past the edge of the frame.

    @constant   kCTLineBreakByTruncatingHead
                Each line is displayed so that the end fits in the frame and the
                missing text is indicated by some kind of ellipsis glyph.

    @constant   kCTLineBreakByTruncatingTail
                Each line is displayed so that the beginning fits in the
                container and the missing text is indicated by some kind of
                ellipsis glyph.

    @constant   kCTLineBreakByTruncatingMiddle
                Each line is displayed so that the beginning and end fit in the
                container and the missing text is indicated by some kind of
                ellipsis glyph in the middle.
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	@enum		CTLineBreakMode
	@abstract	These constants specify what happens when a line is too long for
				its frame.

	@constant	kCTLineBreakByWordWrapping
				Wrapping occurs at word boundaries, unless the word itself doesn't
				fit on a single line.

	@constant	kCTLineBreakByCharWrapping
				Wrapping occurs before the first character that doesn't fit.

	@constant	kCTLineBreakByClipping
				Lines are simply not drawn past the edge of the frame.

	@constant	kCTLineBreakByTruncatingHead
				Each line is displayed so that the end fits in the frame and the
				missing text is indicated by some kind of ellipsis glyph.

	@constant	kCTLineBreakByTruncatingTail
				Each line is displayed so that the beginning fits in the
				container and the missing text is indicated by some kind of
				ellipsis glyph.

	@constant	kCTLineBreakByTruncatingMiddle
				Each line is displayed so that the beginning and end fit in the
				container and the missing text is indicated by some kind of
				ellipsis glyph in the middle.
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
	kCTLineBreakByWordWrapping = 0;
	kCTLineBreakByCharWrapping = 1;
	kCTLineBreakByClipping = 2;
	kCTLineBreakByTruncatingHead = 3;
	kCTLineBreakByTruncatingTail = 4;
	kCTLineBreakByTruncatingMiddle = 5;
type
	CTLineBreakMode = UInt8;


{!
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    @enum       CTWritingDirection
    @abstract   These constants specify the writing direction

    @constant   kCTWritingDirectionNatural
                The writing direction is algorithmically determined
                using the Unicode Bidirectional Algorithm rules P2 and P3.

    @constant   kCTWritingDirectionLeftToRight
                The writing direction is left to right.

    @constant   kCTWritingDirectionRightToLeft
                The writing direction is right to left.
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	@enum		CTWritingDirection
	@abstract	These constants specify the writing direction

	@constant	kCTWritingDirectionNatural
				The writing direction is algorithmically determined
				using the Unicode Bidirectional Algorithm rules P2 and P3.

	@constant	kCTWritingDirectionLeftToRight
				The writing direction is left to right.

	@constant	kCTWritingDirectionRightToLeft
				The writing direction is right to left.
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
	kCTWritingDirectionNatural = -1;
	kCTWritingDirectionLeftToRight = 0;
	kCTWritingDirectionRightToLeft = 1;
type
	CTWritingDirection = SInt8;


{!
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    @enum       CTParagraphStyleSpecifier
    @abstract   These constants are used to query and modify the CTParagraphStyle
                object.

    @discussion Each specifier has a type and a default value associated with it.
                The type must always be observed when setting or fetching the
                value from the CTParagraphStyle object. In addition, some
                specifiers affect the behavior of both the framesetter and
                the typesetter, and others only affect the behavior of the
                framesetter; this is also noted below.

    @constant   kCTParagraphStyleSpecifierAlignment
                The text alignment. Natural text alignment is realized as
                left or right alignment, depending on the line sweep direction
                of the first script contained in the paragraph.

                Type: CTTextAlignment
                Default: kCTNaturalTextAlignment
                Application: CTFramesetter


    @constant   kCTParagraphStyleSpecifierFirstLineHeadIndent
                The distance in points from the leading margin of a frame to
                the beginning of the paragraph's first line. This value is always
                nonnegative.

                Type: CGFloat
                Default: 0.0
                Application: CTFramesetter


    @constant   kCTParagraphStyleSpecifierHeadIndent
                The distance in points from the leading margin of a text
                container to the beginning of lines other than the first.
                This value is always nonnegative.

                Type: CGFloat
                Default: 0.0
                Application: CTFramesetter


    @constant   kCTParagraphStyleSpecifierTailIndent
                The distance in points from the margin of a frame to the end of
                lines. If positive, this value is the distance from the leading
                margin (for example, the left margin in left-to-right text).
                If 0 or negative, it's the distance from the trailing margin.

                Type: CGFloat
                Default: 0.0
                Application: CTFramesetter


    @constant   kCTParagraphStyleSpecifierTabStops
                The CTTextTab objects, sorted by location, that define the tab
                stops for the paragraph style.

                Type: CFArray of CTTextTabRef
                Default: 12 left-aligned tabs, spaced by 28.0 points
                Application: CTFramesetter, CTTypesetter


    @constant   kCTParagraphStyleSpecifierDefaultTabInterval
                The document-wide default tab interval. Tabs after the last
                specified by kCTParagraphStyleSpecifierTabStops are placed at
                integer multiples of this distance (if positive).

                Type: CGFloat
                Default: 0.0
                Application: CTFramesetter, CTTypesetter


    @constant   kCTParagraphStyleSpecifierLineBreakMode
                The mode that should be used to break lines when laying out
                the paragraph's text.

                Type: CTLineBreakMode
                Default: kCTLineBreakByWordWrapping
                Application: CTFramesetter


    @constant   kCTParagraphStyleSpecifierLineHeightMultiple
                The line height multiple. The natural line height of the
                receiver is multiplied by this factor (if positive) before
                being constrained by minimum and maximum line height.

                Type: CGFloat
                Default: 0.0
                Application: CTFramesetter


    @constant   kCTParagraphStyleSpecifierMaximumLineHeight
                The maximum height that any line in the frame will occupy,
                regardless of the font size or size of any attached graphic.
                Glyphs and graphics exceeding this height will overlap
                neighboring lines. A maximum height of 0 implies
                no line height limit. This value is always nonnegative.

                Type: CGFloat
                Default: 0.0
                Application: CTFramesetter


    @constant   kCTParagraphStyleSpecifierMinimumLineHeight
                The minimum height that any line in the frame will occupy,
                regardless of the font size or size of any attached graphic.
                This value is always nonnegative.

                Type: CGFloat
                Default: 0.0
                Application: CTFramesetter


    @constant   kCTParagraphStyleSpecifierLineSpacing
                Deprecated.
                Use kCTParagraphStyleSpecifierMaximumLineSpacing, kCTParagraphStyleSpecifierMinimumLineSpacing,
                and kCTParagraphStyleSpecifierLineSpacingAdjustment to control
                space between lines.


    @constant   kCTParagraphStyleSpecifierParagraphSpacing
                The space added at the end of the paragraph to separate it from
                the following paragraph. This value is always nonnegative and is
                determined by adding the previous paragraph's
                kCTParagraphStyleSpecifierParagraphSpacing setting and the
                current paragraph's kCTParagraphStyleSpecifierParagraphSpacingBefore
                setting.

                Type: CGFloat
                Default: 0.0
                Application: CTFramesetter


    @constant   kCTParagraphStyleSpecifierParagraphSpacingBefore
                The distance between the paragraph's top and the beginning of
                its text content.

                Type: CGFloat
                Default: 0.0
                Application: CTFramesetter


    @constant   kCTParagraphStyleSpecifierBaseWritingDirection
                The base writing direction of the lines.

                Type: CTWritingDirection
                Default: kCTWritingDirectionNatural
                Application: CTFramesetter, CTTypesetter


    @constant   kCTParagraphStyleSpecifierMaximumLineSpacing
                The maximum space in points between lines within the paragraph
                (commonly known as leading). This value is always
                nonnegative.

                Type: CGFloat
                Default: some large number.
                Application: CTFramesetter
 
 
    @constant   kCTParagraphStyleSpecifierMinimumLineSpacing
                The minimum space in points between lines within the paragraph
                (commonly known as leading). This value is always
                nonnegative.

                Type: CGFloat
                Default: 0.0
                Application: CTFramesetter
 
 
    @constant   kCTParagraphStyleSpecifierLineSpacingAdjustment
                The space in points added between lines within the paragraph
                (commonly known as leading). 

                Type: CGFloat
                Default: 0.0
                Application: CTFramesetter


    @constant   kCTParagraphStyleSpecifierLineBoundsOptions
                The options controlling the alignment of the line edges with
                the leading and trailing margins.

                Type: CTLineBoundsOptions
                Default: 0 (no options)
                Application: CTTypesetter
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	@enum		CTParagraphStyleSpecifier
	@abstract	These constants are used to query and modify the CTParagraphStyle
				object.

	@discussion Each specifier has a type and a default value associated with it.
				The type must always be observed when setting or fetching the
				value from the CTParagraphStyle object. In addition, some
				specifiers affect the behavior of both the framesetter and
				the typesetter, and others only affect the behavior of the
				framesetter; this is also noted below.

	@constant	kCTParagraphStyleSpecifierAlignment
				The text alignment. Natural text alignment is realized as
				left or right alignment, depending on the line sweep direction
				of the first script contained in the paragraph.

				Type: CTTextAlignment
				Default: kCTNaturalTextAlignment
				Application: CTFramesetter


	@constant	kCTParagraphStyleSpecifierFirstLineHeadIndent
				The distance in points from the leading margin of a frame to
				the beginning of the paragraph's first line. This value is always
				nonnegative.

				Type: CGFloat
				Default: 0.0
				Application: CTFramesetter


	@constant	kCTParagraphStyleSpecifierHeadIndent
				The distance in points from the leading margin of a text
				container to the beginning of lines other than the first.
				This value is always nonnegative.

				Type: CGFloat
				Default: 0.0
				Application: CTFramesetter


	@constant	kCTParagraphStyleSpecifierTailIndent
				The distance in points from the margin of a frame to the end of
				lines. If positive, this value is the distance from the leading
				margin (for example, the left margin in left-to-right text).
				If 0 or negative, it's the distance from the trailing margin.

				Type: CGFloat
				Default: 0.0
				Application: CTFramesetter


	@constant	kCTParagraphStyleSpecifierTabStops
				The CTTextTab objects, sorted by location, that define the tab
				stops for the paragraph style.

				Type: CFArray of CTTextTabRef
				Default: 12 left-aligned tabs, spaced by 28.0 points
				Application: CTFramesetter, CTTypesetter


	@constant	kCTParagraphStyleSpecifierDefaultTabInterval
				The document-wide default tab interval. Tabs after the last
				specified by kCTParagraphStyleSpecifierTabStops are placed at
				integer multiples of this distance (if positive).

				Type: CGFloat
				Default: 0.0
				Application: CTFramesetter, CTTypesetter


	@constant	kCTParagraphStyleSpecifierLineBreakMode
				The mode that should be used to break lines when laying out
				the paragraph's text.

				Type: CTLineBreakMode
				Default: kCTLineBreakByWordWrapping
				Application: CTFramesetter


	@constant	kCTParagraphStyleSpecifierLineHeightMultiple
				The line height multiple. The natural line height of the
				receiver is multiplied by this factor (if positive) before
				being constrained by minimum and maximum line height.

				Type: CGFloat
				Default: 0.0
				Application: CTFramesetter


	@constant	kCTParagraphStyleSpecifierMaximumLineHeight
				The maximum height that any line in the frame will occupy,
				regardless of the font size or size of any attached graphic.
				Glyphs and graphics exceeding this height will overlap
				neighboring lines. A maximum height of 0 implies
				no line height limit. This value is always nonnegative.

				Type: CGFloat
				Default: 0.0
				Application: CTFramesetter


	@constant	kCTParagraphStyleSpecifierMinimumLineHeight
				The minimum height that any line in the frame will occupy,
				regardless of the font size or size of any attached graphic.
				This value is always nonnegative.

				Type: CGFloat
				Default: 0.0
				Application: CTFramesetter


	@constant	kCTParagraphStyleSpecifierLineSpacing
				The space in points added between lines within the paragraph
				(commonly known as leading). This value is always
				nonnegative.

				Type: CGFloat
				Default: 0.0
				Application: CTFramesetter


	@constant	kCTParagraphStyleSpecifierParagraphSpacing
				The space added at the end of the paragraph to separate it from
				the following paragraph. This value is always nonnegative and is
				determined by adding the previous paragraph's
				kCTParagraphStyleSpecifierParagraphSpacing setting and the
				current paragraph's kCTParagraphStyleSpecifierParagraphSpacingBefore
				setting.

				Type: CGFloat
				Default: 0.0
				Application: CTFramesetter


	@constant	kCTParagraphStyleSpecifierParagraphSpacingBefore
				The distance between the paragraph's top and the beginning of
				its text content.

				Type: CGFloat
				Default: 0.0
				Application: CTFramesetter


	@constant	kCTParagraphStyleSpecifierBaseWritingDirection
				The base writing direction of the lines.

				Type: CTWritingDirection
				Default: kCTWritingDirectionNatural
				Application: CTFramesetter, CTTypesetter
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
	kCTParagraphStyleSpecifierAlignment = 0;
	kCTParagraphStyleSpecifierFirstLineHeadIndent = 1;
	kCTParagraphStyleSpecifierHeadIndent = 2;
	kCTParagraphStyleSpecifierTailIndent = 3;
	kCTParagraphStyleSpecifierTabStops = 4;
	kCTParagraphStyleSpecifierDefaultTabInterval = 5;
	kCTParagraphStyleSpecifierLineBreakMode = 6;
	kCTParagraphStyleSpecifierLineHeightMultiple = 7;
	kCTParagraphStyleSpecifierMaximumLineHeight = 8;
	kCTParagraphStyleSpecifierMinimumLineHeight = 9;
	kCTParagraphStyleSpecifierLineSpacing = 10;
	kCTParagraphStyleSpecifierParagraphSpacing = 11;
	kCTParagraphStyleSpecifierParagraphSpacingBefore = 12;
	kCTParagraphStyleSpecifierBaseWritingDirection = 13;
	kCTParagraphStyleSpecifierCount = 14;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    kCTParagraphStyleSpecifierMinimumLineSpacing = 15;
    kCTParagraphStyleSpecifierLineSpacingAdjustment = 16;
    kCTParagraphStyleSpecifierLineBoundsOptions = 17;
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
	CTParagraphStyleSpecifier = UInt32;


{!
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    @struct     CTParagraphStyleSetting
    @abstract   This structure is used to alter the paragraph style.

    @field      spec
                The specifier of the setting.

    @field      valueSize
                The size of the value pointed to by the "value" field. This
                must match the size of the value required by the
                CTParagraphStyleSpecifier set in the "spec" field.

    @field      value
                A reference to the value of the setting specified by the
                "spec" field. The value must be in the proper range for the
                spec value. The value must also be at least valueSize.
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	@struct		CTParagraphStyleSetting
	@abstract	This structure is used to alter the paragraph style.

	@field		spec
				The specifier of the setting.

	@field		valueSize
				The size of the value pointed to by the "value" field. This
				must match the size of the value required by the
				CTParagraphStyleSpecifier set in the "spec" field.

	@field		value
				A reference to the value of the setting specified by the
				"spec" field. The value must be in the proper range for the
				spec value. The value must also be at least valueSize.
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
type
	CTParagraphStyleSettingPtr = ^CTParagraphStyleSetting;
	CTParagraphStyleSetting = record
		spec: CTParagraphStyleSpecifier;
{$ifc TARGET_CPU_64}
		__alignment_dummy: UInt32;
{$endc}
		valueSize: size_t;
		value: {const} UnivPtr;
	end;


{ --------------------------------------------------------------------------- }
{ Paragraph Style Creation }
{ --------------------------------------------------------------------------- }

{!
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    @function   CTParagraphStyleCreate
    @abstract   Creates an immutable paragraph style.

    @discussion Using this function is the easiest and most efficient way to
                create a paragraph style. Paragraph styles should be kept
                immutable for totally lock-free operation.

                If an invalid paragraph style setting specifier is passed into
                the "settings" parameter, nothing bad will happen but just don't
                expect to be able to query for this value. This is to allow
                backwards compatibility with style setting specifiers that may
                be introduced in future versions.

    @param      settings
                The settings that you wish to pre-load the paragraph style
                with. If you wish to specify the default set of settings,
                then this parameter may be set to NULL.

    @param      settingCount
                The number of settings that you have specified in the
                "settings" parameter. This must be greater than or equal
                to zero.

    @result     If the paragraph style creation was successful, this function
                will return a valid reference to an immutable CTParagraphStyle
                object. Otherwise, this function will return NULL.
}

function CTParagraphStyleCreate( settings: {const} CTParagraphStyleSettingPtr {can be null}; settingCount: size_t ): CTParagraphStyleRef; external name '_CTParagraphStyleCreate';
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)


{!
    @function   CTParagraphStyleCreateCopy
    @abstract   Creates an immutable copy of a paragraph style.

    @param      paragraphStyle
                The style that you wish to copy. This parameter may not be
                set to NULL.

    @result     If the "paragraphStyle" reference is valid, then this
                function will return valid reference to an immutable
                CTParagraphStyle object that is a copy of the one passed into
                "paragraphStyle".
}

function CTParagraphStyleCreateCopy( paragraphStyle: CTParagraphStyleRef ): CTParagraphStyleRef; external name '_CTParagraphStyleCreateCopy';
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
	@function	CTParagraphStyleCreate
	@abstract	Creates an immutable paragraph style.

	@discussion Using this function is the easiest and most efficient way to
				create a paragraph style. Paragraph styles should be kept
				immutable for totally lock-free operation.

				If an invalid paragraph style setting specifier is passed into
				the "settings" parameter, nothing bad will happen but just don't
				expect to be able to query for this value. This is to allow
				backwards compatibility with style setting specifiers that may
				be introduced in future versions.

	@param		settings
				The settings that you wish to pre-load the paragraph style
				with. If you wish to specify the default set of settings,
				then this parameter may be set to NULL.

	@param		settingCount
				The number of settings that you have specified in the
				"settings" parameter. This must be greater than or equal
				to zero.

	@result		If the paragraph style creation was successful, this function
				will return a valid reference to an immutable CTParagraphStyle
				object. Otherwise, this function will return NULL.
}

function CTParagraphStyleCreate( settings: {const} CTParagraphStyleSettingPtr {can be null}; settingCount: CFIndex ): CTParagraphStyleRef; external name '_CTParagraphStyleCreate';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{!
	@function	CTParagraphStyleCreateCopy
	@abstract	Creates an immutable copy of a paragraph style.

	@param		paragraphStyle
				The style that you wish to copy. This parameter may not be
				set to NULL.

	@result		If the "paragraphStyle" reference is valid, then this
				function will return valid reference to an immutable
				CTParagraphStyle object that is a copy of the one passed into
				"paragraphStyle".
}

function CTParagraphStyleCreateCopy( paragraphStyle: CTParagraphStyleRef ): CTParagraphStyleRef; external name '_CTParagraphStyleCreateCopy';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
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


{ --------------------------------------------------------------------------- }
{ Paragraph Style Access }
{ --------------------------------------------------------------------------- }

{!
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    @function   CTParagraphStyleGetValueForSpecifier
    @abstract   Obtains the current value for a single setting specifier.

    @discussion This function will return the current value of the specifier
                whether or not the user had actually set it. If the user has
                not set it, this function will return the default value.

                If an invalid paragraph style setting specifier is passed into
                the "spec" parameter, nothing bad will happen and the buffer
                value will simply be zeroed out. This is to allow backwards
                compatibility with style setting specifier that may be introduced
                in future versions.

    @param      paragraphStyle
                The paragraph style that you wish to get the value from. This
                parameter may not be set to NULL.

    @param      spec
                The setting specifier that you want to get the value for.

    @param      valueBufferSize
                The size of the buffer pointed to by the "valueBuffer" parameter.
                This value must be at least as large as the size the required by
                the CTParagraphSpecifier value set in the "spec" parameter.

    @param      valueBuffer
                The buffer where the requested setting value will be written
                upon successful completion. The buffer's size needs to be at least
                as large as the value passed into "valueBufferSize". This parameter
                is required and may not be set to NULL.

    @result     This function will return "true" if the valueBuffer had been
                successfully filled. Otherwise, this function will return false,
                indicating that one or more of the parameters is not valid.
}

function CTParagraphStyleGetValueForSpecifier( paragraphStyle: CTParagraphStyleRef; spec: CTParagraphStyleSpecifier; valueBufferSize: size_t; valueBuffer: UnivPtr ): CBool; external name '_CTParagraphStyleGetValueForSpecifier';
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
	@function	CTParagraphStyleGetValueForSpecifier
	@abstract	Obtains the current value for a single setting specifier.

	@discussion This function will return the current value of the specifier
				whether or not	the user had actually set it. If the user has
				not set it, this function will return the default value.

				If an invalid paragraph style setting specifier is passed into
				the "spec" parameter, nothing bad will happen and the buffer
				value will simply be zeroed out. This is to allow backwards
				compatibility with style setting specifier that may be introduced
				in future versions.

	@param		paragraphStyle
				The paragraph style that you wish to get the value from. This
				parameter may not be set to NULL.

	@param		spec
				The setting specifier that you want to get the value for.

	@param		valueBufferSize
				The size of the buffer pointed to by the "valueBuffer" parameter.
				This value must be at least as large as the size the required by
				the CTParagraphSpecifier value set in the "spec" parameter.

	@param		valueBuffer
				The buffer where the requested setting value will be written
				upon successful completion. The buffer's size needs to be at least
				as large as the value passed into "valueBufferSize". This parameter
				is required and may not be set to NULL.

	@result		This function will return "true" if the valueBuffer had been
				successfully filled. Otherwise, this function will return false,
				indicating that one or more of the parameters is not valid.
}

function CTParagraphStyleGetValueForSpecifier( paragraphStyle: CTParagraphStyleRef; spec: CTParagraphStyleSpecifier; valueBufferSize: size_t; valueBuffer: UnivPtr ): CBool; external name '_CTParagraphStyleGetValueForSpecifier';
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
