{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  CTStringAttributes.h
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
 *	CTStringAttributes.h
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

unit CTStringAttributes;
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
>>>>>>> origin/fixes_2.4
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
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
uses MacTypes,CFBase,CFNumber,CFString,CGColor;
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


{ The purpose of this file is to define all the attributes to which
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    CoreText will respond when placed in a CFAttributedString. These
    are left out of other header file on purpose in order to avoid
    layering problems. This file is allowed to include any other header
    file it wants to. }
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	CoreText will respond when placed in a CFAttributedString. These
	are left out of other header file on purpose in order to avoid
	layering problems. This file is allowed to include any other header
	file it wants to. }
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
{ CFAttributedStringRef Attribute Prototypes }
{ --------------------------------------------------------------------------- }

{!
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    @const      kCTFontAttributeName
    @abstract   The font.

    @discussion Value must be a CTFontRef. Default is Helvetica 12.
}

var kCTFontAttributeName: CFStringRef; external name '_kCTFontAttributeName'; (* attribute const *)
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)


{!
    @const      kCTForegroundColorFromContextAttributeName
    @abstract   Never set a foreground color in the CGContext; use what is set as
                the context's fill color.

    @discussion Value must be a CFBooleanRef. Default is false. The reason
                why this exists is because an NSAttributedString defaults to a
                black color if no color attribute is set. This forces CoreText to
                set the color in the context. This will allow developers to
                sidestep this, making CoreText set nothing but font information
                in the CGContext. If set, this attribute also determines the
                color used by kCTUnderlineStyleAttributeName, in which case it
                overrides the foreground color.
}

var kCTForegroundColorFromContextAttributeName: CFStringRef; external name '_kCTForegroundColorFromContextAttributeName'; (* attribute const *)
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)


{!
    @const      kCTKernAttributeName
    @abstract   A kerning adjustment.

    @discussion Value must be a CFNumberRef float. Default is standard kerning.
                The kerning attribute indicate how many points the following
                character should be shifted from its default offset as defined
                by the current character's font in points; a positive kern
                indicates a shift farther along and a negative kern indicates a
                shift closer to the current character. If this attribute is not
                present, standard kerning will be used. If this attribute is
                set to 0.0, no kerning will be done at all.
}

var kCTKernAttributeName: CFStringRef; external name '_kCTKernAttributeName'; (* attribute const *)
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)


{!
    @const      kCTLigatureAttributeName
    @abstract   Controls ligature formation.

    @discussion Value must be a CFNumberRef. Default is int value 1. The ligature
                attribute determines what kinds of ligatures should be used when
                displaying the string. A value of 0 indicates that only ligatures
                essential for proper rendering of text should be used, 1
                indicates that standard ligatures should be used, and 2 indicates
                that all available ligatures should be used. Which ligatures are
                standard depends on the script and possibly the font. Arabic
                text, for example, requires ligatures for many character
                sequences, but has a rich set of additional ligatures that
                combine characters. English text has no essential ligatures, and
                typically has only two standard ligatures, those for "fi" and
                "fl" -- all others being considered more advanced or fancy.

                On iOS releases prior to 6.0 essential ligatures are applied
                if the font contains glyphs for any of U+FB00 through U+FB04 and
                the font lacks AAT or OpenType shaping tables, but as of 6.0
                shaping tables (or the lack thereof) are treated as definitive.
                This character-based shaping will still be performed if this
                attribute is explicitly specified with the default value of 1.
}

var kCTLigatureAttributeName: CFStringRef; external name '_kCTLigatureAttributeName'; (* attribute const *)
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)


{!
    @const      kCTForegroundColorAttributeName
    @abstract   The foreground color.

    @discussion Value must be a CGColorRef. Default value is black.
}

var kCTForegroundColorAttributeName: CFStringRef; external name '_kCTForegroundColorAttributeName'; (* attribute const *)
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)


{!
    @const      kCTParagraphStyleAttributeName
    @abstract   A CTParagraphStyle object which is used to specify things like
                line alignment, tab rulers, writing direction, etc.

    @discussion Value must be a CTParagraphStyleRef. Default is an empty
                CTParagraphStyle object. See CTParagraphStyle.h for more
                information.
}

var kCTParagraphStyleAttributeName: CFStringRef; external name '_kCTParagraphStyleAttributeName'; (* attribute const *)
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)


{!
    @const      kCTStrokeWidthAttributeName
    @abstract   The stroke width.

    @discussion Value must be a CFNumberRef. Default value is 0.0, or no stroke.
                This attribute, interpreted as a percentage of font point size,
                controls the text drawing mode: positive values effect drawing
                with stroke only; negative values are for stroke and fill. A
                typical value for outlined text is 3.0.
}

var kCTStrokeWidthAttributeName: CFStringRef; external name '_kCTStrokeWidthAttributeName'; (* attribute const *)
(* CT_AVAILABLE_STARTING( __MAC_10_6, __IPHONE_3_2) *)


{!
    @const      kCTStrokeColorAttributeName
    @abstract   The stroke color.

    @discussion Value must be a CGColorRef. Default is the foreground color.
}

var kCTStrokeColorAttributeName: CFStringRef; external name '_kCTStrokeColorAttributeName'; (* attribute const *)
(* CT_AVAILABLE_STARTING( __MAC_10_6, __IPHONE_3_2) *)


{!
    @const      kCTUnderlineStyleAttributeName
    @abstract   Allows the setting of an underline to be applied at render
                time.

    @discussion Value must be a CFNumberRef. Default is kCTUnderlineStyleNone.
                Set a value of something other than kCTUnderlineStyleNone to draw
                an underline. In addition, the CTUnderlineStyleModifiers can be
                used to modify the look of the underline. The underline color
                will be determined by the text's foreground color.
}

var kCTUnderlineStyleAttributeName: CFStringRef; external name '_kCTUnderlineStyleAttributeName'; (* attribute const *)
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)


{!
    @const      kCTSuperscriptAttributeName
    @abstract   Controls vertical text positioning.

    @discussion Value must be a CFNumberRef. Default is int value 0. If supported
                by the specified font, a value of 1 enables superscripting and a
                value of -1 enables subscripting.
}

var kCTSuperscriptAttributeName: CFStringRef; external name '_kCTSuperscriptAttributeName'; (* attribute const *)
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)


{!
    @const      kCTUnderlineColorAttributeName
    @abstract   The underline color.

    @discussion Value must be a CGColorRef. Default is the foreground color.
}

var kCTUnderlineColorAttributeName: CFStringRef; external name '_kCTUnderlineColorAttributeName'; (* attribute const *)
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)


{!
    @const      kCTVerticalFormsAttributeName
    @abstract   Controls glyph orientation.

    @discussion Value must be a CFBooleanRef. Default is false. A value of false
                indicates that horizontal glyph forms are to be used, true
                indicates that vertical glyph forms are to be used.
}

var kCTVerticalFormsAttributeName: CFStringRef; external name '_kCTVerticalFormsAttributeName'; (* attribute const *)
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_4_3) *)


{!
    @const      kCTGlyphInfoAttributeName
    @abstract   Allows the use of unencoded glyphs.

    @discussion Value must be a CTGlyphInfoRef. The glyph specified by this
                CTGlyphInfo object is assigned to the entire attribute range,
                provided that its contents match the specified base string and
                that the specified glyph is available in the font specified by
                kCTFontAttributeName. See CTGlyphInfo.h for more information.
}

var kCTGlyphInfoAttributeName: CFStringRef; external name '_kCTGlyphInfoAttributeName'; (* attribute const *)
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)


{!
    @const      kCTCharacterShapeAttributeName
    @abstract   Controls glyph selection.

    @discussion Value must be a CFNumberRef. Default is value is 0 (disabled).
                A non-zero value is interpreted as an SFNT kCharacterShapeType
                selector + 1; see SFNTLayoutTypes.h for selectors. For example,
                an attribute value of 1 corresponds to kTraditionalCharactersSelector.
}

var kCTCharacterShapeAttributeName: CFStringRef; external name '_kCTCharacterShapeAttributeName'; (* attribute const *)
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)


{!
    @const      kCTLanguageAttributeName
    @abstract   Specifies text language.

    @discussion Value must be a CFStringRef containing a locale identifier. Default
                is unset. When this attribute is set to a valid identifier, it will
                be used to select localized glyphs (if supported by the font) and
                locale-specific line breaking rules.
}

var kCTLanguageAttributeName: CFStringRef; external name '_kCTLanguageAttributeName'; (* attribute const *)
(* CT_AVAILABLE_STARTING( __MAC_10_9, __IPHONE_7_0) *)


{!
    @const      kCTRunDelegateAttributeName
    @abstract   Allows customization of certain aspects of a range of text's
                appearance.

    @discussion Value must be a CTRunDelegateRef. The values returned by the
                embedded object for an attribute range apply to each glyph
                resulting from the text in that range. Because an embedded object
                is only a display-time modification, care should be taken to
                avoid applying this attribute to a range of text with complex
                behavior, such as a change of writing direction, combining marks,
                etc. Consequently, it is recommended that this attribute be
                applied to a range containing the single character U+FFFC. See
                CTRunDelegate.h for more information.
}

var kCTRunDelegateAttributeName: CFStringRef; external name '_kCTRunDelegateAttributeName'; (* attribute const *)
(* CT_AVAILABLE_STARTING( __MAC_10_5, __IPHONE_3_2) *)



{!
    @enum       CTUnderlineStyle
    @abstract   Underline style specifiers.

    @discussion These underline type specifiers can be applied to the value set
                with the kCTUnderlineStyleAttributeName attribute to tell
                CoreText that you want a different underline style.
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	@const		kCTFontAttributeName
	@abstract	The font.

	@discussion Value must be a CTFontRef. Default is Helvetica 12.
}

var kCTFontAttributeName: CFStringRef; external name '_kCTFontAttributeName'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{!
	@const		kCTForegroundColorFromContextAttributeName
	@abstract	Never set a foreground color in the CGContext; use what is set as
				the context's fill color.

	@discussion Value must be a CFBooleanRef. Default is false. The reason
				why this exists is because an NSAttributedString defaults to a
				black color if no color attribute is set. This forces CoreText to
				set the color in the context. This will allow developers to
				sidestep this, making CoreText set nothing but font information
				in the CGContext. If set, this attribute also determines the
				color used by kCTUnderlineStyleAttributeName, in which case it
				overrides the foreground color.
}

var kCTForegroundColorFromContextAttributeName: CFStringRef; external name '_kCTForegroundColorFromContextAttributeName'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{!
	@const		kCTKernAttributeName
	@abstract	A kerning adjustment.

	@discussion Value must be a CFNumberRef float. Default is standard kerning.
				The kerning attribute indicate how many points the following
				character should be shifted from its default offset as defined
				by the current character's font in points; a positive kern
				indicates a shift farther along and a negative kern indicates a
				shift closer to the current character. If this attribute is not
				present, standard kerning will be used. If this attribute is
				set to 0.0, no kerning will be done at all.
}

var kCTKernAttributeName: CFStringRef; external name '_kCTKernAttributeName'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{!
	@const		kCTLigatureAttributeName
	@abstract	Controls ligature formation.

	@discussion Value must be a CFNumberRef. Default is int value 1. The ligature
				attribute determines what kinds of ligatures should be used when
				displaying the string. A value of 0 indicates that only ligatures
				essential for proper rendering of text should be used, 1
				indicates that standard ligatures should be used, and 2 indicates
				that all available ligatures should be used. Which ligatures are
				standard depends on the script and possibly the font. Arabic
				text, for example, requires ligatures for many character
				sequences, but has a rich set of additional ligatures that
				combine characters. English text has no essential ligatures, and
				typically has only two standard ligatures, those for "fi" and
				"fl" -- all others being considered more advanced or fancy.
}

var kCTLigatureAttributeName: CFStringRef; external name '_kCTLigatureAttributeName'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{!
	@const		kCTForegroundColorAttributeName
	@abstract	The foreground color.

	@discussion Value must be a CGColorRef. Default value is black.
}

var kCTForegroundColorAttributeName: CFStringRef; external name '_kCTForegroundColorAttributeName'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{!
	@const		kCTParagraphStyleAttributeName
	@abstract	A CTParagraphStyle object which is used to specify things like
				line alignment, tab rulers, writing direction, etc.

	@discussion Value must be a CTParagraphStyleRef. Default is an empty
				CTParagraphStyle object. See CTParagraphStyle.h for more
				information.
}

var kCTParagraphStyleAttributeName: CFStringRef; external name '_kCTParagraphStyleAttributeName'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{!
	@const		kCTStrokeWidthAttributeName
	@abstract	The stroke width.

	@discussion Value must be a CFNumberRef. Default value is 0.0, or no stroke.
				This attribute, interpreted as a percentage of font point size,
				controls the text drawing mode: positive values effect drawing
				with stroke only; negative values are for stroke and fill. A
				typical value for outlined text is 3.0.
}

var kCTStrokeWidthAttributeName: CFStringRef; external name '_kCTStrokeWidthAttributeName'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)


{!
	@const		kCTStrokeColorAttributeName
	@abstract	The stroke color.

	@discussion Value must be a CGColorRef. Default is the foreground color.
}

var kCTStrokeColorAttributeName: CFStringRef; external name '_kCTStrokeColorAttributeName'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)


{!
	@const		kCTUnderlineStyleAttributeName
	@abstract	Allows the setting of an underline to be applied at render
				time.

	@discussion Value must be a CFNumberRef. Default is kCTUnderlineStyleNone.
				Set a value of something other than kCTUnderlineStyleNone to draw
				an underline. In addition, the CTUnderlineStyleModifiers can be
				used to modify the look of the underline. The underline color
				will be determined by the text's foreground color.
}

var kCTUnderlineStyleAttributeName: CFStringRef; external name '_kCTUnderlineStyleAttributeName'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{!
	@const		kCTSuperscriptAttributeName
	@abstract	Controls vertical text positioning.

	@discussion Value must be a CFNumberRef. Default is int value 0. If supported
				by the specified font, a value of 1 enables superscripting and a
				value of -1 enables subscripting.
}

var kCTSuperscriptAttributeName: CFStringRef; external name '_kCTSuperscriptAttributeName'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{!
	@const		kCTUnderlineColorAttributeName
	@abstract	The underline color.

	@discussion Value must be a CGColorRef. Default is the foreground color.
}

var kCTUnderlineColorAttributeName: CFStringRef; external name '_kCTUnderlineColorAttributeName'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{!
	@const		kCTVerticalFormsAttributeName
	@abstract	Controls glyph orientation.

	@discussion	Value must be a CFBooleanRef. Default is false. A value of false
				indicates that horizontal glyph forms are to be used, true
				indicates that vertical glyph forms are to be used.
}

var kCTVerticalFormsAttributeName: CFStringRef; external name '_kCTVerticalFormsAttributeName'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{!
	@const		kCTGlyphInfoAttributeName
	@abstract	Allows the use of unencoded glyphs.

	@discussion Value must be a CTGlyphInfoRef. The glyph specified by this
				CTGlyphInfo object is assigned to the entire attribute range,
				provided that its contents match the specified base string and
				that the specified glyph is available in the font specified by
				kCTFontAttributeName. See CTGlyphInfo.h for more information.
}

var kCTGlyphInfoAttributeName: CFStringRef; external name '_kCTGlyphInfoAttributeName'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{!
	@const		kCTCharacterShapeAttributeName
	@abstract	Controls glyph selection.

	@discussion	Value must be a CFNumberRef. Default is value is 0 (disabled).
				A non-zero value is interpreted as Apple Type Services
				kCharacterShapeType selector + 1; see <ATS/SFNTLayoutTypes.h>
				for selectors. For example, an attribute value of 1 corresponds
				to kTraditionalCharactersSelector.
}

var kCTCharacterShapeAttributeName: CFStringRef; external name '_kCTCharacterShapeAttributeName'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{!
	@enum		CTUnderlineStyle
	@abstract	Underline style specifiers.

	@discussion These underline type specifiers can be applied to the value set
				with the kCTUnderlineStyleAttributeName attribute to tell
				CoreText that you want a different underline style.
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
	kCTUnderlineStyleNone = $00;
	kCTUnderlineStyleSingle = $01;
	kCTUnderlineStyleThick = $02;
	kCTUnderlineStyleDouble = $09;
type
	CTUnderlineStyle = SInt32;


{!
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    @enum       CTUnderlineStyleModifiers
    @abstract   Underline style modifiers.

    @discussion Set these bits with the CTUnderlineStyle that you set with the
                kCTUnderlineStyleAttributeName attribute to modify how the
                underline will be drawn.
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	@enum		CTUnderlineStyleModifiers
	@abstract	Underline style modifiers.

	@discussion Set these bits with the CTUnderlineStyle that you set with the
				kCTUnderlineStyleAttributeName attribute to modify how the
				underline will be drawn.
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
	kCTUnderlinePatternSolid = $0000;
	kCTUnderlinePatternDot = $0100;
	kCTUnderlinePatternDash = $0200;
	kCTUnderlinePatternDashDot = $0300;
	kCTUnderlinePatternDashDotDot = $0400;
type
	CTUnderlineStyleModifiers = SInt32;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{!
    @const      kCTBaselineClassAttributeName
    @abstract   Key to reference a baseline class override.

    @discussion Value must be one of the kCTBaselineClass constants. Normally,
                glyphs on the line will be assigned baseline classes according to
                the 'bsln' or 'BASE' table in the font. This attribute may be
                used to change this assignment.

    @seealso    kCTBaselineClassRoman
    @seealso    kCTBaselineClassIdeographicCentered
    @seealso    kCTBaselineClassIdeographicLow
    @seealso    kCTBaselineClassIdeographicHigh
    @seealso    kCTBaselineClassHanging
    @seealso    kCTBaselineClassMath
}

var kCTBaselineClassAttributeName: CFStringRef; external name '_kCTBaselineClassAttributeName'; (* attribute const *)
(* CT_AVAILABLE_STARTING( __MAC_10_8, __IPHONE_6_0) *)


{!
    @const      kCTBaselineInfoAttributeName
    @abstract   Key to reference a baseline info dictionary.

    @discussion Value must be a CFDictionaryRef. Normally, baseline offsets will
                be assigned based on the 'bsln' or 'BASE' table in the font. This
                attribute may be used to assign different offsets. Each key in
                the dictionary is one of the kCTBaselineClass constants and the
                value is a CFNumberRef of the baseline offset in points. You only
                need to specify the offsets you wish to change.

    @seealso    kCTBaselineClassRoman
    @seealso    kCTBaselineClassIdeographicCentered
    @seealso    kCTBaselineClassIdeographicLow
    @seealso    kCTBaselineClassIdeographicHigh
    @seealso    kCTBaselineClassHanging
    @seealso    kCTBaselineClassMath
}

var kCTBaselineInfoAttributeName: CFStringRef; external name '_kCTBaselineInfoAttributeName'; (* attribute const *)
(* CT_AVAILABLE_STARTING( __MAC_10_8, __IPHONE_6_0) *)


{!
    @const      kCTBaselineReferenceInfoAttributeName
    @abstract   Key to reference a baseline info dictionary for the reference baseline.

    @discussion Value must be a CFDictionaryRef. All glyphs in a run are assigned
                a baseline class and then aligned to the offset for that class in
                the reference baseline baseline info. See the discussion of
                kCTBaselineInfoAttributeName for information about the contents
                of the dictionary. You can also use the kCTBaselineReferenceFont
                key to specify that the baseline offsets of a particular
                CTFontRef should be used as the reference offsets.

    @seealso    kCTBaselineClassRoman
    @seealso    kCTBaselineClassIdeographicCentered
    @seealso    kCTBaselineClassIdeographicLow
    @seealso    kCTBaselineClassIdeographicHigh
    @seealso    kCTBaselineClassHanging
    @seealso    kCTBaselineClassMath
    @seealso    kCTBaselineReferenceFont
}

var kCTBaselineReferenceInfoAttributeName: CFStringRef; external name '_kCTBaselineReferenceInfoAttributeName'; (* attribute const *)
(* CT_AVAILABLE_STARTING( __MAC_10_8, __IPHONE_6_0) *)


{!
    @const      kCTWritingDirectionAttributeName
    @abstract   Specifies a bidirectional override or embedding.

    @discussion Value must be a CFArray of CFNumberRefs, each of which should
                have a value of either kCTWritingDirectionLeftToRight or
                kCTWritingDirectionRightToLeft, plus one of
                kCTWritingDirectionEmbedding or kCTWritingDirectionOverride.
                This array represents a sequence of nested bidirectional
                embeddings or overrides, in order from outermost to innermost,
                with (kCTWritingDirectionLeftToRight | kCTTextWritingDirectionEmbedding)
                corresponding to a LRE/PDF pair in plain text or
                <span dir="ltr"></span> in HTML, (kCTWritingDirectionRightToLeft
                | kCTTextWritingDirectionEmbedding) corresponding to a RLE/PDF
                pair in plain text or a <span dir="rtl"></span> in HTML,
                (kCTWritingDirectionLeftToRight | kCTTextWritingDirectionOverride)
                corresponding to a LRO/PDF pair in plain text or
                <bdo dir="ltr"></span> in HTML, and (kCTWritingDirectionRightToLeft
                | kCTTextWritingDirectionOverride) corresponding to a RLO/PDF
                pair in plain text or <bdo dir="rtl"></span> in HTML.

    @seealso    kCTWritingDirectionLeftToRight
    @seealso    kCTWritingDirectionRightToLeft
    @seealso    kCTWritingDirectionEmbedding
    @seealso    kCTWritingDirectionOverride
}

var kCTWritingDirectionAttributeName: CFStringRef; external name '_kCTWritingDirectionAttributeName'; (* attribute const *)
(* CT_AVAILABLE_STARTING( __MAC_10_8, __IPHONE_6_0) *)


{!
    @abstract   Additional values for use with kCTWritingDirectionAttributeName
                in combination with kCTWritingDirectionLeftToRight or
                kCTWritingDirectionRightToLeft.

    @seealso    kCTWritingDirectionAttributeName
    @seealso    kCTWritingDirectionLeftToRight
    @seealso    kCTWritingDirectionRightToLeft
}

const
	kCTWritingDirectionEmbedding = 0 shl 1;
	kCTWritingDirectionOverride = 1 shl 1;

=======
{$endc} {TARGET_OS_MAC}
>>>>>>> graemeg/cpstrnew
=======
{$endc} {TARGET_OS_MAC}
>>>>>>> graemeg/cpstrnew
=======
{$endc} {TARGET_OS_MAC}
>>>>>>> graemeg/cpstrnew
=======
{$endc} {TARGET_OS_MAC}
>>>>>>> origin/cpstrnew
=======
{$endc} {TARGET_OS_MAC}
>>>>>>> origin/fixes_2.4
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
