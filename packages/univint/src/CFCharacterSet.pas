{	CFCharacterSet.h
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	Copyright (c) 1999-2013, Apple Inc. All rights reserved.
}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	Copyright (c) 1999-2009, Apple, Inc. All rights reserved.
}
{   Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, September 2005 }
{   Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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
	Copyright (c) 1999-2005, Apple, Inc. All rights reserved.
=======
	Copyright (c) 1999-2009, Apple, Inc. All rights reserved.
>>>>>>> origin/fixes_2.4
}
{   Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, September 2005 }
{   Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit CFCharacterSet;
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
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
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
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
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
uses MacTypes,CFBase,CFData;
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


{!
	@header CFCharacterSet
        CFCharacterSet represents a set, or a bag, of Unicode characters.
        The API consists of 3 groups:
        1) creation/manipulation of CFCharacterSet instances,
        2) query of a single Unicode character membership,
        and 3) bitmap representation related (reading/writing).
        Conceptually, CFCharacterSet is a 136K byte bitmap array of
        which each bit represents a Unicode code point.  It could
        contain the Unicode characters in ISO 10646 Basic Multilingual
        Plane (BMP) and characters in Plane 1 through Plane 16
        accessible via surrogate paris in the Unicode Transformation
        Format, 16-bit encoding form (UTF-16).  In other words, it can
        store values from 0x00000 to 0x10FFFF in the Unicode
        Transformation Format, 32-bit encoding form (UTF-32).  However,
        in general, how CFCharacterSet stores the information is an
        implementation detail.  Note even CFData used for the external
        bitmap representation rarely has 136K byte.  For detailed
        discussion of the external bitmap representation, refer to the
        comments for CFCharacterSetCreateWithBitmapRepresentation below.
        Note that the existance of non-BMP characters in a character set
        does not imply the membership of the corresponding surrogate
        characters.  For example, a character set with U+10000 does not
        match with U+D800.
}


{!
	@typedef CFCharacterSetRef
	This is the type of a reference to immutable CFCharacterSets.
}
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CFCharacterSetRef = ^__CFCharacterSet; { an opaque type }
	__CFCharacterSet = record end;
=======
	CFCharacterSetRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CFCharacterSetRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CFCharacterSetRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CFCharacterSetRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	CFCharacterSetRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> graemeg/fixes_2_2
=======
	CFCharacterSetRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> origin/fixes_2_2
=======
	CFCharacterSetRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4
	CFCharacterSetRefPtr = ^CFCharacterSetRef;

{!
	@typedef CFMutableCharacterSetRef
	This is the type of a reference to mutable CFMutableCharacterSets.
}
type
	CFMutableCharacterSetRef = CFCharacterSetRef;
	CFMutableCharacterSetRefPtr = ^CFMutableCharacterSetRef;

{!
	@typedef CFCharacterSetPredefinedSet
        Type of the predefined CFCharacterSet selector values.
}
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CFCharacterSetPredefinedSet = CFIndex;
=======
	CFCharacterSetPredefinedSet = SIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	CFCharacterSetPredefinedSet = SIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	CFCharacterSetPredefinedSet = SIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	CFCharacterSetPredefinedSet = SIGNEDLONG;
>>>>>>> origin/cpstrnew
=======
	CFCharacterSetPredefinedSet = SInt32;
>>>>>>> graemeg/fixes_2_2
=======
	CFCharacterSetPredefinedSet = SInt32;
>>>>>>> origin/fixes_2_2
=======
	CFCharacterSetPredefinedSet = SIGNEDLONG;
>>>>>>> origin/fixes_2.4
const
	kCFCharacterSetControl = 1; { Control character set (Unicode General Category Cc and Cf) }
	kCFCharacterSetWhitespace = 2; { Whitespace character set (Unicode General Category Zs and U0009 CHARACTER TABULATION) }
	kCFCharacterSetWhitespaceAndNewline = 3;  { Whitespace and Newline character set (Unicode General Category Z*, U000A ~ U000D, and U0085) }
	kCFCharacterSetDecimalDigit = 4; { Decimal digit character set }
	kCFCharacterSetLetter = 5; { Letter character set (Unicode General Category L* & M*) }
	kCFCharacterSetLowercaseLetter = 6; { Lowercase character set (Unicode General Category Ll) }
	kCFCharacterSetUppercaseLetter = 7; { Uppercase character set (Unicode General Category Lu and Lt) }
	kCFCharacterSetNonBase = 8; { Non-base character set (Unicode General Category M*) }
	kCFCharacterSetDecomposable = 9; { Canonically decomposable character set }
	kCFCharacterSetAlphaNumeric = 10; { Alpha Numeric character set (Unicode General Category L*, M*, & N*) }
	kCFCharacterSetPunctuation = 11; { Punctuation character set (Unicode General Category P*) }
	kCFCharacterSetIllegal = 12; { Illegal character set }
{#if MAC_OS_X_VERSION_10_2 <= MAC_OS_X_VERSION_MAX_ALLOWED}
    kCFCharacterSetCapitalizedLetter = 13; { Titlecase character set (Unicode General Category Lt) }
{#endif}
{#if MAC_OS_X_VERSION_10_3 <= MAC_OS_X_VERSION_MAX_ALLOWED}
    kCFCharacterSetSymbol = 14; { Symbol character set (Unicode General Category S*) }
{#endif}

{!
	@function CFCharacterSetGetTypeID
	Returns the type identifier of all CFCharacterSet instances.
}
function CFCharacterSetGetTypeID: CFTypeID; external name '_CFCharacterSetGetTypeID';

{!
	@function CFCharacterSetGetPredefined
	Returns a predefined CFCharacterSet instance.
	@param theSetIdentifier The CFCharacterSetPredefinedSet selector
                which specifies the predefined character set.  If the
                value is not in CFCharacterSetPredefinedSet, the behavior
                is undefined.
	@result A reference to the predefined immutable CFCharacterSet.
                This instance is owned by CF.
}
function CFCharacterSetGetPredefined( theSetIdentifier: CFCharacterSetPredefinedSet ): CFCharacterSetRef; external name '_CFCharacterSetGetPredefined';

{!
	@function CFCharacterSetCreateWithCharactersInRange
	Creates a new immutable character set with the values from the given range.
	@param alloc The CFAllocator which should be used to allocate
		memory for the array and its storage for values. This
		parameter may be NULL in which case the current default
		CFAllocator is used. If this reference is not a valid
		CFAllocator, the behavior is undefined.
	@param theRange The CFRange which should be used to specify the
                Unicode range the character set is filled with.  It
                accepts the range in 32-bit in the UTF-32 format.  The
                valid character point range is from 0x00000 to 0x10FFFF.
                If the range is outside of the valid Unicode character
                point, the behavior is undefined.
	@result A reference to the new immutable CFCharacterSet.
}
function CFCharacterSetCreateWithCharactersInRange( alloc: CFAllocatorRef; theRange: CFRange ): CFCharacterSetRef; external name '_CFCharacterSetCreateWithCharactersInRange';

{!
	@function CFCharacterSetCreateWithCharactersInString
	Creates a new immutable character set with the values in the given string.
	@param alloc The CFAllocator which should be used to allocate
		memory for the array and its storage for values. This
		parameter may be NULL in which case the current default
		CFAllocator is used. If this reference is not a valid
		CFAllocator, the behavior is undefined.
	@param theString The CFString which should be used to specify
                the Unicode characters the character set is filled with.
                If this parameter is not a valid CFString, the behavior
                is undefined.
        @result A reference to the new immutable CFCharacterSet.
}
function CFCharacterSetCreateWithCharactersInString( alloc: CFAllocatorRef; theString: CFStringRef ): CFCharacterSetRef; external name '_CFCharacterSetCreateWithCharactersInString';

{!
	@function CFCharacterSetCreateWithBitmapRepresentation
	Creates a new immutable character set with the bitmap representtion in the given data.
	@param alloc The CFAllocator which should be used to allocate
		memory for the array and its storage for values. This
		parameter may be NULL in which case the current default
		CFAllocator is used. If this reference is not a valid
		CFAllocator, the behavior is undefined.
	@param theData The CFData which should be used to specify the
                bitmap representation of the Unicode character points
                the character set is filled with.  The bitmap
                representation could contain all the Unicode character
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                range starting from BMP to Plane 16.  The first 8192 bytes
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                of the data represent the BMP range.  The BMP range 8192
=======
                of the data represents the BMP range.  The BMP range 8192
>>>>>>> graemeg/cpstrnew
=======
                of the data represents the BMP range.  The BMP range 8192
>>>>>>> graemeg/cpstrnew
=======
                of the data represents the BMP range.  The BMP range 8192
>>>>>>> graemeg/cpstrnew
=======
                of the data represents the BMP range.  The BMP range 8192
>>>>>>> origin/cpstrnew
                bytes can be followed by zero to sixteen 8192 byte
                bitmaps, each one with the plane index byte prepended.
                For example, the bitmap representing the BMP and Plane 2
                has the size of 16385 bytes (8192 bytes for BMP, 1 byte
                index + 8192 bytes bitmap for Plane 2).  The plane index
=======
=======
>>>>>>> origin/fixes_2_2
                range starting from BMP to Plane 16.  The first 8K bytes
                of the data represents the BMP range.  The BMP range 8K
                bytes can be followed by zero to sixteen 8K byte
                bitmaps, each one with the plane index byte prepended.
                For example, the bitmap representing the BMP and Plane 2
                has the size of 16385 bytes (8K bytes for BMP, 1 byte
                index + 8K bytes bitmap for Plane 2).  The plane index
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
                range starting from BMP to Plane 16.  The first 8192 bytes
                of the data represents the BMP range.  The BMP range 8192
                bytes can be followed by zero to sixteen 8192 byte
                bitmaps, each one with the plane index byte prepended.
                For example, the bitmap representing the BMP and Plane 2
                has the size of 16385 bytes (8192 bytes for BMP, 1 byte
                index + 8192 bytes bitmap for Plane 2).  The plane index
>>>>>>> origin/fixes_2.4
                byte, in this case, contains the integer value two.  If
                this parameter is not a valid CFData or it contains a
                Plane index byte outside of the valid Plane range
                (1 to 16), the behavior is undefined.
        @result A reference to the new immutable CFCharacterSet.
}
function CFCharacterSetCreateWithBitmapRepresentation( alloc: CFAllocatorRef; theData: CFDataRef ): CFCharacterSetRef; external name '_CFCharacterSetCreateWithBitmapRepresentation';

{#if MAC_OS_X_VERSION_10_2 <= MAC_OS_X_VERSION_MAX_ALLOWED}
{!
	@function CFCharacterSetCreateInvertedSet
	Creates a new immutable character set that is the invert of the specified character set.
	@param alloc The CFAllocator which should be used to allocate
			memory for the array and its storage for values. This
			parameter may be NULL in which case the current default
			CFAllocator is used. If this reference is not a valid
			CFAllocator, the behavior is undefined.
	@param theSet The CFCharacterSet which is to be inverted.  If this
                		parameter is not a valid CFCharacterSet, the behavior is
              		undefined.
	@result A reference to the new immutable CFCharacterSet.
}
function CFCharacterSetCreateInvertedSet( alloc: CFAllocatorRef; theSet: CFCharacterSetRef ): CFCharacterSetRef; external name '_CFCharacterSetCreateInvertedSet';

{!
	@function CFCharacterSetIsSupersetOfSet
	Reports whether or not the character set is a superset of the character set specified as the second parameter.
	@param theSet  The character set to be checked for the membership of theOtherSet.
		If this parameter is not a valid CFCharacterSet, the behavior is undefined.
	@param theOtherset  The character set to be checked whether or not it is a subset of theSet.
		If this parameter is not a valid CFCharacterSet, the behavior is undefined.
}
function CFCharacterSetIsSupersetOfSet( theSet: CFCharacterSetRef; theOtherset: CFCharacterSetRef ): Boolean; external name '_CFCharacterSetIsSupersetOfSet';

{!
	@function CFCharacterSetHasMemberInPlane
	Reports whether or not the character set contains at least one member character in the specified plane.
	@param theSet  The character set to be checked for the membership.  If this
		parameter is not a valid CFCharacterSet, the behavior is undefined.
	@param thePlane  The plane number to be checked for the membership.
		The valid value range is from 0 to 16.  If the value is outside of the valid
		plane number range, the behavior is undefined.
}
function CFCharacterSetHasMemberInPlane( theSet: CFCharacterSetRef; thePlane: CFIndex ): Boolean; external name '_CFCharacterSetHasMemberInPlane';
{#endif}

{!
	@function CFCharacterSetCreateMutable
	Creates a new empty mutable character set.
	@param allocator The CFAllocator which should be used to allocate
		memory for the array and its storage for values. This
		parameter may be NULL in which case the current default
		CFAllocator is used. If this reference is not a valid
		CFAllocator, the behavior is undefined.
	@result A reference to the new mutable CFCharacterSet.
}
function CFCharacterSetCreateMutable( alloc: CFAllocatorRef ): CFMutableCharacterSetRef; external name '_CFCharacterSetCreateMutable';

{#if MAC_OS_X_VERSION_10_3 <= MAC_OS_X_VERSION_MAX_ALLOWED}
{!
	@function CFCharacterSetCreateCopy
	Creates a new character set with the values from the given character set.  This function tries to compact the backing store where applicable.
	@param allocator The CFAllocator which should be used to allocate
		memory for the array and its storage for values. This
		parameter may be NULL in which case the current default
		CFAllocator is used. If this reference is not a valid
		CFAllocator, the behavior is undefined.
	@param theSet The CFCharacterSet which is to be copied.  If this
                parameter is not a valid CFCharacterSet, the behavior is
                undefined.
	@result A reference to the new CFCharacterSet.
}
function CFCharacterSetCreateCopy( alloc: CFAllocatorRef; theSet: CFCharacterSetRef ): CFCharacterSetRef; external name '_CFCharacterSetCreateCopy';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
{#endif} { MAC_OS_X_VERSION_10_3 <= MAC_OS_X_VERSION_MAX_ALLOWED }

{!
	@function CFCharacterSetCreateMutableCopy
	Creates a new mutable character set with the values from the given character set.
	@param allocator The CFAllocator which should be used to allocate
		memory for the array and its storage for values. This
		parameter may be NULL in which case the current default
		CFAllocator is used. If this reference is not a valid
		CFAllocator, the behavior is undefined.
	@param theSet The CFCharacterSet which is to be copied.  If this
                parameter is not a valid CFCharacterSet, the behavior is
                undefined.
	@result A reference to the new mutable CFCharacterSet.
}
function CFCharacterSetCreateMutableCopy( alloc: CFAllocatorRef; theSet: CFCharacterSetRef ): CFMutableCharacterSetRef; external name '_CFCharacterSetCreateMutableCopy';

{!
	@function CFCharacterSetIsCharacterMember
	Reports whether or not the Unicode character is in the character set.
	@param theSet The character set to be searched. If this parameter
                is not a valid CFCharacterSet, the behavior is undefined.
	@param theChar The Unicode character for which to test against the
                character set.  Note that this function takes 16-bit Unicode
                character value; hence, it does not support access to the
                non-BMP planes.  
        @result true, if the value is in the character set, otherwise false.
}
function CFCharacterSetIsCharacterMember( theSet: CFCharacterSetRef; theChar: UniChar ): Boolean; external name '_CFCharacterSetIsCharacterMember';

{#if MAC_OS_X_VERSION_10_2 <= MAC_OS_X_VERSION_MAX_ALLOWED}
{!
	@function CFCharacterSetIsLongCharacterMember
	Reports whether or not the UTF-32 character is in the character set.
	@param theSet The character set to be searched. If this parameter
               		 is not a valid CFCharacterSet, the behavior is undefined.
	@param theChar The UTF-32 character for which to test against the
			character set.
        @result true, if the value is in the character set, otherwise false.
}
function CFCharacterSetIsLongCharacterMember( theSet: CFCharacterSetRef; theChar: UTF32Char ): Boolean; external name '_CFCharacterSetIsLongCharacterMember';
{#endif}

{!
	@function CFCharacterSetCreateBitmapRepresentation
	Creates a new immutable data with the bitmap representation from the given character set.
	@param allocator The CFAllocator which should be used to allocate
		memory for the array and its storage for values. This
		parameter may be NULL in which case the current default
		CFAllocator is used. If this reference is not a valid
		CFAllocator, the behavior is undefined.
	@param theSet The CFCharacterSet which is to be used create the
                bitmap representation from.  Refer to the comments for
                CFCharacterSetCreateWithBitmapRepresentation for the
                detailed discussion of the bitmap representation format.
                If this parameter is not a valid CFCharacterSet, the
                behavior is undefined.
	@result A reference to the new immutable CFData.
}
function CFCharacterSetCreateBitmapRepresentation( alloc: CFAllocatorRef; theSet: CFCharacterSetRef ): CFDataRef; external name '_CFCharacterSetCreateBitmapRepresentation';

{!
	@function CFCharacterSetAddCharactersInRange
	Adds the given range to the charaacter set.
	@param theSet The character set to which the range is to be added.
                If this parameter is not a valid mutable CFCharacterSet,
                the behavior is undefined.
        @param theRange The range to add to the character set.  It accepts
                the range in 32-bit in the UTF-32 format.  The valid
                character point range is from 0x00000 to 0x10FFFF.  If the
                range is outside of the valid Unicode character point,
                the behavior is undefined.
}
procedure CFCharacterSetAddCharactersInRange( theSet: CFMutableCharacterSetRef; theRange: CFRange ); external name '_CFCharacterSetAddCharactersInRange';

{!
	@function CFCharacterSetRemoveCharactersInRange
	Removes the given range from the charaacter set.
	@param theSet The character set from which the range is to be
                removed.  If this parameter is not a valid mutable
                CFCharacterSet, the behavior is undefined.
        @param theRange The range to remove from the character set.
                It accepts the range in 32-bit in the UTF-32 format.
                The valid character point range is from 0x00000 to 0x10FFFF.
                If the range is outside of the valid Unicode character point,
                the behavior is undefined.
}
procedure CFCharacterSetRemoveCharactersInRange( theSet: CFMutableCharacterSetRef; theRange: CFRange ); external name '_CFCharacterSetRemoveCharactersInRange';

{!
	@function CFCharacterSetAddCharactersInString
	Adds the characters in the given string to the charaacter set.
	@param theSet The character set to which the characters in the
                string are to be added.  If this parameter is not a
                valid mutable CFCharacterSet, the behavior is undefined.
        @param theString The string to add to the character set.
                If this parameter is not a valid CFString, the behavior
                is undefined.
}
procedure CFCharacterSetAddCharactersInString( theSet: CFMutableCharacterSetRef; theString: CFStringRef ); external name '_CFCharacterSetAddCharactersInString';

{!
	@function CFCharacterSetRemoveCharactersInString
	Removes the characters in the given string from the charaacter set.
	@param theSet The character set from which the characters in the
                string are to be remove.  If this parameter is not a
                valid mutable CFCharacterSet, the behavior is undefined.
        @param theString The string to remove from the character set.
                If this parameter is not a valid CFString, the behavior
                is undefined.
}
procedure CFCharacterSetRemoveCharactersInString( theSet: CFMutableCharacterSetRef; theString: CFStringRef ); external name '_CFCharacterSetRemoveCharactersInString';

{!
	@function CFCharacterSetUnion
	Forms the union with the given character set.
	@param theSet The destination character set into which the
                union of the two character sets is stored.  If this
                parameter is not a valid mutable CFCharacterSet, the
                behavior is undefined.
	@param theOtherSet The character set with which the union is
                formed.  If this parameter is not a valid CFCharacterSet,
                the behavior is undefined.
}
procedure CFCharacterSetUnion( theSet: CFMutableCharacterSetRef; theOtherSet: CFCharacterSetRef ); external name '_CFCharacterSetUnion';

{!
	@function CFCharacterSetIntersect
	Forms the intersection with the given character set.
	@param theSet The destination character set into which the
                intersection of the two character sets is stored.
                If this parameter is not a valid mutable CFCharacterSet,
                the behavior is undefined.
	@param theOtherSet The character set with which the intersection
                is formed.  If this parameter is not a valid CFCharacterSet,
                the behavior is undefined.
}
procedure CFCharacterSetIntersect( theSet: CFMutableCharacterSetRef; theOtherSet: CFCharacterSetRef ); external name '_CFCharacterSetIntersect';

{!
	@function CFCharacterSetInvert
	Inverts the content of the given character set.
	@param theSet The character set to be inverted.
                If this parameter is not a valid mutable CFCharacterSet,
                the behavior is undefined.
}
procedure CFCharacterSetInvert( theSet: CFMutableCharacterSetRef ); external name '_CFCharacterSetInvert';

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
