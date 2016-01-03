{	CFAttributedString.h
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	Copyright (c) 2004-2013, Apple Inc. All rights reserved.
}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	Copyright (c) 2004-2009, Apple Inc. All rights reserved.
}
{       Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Gorazd Krosl, <gorazd_1957@yahoo.ca>, October 2009 }
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
	Copyright (c) 2004-2005, Apple, Inc. All rights reserved.
=======
	Copyright (c) 2004-2009, Apple Inc. All rights reserved.
>>>>>>> origin/fixes_2.4
}
{       Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Gorazd Krosl, <gorazd_1957@yahoo.ca>, October 2009 }
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

unit CFAttributedString;
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
uses MacTypes,CFBase,CFString,CFDictionary;
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


{! @header CFAttributedString
<<<<<<< HEAD
<<<<<<< HEAD
Instances of CFAttributedString manage character strings and associated sets of attributes (for example, font and kerning) that apply to individual characters or ranges of characters in the string. CFAttributedString as defined in CoreFoundation provides the basic container functionality, while higher levels provide definitions for standard attributes, their values, and additional behaviors involving these. 

CFAttributedString is not a "subclass" of CFString; that is, it does not respond to CFString function calls. CFAttributedString conceptually contains a CFString to which it applies attributes. This protects users of attributed strings from ambiguities caused by the semantic differences between simple and attributed strings.
=======
Instance of CFAttributedString manage character strings and associated sets of attributes (for example, font and kerning) that apply to individual characters or ranges of characters in the string. CFAttributedString as defined in CoreFoundation provides the basic container functionality, while higher levels provide definitions for standard attributes, their values, and additional behaviors involving these. 

CFAttributedString is not a "subclass" of CFString; that is, it does not respond to CFString function calls. CFAttributedString conceptually contains a CFString to which it applies attributes. This protects users of attributed strings from ambiguities caused by the semantic differences between simple and attributed string.
>>>>>>> graemeg/fixes_2_2
=======
Instance of CFAttributedString manage character strings and associated sets of attributes (for example, font and kerning) that apply to individual characters or ranges of characters in the string. CFAttributedString as defined in CoreFoundation provides the basic container functionality, while higher levels provide definitions for standard attributes, their values, and additional behaviors involving these. 

CFAttributedString is not a "subclass" of CFString; that is, it does not respond to CFString function calls. CFAttributedString conceptually contains a CFString to which it applies attributes. This protects users of attributed strings from ambiguities caused by the semantic differences between simple and attributed string.
>>>>>>> origin/fixes_2_2

Attributes are identified by key/value pairs stored in CFDictionaryRefs. Keys must be CFStrings, while the values are arbitrary CFTypeRefs.
}


{ CFAttributedString comes in immutable and mutable flavors.
}
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CFAttributedStringRef = ^__CFAttributedString; { an opaque type }
	__CFAttributedString = record end;
	CFMutableAttributedStringRef = ^__CFMutableAttributedString; { an opaque type }
	__CFMutableAttributedString = record end;
=======
	CFAttributedStringRef = ^SInt32; { an opaque type }
	CFMutableAttributedStringRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CFAttributedStringRef = ^SInt32; { an opaque type }
	CFMutableAttributedStringRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CFAttributedStringRef = ^SInt32; { an opaque type }
	CFMutableAttributedStringRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CFAttributedStringRef = ^SInt32; { an opaque type }
	CFMutableAttributedStringRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	CFAttributedStringRef = ^SInt32; { an opaque 32-bit type }
	CFMutableAttributedStringRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> graemeg/fixes_2_2
=======
	CFAttributedStringRef = ^SInt32; { an opaque 32-bit type }
	CFMutableAttributedStringRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> origin/fixes_2_2
=======
	CFAttributedStringRef = ^SInt32; { an opaque type }
	CFMutableAttributedStringRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4

{! @function CFAttributedStringGetTypeID
Returns the type identifier of all CFAttributedString instances.
}
function CFAttributedStringGetTypeID: CFTypeID; external name '_CFAttributedStringGetTypeID';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{** CFAttributedString **}

{! @function CFAttributedStringCreate
Creates an attributed string with the specified string and attributes (both copied).
}
function CFAttributedStringCreate( alloc: CFAllocatorRef; str: CFStringRef; attributes: CFDictionaryRef ): CFAttributedStringRef; external name '_CFAttributedStringCreate';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{! @function CFAttributedStringCreateWithSubstring
Creates a sub-attributed string from the specified range. It's a programming error for range to specify characters outside the bounds of aStr.
}
function CFAttributedStringCreateWithSubstring( alloc: CFAllocatorRef; aStr: CFAttributedStringRef; range: CFRange ): CFAttributedStringRef; external name '_CFAttributedStringCreateWithSubstring';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{! @function CFAttributedStringCreateCopy
Creates an immutable attributed string copy.
}
function CFAttributedStringCreateCopy( alloc: CFAllocatorRef; aStr: CFAttributedStringRef ): CFAttributedStringRef; external name '_CFAttributedStringCreateCopy';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{! @function CFAttributedStringGetString
Returns the string for the attributed string. For performance reasons, this will often point at the backing store of the attributed string, and it might change if the attributed string is edited.  However, this is an implementation detail, and definitely not something that should be counted on.
}
function CFAttributedStringGetString( aStr: CFAttributedStringRef ): CFStringRef; external name '_CFAttributedStringGetString';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{! @function CFAttributedStringGetLength
Returns the length of the attributed string in characters; same as CFStringGetLength(CFAttributedStringGetString(aStr))
}
function CFAttributedStringGetLength( aStr: CFAttributedStringRef ): CFIndex; external name '_CFAttributedStringGetLength';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{! @function CFAttributedStringGetAttributes
Returns the attributes at the specified location. If effectiveRange is not NULL, upon return *effectiveRange contains a range over which the exact same set of attributes apply. Note that for performance reasons, the returned effectiveRange is not necessarily the maximal range - for that, use CFAttributedStringGetAttributesAndLongestEffectiveRange().  It's a programming error for loc to specify a location outside the bounds of the attributed string.

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Note that the returned attribute dictionary might change in unpredictable ways from under the caller if the attributed string is edited after this call. If you wish to hang on to the dictionary long-term, you should make an actual copy of it rather than just retaining it.  Also, no assumptions should be made about the relationship of the actual CFDictionaryRef returned by this call and the dictionary originally used to set the attributes, other than the fact that the values stored in the dictionary will be identical (that is, ==) to those originally specified.
=======
Note that the returned attribute dictionary might change in unpredicatable ways from under the caller if the attributed string is edited after this call. If you wish to hang on to the dictionary long-term, you should make an actual copy of it rather than just retaining it.  Also, no assumptions should be made about the relationship of the actual CFDictionaryRef returned by this call and the dictionary originally used to set the attributes, other than the fact that the values stored in the dictionary will be identical (that is, ==) to those originally specified.
>>>>>>> graemeg/fixes_2_2
=======
Note that the returned attribute dictionary might change in unpredicatable ways from under the caller if the attributed string is edited after this call. If you wish to hang on to the dictionary long-term, you should make an actual copy of it rather than just retaining it.  Also, no assumptions should be made about the relationship of the actual CFDictionaryRef returned by this call and the dictionary originally used to set the attributes, other than the fact that the values stored in the dictionary will be identical (that is, ==) to those originally specified.
>>>>>>> origin/fixes_2_2
=======
Note that the returned attribute dictionary might change in unpredictable ways from under the caller if the attributed string is edited after this call. If you wish to hang on to the dictionary long-term, you should make an actual copy of it rather than just retaining it.  Also, no assumptions should be made about the relationship of the actual CFDictionaryRef returned by this call and the dictionary originally used to set the attributes, other than the fact that the values stored in the dictionary will be identical (that is, ==) to those originally specified.
>>>>>>> origin/fixes_2.4
}
function CFAttributedStringGetAttributes( aStr: CFAttributedStringRef; loc: CFIndex; effectiveRange: CFRangePtr ): CFDictionaryRef; external name '_CFAttributedStringGetAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{! @function CFAttributedStringGetAttribute
Returns the value of a single attribute at the specified location. If the specified attribute doesn't exist at the location, returns NULL. If effectiveRange is not NULL, upon return *effectiveRange contains a range over which the exact same attribute value applies. Note that for performance reasons, the returned effectiveRange is not necessarily the maximal range - for that, use CFAttributedStringGetAttributeAndLongestEffectiveRange(). It's a programming error for loc to specify a location outside the bounds of the attributed string.
}
function CFAttributedStringGetAttribute( aStr: CFAttributedStringRef; loc: CFIndex; attrName: CFStringRef; effectiveRange: CFRangePtr ): CFTypeRef; external name '_CFAttributedStringGetAttribute';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{! @function CFAttributedStringGetAttributesAndLongestEffectiveRange
Returns the attributes at the specified location. If longestEffectiveRange is not NULL, upon return *longestEffectiveRange contains the maximal range within inRange over which the exact same set of attributes apply. The returned range is clipped to inRange. It's a programming error for loc or inRange to specify locations outside the bounds of the attributed string.
}
function CFAttributedStringGetAttributesAndLongestEffectiveRange( aStr: CFAttributedStringRef; loc: CFIndex; inRange: CFRange; longestEffectiveRange: CFRangePtr ): CFDictionaryRef; external name '_CFAttributedStringGetAttributesAndLongestEffectiveRange';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{! @function CFAttributedStringGetAttributeAndLongestEffectiveRange
Returns the value of a single attribute at the specified location. If longestEffectiveRange is not NULL, upon return *longestEffectiveRange contains the maximal range within inRange over which the exact same attribute value applies. The returned range is clipped to inRange. It's a programming error for loc or inRange to specify locations outside the bounds of the attributed string.
}
function CFAttributedStringGetAttributeAndLongestEffectiveRange( aStr: CFAttributedStringRef; loc: CFIndex; attrName: CFStringRef; inRange: CFRange; longestEffectiveRange: CFRangePtr ): CFTypeRef; external name '_CFAttributedStringGetAttributeAndLongestEffectiveRange';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{** CFMutableAttributedString **}

{! @function CFAttributedStringCreateMutableCopy
Creates a mutable attributed string copy. maxLength, if not 0, is a hard bound on the length of the attributed string; exceeding this size limit during any editing operation is a programming error. If 0, there is no limit on the length.
}
function CFAttributedStringCreateMutableCopy( alloc: CFAllocatorRef; maxLength: CFIndex; aStr: CFAttributedStringRef ): CFMutableAttributedStringRef; external name '_CFAttributedStringCreateMutableCopy';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{! @function CFAttributedStringCreateMutable
Creates a mutable empty attributed string. maxLength, if not 0, is a hard bound on the length of the attributed string; exceeding this size limit during any editing operation is a programming error. If 0, there is no limit on the length.
}
function CFAttributedStringCreateMutable( alloc: CFAllocatorRef; maxLength: CFIndex ): CFMutableAttributedStringRef; external name '_CFAttributedStringCreateMutable';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{! @function CFAttributedStringReplaceString
Modifies the string for the attributed string, much like CFStringReplace().  It's an error for range to specify characters outside the bounds of aStr. 

(Note: This function is a convenience on CFAttributedStringGetMutableString(); however, until CFAttributedStringGetMutableString() is implemented, it remains the only way to edit the string of the attributed string.)
}
procedure CFAttributedStringReplaceString( aStr: CFMutableAttributedStringRef; range: CFRange; replacement: CFStringRef ); external name '_CFAttributedStringReplaceString';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{! @function CFAttributedStringGetMutableString
Gets the string for the attributed string as a mutable string, allowing editing the character contents of the string as if it were an CFMutableString. Attributes corresponding to the edited range are appropriately modified. If, as a result of the edit, new characters are introduced into the string, they inherit the attributes of the first replaced character from range. If no existing characters are replaced by the edit, the new characters inherit the attributes of the character preceding range if it has any, otherwise of the character following range. If the initial string is empty, the attributes for the new characters are also empty.

(Note: This function is not yet implemented and will return NULL except for toll-free bridged instances.)
}
function CFAttributedStringGetMutableString( aStr: CFMutableAttributedStringRef ): CFMutableStringRef; external name '_CFAttributedStringGetMutableString';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{! @function CFAttributedStringSetAttributes
Sets the value of multiple attributes over the specified range, which should be valid. If clearOtherAttributes is false, existing attributes (which aren't being replaced) are left alone; otherwise they are cleared. The dictionary should be setup for "usual" CF type usage --- CFString keys, and arbitrary CFType values. Note that after this call, further mutations to the replacement dictionary argument by the caller will not affect the contents of the attributed string.
}
procedure CFAttributedStringSetAttributes( aStr: CFMutableAttributedStringRef; range: CFRange; replacement: CFDictionaryRef; clearOtherAttributes: Boolean ); external name '_CFAttributedStringSetAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{! @function CFAttributedStringSetAttribute
Sets the value of a single attribute over the specified range, which should be valid. value should not be NULL. 
}
procedure CFAttributedStringSetAttribute( aStr: CFMutableAttributedStringRef; range: CFRange; attrName: CFStringRef; value: CFTypeRef ); external name '_CFAttributedStringSetAttribute';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{! @function CFAttributedStringRemoveAttribute
Removes the value of a single attribute over the specified range, which should be valid. It's OK for the attribute not the exist over the specified range.
}
procedure CFAttributedStringRemoveAttribute( aStr: CFMutableAttributedStringRef; range: CFRange; attrName: CFStringRef ); external name '_CFAttributedStringRemoveAttribute';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{! @function CFAttributedStringReplaceAttributedString
Replaces the attributed substring over the specified range with the attributed string specified in replacement. range should be valid. To delete a range of the attributed string, call CFAttributedStringReplaceString() with empty string and specified range. 
}
procedure CFAttributedStringReplaceAttributedString( aStr: CFMutableAttributedStringRef; range: CFRange; replacement: CFAttributedStringRef ); external name '_CFAttributedStringReplaceAttributedString';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{! @function CFAttributedStringBeginEditing
In cases where attributed string might do a bunch of work to assure self-consistency, CFAttributedStringBeginEditing/CFAttributedStringEndEditing allow disabling that to allow deferring and coalescing any work. It's a good idea to call these around a set of related mutation calls which don't require the string to be in consistent state in between. These calls can be nested. 
}
procedure CFAttributedStringBeginEditing( aStr: CFMutableAttributedStringRef ); external name '_CFAttributedStringBeginEditing';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{! @function CFAttributedStringEndEditing
In cases where attributed string might do a bunch of work to assure self-consistency, CFAttributedStringBeginEditing/CFAttributedStringEndEditing allow disabling that to allow deferring and coalescing any work. It's a good idea to call these around a set of related mutation calls which don't require the string to be in consistent state in between. These calls can be nested. 
}
procedure CFAttributedStringEndEditing( aStr: CFMutableAttributedStringRef ); external name '_CFAttributedStringEndEditing';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

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
