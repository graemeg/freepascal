{	CFNumberFormatter.h
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	Copyright (c) 2003-2013, Apple Inc. All rights reserved.
}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	Copyright (c) 2003-2009, Apple Inc. All rights reserved.
}
{   Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, 2004 }
{   Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, September 2005 }
{   Pascal Translation Updated:  Gorazd Krosl, <gorazd_1957@yahoo.ca>, October 2009 }
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
	Copyright (c) 2003-2005, Apple, Inc. All rights reserved.
=======
	Copyright (c) 2003-2009, Apple Inc. All rights reserved.
>>>>>>> origin/fixes_2.4
}
{   Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, 2004 }
{   Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, September 2005 }
{   Pascal Translation Updated:  Gorazd Krosl, <gorazd_1957@yahoo.ca>, October 2009 }
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

unit CFNumberFormatter;
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
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
>>>>>>> graemeg/cpstrnew
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
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
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
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
=======
{$elsec}
>>>>>>> graemeg/cpstrnew
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
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
uses MacTypes,CFBase,CFNumber,CFLocale;
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


{#if MAC_OS_X_VERSION_MAX_ALLOWED >= MAC_OS_X_VERSION_10_3}


type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CFNumberFormatterRef = ^__CFNumberFormatter; { an opaque type }
	__CFNumberFormatter = record end;
=======
	CFNumberFormatterRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CFNumberFormatterRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CFNumberFormatterRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CFNumberFormatterRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	CFNumberFormatterRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> graemeg/fixes_2_2
=======
	CFNumberFormatterRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> origin/fixes_2_2
=======
	CFNumberFormatterRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4

// CFNumberFormatters are not thread-safe.  Do not use one from multiple threads!

function CFNumberFormatterGetTypeID: CFTypeID; external name '_CFNumberFormatterGetTypeID';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

// number format styles
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CFNumberFormatterStyle = CFIndex;
=======
	CFNumberFormatterStyle = SIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	CFNumberFormatterStyle = SIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	CFNumberFormatterStyle = SIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	CFNumberFormatterStyle = SIGNEDLONG;
>>>>>>> origin/cpstrnew
=======
	CFNumberFormatterStyle = SInt32;
>>>>>>> graemeg/fixes_2_2
=======
	CFNumberFormatterStyle = SInt32;
>>>>>>> origin/fixes_2_2
=======
	CFNumberFormatterStyle = SIGNEDLONG;
>>>>>>> origin/fixes_2.4
const
	kCFNumberFormatterNoStyle = 0;
	kCFNumberFormatterDecimalStyle = 1;
	kCFNumberFormatterCurrencyStyle = 2;
	kCFNumberFormatterPercentStyle = 3;
	kCFNumberFormatterScientificStyle = 4;
	kCFNumberFormatterSpellOutStyle = 5;


function CFNumberFormatterCreate( allocator: CFAllocatorRef; locale: CFLocaleRef; style: CFNumberFormatterStyle ): CFNumberFormatterRef; external name '_CFNumberFormatterCreate';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
	// Returns a CFNumberFormatter, localized to the given locale, which
	// will format numbers to the given style.

function CFNumberFormatterGetLocale( formatter: CFNumberFormatterRef ): CFLocaleRef; external name '_CFNumberFormatterGetLocale';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

function CFNumberFormatterGetStyle( formatter: CFNumberFormatterRef ): CFNumberFormatterStyle; external name '_CFNumberFormatterGetStyle';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
	// Get the properties with which the number formatter was created.

function CFNumberFormatterGetFormat( formatter: CFNumberFormatterRef ): CFStringRef; external name '_CFNumberFormatterGetFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

procedure CFNumberFormatterSetFormat( formatter: CFNumberFormatterRef; formatString: CFStringRef ); external name '_CFNumberFormatterSetFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
	// Set the format description string of the number formatter.  This
	// overrides the style settings.  The format of the format string
	// is as defined by the ICU library, and is similar to that found
<<<<<<< HEAD
<<<<<<< HEAD
	// in Microsoft Excel and NSNumberFormatter.
=======
	// in Microsoft Excel and NSNumberFormatter (and Java I believe).
>>>>>>> graemeg/fixes_2_2
=======
	// in Microsoft Excel and NSNumberFormatter (and Java I believe).
>>>>>>> origin/fixes_2_2
	// The number formatter starts with a default format string defined
	// by the style argument with which it was created.


function CFNumberFormatterCreateStringWithNumber( allocator: CFAllocatorRef; formatter: CFNumberFormatterRef; number: CFNumberRef ): CFStringRef; external name '_CFNumberFormatterCreateStringWithNumber';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

function CFNumberFormatterCreateStringWithValue( allocator: CFAllocatorRef; formatter: CFNumberFormatterRef; numberType: CFNumberType; valuePtr: {const} UnivPtr ): CFStringRef; external name '_CFNumberFormatterCreateStringWithValue';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
	// Create a string representation of the given number or value
	// using the current state of the number formatter.


type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CFNumberFormatterOptionFlags = CFOptionFlags;
=======
	CFNumberFormatterOptionFlags = UNSIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	CFNumberFormatterOptionFlags = UNSIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	CFNumberFormatterOptionFlags = UNSIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	CFNumberFormatterOptionFlags = UNSIGNEDLONG;
>>>>>>> origin/cpstrnew
=======
	CFNumberFormatterOptionFlags = SInt32;
>>>>>>> graemeg/fixes_2_2
=======
	CFNumberFormatterOptionFlags = SInt32;
>>>>>>> origin/fixes_2_2
=======
	CFNumberFormatterOptionFlags = UNSIGNEDLONG;
>>>>>>> origin/fixes_2.4
const
	kCFNumberFormatterParseIntegersOnly = 1;	{ only parse integers }

function CFNumberFormatterCreateNumberFromString( allocator: CFAllocatorRef; formatter: CFNumberFormatterRef; strng: CFStringRef; rangep: CFRangePtr; options: CFOptionFlags ): CFNumberRef; external name '_CFNumberFormatterCreateNumberFromString';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

function CFNumberFormatterGetValueFromString( formatter: CFNumberFormatterRef; strng: CFStringRef; rangep: CFRangePtr; numberType: CFNumberType; valuePtr: UnivPtr ): Boolean; external name '_CFNumberFormatterGetValueFromString';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
	// Parse a string representation of a number using the current state
	// of the number formatter.  The range parameter specifies the range
	// of the string in which the parsing should occur in input, and on
	// output indicates the extent that was used; this parameter can
	// be NULL, in which case the whole string may be used.  The
	// return value indicates whether some number was computed and
	// (if valuePtr is not NULL) stored at the location specified by
	// valuePtr.  The numberType indicates the type of value pointed
	// to by valuePtr.


procedure CFNumberFormatterSetProperty( formatter: CFNumberFormatterRef; key: CFStringRef; value: CFTypeRef ); external name '_CFNumberFormatterSetProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

function CFNumberFormatterCopyProperty( formatter: CFNumberFormatterRef; key: CFStringRef ): CFTypeRef; external name '_CFNumberFormatterCopyProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
	// Set and get various properties of the number formatter, the set of
	// which may be expanded in the future.

var kCFNumberFormatterCurrencyCode: CFStringRef; external name '_kCFNumberFormatterCurrencyCode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)		// CFString
var kCFNumberFormatterDecimalSeparator: CFStringRef; external name '_kCFNumberFormatterDecimalSeparator'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)	// CFString
var kCFNumberFormatterCurrencyDecimalSeparator: CFStringRef; external name '_kCFNumberFormatterCurrencyDecimalSeparator'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *) // CFString
var kCFNumberFormatterAlwaysShowDecimalSeparator: CFStringRef; external name '_kCFNumberFormatterAlwaysShowDecimalSeparator'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *) // CFBoolean
var kCFNumberFormatterGroupingSeparator: CFStringRef; external name '_kCFNumberFormatterGroupingSeparator'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)	// CFString
var kCFNumberFormatterUseGroupingSeparator: CFStringRef; external name '_kCFNumberFormatterUseGroupingSeparator'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)	// CFBoolean
var kCFNumberFormatterPercentSymbol: CFStringRef; external name '_kCFNumberFormatterPercentSymbol'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)		// CFString
var kCFNumberFormatterZeroSymbol: CFStringRef; external name '_kCFNumberFormatterZeroSymbol'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)		// CFString
var kCFNumberFormatterNaNSymbol: CFStringRef; external name '_kCFNumberFormatterNaNSymbol'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)		// CFString
var kCFNumberFormatterInfinitySymbol: CFStringRef; external name '_kCFNumberFormatterInfinitySymbol'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)		// CFString
var kCFNumberFormatterMinusSign: CFStringRef; external name '_kCFNumberFormatterMinusSign'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)		// CFString
var kCFNumberFormatterPlusSign: CFStringRef; external name '_kCFNumberFormatterPlusSign'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)		// CFString
var kCFNumberFormatterCurrencySymbol: CFStringRef; external name '_kCFNumberFormatterCurrencySymbol'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)		// CFString
var kCFNumberFormatterExponentSymbol: CFStringRef; external name '_kCFNumberFormatterExponentSymbol'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)		// CFString
var kCFNumberFormatterMinIntegerDigits: CFStringRef; external name '_kCFNumberFormatterMinIntegerDigits'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)	// CFNumber
var kCFNumberFormatterMaxIntegerDigits: CFStringRef; external name '_kCFNumberFormatterMaxIntegerDigits'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)	// CFNumber
var kCFNumberFormatterMinFractionDigits: CFStringRef; external name '_kCFNumberFormatterMinFractionDigits'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)	// CFNumber
var kCFNumberFormatterMaxFractionDigits: CFStringRef; external name '_kCFNumberFormatterMaxFractionDigits'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)	// CFNumber
var kCFNumberFormatterGroupingSize: CFStringRef; external name '_kCFNumberFormatterGroupingSize'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)		// CFNumber
var kCFNumberFormatterSecondaryGroupingSize: CFStringRef; external name '_kCFNumberFormatterSecondaryGroupingSize'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)	// CFNumber
var kCFNumberFormatterRoundingMode: CFStringRef; external name '_kCFNumberFormatterRoundingMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)		// CFNumber
var kCFNumberFormatterRoundingIncrement: CFStringRef; external name '_kCFNumberFormatterRoundingIncrement'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)	// CFNumber
var kCFNumberFormatterFormatWidth: CFStringRef; external name '_kCFNumberFormatterFormatWidth'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)		// CFNumber
var kCFNumberFormatterPaddingPosition: CFStringRef; external name '_kCFNumberFormatterPaddingPosition'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)	// CFNumber
var kCFNumberFormatterPaddingCharacter: CFStringRef; external name '_kCFNumberFormatterPaddingCharacter'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)	// CFString
var kCFNumberFormatterDefaultFormat: CFStringRef; external name '_kCFNumberFormatterDefaultFormat'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)		// CFString
var kCFNumberFormatterMultiplier: CFStringRef; external name '_kCFNumberFormatterMultiplier'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)		// CFNumber
var kCFNumberFormatterPositivePrefix: CFStringRef; external name '_kCFNumberFormatterPositivePrefix'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)		// CFString
var kCFNumberFormatterPositiveSuffix: CFStringRef; external name '_kCFNumberFormatterPositiveSuffix'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)		// CFString
var kCFNumberFormatterNegativePrefix: CFStringRef; external name '_kCFNumberFormatterNegativePrefix'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)		// CFString
var kCFNumberFormatterNegativeSuffix: CFStringRef; external name '_kCFNumberFormatterNegativeSuffix'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)		// CFString
var kCFNumberFormatterPerMillSymbol: CFStringRef; external name '_kCFNumberFormatterPerMillSymbol'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)		// CFString
var kCFNumberFormatterInternationalCurrencySymbol: CFStringRef; external name '_kCFNumberFormatterInternationalCurrencySymbol'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *) // CFString
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
var kCFNumberFormatterCurrencyGroupingSeparator: CFStringRef; external name '_kCFNumberFormatterCurrencyGroupingSeparator'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CF_AVAILABLE_STARTING(10_5, 2_0) *) // CFString
var kCFNumberFormatterIsLenient: CFStringRef; external name '_kCFNumberFormatterIsLenient'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_5, 2_0) *)		// CFBoolean
var kCFNumberFormatterUseSignificantDigits: CFStringRef; external name '_kCFNumberFormatterUseSignificantDigits'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_5, 2_0) *)	// CFBoolean
var kCFNumberFormatterMinSignificantDigits: CFStringRef; external name '_kCFNumberFormatterMinSignificantDigits'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_5, 2_0) *)	// CFNumber
var kCFNumberFormatterMaxSignificantDigits: CFStringRef; external name '_kCFNumberFormatterMaxSignificantDigits'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_5, 2_0) *)	// CFNumber

type
	CFNumberFormatterRoundingMode = CFIndex;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
var kCFNumberFormatterCurrencyGroupingSeparator: CFStringRef; external name '_kCFNumberFormatterCurrencyGroupingSeparator'; (* attribute const *)
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *) // CFString
var kCFNumberFormatterIsLenient: CFStringRef; external name '_kCFNumberFormatterIsLenient'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)		// CFBoolean
var kCFNumberFormatterUseSignificantDigits: CFStringRef; external name '_kCFNumberFormatterUseSignificantDigits'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)	// CFBoolean
var kCFNumberFormatterMinSignificantDigits: CFStringRef; external name '_kCFNumberFormatterMinSignificantDigits'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)	// CFNumber
var kCFNumberFormatterMaxSignificantDigits: CFStringRef; external name '_kCFNumberFormatterMaxSignificantDigits'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)	// CFNumber
<<<<<<< HEAD

type
	CFNumberFormatterRoundingMode = SIGNEDLONG;
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
const
	kCFNumberFormatterRoundCeiling = 0;
	kCFNumberFormatterRoundFloor = 1;
	kCFNumberFormatterRoundDown = 2;
	kCFNumberFormatterRoundUp = 3;
	kCFNumberFormatterRoundHalfEven = 4;
	kCFNumberFormatterRoundHalfDown = 5;
	kCFNumberFormatterRoundHalfUp = 6;

type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CFNumberFormatterPadPosition = CFIndex;
=======
	CFNumberFormatterPadPosition = SIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	CFNumberFormatterPadPosition = SIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	CFNumberFormatterPadPosition = SIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	CFNumberFormatterPadPosition = SIGNEDLONG;
>>>>>>> origin/cpstrnew
const
	kCFNumberFormatterPadBeforePrefix = 0;
	kCFNumberFormatterPadAfterPrefix = 1;
	kCFNumberFormatterPadBeforeSuffix = 2;
	kCFNumberFormatterPadAfterSuffix = 3;
=======
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

type
	CFNumberFormatterRoundingMode = SIGNEDLONG;
const
	kCFNumberFormatterRoundCeiling = 0;
	kCFNumberFormatterRoundFloor = 1;
	kCFNumberFormatterRoundDown = 2;
	kCFNumberFormatterRoundUp = 3;
	kCFNumberFormatterRoundHalfEven = 4;
	kCFNumberFormatterRoundHalfDown = 5;
	kCFNumberFormatterRoundHalfUp = 6;

type
	CFNumberFormatterPadPosition = SIGNEDLONG;
const
<<<<<<< HEAD
    kCFNumberFormatterPadBeforePrefix = 0;
    kCFNumberFormatterPadAfterPrefix = 1;
    kCFNumberFormatterPadBeforeSuffix = 2;
    kCFNumberFormatterPadAfterSuffix = 3;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kCFNumberFormatterPadBeforePrefix = 0;
	kCFNumberFormatterPadAfterPrefix = 1;
	kCFNumberFormatterPadBeforeSuffix = 2;
	kCFNumberFormatterPadAfterSuffix = 3;
>>>>>>> origin/fixes_2.4


function CFNumberFormatterGetDecimalInfoForCurrencyCode( currencyCode: CFStringRef; defaultFractionDigits: SInt32Ptr; roundingIncrement: Float64Ptr ): Boolean; external name '_CFNumberFormatterGetDecimalInfoForCurrencyCode';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
	// Returns the number of fraction digits that should be displayed, and
	// the rounding increment (or 0.0 if no rounding is done by the currency)
	// for the given currency.  Returns false if the currency code is unknown
	// or the information is not available.
	// Not localized because these are properties of the currency.


{#endif}

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
