{
 *  AXTextAttributedString.h
 *
 *  Copyright (c) 2002 Apple Computer, Inc. All rights reserved.
 *
 }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{  Pascal Translation:  Gale R Paeper, <gpaeper@empirenet.com>, 2006 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
=======
{	 Pascal Translation:  Gale R Paeper, <gpaeper@empirenet.com>, 2006 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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
{	 Pascal Translation:  Gale R Paeper, <gpaeper@empirenet.com>, 2006 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }

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

unit AXTextAttributedString;
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
>>>>>>> graemeg/cpstrnew
=======
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
>>>>>>> origin/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
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
=======
>>>>>>> origin/cpstrnew
{$ifc defined(iphonesim)}
 	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := TRUE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
	{$setc TARGET_OS_EMBEDDED := FALSE}
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
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
=======
{$elsec}
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
>>>>>>> origin/cpstrnew
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
uses MacTypes, CFBase;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not MACOSALLINCLUDE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}

var kAXFontTextAttribute: CFStringRef; external name '_kAXFontTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CFDictionaryRef - see kAXFontTextAttribute keys below
var kAXForegroundColorTextAttribute: CFStringRef; external name '_kAXForegroundColorTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CGColorRef
var kAXBackgroundColorTextAttribute: CFStringRef; external name '_kAXBackgroundColorTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CGColorRef
var kAXUnderlineColorTextAttribute: CFStringRef; external name '_kAXUnderlineColorTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CGColorRef
var kAXStrikethroughColorTextAttribute: CFStringRef; external name '_kAXStrikethroughColorTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CGColorRef
var kAXUnderlineTextAttribute: CFStringRef; external name '_kAXUnderlineTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CFNumberRef - AXUnderlineStyle
var kAXSuperscriptTextAttribute: CFStringRef; external name '_kAXSuperscriptTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CFNumberRef = + number for superscript - for subscript
var kAXStrikethroughTextAttribute: CFStringRef; external name '_kAXStrikethroughTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CFBooleanRef
var kAXShadowTextAttribute: CFStringRef; external name '_kAXShadowTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CFBooleanRef

var kAXAttachmentTextAttribute: CFStringRef; external name '_kAXAttachmentTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// AXUIElementRef
var kAXLinkTextAttribute: CFStringRef; external name '_kAXLinkTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// AXUIElementRef

var kAXNaturalLanguageTextAttribute: CFStringRef; external name '_kAXNaturalLanguageTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CFStringRef - the spoken language of the text
var kAXReplacementStringTextAttribute: CFStringRef; external name '_kAXReplacementStringTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CFStringRef

var kAXMisspelledTextAttribute: CFStringRef; external name '_kAXMisspelledTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// AXUIElementRef

var kAXAutocorrectedTextAttribute: CFStringRef; external name '_kAXAutocorrectedTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_7_AND_LATER *)	// CFBooleanRef
=======


{$ifc TARGET_OS_MAC}

=======


{$ifc TARGET_OS_MAC}

>>>>>>> origin/cpstrnew
{$ALIGN POWER}

var kAXFontTextAttribute: CFStringRef; external name '_kAXFontTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CFDictionaryRef - see kAXFontTextAttribute keys below
var kAXForegroundColorTextAttribute: CFStringRef; external name '_kAXForegroundColorTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CGColorRef
var kAXBackgroundColorTextAttribute: CFStringRef; external name '_kAXBackgroundColorTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CGColorRef
var kAXUnderlineColorTextAttribute: CFStringRef; external name '_kAXUnderlineColorTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CGColorRef
var kAXStrikethroughColorTextAttribute: CFStringRef; external name '_kAXStrikethroughColorTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CGColorRef
var kAXUnderlineTextAttribute: CFStringRef; external name '_kAXUnderlineTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CFNumberRef - AXUnderlineStyle
var kAXSuperscriptTextAttribute: CFStringRef; external name '_kAXSuperscriptTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CFNumberRef = + number for superscript - for subscript
var kAXStrikethroughTextAttribute: CFStringRef; external name '_kAXStrikethroughTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CFBooleanRef
var kAXShadowTextAttribute: CFStringRef; external name '_kAXShadowTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CFBooleanRef

var kAXAttachmentTextAttribute: CFStringRef; external name '_kAXAttachmentTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// AXUIElementRef
var kAXLinkTextAttribute: CFStringRef; external name '_kAXLinkTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// AXUIElementRef

var kAXNaturalLanguageTextAttribute: CFStringRef; external name '_kAXNaturalLanguageTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CFStringRef - the spoken language of the text
var kAXReplacementStringTextAttribute: CFStringRef; external name '_kAXReplacementStringTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CFStringRef

var kAXMisspelledTextAttribute: CFStringRef; external name '_kAXMisspelledTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// AXUIElementRef
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}

=======


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}

>>>>>>> graemeg/cpstrnew
var kAXFontTextAttribute: CFStringRef; external name '_kAXFontTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CFDictionaryRef - see kAXFontTextAttribute keys below
var kAXForegroundColorTextAttribute: CFStringRef; external name '_kAXForegroundColorTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CGColorRef
var kAXBackgroundColorTextAttribute: CFStringRef; external name '_kAXBackgroundColorTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CGColorRef
var kAXUnderlineColorTextAttribute: CFStringRef; external name '_kAXUnderlineColorTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CGColorRef
var kAXStrikethroughColorTextAttribute: CFStringRef; external name '_kAXStrikethroughColorTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CGColorRef
var kAXUnderlineTextAttribute: CFStringRef; external name '_kAXUnderlineTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CFNumberRef - AXUnderlineStyle
var kAXSuperscriptTextAttribute: CFStringRef; external name '_kAXSuperscriptTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CFNumberRef = + number for superscript - for subscript
var kAXStrikethroughTextAttribute: CFStringRef; external name '_kAXStrikethroughTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CFBooleanRef
var kAXShadowTextAttribute: CFStringRef; external name '_kAXShadowTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CFBooleanRef

var kAXAttachmentTextAttribute: CFStringRef; external name '_kAXAttachmentTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// AXUIElementRef
var kAXLinkTextAttribute: CFStringRef; external name '_kAXLinkTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// AXUIElementRef

var kAXNaturalLanguageTextAttribute: CFStringRef; external name '_kAXNaturalLanguageTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CFStringRef - the spoken language of the text
var kAXReplacementStringTextAttribute: CFStringRef; external name '_kAXReplacementStringTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CFStringRef

var kAXMisspelledTextAttribute: CFStringRef; external name '_kAXMisspelledTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// AXUIElementRef
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

// kAXFontTextAttribute keys
var kAXFontNameKey: CFStringRef; external name '_kAXFontNameKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CFStringRef - required
var kAXFontFamilyKey: CFStringRef; external name '_kAXFontFamilyKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CFStringRef - not required
var kAXVisibleNameKey: CFStringRef; external name '_kAXVisibleNameKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CFStringRef - not required
var kAXFontSizeKey: CFStringRef; external name '_kAXFontSizeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CFNumberRef - required
=======
=======
>>>>>>> origin/fixes_2_2
{$ALIGN POWER}
=======
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}

var kAXFontTextAttribute: CFStringRef; external name '_kAXFontTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CFDictionaryRef - see kAXFontTextAttribute keys below
var kAXForegroundColorTextAttribute: CFStringRef; external name '_kAXForegroundColorTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CGColorRef
var kAXBackgroundColorTextAttribute: CFStringRef; external name '_kAXBackgroundColorTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CGColorRef
var kAXUnderlineColorTextAttribute: CFStringRef; external name '_kAXUnderlineColorTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CGColorRef
var kAXStrikethroughColorTextAttribute: CFStringRef; external name '_kAXStrikethroughColorTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CGColorRef
var kAXUnderlineTextAttribute: CFStringRef; external name '_kAXUnderlineTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CFNumberRef - AXUnderlineStyle
var kAXSuperscriptTextAttribute: CFStringRef; external name '_kAXSuperscriptTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CFNumberRef = + number for superscript - for subscript
var kAXStrikethroughTextAttribute: CFStringRef; external name '_kAXStrikethroughTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CFBooleanRef
var kAXShadowTextAttribute: CFStringRef; external name '_kAXShadowTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CFBooleanRef

var kAXAttachmentTextAttribute: CFStringRef; external name '_kAXAttachmentTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// AXUIElementRef
var kAXLinkTextAttribute: CFStringRef; external name '_kAXLinkTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// AXUIElementRef

var kAXNaturalLanguageTextAttribute: CFStringRef; external name '_kAXNaturalLanguageTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CFStringRef - the spoken language of the text
var kAXReplacementStringTextAttribute: CFStringRef; external name '_kAXReplacementStringTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CFStringRef

var kAXMisspelledTextAttribute: CFStringRef; external name '_kAXMisspelledTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// AXUIElementRef

// kAXFontTextAttribute keys
<<<<<<< HEAD
var kAXFontNameKey: CFStringRef; external name '_kAXFontNameKey'; (* attribute const *)		// CFStringRef - required
var kAXFontFamilyKey: CFStringRef; external name '_kAXFontFamilyKey'; (* attribute const *)		// CFStringRef - not required
var kAXVisibleNameKey: CFStringRef; external name '_kAXVisibleNameKey'; (* attribute const *)		// CFStringRef - not required
var kAXFontSizeKey: CFStringRef; external name '_kAXFontSizeKey'; (* attribute const *)		// CFNumberRef - required
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
var kAXFontNameKey: CFStringRef; external name '_kAXFontNameKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CFStringRef - required
var kAXFontFamilyKey: CFStringRef; external name '_kAXFontFamilyKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CFStringRef - not required
var kAXVisibleNameKey: CFStringRef; external name '_kAXVisibleNameKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CFStringRef - not required
var kAXFontSizeKey: CFStringRef; external name '_kAXFontSizeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CFNumberRef - required
>>>>>>> origin/fixes_2.4

const
	kAXUnderlineStyleNone = $0;
	kAXUnderlineStyleSingle = $1;
	kAXUnderlineStyleThick = $2;
	kAXUnderlineStyleDouble = $9;
type
	AXUnderlineStyle = UInt32;


// DO NOT USE. This is an old, misspelled version of one of the above constants.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
var kAXForegoundColorTextAttribute: CFStringRef; external name '_kAXForegoundColorTextAttribute'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)	// CGColorRef

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
<<<<<<< HEAD

end.
{$endc} {not MACOSALLINCLUDE}
=======
var kAXForegoundColorTextAttribute: CFStringRef; external name '_kAXForegoundColorTextAttribute'; (* attribute const *)	// CGColorRef

end.
>>>>>>> graemeg/fixes_2_2
=======
var kAXForegoundColorTextAttribute: CFStringRef; external name '_kAXForegoundColorTextAttribute'; (* attribute const *)	// CGColorRef

end.
>>>>>>> origin/fixes_2_2
=======

end.
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4
