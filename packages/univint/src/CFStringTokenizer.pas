{	CFStringTokenizer.h
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	Copyright (c) 2006-2013, Apple Inc. All rights reserved.
}
=======
	Copyright (c) 2006-2009, Apple Inc. All rights reserved.
}
{	  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, September 2010 }
>>>>>>> graemeg/cpstrnew
=======
	Copyright (c) 2006-2009, Apple Inc. All rights reserved.
}
{	  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, September 2010 }
>>>>>>> graemeg/cpstrnew
=======
	Copyright (c) 2006-2009, Apple Inc. All rights reserved.
}
{	  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, September 2010 }
>>>>>>> graemeg/cpstrnew
=======
	Copyright (c) 2006-2009, Apple Inc. All rights reserved.
}
{	  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, September 2010 }
>>>>>>> origin/cpstrnew
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

unit CFStringTokenizer;
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
>>>>>>> origin/cpstrnew
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
uses MacTypes,CFBase,CFLocale,CFArray,CFString;
{$endc} {not MACOSALLINCLUDE}

{$ALIGN POWER}


{!
    @header CFStringTokenizer
    @abstract A CFStringTokenizer object provides the means by which you can
		tokenize a string. To find a token that includes the character
		specified by character index and set it as the current token, you call
		CFStringTokenizerGoToTokenAtIndex. To advance to the next token and set
		it as the current token, you call CFStringTokenizerAdvanceToNextToken.
		To get the range of current token, you call
		CFStringTokenizerGetCurrentTokenRange. You can use
		CFStringTokenizerCopyCurrentTokenAttribute to get the attribute of
		current token. If current token is a compound, you can call
		CFStringTokenizerGetCurrentSubTokens to retrieve the subtokens or derived
		subtokens contained in the compound token.
		To guess the language of a string, you call 
		CFStringTokenizerCopyBestStringLanguage.
}


{
===================================================================
	Language Identifier
===================================================================
}

{!
	@function CFStringTokenizerCopyBestStringLanguage
	@abstract Guesses the language of a string and returns the BCP 47 string of the
		language.
	@param string The string whose language is to be guessed.
	@param range The range of characters in string whose language to be
		guessed. The specified range must not exceed the bounds of the string.
		If the range is empty (length 0), the first few hundred characters in
		the string are used.
	@result A language represented in BCP 47 string, or NULL if the
		language of the string cannot be guessed. 
	@discussion The result is not guaranteed to be accurate. Typically 200-400
		characters are required to reliably guess the language of a string.
}
function CFStringTokenizerCopyBestStringLanguage( strng: CFStringRef; range: CFRange ): CFStringRef; external name '_CFStringTokenizerCopyBestStringLanguage';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CF_AVAILABLE_STARTING(10_5, 3_0) *)
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

{
===================================================================
	Tokenization
===================================================================
}

type
	CFStringTokenizerRef = ^__CFStringTokenizer; { an opaque type }
	__CFStringTokenizer = record end;

{!
	Tokenization options
}
const
{!
	Tokenization Unit
	Use one of tokenization unit options with CFStringTokenizerCreate to
	specify how the string should be tokenized. 
	}
    { kCFStringTokenizerUnitWord is not locale sensitive. It doesn't return
    space between words as a token. }
	kCFStringTokenizerUnitWord = 0;
	kCFStringTokenizerUnitSentence = 1;
	kCFStringTokenizerUnitParagraph = 2;
	kCFStringTokenizerUnitLineBreak = 3;
    { kCFStringTokenizerUnitWordBoundary can be used in double click detection
    and whole word search. It is locale sensitive. If the locale parameter of
    CFStringTokenizerCreate is NULL, default locale is used.
    kCFStringTokenizerUnitWordBoundary returns space between words as a token. }
	kCFStringTokenizerUnitWordBoundary = 4;

	{!
    Attribute Specifier
    Use attribute specifier to tell tokenizer to prepare the specified attribute
	when it tokenizes the given string. The attribute value can be retrieved by
	calling CFStringTokenizerCopyCurrentTokenAttribute with one of the attribute
	option. 
	}	
    { Latin Transcription. Used with kCFStringTokenizerUnitWord or
        kCFStringTokenizerUnitWordBoundary }
	kCFStringTokenizerAttributeLatinTranscription = 1 shl 16;
    { Language in BCP 47 string. Used with kCFStringTokenizerUnitSentence
	   or kCFStringTokenizerUnitParagraph. }
	kCFStringTokenizerAttributeLanguage = 1 shl 17;

{!
	Token type
	CFStringTokenizerGoToTokenAtIndex / CFStringTokenizerAdvanceToNextToken returns
	the type of current token.
}
const
{ Have no token. }
	kCFStringTokenizerTokenNone = 0;
    
	{ Normal token }
	kCFStringTokenizerTokenNormal = 1 shl 0;
    
	{!
    Compound token which may contain subtokens but with no derived subtokens.
    Its subtokens can be obtained by calling CFStringTokenizerGetCurrentSubTokens.
    }
	kCFStringTokenizerTokenHasSubTokensMask = 1 shl 1;
    
	{!
    Compound token which may contain derived subtokens. 
    Its subtokens and derived subtokens can be obtained by calling
    CFStringTokenizerGetCurrentSubTokens.
    }
	kCFStringTokenizerTokenHasDerivedSubTokensMask = 1 shl 2;
	kCFStringTokenizerTokenHasHasNumbersMask = 1 shl 3;
	kCFStringTokenizerTokenHasNonLettersMask = 1 shl 4;
	kCFStringTokenizerTokenIsCJWordMask = 1 shl 5;
type
	CFStringTokenizerTokenType = CFOptionFlags;

{!
	@function CFStringTokenizerGetTypeID
	@abstract Get the type identifier.
	@result the type identifier of all CFStringTokenizer instances.
}
function CFStringTokenizerGetTypeID: CFTypeID; external name '_CFStringTokenizerGetTypeID';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CF_AVAILABLE_STARTING(10_5, 3_0) *)
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
																				
{!
	@function CFStringTokenizerCreate
	@abstract Creates a tokenizer instance.
	@param alloc The CFAllocator which should be used to allocate memory for the
		tokenizer and its storage for values. This parameter may be NULL in which 
		case the current default CFAllocator is used. 	
	@param string The string to tokenize.
	@param range The range of characters within the string to be tokenized. The
		specified range must not exceed the length of the string.
	@param options Use one of the Tokenization Unit options to specify how the 
		string should be tokenized. Optionally specify one or more attribute
		specifiers to tell the tokenizer to prepare specified attributes when it
		tokenizes the string.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	@param locale The locale to specify language or region specific behavior. Pass
               NULL if you want tokenizer to identify the locale automatically.
	@result A reference to the new CFStringTokenizer.
}
function CFStringTokenizerCreate( alloc: CFAllocatorRef; strng: CFStringRef; range: CFRange; options: CFOptionFlags; locale: CFLocaleRef ): CFStringTokenizerRef; external name '_CFStringTokenizerCreate';
(* CF_AVAILABLE_STARTING(10_5, 3_0) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	@param locale The locale to specify language or region specific behavior.
		If the locale is NULL and the unit is not kCFStringTokenizerUnitWord, 
        default locale will be used.
        The locale is ignored if the unit is kCFStringTokenizerUnitWord.
	@result A reference to the new CFStringTokenizer.
}
function CFStringTokenizerCreate( alloc: CFAllocatorRef; strng: CFStringRef; range: CFRange; options: CFOptionFlags; locale: CFLocaleRef ): CFStringTokenizerRef; external name '_CFStringTokenizerCreate';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
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

{!
	@function CFStringTokenizerSetString
	@abstract Set the string to tokenize.
	@param tokenizer The reference to CFStringTokenizer returned by
		CFStringTokenizerCreate.
	@param string The string to tokenize.
	@param range The range of characters within the string to be tokenized. The
		specified range must not exceed the length of the string.
}
procedure CFStringTokenizerSetString( tokenizer: CFStringTokenizerRef; strng: CFStringRef; range: CFRange ); external name '_CFStringTokenizerSetString';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CF_AVAILABLE_STARTING(10_5, 3_0) *)
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
																		
{!
	@function CFStringTokenizerGoToTokenAtIndex
	@abstract Random access to a token. Find a token that includes the character specified
		by character index, and set it as the current token.
	@param tokenizer The reference to CFStringTokenizer returned by
		CFStringTokenizerCreate.
	@param index The index of the Unicode character in the CFString.
	@result Type of the token if succeeded in finding a token and setting it as
		current token. kCFStringTokenizerTokenNone if failed in finding a token.
	@discussion The range and attribute of the token can be obtained by calling
		CFStringTokenizerGetCurrentTokenRange and CFStringTokenizerCopyCurrentTokenAttribute.
		If the token is a compound (with type kCFStringTokenizerTokenHasSubTokensMask or
		kCFStringTokenizerTokenHasDerivedSubTokensMask), its subtokens and
		(or) derived subtokens can be obtained by calling CFStringTokenizerGetCurrentSubTokens. 
}
function CFStringTokenizerGoToTokenAtIndex( tokenizer: CFStringTokenizerRef; index: CFIndex ): CFStringTokenizerTokenType; external name '_CFStringTokenizerGoToTokenAtIndex';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CF_AVAILABLE_STARTING(10_5, 3_0) *)
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

{!
	@function CFStringTokenizerAdvanceToNextToken
	@abstract Token enumerator.
	@param tokenizer The reference to CFStringTokenizer returned by
		CFStringTokenizerCreate.
	@result Type of the token if succeeded in finding a token and setting it as
		current token. kCFStringTokenizerTokenNone if failed in finding a token.
	@discussion If there is no preceding call to CFStringTokenizerGoToTokenAtIndex 
		or CFStringTokenizerAdvanceToNextToken, it finds the first token in the range
		specified to CFStringTokenizerCreate. If there is a current token after successful
		call to CFStringTokenizerGoToTokenAtIndex or CFStringTokenizerAdvanceToNextToken,
		it proceeds to the next token. If succeeded in finding a token, set it as current 
		token and return its token type. Otherwise invalidate current token and return
		kCFStringTokenizerTokenNone.
		The range and attribute of the token can be obtained by calling
		CFStringTokenizerGetCurrentTokenRange and 
        CFStringTokenizerCopyCurrentTokenAttribute. If the token is a compound
		(with type kCFStringTokenizerTokenHasSubTokensMask or
		kCFStringTokenizerTokenHasDerivedSubTokensMask), its subtokens and
		(or) derived subtokens can be obtained by calling CFStringTokenizerGetCurrentSubTokens. 
}
function CFStringTokenizerAdvanceToNextToken( tokenizer: CFStringTokenizerRef ): CFStringTokenizerTokenType; external name '_CFStringTokenizerAdvanceToNextToken';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CF_AVAILABLE_STARTING(10_5, 3_0) *)
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

{!
	@function CFStringTokenizerGetCurrentTokenRange
	@abstract Returns the range of current token.
	@param tokenizer The reference to CFStringTokenizer returned by
		CFStringTokenizerCreate.
	@result Range of current token, or (kCFNotFound,0) if there is no current token.
}
function CFStringTokenizerGetCurrentTokenRange( tokenizer: CFStringTokenizerRef ): CFRange; external name '_CFStringTokenizerGetCurrentTokenRange';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CF_AVAILABLE_STARTING(10_5, 3_0) *)
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
																				
{!
	@function CFStringTokenizerCopyCurrentTokenAttribute
	@abstract Copies the specified attribute of current token.
	@param tokenizer The reference to CFStringTokenizer returned by
		CFStringTokenizerCreate.
	@param attribute Specify a token attribute you want to obtain. The value is
		one of kCFStringTokenizerAttributeLatinTranscription or
		kCFStringTokenizerAttributeLanguage.
	@result Token attribute, or NULL if current token does not have the specified
		attribute or if there is no current token.
}
function CFStringTokenizerCopyCurrentTokenAttribute( tokenizer: CFStringTokenizerRef; attribute: CFOptionFlags ): CFTypeRef; external name '_CFStringTokenizerCopyCurrentTokenAttribute';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CF_AVAILABLE_STARTING(10_5, 3_0) *)
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

{!
	@function CFStringTokenizerGetCurrentSubTokens
	@abstract Retrieves the subtokens or derived subtokens contained in the compound token.
	@param tokenizer The reference to CFStringTokenizer returned by CFStringTokenizerCreate.
	@param ranges An array of CFRange to fill in with the ranges of subtokens. The filled in 
		ranges are relative to the string specified to CFStringTokenizerCreate. This parameter
		can be NULL.
	@param maxRangeLength The maximum number of ranges to return.
	@param derivedSubTokens An array of CFMutableArray to which the derived subtokens are to
		be added. This parameter can be NULL.
	@result number of subtokens.
	@discussion If token type is kCFStringTokenizerTokenNone, the ranges array and 
		derivedSubTokens array are untouched and the return value is 0.
        If token type is kCFStringTokenizerTokenNormal, the ranges array has one item
        filled in with the entire range of the token (if maxRangeLength >= 1) and a string
		taken from the entire token range is added to the derivedSubTokens array and the 
		return value is 1.
		If token type is kCFStringTokenizerTokenHasSubTokensMask or
        kCFStringTokenizerTokenHasDerivedSubTokensMask, the ranges array is filled
        in with as many items as there are subtokens (up to a limit of maxRangeLength).
		The derivedSubTokens array will have sub tokens added even when the sub token is a 
		substring of the token. If token type is kCFStringTokenizerTokenHasSubTokensMask,
		the ordinary non-derived subtokens are added to the derivedSubTokens array. 
}
function CFStringTokenizerGetCurrentSubTokens( tokenizer: CFStringTokenizerRef; var ranges: CFRange; maxRangeLength: CFIndex; derivedSubTokens: CFMutableArrayRef ): CFIndex; external name '_CFStringTokenizerGetCurrentSubTokens';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CF_AVAILABLE_STARTING(10_5, 3_0) *)

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
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
