{	CFError.h
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	Copyright (c) 2006-2013, Apple Inc. All rights reserved.
}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	Copyright (c) 2006-2007, Apple Inc. All rights reserved.
}

{	 Pascal Translation:  Gale R Paeper, <gpaeper@empirenet.com>, 2008 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }

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

unit CFError;
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
uses MacTypes, CFBase, CFDictionary;
{$endc} {not MACOSALLINCLUDE}

{$ALIGN POWER}


{!
	@header CFError
        @discussion
            CFErrors are used to encompass information about errors. At minimum, errors are identified by their domain (a string) and an error code within that domain. In addition a "userInfo" dictionary supplied at creation time enables providing additional info that might be useful for the interpretation and reporting of the error. This dictionary can even contain an "underlying" error, which is wrapped as an error bubbles up through various layers. 
            
            CFErrors have the ability to provide human-readable descriptions for the errors; in fact, they are designed to provide localizable, end-user presentable errors that can appear in the UI. CFError has a number of predefined userInfo keys to enable developers to supply the info.
            
            Usage recommendation for CFErrors is to return them as by-ref parameters in functions. This enables the caller to pass NULL in when they don't actually want information about the error. The presence of an error should be reported by other means, for instance a NULL or false return value from the function call proper:
            
            CFError *error;
            if (!ReadFromFile(fd, &error)) (
                ... process error ...
                CFRelease(error);   // If an error occurs, the returned CFError must be released.
            )
            
            It is the responsibility of anyone returning CFErrors this way to:
            - Not touch the error argument if no error occurs
            - Create and assign the error for return only if the error argument is non-NULL
            
            In addition, it's recommended that CFErrors be used in error situations only (not status), and where there are multiple possible errors to distinguish between. For instance there is no plan to add CFErrors to existing APIs in CF which currently don't return errors; in many cases, there is one possible reason for failure, and a false or NULL return is enough to indicate it.

            CFError is toll-free bridged to NSError in Foundation. NSError in Foundation has some additional guidelines which makes it easy to automatically report errors to users and even try to recover from them.  See http://developer.apple.com/documentation/Cocoa/Conceptual/ErrorHandlingCocoa/ErrorHandling/chapter_1_section_1.html for more info on NSError programming guidelines.
}

{!
	@typedef CFErrorRef
	    This is the type of a reference to CFErrors.  CFErrorRef is toll-free bridged with NSError.
}
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CFErrorRef = ^__CFError; { an opaque type }
	__CFError = record end;
=======
	CFErrorRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CFErrorRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CFErrorRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CFErrorRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	CFErrorRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4
	CFErrorRefPtr = ^CFErrorRef;

{!
	@function CFErrorGetTypeID
	    Returns the type identifier of all CFError instances.
}
function CFErrorGetTypeID: CFTypeID; external name '_CFErrorGetTypeID';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CF_AVAILABLE_STARTING(10_5, 2_0) *)
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


// Predefined domains; value of "code" will correspond to preexisting values in these domains.
var kCFErrorDomainPOSIX: CFStringRef; external name '_kCFErrorDomainPOSIX'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CF_AVAILABLE_STARTING(10_5, 2_0) *)
var kCFErrorDomainOSStatus: CFStringRef; external name '_kCFErrorDomainOSStatus'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_5, 2_0) *)
var kCFErrorDomainMach: CFStringRef; external name '_kCFErrorDomainMach'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_5, 2_0) *)
var kCFErrorDomainCocoa: CFStringRef; external name '_kCFErrorDomainCocoa'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_5, 2_0) *)

// Keys in userInfo for localizable, end-user presentable error messages. At minimum provide one of first two; ideally provide all three.
var kCFErrorLocalizedDescriptionKey: CFStringRef; external name '_kCFErrorLocalizedDescriptionKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_5, 2_0) *)   // Key to identify the end user-presentable description in userInfo.
var kCFErrorLocalizedFailureReasonKey: CFStringRef; external name '_kCFErrorLocalizedFailureReasonKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_5, 2_0) *)   // Key to identify the end user-presentable failure reason in userInfo.
var kCFErrorLocalizedRecoverySuggestionKey: CFStringRef; external name '_kCFErrorLocalizedRecoverySuggestionKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_5, 2_0) *)   // Key to identify the end user-presentable recovery suggestion in userInfo.

// If you do not have localizable error strings, you can provide a value for this key instead.
var kCFErrorDescriptionKey: CFStringRef; external name '_kCFErrorDescriptionKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_5, 2_0) *)   // Key to identify the description in the userInfo dictionary. Should be a complete sentence if possible. Should not contain domain name or error code.

// Other keys in userInfo.
var kCFErrorUnderlyingErrorKey: CFStringRef; external name '_kCFErrorUnderlyingErrorKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_5, 2_0) *)   // Key to identify the underlying error in userInfo.
var kCFErrorURLKey: CFStringRef; external name '_kCFErrorURLKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)    // Key to identify associated URL in userInfo.  Typically one of this or kCFErrorFilePathKey is provided.
var kCFErrorFilePathKey: CFStringRef; external name '_kCFErrorFilePathKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)    // Key to identify associated file path in userInfo.    Typically one of this or kCFErrorURLKey is provided.
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kCFErrorDomainOSStatus: CFStringRef; external name '_kCFErrorDomainOSStatus'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kCFErrorDomainMach: CFStringRef; external name '_kCFErrorDomainMach'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
var kCFErrorDomainCocoa: CFStringRef; external name '_kCFErrorDomainCocoa'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

// Keys in userInfo for localizable, end-user presentable error messages. At minimum provide one of first two; ideally provide all three.
var kCFErrorLocalizedDescriptionKey: CFStringRef; external name '_kCFErrorLocalizedDescriptionKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)   // Key to identify the end user-presentable description in userInfo.
var kCFErrorLocalizedFailureReasonKey: CFStringRef; external name '_kCFErrorLocalizedFailureReasonKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)   // Key to identify the end user-presentable failure reason in userInfo.
var kCFErrorLocalizedRecoverySuggestionKey: CFStringRef; external name '_kCFErrorLocalizedRecoverySuggestionKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)   // Key to identify the end user-presentable recovery suggestion in userInfo.

// If you do not have localizable error strings, you can provide a value for this key instead.
var kCFErrorDescriptionKey: CFStringRef; external name '_kCFErrorDescriptionKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)   // Key to identify the description in the userInfo dictionary. Should be a complete sentence if possible. Should not contain domain name or error code.

// Other keys in userInfo.
var kCFErrorUnderlyingErrorKey: CFStringRef; external name '_kCFErrorUnderlyingErrorKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)   // Key to identify the underlying error in userInfo.
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


{!
	@function CFErrorCreate
	@abstract Creates a new CFError.
	@param allocator The CFAllocator which should be used to allocate memory for the error. This parameter may be NULL in which case the 
	    current default CFAllocator is used. If this reference is not a valid CFAllocator, the behavior is undefined.
	@param domain A CFString identifying the error domain. If this reference is NULL or is otherwise not a valid CFString, the behavior is undefined.
	@param code A CFIndex identifying the error code. The code is interpreted within the context of the error domain.
	@param userInfo A CFDictionary created with kCFCopyStringDictionaryKeyCallBacks and kCFTypeDictionaryValueCallBacks. It will be copied with CFDictionaryCreateCopy(). 
	    If no userInfo dictionary is desired, NULL may be passed in as a convenience, in which case an empty userInfo dictionary will be assigned.
	@result A reference to the new CFError.
}
function CFErrorCreate( allocator: CFAllocatorRef; domain: CFStringRef; code: CFIndex; userInfo: CFDictionaryRef ): CFErrorRef; external name '_CFErrorCreate';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CF_AVAILABLE_STARTING(10_5, 2_0) *)
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
	@function CFErrorCreateWithUserInfoKeysAndValues
	@abstract Creates a new CFError without having to create an intermediate userInfo dictionary.
	@param allocator The CFAllocator which should be used to allocate memory for the error. This parameter may be NULL in which case the 
	    current default CFAllocator is used. If this reference is not a valid CFAllocator, the behavior is undefined.
	@param domain A CFString identifying the error domain. If this reference is NULL or is otherwise not a valid CFString, the behavior is undefined.
	@param code A CFIndex identifying the error code. The code is interpreted within the context of the error domain.
	@param userInfoKeys An array of numUserInfoValues CFStrings used as keys in creating the userInfo dictionary. NULL is valid only if numUserInfoValues is 0.
	@param userInfoValues An array of numUserInfoValues CF types used as values in creating the userInfo dictionary.  NULL is valid only if numUserInfoValues is 0.
	@param numUserInfoValues CFIndex representing the number of keys and values in the userInfoKeys and userInfoValues arrays.
	@result A reference to the new CFError. numUserInfoValues CF types are gathered from each of userInfoKeys and userInfoValues to create the userInfo dictionary.
}
function CFErrorCreateWithUserInfoKeysAndValues( allocator: CFAllocatorRef; domain: CFStringRef; code: CFIndex; {const} userInfoKeys: {variable-size-array} UnivPtrPtr; {const} userInfoValues: {variable-size-array} UnivPtrPtr; numUserInfoValues: CFIndex ): CFErrorRef; external name '_CFErrorCreateWithUserInfoKeysAndValues';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CF_AVAILABLE_STARTING(10_5, 2_0) *)
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
	@function CFErrorGetDomain
	@abstract Returns the error domain the CFError was created with.
	@param err The CFError whose error domain is to be returned. If this reference is not a valid CFError, the behavior is undefined.
	@result The error domain of the CFError. Since this is a "Get" function, the caller shouldn't CFRelease the return value.
}
function CFErrorGetDomain( err: CFErrorRef ): CFStringRef; external name '_CFErrorGetDomain';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CF_AVAILABLE_STARTING(10_5, 2_0) *)
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
	@function CFErrorGetCode
	@abstract Returns the error code the CFError was created with.
	@param err The CFError whose error code is to be returned. If this reference is not a valid CFError, the behavior is undefined.
	@result The error code of the CFError (not an error return for the current call).
}
function CFErrorGetCode( err: CFErrorRef ): CFIndex; external name '_CFErrorGetCode';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CF_AVAILABLE_STARTING(10_5, 2_0) *)
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
	@function CFErrorCopyUserInfo
        @abstract Returns CFError userInfo dictionary.
	@discussion Returns a dictionary containing the same keys and values as in the userInfo dictionary the CFError was created with. Returns an empty dictionary if NULL was supplied to CFErrorCreate().
	@param err The CFError whose error user info is to be returned. If this reference is not a valid CFError, the behavior is undefined.
	@result The user info of the CFError.
}
function CFErrorCopyUserInfo( err: CFErrorRef ): CFDictionaryRef; external name '_CFErrorCopyUserInfo';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CF_AVAILABLE_STARTING(10_5, 2_0) *)
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
	@function CFErrorCopyDescription
	@abstract Returns a human-presentable description for the error. CFError creators should strive to make sure the return value is human-presentable and localized by providing a value for kCFErrorLocalizedDescriptionKey at the time of CFError creation.
        @discussion This is a complete sentence or two which says what failed and why it failed. Rules for computing the return value:
            - Look for kCFErrorLocalizedDescriptionKey in the user info and if not NULL, returns that as-is.  
            - Otherwise, if there is a kCFErrorLocalizedFailureReasonKey in the user info, generate an error from that. Something like: "Operation code not be completed. " + kCFErrorLocalizedFailureReasonKey
            - Otherwise, generate a semi-user presentable string from kCFErrorDescriptionKey, the domain, and code. Something like: "Operation could not be completed. Error domain/code occurred. " or "Operation could not be completed. " + kCFErrorDescriptionKey + " (Error domain/code)"
            Toll-free bridged NSError instances might provide additional behaviors for manufacturing a description string.  Do not count on the exact contents or format of the returned string, it might change.
	@param err The CFError whose description is to be returned. If this reference is not a valid CFError, the behavior is undefined.
	@result A CFString with human-presentable description of the CFError. Never NULL.
}
function CFErrorCopyDescription( err: CFErrorRef ): CFStringRef; external name '_CFErrorCopyDescription';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CF_AVAILABLE_STARTING(10_5, 2_0) *)
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
	@function CFErrorCopyFailureReason
        @abstract Returns a human-presentable failure reason for the error.  May return NULL.  CFError creators should strive to make sure the return value is human-presentable and localized by providing a value for kCFErrorLocalizedFailureReasonKey at the time of CFError creation.
        @discussion This is a complete sentence which describes why the operation failed. In many cases this will be just the "because" part of the description (but as a complete sentence, which makes localization easier). By default this looks for kCFErrorLocalizedFailureReasonKey in the user info. Toll-free bridged NSError instances might provide additional behaviors for manufacturing this value. If no user-presentable string is available, returns NULL.
            Example Description: "Could not save file 'Letter' in folder 'Documents' because the volume 'MyDisk' doesn't have enough space."
            Corresponding FailureReason: "The volume 'MyDisk' doesn't have enough space."
	@param err The CFError whose failure reason is to be returned. If this reference is not a valid CFError, the behavior is undefined.
	@result A CFString with the localized, end-user presentable failure reason of the CFError, or NULL. 
}
function CFErrorCopyFailureReason( err: CFErrorRef ): CFStringRef; external name '_CFErrorCopyFailureReason';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CF_AVAILABLE_STARTING(10_5, 2_0) *)
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
	@function CFErrorCopyRecoverySuggestion
        @abstract Returns a human presentable recovery suggestion for the error.  May return NULL.  CFError creators should strive to make sure the return value is human-presentable and localized by providing a value for kCFErrorLocalizedRecoverySuggestionKey at the time of CFError creation.
        @discussion This is the string that can be displayed as the "informative" (aka "secondary") message on an alert panel. By default this looks for kCFErrorLocalizedRecoverySuggestionKey in the user info. Toll-free bridged NSError instances might provide additional behaviors for manufacturing this value. If no user-presentable string is available, returns NULL.
            Example Description: "Could not save file 'Letter' in folder 'Documents' because the volume 'MyDisk' doesn't have enough space."
            Corresponding RecoverySuggestion: "Remove some files from the volume and try again."
	@param err The CFError whose recovery suggestion is to be returned. If this reference is not a valid CFError, the behavior is undefined.
	@result A CFString with the localized, end-user presentable recovery suggestion of the CFError, or NULL. 
}
function CFErrorCopyRecoverySuggestion( err: CFErrorRef ): CFStringRef; external name '_CFErrorCopyRecoverySuggestion';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CF_AVAILABLE_STARTING(10_5, 2_0) *)
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
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
