{	CFDictionary.h
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	Copyright (c) 1998-2013, Apple Inc. All rights reserved.
}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	Copyright (c) 1998-2009, Apple, Inc. All rights reserved.
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
	Copyright (c) 1998-2005, Apple, Inc. All rights reserved.
=======
	Copyright (c) 1998-2009, Apple, Inc. All rights reserved.
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

unit CFDictionary;
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
=======
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
>>>>>>> origin/cpstrnew
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
{$ifc defined(iphonesim)}
 	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := TRUE}
<<<<<<< HEAD
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
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
=======
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
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
uses MacTypes,CFBase;
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
	@header CFDictionary
	CFDictionary implements a container which pairs pointer-sized keys
	with pointer-sized values. Values are accessed via arbitrary
	user-defined keys. A CFDictionary differs from a CFArray in that
	the key used to access a particular value in the dictionary remains
	the same as values are added to or removed from the dictionary,
	unless a value associated with its particular key is replaced or
	removed. In a CFArray, the key (or index) used to retrieve a
	particular value can change over time as values are added to or
	deleted from the array. Also unlike an array, there is no ordering
	among values in a dictionary. To enable later retrieval of a value,
	the key of the key-value pair should be constant (or treated as
	constant); if the key changes after being used to put a value in
	the dictionary, the value may not be retrievable. The keys of a
	dictionary form a set; that is, no two keys which are equal to
	one another are present in the dictionary at any time.

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
#if MAC_OS_X_VERSION_MIN_REQUIRED <= MAC_OS_X_VERSION_10_4
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
#if MAC_OS_X_VERSION_MIN_REQUIRED <= MAC_OS_X_VERSION_10_4
>>>>>>> origin/fixes_2.4
	Dictionaries come in two flavors, immutable, which cannot have
	values added to them or removed from them after the dictionary is
	created, and mutable, to which you can add values or from which
	remove values. Mutable dictionaries have two subflavors,
	fixed-capacity, for which there is a maximum number set at creation
	time of values which can be put into the dictionary, and variable
	capacity, which can have an unlimited number of values (or rather,
	limited only by constraints external to CFDictionary, like the
	amount of available memory). Fixed-capacity dictionaries can be
	somewhat higher performing, if you can put a definate upper limit
	on the number of values that might be put into the dictionary.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
#else
	Dictionaries come in two flavors, immutable, which cannot have
	values added to them or removed from them after the dictionary is
	created, and mutable, to which you can add values or from which
	remove values. Mutable dictionaries can have an unlimited number
	of values (or rather, limited only by constraints external to
	CFDictionary, like the amount of available memory).
#endif
<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

	As with all CoreFoundation collection types, dictionaries maintain
	hard references on the values you put in them, but the retaining and
	releasing functions are user-defined callbacks that can actually do
	whatever the user wants (for example, nothing).

	Although a particular implementation of CFDictionary may not use
	hashing and a hash table for storage of the values, the keys have
	a hash-code generating function defined for them, and a function
	to test for equality of two keys. These two functions together
	must maintain the invariant that if equal(X, Y), then hash(X) ==
	hash(Y). Note that the converse will not generally be true (but
	the contrapositive, if hash(X) != hash(Y), then !equal(X, Y),
	will be as required by Boolean logic). If the hash() and equal()
	key callbacks are NULL, the key is used as a pointer-sized integer,
	and pointer equality is used. Care should be taken to provide a
	hash() callback which will compute sufficiently dispersed hash
	codes for the key set for best performance.

	Computational Complexity
	The access time for a value in the dictionary is guaranteed to be at
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	worst O(N) for any implementation, current and future, but will
	often be O(1) (constant time). Insertion or deletion operations
	will typically be constant time as well, but are O(N*N) in the
=======
	worst O(lg N) for any implementation, current and future, but will
	often be O(1) (constant time). Insertion or deletion operations
	will typically be constant time as well, but are O(N*lg N) in the
>>>>>>> graemeg/fixes_2_2
=======
	worst O(lg N) for any implementation, current and future, but will
	often be O(1) (constant time). Insertion or deletion operations
	will typically be constant time as well, but are O(N*lg N) in the
>>>>>>> origin/fixes_2_2
=======
	worst O(N) for any implementation, current and future, but will
	often be O(1) (constant time). Insertion or deletion operations
	will typically be constant time as well, but are O(N*N) in the
>>>>>>> origin/fixes_2.4
	worst case in some implementations. Access of values through a key
	is faster than accessing values directly (if there are any such
	operations). Dictionaries will tend to use significantly more memory
	than a array with the same number of values.
}


{!
	@typedef CFDictionaryKeyCallBacks
	Structure containing the callbacks for keys of a CFDictionary.
	@field version The version number of the structure type being passed
		in as a parameter to the CFDictionary creation functions.
		This structure is version 0.
	@field retain The callback used to add a retain for the dictionary
		on keys as they are used to put values into the dictionary.
		This callback returns the value to use as the key in the
		dictionary, which is usually the value parameter passed to
		this callback, but may be a different value if a different
		value should be used as the key. The dictionary's allocator
		is passed as the first argument.
	@field release The callback used to remove a retain previously added
		for the dictionary from keys as their values are removed from
		the dictionary. The dictionary's allocator is passed as the
		first argument.
	@field copyDescription The callback used to create a descriptive
		string representation of each key in the dictionary. This
		is used by the CFCopyDescription() function.
	@field equal The callback used to compare keys in the dictionary for
		equality.
	@field hash The callback used to compute a hash code for keys as they
		are used to access, add, or remove values in the dictionary.
}
type
	CFDictionaryRetainCallBack = function( allocator: CFAllocatorRef; value: {const} UnivPtr ): UnivPtr;
	CFDictionaryReleaseCallBack = procedure( allocator: CFAllocatorRef; value: {const} UnivPtr );
	CFDictionaryCopyDescriptionCallBack = function( value: {const} UnivPtr ): CFStringRef;
	CFDictionaryEqualCallBack = function( value1: {const} UnivPtr; value2: {const} UnivPtr ): Boolean;
	CFDictionaryHashCallBack = function( value: {const} UnivPtr ): CFHashCode;
	CFDictionaryKeyCallBacks = record
		version: CFIndex;
		retain: CFDictionaryRetainCallBack;
		release: CFDictionaryReleaseCallBack;
		copyDescription: CFDictionaryCopyDescriptionCallBack;
		equal: CFDictionaryEqualCallBack;
		hash: CFDictionaryHashCallBack;
	end;
	CFDictionaryKeyCallBacksPtr = ^CFDictionaryKeyCallBacks;

{!
	@constant kCFTypeDictionaryKeyCallBacks
	Predefined CFDictionaryKeyCallBacks structure containing a
	set of callbacks appropriate for use when the keys of a
	CFDictionary are all CFTypes.
}
var kCFTypeDictionaryKeyCallBacks: CFDictionaryKeyCallBacks; external name '_kCFTypeDictionaryKeyCallBacks'; (* attribute const *)

{!
	@constant kCFCopyStringDictionaryKeyCallBacks
	Predefined CFDictionaryKeyCallBacks structure containing a
	set of callbacks appropriate for use when the keys of a
	CFDictionary are all CFStrings, which may be mutable and
	need to be copied in order to serve as constant keys for
	the values in the dictionary.
}
var kCFCopyStringDictionaryKeyCallBacks: CFDictionaryKeyCallBacks; external name '_kCFCopyStringDictionaryKeyCallBacks'; (* attribute const *)

{!
	@typedef CFDictionaryValueCallBacks
	Structure containing the callbacks for values of a CFDictionary.
	@field version The version number of the structure type being passed
		in as a parameter to the CFDictionary creation functions.
		This structure is version 0.
	@field retain The callback used to add a retain for the dictionary
		on values as they are put into the dictionary.
		This callback returns the value to use as the value in the
		dictionary, which is usually the value parameter passed to
		this callback, but may be a different value if a different
		value should be added to the dictionary. The dictionary's
		allocator is passed as the first argument.
	@field release The callback used to remove a retain previously added
		for the dictionary from values as they are removed from
		the dictionary. The dictionary's allocator is passed as the
		first argument.
	@field copyDescription The callback used to create a descriptive
		string representation of each value in the dictionary. This
		is used by the CFCopyDescription() function.
	@field equal The callback used to compare values in the dictionary for
		equality in some operations.
}
type
	CFDictionaryValueCallBacks = record
		version: CFIndex;
		retain: CFDictionaryRetainCallBack;
		release: CFDictionaryReleaseCallBack;
		copyDescription: CFDictionaryCopyDescriptionCallBack;
		equal: CFDictionaryEqualCallBack;
	end;
	CFDictionaryValueCallBacksPtr = ^CFDictionaryValueCallBacks;

{!
	@constant kCFTypeDictionaryValueCallBacks
	Predefined CFDictionaryValueCallBacks structure containing a set
	of callbacks appropriate for use when the values in a CFDictionary
	are all CFTypes.
}
var kCFTypeDictionaryValueCallBacks: CFDictionaryValueCallBacks; external name '_kCFTypeDictionaryValueCallBacks'; (* attribute const *)

{!
	@typedef CFDictionaryApplierFunction
	Type of the callback function used by the apply functions of
		CFDictionarys.
	@param key The current key for the value.
	@param value The current value from the dictionary.
	@param context The user-defined context parameter given to the apply
		function.
}
type
	CFDictionaryApplierFunction = procedure( key: {const} UnivPtr; value: {const} UnivPtr; context: UnivPtr );

{!
	@typedef CFDictionaryRef
	This is the type of a reference to immutable CFDictionarys.
}
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CFDictionaryRef = ^__CFDictionary; { an opaque type }
	__CFDictionary = record end;
=======
	CFDictionaryRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CFDictionaryRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CFDictionaryRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CFDictionaryRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	CFDictionaryRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> graemeg/fixes_2_2
=======
	CFDictionaryRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> origin/fixes_2_2
=======
	CFDictionaryRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4
	CFDictionaryRefPtr = ^CFDictionaryRef;

{!
	@typedef CFMutableDictionaryRef
	This is the type of a reference to mutable CFDictionarys.
}
type
	CFMutableDictionaryRef = CFDictionaryRef;
	CFMutableDictionaryRefPtr = ^CFMutableDictionaryRef;

{!
	@function CFDictionaryGetTypeID
	Returns the type identifier of all CFDictionary instances.
}
function CFDictionaryGetTypeID: CFTypeID; external name '_CFDictionaryGetTypeID';

{!
	@function CFDictionaryCreate
	Creates a new immutable dictionary with the given values.
	@param allocator The CFAllocator which should be used to allocate
		memory for the dictionary and its storage for values. This
		parameter may be NULL in which case the current default
		CFAllocator is used. If this reference is not a valid
		CFAllocator, the behavior is undefined.
	@param keys A C array of the pointer-sized keys to be used for
		the parallel C array of values to be put into the dictionary.
		This parameter may be NULL if the numValues parameter is 0.
		This C array is not changed or freed by this function. If
		this parameter is not a valid pointer to a C array of at
		least numValues pointers, the behavior is undefined.
	@param values A C array of the pointer-sized values to be in the
		dictionary. This parameter may be NULL if the numValues
		parameter is 0. This C array is not changed or freed by
		this function. If this parameter is not a valid pointer to
		a C array of at least numValues pointers, the behavior is
		undefined.
	@param numValues The number of values to copy from the keys and
		values C arrays into the CFDictionary. This number will be
		the count of the dictionary. If this parameter is
		negative, or greater than the number of values actually
		in the keys or values C arrays, the behavior is undefined.
	@param keyCallBacks A pointer to a CFDictionaryKeyCallBacks structure
		initialized with the callbacks for the dictionary to use on
		each key in the dictionary. The retain callback will be used
		within this function, for example, to retain all of the new
		keys from the keys C array. A copy of the contents of the
		callbacks structure is made, so that a pointer to a structure
		on the stack can be passed in, or can be reused for multiple
		dictionary creations. If the version field of this
		callbacks structure is not one of the defined ones for
		CFDictionary, the behavior is undefined. The retain field may
		be NULL, in which case the CFDictionary will do nothing to add
		a retain to the keys of the contained values. The release field
		may be NULL, in which case the CFDictionary will do nothing
		to remove the dictionary's retain (if any) on the keys when the
		dictionary is destroyed or a key-value pair is removed. If the
		copyDescription field is NULL, the dictionary will create a
		simple description for a key. If the equal field is NULL, the
		dictionary will use pointer equality to test for equality of
		keys. If the hash field is NULL, a key will be converted from
		a pointer to an integer to compute the hash code. This callbacks
		parameter itself may be NULL, which is treated as if a valid
		structure of version 0 with all fields NULL had been passed in.
		Otherwise, if any of the fields are not valid pointers to
		functions of the correct type, or this parameter is not a
		valid pointer to a CFDictionaryKeyCallBacks callbacks structure,
		the behavior is undefined. If any of the keys put into the
		dictionary is not one understood by one of the callback functions
		the behavior when that callback function is used is undefined.
	@param valueCallBacks A pointer to a CFDictionaryValueCallBacks structure
		initialized with the callbacks for the dictionary to use on
		each value in the dictionary. The retain callback will be used
		within this function, for example, to retain all of the new
		values from the values C array. A copy of the contents of the
		callbacks structure is made, so that a pointer to a structure
		on the stack can be passed in, or can be reused for multiple
		dictionary creations. If the version field of this callbacks
		structure is not one of the defined ones for CFDictionary, the
		behavior is undefined. The retain field may be NULL, in which
		case the CFDictionary will do nothing to add a retain to values
		as they are put into the dictionary. The release field may be
		NULL, in which case the CFDictionary will do nothing to remove
		the dictionary's retain (if any) on the values when the
		dictionary is destroyed or a key-value pair is removed. If the
		copyDescription field is NULL, the dictionary will create a
		simple description for a value. If the equal field is NULL, the
		dictionary will use pointer equality to test for equality of
		values. This callbacks parameter itself may be NULL, which is
		treated as if a valid structure of version 0 with all fields
		NULL had been passed in. Otherwise,
		if any of the fields are not valid pointers to functions
		of the correct type, or this parameter is not a valid
		pointer to a CFDictionaryValueCallBacks callbacks structure,
		the behavior is undefined. If any of the values put into the
		dictionary is not one understood by one of the callback functions
		the behavior when that callback function is used is undefined.
	@result A reference to the new immutable CFDictionary.
}
function CFDictionaryCreate( allocator: CFAllocatorRef; {const} keys: UnivPtrPtr; {const} values: UnivPtrPtr; numValues: CFIndex; {const} keyCallBacks: CFDictionaryKeyCallBacksPtr { can be NULL }; {const} valueCallBacks: CFDictionaryValueCallBacksPtr { can be NULL } ): CFDictionaryRef; external name '_CFDictionaryCreate';

{!
	@function CFDictionaryCreateCopy
	Creates a new immutable dictionary with the key-value pairs from
		the given dictionary.
	@param allocator The CFAllocator which should be used to allocate
		memory for the dictionary and its storage for values. This
		parameter may be NULL in which case the current default
		CFAllocator is used. If this reference is not a valid
		CFAllocator, the behavior is undefined.
	@param theDict The dictionary which is to be copied. The keys and values
		from the dictionary are copied as pointers into the new
		dictionary (that is, the values themselves are copied, not
		that which the values point to, if anything). However, the
		keys and values are also retained by the new dictionary using
		the retain function of the original dictionary.
		The count of the new dictionary will be the same as the
		given dictionary. The new dictionary uses the same callbacks
		as the dictionary to be copied. If this parameter is
		not a valid CFDictionary, the behavior is undefined.
	@result A reference to the new immutable CFDictionary.
}
function CFDictionaryCreateCopy( allocator: CFAllocatorRef; theDict: CFDictionaryRef ): CFDictionaryRef; external name '_CFDictionaryCreateCopy';

{!
	@function CFDictionaryCreateMutable
	Creates a new mutable dictionary.
	@param allocator The CFAllocator which should be used to allocate
		memory for the dictionary and its storage for values. This
		parameter may be NULL in which case the current default
		CFAllocator is used. If this reference is not a valid
		CFAllocator, the behavior is undefined.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
#if MAC_OS_X_VERSION_MIN_REQUIRED <= MAC_OS_X_VERSION_10_4
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
#if MAC_OS_X_VERSION_MIN_REQUIRED <= MAC_OS_X_VERSION_10_4
>>>>>>> origin/fixes_2.4
	@param capacity The maximum number of values that can be contained by
		the CFDictionary. The dictionary starts empty, and can grow
		to this number of values (and it can have less). If this
		parameter is 0, the dictionary's maximum capacity is unlimited
		(or rather, only limited by address space and available memory
		constraints). If this parameter is negative, the behavior is
		undefined.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
#else
  @param capacity A hint about the number of values that will be held
    by the CFDictionary. Pass 0 for no hint. The implementation may
    ignore this hint, or may use it to optimize various
    operations. A dictionary's actual capacity is only limited by 
    address space and available memory constraints). If this 
    parameter is negative, the behavior is undefined.
#endif
<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	@param keyCallBacks A pointer to a CFDictionaryKeyCallBacks structure
		initialized with the callbacks for the dictionary to use on
		each key in the dictionary. A copy of the contents of the
		callbacks structure is made, so that a pointer to a structure
		on the stack can be passed in, or can be reused for multiple
		dictionary creations. If the version field of this
		callbacks structure is not one of the defined ones for
		CFDictionary, the behavior is undefined. The retain field may
		be NULL, in which case the CFDictionary will do nothing to add
		a retain to the keys of the contained values. The release field
		may be NULL, in which case the CFDictionary will do nothing
		to remove the dictionary's retain (if any) on the keys when the
		dictionary is destroyed or a key-value pair is removed. If the
		copyDescription field is NULL, the dictionary will create a
		simple description for a key. If the equal field is NULL, the
		dictionary will use pointer equality to test for equality of
		keys. If the hash field is NULL, a key will be converted from
		a pointer to an integer to compute the hash code. This callbacks
		parameter itself may be NULL, which is treated as if a valid
		structure of version 0 with all fields NULL had been passed in.
		Otherwise, if any of the fields are not valid pointers to
		functions of the correct type, or this parameter is not a
		valid pointer to a CFDictionaryKeyCallBacks callbacks structure,
		the behavior is undefined. If any of the keys put into the
		dictionary is not one understood by one of the callback functions
		the behavior when that callback function is used is undefined.
	@param valueCallBacks A pointer to a CFDictionaryValueCallBacks structure
		initialized with the callbacks for the dictionary to use on
		each value in the dictionary. The retain callback will be used
		within this function, for example, to retain all of the new
		values from the values C array. A copy of the contents of the
		callbacks structure is made, so that a pointer to a structure
		on the stack can be passed in, or can be reused for multiple
		dictionary creations. If the version field of this callbacks
		structure is not one of the defined ones for CFDictionary, the
		behavior is undefined. The retain field may be NULL, in which
		case the CFDictionary will do nothing to add a retain to values
		as they are put into the dictionary. The release field may be
		NULL, in which case the CFDictionary will do nothing to remove
		the dictionary's retain (if any) on the values when the
		dictionary is destroyed or a key-value pair is removed. If the
		copyDescription field is NULL, the dictionary will create a
		simple description for a value. If the equal field is NULL, the
		dictionary will use pointer equality to test for equality of
		values. This callbacks parameter itself may be NULL, which is
		treated as if a valid structure of version 0 with all fields
		NULL had been passed in. Otherwise,
		if any of the fields are not valid pointers to functions
		of the correct type, or this parameter is not a valid
		pointer to a CFDictionaryValueCallBacks callbacks structure,
		the behavior is undefined. If any of the values put into the
		dictionary is not one understood by one of the callback functions
		the behavior when that callback function is used is undefined.
	@result A reference to the new mutable CFDictionary.
}
function CFDictionaryCreateMutable( allocator: CFAllocatorRef; capacity: CFIndex; {const} keyCallBacks: CFDictionaryKeyCallBacksPtr { can be NULL }; {const} valueCallBacks: CFDictionaryValueCallBacksPtr { can be NULL } ): CFMutableDictionaryRef; external name '_CFDictionaryCreateMutable';

{!
	@function CFDictionaryCreateMutableCopy
	Creates a new mutable dictionary with the key-value pairs from
		the given dictionary.
	@param allocator The CFAllocator which should be used to allocate
		memory for the dictionary and its storage for values. This
		parameter may be NULL in which case the current default
		CFAllocator is used. If this reference is not a valid
		CFAllocator, the behavior is undefined.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
#if MAC_OS_X_VERSION_MIN_REQUIRED <= MAC_OS_X_VERSION_10_4
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
#if MAC_OS_X_VERSION_MIN_REQUIRED <= MAC_OS_X_VERSION_10_4
>>>>>>> origin/fixes_2.4
	@param capacity The maximum number of values that can be contained
		by the CFDictionary. The dictionary starts empty, and can grow
		to this number of values (and it can have less). If this
		parameter is 0, the dictionary's maximum capacity is unlimited
		(or rather, only limited by address space and available memory
		constraints). This parameter must be greater than or equal
		to the count of the dictionary which is to be copied, or the
		behavior is undefined. If this parameter is negative, the
		behavior is undefined.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
#else
  @param capacity A hint about the number of values that will be held
    by the CFDictionary. Pass 0 for no hint. The implementation may
    ignore this hint, or may use it to optimize various
    operations. A dictionary's actual capacity is only limited by
    address space and available memory constraints). 
    This parameter must be greater than or equal
    to the count of the dictionary which is to be copied, or the
    behavior is undefined. If this parameter is negative, the
    behavior is undefined.
#endif
<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	@param theDict The dictionary which is to be copied. The keys and values
		from the dictionary are copied as pointers into the new
		dictionary (that is, the values themselves are copied, not
		that which the values point to, if anything). However, the
		keys and values are also retained by the new dictionary using
		the retain function of the original dictionary.
		The count of the new dictionary will be the same as the
		given dictionary. The new dictionary uses the same callbacks
		as the dictionary to be copied. If this parameter is
		not a valid CFDictionary, the behavior is undefined.
	@result A reference to the new mutable CFDictionary.
}
function CFDictionaryCreateMutableCopy( allocator: CFAllocatorRef; capacity: CFIndex; theDict: CFDictionaryRef ): CFMutableDictionaryRef; external name '_CFDictionaryCreateMutableCopy';

{!
	@function CFDictionaryGetCount
	Returns the number of values currently in the dictionary.
	@param theDict The dictionary to be queried. If this parameter is
		not a valid CFDictionary, the behavior is undefined.
	@result The number of values in the dictionary.
}
function CFDictionaryGetCount( theDict: CFDictionaryRef ): CFIndex; external name '_CFDictionaryGetCount';

{!
	@function CFDictionaryGetCountOfKey
	Counts the number of times the given key occurs in the dictionary.
	@param theDict The dictionary to be searched. If this parameter is
		not a valid CFDictionary, the behavior is undefined.
	@param key The key for which to find matches in the dictionary. The
		hash() and equal() key callbacks provided when the dictionary
		was created are used to compare. If the hash() key callback
		was NULL, the key is treated as a pointer and converted to
		an integer. If the equal() key callback was NULL, pointer
		equality (in C, ==) is used. If key, or any of the keys in
		the dictionary, are not understood by the equal() callback,
		the behavior is undefined.
	@result Returns 1 if a matching key is used by the dictionary,
		0 otherwise.
}
function CFDictionaryGetCountOfKey( theDict: CFDictionaryRef; key: {const} UnivPtr ): CFIndex; external name '_CFDictionaryGetCountOfKey';

{!
	@function CFDictionaryGetCountOfValue
	Counts the number of times the given value occurs in the dictionary.
	@param theDict The dictionary to be searched. If this parameter is
		not a valid CFDictionary, the behavior is undefined.
	@param value The value for which to find matches in the dictionary. The
		equal() callback provided when the dictionary was created is
		used to compare. If the equal() value callback was NULL, pointer
		equality (in C, ==) is used. If value, or any of the values in
		the dictionary, are not understood by the equal() callback,
		the behavior is undefined.
	@result The number of times the given value occurs in the dictionary.
}
function CFDictionaryGetCountOfValue( theDict: CFDictionaryRef; value: {const} UnivPtr ): CFIndex; external name '_CFDictionaryGetCountOfValue';

{!
	@function CFDictionaryContainsKey
	Reports whether or not the key is in the dictionary.
	@param theDict The dictionary to be searched. If this parameter is
		not a valid CFDictionary, the behavior is undefined.
	@param key The key for which to find matches in the dictionary. The
		hash() and equal() key callbacks provided when the dictionary
		was created are used to compare. If the hash() key callback
		was NULL, the key is treated as a pointer and converted to
		an integer. If the equal() key callback was NULL, pointer
		equality (in C, ==) is used. If key, or any of the keys in
		the dictionary, are not understood by the equal() callback,
		the behavior is undefined.
	@result true, if the key is in the dictionary, otherwise false.
}
function CFDictionaryContainsKey( theDict: CFDictionaryRef; key: {const} UnivPtr ): Boolean; external name '_CFDictionaryContainsKey';

{!
	@function CFDictionaryContainsValue
	Reports whether or not the value is in the dictionary.
	@param theDict The dictionary to be searched. If this parameter is
		not a valid CFDictionary, the behavior is undefined.
	@param value The value for which to find matches in the dictionary. The
		equal() callback provided when the dictionary was created is
		used to compare. If the equal() callback was NULL, pointer
		equality (in C, ==) is used. If value, or any of the values
		in the dictionary, are not understood by the equal() callback,
		the behavior is undefined.
	@result true, if the value is in the dictionary, otherwise false.
}
function CFDictionaryContainsValue( theDict: CFDictionaryRef; value: {const} UnivPtr ): Boolean; external name '_CFDictionaryContainsValue';

{!
	@function CFDictionaryGetValue
	Retrieves the value associated with the given key.
	@param theDict The dictionary to be queried. If this parameter is
		not a valid CFDictionary, the behavior is undefined.
	@param key The key for which to find a match in the dictionary. The
		hash() and equal() key callbacks provided when the dictionary
		was created are used to compare. If the hash() key callback
		was NULL, the key is treated as a pointer and converted to
		an integer. If the equal() key callback was NULL, pointer
		equality (in C, ==) is used. If key, or any of the keys in
		the dictionary, are not understood by the equal() callback,
		the behavior is undefined.
	@result The value with the given key in the dictionary, or NULL if
		no key-value pair with a matching key exists. Since NULL
		can be a valid value in some dictionaries, the function
		CFDictionaryGetValueIfPresent() must be used to distinguish
		NULL-no-found from NULL-is-the-value.
}
function CFDictionaryGetValue( theDict: CFDictionaryRef; key: {const} UnivPtr ): UnivPtr; external name '_CFDictionaryGetValue';

{!
	@function CFDictionaryGetValueIfPresent
	Retrieves the value associated with the given key.
	@param theDict The dictionary to be queried. If this parameter is
		not a valid CFDictionary, the behavior is undefined.
	@param key The key for which to find a match in the dictionary. The
		hash() and equal() key callbacks provided when the dictionary
		was created are used to compare. If the hash() key callback
		was NULL, the key is treated as a pointer and converted to
		an integer. If the equal() key callback was NULL, pointer
		equality (in C, ==) is used. If key, or any of the keys in
		the dictionary, are not understood by the equal() callback,
		the behavior is undefined.
	@param value A pointer to memory which should be filled with the
		pointer-sized value if a matching key is found. If no key
		match is found, the contents of the storage pointed to by
		this parameter are undefined. This parameter may be NULL,
		in which case the value from the dictionary is not returned
		(but the return value of this function still indicates
		whether or not the key-value pair was present).
	@result true, if a matching key was found, false otherwise.
}
function CFDictionaryGetValueIfPresent( theDict: CFDictionaryRef; key: {const} UnivPtr; {const} value: UnivPtrPtr ): Boolean; external name '_CFDictionaryGetValueIfPresent';

{!
	@function CFDictionaryGetKeysAndValues
	Fills the two buffers with the keys and values from the dictionary.
	@param theDict The dictionary to be queried. If this parameter is
		not a valid CFDictionary, the behavior is undefined.
	@param keys A C array of pointer-sized values to be filled with keys
		from the dictionary. The keys and values C arrays are parallel
		to each other (that is, the items at the same indices form a
		key-value pair from the dictionary). This parameter may be NULL
		if the keys are not desired. If this parameter is not a valid
		pointer to a C array of at least CFDictionaryGetCount() pointers,
		or NULL, the behavior is undefined.
	@param values A C array of pointer-sized values to be filled with values
		from the dictionary. The keys and values C arrays are parallel
		to each other (that is, the items at the same indices form a
		key-value pair from the dictionary). This parameter may be NULL
		if the values are not desired. If this parameter is not a valid
		pointer to a C array of at least CFDictionaryGetCount() pointers,
		or NULL, the behavior is undefined.
}
procedure CFDictionaryGetKeysAndValues( theDict: CFDictionaryRef; {const} keys: UnivPtrPtr; {const} values: UnivPtrPtr ); external name '_CFDictionaryGetKeysAndValues';

{!
	@function CFDictionaryApplyFunction
	Calls a function once for each value in the dictionary.
	@param theDict The dictionary to be queried. If this parameter is
		not a valid CFDictionary, the behavior is undefined.
	@param applier The callback function to call once for each value in
		the dictionary. If this parameter is not a
		pointer to a function of the correct prototype, the behavior
		is undefined. If there are keys or values which the
		applier function does not expect or cannot properly apply
		to, the behavior is undefined. 
	@param context A pointer-sized user-defined value, which is passed
		as the third parameter to the applier function, but is
		otherwise unused by this function. If the context is not
		what is expected by the applier function, the behavior is
		undefined.
}
procedure CFDictionaryApplyFunction( theDict: CFDictionaryRef; applier: CFDictionaryApplierFunction; context: UnivPtr ); external name '_CFDictionaryApplyFunction';

{!
	@function CFDictionaryAddValue
	Adds the key-value pair to the dictionary if no such key already exists.
	@param theDict The dictionary to which the value is to be added. If this
		parameter is not a valid mutable CFDictionary, the behavior is
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		undefined.
#if MAC_OS_X_VERSION_MIN_REQUIRED <= MAC_OS_X_VERSION_10_4
		If the dictionary is a fixed-capacity dictionary and
		it is full before this operation, the behavior is undefined.
#endif
=======
		undefined. If the dictionary is a fixed-capacity dictionary and
		it is full before this operation, the behavior is undefined.
>>>>>>> graemeg/fixes_2_2
=======
		undefined. If the dictionary is a fixed-capacity dictionary and
		it is full before this operation, the behavior is undefined.
>>>>>>> origin/fixes_2_2
=======
		undefined.
#if MAC_OS_X_VERSION_MIN_REQUIRED <= MAC_OS_X_VERSION_10_4
		If the dictionary is a fixed-capacity dictionary and
		it is full before this operation, the behavior is undefined.
#endif
>>>>>>> origin/fixes_2.4
	@param key The key of the value to add to the dictionary. The key is
		retained by the dictionary using the retain callback provided
		when the dictionary was created. If the key is not of the sort
		expected by the retain callback, the behavior is undefined. If
		a key which matches this key is already present in the dictionary,
		this function does nothing ("add if absent").
	@param value The value to add to the dictionary. The value is retained
		by the dictionary using the retain callback provided when the
		dictionary was created. If the value is not of the sort expected
		by the retain callback, the behavior is undefined.
}
procedure CFDictionaryAddValue( theDict: CFMutableDictionaryRef; key: {const} UnivPtr; value: {const} UnivPtr ); external name '_CFDictionaryAddValue';

{!
	@function CFDictionarySetValue
	Sets the value of the key in the dictionary.
	@param theDict The dictionary to which the value is to be set. If this
		parameter is not a valid mutable CFDictionary, the behavior is
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		undefined.
#if MAC_OS_X_VERSION_MIN_REQUIRED <= MAC_OS_X_VERSION_10_4
		If the dictionary is a fixed-capacity dictionary and
		it is full before this operation, and the key does not exist in
		the dictionary, the behavior is undefined.
#endif
=======
		undefined. If the dictionary is a fixed-capacity dictionary and
		it is full before this operation, and the key does not exist in
		the dictionary, the behavior is undefined.
>>>>>>> graemeg/fixes_2_2
=======
		undefined. If the dictionary is a fixed-capacity dictionary and
		it is full before this operation, and the key does not exist in
		the dictionary, the behavior is undefined.
>>>>>>> origin/fixes_2_2
=======
		undefined.
#if MAC_OS_X_VERSION_MIN_REQUIRED <= MAC_OS_X_VERSION_10_4
		If the dictionary is a fixed-capacity dictionary and
		it is full before this operation, and the key does not exist in
		the dictionary, the behavior is undefined.
#endif
>>>>>>> origin/fixes_2.4
	@param key The key of the value to set into the dictionary. If a key 
		which matches this key is already present in the dictionary, only
		the value is changed ("add if absent, replace if present"). If
		no key matches the given key, the key-value pair is added to the
		dictionary. If added, the key is retained by the dictionary,
		using the retain callback provided
		when the dictionary was created. If the key is not of the sort
		expected by the key retain callback, the behavior is undefined.
	@param value The value to add to or replace into the dictionary. The value
		is retained by the dictionary using the retain callback provided
		when the dictionary was created, and the previous value if any is
		released. If the value is not of the sort expected by the
		retain or release callbacks, the behavior is undefined.
}
procedure CFDictionarySetValue( theDict: CFMutableDictionaryRef; key: {const} UnivPtr; value: {const} UnivPtr ); external name '_CFDictionarySetValue';

{!
	@function CFDictionaryReplaceValue
	Replaces the value of the key in the dictionary.
	@param theDict The dictionary to which the value is to be replaced. If this
		parameter is not a valid mutable CFDictionary, the behavior is
		undefined.
	@param key The key of the value to replace in the dictionary. If a key 
		which matches this key is present in the dictionary, the value
		is changed to the given value, otherwise this function does
		nothing ("replace if present").
	@param value The value to replace into the dictionary. The value
		is retained by the dictionary using the retain callback provided
		when the dictionary was created, and the previous value is
		released. If the value is not of the sort expected by the
		retain or release callbacks, the behavior is undefined.
}
procedure CFDictionaryReplaceValue( theDict: CFMutableDictionaryRef; key: {const} UnivPtr; value: {const} UnivPtr ); external name '_CFDictionaryReplaceValue';

{!
	@function CFDictionaryRemoveValue
	Removes the value of the key from the dictionary.
	@param theDict The dictionary from which the value is to be removed. If this
		parameter is not a valid mutable CFDictionary, the behavior is
		undefined.
	@param key The key of the value to remove from the dictionary. If a key 
		which matches this key is present in the dictionary, the key-value
		pair is removed from the dictionary, otherwise this function does
		nothing ("remove if present").
}
procedure CFDictionaryRemoveValue( theDict: CFMutableDictionaryRef; key: {const} UnivPtr ); external name '_CFDictionaryRemoveValue';

{!
	@function CFDictionaryRemoveAllValues
	Removes all the values from the dictionary, making it empty.
	@param theDict The dictionary from which all of the values are to be
		removed. If this parameter is not a valid mutable
		CFDictionary, the behavior is undefined.
}
procedure CFDictionaryRemoveAllValues( theDict: CFMutableDictionaryRef ); external name '_CFDictionaryRemoveAllValues';

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
