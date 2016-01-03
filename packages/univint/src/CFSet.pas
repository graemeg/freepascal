{	CFSet.h
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

unit CFSet;
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
=======
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
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
	{$setc TARGET_CPU_ARM64 := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
=======
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
>>>>>>> graemeg/cpstrnew
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
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
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
=======
>>>>>>> graemeg/cpstrnew
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := TRUE}
{$elifc defined __arm64__ and __arm64__}
=======
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
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_CPU_ARM64 := TRUE}
=======
	{$setc TARGET_CPU_ARM := TRUE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_CPU_ARM := TRUE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_CPU_ARM := TRUE}
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
        @header CFSet
        CFSet implements a container which stores unique values.
}


{!
	@typedef CFSetRetainCallBack
	Type of the callback function used by CFSets for retaining values.
        @param allocator The allocator of the CFSet.
	@param value The value to retain.
        @result The value to store in the set, which is usually the value
		parameter passed to this callback, but may be a different
		value if a different value should be stored in the set.
}
type
	CFSetRetainCallBack = function( allocator: CFAllocatorRef; value: {const} UnivPtr ): UnivPtr;

{!
	@typedef CFSetReleaseCallBack
	Type of the callback function used by CFSets for releasing a retain on values.
        @param allocator The allocator of the CFSet.
	@param value The value to release.
}
type
	CFSetReleaseCallBack = procedure( allocator: CFAllocatorRef; value: {const} UnivPtr );

{!
	@typedef CFSetCopyDescriptionCallBack
	Type of the callback function used by CFSets for describing values.
	@param value The value to describe.
        @result A description of the specified value.
}
type
	CFSetCopyDescriptionCallBack = function( value: {const} UnivPtr ): CFStringRef;

{!
	@typedef CFSetEqualCallBack
	Type of the callback function used by CFSets for comparing values.
	@param value1 The first value to compare.
	@param value2 The second value to compare.
        @result True if the values are equal, otherwise false.
}
type
	CFSetEqualCallBack = function( value1: {const} UnivPtr; value2: {const} UnivPtr ): Boolean;

{!
	@typedef CFSetHashCallBack
	Type of the callback function used by CFSets for hashing values.
	@param value The value to hash.
        @result The hash of the value.
}
type
	CFSetHashCallBack = function( value: {const} UnivPtr ): CFHashCode;

{!
	@typedef CFSetCallBacks
	Structure containing the callbacks of a CFSet.
	@field version The version number of the structure type being passed
		in as a parameter to the CFSet creation functions. This
		structure is version 0.
	@field retain The callback used to add a retain for the set on
		values as they are put into the set. This callback returns
		the value to store in the set, which is usually the value
		parameter passed to this callback, but may be a different
		value if a different value should be stored in the set.
		The set's allocator is passed as the first argument.
	@field release The callback used to remove a retain previously added
		for the set from values as they are removed from the
		set. The set's allocator is passed as the first
		argument.
	@field copyDescription The callback used to create a descriptive
		string representation of each value in the set. This is
		used by the CFCopyDescription() function.
	@field equal The callback used to compare values in the set for
		equality for some operations.
	@field hash The callback used to compare values in the set for
		uniqueness for some operations.
}
type
	CFSetCallBacks = record
		version: CFIndex;
		retain: CFSetRetainCallBack;
		release: CFSetReleaseCallBack;
		copyDescription: CFSetCopyDescriptionCallBack;
		equal: CFSetEqualCallBack;
		hash: CFSetHashCallBack;
	end;
	CFSetCallBacksPtr = ^CFSetCallBacks;

{!
	@constant kCFTypeSetCallBacks
	Predefined CFSetCallBacks structure containing a set of callbacks
	appropriate for use when the values in a CFSet are all CFTypes.
}
var kCFTypeSetCallBacks: CFSetCallBacks; external name '_kCFTypeSetCallBacks'; (* attribute const *)

{!
	@constant kCFCopyStringSetCallBacks
	Predefined CFSetCallBacks structure containing a set of callbacks
	appropriate for use when the values in a CFSet should be copies
        of a CFString.
}
var kCFCopyStringSetCallBacks: CFSetCallBacks; external name '_kCFCopyStringSetCallBacks'; (* attribute const *)

{!
	@typedef CFSetApplierFunction
	Type of the callback function used by the apply functions of
		CFSets.
	@param value The current value from the set.
	@param context The user-defined context parameter given to the apply
		function.
}
type
	CFSetApplierFunction = procedure( value: {const} UnivPtr; context: UnivPtr );

{!
        @typedef CFSetRef
	This is the type of a reference to immutable CFSets.
}
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CFSetRef = ^__CFSet; { an opaque type }
	__CFSet = record end;
=======
	CFSetRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CFSetRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CFSetRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CFSetRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	CFSetRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> graemeg/fixes_2_2
=======
	CFSetRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> origin/fixes_2_2
=======
	CFSetRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4
	CFSetRefPtr = ^CFSetRef;

{!
        @typedef CFMutableSetRef
	This is the type of a reference to mutable CFSets.
}
type
	CFMutableSetRef = CFSetRef;
	CFMutableSetRefPtr = ^CFMutableSetRef;

{!
        @function CFSetGetTypeID
        Returns the type identifier of all CFSet instances.
}
function CFSetGetTypeID: CFTypeID; external name '_CFSetGetTypeID';

{!
        @function CFSetCreate
        Creates a new immutable set with the given values.
	@param allocator The CFAllocator which should be used to allocate
		memory for the set and its storage for values. This
		parameter may be NULL in which case the current default
		CFAllocator is used. If this reference is not a valid
		CFAllocator, the behavior is undefined.
	@param values A C array of the pointer-sized values to be in the
		set.  This C array is not changed or freed by this function. 
                If this parameter is not a valid pointer to a C array of at
                least numValues pointers, the behavior is undefined.
	@param numValues The number of values to copy from the values C
		array into the CFSet. This number will be the count of the
		set.  If this parameter is zero, negative, or greater than 
                the number of values actually in the values C array, the 
                behavior is undefined.
	@param callBacks A C pointer to a CFSetCallBacks structure
		initialized with the callbacks for the set to use on each
		value in the set. A copy of the contents of the
		callbacks structure is made, so that a pointer to a
		structure on the stack can be passed in, or can be reused
		for multiple set creations. If the version field of this
		callbacks structure is not one of the defined ones for
		CFSet, the behavior is undefined. The retain field may be
		NULL, in which case the CFSet will do nothing to add a
		retain to the contained values for the set. The release
		field may be NULL, in which case the CFSet will do nothing
		to remove the set's retain (if any) on the values when the
		set is destroyed. If the copyDescription field is NULL,
		the set will create a simple description for the value. If
		the equal field is NULL, the set will use pointer equality
		to test for equality of values. The hash field may be NULL,
                in which case the CFSet will determine uniqueness by pointer
                equality. This callbacks parameter
		itself may be NULL, which is treated as if a valid structure
		of version 0 with all fields NULL had been passed in.
		Otherwise, if any of the fields are not valid pointers to
		functions of the correct type, or this parameter is not a
		valid pointer to a  CFSetCallBacks callbacks structure,
		the behavior is undefined. If any of the values put into the
		set is not one understood by one of the callback functions
		the behavior when that callback function is used is
		undefined.
	@result A reference to the new immutable CFSet.
}
function CFSetCreate( allocator: CFAllocatorRef; {const} values: UnivPtrPtr; numValues: CFIndex; {const} callBacks: CFSetCallBacksPtr { can be NULL } ): CFSetRef; external name '_CFSetCreate';

{!
	@function CFSetCreateCopy
	Creates a new immutable set with the values from the given set.
	@param allocator The CFAllocator which should be used to allocate
		memory for the set and its storage for values. This
		parameter may be NULL in which case the current default
		CFAllocator is used. If this reference is not a valid
		CFAllocator, the behavior is undefined.
	@param theSet The set which is to be copied. The values from the
		set are copied as pointers into the new set (that is,
		the values themselves are copied, not that which the values
		point to, if anything). However, the values are also
		retained by the new set. The count of the new set will
		be the same as the copied set. The new set uses the same
		callbacks as the set to be copied. If this parameter is
		not a valid CFSet, the behavior is undefined.
	@result A reference to the new immutable CFSet.
}
function CFSetCreateCopy( allocator: CFAllocatorRef; theSet: CFSetRef ): CFSetRef; external name '_CFSetCreateCopy';

{!
	@function CFSetCreateMutable
	Creates a new empty mutable set.
	@param allocator The CFAllocator which should be used to allocate
		memory for the set and its storage for values. This
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
		by the CFSet. The set starts empty, and can grow to this
		number of values (and it can have less). If this parameter
		is 0, the set's maximum capacity is unlimited (or rather,
		only limited by address space and available memory
		constraints). If this parameter is negative, the behavior is
		undefined.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
#else
  @param capacity A hint about the number of values that will be held
    by the CFSet. Pass 0 for no hint. The implementation may
    ignore this hint, or may use it to optimize various
    operations. A set's actual capacity is only limited by 
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
	@param callBacks A C pointer to a CFSetCallBacks structure
		initialized with the callbacks for the set to use on each
		value in the set. A copy of the contents of the
		callbacks structure is made, so that a pointer to a
		structure on the stack can be passed in, or can be reused
		for multiple set creations. If the version field of this
		callbacks structure is not one of the defined ones for
		CFSet, the behavior is undefined. The retain field may be
		NULL, in which case the CFSet will do nothing to add a
		retain to the contained values for the set. The release
		field may be NULL, in which case the CFSet will do nothing
		to remove the set's retain (if any) on the values when the
		set is destroyed. If the copyDescription field is NULL,
		the set will create a simple description for the value. If
		the equal field is NULL, the set will use pointer equality
		to test for equality of values. The hash field may be NULL,
                in which case the CFSet will determine uniqueness by pointer
                equality. This callbacks parameter
		itself may be NULL, which is treated as if a valid structure
		of version 0 with all fields NULL had been passed in.
		Otherwise, if any of the fields are not valid pointers to
		functions of the correct type, or this parameter is not a
		valid pointer to a  CFSetCallBacks callbacks structure,
		the behavior is undefined. If any of the values put into the
		set is not one understood by one of the callback functions
		the behavior when that callback function is used is
		undefined.
	@result A reference to the new mutable CFSet.
}
function CFSetCreateMutable( allocator: CFAllocatorRef; capacity: CFIndex; {const} callBacks: CFSetCallBacksPtr { can be NULL } ): CFMutableSetRef; external name '_CFSetCreateMutable';

{!
	@function CFSetCreateMutableCopy
	Creates a new immutable set with the values from the given set.
	@param allocator The CFAllocator which should be used to allocate
		memory for the set and its storage for values. This
		parameter may be NULL in which case the current default
		CFAllocator is used. If this reference is not a valid
		CFAllocator, the behavior is undefined.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
#if MAC_OS_X_VERSION_MIN_REQUIRED <= MAC_OS_X_VERSION_10_4
	@param capacity The maximum number of values that can be contained
		by the CFSet. The set starts with the same values as the
    set to be copied, and can grow to this number of values.
    If this parameter is 0, the set's maximum capacity is 
    unlimited (or rather, only limited by address space and 
    available memory constraints). This parameter must be 
    greater than or equal to the count of the set which is to
    be copied, or the behavior is undefined.
#else
  @param capacity A hint about the number of values that will be held
    by the CFSet. Pass 0 for no hint. The implementation may
    ignore this hint, or may use it to optimize various
    operations. A set's actual capacity is only limited by
    address space and available memory constraints). 
    This parameter must be greater than or equal
    to the count of the set which is to be copied, or the
    behavior is undefined. If this parameter is negative, the
    behavior is undefined.
#endif
=======
=======
>>>>>>> origin/fixes_2_2
	@param capacity The maximum number of values that can be contained
		by the CFSet. The set starts with the same values as the
                set to be copied, and can grow to this number of values.
                If this parameter is 0, the set's maximum capacity is 
                unlimited (or rather, only limited by address space and 
                available memory constraints). This parameter must be 
                greater than or equal to the count of the set which is to
                be copied, or the behavior is undefined.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
#if MAC_OS_X_VERSION_MIN_REQUIRED <= MAC_OS_X_VERSION_10_4
	@param capacity The maximum number of values that can be contained
		by the CFSet. The set starts with the same values as the
    set to be copied, and can grow to this number of values.
    If this parameter is 0, the set's maximum capacity is 
    unlimited (or rather, only limited by address space and 
    available memory constraints). This parameter must be 
    greater than or equal to the count of the set which is to
    be copied, or the behavior is undefined.
#else
  @param capacity A hint about the number of values that will be held
    by the CFSet. Pass 0 for no hint. The implementation may
    ignore this hint, or may use it to optimize various
    operations. A set's actual capacity is only limited by
    address space and available memory constraints). 
    This parameter must be greater than or equal
    to the count of the set which is to be copied, or the
    behavior is undefined. If this parameter is negative, the
    behavior is undefined.
#endif
>>>>>>> origin/fixes_2.4
	@param theSet The set which is to be copied. The values from the
		set are copied as pointers into the new set (that is,
		the values themselves are copied, not that which the values
		point to, if anything). However, the values are also
		retained by the new set. The count of the new set will
		be the same as the copied set. The new set uses the same
		callbacks as the set to be copied. If this parameter is
		not a valid CFSet, the behavior is undefined.
	@result A reference to the new mutable CFSet.
}
function CFSetCreateMutableCopy( allocator: CFAllocatorRef; capacity: CFIndex; theSet: CFSetRef ): CFMutableSetRef; external name '_CFSetCreateMutableCopy';

{!
	@function CFSetGetCount
	Returns the number of values currently in the set.
	@param theSet The set to be queried. If this parameter is not a valid
		CFSet, the behavior is undefined.
	@result The number of values in the set.
}
function CFSetGetCount( theSet: CFSetRef ): CFIndex; external name '_CFSetGetCount';

{!
	@function CFSetGetCountOfValue
	Counts the number of times the given value occurs in the set. Since 
        sets by definition contain only one instance of a value, this function
<<<<<<< HEAD
<<<<<<< HEAD
        is synonymous to CFSetContainsValue.
=======
        is synomous to SFSetContainsValue.
>>>>>>> graemeg/fixes_2_2
=======
        is synomous to SFSetContainsValue.
>>>>>>> origin/fixes_2_2
	@param theSet The set to be searched. If this parameter is not a
		valid CFSet, the behavior is undefined.
	@param value The value for which to find matches in the set. The
		equal() callback provided when the set was created is
		used to compare. If the equal() callback was NULL, pointer
		equality (in C, ==) is used. If value, or any of the values
		in the set, are not understood by the equal() callback,
		the behavior is undefined.
	@result The number of times the given value occurs in the set.
}
function CFSetGetCountOfValue( theSet: CFSetRef; value: {const} UnivPtr ): CFIndex; external name '_CFSetGetCountOfValue';

{!
	@function CFSetContainsValue
	Reports whether or not the value is in the set.
	@param theSet The set to be searched. If this parameter is not a
		valid CFSet, the behavior is undefined.
	@param value The value for which to find matches in the set. The
		equal() callback provided when the set was created is
		used to compare. If the equal() callback was NULL, pointer
		equality (in C, ==) is used. If value, or any of the values
		in the set, are not understood by the equal() callback,
		the behavior is undefined.
	@result true, if the value is in the set, otherwise false.
}
function CFSetContainsValue( theSet: CFSetRef; value: {const} UnivPtr ): Boolean; external name '_CFSetContainsValue';

{!
	@function CFSetGetValue
	Retrieves a value in the set which hashes the same as the specified value.
	@param theSet The set to be queried. If this parameter is not a
		valid CFSet, the behavior is undefined.
	@param value The value to retrieve. The equal() callback provided when
                the set was created is used to compare. If the equal() callback
                was NULL, pointer equality (in C, ==) is used. If a value, or
                any of the values in the set, are not understood by the equal()
                callback, the behavior is undefined.
        @result The value in the set with the given hash.
}
function CFSetGetValue( theSet: CFSetRef; value: {const} UnivPtr ): UnivPtr; external name '_CFSetGetValue';

{!
<<<<<<< HEAD
<<<<<<< HEAD
	@function CFSetGetValueIfPresent
=======
	@function CFSetGetValue
>>>>>>> graemeg/fixes_2_2
=======
	@function CFSetGetValue
>>>>>>> origin/fixes_2_2
	Retrieves a value in the set which hashes the same as the specified value,
        if present.
	@param theSet The set to be queried. If this parameter is not a
		valid CFSet, the behavior is undefined.
	@param candidate This value is hashed and compared with values in the
                set to determine which value to retrieve. The equal() callback provided when
                the set was created is used to compare. If the equal() callback
                was NULL, pointer equality (in C, ==) is used. If a value, or
                any of the values in the set, are not understood by the equal()
                callback, the behavior is undefined.
	@param value A pointer to memory which should be filled with the
		pointer-sized value if a matching value is found. If no
		match is found, the contents of the storage pointed to by
		this parameter are undefined. This parameter may be NULL,
		in which case the value from the dictionary is not returned
		(but the return value of this function still indicates
		whether or not the value was present).
        @result True if the value was present in the set, otherwise false.
}
function CFSetGetValueIfPresent( theSet: CFSetRef; candidate: {const} UnivPtr; {const} value: UnivPtrPtr ): Boolean; external name '_CFSetGetValueIfPresent';

{!
	@function CFSetGetValues
	Fills the buffer with values from the set.
	@param theSet The set to be queried. If this parameter is not a
		valid CFSet, the behavior is undefined.
	@param values A C array of pointer-sized values to be filled with
		values from the set. The values in the C array are ordered
		in the same order in which they appear in the set. If this
		parameter is not a valid pointer to a C array of at least
		CFSetGetCount() pointers, the behavior is undefined.
}
procedure CFSetGetValues( theSet: CFSetRef; {const} values: UnivPtrPtr ); external name '_CFSetGetValues';

{!
	@function CFSetApplyFunction
	Calls a function once for each value in the set.
	@param theSet The set to be operated upon. If this parameter is not
		a valid CFSet, the behavior is undefined.
	@param applier The callback function to call once for each value in
		the given set. If this parameter is not a
		pointer to a function of the correct prototype, the behavior
		is undefined. If there are values in the set which the
		applier function does not expect or cannot properly apply
		to, the behavior is undefined. 
	@param context A pointer-sized user-defined value, which is passed
		as the second parameter to the applier function, but is
		otherwise unused by this function. If the context is not
		what is expected by the applier function, the behavior is
		undefined.
}
procedure CFSetApplyFunction( theSet: CFSetRef; applier: CFSetApplierFunction; context: UnivPtr ); external name '_CFSetApplyFunction';

{!
	@function CFSetAddValue
	Adds the value to the set if it is not already present.
	@param theSet The set to which the value is to be added. If this
		parameter is not a valid mutable CFSet, the behavior is
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		undefined.
#if MAC_OS_X_VERSION_MIN_REQUIRED <= MAC_OS_X_VERSION_10_4
		If the set is a fixed-capacity set and it
		is full before this operation, the behavior is undefined.
#endif
=======
		undefined. If the set is a fixed-capacity set and it
		is full before this operation, the behavior is undefined.
>>>>>>> graemeg/fixes_2_2
=======
		undefined. If the set is a fixed-capacity set and it
		is full before this operation, the behavior is undefined.
>>>>>>> origin/fixes_2_2
=======
		undefined.
#if MAC_OS_X_VERSION_MIN_REQUIRED <= MAC_OS_X_VERSION_10_4
		If the set is a fixed-capacity set and it
		is full before this operation, the behavior is undefined.
#endif
>>>>>>> origin/fixes_2.4
	@param value The value to add to the set. The value is retained by
		the set using the retain callback provided when the set
		was created. If the value is not of the sort expected by the
		retain callback, the behavior is undefined. The count of the 
                set is increased by one.
}
procedure CFSetAddValue( theSet: CFMutableSetRef; value: {const} UnivPtr ); external name '_CFSetAddValue';

{!
	@function CFSetReplaceValue
	Replaces the value in the set if it is present.
	@param theSet The set to which the value is to be replaced. If this
		parameter is not a valid mutable CFSet, the behavior is
		undefined.
        @param value The value to replace in the set. The equal() callback provided when
                the set was created is used to compare. If the equal() callback
                was NULL, pointer equality (in C, ==) is used. If a value, or
                any of the values in the set, are not understood by the equal()
                callback, the behavior is undefined. The value is retained by
		the set using the retain callback provided when the set
		was created. If the value is not of the sort expected by the
		retain callback, the behavior is undefined. The count of the 
                set is increased by one.
}
procedure CFSetReplaceValue( theSet: CFMutableSetRef; value: {const} UnivPtr ); external name '_CFSetReplaceValue';

{!
	@function CFSetSetValue
	Replaces the value in the set if it is present, or adds the value to 
        the set if it is absent.
	@param theSet The set to which the value is to be replaced. If this
		parameter is not a valid mutable CFSet, the behavior is
		undefined.
        @param value The value to set in the CFSet. The equal() callback provided when
                the set was created is used to compare. If the equal() callback
                was NULL, pointer equality (in C, ==) is used. If a value, or
                any of the values in the set, are not understood by the equal()
                callback, the behavior is undefined. The value is retained by
		the set using the retain callback provided when the set
		was created. If the value is not of the sort expected by the
		retain callback, the behavior is undefined. The count of the 
                set is increased by one.
}
procedure CFSetSetValue( theSet: CFMutableSetRef; value: {const} UnivPtr ); external name '_CFSetSetValue';

{!
	@function CFSetRemoveValue
	Removes the specified value from the set.
	@param theSet The set from which the value is to be removed.
                If this parameter is not a valid mutable CFSet,
		the behavior is undefined.
        @param value The value to remove. The equal() callback provided when
                the set was created is used to compare. If the equal() callback
                was NULL, pointer equality (in C, ==) is used. If a value, or
                any of the values in the set, are not understood by the equal()
                callback, the behavior is undefined.
}
procedure CFSetRemoveValue( theSet: CFMutableSetRef; value: {const} UnivPtr ); external name '_CFSetRemoveValue';

{!
	@function CFSetRemoveAllValues
	Removes all the values from the set, making it empty.
	@param theSet The set from which all of the values are to be
		removed. If this parameter is not a valid mutable CFSet,
		the behavior is undefined.
}
procedure CFSetRemoveAllValues( theSet: CFMutableSetRef ); external name '_CFSetRemoveAllValues';

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
