{	CFPropertyList.h
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

unit CFPropertyList;
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
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/cpstrnew
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
=======
>>>>>>> origin/cpstrnew
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
	{$setc TARGET_CPU_ARM64 := FALSE}
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
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
=======
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
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
uses MacTypes,CFBase,CFData,CFError,CFString,CFStream;
{$endc} {not MACOSALLINCLUDE}

=======
uses MacTypes,CFBase,CFData,CFString,CFStream;
>>>>>>> graemeg/fixes_2_2
=======
uses MacTypes,CFBase,CFData,CFString,CFStream;
>>>>>>> origin/fixes_2_2
=======
uses MacTypes,CFBase,CFData,CFError,CFString,CFStream;
{$endc} {not MACOSALLINCLUDE}

>>>>>>> origin/fixes_2.4
{$ALIGN POWER}


type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CFPropertyListMutabilityOptions = CFOptionFlags;
=======
	CFPropertyListMutabilityOptions = UNSIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	CFPropertyListMutabilityOptions = UNSIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	CFPropertyListMutabilityOptions = UNSIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	CFPropertyListMutabilityOptions = UNSIGNEDLONG;
>>>>>>> origin/cpstrnew
=======
	CFPropertyListMutabilityOptions = SInt32;
>>>>>>> graemeg/fixes_2_2
=======
	CFPropertyListMutabilityOptions = SInt32;
>>>>>>> origin/fixes_2_2
=======
	CFPropertyListMutabilityOptions = UNSIGNEDLONG;
>>>>>>> origin/fixes_2.4
const
	kCFPropertyListImmutable = 0;
	kCFPropertyListMutableContainers = 1;
	kCFPropertyListMutableContainersAndLeaves = 2;
    
{
	Creates a property list object from its XML description; xmlData should
	be the raw bytes of that description, possibly the contents of an XML
	file. Returns NULL if the data cannot be parsed; if the parse fails
	and errorString is non-NULL, a human-readable description of the failure
	is returned in errorString. It is the caller's responsibility to release
	either the returned object or the error string, whichever is applicable.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 
        This function is obsolete and will be deprecated soon. See CFPropertyListCreateWithData() for a replacement.
=======

  This function is obsolete and will be deprecated soon. See CFPropertyListCreateWithData() for a replacement.
>>>>>>> graemeg/cpstrnew
=======

  This function is obsolete and will be deprecated soon. See CFPropertyListCreateWithData() for a replacement.
>>>>>>> graemeg/cpstrnew
=======

  This function is obsolete and will be deprecated soon. See CFPropertyListCreateWithData() for a replacement.
>>>>>>> graemeg/cpstrnew
=======

  This function is obsolete and will be deprecated soon. See CFPropertyListCreateWithData() for a replacement.
>>>>>>> origin/cpstrnew
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

  This function is obsolete and will be deprecated soon. See CFPropertyListCreateWithData() for a replacement.
>>>>>>> origin/fixes_2.4
}
function CFPropertyListCreateFromXMLData( allocator: CFAllocatorRef; xmlData: CFDataRef; mutabilityOption: CFOptionFlags; errorString: CFStringRefPtr ): CFPropertyListRef; external name '_CFPropertyListCreateFromXMLData';

{
	Returns the XML description of the given object; propertyList must
	be one of the supported property list types, and (for composite types
	like CFArray and CFDictionary) must not contain any elements that
	are not themselves of a property list type. If a non-property list
	type is encountered, NULL is returned. The returned data is
	appropriate for writing out to an XML file. Note that a data, not a
	string, is returned because the bytes contain in them a description
	of the string encoding used.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 
        This function is obsolete and will be deprecated soon. See CFPropertyListCreateData() for a replacement.
=======

  This function is obsolete and will be deprecated soon. See CFPropertyListCreateData() for a replacement.
>>>>>>> graemeg/cpstrnew
=======

  This function is obsolete and will be deprecated soon. See CFPropertyListCreateData() for a replacement.
>>>>>>> graemeg/cpstrnew
=======

  This function is obsolete and will be deprecated soon. See CFPropertyListCreateData() for a replacement.
>>>>>>> graemeg/cpstrnew
=======

  This function is obsolete and will be deprecated soon. See CFPropertyListCreateData() for a replacement.
>>>>>>> origin/cpstrnew
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

  This function is obsolete and will be deprecated soon. See CFPropertyListCreateData() for a replacement.
>>>>>>> origin/fixes_2.4
}
function CFPropertyListCreateXMLData( allocator: CFAllocatorRef; propertyList: CFPropertyListRef ): CFDataRef; external name '_CFPropertyListCreateXMLData';

{
	Recursively creates a copy of the given property list (so nested arrays
	and dictionaries are copied as well as the top-most container). The
	resulting property list has the mutability characteristics determined
	by mutabilityOption.
}
function CFPropertyListCreateDeepCopy( allocator: CFAllocatorRef; propertyList: CFPropertyListRef; mutabilityOption: CFOptionFlags ): CFPropertyListRef; external name '_CFPropertyListCreateDeepCopy';

{#if MAC_OS_X_VERSION_10_2 <= MAC_OS_X_VERSION_MAX_ALLOWED}

type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CFPropertyListFormat = CFIndex;
=======
	CFPropertyListFormat = SIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	CFPropertyListFormat = SIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	CFPropertyListFormat = SIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	CFPropertyListFormat = SIGNEDLONG;
>>>>>>> origin/cpstrnew
	CFPropertyListFormatPtr = ^CFPropertyListFormat;

=======
	CFPropertyListFormat = SInt32;
>>>>>>> graemeg/fixes_2_2
=======
	CFPropertyListFormat = SInt32;
>>>>>>> origin/fixes_2_2
=======
	CFPropertyListFormat = SIGNEDLONG;
	CFPropertyListFormatPtr = ^CFPropertyListFormat;

>>>>>>> origin/fixes_2.4
const
    kCFPropertyListOpenStepFormat = 1;
    kCFPropertyListXMLFormat_v1_0 = 100;
    kCFPropertyListBinaryFormat_v1_0 = 200;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
function CFPropertyListIsValid( plist: CFPropertyListRef; format: CFPropertyListFormat ): Boolean; external name '_CFPropertyListIsValid';

>>>>>>> graemeg/fixes_2_2
=======
function CFPropertyListIsValid( plist: CFPropertyListRef; format: CFPropertyListFormat ): Boolean; external name '_CFPropertyListIsValid';

>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{ Returns true if the object graph rooted at plist is a valid property list
 * graph -- that is, no cycles, containing only plist objects, and dictionary
 * keys are strings. The debugging library version spits out some messages
 * to be helpful. The plist structure which is to be allowed is given by
 * the format parameter. }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function CFPropertyListIsValid( plist: CFPropertyListRef; format: CFPropertyListFormat ): Boolean; external name '_CFPropertyListIsValid';
=======

function CFPropertyListWriteToStream( propertyList: CFPropertyListRef; stream: CFWriteStreamRef; format: CFPropertyListFormat; var errorString: CFStringRef ): CFIndex; external name '_CFPropertyListWriteToStream';
>>>>>>> graemeg/fixes_2_2
=======

function CFPropertyListWriteToStream( propertyList: CFPropertyListRef; stream: CFWriteStreamRef; format: CFPropertyListFormat; var errorString: CFStringRef ): CFIndex; external name '_CFPropertyListWriteToStream';
>>>>>>> origin/fixes_2_2
=======
function CFPropertyListIsValid( plist: CFPropertyListRef; format: CFPropertyListFormat ): Boolean; external name '_CFPropertyListIsValid';
>>>>>>> origin/fixes_2.4

{ Writes the bytes of a plist serialization out to the stream.  The
 * stream must be opened and configured -- the function simply writes
 * a bunch of bytes to the stream. The output plist format can be chosen.
 * Leaves the stream open, but note that reading a plist expects the
 * reading stream to end wherever the writing ended, so that the
 * end of the plist data can be identified. Returns the number of bytes
 * written, or 0 on error. Error messages are not currently localized, but
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 * may be in the future, so they are not suitable for comparison. 
 *
 * This function is obsolete and will be deprecated soon. See CFPropertyListWrite() for a replacement. }
function CFPropertyListWriteToStream( propertyList: CFPropertyListRef; stream: CFWriteStreamRef; format: CFPropertyListFormat; var errorString: CFStringRef ): CFIndex; external name '_CFPropertyListWriteToStream';

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
 * may be in the future, so they are not suitable for comparison.
 *
 * This function is obsolete and will be deprecated soon. See CFPropertyListWrite() for a replacement. }
function CFPropertyListWriteToStream( propertyList: CFPropertyListRef; stream: CFWriteStreamRef; format: CFPropertyListFormat; var errorString: CFStringRef ): CFIndex; external name '_CFPropertyListWriteToStream';
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
 * may be in the future, so they are not suitable for comparison. }

function CFPropertyListCreateFromStream( allocator: CFAllocatorRef; stream: CFReadStreamRef; streamLength: CFIndex; mutabilityOption: CFOptionFlags; var format: CFPropertyListFormat; var errorString: CFStringRef ): CFPropertyListRef; external name '_CFPropertyListCreateFromStream';
>>>>>>> graemeg/fixes_2_2
=======
 * may be in the future, so they are not suitable for comparison. }

function CFPropertyListCreateFromStream( allocator: CFAllocatorRef; stream: CFReadStreamRef; streamLength: CFIndex; mutabilityOption: CFOptionFlags; var format: CFPropertyListFormat; var errorString: CFStringRef ): CFPropertyListRef; external name '_CFPropertyListCreateFromStream';
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{ Same as current function CFPropertyListCreateFromXMLData()
 * but takes a stream instead of data, and works on any plist file format.
 * CFPropertyListCreateFromXMLData() also works on any plist file format.
 * The stream must be open and configured -- the function simply reads a bunch
 * of bytes from it starting at the current location in the stream, to the END
 * of the stream, which is expected to be the end of the plist, or up to the
 * number of bytes given by the length parameter if it is not 0. Error messages
 * are not currently localized, but may be in the future, so they are not
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 * suitable for comparison. 
=======
 * suitable for comparison.
>>>>>>> graemeg/cpstrnew
=======
 * suitable for comparison.
>>>>>>> graemeg/cpstrnew
 *
 * This function is obsolete and will be deprecated soon. See CFPropertyListCreateWithStream() for a replacement. }
function CFPropertyListCreateFromStream( allocator: CFAllocatorRef; stream: CFReadStreamRef; streamLength: CFIndex; mutabilityOption: CFOptionFlags; var format: CFPropertyListFormat; var errorString: CFStringRef ): CFPropertyListRef; external name '_CFPropertyListCreateFromStream';

{#endif}
=======
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
 * suitable for comparison.
 *
 * This function is obsolete and will be deprecated soon. See CFPropertyListCreateWithStream() for a replacement. }
function CFPropertyListCreateFromStream( allocator: CFAllocatorRef; stream: CFReadStreamRef; streamLength: CFIndex; mutabilityOption: CFOptionFlags; var format: CFPropertyListFormat; var errorString: CFStringRef ): CFPropertyListRef; external name '_CFPropertyListCreateFromStream';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew

{#if MAC_OS_X_VERSION_10_6 <= MAC_OS_X_VERSION_MAX_ALLOWED}
const
	kCFPropertyListReadCorruptError = 3840; (* CF_ENUM_AVAILABLE(10_6, 4_0) *)             // Error parsing a property list
	kCFPropertyListReadUnknownVersionError = 3841; (* CF_ENUM_AVAILABLE(10_6, 4_0) *)      // The version number in the property list is unknown
	kCFPropertyListReadStreamError = 3842; (* CF_ENUM_AVAILABLE(10_6, 4_0) *)              // Stream error reading a property list
	kCFPropertyListWriteStreamError = 3851; (* CF_ENUM_AVAILABLE(10_6, 4_0) *)             // Stream error writing a property list
{#endif}

<<<<<<< HEAD
{ Create a property list with a CFData input. If the format parameter is non-NULL, it will be set to the format of the data after parsing is complete. The options parameter is used to specify CFPropertyListMutabilityOptions. If an error occurs while parsing the data, the return value will be NULL. Additionally, if an error occurs and the error parameter is non-NULL, the error parameter will be set to a CFError describing the problem, which the caller must release. If the parse succeeds, the returned value is a reference to the new property list. It is the responsibility of the caller to release this value.
 }
function CFPropertyListCreateWithData( allocator: CFAllocatorRef; data: CFDataRef; options: CFOptionFlags; format: CFPropertyListFormatPtr { can be NULL };  error: CFErrorRefPtr { can be NULL } ): CFPropertyListRef; external name '_CFPropertyListCreateWithData';
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
=======
>>>>>>> origin/cpstrnew

{ Apple sets TARGET_OS_MAC to true for TARGET_IPHONE_SIMULATOR, but since we use unified
  headers for Mac OS X and iPhoneSimulator, that would result in many problems -> specify
  it explicitly }
{$ifc TARGET_OS_MAC or TARGET_OS_WIN32 or TARGET_OS_EMBEDDED or TARGET_IPHONE_SIMULATOR}
{ Create and return a property list with a CFReadStream input. TIf the format parameter is non-NULL, it will be set to the format of the data after parsing is complete. The options parameter is used to specify CFPropertyListMutabilityOptions. The streamLength parameter specifies the number of bytes to read from the stream. Set streamLength to 0 to read until the end of the stream is detected. If an error occurs while parsing the data, the return value will be NULL. Additionally, if an error occurs and the error parameter is non-NULL, the error parameter will be set to a CFError describing the problem, which the caller must release. If the parse succeeds, the returned value is a reference to the new property list. It is the responsibility of the caller to release this value.
 }
function CFPropertyListCreateWithStream( allocator: CFAllocatorRef; stream: CFReadStreamRef; streamLength: CFIndex; options: CFOptionFlags;  format: CFPropertyListFormatPtr { can be NULL }; error: CFErrorRefPtr { can be NULL } ): CFPropertyListRef; external name '_CFPropertyListCreateWithStream';
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
=======
{#if MAC_OS_X_VERSION_10_6 <= MAC_OS_X_VERSION_MAX_ALLOWED}
const
	kCFPropertyListReadCorruptError = 3840; (* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)             // Error parsing a property list
	kCFPropertyListReadUnknownVersionError = 3841; (* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)      // The version number in the property list is unknown
	kCFPropertyListReadStreamError = 3842; (* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)              // Stream error reading a property list
	kCFPropertyListWriteStreamError = 3851; (* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)             // Stream error writing a property list
{#endif}

{ Create a property list with a CFData input. If the format parameter is non-NULL, it will be set to the format of the data after parsing is complete. The options parameter is used to specify CFPropertyListMutabilityOptions. If an error occurs while parsing the data, the return value will be NULL. Additionally, if an error occurs and the error parameter is non-NULL, the error parameter will be set to a CFError describing the problem, which the caller must release. If the parse succeeds, the returned value is a reference to the new property list. It is the responsibility of the caller to release this value.
 }
function CFPropertyListCreateWithData( allocator: CFAllocatorRef; data: CFDataRef; options: CFOptionFlags; format: CFPropertyListFormatPtr { can be NULL };  error: CFErrorRefPtr { can be NULL } ): CFPropertyListRef; external name '_CFPropertyListCreateWithData';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{ Create and return a property list with a CFReadStream input. TIf the format parameter is non-NULL, it will be set to the format of the data after parsing is complete. The options parameter is used to specify CFPropertyListMutabilityOptions. The streamLength parameter specifies the number of bytes to read from the stream. Set streamLength to 0 to read until the end of the stream is detected. If an error occurs while parsing the data, the return value will be NULL. Additionally, if an error occurs and the error parameter is non-NULL, the error parameter will be set to a CFError describing the problem, which the caller must release. If the parse succeeds, the returned value is a reference to the new property list. It is the responsibility of the caller to release this value.
 }
function CFPropertyListCreateWithStream( allocator: CFAllocatorRef; stream: CFReadStreamRef; streamLength: CFIndex; options: CFOptionFlags;  format: CFPropertyListFormatPtr { can be NULL }; error: CFErrorRefPtr { can be NULL } ): CFPropertyListRef; external name '_CFPropertyListCreateWithStream';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)
>>>>>>> graemeg/cpstrnew

{ Write the bytes of a serialized property list out to a stream. The stream must be opened and configured. The format of the property list can be chosen with the format parameter. The options parameter is currently unused and should be set to 0. The return value is the number of bytes written or 0 in the case of an error. If an error occurs and the error parameter is non-NULL, the error parameter will be set to a CFError describing the problem, which the caller must release.
 }
function CFPropertyListWrite( propertyList: CFPropertyListRef; stream: CFWriteStreamRef; format: CFPropertyListFormat; options: CFOptionFlags; error: CFErrorRefPtr { can be NULL } ): CFIndex; external name '_CFPropertyListWrite';
<<<<<<< HEAD
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)

{$endc} {TARGET_OS_MAC or TARGET_OS_WIN32 or TARGET_OS_EMBEDDED or TARGET_IPHONE_SIMULATOR}
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)
>>>>>>> graemeg/cpstrnew

{ Create a CFData with the bytes of a serialized property list. The format of the property list can be chosen with the format parameter. The options parameter is currently unused and should be set to 0. If an error occurs while parsing the data, the return value will be NULL. Additionally, if an error occurs and the error parameter is non-NULL, the error parameter will be set to a CFError describing the problem, which the caller must release. If the conversion succeeds, the returned value is a reference to the created data. It is the responsibility of the caller to release this value.
 }
function CFPropertyListCreateData( allocator: CFAllocatorRef; propertyList: CFPropertyListRef; format: CFPropertyListFormat; options: CFOptionFlags; error: CFErrorRefPtr { can be NULL } ): CFDataRef; external name '_CFPropertyListCreateData';
<<<<<<< HEAD
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)
>>>>>>> graemeg/cpstrnew


<<<<<<< HEAD
=======
{#if MAC_OS_X_VERSION_10_6 <= MAC_OS_X_VERSION_MAX_ALLOWED}
const
	kCFPropertyListReadCorruptError = 3840; (* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)             // Error parsing a property list
	kCFPropertyListReadUnknownVersionError = 3841; (* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)      // The version number in the property list is unknown
	kCFPropertyListReadStreamError = 3842; (* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)              // Stream error reading a property list
	kCFPropertyListWriteStreamError = 3851; (* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)             // Stream error writing a property list
{#endif}

<<<<<<< HEAD
{ Create a property list with a CFData input. If the format parameter is non-NULL, it will be set to the format of the data after parsing is complete. The options parameter is used to specify CFPropertyListMutabilityOptions. If an error occurs while parsing the data, the return value will be NULL. Additionally, if an error occurs and the error parameter is non-NULL, the error parameter will be set to a CFError describing the problem, which the caller must release. If the parse succeeds, the returned value is a reference to the new property list. It is the responsibility of the caller to release this value.
 }
function CFPropertyListCreateWithData( allocator: CFAllocatorRef; data: CFDataRef; options: CFOptionFlags; format: CFPropertyListFormatPtr { can be NULL };  error: CFErrorRefPtr { can be NULL } ): CFPropertyListRef; external name '_CFPropertyListCreateWithData';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{ Create and return a property list with a CFReadStream input. TIf the format parameter is non-NULL, it will be set to the format of the data after parsing is complete. The options parameter is used to specify CFPropertyListMutabilityOptions. The streamLength parameter specifies the number of bytes to read from the stream. Set streamLength to 0 to read until the end of the stream is detected. If an error occurs while parsing the data, the return value will be NULL. Additionally, if an error occurs and the error parameter is non-NULL, the error parameter will be set to a CFError describing the problem, which the caller must release. If the parse succeeds, the returned value is a reference to the new property list. It is the responsibility of the caller to release this value.
 }
function CFPropertyListCreateWithStream( allocator: CFAllocatorRef; stream: CFReadStreamRef; streamLength: CFIndex; options: CFOptionFlags;  format: CFPropertyListFormatPtr { can be NULL }; error: CFErrorRefPtr { can be NULL } ): CFPropertyListRef; external name '_CFPropertyListCreateWithStream';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{ Write the bytes of a serialized property list out to a stream. The stream must be opened and configured. The format of the property list can be chosen with the format parameter. The options parameter is currently unused and should be set to 0. The return value is the number of bytes written or 0 in the case of an error. If an error occurs and the error parameter is non-NULL, the error parameter will be set to a CFError describing the problem, which the caller must release.
 }
function CFPropertyListWrite( propertyList: CFPropertyListRef; stream: CFWriteStreamRef; format: CFPropertyListFormat; options: CFOptionFlags; error: CFErrorRefPtr { can be NULL } ): CFIndex; external name '_CFPropertyListWrite';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{ Create a CFData with the bytes of a serialized property list. The format of the property list can be chosen with the format parameter. The options parameter is currently unused and should be set to 0. If an error occurs while parsing the data, the return value will be NULL. Additionally, if an error occurs and the error parameter is non-NULL, the error parameter will be set to a CFError describing the problem, which the caller must release. If the conversion succeeds, the returned value is a reference to the created data. It is the responsibility of the caller to release this value.
 }
function CFPropertyListCreateData( allocator: CFAllocatorRef; propertyList: CFPropertyListRef; format: CFPropertyListFormat; options: CFOptionFlags; error: CFErrorRefPtr { can be NULL } ): CFDataRef; external name '_CFPropertyListCreateData';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)


<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/cpstrnew
{#if MAC_OS_X_VERSION_10_6 <= MAC_OS_X_VERSION_MAX_ALLOWED}
const
	kCFPropertyListReadCorruptError = 3840; (* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)             // Error parsing a property list
	kCFPropertyListReadUnknownVersionError = 3841; (* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)      // The version number in the property list is unknown
	kCFPropertyListReadStreamError = 3842; (* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)              // Stream error reading a property list
	kCFPropertyListWriteStreamError = 3851; (* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)             // Stream error writing a property list
{#endif}

{ Create a property list with a CFData input. If the format parameter is non-NULL, it will be set to the format of the data after parsing is complete. The options parameter is used to specify CFPropertyListMutabilityOptions. If an error occurs while parsing the data, the return value will be NULL. Additionally, if an error occurs and the error parameter is non-NULL, the error parameter will be set to a CFError describing the problem, which the caller must release. If the parse succeeds, the returned value is a reference to the new property list. It is the responsibility of the caller to release this value.
 }
function CFPropertyListCreateWithData( allocator: CFAllocatorRef; data: CFDataRef; options: CFOptionFlags; format: CFPropertyListFormatPtr { can be NULL };  error: CFErrorRefPtr { can be NULL } ): CFPropertyListRef; external name '_CFPropertyListCreateWithData';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{ Create and return a property list with a CFReadStream input. TIf the format parameter is non-NULL, it will be set to the format of the data after parsing is complete. The options parameter is used to specify CFPropertyListMutabilityOptions. The streamLength parameter specifies the number of bytes to read from the stream. Set streamLength to 0 to read until the end of the stream is detected. If an error occurs while parsing the data, the return value will be NULL. Additionally, if an error occurs and the error parameter is non-NULL, the error parameter will be set to a CFError describing the problem, which the caller must release. If the parse succeeds, the returned value is a reference to the new property list. It is the responsibility of the caller to release this value.
 }
function CFPropertyListCreateWithStream( allocator: CFAllocatorRef; stream: CFReadStreamRef; streamLength: CFIndex; options: CFOptionFlags;  format: CFPropertyListFormatPtr { can be NULL }; error: CFErrorRefPtr { can be NULL } ): CFPropertyListRef; external name '_CFPropertyListCreateWithStream';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{ Write the bytes of a serialized property list out to a stream. The stream must be opened and configured. The format of the property list can be chosen with the format parameter. The options parameter is currently unused and should be set to 0. The return value is the number of bytes written or 0 in the case of an error. If an error occurs and the error parameter is non-NULL, the error parameter will be set to a CFError describing the problem, which the caller must release.
 }
function CFPropertyListWrite( propertyList: CFPropertyListRef; stream: CFWriteStreamRef; format: CFPropertyListFormat; options: CFOptionFlags; error: CFErrorRefPtr { can be NULL } ): CFIndex; external name '_CFPropertyListWrite';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{ Create a CFData with the bytes of a serialized property list. The format of the property list can be chosen with the format parameter. The options parameter is currently unused and should be set to 0. If an error occurs while parsing the data, the return value will be NULL. Additionally, if an error occurs and the error parameter is non-NULL, the error parameter will be set to a CFError describing the problem, which the caller must release. If the conversion succeeds, the returned value is a reference to the created data. It is the responsibility of the caller to release this value.
 }
function CFPropertyListCreateData( allocator: CFAllocatorRef; propertyList: CFPropertyListRef; format: CFPropertyListFormat; options: CFOptionFlags; error: CFErrorRefPtr { can be NULL } ): CFDataRef; external name '_CFPropertyListCreateData';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)


<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
 * suitable for comparison. }
=======
>>>>>>> origin/fixes_2.4

{#endif}

{#if MAC_OS_X_VERSION_10_6 <= MAC_OS_X_VERSION_MAX_ALLOWED}
const
	kCFPropertyListReadCorruptError = 3840; (* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)             // Error parsing a property list
	kCFPropertyListReadUnknownVersionError = 3841; (* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)      // The version number in the property list is unknown
	kCFPropertyListReadStreamError = 3842; (* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)              // Stream error reading a property list
	kCFPropertyListWriteStreamError = 3851; (* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)             // Stream error writing a property list
{#endif}

{ Create a property list with a CFData input. If the format parameter is non-NULL, it will be set to the format of the data after parsing is complete. The options parameter is used to specify CFPropertyListMutabilityOptions. If an error occurs while parsing the data, the return value will be NULL. Additionally, if an error occurs and the error parameter is non-NULL, the error parameter will be set to a CFError describing the problem, which the caller must release. If the parse succeeds, the returned value is a reference to the new property list. It is the responsibility of the caller to release this value.
 }
function CFPropertyListCreateWithData( allocator: CFAllocatorRef; data: CFDataRef; options: CFOptionFlags; format: CFPropertyListFormatPtr { can be NULL };  error: CFErrorRefPtr { can be NULL } ): CFPropertyListRef; external name '_CFPropertyListCreateWithData';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{ Create and return a property list with a CFReadStream input. TIf the format parameter is non-NULL, it will be set to the format of the data after parsing is complete. The options parameter is used to specify CFPropertyListMutabilityOptions. The streamLength parameter specifies the number of bytes to read from the stream. Set streamLength to 0 to read until the end of the stream is detected. If an error occurs while parsing the data, the return value will be NULL. Additionally, if an error occurs and the error parameter is non-NULL, the error parameter will be set to a CFError describing the problem, which the caller must release. If the parse succeeds, the returned value is a reference to the new property list. It is the responsibility of the caller to release this value.
 }
function CFPropertyListCreateWithStream( allocator: CFAllocatorRef; stream: CFReadStreamRef; streamLength: CFIndex; options: CFOptionFlags;  format: CFPropertyListFormatPtr { can be NULL }; error: CFErrorRefPtr { can be NULL } ): CFPropertyListRef; external name '_CFPropertyListCreateWithStream';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{ Write the bytes of a serialized property list out to a stream. The stream must be opened and configured. The format of the property list can be chosen with the format parameter. The options parameter is currently unused and should be set to 0. The return value is the number of bytes written or 0 in the case of an error. If an error occurs and the error parameter is non-NULL, the error parameter will be set to a CFError describing the problem, which the caller must release.
 }
function CFPropertyListWrite( propertyList: CFPropertyListRef; stream: CFWriteStreamRef; format: CFPropertyListFormat; options: CFOptionFlags; error: CFErrorRefPtr { can be NULL } ): CFIndex; external name '_CFPropertyListWrite';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{ Create a CFData with the bytes of a serialized property list. The format of the property list can be chosen with the format parameter. The options parameter is currently unused and should be set to 0. If an error occurs while parsing the data, the return value will be NULL. Additionally, if an error occurs and the error parameter is non-NULL, the error parameter will be set to a CFError describing the problem, which the caller must release. If the conversion succeeds, the returned value is a reference to the created data. It is the responsibility of the caller to release this value.
 }
function CFPropertyListCreateData( allocator: CFAllocatorRef; propertyList: CFPropertyListRef; format: CFPropertyListFormat; options: CFOptionFlags; error: CFErrorRefPtr { can be NULL } ): CFDataRef; external name '_CFPropertyListCreateData';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)


{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4
