{	CFString.h
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
{	  Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, November 2005 }
{	  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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
{	  Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, November 2005 }
{	  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit CFString;
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
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
=======
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
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
	{$setc TARGET_OS_EMBEDDED := FALSE}
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
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
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
=======
>>>>>>> graemeg/cpstrnew
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
>>>>>>> graemeg/cpstrnew
=======
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
>>>>>>> graemeg/cpstrnew
=======
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
uses MacTypes,CFBase,CFArray,CFData,CFDictionary,CFCharacterSet,CFLocale;
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


{
Please note: CFStrings are conceptually an array of Unicode characters.
However, in general, how a CFString stores this array is an implementation
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
detail. For instance, CFString might choose to use an array of 8-bit characters
to store its contents, or it might use multiple blocks of memory, or whatever.
This is especially true since CFString is toll-free bridged with NSString, enabling
any NSString instance to be used as a CFString. Furthermore, the implementation
may change depending on the default system encoding, the user's language, 
or even a release or update of the OS.
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
detail. For instance, CFString might choose to use an array of 8-bit characters;
to store its contents; or it might use multiple blocks of memory; or whatever.
Furthermore, the implementation might change depending on the default
system encoding, the user's language, the OS, or even a given release.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

What this means is that you should use the following advanced functions with care:

  CFStringGetPascalStringPtr()
  CFStringGetCStringPtr()
  CFStringGetCharactersPtr()

These functions are provided for optimization only. They will either return the desired
pointer quickly, in constant time, or they return NULL. They might choose to return NULL
for many reasons; for instance it's possible that for users running in different
languages these sometimes return NULL; or in a future OS release the first two might
switch to always returning NULL. Never observing NULL returns in your usages of these
functions does not mean they won't ever return NULL. (But note the CFStringGetCharactersPtr()
exception mentioned further below.)

In your usages of these functions, if you get a NULL return, use the non-Ptr version
of the functions as shown in this example:

  Str255 buffer;
  StringPtr ptr = CFStringGetPascalStringPtr(str, encoding);
  if (ptr == NULL) (
      if (CFStringGetPascalString(str, buffer, 256, encoding)) ptr = buffer;
  )

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Note that CFStringGetCString() or CFStringGetPascalString() calls might still fail --- but
=======
Note that CFStringGetPascalString() or CFStringGetCString() calls might still fail --- but
>>>>>>> graemeg/fixes_2_2
=======
Note that CFStringGetPascalString() or CFStringGetCString() calls might still fail --- but
>>>>>>> origin/fixes_2_2
=======
Note that CFStringGetCString() or CFStringGetPascalString() calls might still fail --- but
>>>>>>> origin/fixes_2.4
that will happen in two circumstances only: The conversion from the UniChar contents of CFString
to the specified encoding fails, or the buffer is too small. If they fail, that means
the conversion was not possible.

<<<<<<< HEAD
<<<<<<< HEAD
If you need a copy of the buffer in the above example, you might consider simply calling
CFStringGetCString() in all cases --- CFStringGetCStringPtr() is simply an optimization.
=======
If you need a copy of the buffer in the above example, you might consider simply
calling CFStringGetPascalString() in all cases --- CFStringGetPascalStringPtr()
is simply an optimization.
>>>>>>> graemeg/fixes_2_2
=======
If you need a copy of the buffer in the above example, you might consider simply
calling CFStringGetPascalString() in all cases --- CFStringGetPascalStringPtr()
is simply an optimization.
>>>>>>> origin/fixes_2_2

In addition, the following functions, which create immutable CFStrings from developer
supplied buffers without copying the buffers, might have to actually copy
under certain circumstances (If they do copy, the buffer will be dealt with by the
"contentsDeallocator" argument.):

  CFStringCreateWithPascalStringNoCopy()
  CFStringCreateWithCStringNoCopy()
  CFStringCreateWithCharactersNoCopy()

You should of course never depend on the backing store of these CFStrings being
what you provided, and in other no circumstance should you change the contents
of that buffer (given that would break the invariant about the CFString being immutable).

Having said all this, there are actually ways to create a CFString where the backing store
is external, and can be manipulated by the developer or CFString itself:

  CFStringCreateMutableWithExternalCharactersNoCopy()
  CFStringSetExternalCharactersNoCopy()

A "contentsAllocator" is used to realloc or free the backing store by CFString.
kCFAllocatorNull can be provided to assure CFString will never realloc or free the buffer.
Developer can call CFStringSetExternalCharactersNoCopy() to update
CFString's idea of what's going on, if the buffer is changed externally. In these
strings, CFStringGetCharactersPtr() is guaranteed to return the external buffer.

These functions are here to allow wrapping a buffer of UniChar characters in a CFString,
allowing the buffer to passed into CFString functions and also manipulated via CFString
mutation functions. In general, developers should not use this technique for all strings,
as it prevents CFString from using certain optimizations.
}

{ Identifier for character encoding; the values are the same as Text Encoding Converter TextEncoding.
}
type
	CFStringEncoding = UInt32;
	CFStringEncodingPtr = ^CFStringEncoding;

{ Platform-independent built-in encodings; always available on all platforms.
   Call CFStringGetSystemEncoding() to get the default system encoding.
}
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CFStringBuiltInEncodings = CFStringEncoding;
=======
	CFStringBuiltInEncodings = UInt32;
>>>>>>> graemeg/cpstrnew
=======
	CFStringBuiltInEncodings = UInt32;
>>>>>>> graemeg/cpstrnew
=======
	CFStringBuiltInEncodings = UInt32;
>>>>>>> graemeg/cpstrnew
=======
	CFStringBuiltInEncodings = UInt32;
>>>>>>> origin/cpstrnew
=======
	CFStringBuiltInEncodings = SInt32;
>>>>>>> graemeg/fixes_2_2
=======
	CFStringBuiltInEncodings = SInt32;
>>>>>>> origin/fixes_2_2
=======
	CFStringBuiltInEncodings = UInt32;
>>>>>>> origin/fixes_2.4
const
	kCFStringEncodingInvalidId = $FFFFFFFF;
	kCFStringEncodingMacRoman = 0;
	kCFStringEncodingWindowsLatin1 = $0500; {  ANSI codepage 1252  }
	kCFStringEncodingISOLatin1 = $0201; {  ISO 8859-1  }
	kCFStringEncodingNextStepLatin = $0B01; {  NextStep encoding }
	kCFStringEncodingASCII = $0600; {  0..127 (in creating CFString, values greater than 0x7F are treated as corresponding Unicode value)  }
	kCFStringEncodingUnicode = $0100; {  kTextEncodingUnicodeDefault  + kTextEncodingDefaultFormat (aka kUnicode16BitFormat)  }
	kCFStringEncodingUTF8 = $08000100; {  kTextEncodingUnicodeDefault + kUnicodeUTF8Format  }
	kCFStringEncodingNonLossyASCII = $0BFF; {  7bit Unicode variants used by YellowBox & Java  }
{#if MAC_OS_X_VERSION_MAX_ALLOWED >= MAC_OS_X_VERSION_10_4}
	kCFStringEncodingUTF16 = $0100; { kTextEncodingUnicodeDefault + kUnicodeUTF16Format (alias of kCFStringEncodingUnicode) }
	kCFStringEncodingUTF16BE = $10000100; { kTextEncodingUnicodeDefault + kUnicodeUTF16BEFormat }
	kCFStringEncodingUTF16LE = $14000100; { kTextEncodingUnicodeDefault + kUnicodeUTF16LEFormat }

	kCFStringEncodingUTF32 = $0C000100; { kTextEncodingUnicodeDefault + kUnicodeUTF32Format }
	kCFStringEncodingUTF32BE = $18000100; { kTextEncodingUnicodeDefault + kUnicodeUTF32BEFormat }
	kCFStringEncodingUTF32LE = $1C000100; { kTextEncodingUnicodeDefault + kUnicodeUTF32LEFormat }
{#endif} { MAC_OS_X_VERSION_MAX_ALLOWED >= MAC_OS_X_VERSION_10_4 }

{ CFString type ID }
function CFStringGetTypeID: CFTypeID; external name '_CFStringGetTypeID';

<<<<<<< HEAD
<<<<<<< HEAD
{ CFSTR() allows creation of compile-time constant CFStringRefs; the argument 
should be a constant C-string.
=======
{ Macro to allow creation of compile-time constant strings; the argument should be a constant string.
>>>>>>> graemeg/fixes_2_2
=======
{ Macro to allow creation of compile-time constant strings; the argument should be a constant string.
>>>>>>> origin/fixes_2_2

CFSTR(), not being a "Copy" or "Create" function, does not return a new
reference for you. So, you should not release the return value. This is
much like constant C or Pascal strings --- when you use "hello world"
in a program, you do not free it.

However, strings returned from CFSTR() can be retained and released in a
properly nested fashion, just like any other CF type. That is, if you pass
a CFSTR() return value to a function such as SetMenuItemWithCFString(), the
function can retain it, then later, when it's done with it, it can release it.

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
#if MAC_OS_X_VERSION_MIN_REQUIRED <= MAC_OS_X_VERSION_10_5
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
#if MAC_OS_X_VERSION_MIN_REQUIRED <= MAC_OS_X_VERSION_10_5
>>>>>>> origin/fixes_2.4
At this point non-7 bit characters (that is, characters > 127) in CFSTR() are not 
supported and using them will lead to unpredictable results. This includes escaped
(\nnn) characters whose values are > 127. Even if it works for you in testing, 
it might not work for a user with a different language preference.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
#else
Non-7 bit characters (that is, above 127) in CFSTR() are supported, although care must
be taken in dealing with files containing them. If you can trust your editor and tools
to deal with non-ASCII characters in the source code, then you can use them directly 
in CFSTR(); otherwise, you can represent such characters with their escaped octal 
equivalents in the encoding the compiler will use to interpret them (for instance, 
O-umlaut is \303\226 in UTF-8). UTF-8 is the recommended encoding here, 
since it is the default choice with Mac OS X developer tools.
#endif

<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
}
{
	*** Pascal Usage of CFSTR et al ***
	
	For Pascal:
	
		To define your own constant CFStrings, call call CFSTRP with a 
		constant Pascal single-quoted string only, for example 
		CFSTRP('a constant string').

	For MetroWerks Pascal:
	
		Include the CFStringGlue file in your project (it defines the private function,
		CFSTRP0, which you should never call directly.
		
		In your uses clause, use CFString and CFStringGlue and whichever PInterface files
		defining the constant CFStrings (or define your own using CFSTRP('pascal string')).
	
	For GNU Pascal:
	
		Include the GPCMacros.inc file (for example $I GPCMacros.inc).
		
		In your uses clause, use CFString (and, optionally, CFStringGlue and the 
		PInterface files defining the constant CFStrings).
		
		You can optionally use CFSTR with a constant double-quoted C String (for example,
		CFSTR("a constant C string")) although it will make no difference, but will
		result in closer  C source code similarity.
	
	For maximum compatibility in both GPC and MetroWerks Pascal
	
		Include the GPCMacros.inc file in GPC as part of your GPC prefix.
		
		Include the CFStringGlue file in your MW project.
		
		Only ever use CFSTRP with a constant Pascal string.
		
		In your uses clause for a unit, use the CFString, CFStringGlue and any 
		PInterface files defining any constant CFStrings you need.
}

{
	CFSTR Must only be called with a constant CString
}
function CFSTR( c: PChar ): CFStringRef; external name '___CFStringMakeConstantString';
{
	CFSTRP Must only be called with a constant Pascal String
}
function CFSTRP( c: PChar ): CFStringRef; external name '___CFStringMakeConstantString';

{
	CFSTRP0 Must only be called with a constant Pascal String terminated with a chr(0)
}
function CFSTRP0( c: PChar ): CFStringRef; external name '___CFStringMakeConstantString';

{** Immutable string creation functions **}

{ Functions to create basic immutable strings. The provided allocator is used for all memory activity in these functions.
}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ The following four functions copy the provided buffer into CFString's internal storage. }
=======
{ These functions copy the provided buffer into CFString's internal storage. }
>>>>>>> graemeg/fixes_2_2
=======
{ These functions copy the provided buffer into CFString's internal storage. }
>>>>>>> origin/fixes_2_2
=======
{ The following four functions copy the provided buffer into CFString's internal storage. }
>>>>>>> origin/fixes_2.4
function CFStringCreateWithPascalString( alloc: CFAllocatorRef; const (*var*) pStr: Str255; encoding: CFStringEncoding ): CFStringRef; external name '_CFStringCreateWithPascalString';

function CFStringCreateWithCString( alloc: CFAllocatorRef; cStr: ConstCStringPtr; encoding: CFStringEncoding ): CFStringRef; external name '_CFStringCreateWithCString';

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{ The following takes an explicit length, and allows you to specify whether the data is an external format --- that is, whether to pay attention to the BOM character (if any) and do byte swapping if necessary
}
function CFStringCreateWithBytes( alloc: CFAllocatorRef; bytes: UnivPtr; numBytes: CFIndex; encoding: CFStringEncoding; isExternalRepresentation: Boolean ): CFStringRef; external name '_CFStringCreateWithBytes';

<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
function CFStringCreateWithCharacters( alloc: CFAllocatorRef; chars: UniCharPtr; numChars: CFIndex ): CFStringRef; external name '_CFStringCreateWithCharacters';

{ These functions try not to copy the provided buffer. The buffer will be deallocated 
with the provided contentsDeallocator when it's no longer needed; to not free
the buffer, specify kCFAllocatorNull here. As usual, NULL means default allocator.

NOTE: Do not count on these buffers as being used by the string; 
in some cases the CFString might free the buffer and use something else
(for instance if it decides to always use Unicode encoding internally). 

NOTE: If you are not transferring ownership of the buffer to the CFString
(for instance, you supplied contentsDeallocator = kCFAllocatorNull), it is your
responsibility to assure the buffer does not go away during the lifetime of the string.
If the string is retained or copied, its lifetime might extend in ways you cannot
predict. So, for strings created with buffers whose lifetimes you cannot
guarantee, you need to be extremely careful --- do not hand it out to any
APIs which might retain or copy the strings.
}
function CFStringCreateWithPascalStringNoCopy( alloc: CFAllocatorRef; const (*var*) pStr: Str255; encoding: CFStringEncoding; contentsDeallocator: CFAllocatorRef ): CFStringRef; external name '_CFStringCreateWithPascalStringNoCopy';

function CFStringCreateWithCStringNoCopy( alloc: CFAllocatorRef; cStr: ConstCStringPtr; encoding: CFStringEncoding; contentsDeallocator: CFAllocatorRef ): CFStringRef; external name '_CFStringCreateWithCStringNoCopy';

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ The following takes an explicit length, and allows you to specify whether the data is an external format --- that is, whether to pay attention to the BOM character (if any) and do byte swapping if necessary
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function CFStringCreateWithBytesNoCopy( alloc: CFAllocatorRef; bytes: UnivPtr; numBytes: CFIndex; encoding: CFStringEncoding; isExternalRepresentation: Boolean; contentsDeallocator: CFAllocatorRef ): CFStringRef; external name '_CFStringCreateWithBytesNoCopy';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
=======
function CFStringCreateWithBytesNoCopy( alloc: CFAllocatorRef; bytes: UnivPtr; numBytes: CFIndex; encoding: CFStringEncoding; isExternalRepresentation: Boolean; contentsDeallocator: CFAllocatorRef ): CFStringRef; external name '_CFStringCreateWithBytesNoCopy'; (* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
function CFStringCreateWithBytesNoCopy( alloc: CFAllocatorRef; bytes: UnivPtr; numBytes: CFIndex; encoding: CFStringEncoding; isExternalRepresentation: Boolean; contentsDeallocator: CFAllocatorRef ): CFStringRef; external name '_CFStringCreateWithBytesNoCopy'; (* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
function CFStringCreateWithBytesNoCopy( alloc: CFAllocatorRef; bytes: UnivPtr; numBytes: CFIndex; encoding: CFStringEncoding; isExternalRepresentation: Boolean; contentsDeallocator: CFAllocatorRef ): CFStringRef; external name '_CFStringCreateWithBytesNoCopy'; (* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
function CFStringCreateWithBytesNoCopy( alloc: CFAllocatorRef; bytes: UnivPtr; numBytes: CFIndex; encoding: CFStringEncoding; isExternalRepresentation: Boolean; contentsDeallocator: CFAllocatorRef ): CFStringRef; external name '_CFStringCreateWithBytesNoCopy'; (* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ The following takes an explicit length, and allows you to specify whether the data is an external format --- that is, whether to pay attention to the BOM character (if any) and do byte swapping if necessary
}
function CFStringCreateWithBytesNoCopy( alloc: CFAllocatorRef; bytes: UnivPtr; numBytes: CFIndex; encoding: CFStringEncoding; isExternalRepresentation: Boolean; contentsDeallocator: CFAllocatorRef ): CFStringRef; external name '_CFStringCreateWithBytesNoCopy'; (* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

>>>>>>> origin/fixes_2.4
function CFStringCreateWithCharactersNoCopy( alloc: CFAllocatorRef; chars: UniCharPtr; numChars: CFIndex; contentsDeallocator: CFAllocatorRef ): CFStringRef; external name '_CFStringCreateWithCharactersNoCopy';

{ Create copies of part or all of the string.
}
function CFStringCreateWithSubstring( alloc: CFAllocatorRef; str: CFStringRef; range: CFRange ): CFStringRef; external name '_CFStringCreateWithSubstring';

function CFStringCreateCopy( alloc: CFAllocatorRef; theString: CFStringRef ): CFStringRef; external name '_CFStringCreateCopy';

{ These functions create a CFString from the provided printf-like format string and arguments.
}
function CFStringCreateWithFormat( alloc: CFAllocatorRef; formatOptions: CFDictionaryRef; format: CFStringRef; ... ): CFStringRef; external name '_CFStringCreateWithFormat';

function CFStringCreateWithFormatAndArguments( alloc: CFAllocatorRef; formatOptions: CFDictionaryRef; format: CFStringRef; arguments: UnivPtr ): CFStringRef; external name '_CFStringCreateWithFormatAndArguments';

{ Functions to create mutable strings. "maxLength", if not 0, is a hard bound on the length of the string. If 0, there is no limit on the length.
}
function CFStringCreateMutable( alloc: CFAllocatorRef; maxLength: CFIndex ): CFMutableStringRef; external name '_CFStringCreateMutable';

function CFStringCreateMutableCopy( alloc: CFAllocatorRef; maxLength: CFIndex; theString: CFStringRef ): CFMutableStringRef; external name '_CFStringCreateMutableCopy';

{ This function creates a mutable string that has a developer supplied and directly editable backing store.
The string will be manipulated within the provided buffer (if any) until it outgrows capacity; then the
externalCharactersAllocator will be consulted for more memory. When the CFString is deallocated, the
buffer will be freed with the externalCharactersAllocator. Provide kCFAllocatorNull here to prevent the buffer
from ever being reallocated or deallocated by CFString. See comments at top of this file for more info.
}
function CFStringCreateMutableWithExternalCharactersNoCopy( alloc: CFAllocatorRef; chars: UniCharPtr; numChars: CFIndex; capacity: CFIndex; externalCharactersAllocator: CFAllocatorRef ): CFMutableStringRef; external name '_CFStringCreateMutableWithExternalCharactersNoCopy';

{** Basic accessors for the contents **}

{ Number of 16-bit Unicode characters in the string.
}
function CFStringGetLength( theString: CFStringRef ): CFIndex; external name '_CFStringGetLength';

{ Extracting the contents of the string. For obtaining multiple characters, calling
CFStringGetCharacters() is more efficient than multiple calls to CFStringGetCharacterAtIndex().
If the length of the string is not known (so you can't use a fixed size buffer for CFStringGetCharacters()),
another method is to use is CFStringGetCharacterFromInlineBuffer() (see further below).
}
function CFStringGetCharacterAtIndex( theString: CFStringRef; idx: CFIndex ): UniChar; external name '_CFStringGetCharacterAtIndex';

procedure CFStringGetCharacters( theString: CFStringRef; range: CFRange; buffer: UniCharPtr ); external name '_CFStringGetCharacters';


{** Conversion to other encodings **}

{ These two convert into the provided buffer; they return false if conversion isn't possible
(due to conversion error, or not enough space in the provided buffer). 
These functions do zero-terminate or put the length byte; the provided bufferSize should include
space for this (so pass 256 for Str255). More sophisticated usages can go through CFStringGetBytes().
These functions are equivalent to calling CFStringGetBytes() with 
the range of the string; lossByte = 0; and isExternalRepresentation = false; 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
if successful, they then insert the leading length or terminating zero, as desired.
=======
if successful, they then insert the leading length of terminating zero, as desired.
>>>>>>> graemeg/fixes_2_2
=======
if successful, they then insert the leading length of terminating zero, as desired.
>>>>>>> origin/fixes_2_2
=======
if successful, they then insert the leading length or terminating zero, as desired.
>>>>>>> origin/fixes_2.4
}
function CFStringGetPascalString( theString: CFStringRef; buffer: StringPtr; bufferSize: CFIndex; encoding: CFStringEncoding ): Boolean; external name '_CFStringGetPascalString';

function CFStringGetCString( theString: CFStringRef; buffer: CStringPtr; bufferSize: CFIndex; encoding: CFStringEncoding ): Boolean; external name '_CFStringGetCString';

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{ These functions attempt to return in O(1) time the desired format for the string.
Note that although this means a pointer to the internal structure is being returned,
this can't always be counted on. Please see note at the top of the file for more
details.
}
function CFStringGetPascalStringPtr( theString: CFStringRef; encoding: CFStringEncoding ): ConstStringPtr; external name '_CFStringGetPascalStringPtr'; { May return NULL at any time; be prepared for NULL }

function CFStringGetCStringPtr( theString: CFStringRef; encoding: CFStringEncoding ): ConstCStringPtr; external name '_CFStringGetCStringPtr'; { May return NULL at any time; be prepared for NULL }

function CFStringGetCharactersPtr( theString: CFStringRef ): UniCharPtr; external name '_CFStringGetCharactersPtr'; { May return NULL at any time; be prepared for NULL }

{ The primitive conversion routine; allows you to convert a string piece at a time
<<<<<<< HEAD
<<<<<<< HEAD
       into a fixed size buffer. Returns number of characters converted. 
   Characters that cannot be converted to the specified encoding are represented
       with the byte specified by lossByte; if lossByte is 0, then lossy conversion
       is not allowed and conversion stops, returning partial results.
   Pass buffer==NULL if you don't care about the converted string (but just the convertability,
       or number of bytes required). 
   maxBufLength indicates the maximum number of bytes to generate. It is ignored when buffer==NULL.
   Does not zero-terminate. If you want to create Pascal or C string, allow one extra byte at start or end. 
   Setting isExternalRepresentation causes any extra bytes that would allow 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
       the data to be made persistent to be included; for instance, the Unicode BOM. Note that
       CFString prepends UTF encoded data with the Unicode BOM <http://www.unicode.org/faq/utf_bom.html> 
       when generating external representation if the target encoding allows. It's important to note that
       only UTF-8, UTF-16, and UTF-32 define the handling of the byte order mark character, and the "LE"
       and "BE" variants of UTF-16 and UTF-32 don't.
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
   the data to be made persistent to be included; for instance, the Unicode BOM <http://www.unicode.org/faq/utf_bom.html> 
   when generating external representation if the target encoding allows. It's important to note that
   only UTF-8, UTF-16, and UTF-32 define the handling of the byte order mark character, and the "LE"
   and "BE" variants of UTF-16 and UTF-32 don't.
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
}
function CFStringGetBytes( theString: CFStringRef; range: CFRange; encoding: CFStringEncoding; lossByte: ByteParameter; isExternalRepresentation: Boolean; buffer: UInt8Ptr; maxBufLen: CFIndex; var usedBufLen: CFIndex ): CFIndex; external name '_CFStringGetBytes';

{ Convenience functions String <-> Data. These generate "external" formats, that is, formats that
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   can be written out to disk. For instance, if the encoding is Unicode,
   CFStringCreateFromExternalRepresentation() pays attention to the BOM character (if any) 
   and does byte swapping if necessary. Similarly CFStringCreateExternalRepresentation() will  
   include a BOM character if appropriate. See CFStringGetBytes() for more on this and lossByte.
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  can be written out to disk. For instance, if the encoding is Unicode,
  CFStringCreateFromExternalRepresentation() pays attention to the BOM character (if any) 
  and does byte swapping if necessary. Similarly CFStringCreateExternalRepresentation() will  
  include a BOM character if appropriate. See CFStringGetBytes() for more on this and lossByte.
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
}
function CFStringCreateFromExternalRepresentation( alloc: CFAllocatorRef; data: CFDataRef; encoding: CFStringEncoding ): CFStringRef; external name '_CFStringCreateFromExternalRepresentation';	{ May return NULL on conversion error }
=======
=======
>>>>>>> origin/fixes_2_2
   into a fixed size buffer. Returns number of characters converted. 
   Characters that cannot be converted to the specified encoding are represented
   with the byte specified by lossByte; if lossByte is 0, then lossy conversion
   is not allowed and conversion stops, returning partial results.
   Pass buffer==NULL if you don't care about the converted string (but just the convertability,
   or number of bytes required). 
   maxBufLength indicates the maximum number of bytes to generate. It is ignored when buffer==NULL.
   Does not zero-terminate. If you want to create Pascal or C string, allow one extra byte at start or end. 
   Setting isExternalRepresentation causes any extra bytes that would allow 
   the data to be made persistent to be included; for instance, the Unicode BOM <http://www.unicode.org/faq/utf_bom.html> 
   when generating external representation if the target encoding allows. It's important to note that
   only UTF-8, UTF-16, and UTF-32 define the handling of the byte order mark character, and the "LE"
   and "BE" variants of UTF-16 and UTF-32 don't.
}
function CFStringGetBytes( theString: CFStringRef; range: CFRange; encoding: CFStringEncoding; lossByte: ByteParameter; isExternalRepresentation: Boolean; buffer: UInt8Ptr; maxBufLen: CFIndex; var usedBufLen: CFIndex ): CFIndex; external name '_CFStringGetBytes';

{ Convenience functions String <-> Data. These generate "external" formats, that is, formats that
  can be written out to disk. For instance, if the encoding is Unicode,
  CFStringCreateFromExternalRepresentation() pays attention to the BOM character (if any) 
  and does byte swapping if necessary. Similarly CFStringCreateExternalRepresentation() will  
  include a BOM character if appropriate. See CFStringGetBytes() for more on this and lossByte.
}
function CFStringCreateFromExternalRepresentation( alloc: CFAllocatorRef; data: CFDataRef; encoding: CFStringEncoding ): CFStringRef; external name '_CFStringCreateFromExternalRepresentation'; { May return NULL on conversion error }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

function CFStringCreateExternalRepresentation( alloc: CFAllocatorRef; theString: CFStringRef; encoding: CFStringEncoding; lossByte: ByteParameter ): CFDataRef; external name '_CFStringCreateExternalRepresentation'; { May return NULL on conversion error }

{ Hints about the contents of a string
}
<<<<<<< HEAD
<<<<<<< HEAD
function CFStringGetSmallestEncoding( theString: CFStringRef ): CFStringEncoding; external name '_CFStringGetSmallestEncoding';	{ Result in O(n) time max }

function CFStringGetFastestEncoding( theString: CFStringRef ): CFStringEncoding; external name '_CFStringGetFastestEncoding';	{ Result in O(1) time max }

{ General encoding info
}
function CFStringGetSystemEncoding: CFStringEncoding; external name '_CFStringGetSystemEncoding';		{ The default encoding for the system; untagged 8-bit characters are usually in this encoding }
=======
=======
>>>>>>> origin/fixes_2_2
function CFStringGetSmallestEncoding( theString: CFStringRef ): CFStringEncoding; external name '_CFStringGetSmallestEncoding'; { Result in O(n) time max }

function CFStringGetFastestEncoding( theString: CFStringRef ): CFStringEncoding; external name '_CFStringGetFastestEncoding'; { Result in O(1) time max }

{ General encoding info
}
function CFStringGetSystemEncoding: CFStringEncoding; external name '_CFStringGetSystemEncoding'; { The default encoding for the system; untagged 8-bit characters are usually in this encoding }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

function CFStringGetMaximumSizeForEncoding( length: CFIndex; encoding: CFStringEncoding ): CFIndex; external name '_CFStringGetMaximumSizeForEncoding'; { Max bytes a string of specified length (in UniChars) will take up if encoded }


{** FileSystem path conversion functions **}

<<<<<<< HEAD
<<<<<<< HEAD
{ Extract the contents of the string as a NULL-terminated 8-bit string appropriate for passing to POSIX APIs (for example, normalized for HFS+).  The string is zero-terminated. false will be returned if the conversion results don't fit into the buffer.  Use CFStringGetMaximumSizeOfFileSystemRepresentation() if you want to make sure the buffer is of sufficient length.
=======
{ Extract the contents of the string as a NULL-terminated 8-bit string appropriate for passing to POSIX APIs.  The string is zero-terminated. false will be returned if the conversion results don't fit into the buffer.  Use CFStringGetMaximumSizeOfFileSystemRepresentation() if you want to make sure the buffer is of sufficient length.
>>>>>>> graemeg/fixes_2_2
=======
{ Extract the contents of the string as a NULL-terminated 8-bit string appropriate for passing to POSIX APIs.  The string is zero-terminated. false will be returned if the conversion results don't fit into the buffer.  Use CFStringGetMaximumSizeOfFileSystemRepresentation() if you want to make sure the buffer is of sufficient length.
>>>>>>> origin/fixes_2_2
}
function CFStringGetFileSystemRepresentation( strng: CFStringRef; buffer: CStringPtr; maxBufLen: CFIndex ): Boolean; external name '_CFStringGetFileSystemRepresentation';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{ Get the upper bound on the number of bytes required to hold the file system representation for the string. This result is returned quickly as a very rough approximation, and could be much larger than the actual space required. The result includes space for the zero termination. If you are allocating a buffer for long-term keeping, it's recommended that you reallocate it smaller (to be the right size) after calling CFStringGetFileSystemRepresentation(). 
}
function CFStringGetMaximumSizeOfFileSystemRepresentation( strng: CFStringRef ): CFIndex; external name '_CFStringGetMaximumSizeOfFileSystemRepresentation';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{ Create a CFString from the specified zero-terminated POSIX file system representation.  If the conversion fails (possible due to bytes in the buffer not being a valid sequence of bytes for the appropriate character encoding), NULL is returned.
}
function CFStringCreateWithFileSystemRepresentation( alloc: CFAllocatorRef; buffer: ConstCStringPtr ): CFStringRef; external name '_CFStringCreateWithFileSystemRepresentation';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{** Comparison functions. **}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ Find and compare flags; these are OR'ed together and provided as CFStringCompareFlags in the various functions. 
}
type
	CFStringCompareFlags = CFOptionFlags;
=======
{ Find and compare flags; these are OR'ed together as compareOptions or searchOptions in the various functions. 
}
type
	CFStringCompareFlags = UNSIGNEDLONG;
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
=======
>>>>>>> origin/fixes_2_2
{ Find and compare flags; these are OR'ed together as compareOptions or searchOptions in the various functions. 
}
type
<<<<<<< HEAD
	CFStringCompareFlags = SInt32;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	CFStringCompareFlags = UNSIGNEDLONG;
>>>>>>> origin/fixes_2.4
const
																{  Flags used in all find and compare operations  }
	kCFCompareCaseInsensitive = 1;
	kCFCompareBackwards = 4;							{  Starting from the end of the string  }
	kCFCompareAnchored = 8;							{  Only at the specified starting point  }
	kCFCompareNonliteral = 16;							{  If specified, loose equivalence is performed (o-umlaut == o, umlaut)  }
	kCFCompareLocalized = 32;							{  User's default locale is used for the comparisons  }
	kCFCompareNumerically = 64;							{  Numeric comparison is used; that is, Foo2.txt < Foo7.txt < Foo25.txt  }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kCFCompareDiacriticInsensitive = 128; (* CF_AVAILABLE_STARTING(10_5, 2_0) *) { If specified, ignores diacritics (o-umlaut == o) }
	kCFCompareWidthInsensitive = 256; (* CF_AVAILABLE_STARTING(10_5, 2_0) *) { If specified, ignores width differences ('a' == UFF41) }
	kCFCompareForcedOrdering = 512; (* CF_AVAILABLE_STARTING(10_5, 2_0) *) { If specified, comparisons are forced to return either kCFCompareLessThan or kCFCompareGreaterThan if the strings are equivalent but not strictly equal, for stability when sorting (e.g. "aaa" > "AAA" with kCFCompareCaseInsensitive specified) }

{ The main comparison routine; compares specified range of the first string to (the full range of) the second string.
locale == NULL indicates canonical locale (the return value from CFLocaleGetSystem()).
kCFCompareNumerically, added in 10.2, does not work if kCFCompareLocalized is specified on systems before 10.3
kCFCompareBackwards and kCFCompareAnchored are not applicable.
}
function CFStringCompareWithOptionsAndLocale( theString1: CFStringRef; theString2: CFStringRef; rangeToCompare: CFRange; compareOptions: CFStringCompareFlags; locale: CFLocaleRef ): CFComparisonResult; external name '_CFStringCompareWithOptionsAndLocale';
(* CF_AVAILABLE_STARTING(10_5, 2_0) *)

{ Comparison convenience. Uses the current user locale (the return value from CFLocaleCopyCurrent()) if kCFCompareLocalized.
}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
	kCFCompareDiacriticInsensitive = 128; (* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *) { If specified, ignores diacritics (o-umlaut == o) }
	kCFCompareWidthInsensitive = 256; (* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *) { If specified, ignores width differences ('a' == UFF41) }
	kCFCompareForcedOrdering = 512; (* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *) { If specified, comparisons are forced to return either kCFCompareLessThan or kCFCompareGreaterThan if the strings are equivalent but not strictly equal, for stability when sorting (e.g. "aaa" > "AAA" with kCFCompareCaseInsensitive specified) }

{ The main comparison routine; compares specified range of the first string to (the full range of) the second string.
=======
	kCFCompareDiacriticInsensitive = 128; (* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *) { If specified, ignores diacritics (o-umlaut == o) }
	kCFCompareWidthInsensitive = 256; (* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *) { If specified, ignores width differences ('a' == UFF41) }
	kCFCompareForcedOrdering = 512; (* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *) { If specified, comparisons are forced to return either kCFCompareLessThan or kCFCompareGreaterThan if the strings are equivalent but not strictly equal, for stability when sorting (e.g. "aaa" > "AAA" with kCFCompareCaseInsensitive specified) }

{ The main comparison routine; compares specified range of the first string to (the full range of) the second string.
>>>>>>> origin/cpstrnew
   locale == NULL indicates canonical locale (the return value from CFLocaleGetSystem()).
   kCFCompareNumerically, added in 10.2, does not work if kCFCompareLocalized is specified on systems before 10.3
   kCFCompareBackwards and kCFCompareAnchored are not applicable.
}
function CFStringCompareWithOptionsAndLocale( theString1: CFStringRef; theString2: CFStringRef; rangeToCompare: CFRange; compareOptions: CFStringCompareFlags; locale: CFLocaleRef ): CFComparisonResult; external name '_CFStringCompareWithOptionsAndLocale';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{ Comparison convenience. Uses the current user locale (the return value from CFLocaleCopyCurrent()) if kCFCompareLocalized.
}
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
function CFStringCompareWithOptions( theString1: CFStringRef; theString2: CFStringRef; rangeToCompare: CFRange; compareOptions: CFStringCompareFlags ): CFComparisonResult; external name '_CFStringCompareWithOptions';
=======
=======
>>>>>>> origin/fixes_2_2
=======
	kCFCompareDiacriticInsensitive = 128; (* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *) { If specified, ignores diacritics (o-umlaut == o) }
	kCFCompareWidthInsensitive = 256; (* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *) { If specified, ignores width differences ('a' == UFF41) }
	kCFCompareForcedOrdering = 512; (* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *) { If specified, comparisons are forced to return either kCFCompareLessThan or kCFCompareGreaterThan if the strings are equivalent but not strictly equal, for stability when sorting (e.g. "aaa" > "AAA" with kCFCompareCaseInsensitive specified) }
>>>>>>> origin/fixes_2.4

{ The main comparison routine; compares specified range of the first string to (the full range of) the second string.
   locale == NULL indicates canonical locale (the return value from CFLocaleGetSystem()).
   kCFCompareNumerically, added in 10.2, does not work if kCFCompareLocalized is specified on systems before 10.3
   kCFCompareBackwards and kCFCompareAnchored are not applicable.
}
<<<<<<< HEAD
function CFStringCompareWithOptions( theString1: CFStringRef; theString2: CFStringRef; rangeToCompare: CFRange; compareOptions: CFOptionFlags ): CFComparisonResult; external name '_CFStringCompareWithOptions';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CFStringCompareWithOptionsAndLocale( theString1: CFStringRef; theString2: CFStringRef; rangeToCompare: CFRange; compareOptions: CFStringCompareFlags; locale: CFLocaleRef ): CFComparisonResult; external name '_CFStringCompareWithOptionsAndLocale';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{ Comparison convenience. Uses the current user locale (the return value from CFLocaleCopyCurrent()) if kCFCompareLocalized.
}
function CFStringCompareWithOptions( theString1: CFStringRef; theString2: CFStringRef; rangeToCompare: CFRange; compareOptions: CFStringCompareFlags ): CFComparisonResult; external name '_CFStringCompareWithOptions';
>>>>>>> origin/fixes_2.4

{ Comparison convenience suitable for passing as sorting functions.
   kCFCompareNumerically, added in 10.2, does not work if kCFCompareLocalized is specified on systems before 10.3
   kCFCompareBackwards and kCFCompareAnchored are not applicable.
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function CFStringCompare( theString1: CFStringRef; theString2: CFStringRef; compareOptions: CFStringCompareFlags ): CFComparisonResult; external name '_CFStringCompare';
<<<<<<< HEAD
<<<<<<< HEAD

{ CFStringFindWithOptionsAndLocale() returns the found range in the CFRange * argument; you can pass NULL for simple discovery check.
 locale == NULL indicates canonical locale (the return value from CFLocaleGetSystem()).
 If stringToFind is the empty string (zero length), nothing is found.
 Ignores the kCFCompareNumerically option.
<<<<<<< HEAD
<<<<<<< HEAD
}
function CFStringFindWithOptionsAndLocale( theString: CFStringRef; stringToFind: CFStringRef; rangeToSearch: CFRange; searchOptions: CFStringCompareFlags; locale: CFLocaleRef; var result: CFRange ): Boolean; external name '_CFStringFindWithOptionsAndLocale';
(* CF_AVAILABLE_STARTING(10_5, 2_0) *)

{ Find convenience. Uses the current user locale (the return value from CFLocaleCopyCurrent()) if kCFCompareLocalized.
}
=======
=======
>>>>>>> graemeg/cpstrnew
}
function CFStringFindWithOptionsAndLocale( theString: CFStringRef; stringToFind: CFStringRef; rangeToSearch: CFRange; searchOptions: CFStringCompareFlags; locale: CFLocaleRef; var result: CFRange ): Boolean; external name '_CFStringFindWithOptionsAndLocale';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{ Find convenience. Uses the current user locale (the return value from CFLocaleCopyCurrent()) if kCFCompareLocalized.
}
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======

{ CFStringFindWithOptionsAndLocale() returns the found range in the CFRange * argument; you can pass NULL for simple discovery check.
 locale == NULL indicates canonical locale (the return value from CFLocaleGetSystem()).
 If stringToFind is the empty string (zero length), nothing is found.
 Ignores the kCFCompareNumerically option.
}
function CFStringFindWithOptionsAndLocale( theString: CFStringRef; stringToFind: CFStringRef; rangeToSearch: CFRange; searchOptions: CFStringCompareFlags; locale: CFLocaleRef; var result: CFRange ): Boolean; external name '_CFStringFindWithOptionsAndLocale';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{ Find convenience. Uses the current user locale (the return value from CFLocaleCopyCurrent()) if kCFCompareLocalized.
}
>>>>>>> graemeg/cpstrnew
=======

{ CFStringFindWithOptionsAndLocale() returns the found range in the CFRange * argument; you can pass NULL for simple discovery check.
 locale == NULL indicates canonical locale (the return value from CFLocaleGetSystem()).
 If stringToFind is the empty string (zero length), nothing is found.
 Ignores the kCFCompareNumerically option.
}
function CFStringFindWithOptionsAndLocale( theString: CFStringRef; stringToFind: CFStringRef; rangeToSearch: CFRange; searchOptions: CFStringCompareFlags; locale: CFLocaleRef; var result: CFRange ): Boolean; external name '_CFStringFindWithOptionsAndLocale';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{ Find convenience. Uses the current user locale (the return value from CFLocaleCopyCurrent()) if kCFCompareLocalized.
}
>>>>>>> origin/cpstrnew
function CFStringFindWithOptions( theString: CFStringRef; stringToFind: CFStringRef; rangeToSearch: CFRange; searchOptions: CFStringCompareFlags; var result: CFRange ): Boolean; external name '_CFStringFindWithOptions';
=======
=======
>>>>>>> origin/fixes_2_2
function CFStringCompare( theString1: CFStringRef; theString2: CFStringRef; compareOptions: CFOptionFlags ): CFComparisonResult; external name '_CFStringCompare';
=======
function CFStringCompare( theString1: CFStringRef; theString2: CFStringRef; compareOptions: CFStringCompareFlags ): CFComparisonResult; external name '_CFStringCompare';
>>>>>>> origin/fixes_2.4

{ CFStringFindWithOptionsAndLocale() returns the found range in the CFRange * argument; you can pass NULL for simple discovery check.
 locale == NULL indicates canonical locale (the return value from CFLocaleGetSystem()).
 If stringToFind is the empty string (zero length), nothing is found.
 Ignores the kCFCompareNumerically option.
}
function CFStringFindWithOptionsAndLocale( theString: CFStringRef; stringToFind: CFStringRef; rangeToSearch: CFRange; searchOptions: CFStringCompareFlags; locale: CFLocaleRef; var result: CFRange ): Boolean; external name '_CFStringFindWithOptionsAndLocale';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{ Find convenience. Uses the current user locale (the return value from CFLocaleCopyCurrent()) if kCFCompareLocalized.
}
<<<<<<< HEAD
function CFStringFindWithOptions( theString: CFStringRef; stringToFind: CFStringRef; rangeToSearch: CFRange; searchOptions: CFOptionFlags; var result: CFRange ): Boolean; external name '_CFStringFindWithOptions';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CFStringFindWithOptions( theString: CFStringRef; stringToFind: CFStringRef; rangeToSearch: CFRange; searchOptions: CFStringCompareFlags; var result: CFRange ): Boolean; external name '_CFStringFindWithOptions';
>>>>>>> origin/fixes_2.4

{ CFStringCreateArrayWithFindResults() returns an array of CFRange pointers, or NULL if there are no matches.
   Overlapping instances are not found; so looking for "AA" in "AAA" finds just one range.
   Post 10.1: If kCFCompareBackwards is provided, the scan is done from the end (which can give a different result), and
      the results are stored in the array backwards (last found range in slot 0).
   If stringToFind is the empty string (zero length), nothing is found.
   kCFCompareAnchored causes just the consecutive instances at start (or end, if kCFCompareBackwards) to be reported. So, searching for "AB" in "ABABXAB..." you just get the first two occurrences.
   Ignores the kCFCompareNumerically option.
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function CFStringCreateArrayWithFindResults( alloc: CFAllocatorRef; theString: CFStringRef; stringToFind: CFStringRef; rangeToSearch: CFRange; compareOptions: CFStringCompareFlags ): CFArrayRef; external name '_CFStringCreateArrayWithFindResults';

{ Find conveniences; see comments above concerning empty string and options.
}
function CFStringFind( theString: CFStringRef; stringToFind: CFStringRef; compareOptions: CFStringCompareFlags ): CFRange; external name '_CFStringFind';
=======
=======
>>>>>>> origin/fixes_2_2
function CFStringCreateArrayWithFindResults( alloc: CFAllocatorRef; theString: CFStringRef; stringToFind: CFStringRef; rangeToSearch: CFRange; compareOptions: CFOptionFlags ): CFArrayRef; external name '_CFStringCreateArrayWithFindResults';

{ Find conveniences; see comments above concerning empty string and options.
}
function CFStringFind( theString: CFStringRef; stringToFind: CFStringRef; compareOptions: CFOptionFlags ): CFRange; external name '_CFStringFind';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CFStringCreateArrayWithFindResults( alloc: CFAllocatorRef; theString: CFStringRef; stringToFind: CFStringRef; rangeToSearch: CFRange; compareOptions: CFStringCompareFlags ): CFArrayRef; external name '_CFStringCreateArrayWithFindResults';

{ Find conveniences; see comments above concerning empty string and options.
}
function CFStringFind( theString: CFStringRef; stringToFind: CFStringRef; compareOptions: CFStringCompareFlags ): CFRange; external name '_CFStringFind';
>>>>>>> origin/fixes_2.4

function CFStringHasPrefix( theString: CFStringRef; prefix: CFStringRef ): Boolean; external name '_CFStringHasPrefix';

function CFStringHasSuffix( theString: CFStringRef; suffix: CFStringRef ): Boolean; external name '_CFStringHasSuffix';

{#if MAC_OS_X_VERSION_10_2 <= MAC_OS_X_VERSION_MAX_ALLOWED}
{!
	@function CFStringGetRangeOfComposedCharactersAtIndex
	Returns the range of the composed character sequence at the specified index.
	@param theString The CFString which is to be searched.  If this
                		parameter is not a valid CFString, the behavior is
              		undefined.
	@param theIndex The index of the character contained in the
			composed character sequence.  If the index is
			outside the index space of the string (0 to N-1 inclusive,
			where N is the length of the string), the behavior is
			undefined.
	@result The range of the composed character sequence.
}
function CFStringGetRangeOfComposedCharactersAtIndex( theString: CFStringRef; theIndex: CFIndex ): CFRange; external name '_CFStringGetRangeOfComposedCharactersAtIndex';

{!
	@function CFStringFindCharacterFromSet
	Query the range of the first character contained in the specified character set.
	@param theString The CFString which is to be searched.  If this
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      parameter is not a valid CFString, the behavior is
      undefined.
=======
                		parameter is not a valid CFString, the behavior is
              		undefined.
>>>>>>> graemeg/fixes_2_2
=======
                		parameter is not a valid CFString, the behavior is
              		undefined.
>>>>>>> origin/fixes_2_2
=======
      parameter is not a valid CFString, the behavior is
      undefined.
>>>>>>> origin/fixes_2.4
	@param theSet The CFCharacterSet against which the membership
			of characters is checked.  If this parameter is not a valid
			CFCharacterSet, the behavior is undefined.
	@param range The range of characters within the string to search. If
			the range location or end point (defined by the location
			plus length minus 1) are outside the index space of the
			string (0 to N-1 inclusive, where N is the length of the
			string), the behavior is undefined. If the range length is
			negative, the behavior is undefined. The range may be empty
			(length 0), in which case no search is performed.
	@param searchOptions The bitwise-or'ed option flags to control
			the search behavior.  The supported options are
			kCFCompareBackwards andkCFCompareAnchored.
			If other option flags are specified, the behavior
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      is undefined.
	@param result The pointer to a CFRange supplied by the caller in
			which the search result is stored.  Note that the length
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
			of this range can be more than 1, if for instance the 
			result is a composed character. If a pointer to an invalid
=======
      of this range can be more than 1, if for instance the
      result is a composed character. If a pointer to an invalid
>>>>>>> graemeg/cpstrnew
=======
      of this range can be more than 1, if for instance the
      result is a composed character. If a pointer to an invalid
>>>>>>> graemeg/cpstrnew
=======
      of this range can be more than 1, if for instance the
      result is a composed character. If a pointer to an invalid
>>>>>>> graemeg/cpstrnew
=======
      of this range can be more than 1, if for instance the
      result is a composed character. If a pointer to an invalid
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
                        is undefined.
	@param result The pointer to a CFRange supplied by the caller in
			which the search result is stored.  Note that the length
                        of this range could be more than If a pointer to an invalid
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
      is undefined.
	@param result The pointer to a CFRange supplied by the caller in
			which the search result is stored.  Note that the length
      of this range can be more than 1, if for instance the
      result is a composed character. If a pointer to an invalid
>>>>>>> origin/fixes_2.4
			memory is specified, the behavior is undefined.
	@result true, if at least a character which is a member of the character
			set is found and result is filled, otherwise, false.
}
function CFStringFindCharacterFromSet( theString: CFStringRef; theSet: CFCharacterSetRef; rangeToSearch: CFRange; searchOptions: CFOptionFlags; var result: CFRange ): Boolean; external name '_CFStringFindCharacterFromSet';
{#endif}

{ Find range of bounds of the line(s) that span the indicated range (startIndex, numChars),
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   taking into account various possible line separator sequences (CR, CRLF, LF, and Unicode NextLine, LineSeparator, ParagraphSeparator).
   All return values are "optional" (provide NULL if you don't want them)
     lineBeginIndex: index of first character in line
     lineEndIndex: index of first character of the next line (including terminating line separator characters)
     contentsEndIndex: index of the first line separator character
   Thus, lineEndIndex - lineBeginIndex is the number of chars in the line, including the line separators
         contentsEndIndex - lineBeginIndex is the number of chars in the line w/out the line separators
}
procedure CFStringGetLineBounds( theString: CFStringRef; range: CFRange; var lineBeginIndex: CFIndex; var lineEndIndex: CFIndex; var contentsEndIndex: CFIndex ); external name '_CFStringGetLineBounds';

{ Same as CFStringGetLineBounds(), however, will only look for paragraphs. Won't stop at Unicode NextLine or LineSeparator characters.
}
procedure CFStringGetParagraphBounds( strng: CFStringRef; range: CFRange; var parBeginIndex: CFIndex; var parEndIndex: CFIndex; var contentsEndIndex: CFIndex ); external name '_CFStringGetParagraphBounds';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CF_AVAILABLE_STARTING(10_5, 2_0) *)

{!
	@function CFStringGetHyphenationLocationBeforeIndex
	Retrieve the first potential hyphenation location found before the specified location.
	@param string The CFString which is to be hyphenated.  If this
                		parameter is not a valid CFString, the behavior is
              		undefined.
	@param location An index in the string.  If a valid hyphen index is returned, it 
	                will be before this index.
	@param limitRange The range of characters within the string to search. If
			the range location or end point (defined by the location
			plus length minus 1) are outside the index space of the
			string (0 to N-1 inclusive, where N is the length of the
			string), the behavior is undefined. If the range length is
			negative, the behavior is undefined. The range may be empty
			(length 0), in which case no hyphen location is generated.
	@param options Reserved for future use.
	@param locale Specifies which language's hyphenation conventions to use.
			This must be a valid locale.  Hyphenation data is not available
			for all locales.  You can use CFStringIsHyphenationAvailableForLocale
			to test for availability of hyphenation data.
	@param character The suggested hyphen character to insert.  Pass NULL if you
			do not need this information.
	@result an index in the string where it is appropriate to insert a hyphen, if
			one exists; else kCFNotFound
}
function CFStringGetHyphenationLocationBeforeIndex( strng: CFStringRef; location: CFIndex; limitRange: CFRange; options: CFOptionFlags; locale: CFLocaleRef; character { may be null}: UTF32CharPtr ): CFIndex; external name '_CFStringGetHyphenationLocationBeforeIndex';
(* CF_AVAILABLE_STARTING(10_7, 4_2) *)

function CFStringIsHyphenationAvailableForLocale( locale: CFLocaleRef ): Boolean; external name '_CFStringIsHyphenationAvailableForLocale';
(* CF_AVAILABLE_STARTING(10_7, 4_3) *)
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

{** Exploding and joining strings with a separator string **}

function CFStringCreateByCombiningStrings( alloc: CFAllocatorRef; theArray: CFArrayRef; separatorString: CFStringRef ): CFStringRef; external name '_CFStringCreateByCombiningStrings';	{ Empty array returns empty string; one element array returns the element }

function CFStringCreateArrayBySeparatingStrings( alloc: CFAllocatorRef; theString: CFStringRef; separatorString: CFStringRef ): CFArrayRef; external name '_CFStringCreateArrayBySeparatingStrings';	{ No separators in the string returns array with that string; string == sep returns two empty strings }
=======
=======
>>>>>>> origin/fixes_2_2
   taking into account various possible line separator sequences (CR, CRLF, LF, and Unicode LS, PS).
=======
   taking into account various possible line separator sequences (CR, CRLF, LF, and Unicode NextLine, LineSeparator, ParagraphSeparator).
>>>>>>> origin/fixes_2.4
   All return values are "optional" (provide NULL if you don't want them)
     lineBeginIndex: index of first character in line
     lineEndIndex: index of first character of the next line (including terminating line separator characters)
     contentsEndIndex: index of the first line separator character
   Thus, lineEndIndex - lineBeginIndex is the number of chars in the line, including the line separators
         contentsEndIndex - lineBeginIndex is the number of chars in the line w/out the line separators
}
procedure CFStringGetLineBounds( theString: CFStringRef; range: CFRange; var lineBeginIndex: CFIndex; var lineEndIndex: CFIndex; var contentsEndIndex: CFIndex ); external name '_CFStringGetLineBounds';

{ Same as CFStringGetLineBounds(), however, will only look for paragraphs. Won't stop at Unicode NextLine or LineSeparator characters.
}
procedure CFStringGetParagraphBounds( strng: CFStringRef; range: CFRange; var parBeginIndex: CFIndex; var parEndIndex: CFIndex; var contentsEndIndex: CFIndex ); external name '_CFStringGetParagraphBounds';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{** Exploding and joining strings with a separator string **}

function CFStringCreateByCombiningStrings( alloc: CFAllocatorRef; theArray: CFArrayRef; separatorString: CFStringRef ): CFStringRef; external name '_CFStringCreateByCombiningStrings'; { Empty array returns empty string; one element array returns the element }

function CFStringCreateArrayBySeparatingStrings( alloc: CFAllocatorRef; theString: CFStringRef; separatorString: CFStringRef ): CFArrayRef; external name '_CFStringCreateArrayBySeparatingStrings'; { No separators in the string returns array with that string; string == sep returns two empty strings }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2


{** Parsing non-localized numbers from strings **}

<<<<<<< HEAD
<<<<<<< HEAD
function CFStringGetIntValue( str: CFStringRef ): SInt32; external name '_CFStringGetIntValue';		{ Skips whitespace; returns 0 on error, MAX or -MAX on overflow }

function CFStringGetDoubleValue( str: CFStringRef ): Float64; external name '_CFStringGetDoubleValue';	{ Skips whitespace; returns 0.0 on error }
=======
function CFStringGetIntValue( str: CFStringRef ): SInt32; external name '_CFStringGetIntValue'; { Skips whitespace; returns 0 on error, MAX or -MAX on overflow }

function CFStringGetDoubleValue( str: CFStringRef ): Float64; external name '_CFStringGetDoubleValue'; { Skips whitespace; returns 0.0 on error }
>>>>>>> graemeg/fixes_2_2
=======
function CFStringGetIntValue( str: CFStringRef ): SInt32; external name '_CFStringGetIntValue'; { Skips whitespace; returns 0 on error, MAX or -MAX on overflow }

function CFStringGetDoubleValue( str: CFStringRef ): Float64; external name '_CFStringGetDoubleValue'; { Skips whitespace; returns 0.0 on error }
>>>>>>> origin/fixes_2_2


{** MutableString functions **}

{ CFStringAppend("abcdef", "xxxxx") -> "abcdefxxxxx"
   CFStringDelete("abcdef", CFRangeMake(2, 3)) -> "abf"
   CFStringReplace("abcdef", CFRangeMake(2, 3), "xxxxx") -> "abxxxxxf"
   CFStringReplaceAll("abcdef", "xxxxx") -> "xxxxx"
}
procedure CFStringAppend( theString: CFMutableStringRef; appendedString: CFStringRef ); external name '_CFStringAppend';

procedure CFStringAppendCharacters( theString: CFMutableStringRef; chars: UniCharPtr; numChars: CFIndex ); external name '_CFStringAppendCharacters';

procedure CFStringAppendPascalString( theString: CFMutableStringRef; const (*var*) pStr: Str255; encoding: CFStringEncoding ); external name '_CFStringAppendPascalString';

procedure CFStringAppendCString( theString: CFMutableStringRef; cStr: ConstCStringPtr; encoding: CFStringEncoding ); external name '_CFStringAppendCString';

procedure CFStringAppendFormat( theString: CFMutableStringRef; formatOptions: CFDictionaryRef; format: CFStringRef; ... ); external name '_CFStringAppendFormat';

procedure CFStringAppendFormatAndArguments( theString: CFMutableStringRef; formatOptions: CFDictionaryRef; format: CFStringRef; arguments: UnivPtr ); external name '_CFStringAppendFormatAndArguments';

procedure CFStringInsert( str: CFMutableStringRef; idx: CFIndex; insertedStr: CFStringRef ); external name '_CFStringInsert';

procedure CFStringDelete( theString: CFMutableStringRef; range: CFRange ); external name '_CFStringDelete';

procedure CFStringReplace( theString: CFMutableStringRef; range: CFRange; replacement: CFStringRef ); external name '_CFStringReplace';

<<<<<<< HEAD
<<<<<<< HEAD
procedure CFStringReplaceAll( theString: CFMutableStringRef; replacement: CFStringRef ); external name '_CFStringReplaceAll';	{ Replaces whole string }
=======
procedure CFStringReplaceAll( theString: CFMutableStringRef; replacement: CFStringRef ); external name '_CFStringReplaceAll'; { Replaces whole string }
>>>>>>> graemeg/fixes_2_2
=======
procedure CFStringReplaceAll( theString: CFMutableStringRef; replacement: CFStringRef ); external name '_CFStringReplaceAll'; { Replaces whole string }
>>>>>>> origin/fixes_2_2

{#if MAC_OS_X_VERSION_10_2 <= MAC_OS_X_VERSION_MAX_ALLOWED}
{ Replace all occurrences of target in rangeToSearch of theString with replacement.
   Pays attention to kCFCompareCaseInsensitive, kCFCompareBackwards, kCFCompareNonliteral, and kCFCompareAnchored.
   kCFCompareBackwards can be used to do the replacement starting from the end, which could give a different result.
     ex. AAAAA, replace AA with B -> BBA or ABB; latter if kCFCompareBackwards
   kCFCompareAnchored assures only anchored but multiple instances are found (the instances must be consecutive at start or end)
     ex. AAXAA, replace A with B -> BBXBB or BBXAA; latter if kCFCompareAnchored
   Returns number of replacements performed.
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function CFStringFindAndReplace( theString: CFMutableStringRef; stringToFind: CFStringRef; replacementString: CFStringRef; rangeToSearch: CFRange; compareOptions: CFStringCompareFlags ): CFIndex; external name '_CFStringFindAndReplace';
=======
function CFStringFindAndReplace( theString: CFMutableStringRef; stringToFind: CFStringRef; replacementString: CFStringRef; rangeToSearch: CFRange; compareOptions: CFOptionFlags ): CFIndex; external name '_CFStringFindAndReplace';
>>>>>>> graemeg/fixes_2_2
=======
function CFStringFindAndReplace( theString: CFMutableStringRef; stringToFind: CFStringRef; replacementString: CFStringRef; rangeToSearch: CFRange; compareOptions: CFOptionFlags ): CFIndex; external name '_CFStringFindAndReplace';
>>>>>>> origin/fixes_2_2
=======
function CFStringFindAndReplace( theString: CFMutableStringRef; stringToFind: CFStringRef; replacementString: CFStringRef; rangeToSearch: CFRange; compareOptions: CFStringCompareFlags ): CFIndex; external name '_CFStringFindAndReplace';
>>>>>>> origin/fixes_2.4

{#endif}

{ This function will make the contents of a mutable CFString point directly at the specified UniChar array.
   It works only with CFStrings created with CFStringCreateMutableWithExternalCharactersNoCopy().
   This function does not free the previous buffer.
   The string will be manipulated within the provided buffer (if any) until it outgrows capacity; then the
     externalCharactersAllocator will be consulted for more memory.
   See comments at the top of this file for more info.
}
procedure CFStringSetExternalCharactersNoCopy( theString: CFMutableStringRef; chars: UniCharPtr; length: CFIndex; capacity: CFIndex ); external name '_CFStringSetExternalCharactersNoCopy'; { Works only on specially created mutable strings! }

{ CFStringPad() will pad or cut down a string to the specified size.
   The pad string is used as the fill string; indexIntoPad specifies which character to start with.
     CFStringPad("abc", " ", 9, 0) ->  "abc      "
     CFStringPad("abc", ". ", 9, 1) -> "abc . . ."
     CFStringPad("abcdef", ?, 3, ?) -> "abc"

     CFStringTrim() will trim the specified string from both ends of the string.
     CFStringTrimWhitespace() will do the same with white space characters (tab, newline, etc)
     CFStringTrim("  abc ", " ") -> "abc"
     CFStringTrim("* * * *abc * ", "* ") -> "*abc "
}
procedure CFStringPad( theString: CFMutableStringRef; padString: CFStringRef; length: CFIndex; indexIntoPad: CFIndex ); external name '_CFStringPad';

procedure CFStringTrim( theString: CFMutableStringRef; trimString: CFStringRef ); external name '_CFStringTrim';

procedure CFStringTrimWhitespace( theString: CFMutableStringRef ); external name '_CFStringTrimWhitespace';

{#if MAC_OS_X_VERSION_10_3 <= MAC_OS_X_VERSION_MAX_ALLOWED}
procedure CFStringLowercase( theString: CFMutableStringRef; locale: CFLocaleRef ); external name '_CFStringLowercase';

procedure CFStringUppercase( theString: CFMutableStringRef; locale: CFLocaleRef ); external name '_CFStringUppercase';

procedure CFStringCapitalize( theString: CFMutableStringRef; locale: CFLocaleRef ); external name '_CFStringCapitalize';
{#else}
//procedure CFStringLowercase( theString: CFMutableStringRef; localeTBD: {const} UnivPtr ); // localeTBD must be NULL on pre-10.3

//procedure CFStringUppercase( theString: CFMutableStringRef; localeTBD: {const} UnivPtr ); // localeTBD must be NULL on pre-10.3

//procedure CFStringCapitalize( theString: CFMutableStringRef; localeTBD: {const} UnivPtr ); // localeTBD must be NULL on pre-10.3
{#endif}

{#if MAC_OS_X_VERSION_10_2 <= MAC_OS_X_VERSION_MAX_ALLOWED}
{!
	@typedef CFStringNormalizationForm
	This is the type of Unicode normalization forms as described in
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	Unicode Technical Report #15. To normalize for use with file
	system calls, use CFStringGetFileSystemRepresentation().
}
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CFStringNormalizationForm = CFIndex;
=======
	CFStringNormalizationForm = SIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	CFStringNormalizationForm = SIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	CFStringNormalizationForm = SIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	CFStringNormalizationForm = SIGNEDLONG;
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
	Unicode Technical Report #15.
}
type
	CFStringNormalizationForm = SInt32;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	Unicode Technical Report #15. To normalize for use with file
	system calls, use CFStringGetFileSystemRepresentation().
}
type
	CFStringNormalizationForm = SIGNEDLONG;
>>>>>>> origin/fixes_2.4
const
	kCFStringNormalizationFormD = 0; // Canonical Decomposition
	kCFStringNormalizationFormKD = 1; // Compatibility Decomposition
	kCFStringNormalizationFormC = 2; // Canonical Decomposition followed by Canonical Composition
	kCFStringNormalizationFormKC = 3; // Compatibility Decomposition followed by Canonical Composition

{!
	@function CFStringNormalize
	Normalizes the string into the specified form as described in
	Unicode Technical Report #15.
	@param theString  The string which is to be normalized.  If this
		parameter is not a valid mutable CFString, the behavior is
		undefined.
	@param theForm  The form into which the string is to be normalized.
		If this parameter is not a valid CFStringNormalizationForm value,
		the behavior is undefined.
}
procedure CFStringNormalize( theString: CFMutableStringRef; theForm: CFStringNormalizationForm ); external name '_CFStringNormalize';
{#endif}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{#if MAC_OS_X_VERSION_10_5 <= MAC_OS_X_VERSION_MAX_ALLOWED}
{!
	@function CFStringFold
	Folds the string into the form specified by the flags.
		Character foldings are operations that convert any of a set of characters
		sharing similar semantics into a single representative from that set.
		This function can be used to preprocess strings that are to be compared,
		searched, or indexed.
		Note that folding does not include normalization, so it is necessary
		to use CFStringNormalize in addition to CFStringFold in order to obtain
		the effect of kCFCompareNonliteral.
	@param theString  The string which is to be folded.  If this parameter is not
		a valid mutable CFString, the behavior is undefined.
	@param theFlag  The equivalency flags which describes the character folding form.
		Only those flags containing the word "insensitive" are recognized here; other flags are ignored.		
		Folding with kCFCompareCaseInsensitive removes case distinctions in accordance with the mapping
		specified by ftp://ftp.unicode.org/Public/UNIDATA/CaseFolding.txt.  Folding with
		kCFCompareDiacriticInsensitive removes distinctions of accents and other diacritics.  Folding
		with kCFCompareWidthInsensitive removes character width distinctions by mapping characters in
		the range U+FF00-U+FFEF to their ordinary equivalents.
	@param theLocale The locale tailoring the character folding behavior. If NULL,
		it's considered to be the system locale returned from CFLocaleGetSystem().
		If non-NULL and not a valid CFLocale object, the behavior is undefined.
}

procedure CFStringFold( theString: CFMutableStringRef; theFlags: CFOptionFlags; theLocale: CFLocaleRef ); external name '_CFStringFold';
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
{#endif MAC_OS_X_VERSION_10_5 <= MAC_OS_X_VERSION_MAX_ALLOWED }


{ Perform string transliteration.  The transformation represented by transform is applied to the given range of string, modifying it in place. Only the specified range will be modified, but the transform may look at portions of the string outside that range for context. NULL range pointer causes the whole string to be transformed. On return, range is modified to reflect the new range corresponding to the original range. reverse indicates that the inverse transform should be used instead, if it exists. If the transform is successful, true is returned; if unsuccessful, false. Reasons for the transform being unsuccessful include an invalid transform identifier, or attempting to reverse an irreversible transform.

You can pass one of the predefined transforms below, or any valid ICU transform ID as defined in the ICU User Guide. Note that we do not support arbitrary set of ICU transform rules.
<<<<<<< HEAD
=======
{ Perform string transliteration.  The transformation represented by transform (see below for the full list of transforms supported) is applied to the given range of string, modifying it in place. Only the specified range will be modified, but the transform may look at portions of the string outside that range for context. NULL range pointer causes the whole string to be transformed. On return, range is modified to reflect the new range corresponding to the original range. reverse indicates that the inverse transform should be used instead, if it exists. If the transform is successful, true is returned; if unsuccessful, false. Reasons for the transform being unsuccessful include an invalid transform identifier, or attempting to reverse an irreversible transform.
>>>>>>> graemeg/fixes_2_2
=======
{ Perform string transliteration.  The transformation represented by transform (see below for the full list of transforms supported) is applied to the given range of string, modifying it in place. Only the specified range will be modified, but the transform may look at portions of the string outside that range for context. NULL range pointer causes the whole string to be transformed. On return, range is modified to reflect the new range corresponding to the original range. reverse indicates that the inverse transform should be used instead, if it exists. If the transform is successful, true is returned; if unsuccessful, false. Reasons for the transform being unsuccessful include an invalid transform identifier, or attempting to reverse an irreversible transform.
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
}
function CFStringTransform( strng: CFMutableStringRef; var range: CFRange; transform: CFStringRef; reverse: Boolean ): Boolean; external name '_CFStringTransform';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{ Transform identifiers for CFStringTransform()
}
var kCFStringTransformStripCombiningMarks: CFStringRef; external name '_kCFStringTransformStripCombiningMarks'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kCFStringTransformToLatin: CFStringRef; external name '_kCFStringTransformToLatin'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kCFStringTransformFullwidthHalfwidth: CFStringRef; external name '_kCFStringTransformFullwidthHalfwidth'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kCFStringTransformLatinKatakana: CFStringRef; external name '_kCFStringTransformLatinKatakana'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kCFStringTransformLatinHiragana: CFStringRef; external name '_kCFStringTransformLatinHiragana'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kCFStringTransformHiraganaKatakana: CFStringRef; external name '_kCFStringTransformHiraganaKatakana'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kCFStringTransformMandarinLatin: CFStringRef; external name '_kCFStringTransformMandarinLatin'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kCFStringTransformLatinHangul: CFStringRef; external name '_kCFStringTransformLatinHangul'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kCFStringTransformLatinArabic: CFStringRef; external name '_kCFStringTransformLatinArabic'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kCFStringTransformLatinHebrew: CFStringRef; external name '_kCFStringTransformLatinHebrew'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kCFStringTransformLatinThai: CFStringRef; external name '_kCFStringTransformLatinThai'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kCFStringTransformLatinCyrillic: CFStringRef; external name '_kCFStringTransformLatinCyrillic'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kCFStringTransformLatinGreek: CFStringRef; external name '_kCFStringTransformLatinGreek'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kCFStringTransformToXMLHex: CFStringRef; external name '_kCFStringTransformToXMLHex'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
var kCFStringTransformToUnicodeName: CFStringRef; external name '_kCFStringTransformToUnicodeName'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
var kCFStringTransformStripDiacritics: CFStringRef; external name '_kCFStringTransformStripDiacritics'; (* attribute const *)
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
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
var kCFStringTransformStripDiacritics: CFStringRef; external name '_kCFStringTransformStripDiacritics'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{** General encoding related functionality **}

{ This returns availability of the encoding on the system
}
function CFStringIsEncodingAvailable( encoding: CFStringEncoding ): Boolean; external name '_CFStringIsEncodingAvailable';

{ This function returns list of available encodings.  The returned list is terminated with kCFStringEncodingInvalidId and owned by the system.
}
function CFStringGetListOfAvailableEncodings: CFStringEncodingPtr; external name '_CFStringGetListOfAvailableEncodings';

{ Returns name of the encoding; non-localized.
}
function CFStringGetNameOfEncoding( encoding: CFStringEncoding ): CFStringRef; external name '_CFStringGetNameOfEncoding';

{ ID mapping functions from/to Cocoa NSStringEncoding.  Returns kCFStringEncodingInvalidId if no mapping exists.
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function CFStringConvertEncodingToNSStringEncoding( encoding: CFStringEncoding ): UNSIGNEDLONG; external name '_CFStringConvertEncodingToNSStringEncoding';

function CFStringConvertNSStringEncodingToEncoding( encoding: UNSIGNEDLONG ): CFStringEncoding; external name '_CFStringConvertNSStringEncodingToEncoding';
=======
function CFStringConvertEncodingToNSStringEncoding( encoding: CFStringEncoding ): UInt32; external name '_CFStringConvertEncodingToNSStringEncoding';

function CFStringConvertNSStringEncodingToEncoding( encoding: UInt32 ): CFStringEncoding; external name '_CFStringConvertNSStringEncodingToEncoding';
>>>>>>> graemeg/fixes_2_2
=======
function CFStringConvertEncodingToNSStringEncoding( encoding: CFStringEncoding ): UInt32; external name '_CFStringConvertEncodingToNSStringEncoding';

function CFStringConvertNSStringEncodingToEncoding( encoding: UInt32 ): CFStringEncoding; external name '_CFStringConvertNSStringEncodingToEncoding';
>>>>>>> origin/fixes_2_2
=======
function CFStringConvertEncodingToNSStringEncoding( encoding: CFStringEncoding ): UNSIGNEDLONG; external name '_CFStringConvertEncodingToNSStringEncoding';

function CFStringConvertNSStringEncodingToEncoding( encoding: UNSIGNEDLONG ): CFStringEncoding; external name '_CFStringConvertNSStringEncodingToEncoding';
>>>>>>> origin/fixes_2.4

{ ID mapping functions from/to Microsoft Windows codepage (covers both OEM & ANSI).  Returns kCFStringEncodingInvalidId if no mapping exists.
}
function CFStringConvertEncodingToWindowsCodepage( encoding: CFStringEncoding ): UInt32; external name '_CFStringConvertEncodingToWindowsCodepage';

function CFStringConvertWindowsCodepageToEncoding( codepage: UInt32 ): CFStringEncoding; external name '_CFStringConvertWindowsCodepageToEncoding';

{ ID mapping functions from/to IANA registery charset names.  Returns kCFStringEncodingInvalidId if no mapping exists.
}
function CFStringConvertIANACharSetNameToEncoding( theString: CFStringRef ): CFStringEncoding; external name '_CFStringConvertIANACharSetNameToEncoding';

function CFStringConvertEncodingToIANACharSetName( encoding: CFStringEncoding ): CFStringRef; external name '_CFStringConvertEncodingToIANACharSetName';

{ Returns the most compatible MacOS script value for the input encoding }
{ i.e. kCFStringEncodingMacRoman -> kCFStringEncodingMacRoman }
{	kCFStringEncodingWindowsLatin1 -> kCFStringEncodingMacRoman }
{	kCFStringEncodingISO_2022_JP -> kCFStringEncodingMacJapanese }
function CFStringGetMostCompatibleMacStringEncoding( encoding: CFStringEncoding ): CFStringEncoding; external name '_CFStringGetMostCompatibleMacStringEncoding';


{ The next two functions allow fast access to the contents of a string, 
   assuming you are doing sequential or localized accesses. To use, call
   CFStringInitInlineBuffer() with a CFStringInlineBuffer (on the stack, say),
   and a range in the string to look at. Then call CFStringGetCharacterFromInlineBuffer()
   as many times as you want, with a index into that range (relative to the start
   of that range). These are INLINE functions and will end up calling CFString only 
   once in a while, to fill a buffer.  CFStringGetCharacterFromInlineBuffer() returns 0 if
   a location outside the original range is specified.
}
const
	__kCFStringInlineBufferLength = 64;
type
	CFStringInlineBuffer = record
		buffer: array[0..__kCFStringInlineBufferLength-1] of UniChar;
		theString: CFStringRef;
		directBuffer: UniCharPtr;
		rangeToBuffer: CFRange;		{ Range in string to buffer }
		bufferedRangeStart: CFIndex;		{ Start of range currently buffered (relative to rangeToBuffer.location) }
		bufferedRangeEnd: CFIndex;		{ bufferedRangeStart + number of chars actually buffered }
	end;

// Not currently converted to Pascal
// #if defined(CF_INLINE)
// CF_INLINE void CFStringInitInlineBuffer(CFStringRef str, CFStringInlineBuffer *buf, CFRange range) {
//     buf->theString = str;
//     buf->rangeToBuffer = range;
//     buf->directBuffer = CFStringGetCharactersPtr(str);
//     buf->bufferedRangeStart = buf->bufferedRangeEnd = 0;
// }
// 
// CF_INLINE UniChar CFStringGetCharacterFromInlineBuffer(CFStringInlineBuffer *buf, CFIndex idx) {
//     if (buf->directBuffer) {
// 	if (idx < 0 || idx >= buf->rangeToBuffer.length) return 0;
//         return buf->directBuffer[idx + buf->rangeToBuffer.location];
//     }
//     if (idx >= buf->bufferedRangeEnd || idx < buf->bufferedRangeStart) {
// 	if (idx < 0 || idx >= buf->rangeToBuffer.length) return 0;
// 	if ((buf->bufferedRangeStart = idx - 4) < 0) buf->bufferedRangeStart = 0;
// 	buf->bufferedRangeEnd = buf->bufferedRangeStart + __kCFStringInlineBufferLength;
// 	if (buf->bufferedRangeEnd > buf->rangeToBuffer.length) buf->bufferedRangeEnd = buf->rangeToBuffer.length;
// 	CFStringGetCharacters(buf->theString, CFRangeMake(buf->rangeToBuffer.location + buf->bufferedRangeStart, buf->bufferedRangeEnd - buf->bufferedRangeStart), buf->buffer);
//     }
//     return buf->buffer[idx - buf->bufferedRangeStart];
// }
// 
// #else
// { If INLINE functions are not available, we do somewhat less powerful macros that work similarly (except be aware that the buf argument is evaluated multiple times).
// }
// #define CFStringInitInlineBuffer(str, buf, range) \
//     do {(buf)->theString = str; (buf)->rangeToBuffer = range; (buf)->directBuffer = CFStringGetCharactersPtr(str);} while (0)
// 
// #define CFStringGetCharacterFromInlineBuffer(buf, idx) \
//     (((idx) < 0 || (idx) >= (buf)->rangeToBuffer.length) ? 0 : ((buf)->directBuffer ? (buf)->directBuffer[(idx) + (buf)->rangeToBuffer.location] : CFStringGetCharacterAtIndex((buf)->theString, (idx) + (buf)->rangeToBuffer.location)))
// 
// #endif { CF_INLINE }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
// 
// { UTF-16 surrogate support
//  }
// CF_INLINE Boolean CFStringIsSurrogateHighCharacter(UniChar character) {
//     return ((character >= 0xD800UL) && (character <= 0xDBFFUL) ? true : false);
// }
// 
// CF_INLINE Boolean CFStringIsSurrogateLowCharacter(UniChar character) {
//     return ((character >= 0xDC00UL) && (character <= 0xDFFFUL) ? true : false);
// }
// 
// CF_INLINE UTF32Char CFStringGetLongCharacterForSurrogatePair(UniChar surrogateHigh, UniChar surrogateLow) {
//     return ((surrogateHigh - 0xD800UL) << 10) + (surrogateLow - 0xDC00UL) + 0x0010000UL;
// }
// 
// // Maps a UTF-32 character to a pair of UTF-16 surrogate characters. The buffer pointed by surrogates has to have space for at least 2 UTF-16 characters. Returns true if mapped to a surrogate pair.
// CF_INLINE Boolean CFStringGetSurrogatePairForLongCharacter(UTF32Char character, UniChar *surrogates) {
//     if ((character > 0xFFFFUL) && (character < 0x110000UL)) { // Non-BMP character
//         character -= 0x10000;
//         if (NULL != surrogates) {
//             surrogates[0] = (UniChar)((character >> 10) + 0xD800UL);
//             surrogates[1] = (UniChar)((character & 0x3FF) + 0xDC00UL);
//         }
//         return true;
//     } else {
//         if (NULL != surrogates) *surrogates = (UniChar)character;
//         return false;
//     }
// }
<<<<<<< HEAD

{ Rest of the stuff in this file is private and should not be used directly
}
{ For debugging only; output goes to stderr
=======
=======
>>>>>>> origin/fixes_2_2

{ Rest of the stuff in this file is private and should not be used directly
}
{ For debugging only
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

{ Rest of the stuff in this file is private and should not be used directly
}
{ For debugging only; output goes to stderr
>>>>>>> origin/fixes_2.4
   Use CFShow() to printf the description of any CFType;
   Use CFShowStr() to printf detailed info about a CFString
}
procedure CFShow( obj: CFTypeRef ); external name '_CFShow';

procedure CFShowStr( str: CFStringRef ); external name '_CFShowStr';

{ This function is private and should not be used directly }
function __CFStringMakeConstantString( cStr: ConstCStringPtr ): CFStringRef; external name '___CFStringMakeConstantString';	{ Private; do not use }


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4
implementation


end.
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
