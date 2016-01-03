{	CFURL.h
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
	Copyright (c) 1998-2009, Apple Inc. All rights reserved.
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
	Copyright (c) 1998-2009, Apple Inc. All rights reserved.
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

unit CFURL;
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
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
uses MacTypes,CFBase,CFArray,CFData,CFDictionary,CFError,CFString,Files;
=======
uses MacTypes,CFBase,CFData,CFError,CFString,Files;
>>>>>>> graemeg/cpstrnew
=======
uses MacTypes,CFBase,CFData,CFError,CFString,Files;
>>>>>>> graemeg/cpstrnew
=======
uses MacTypes,CFBase,CFData,CFError,CFString,Files;
>>>>>>> graemeg/cpstrnew
=======
uses MacTypes,CFBase,CFData,CFError,CFString,Files;
>>>>>>> origin/cpstrnew
{$endc} {not MACOSALLINCLUDE}

=======
uses MacTypes,CFBase,CFData,CFString,Files;
>>>>>>> graemeg/fixes_2_2
=======
uses MacTypes,CFBase,CFData,CFString,Files;
>>>>>>> origin/fixes_2_2
=======
uses MacTypes,CFBase,CFData,CFError,CFString,Files;
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
	CFURLPathStyle = CFIndex;
=======
	CFURLPathStyle = SIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	CFURLPathStyle = SIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	CFURLPathStyle = SIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	CFURLPathStyle = SIGNEDLONG;
>>>>>>> origin/cpstrnew
const
	kCFURLPOSIXPathStyle = 0;
	kCFURLHFSPathStyle = 1; { The use of kCFURLHFSPathStyle is deprecated. The Carbon File Manager, which uses HFS style paths, is deprecated. HFS style paths are unreliable because they can arbitrarily refer to multiple volumes if those volumes have identical volume names. You should instead use kCFURLPOSIXPathStyle wherever possible. }
	kCFURLWindowsPathStyle = 2;

{ CFURLRef moved to CFBase to avoid circular dependency with Files unit }    
=======
=======
>>>>>>> origin/fixes_2_2
	CFURLPathStyle = SInt32;
=======
	CFURLPathStyle = SIGNEDLONG;
>>>>>>> origin/fixes_2.4
const
	kCFURLPOSIXPathStyle = 0;
	kCFURLHFSPathStyle = 1;
	kCFURLWindowsPathStyle = 2;
<<<<<<< HEAD
    
type
	CFURLRef = ^SInt32; { an opaque 32-bit type }
	CFURLRefPtr = ^CFURLRef;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

{ CFURLRef moved to CFBase to avoid circular dependency with Files unit }    
>>>>>>> origin/fixes_2.4

{ CFURLs are composed of two fundamental pieces - their string, and a }
{ (possibly NULL) base URL.  A relative URL is one in which the string }
{ by itself does not fully specify the URL (for instance "myDir/image.tiff"); }
{ an absolute URL is one in which the string does fully specify the URL }
{ ("file://localhost/myDir/image.tiff").  Absolute URLs always have NULL }
{ base URLs; however, it is possible for a URL to have a NULL base, and still }
{ not be absolute.  Such a URL has only a relative string, and cannot be }
{ resolved.  Two CFURLs are considered equal if and only if their strings }
{ are equal and their bases are equal.  In other words, }
{ "file://localhost/myDir/image.tiff" is NOT equal to the URL with relative }
{ string "myDir/image.tiff" and base URL "file://localhost/".  Clients that }
{ need these less strict form of equality should convert all URLs to their }
{ absolute form via CFURLCopyAbsoluteURL(), then compare the absolute forms. }

function CFURLGetTypeID: CFTypeID; external name '_CFURLGetTypeID';

{ encoding will be used both to interpret the bytes of URLBytes, and to }
{ interpret any percent-escapes within the bytes. }
<<<<<<< HEAD
<<<<<<< HEAD
{ Using a string encoding which isn't a superset of ASCII encoding is not }
{ supported because CFURLGetBytes and CFURLGetByteRangeForComponent require }
{ 7-bit ASCII characters to be stored in a single 8-bit byte. }
{ CFStringEncodings which are a superset of ASCII encoding include MacRoman, }
{ WindowsLatin1, ISOLatin1, NextStepLatin, ASCII, and UTF8. }
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function CFURLCreateWithBytes( allocator: CFAllocatorRef; URLBytes: UInt8Ptr; length: CFIndex; encoding: CFStringEncoding; baseURL: CFURLRef ): CFURLRef; external name '_CFURLCreateWithBytes';

{ Escapes any character that is not 7-bit ASCII with the byte-code }
{ for the given encoding.  If escapeWhitespace is true, whitespace }
{ characters (' ', '\t', '\r', '\n') will be escaped also (desirable }
{ if embedding the URL into a larger text stream like HTML) }
function CFURLCreateData( allocator: CFAllocatorRef; url: CFURLRef; encoding: CFStringEncoding; escapeWhitespace: Boolean ): CFDataRef; external name '_CFURLCreateData';

{ Any escape sequences in URLString will be interpreted via UTF-8. }
function CFURLCreateWithString( allocator: CFAllocatorRef; URLString: CFStringRef; baseURL: CFURLRef ): CFURLRef; external name '_CFURLCreateWithString';

{#if MAC_OS_X_VERSION_10_3 <= MAC_OS_X_VERSION_MAX_ALLOWED}

{ Create an absolute URL directly, without requiring the extra step }
{ of calling CFURLCopyAbsoluteURL().  If useCompatibilityMode is  }
{ true, the rules historically used on the web are used to resolve }
{ relativeString against baseURL - these rules are generally listed }
{ in the RFC as optional or alternate interpretations.  Otherwise, }
{ the strict rules from the RFC are used.  The major differences are }
{ that in compatibility mode, we are lenient of the scheme appearing }
{ in relative portion, leading "../" components are removed from the }
{ final URL's path, and if the relative portion contains only }
{ resource specifier pieces (query, parameters, and fragment), then }
<<<<<<< HEAD
<<<<<<< HEAD
{ the last path component of the base URL will not be deleted.  }
{ Using a string encoding which isn't a superset of ASCII encoding is not }
{ supported because CFURLGetBytes and CFURLGetByteRangeForComponent require }
{ 7-bit ASCII characters to be stored in a single 8-bit byte. }
{ CFStringEncodings which are a superset of ASCII encoding include MacRoman, }
{ WindowsLatin1, ISOLatin1, NextStepLatin, ASCII, and UTF8. }
=======
{ the last path component of the base URL will not be deleted  }
>>>>>>> graemeg/fixes_2_2
=======
{ the last path component of the base URL will not be deleted  }
>>>>>>> origin/fixes_2_2
function CFURLCreateAbsoluteURLWithBytes( alloc: CFAllocatorRef; relativeURLBytes: UInt8Ptr; length: CFIndex; encoding: CFStringEncoding; baseURL: CFURLRef; useCompatibilityMode: Boolean ): CFURLRef; external name '_CFURLCreateAbsoluteURLWithBytes';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
{#endif}

{ filePath should be the URL's path expressed as a path of the type }
{ fsType.  If filePath is not absolute, the resulting URL will be }
{ considered relative to the current working directory (evaluated }
{ at creation time).  isDirectory determines whether filePath is }
{ treated as a directory path when resolving against relative path }
{ components }
function CFURLCreateWithFileSystemPath( allocator: CFAllocatorRef; filePath: CFStringRef; pathStyle: CFURLPathStyle; isDirectory: Boolean ): CFURLRef; external name '_CFURLCreateWithFileSystemPath';

function CFURLCreateFromFileSystemRepresentation( allocator: CFAllocatorRef; buffer: CStringPtr; bufLen: CFIndex; isDirectory: Boolean ): CFURLRef; external name '_CFURLCreateFromFileSystemRepresentation';

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{ The path style of the baseURL must match the path style of the relative }
{ url or the results are undefined.  If the provided filePath looks like an }
{ absolute path ( starting with '/' if pathStyle is kCFURLPosixPathStyle, }
{ not starting with ':' for kCFURLHFSPathStyle, or starting with what looks }
{ like a drive letter and colon for kCFURLWindowsPathStyle ) then the baseURL }
{ is ignored. }
<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
function CFURLCreateWithFileSystemPathRelativeToBase( allocator: CFAllocatorRef; filePath: CFStringRef; pathStyle: CFURLPathStyle; isDirectory: Boolean; baseURL: CFURLRef ): CFURLRef; external name '_CFURLCreateWithFileSystemPathRelativeToBase';

function CFURLCreateFromFileSystemRepresentationRelativeToBase( allocator: CFAllocatorRef; buffer: CStringPtr; bufLen: CFIndex; isDirectory: Boolean; baseURL: CFURLRef ): CFURLRef; external name '_CFURLCreateFromFileSystemRepresentationRelativeToBase';
                                                                         
{ Fills buffer with the file system's native representation of }
{ url's path. No more than maxBufLen bytes are written to buffer. }
{ The buffer should be at least the maximum path length for }
{ the file system in question to avoid failures for insufficiently }
{ large buffers.  If resolveAgainstBase is true, the url's relative }
{ portion is resolved against its base before the path is computed. }
{ Returns success or failure. }
function CFURLGetFileSystemRepresentation( url: CFURLRef; resolveAgainstBase: Boolean; buffer: CStringPtr; maxBufLen: CFIndex ): Boolean; external name '_CFURLGetFileSystemRepresentation';

{ Creates a new URL by resolving the relative portion of relativeURL against its base. }
function CFURLCopyAbsoluteURL( relativeURL: CFURLRef ): CFURLRef; external name '_CFURLCopyAbsoluteURL';

{ Returns the URL's string. }
function CFURLGetString( anURL: CFURLRef ): CFStringRef; external name '_CFURLGetString';

{ Returns the base URL if it exists }
function CFURLGetBaseURL( anURL: CFURLRef ): CFURLRef; external name '_CFURLGetBaseURL';

{
All URLs can be broken into two pieces - the scheme (preceding the
first colon) and the resource specifier (following the first colon).
Most URLs are also "standard" URLs conforming to RFC 1808 (available
from www.w3c.org).  This category includes URLs of the file, http,
https, and ftp schemes, to name a few.  Standard URLs start the
resource specifier with two slashes ("//"), and can be broken into
four distinct pieces - the scheme, the net location, the path, and
further resource specifiers (typically an optional parameter, query,
and/or fragment).  The net location appears immediately following
the two slashes and goes up to the next slash; it's format is
scheme-specific, but is usually composed of some or all of a username,
password, host name, and port.  The path is a series of path components
separated by slashes; if the net location is present, the path always
begins with a slash.  Standard URLs can be relative to another URL,
in which case at least the scheme and possibly other pieces as well
come from the base URL (see RFC 1808 for precise details when resolving
a relative URL against its base).  The full URL is therefore

<scheme> "://" <net location> <path, always starting with slash> <add'l resource specifiers>

If a given CFURL can be decomposed (that is, conforms to RFC 1808), you
can ask for each of the four basic pieces (scheme, net location, path,
and resource specifer) separately, as well as for its base URL.  The
basic pieces are returned with any percent escape sequences still in
place (although note that the scheme may not legally include any
percent escapes); this is to allow the caller to distinguish between
percent sequences that may have syntactic meaning if replaced by the
character being escaped (for instance, a '/' in a path component).
Since only the individual schemes know which characters are
syntactically significant, CFURL cannot safely replace any percent
escape sequences.  However, you can use
CFURLCreateStringByReplacingPercentEscapes() to create a new string with
the percent escapes removed; see below.

If a given CFURL can not be decomposed, you can ask for its scheme and its
resource specifier; asking it for its net location or path will return NULL.

To get more refined information about the components of a decomposable
CFURL, you may ask for more specific pieces of the URL, expressed with
the percent escapes removed.  The available functions are CFURLCopyHostName(),
CFURLGetPortNumber() (returns an Int32), CFURLCopyUserName(),
CFURLCopyPassword(), CFURLCopyQuery(), CFURLCopyParameters(), and
CFURLCopyFragment().  Because the parameters, query, and fragment of an
URL may contain scheme-specific syntaxes, these methods take a second
argument, giving a list of characters which should NOT be replaced if
percent escaped.  For instance, the ftp parameter syntax gives simple
key-value pairs as "<key>=<value>;"  Clearly if a key or value includes
either '=' or ';', it must be escaped to avoid corrupting the meaning of
the parameters, so the caller may request the parameter string as

CFStringRef myParams = CFURLCopyParameters(ftpURL, CFSTR("=;%"));

requesting that all percent escape sequences be replaced by the represented
characters, except for escaped '=', '%' or ';' characters.  Pass the empty
string (CFSTR("")) to request that all percent escapes be replaced, or NULL
to request that none be.
}

{ Returns true if anURL conforms to RFC 1808 }
<<<<<<< HEAD
<<<<<<< HEAD
function CFURLCanBeDecomposed( anURL: CFURLRef ): Boolean; external name '_CFURLCanBeDecomposed'; 
=======
function CFURLCanBeDecomposed( anURL: CFURLRef ): Boolean; external name '_CFURLCanBeDecomposed';
>>>>>>> graemeg/fixes_2_2
=======
function CFURLCanBeDecomposed( anURL: CFURLRef ): Boolean; external name '_CFURLCanBeDecomposed';
>>>>>>> origin/fixes_2_2

{ The next several methods leave any percent escape sequences intact }

function CFURLCopyScheme( anURL: CFURLRef ): CFStringRef; external name '_CFURLCopyScheme';

{ NULL if CFURLCanBeDecomposed(anURL) is false }
<<<<<<< HEAD
<<<<<<< HEAD
function CFURLCopyNetLocation( anURL: CFURLRef ): CFStringRef; external name '_CFURLCopyNetLocation'; 
=======
function CFURLCopyNetLocation( anURL: CFURLRef ): CFStringRef; external name '_CFURLCopyNetLocation';
>>>>>>> graemeg/fixes_2_2
=======
function CFURLCopyNetLocation( anURL: CFURLRef ): CFStringRef; external name '_CFURLCopyNetLocation';
>>>>>>> origin/fixes_2_2

{ NULL if CFURLCanBeDecomposed(anURL) is false; also does not resolve the URL }
{ against its base.  See also CFURLCopyAbsoluteURL().  Note that, strictly }
{ speaking, any leading '/' is not considered part of the URL's path, although }
{ its presence or absence determines whether the path is absolute. }
{ CFURLCopyPath()'s return value includes any leading slash (giving the path }
{ the normal POSIX appearance); CFURLCopyStrictPath()'s return value omits any }
{ leading slash, and uses isAbsolute to report whether the URL's path is absolute. }

{ CFURLCopyFileSystemPath() returns the URL's path as a file system path for the }
{ given path style.  All percent escape sequences are replaced.  The URL is not }
{ resolved against its base before computing the path. }
function CFURLCopyPath( anURL: CFURLRef ): CFStringRef; external name '_CFURLCopyPath';

function CFURLCopyStrictPath( anURL: CFURLRef; var isAbsolute: Boolean ): CFStringRef; external name '_CFURLCopyStrictPath';

function CFURLCopyFileSystemPath( anURL: CFURLRef; pathStyle: CFURLPathStyle ): CFStringRef; external name '_CFURLCopyFileSystemPath';

{ Returns whether anURL's path represents a directory }
{ (true returned) or a simple file (false returned) }
function CFURLHasDirectoryPath( anURL: CFURLRef ): Boolean; external name '_CFURLHasDirectoryPath';

{ Any additional resource specifiers after the path.  For URLs }
{ that cannot be decomposed, this is everything except the scheme itself. }
<<<<<<< HEAD
<<<<<<< HEAD
function CFURLCopyResourceSpecifier( anURL: CFURLRef ): CFStringRef; external name '_CFURLCopyResourceSpecifier'; 
=======
function CFURLCopyResourceSpecifier( anURL: CFURLRef ): CFStringRef; external name '_CFURLCopyResourceSpecifier';
>>>>>>> graemeg/fixes_2_2
=======
function CFURLCopyResourceSpecifier( anURL: CFURLRef ): CFStringRef; external name '_CFURLCopyResourceSpecifier';
>>>>>>> origin/fixes_2_2

function CFURLCopyHostName( anURL: CFURLRef ): CFStringRef; external name '_CFURLCopyHostName';

function CFURLGetPortNumber( anURL: CFURLRef ): SInt32; external name '_CFURLGetPortNumber'; { Returns -1 if no port number is specified }

function CFURLCopyUserName( anURL: CFURLRef ): CFStringRef; external name '_CFURLCopyUserName';

function CFURLCopyPassword( anURL: CFURLRef ): CFStringRef; external name '_CFURLCopyPassword';

{ These remove all percent escape sequences except those for }
{ characters in charactersToLeaveEscaped.  If charactersToLeaveEscaped }
{ is empty (""), all percent escape sequences are replaced by their }
{ corresponding characters.  If charactersToLeaveEscaped is NULL, }
{ then no escape sequences are removed at all }
function CFURLCopyParameterString( anURL: CFURLRef; charactersToLeaveEscaped: CFStringRef ): CFStringRef; external name '_CFURLCopyParameterString';

function CFURLCopyQueryString( anURL: CFURLRef; charactersToLeaveEscaped: CFStringRef ): CFStringRef; external name '_CFURLCopyQueryString';

function CFURLCopyFragment( anURL: CFURLRef; charactersToLeaveEscaped: CFStringRef ): CFStringRef; external name '_CFURLCopyFragment';

function CFURLCopyLastPathComponent( url: CFURLRef ): CFStringRef; external name '_CFURLCopyLastPathComponent';

function CFURLCopyPathExtension( url: CFURLRef ): CFStringRef; external name '_CFURLCopyPathExtension';

{ These functions all treat the base URL of the supplied url as }
{ invariant.  In other words, the URL returned will always have }
{ the same base as the URL supplied as an argument. }

function CFURLCreateCopyAppendingPathComponent( allocator: CFAllocatorRef; url: CFURLRef; pathComponent: CFStringRef; isDirectory: Boolean ): CFURLRef; external name '_CFURLCreateCopyAppendingPathComponent';

function CFURLCreateCopyDeletingLastPathComponent( allocator: CFAllocatorRef; url: CFURLRef ): CFURLRef; external name '_CFURLCreateCopyDeletingLastPathComponent';

function CFURLCreateCopyAppendingPathExtension( allocator: CFAllocatorRef; url: CFURLRef; extension: CFStringRef ): CFURLRef; external name '_CFURLCreateCopyAppendingPathExtension';

function CFURLCreateCopyDeletingPathExtension( allocator: CFAllocatorRef; url: CFURLRef ): CFURLRef; external name '_CFURLCreateCopyDeletingPathExtension';

{#if MAC_OS_X_VERSION_10_3 <= MAC_OS_X_VERSION_MAX_ALLOWED}
{ Fills buffer with the bytes for url, returning the number of bytes }
{ filled.  If buffer is of insufficient size, returns -1 and no bytes }
{ are placed in buffer.  If buffer is NULL, the needed length is }
{ computed and returned.  The returned bytes are the original bytes } 
{ from which the URL was created; if the URL was created from a }
{ string, the bytes will be the bytes of the string encoded via UTF-8  }
function CFURLGetBytes( url: CFURLRef; buffer: CStringPtr; bufferLength: CFIndex ): CFIndex; external name '_CFURLGetBytes';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CFURLComponentType = CFIndex;
=======
	CFURLComponentType = SIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	CFURLComponentType = SIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	CFURLComponentType = SIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	CFURLComponentType = SIGNEDLONG;
>>>>>>> origin/cpstrnew
=======
	CFURLComponentType = SInt32;
>>>>>>> graemeg/fixes_2_2
=======
	CFURLComponentType = SInt32;
>>>>>>> origin/fixes_2_2
=======
	CFURLComponentType = SIGNEDLONG;
>>>>>>> origin/fixes_2.4
const
	kCFURLComponentScheme = 1;
	kCFURLComponentNetLocation = 2;
	kCFURLComponentPath = 3;
	kCFURLComponentResourceSpecifier = 4;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
	
>>>>>>> graemeg/fixes_2_2
=======
	
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	kCFURLComponentUser = 5;
	kCFURLComponentPassword = 6;
	kCFURLComponentUserInfo = 7;
	kCFURLComponentHost = 8;
	kCFURLComponentPort = 9;
	kCFURLComponentParameterString = 10;
	kCFURLComponentQuery = 11;
	kCFURLComponentFragment = 12;
 
{ 
Gets the  range of the requested component in the bytes of url, as
returned by CFURLGetBytes().  This range is only good for use in the
bytes returned by CFURLGetBytes!

If non-NULL, rangeIncludingSeparators gives the range of component
including the sequences that separate component from the previous and
next components.  If there is no previous or next component, that end of
rangeIncludingSeparators will match the range of the component itself.
If url does not contain the given component type, (kCFNotFound, 0) is
returned, and rangeIncludingSeparators is set to the location where the
component would be inserted.  Some examples -

For the URL http://www.apple.com/hotnews/

Component           returned range      rangeIncludingSeparators
scheme              (0, 4)              (0, 7)
net location        (7, 13)             (4, 16)
path                (20, 9)             (20, 9)    
resource specifier  (kCFNotFound, 0)    (29, 0)
user                (kCFNotFound, 0)    (7, 0)
password            (kCFNotFound, 0)    (7, 0)
user info           (kCFNotFound, 0)    (7, 0)
host                (7, 13)             (4, 16)
port                (kCFNotFound, 0)    (20, 0)
parameter           (kCFNotFound, 0)    (29, 0)
query               (kCFNotFound, 0)    (29, 0)
fragment            (kCFNotFound, 0)    (29, 0)


For the URL ./relPath/file.html#fragment

Component           returned range      rangeIncludingSeparators
scheme              (kCFNotFound, 0)    (0, 0)
net location        (kCFNotFound, 0)    (0, 0)
path                (0, 19)             (0, 20)
resource specifier  (20, 8)             (19, 9)
user                (kCFNotFound, 0)    (0, 0)
password            (kCFNotFound, 0)    (0, 0)
user info           (kCFNotFound, 0)    (0, 0)
host                (kCFNotFound, 0)    (0, 0)
port                (kCFNotFound, 0)    (0, 0)
parameter           (kCFNotFound, 0)    (19, 0)
query               (kCFNotFound, 0)    (19, 0)
fragment            (20, 8)             (19, 9)


For the URL scheme://user:pass@host:1/path/path2/file.html;params?query#fragment

Component           returned range      rangeIncludingSeparators
scheme              (0, 6)              (0, 9)
net location        (9, 16)             (6, 19)
path                (25, 21)            (25, 22) 
resource specifier  (47, 21)            (46, 22)
user                (9, 4)              (6, 8)
password            (14, 4)             (13, 6)
user info           (9, 9)              (6, 13)
host                (19, 4)             (18, 6)
port                (24, 1)             (23, 2)
parameter           (47, 6)             (46, 8)
query               (54, 5)             (53, 7)
fragment            (60, 8)             (59, 9)
}
function CFURLGetByteRangeForComponent( url: CFURLRef; component: CFURLComponentType; var rangeIncludingSeparators: CFRange ): CFRange; external name '_CFURLGetByteRangeForComponent';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
{#endif}

{ Returns a string with any percent escape sequences that do NOT }
{ correspond to characters in charactersToLeaveEscaped with their }
{ equivalent.  Returns NULL on failure (if an invalid percent sequence }
{ is encountered), or the original string (retained) if no characters }
{ need to be replaced. Pass NULL to request that no percent escapes be }
{ replaced, or the empty string (CFSTR("")) to request that all percent }
{ escapes be replaced.  Uses UTF8 to interpret percent escapes. }
function CFURLCreateStringByReplacingPercentEscapes( allocator: CFAllocatorRef; originalString: CFStringRef; charactersToLeaveEscaped: CFStringRef ): CFStringRef; external name '_CFURLCreateStringByReplacingPercentEscapes';

{#if MAC_OS_X_VERSION_10_3 <= MAC_OS_X_VERSION_MAX_ALLOWED}
{ As above, but allows you to specify the encoding to use when interpreting percent escapes }
function CFURLCreateStringByReplacingPercentEscapesUsingEncoding( allocator: CFAllocatorRef; origString: CFStringRef; charsToLeaveEscaped: CFStringRef; encoding: CFStringEncoding ): CFStringRef; external name '_CFURLCreateStringByReplacingPercentEscapesUsingEncoding';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
{#endif}

{ Creates a copy or originalString, replacing certain characters with }
{ the equivalent percent escape sequence based on the encoding specified. }
{ If the originalString does not need to be modified (no percent escape }
{ sequences are missing), may retain and return originalString. }
{ If you are uncertain of the correct encoding, you should use UTF-8, }
{ which is the encoding designated by RFC 2396 as the correct encoding }
{ for use in URLs.  The characters so escaped are all characters that }
{ are not legal URL characters (based on RFC 2396), plus any characters }
{ in legalURLCharactersToBeEscaped, less any characters in }
{ charactersToLeaveUnescaped.  To simply correct any non-URL characters }
{ in an otherwise correct URL string, do: }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ newString = CFURLCreateStringByAddingPercentEscapes(kCFAllocatorDefault, origString, NULL, NULL, kCFStringEncodingUTF8); }
function CFURLCreateStringByAddingPercentEscapes( allocator: CFAllocatorRef; originalString: CFStringRef; charactersToLeaveUnescaped: CFStringRef; legalURLCharactersToBeEscaped: CFStringRef; encoding: CFStringEncoding ): CFStringRef; external name '_CFURLCreateStringByAddingPercentEscapes';

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{ #if (TARGET_OS_MAC || TARGET_OS_EMBEDDED || TARGET_OS_IPHONE) || CF_BUILDING_CF || NSBUILDINGFOUNDATION }
{ CF_IMPLICIT_BRIDGING_DISABLED }

{
    CFURLIsFileReferenceURL

    Returns whether the URL is a file reference URL.

    Parameters
        url
            The URL specifying the resource.
 }
function CFURLIsFileReferenceURL( url: CFURLRef ): Boolean; external name '_CFURLIsFileReferenceURL';
(* CF_AVAILABLE_STARTING(10_9, 7_0) *)

{
    CFURLCreateFileReferenceURL
    
    Returns a new file reference URL that refers to the same resource as a specified URL.

    Parameters
        allocator
            The memory allocator for creating the new URL.
        url
            The file URL specifying the resource.
        error
            On output when the result is NULL, the error that occurred. This parameter is optional; if you do not wish the error returned, pass NULL here. The caller is responsible for releasing a valid output error.

    Return Value
        The new file reference URL, or NULL if an error occurs.

    Discussion
        File reference URLs use a URL path syntax that identifies a file system object by reference, not by path. This form of file URL remains valid when the file system path of the URL’s underlying resource changes. An error will occur if the url parameter is not a file URL. File reference URLs cannot be created to file system objects which do not exist or are not reachable. In some areas of the file system hierarchy, file reference URLs cannot be generated to the leaf node of the URL path. A file reference URL's path should never be persistently stored because is not valid across system restarts, and across remounts of volumes -- if you want to create a persistent reference to a file system object, use a bookmark (see CFURLCreateBookmarkData). If this function returns NULL, the optional error is populated. This function is currently applicable only to URLs for file system resources.
        Symbol is present in iOS 4, but performs no operation.
 }
function CFURLCreateFileReferenceURL( allocator: CFAllocatorRef; url: CFURLRef; var error: CFErrorRef ): CFURLRef; external name '_CFURLCreateFileReferenceURL';
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)


{
    CFURLCreateFilePathURL
    
    Returns a new file path URL that refers to the same resource as a specified URL.

    Parameters
        allocator
            The memory allocator for creating the new URL.
        url
            The file URL specifying the resource.
        error
            On output when the result is NULL, the error that occurred. This parameter is optional; if you do not wish the error returned, pass NULL here. The caller is responsible for releasing a valid output error.

    Return Value
        The new file path URL, or NULL if an error occurs.

    Discussion
        File path URLs use a file system style path. An error will occur if the url parameter is not a file URL. A file reference URL's resource must exist and be reachable to be converted to a file path URL. If this function returns NULL, the optional error is populated. This function is currently applicable only to URLs for file system resources.
        Symbol is present in iOS 4, but performs no operation.
 }
function CFURLCreateFilePathURL( allocator: CFAllocatorRef; url: CFURLRef; var error: CFErrorRef ): CFURLRef; external name '_CFURLCreateFilePathURL';
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)

{ CF_IMPLICIT_BRIDGING_ENABLED }
{#ifndef CF_OPEN_SOURCE}

// Note: CFURLCreateFromFSRef and CFURLGetFSRef have never been functional on iOS because the Carbon File Manager is not on iOS.
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{ Returns a file reference URL, a path-idependent form of file URL. }
{ Converts a file path URL if necessary. For non-file URLs, returns NULL. }
{ Also returns NULL when the conversion fails because the target resource doesn't exist. }
{ Optional output error: The error is set to a valid CFErrorRef when the function }
{ result is NULL. A valid output error must be released by the caller. }
function CFURLCreateFileReferenceURL( allocator: CFAllocatorRef; url: CFURLRef; var error: CFErrorRef ): CFURLRef; external name '_CFURLCreateFileReferenceURL';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)


{ Returns a file path URL, converting a file reference URL if necessary. }
{ For non-file URLs, returns NULL. Also returns NULL when the conversion fails }
{ because the target resource doesn't exist. }
{ Optional output error: The error is set to a valid CFErrorRef when the function }
{ result is NULL. A valid output error must be released by the caller. }
function CFURLCreateFilePathURL( allocator: CFAllocatorRef; url: CFURLRef; var error: CFErrorRef ): CFURLRef; external name '_CFURLCreateFilePathURL';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{#ifndef CF_OPEN_SOURCE}

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
{ Returns a file reference URL, a path-idependent form of file URL. }
{ Converts a file path URL if necessary. For non-file URLs, returns NULL. }
{ Also returns NULL when the conversion fails because the target resource doesn't exist. }
{ Optional output error: The error is set to a valid CFErrorRef when the function }
{ result is NULL. A valid output error must be released by the caller. }
function CFURLCreateFileReferenceURL( allocator: CFAllocatorRef; url: CFURLRef; var error: CFErrorRef ): CFURLRef; external name '_CFURLCreateFileReferenceURL';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)


{ Returns a file path URL, converting a file reference URL if necessary. }
{ For non-file URLs, returns NULL. Also returns NULL when the conversion fails }
{ because the target resource doesn't exist. }
{ Optional output error: The error is set to a valid CFErrorRef when the function }
{ result is NULL. A valid output error must be released by the caller. }
function CFURLCreateFilePathURL( allocator: CFAllocatorRef; url: CFURLRef; var error: CFErrorRef ): CFURLRef; external name '_CFURLCreateFilePathURL';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{#ifndef CF_OPEN_SOURCE}

>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifc TARGET_OS_MAC}

function CFURLCreateFromFSRef( allocator: CFAllocatorRef; const (*var*) fsRef_: FSRef ): CFURLRef; external name '_CFURLCreateFromFSRef';
(* CF_DEPRECATED(10_0, 10_9, 2_0, 7_0) *)

function CFURLGetFSRef( url: CFURLRef; var fsRef_: FSRef ): Boolean; external name '_CFURLGetFSRef';
(* CF_DEPRECATED(10_0, 10_9, 2_0, 7_0) *)

{$endc} {TARGET_OS_MAC}

{$endc} {TARGET_OS_MAC}

{$endc} {TARGET_OS_MAC}

{$endc} {TARGET_OS_MAC}

{$endc} {TARGET_OS_MAC}

{#endif} // !CF_OPEN_SOURCE

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ Resource access

    The behavior of resource value caching is slightly different between the NSURL and CFURL API.

    When the NSURL methods which get, set, or use cached resource values are used from the main thread, resource values cached by the URL (except those added as temporary properties) are invalidated the next time the main thread's run loop runs.

    The CFURL functions do not automatically clear any resource values cached by the URL. The client has complete control over the cache lifetime. If you are using CFURL API, you must use CFURLClearResourcePropertyCacheForKey or CFURLClearResourcePropertyCache to clear cached resource values.
 }


{
    CFURLCopyResourcePropertyForKey
    
    Returns the resource value identified by a given resource key.

    Parameters
        url
            The URL specifying the resource.
        key
            The resource key that identifies the resource property.
        propertyValueTypeRefPtr
            On output when the result is true, the resource value or NULL.
        error
            On output when the result is false, the error that occurred. This parameter is optional; if you do not wish the error returned, pass NULL here. The caller is responsible for releasing a valid output error.

    Return Value
        true if propertyValueTypeRefPtr is successfully populated; false if an error occurs.

    Discussion
        CFURLCopyResourcePropertyForKey first checks if the URL object already caches the resource value. If so, it returns the cached resource value to the caller. If not, then CFURLCopyResourcePropertyForKey synchronously obtains the resource value from the backing store, adds the resource value to the URL object's cache, and returns the resource value to the caller. The type of the resource value varies by resource property (see resource key definitions). If this function returns true and propertyValueTypeRefPtr is populated with NULL, it means the resource property is not available for the specified resource and no errors occurred when determining the resource property was not available. If this function returns false, the optional error is populated. This function is currently applicable only to URLs for file system resources.
        Symbol is present in iOS 4, but performs no operation.
 }
function CFURLCopyResourcePropertyForKey( url: CFURLRef; key: CFStringRef; propertyValueTypeRefPtr: UnivPtr; error: CFErrorRefPtr ): Boolean; external name '_CFURLCopyResourcePropertyForKey';
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)


{
    CFURLCopyResourcePropertiesForKeys
    
    Returns the resource values identified by specified array of resource keys.

    Parameters
        url
            The URL specifying the resource.
        keys
            An array of resource keys that identify the resource properties.
        error
            On output when the result is NULL, the error that occurred. This parameter is optional; if you do not wish the error returned, pass NULL here. The caller is responsible for releasing a valid output error.

    Return Value
        A dictionary of resource values indexed by resource key; NULL if an error occurs.

    Discussion
        CFURLCopyResourcePropertiesForKeys first checks if the URL object already caches the resource values. If so, it returns the cached resource values to the caller. If not, then CFURLCopyResourcePropertyForKey synchronously obtains the resource values from the backing store, adds the resource values to the URL object's cache, and returns the resource values to the caller. The type of the resource values vary by property (see resource key definitions). If the result dictionary does not contain a resource value for one or more of the requested resource keys, it means those resource properties are not available for the specified resource and no errors occurred when determining those resource properties were not available. If this function returns NULL, the optional error is populated. This function is currently applicable only to URLs for file system resources.
        Symbol is present in iOS 4, but performs no operation.
 }
function CFURLCopyResourcePropertiesForKeys( url: CFURLRef; keys: CFArrayRef; error: CFErrorRefPtr ): CFDictionaryRef; external name '_CFURLCopyResourcePropertiesForKeys';
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)


{
    CFURLSetResourcePropertyForKey
    
    Sets the resource value identified by a given resource key.

    Parameters
        url
            The URL specifying the resource.
        key
            The resource key that identifies the resource property.
        propertyValue
            The resource value.
        error
            On output when the result is false, the error that occurred. This parameter is optional; if you do not wish the error returned, pass NULL here. The caller is responsible for releasing a valid output error.

    Return Value
        true if the attempt to set the resource value completed with no errors; otherwise, false.

    Discussion
        CFURLSetResourcePropertyForKey writes the new resource value out to the backing store. Attempts to set a read-only resource property or to set a resource property not supported by the resource are ignored and are not considered errors. If this function returns false, the optional error is populated. This function is currently applicable only to URLs for file system resources.
        Symbol is present in iOS 4, but performs no operation.
 }
function CFURLSetResourcePropertyForKey( url: CFURLRef; key: CFStringRef; propertyValue: CFTypeRef; error: CFErrorRefPtr ): Boolean; external name '_CFURLSetResourcePropertyForKey';
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)


{
    CFURLSetResourcePropertiesForKeys
    
    Sets any number of resource values of a URL's resource.

    Parameters
        url
            The URL specifying the resource.
        keyedPropertyValues
            A dictionary of resource values indexed by resource keys.
        error
            On output when the result is false, the error that occurred. This parameter is optional; if you do not wish the error returned, pass NULL here. The caller is responsible for releasing a valid output error.

    Return Value
        true if the attempt to set the resource values completed with no errors; otherwise, false.

    Discussion
        CFURLSetResourcePropertiesForKeys writes the new resource values out to the backing store. Attempts to set read-only resource properties or to set resource properties not supported by the resource are ignored and are not considered errors. If an error occurs after some resource properties have been successfully changed, the userInfo dictionary in the returned error contains an array of resource keys that were not set with the key kCFURLKeysOfUnsetValuesKey. The order in which the resource values are set is not defined. If you need to guarantee the order resource values are set, you should make multiple requests to CFURLSetResourcePropertiesForKeys or CFURLSetResourcePropertyForKey to guarantee the order. If this function returns false, the optional error is populated. This function is currently applicable only to URLs for file system resources.
        Symbol is present in iOS 4, but performs no operation.
 }
function CFURLSetResourcePropertiesForKeys( url: CFURLRef; keyedPropertyValues: CFDictionaryRef; error: CFErrorRefPtr ): Boolean; external name '_CFURLSetResourcePropertiesForKeys';
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)


var kCFURLKeysOfUnsetValuesKey: CFStringRef; external name '_kCFURLKeysOfUnsetValuesKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
    { Key for the resource properties that have not been set after the CFURLSetResourcePropertiesForKeys function returns an error, returned as an array of of CFString objects. }


{
    CFURLClearResourcePropertyCacheForKey
    
    Discards a cached resource value of a URL.

    Parameters
        url
            The URL specifying the resource.
        key
            The resource key that identifies the resource property.

    Discussion
        Discarding a cached resource value may discard other cached resource values, because some resource values are cached as a set of values and because some resource values depend on other resource values (temporary properties have no dependencies). This function is currently applicable only to URLs for file system resources.
        Symbol is present in iOS 4, but performs no operation.
 }
procedure CFURLClearResourcePropertyCacheForKey( url: CFURLRef; key: CFStringRef ); external name '_CFURLClearResourcePropertyCacheForKey';
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)


{
    CFURLClearResourcePropertyCache
    
    Discards all cached resource values of a URL.

    Parameters
        url
            The URL specifying the resource.

    Discussion
        All temporary properties are also cleared from the URL object's cache. This function is currently applicable only to URLs for file system resources.
        Symbol is present in iOS 4, but performs no operation.
 }
procedure CFURLClearResourcePropertyCache( url: CFURLRef ); external name '_CFURLClearResourcePropertyCache';
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)


{
    CFURLSetTemporaryResourcePropertyForKey
    
    Sets a temporary resource value on the URL object.

    Parameters
        url
            The URL object.
        key
            The resource key that identifies the temporary resource property.
        propertyValue
            The resource value.

    Discussion
        Temporary properties are for client use. Temporary properties exist only in memory and are never written to the resource's backing store. Once set, a temporary value can be copied from the URL object with CFURLCopyResourcePropertyForKey and CFURLCopyResourcePropertiesForKeys. To remove a temporary value from the URL object, use CFURLClearResourcePropertyCacheForKey. Temporary values must be valid Core Foundation types, and will be retained by CFURLSetTemporaryResourcePropertyForKey. Care should be taken to ensure the key that identifies a temporary resource property is unique and does not conflict with system defined keys (using reverse domain name notation in your temporary resource property keys is recommended). This function is currently applicable only to URLs for file system resources.
        Symbol is present in iOS 4, but performs no operation.
 }
procedure CFURLSetTemporaryResourcePropertyForKey( url: CFURLRef; key: CFStringRef; propertyValue: CFTypeRef ); external name '_CFURLSetTemporaryResourcePropertyForKey';
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)


{
    CFURLResourceIsReachable
    
    Returns whether the URL's resource exists and is reachable.

    Parameters
        url
            The URL object.
        error
            On output when the result is false, the error that occurred. This parameter is optional; if you do not wish the error returned, pass NULL here. The caller is responsible for releasing a valid output error.

    Return Value
        true if the resource is reachable; otherwise, false.

    Discussion
        CFURLResourceIsReachable synchronously checks if the resource's backing store is reachable. Checking reachability is appropriate when making decisions that do not require other immediate operations on the resource, e.g. periodic maintenance of UI state that depends on the existence of a specific document. When performing operations such as opening a file or copying resource properties, it is more efficient to simply try the operation and handle failures. This function is currently applicable only to URLs for file system resources. If this function returns false, the optional error is populated. For other URL types, false is returned. 
        Symbol is present in iOS 4, but performs no operation.
 }
function CFURLResourceIsReachable( url: CFURLRef; error: CFErrorRefPtr ): Boolean; external name '_CFURLResourceIsReachable';
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)

{ CF_IMPLICIT_BRIDGING_ENABLED }


{ Properties of File System Resources }

var kCFURLNameKey: CFStringRef; external name '_kCFURLNameKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    { The resource name provided by the file system (Read-write, value type CFString) }

var kCFURLLocalizedNameKey: CFStringRef; external name '_kCFURLLocalizedNameKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    { Localized or extension-hidden name as displayed to users (Read-only, value type CFString) }

var kCFURLIsRegularFileKey: CFStringRef; external name '_kCFURLIsRegularFileKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    { True for regular files (Read-only, value type CFBoolean) }

var kCFURLIsDirectoryKey: CFStringRef; external name '_kCFURLIsDirectoryKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    { True for directories (Read-only, CFBoolean) }

var kCFURLIsSymbolicLinkKey: CFStringRef; external name '_kCFURLIsSymbolicLinkKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    { True for symlinks (Read-only, value type CFBoolean) }

var kCFURLIsVolumeKey: CFStringRef; external name '_kCFURLIsVolumeKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    { True for the root directory of a volume (Read-only, value type CFBoolean) }

var kCFURLIsPackageKey: CFStringRef; external name '_kCFURLIsPackageKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    { True for packaged directories (Read-only 10_6 and 10_7, read-write 10_8, value type CFBoolean). Note: You can only set or clear this property on directories; if you try to set this property on non-directory objects, the property is ignored. If the directory is a package for some other reason (extension type, etc), setting this property to false will have no effect. }

var kCFURLIsSystemImmutableKey: CFStringRef; external name '_kCFURLIsSystemImmutableKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    { True for system-immutable resources (Read-write, value type CFBoolean) }

var kCFURLIsUserImmutableKey: CFStringRef; external name '_kCFURLIsUserImmutableKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    { True for user-immutable resources (Read-write, value type CFBoolean) }

var kCFURLIsHiddenKey: CFStringRef; external name '_kCFURLIsHiddenKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    { True for resources normally not displayed to users (Read-write, value type CFBoolean). Note: If the resource is a hidden because its name starts with a period, setting this property to false will not change the property. }

var kCFURLHasHiddenExtensionKey: CFStringRef; external name '_kCFURLHasHiddenExtensionKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    { True for resources whose filename extension is removed from the localized name property (Read-write, value type CFBoolean) }

var kCFURLCreationDateKey: CFStringRef; external name '_kCFURLCreationDateKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    { The date the resource was created (Read-write, value type CFDate) }

var kCFURLContentAccessDateKey: CFStringRef; external name '_kCFURLContentAccessDateKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    { The date the resource was last accessed (Read-only, value type CFDate) }

var kCFURLContentModificationDateKey: CFStringRef; external name '_kCFURLContentModificationDateKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    { The time the resource content was last modified (Read-write, value type CFDate) }

var kCFURLAttributeModificationDateKey: CFStringRef; external name '_kCFURLAttributeModificationDateKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    { The time the resource's attributes were last modified (Read-write, value type CFDate) }

var kCFURLLinkCountKey: CFStringRef; external name '_kCFURLLinkCountKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    { Number of hard links to the resource (Read-only, value type CFNumber) }

var kCFURLParentDirectoryURLKey: CFStringRef; external name '_kCFURLParentDirectoryURLKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    { The resource's parent directory, if any (Read-only, value type CFURL) }

var kCFURLVolumeURLKey: CFStringRef; external name '_kCFURLVolumeURLKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    { URL of the volume on which the resource is stored (Read-only, value type CFURL) }

var kCFURLTypeIdentifierKey: CFStringRef; external name '_kCFURLTypeIdentifierKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    { Uniform type identifier (UTI) for the resource (Read-only, value type CFString) }

var kCFURLLocalizedTypeDescriptionKey: CFStringRef; external name '_kCFURLLocalizedTypeDescriptionKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    { User-visible type or "kind" description (Read-only, value type CFString) }

var kCFURLLabelNumberKey: CFStringRef; external name '_kCFURLLabelNumberKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    { The label number assigned to the resource (Read-write, value type CFNumber) }

var kCFURLLabelColorKey: CFStringRef; external name '_kCFURLLabelColorKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    { The color of the assigned label (Currently not implemented, value type CGColorRef, must link with Application Services) }

var kCFURLLocalizedLabelKey: CFStringRef; external name '_kCFURLLocalizedLabelKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    { The user-visible label text (Read-only, value type CFString) }

var kCFURLEffectiveIconKey: CFStringRef; external name '_kCFURLEffectiveIconKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    { The icon normally displayed for the resource (Read-only, value type CGImageRef, must link with Application Services) }

var kCFURLCustomIconKey: CFStringRef; external name '_kCFURLCustomIconKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    { The custom icon assigned to the resource, if any (Currently not implemented, value type CGImageRef, must link with Application Services) }

var kCFURLFileResourceIdentifierKey: CFStringRef; external name '_kCFURLFileResourceIdentifierKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
    { An identifier which can be used to compare two file system objects for equality using CFEqual (i.e, two object identifiers are equal if they have the same file system path or if the paths are linked to same inode on the same file system). This identifier is not persistent across system restarts. (Read-only, value type CFType) }

var kCFURLVolumeIdentifierKey: CFStringRef; external name '_kCFURLVolumeIdentifierKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
    { An identifier that can be used to identify the volume the file system object is on. Other objects on the same volume will have the same volume identifier and can be compared using for equality using CFEqual. This identifier is not persistent across system restarts. (Read-only, value type CFType) }

var kCFURLPreferredIOBlockSizeKey: CFStringRef; external name '_kCFURLPreferredIOBlockSizeKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
    { The optimal block size when reading or writing this file's data, or NULL if not available. (Read-only, value type CFNumber) }

var kCFURLIsReadableKey: CFStringRef; external name '_kCFURLIsReadableKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
    { true if this process (as determined by EUID) can read the resource. (Read-only, value type CFBoolean) }

var kCFURLIsWritableKey: CFStringRef; external name '_kCFURLIsWritableKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
    { true if this process (as determined by EUID) can write to the resource. (Read-only, value type CFBoolean) }

var kCFURLIsExecutableKey: CFStringRef; external name '_kCFURLIsExecutableKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
    { true if this process (as determined by EUID) can execute a file resource or search a directory resource. (Read-only, value type CFBoolean) }

var kCFURLFileSecurityKey: CFStringRef; external name '_kCFURLFileSecurityKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
    { The file system object's security information encapsulated in a CFFileSecurity object. (Read-write, value type CFFileSecurity) }

var kCFURLIsExcludedFromBackupKey: CFStringRef; external name '_kCFURLIsExcludedFromBackupKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_8, 5_1) *)
    { true if resource should be excluded from backups, false otherwise (Read-write, value type CFBoolean). This property is only useful for excluding cache and other application support files which are not needed in a backup. Some operations commonly made to user documents will cause this property to be reset to false and so this property should not be used on user documents. }

var kCFURLTagNamesKey: CFStringRef; external name '_kCFURLTagNamesKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_9, NA) *)
    { The array of Tag names (Read-write, value type CFArray of CFString) }
    
var kCFURLPathKey: CFStringRef; external name '_kCFURLPathKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_8, 6_0) *)
    { the URL's path as a file system path (Read-only, value type CFString) }

var kCFURLIsMountTriggerKey: CFStringRef; external name '_kCFURLIsMountTriggerKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 4_0) *)
    { true if this URL is a file system trigger directory. Traversing or opening a file system trigger will cause an attempt to mount a file system on the trigger directory. (Read-only, value type CFBoolean) }

var kCFURLFileResourceTypeKey: CFStringRef; external name '_kCFURLFileResourceTypeKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
    { Returns the file system object type. (Read-only, value type CFString) }

{ The file system object type values returned for the kCFURLFileResourceTypeKey }
var kCFURLFileResourceTypeNamedPipe: CFStringRef; external name '_kCFURLFileResourceTypeNamedPipe'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
var kCFURLFileResourceTypeCharacterSpecial: CFStringRef; external name '_kCFURLFileResourceTypeCharacterSpecial'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
var kCFURLFileResourceTypeDirectory: CFStringRef; external name '_kCFURLFileResourceTypeDirectory'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
var kCFURLFileResourceTypeBlockSpecial: CFStringRef; external name '_kCFURLFileResourceTypeBlockSpecial'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
var kCFURLFileResourceTypeRegular: CFStringRef; external name '_kCFURLFileResourceTypeRegular'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
var kCFURLFileResourceTypeSymbolicLink: CFStringRef; external name '_kCFURLFileResourceTypeSymbolicLink'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
var kCFURLFileResourceTypeSocket: CFStringRef; external name '_kCFURLFileResourceTypeSocket'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
var kCFURLFileResourceTypeUnknown: CFStringRef; external name '_kCFURLFileResourceTypeUnknown'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)

{ File Properties }

var kCFURLFileSizeKey: CFStringRef; external name '_kCFURLFileSizeKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    { Total file size in bytes (Read-only, value type CFNumber) }

var kCFURLFileAllocatedSizeKey: CFStringRef; external name '_kCFURLFileAllocatedSizeKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    { Total size allocated on disk for the file in bytes (number of blocks times block size) (Read-only, value type CFNumber) }

var kCFURLTotalFileSizeKey: CFStringRef; external name '_kCFURLTotalFileSizeKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
    { Total displayable size of the file in bytes (this may include space used by metadata), or NULL if not available. (Read-only, value type CFNumber) }

var kCFURLTotalFileAllocatedSizeKey: CFStringRef; external name '_kCFURLTotalFileAllocatedSizeKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
    { Total allocated size of the file in bytes (this may include space used by metadata), or NULL if not available. This can be less than the value returned by kCFURLTotalFileSizeKey if the resource is compressed. (Read-only, value type CFNumber) }

var kCFURLIsAliasFileKey: CFStringRef; external name '_kCFURLIsAliasFileKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    {  true if the resource is a Finder alias file or a symlink, false otherwise ( Read-only, value type CFBooleanRef) }


{ Volume Properties }

{ As a convenience, volume properties can be requested from any file system URL. The value returned will reflect the property value for the volume on which the resource is located. }

var kCFURLVolumeLocalizedFormatDescriptionKey: CFStringRef; external name '_kCFURLVolumeLocalizedFormatDescriptionKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    { The user-visible volume format (Read-only, value type CFString) }

var kCFURLVolumeTotalCapacityKey: CFStringRef; external name '_kCFURLVolumeTotalCapacityKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    { Total volume capacity in bytes (Read-only, value type CFNumber) }

var kCFURLVolumeAvailableCapacityKey: CFStringRef; external name '_kCFURLVolumeAvailableCapacityKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    { Total free space in bytes (Read-only, value type CFNumber) }

var kCFURLVolumeResourceCountKey: CFStringRef; external name '_kCFURLVolumeResourceCountKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    { Total number of resources on the volume (Read-only, value type CFNumber) }

var kCFURLVolumeSupportsPersistentIDsKey: CFStringRef; external name '_kCFURLVolumeSupportsPersistentIDsKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    { true if the volume format supports persistent object identifiers and can look up file system objects by their IDs (Read-only, value type CFBoolean) }

var kCFURLVolumeSupportsSymbolicLinksKey: CFStringRef; external name '_kCFURLVolumeSupportsSymbolicLinksKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    { true if the volume format supports symbolic links (Read-only, value type CFBoolean) }

var kCFURLVolumeSupportsHardLinksKey: CFStringRef; external name '_kCFURLVolumeSupportsHardLinksKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    { true if the volume format supports hard links (Read-only, value type CFBoolean) }

var kCFURLVolumeSupportsJournalingKey: CFStringRef; external name '_kCFURLVolumeSupportsJournalingKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    { true if the volume format supports a journal used to speed recovery in case of unplanned restart (such as a power outage or crash). This does not necessarily mean the volume is actively using a journal. (Read-only, value type CFBoolean) }

var kCFURLVolumeIsJournalingKey: CFStringRef; external name '_kCFURLVolumeIsJournalingKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    { true if the volume is currently using a journal for speedy recovery after an unplanned restart. (Read-only, value type CFBoolean) }

var kCFURLVolumeSupportsSparseFilesKey: CFStringRef; external name '_kCFURLVolumeSupportsSparseFilesKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    { true if the volume format supports sparse files, that is, files which can have 'holes' that have never been written to, and thus do not consume space on disk. A sparse file may have an allocated size on disk that is less than its logical length. (Read-only, value type CFBoolean) }

var kCFURLVolumeSupportsZeroRunsKey: CFStringRef; external name '_kCFURLVolumeSupportsZeroRunsKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    { For security reasons, parts of a file (runs) that have never been written to must appear to contain zeroes. true if the volume keeps track of allocated but unwritten runs of a file so that it can substitute zeroes without actually writing zeroes to the media. (Read-only, value type CFBoolean) }

var kCFURLVolumeSupportsCaseSensitiveNamesKey: CFStringRef; external name '_kCFURLVolumeSupportsCaseSensitiveNamesKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    { true if the volume format treats upper and lower case characters in file and directory names as different. Otherwise an upper case character is equivalent to a lower case character, and you can't have two names that differ solely in the case of the characters. (Read-only, value type CFBoolean) }

var kCFURLVolumeSupportsCasePreservedNamesKey: CFStringRef; external name '_kCFURLVolumeSupportsCasePreservedNamesKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)
    { true if the volume format preserves the case of file and directory names.  Otherwise the volume may change the case of some characters (typically making them all upper or all lower case). (Read-only, value type CFBoolean) }

var kCFURLVolumeSupportsRootDirectoryDatesKey: CFStringRef; external name '_kCFURLVolumeSupportsRootDirectoryDatesKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
    { true if the volume supports reliable storage of times for the root directory. (Read-only, value type CFBoolean) }

var kCFURLVolumeSupportsVolumeSizesKey: CFStringRef; external name '_kCFURLVolumeSupportsVolumeSizesKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
    { true if the volume supports returning volume size values (kCFURLVolumeTotalCapacityKey and kCFURLVolumeAvailableCapacityKey). (Read-only, value type CFBoolean) }

var kCFURLVolumeSupportsRenamingKey: CFStringRef; external name '_kCFURLVolumeSupportsRenamingKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
    { true if the volume can be renamed. (Read-only, value type CFBoolean) }

var kCFURLVolumeSupportsAdvisoryFileLockingKey: CFStringRef; external name '_kCFURLVolumeSupportsAdvisoryFileLockingKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
    { true if the volume implements whole-file flock(2) style advisory locks, and the O_EXLOCK and O_SHLOCK flags of the open(2) call. (Read-only, value type CFBoolean) }

var kCFURLVolumeSupportsExtendedSecurityKey: CFStringRef; external name '_kCFURLVolumeSupportsExtendedSecurityKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
    { true if the volume implements extended security (ACLs). (Read-only, value type CFBoolean) }

var kCFURLVolumeIsBrowsableKey: CFStringRef; external name '_kCFURLVolumeIsBrowsableKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
    { true if the volume should be visible via the GUI (i.e., appear on the Desktop as a separate volume). (Read-only, value type CFBoolean) }

var kCFURLVolumeMaximumFileSizeKey: CFStringRef; external name '_kCFURLVolumeMaximumFileSizeKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
    { The largest file size (in bytes) supported by this file system, or NULL if this cannot be determined. (Read-only, value type CFNumber) }

var kCFURLVolumeIsEjectableKey: CFStringRef; external name '_kCFURLVolumeIsEjectableKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
    { true if the volume's media is ejectable from the drive mechanism under software control. (Read-only, value type CFBoolean) }

var kCFURLVolumeIsRemovableKey: CFStringRef; external name '_kCFURLVolumeIsRemovableKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
    { true if the volume's media is removable from the drive mechanism. (Read-only, value type CFBoolean) }

var kCFURLVolumeIsInternalKey: CFStringRef; external name '_kCFURLVolumeIsInternalKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
    { true if the volume's device is connected to an internal bus, false if connected to an external bus, or NULL if not available. (Read-only, value type CFBoolean) }

var kCFURLVolumeIsAutomountedKey: CFStringRef; external name '_kCFURLVolumeIsAutomountedKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
    { true if the volume is automounted. Note: do not mistake this with the functionality provided by kCFURLVolumeSupportsBrowsingKey. (Read-only, value type CFBoolean) }

var kCFURLVolumeIsLocalKey: CFStringRef; external name '_kCFURLVolumeIsLocalKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
    { true if the volume is stored on a local device. (Read-only, value type CFBoolean) }

var kCFURLVolumeIsReadOnlyKey: CFStringRef; external name '_kCFURLVolumeIsReadOnlyKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
    { true if the volume is read-only. (Read-only, value type CFBoolean) }

var kCFURLVolumeCreationDateKey: CFStringRef; external name '_kCFURLVolumeCreationDateKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
    { The volume's creation date, or NULL if this cannot be determined. (Read-only, value type CFDate) }

var kCFURLVolumeURLForRemountingKey: CFStringRef; external name '_kCFURLVolumeURLForRemountingKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
    { The CFURL needed to remount a network volume, or NULL if not available. (Read-only, value type CFURL) }

var kCFURLVolumeUUIDStringKey: CFStringRef; external name '_kCFURLVolumeUUIDStringKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
    { The volume's persistent UUID as a string, or NULL if a persistent UUID is not available for the volume. (Read-only, value type CFString) }

var kCFURLVolumeNameKey: CFStringRef; external name '_kCFURLVolumeNameKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
    { The name of the volume (Read-write, settable if kCFURLVolumeSupportsRenamingKey is true and permissions allow, value type CFString) }

var kCFURLVolumeLocalizedNameKey: CFStringRef; external name '_kCFURLVolumeLocalizedNameKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
    { The user-presentable name of the volume (Read-only, value type CFString) }

{ UbiquitousItem Properties }

var kCFURLIsUbiquitousItemKey: CFStringRef; external name '_kCFURLIsUbiquitousItemKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
    { true if this item is synced to the cloud, false if it is only a local file. (Read-only, value type CFBoolean) }

var kCFURLUbiquitousItemHasUnresolvedConflictsKey: CFStringRef; external name '_kCFURLUbiquitousItemHasUnresolvedConflictsKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
    { true if this item has conflicts outstanding. (Read-only, value type CFBoolean) }

var kCFURLUbiquitousItemIsDownloadedKey: CFStringRef; external name '_kCFURLUbiquitousItemIsDownloadedKey'; (* attribute const *)
(* CF_DEPRECATED(10_7, 10_9, 5_0, 7_0, "Use kCFURLUbiquitousItemDownloadingStatusKey instead") *)
    { Equivalent to NSURLUbiquitousItemDownloadingStatusKey == NSURLUbiquitousItemDownloadingStatusCurrent. Has never behaved as documented in earlier releases, hence deprecated. (Read-only, value type CFBoolean) }

var kCFURLUbiquitousItemIsDownloadingKey: CFStringRef; external name '_kCFURLUbiquitousItemIsDownloadingKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
    { true if data is being downloaded for this item. (Read-only, value type CFBoolean) }

var kCFURLUbiquitousItemIsUploadedKey: CFStringRef; external name '_kCFURLUbiquitousItemIsUploadedKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
    { true if there is data present in the cloud for this item. (Read-only, value type CFBoolean) }

var kCFURLUbiquitousItemIsUploadingKey: CFStringRef; external name '_kCFURLUbiquitousItemIsUploadingKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_7, 5_0) *)
    { true if data is being uploaded for this item. (Read-only, value type CFBoolean) }

var kCFURLUbiquitousItemPercentDownloadedKey: CFStringRef; external name '_kCFURLUbiquitousItemPercentDownloadedKey'; (* attribute const *)
(* CF_DEPRECATED(10_7, 10_8, 5_0, 6_0, "Use NSMetadataQuery and NSMetadataUbiquitousItemPercentDownloadedKey on NSMetadataItem instead") *)
    { Use NSMetadataQuery and NSMetadataUbiquitousItemPercentDownloadedKey on NSMetadataItem instead }

var kCFURLUbiquitousItemPercentUploadedKey: CFStringRef; external name '_kCFURLUbiquitousItemPercentUploadedKey'; (* attribute const *)
(* CF_DEPRECATED(10_7, 10_8, 5_0, 6_0, "Use NSMetadataQuery and NSMetadataUbiquitousItemPercentUploadedKey on NSMetadataItem instead") *)

    { Use NSMetadataQuery and NSMetadataUbiquitousItemPercentUploadedKey on NSMetadataItem instead }

var kCFURLUbiquitousItemDownloadingStatusKey: CFStringRef; external name '_kCFURLUbiquitousItemDownloadingStatusKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_9, 7_0) *)
    { Returns the download status of this item. (Read-only, value type CFString). Possible values below. }

var kCFURLUbiquitousItemDownloadingErrorKey: CFStringRef; external name '_kCFURLUbiquitousItemDownloadingErrorKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_9, 7_0) *)
    { returns the error when downloading the item from iCloud failed. See the NSUbiquitousFile section in FoundationErrors.h. }

var kCFURLUbiquitousItemUploadingErrorKey: CFStringRef; external name '_kCFURLUbiquitousItemUploadingErrorKey'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_9, 7_0) *)
    { returns the error when uploading the item to iCloud failed. See the NSUbiquitousFile section in FoundationErrors.h. }

{ The values returned for kCFURLUbiquitousItemDownloadingStatusKey
 }
var kCFURLUbiquitousItemDownloadingStatusNotDownloaded: CFStringRef; external name '_kCFURLUbiquitousItemDownloadingStatusNotDownloaded'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_9, 7_0) *)
    { this item has not been downloaded yet. Use NSFileManager's startDownloadingUbiquitousItemAtURL:error: to download it }

var kCFURLUbiquitousItemDownloadingStatusDownloaded: CFStringRef; external name '_kCFURLUbiquitousItemDownloadingStatusDownloaded'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_9, 7_0) *)
    { there is a local version of this item available. The most current version will get downloaded as soon as possible. }

var kCFURLUbiquitousItemDownloadingStatusCurrent: CFStringRef; external name '_kCFURLUbiquitousItemDownloadingStatusCurrent'; (* attribute const *)
(* CF_AVAILABLE_STARTING(10_9, 7_0) *)
    { there is a local version of this item and it is the most up-to-date version known to this device. }

type
	CFURLBookmarkCreationOptions = CFOptionFlags;
const
    kCFURLBookmarkCreationPreferFileIDResolutionMask = 1 shl 8;  { At resolution time, this alias will prefer resolving by the embedded fileID to the path }
    kCFURLBookmarkCreationMinimalBookmarkMask = 1 shl 9; { Creates a bookmark with "less" information, which may be smaller but still be able to resolve in certain ways }
    kCFURLBookmarkCreationSuitableForBookmarkFile = 1 shl 10; { includes in the created bookmark those properties which are needed for a bookmark/alias file }
{$ifc TARGET_OS_MAC}    
    kCFURLBookmarkCreationWithSecurityScope = 1 shl 11; (* CF_AVAILABLE_STARTING(10_7,NA) *) { Mac OS X 10.7.3 and later, include information in the bookmark data which allows the same  sandboxed process to access the resource after being relaunched }
    kCFURLBookmarkCreationSecurityScopeAllowOnlyReadAccess = 1 shl 12; (* CF_AVAILABLE_STARTING(10_7,NA) *) { Mac OS X 10.7.3 and later, if used with kCFURLBookmarkCreationWithSecurityScope, at resolution time only read access to the resource will be granted }
{$endc}

const
	kCFBookmarkResolutionWithoutUIMask =  1 shl 8;		{ don't perform any UI during bookmark resolution }
	kCFBookmarkResolutionWithoutMountingMask =  1 shl 9 ;	{ don't mount a volume during bookmark resolution }

{$ifc TARGET_OS_MAC}
const
	kCFURLBookmarkResolutionWithSecurityScope =  1 shl 10 ; (* CF_ENUM_AVAILABLE(10_7,NA) *) { Mac OS X 10.7.3 and later, extract the security scope included at creation time to provide the ability to access the resource. }
{$endc}

type
	CFURLBookmarkResolutionOptions = CFOptionFlags;

type
	CFURLBookmarkFileCreationOptions = CFOptionFlags;

 { CF_IMPLICIT_BRIDGING_DISABLED }

{	@function CFURLCreateBookmarkData
	@discussion	Create a CFDataRef containing an externalizable representation from a CFURLRef, modified with the given options, including ( at the minimum ) any
		properties in the propertiesToInclude array which are retrievable from the given url.
	@param	allocator		the CFAllocator to use to create this object
	@param	url	the CFURLRef to create a bookmark data from.
	@param	options	a set of options which control creation of the bookmark data
	@param resourcePropertiesToInclude	If non-NULL, an CFArrayRef of additional properties copied from the url to include in the created bookmark data.
	@param relativeToURL If non-NULL, the created bookmark will be relative to the given url.  If kCFURLBookmarkCreationWithSecurityScope is given as
                an option and relativeToURL is non-NULL, then a collection-scoped bookmark is created which enables future access to url provided the caller has
                access to relativeURL.
	@param error	If non-NULL, on exit will be filled in with a CFErrorRef representing any error which occured during creation of the bookmark data
	@result	A CFDataRef containing an data, which can be later be passed to CFURLCreateByResolvingBookmarkData() or to CFURLCopyPropertiesForKeysFromBookmarkData() / CFURLCopyPropertyForKeyFromBookmarkData() }
function CFURLCreateBookmarkData( allocator: CFAllocatorRef; url: CFURLRef; options: CFURLBookmarkCreationOptions; resourcePropertiesToInclude: CFArrayRef; relativeToURL: CFURLRef; error: CFErrorRefPtr ): CFDataRef; external name '_CFURLCreateBookmarkData';
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)

{	@function CFURLCreateByResolvingBookmarkData
	@discussion Given a CFDataRef created with CFURLCreateBookmarkRepresentation(), return a CFURLRef of the item it was a bookmark to, and
		attempt to pre-cache those properties in propertiesToInclude in the resulting url.  If in the process of resolving the bookmark into the CFURLRef
	 	it points to this determines that  some properties in the bookmark are out of date or not correct for the item it resolves to, set *isStale to YES,
		which the client may want to use to decide to make a new bookmark from the returned item and replace the saved bookmark it has.  If the bookmarked
		item cannot be found, return NULL.  A bookmark created with security scope may fail to resolve if the caller does not have the same code signing identity
               as the caller which created the bookmark.
                After resolving a security scoped bookmark, the caller must call CFURLStartAccessingSecurityScopedResource() in order to gain access to the resource.
                If an error ( other than "original item can not be found" ) occurs during the process, return NULL and fill in error )
	@param	allocator	 the CFAllocator to use to create this object
	@param	 bookmark a CFDataRef containing a bookmark data, created with CFURLCreateBookmarkData
	@param	options options which affect the resolution
	@param relativeToURL If non-NULL, and if the bookmark was created relative to another url, then resolve it relative to this url.  If
                kCFURLBookmarkCreationWithSecurityScope was provided at creation, and kCFURLBookmarkResolutionWithSecurityScope is set, then relativeURL
                should point to the same item which was passed as relavitiveURL at creation time.
	@param resourcePropertiesToInclude If non-NULL, a CFArray containing those properties which the caller would like to already be cached on the given url
	@param isStale If non-NULL, on exit will be set to true if during resolution any of the properties in the bookmark no longer seemed to match the
		corresponding properties on the returned file.  Clients, upon seeing a stale representation, may want to replace whatever stored bookmark data they
		have saved and create a new one.
	@param error	If non-NULL, on exit will be filled in with a CFErrorRef representing any error which occured during resolution of the bookmark data
	@result A CFURLRef of a file which is the closest match to the file the bookmark data }
function CFURLCreateByResolvingBookmarkData( allocator: CFAllocatorRef; bookmark: CFDataRef; options: CFURLBookmarkResolutionOptions; relativeToURL: CFURLRef; resourcePropertiesToInclude: CFArrayRef; isStale: BooleanPtr; error: CFErrorRefPtr ): CFURLRef; external name '_CFURLCreateByResolvingBookmarkData';
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)

{	@function	CFURLCreatePropertiesForKeysFromBookmarkData
	@discussion	Given a bookmark, return a dictionary of properties ( all properties if propertiesToReturn == NULL ).
				This returns only the properties stored within the bookmark and will not attempt to resolve the bookmark or do i/o.
	@param	allocator	 the CFAllocator to use to create this object
	@param	 bookmark a CFDataRef containing a bookmark data, created with CFURLCreateBookmarkData
	@param	propertiesToReturn a CFArrayRef of the properties of the bookmark data which the client would like returned.
	@result	a CFDictionaryRef containing the values for the properties passed in obtained from the bookmark data ( not by attempting to resolve it or do i/o in any way ) }
function CFURLCreateResourcePropertiesForKeysFromBookmarkData( allocator: CFAllocatorRef; resourcePropertiesToReturn: CFArrayRef; bookmark: CFDataRef ): CFDictionaryRef; external name '_CFURLCreateResourcePropertiesForKeysFromBookmarkData';
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)

{	@function	CFURLCreatePropertyForKeyFromBookmarkData
	@discussion	Given a bookmark, return the value for a given property from the bookmark data
				This returns only the properties stored within the bookmark and will not attempt to resolve the bookmark or do i/o.
	@param	allocator	 the CFAllocator to use to create this object
	@param	 bookmark a CFDataRef containing a bookmark data, created with CFURLCreateBookmarkData
	@param	propertyKey the property key to return.
	@result	a CFTypeRef value for the property passed in obtained from the bookmark data ( not by attempting to resolve it or do i/o in any way ) }
function CFURLCreateResourcePropertyForKeyFromBookmarkData( allocator: CFAllocatorRef; resourcePropertyKey: CFStringRef; bookmark: CFDataRef ): CFTypeRef; external name '_CFURLCreateResourcePropertyForKeyFromBookmarkData';
(* CF_AVAILABLE_STARTING(10_6, 4_0) *)

{!	@function 	CFURLCreateBookmarkDataFromFile
	@description	Given a fileURL of a file which is a Finder "alias" file, return a CFDataRef with the bookmark data from the file.  If urlRef points to an alias file
			created before SnowLeopard which contains Alias Manager information and no bookmark data, then a CFDataRef will be synthesized which contains
			a approximation of the alias information in a format which can be used to resolve the bookmark.  If an error prevents reading the data or
			if it is corrupt, NULL will be returned and error will be filled in if errorRef is non-NULL.
	@param	allocator the CFAllocator to use to create this object
	@param	fileURL a CFURLRef to to the alias file to create the bookmark data from
	@param	errorRef    if non-NULL, on exit will be filled in with a CFErrorRef representing any error which occurred during the creation of the bookmark data from the file
	@result	A CFDataRef containing bookmark data, or NULL if there was an error creating bookmark data from the file, such as if the file is not an alias file.
 }
function CFURLCreateBookmarkDataFromFile( allocator: CFAllocatorRef; fileURL: CFURLRef; var errorRef: CFErrorRef ): CFDataRef; external name '_CFURLCreateBookmarkDataFromFile';
(* CF_AVAILABLE_STARTING(10_6, 5_0) *)

{!	@function	CFURLWriteBookmarkDataToFile
	@description	Given a created bookmarkData object, create a new Finder "alias" file at fileURL which contains the bookmark data.  If fileURL is a url to a directory, an alias file
			will be created with the same name as the bookmarked item and a ".alias" extension.  If fileURL is a url for a file and it exists it will be overwritten.  If a
			.alias extension is not present it will be added.  In addition to the bookmark data, sufficient pre-SnowLeopard alias data will added to the file to allow
			systems running something before SnowLeopard to resolve this file using Alias Manager routines and get back the same file as the bookmark routines.
			The bookmark data must have been created with the kCFURLBookmarkCreationSuitableForBookmarkFile option and an error will be returned if not.
	@param	allocator	 the CFAllocator to use to create this object
	@param	 bookmark a CFDataRef containing a bookmark data, created with CFURLCreateBookmarkData
	@param	options	options flags 
	@param	errorRef    if non-NULL, on exit will be filled in with a CFErrorRef representing any error which occurred during the creation of the alias file
 }
function CFURLWriteBookmarkDataToFile( bookmarkRef: CFDataRef; fileURL: CFURLRef; options: CFURLBookmarkFileCreationOptions; errorRef: CFErrorRefPtr ): Boolean; external name '_CFURLWriteBookmarkDataToFile';
(* CF_AVAILABLE_STARTING(10_6, 5_0) *)

{$ifc TARGET_OS_MAC}
{!	@function	CFURLCreateBookmarkDataFromAliasRecord
	@discussion	Create a CFDataRef containing bookmarkdata by converting the alias data in aliasRecordDataRef, which should be the contents of an AliasRecord copied into a CFDataRef object.
		The created bookmarkdata can be passed into CFURLCreateByResolvingBookmarkData() to resolve the item into a CFURLRef, or a small set of information can be returned from
		CFURLCreateResourcePropertiesForKeysFromBookmarkData() / CFURLCreateResourcePropertyForKeyFromBookmarkData().
		@param	allocator		the CFAllocator to use to create this object
		@param	aliasRecordDataRef	the contents of an AliasRecord to create bookmark data for
		@result	A CFDataRef containing an data, which can be later be passed to CFURLCreateByResolvingBookmarkData() or to CFURLCopyPropertiesForKeysFromBookmarkData() / CFURLCopyPropertyForKeyFromBookmarkData()
 }
function CFURLCreateBookmarkDataFromAliasRecord ( allocatorRef: CFAllocatorRef; aliasRecordDataRef: CFDataRef ): CFDataRef; external name '_CFURLCreateBookmarkDataFromAliasRecord';
(* CF_AVAILABLE_STARTING(10_6, NA) *)
{$endc}

{ CF_IMPLICIT_BRIDGING_ENABLED }

{$ifc TARGET_OS_MAC}
{!     @function	CFURLStartAccessingSecurityScopedResource
        @discussion	Given a CFURLRef created by resolving a bookmark data created with security scope, make the resource referenced by the
                        url accessible to the process.  When access to this resource is no longer needed the client should call
                        CFURLStopAccessingSecurityScopedResource().  Each call to CFURLStartAccessingSecurityScopedResource() must be balanced
                        with a call to CFURLStopAccessingSecurityScopedResource().
        @param	url     the CFURLRef for the resource returned by CFURLCreateByResolvingBookmarkData() using kCFURLBookmarkResolutionWithSecurityScope.
        @result        returns TRUE if access was granted and FALSE if the url does not reference a security scoped resource, or if some error occurred
                        which didn't allow access to be granted
 }
function CFURLStartAccessingSecurityScopedResource( url: CFURLRef ): Boolean; external name '_CFURLStartAccessingSecurityScopedResource';
(* CF_AVAILABLE_STARTING(10_7, NA) *) // Available in MacOS X 10.7.3 and later

{!     @function	CFURLStopAccessingSecurityScopedResource
        @discussion    Revokes the access granted to the url by a prior successful call to CFURLStartAccessingSecurityScopedResource().
        @param	url     the CFURLRef for the resource to stop accessing.
 }
procedure CFURLStopAccessingSecurityScopedResource( url: CFURLRef ); external name '_CFURLStopAccessingSecurityScopedResource';
(* CF_AVAILABLE_STARTING(10_7, NA) *)
{$endc}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
{ newString = CFURLCreateStringByAddingPercentEscapes(NULL, origString, NULL, NULL, kCFStringEncodingUTF8); }
=======
{ newString = CFURLCreateStringByAddingPercentEscapes(kCFAllocatorDefault, origString, NULL, NULL, kCFStringEncodingUTF8); }
>>>>>>> origin/fixes_2.4
function CFURLCreateStringByAddingPercentEscapes( allocator: CFAllocatorRef; originalString: CFStringRef; charactersToLeaveUnescaped: CFStringRef; legalURLCharactersToBeEscaped: CFStringRef; encoding: CFStringEncoding ): CFStringRef; external name '_CFURLCreateStringByAddingPercentEscapes';

{ Returns a file reference URL, a path-idependent form of file URL. }
{ Converts a file path URL if necessary. For non-file URLs, returns NULL. }
{ Also returns NULL when the conversion fails because the target resource doesn't exist. }
{ Optional output error: The error is set to a valid CFErrorRef when the function }
{ result is NULL. A valid output error must be released by the caller. }
function CFURLCreateFileReferenceURL( allocator: CFAllocatorRef; url: CFURLRef; var error: CFErrorRef ): CFURLRef; external name '_CFURLCreateFileReferenceURL';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)


{ Returns a file path URL, converting a file reference URL if necessary. }
{ For non-file URLs, returns NULL. Also returns NULL when the conversion fails }
{ because the target resource doesn't exist. }
{ Optional output error: The error is set to a valid CFErrorRef when the function }
{ result is NULL. A valid output error must be released by the caller. }
function CFURLCreateFilePathURL( allocator: CFAllocatorRef; url: CFURLRef; var error: CFErrorRef ): CFURLRef; external name '_CFURLCreateFilePathURL';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)

{#ifndef CF_OPEN_SOURCE}

{$ifc TARGET_OS_MAC}

function CFURLCreateFromFSRef( allocator: CFAllocatorRef; const (*var*) fsRef_: FSRef ): CFURLRef; external name '_CFURLCreateFromFSRef';

function CFURLGetFSRef( url: CFURLRef; var fsRef_: FSRef ): Boolean; external name '_CFURLGetFSRef';

{$endc} {TARGET_OS_MAC}

{#endif} // !CF_OPEN_SOURCE

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
