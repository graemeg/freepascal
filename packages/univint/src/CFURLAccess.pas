{	CFURLAccess.h
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	Copyright (c) 1998-2013, Apple Inc. All rights reserved.

        CFURLAccess is deprecated as of Mac OS X 10.9 and iOS 7.0. The suggested replacement for URLs with network schemes (http, https, ftp, data) is the NSURLConnection class. The suggested replacement for URLs with the file scheme are the foundation classes NSFileManager, NSFileHandle and NSURL, or the CoreFoundation classes CFStream and CFURL.
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
{   Pascal Translation Updated:  Gorazd Krosl, <gorazd_1957@yahoo.ca>, October 2009 }
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
	Copyright (c) 1998-2005, Apple, Inc. All rights reserved.
=======
	Copyright (c) 1998-2009, Apple Inc. All rights reserved.
>>>>>>> origin/fixes_2.4
}
{	  Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, November 2005 }
{   Pascal Translation Updated:  Gorazd Krosl, <gorazd_1957@yahoo.ca>, October 2009 }
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

unit CFURLAccess;
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
>>>>>>> origin/cpstrnew
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
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
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
>>>>>>> origin/cpstrnew
{$ifc defined(iphonesim)}
 	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := TRUE}
<<<<<<< HEAD
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
uses MacTypes,CFBase,CFArray,CFData,CFDictionary,CFString,CFURL;
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


{ Attempts to read the data and properties for the given URL.  If
only interested in one of the resourceData and properties, pass NULL
for the other.  If properties is non-NULL and desiredProperties is
NULL, then all properties are fetched.  Returns success or failure;
note that as much work as possible is done even if false is returned.
So for instance if one property is not available, the others are
fetched anyway. errorCode is set to 0 on success, and some other
value on failure.  If non-NULL, it is the caller 's responsibility
to release resourceData and properties.

    Apple reserves for its use all negative error code values; these
values represent errors common to any scheme.  Scheme-specific error
codes should be positive, non-zero, and should be used only if one of
the predefined Apple error codes does not apply.  Error codes should
be publicized and documented with the scheme-specific properties.

NOTE: When asking for the resource data, this call will allocate the entire
resource in memory. This can be very expensive, depending on the size of the
resource (file). Please use CFStream or other techniques if you are downloading
large files.

}
<<<<<<< HEAD
<<<<<<< HEAD
{ Deprecated -- see top of this file for suggested replacement classes }
function CFURLCreateDataAndPropertiesFromResource( alloc: CFAllocatorRef; url: CFURLRef; resourceData: CFDataRefPtr; properties: CFDictionaryRefPtr; desiredProperties: CFArrayRef; var errorCode: SInt32 ): Boolean; external name '_CFURLCreateDataAndPropertiesFromResource';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CF_DEPRECATED(10_0, 10_9, 2_0, 7_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
function CFURLCreateDataAndPropertiesFromResource( alloc: CFAllocatorRef; url: CFURLRef; resourceData: CFDataRefPtr; properties: CFDictionaryRefPtr; desiredProperties: CFArrayRef; var errorCode: SInt32 ): Boolean; external name '_CFURLCreateDataAndPropertiesFromResource';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
function CFURLCreateDataAndPropertiesFromResource( alloc: CFAllocatorRef; url: CFURLRef; resourceData: CFDataRefPtr; properties: CFDictionaryRefPtr; desiredProperties: CFArrayRef; var errorCode: SInt32 ): Boolean; external name '_CFURLCreateDataAndPropertiesFromResource';
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4

{ Attempts to write the given data and properties to the given URL.
If dataToWrite is NULL, only properties are written out (use
CFURLDestroyResource() to delete a resource).  Properties not present
in propertiesToWrite are left unchanged, hence if propertiesToWrite
is NULL or empty, the URL's properties are not changed at all.
Returns success or failure; errorCode is set as for
CFURLCreateDataAndPropertiesFromResource(), above.
}
<<<<<<< HEAD
<<<<<<< HEAD
{ Deprecated -- see top of this file for suggested replacement classes }
function CFURLWriteDataAndPropertiesToResource( url: CFURLRef; dataToWrite: CFDataRef; propertiesToWrite: CFDictionaryRef; var errorCode: SInt32 ): Boolean; external name '_CFURLWriteDataAndPropertiesToResource';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CF_DEPRECATED(10_0, 10_9, 2_0, 7_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

{ Destroys the resource indicated by url.
Returns success or failure; errorCode set as above.
}
{ Deprecated -- see top of this file for suggested replacement classes }
function CFURLDestroyResource( url: CFURLRef; var errorCode: SInt32 ): Boolean; external name '_CFURLDestroyResource';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CF_DEPRECATED(10_0, 10_9, 2_0, 7_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

{ Convenience method which calls through to CFURLCreateDataAndPropertiesFromResource().
Returns NULL on error and sets errorCode accordingly.
}
{ Deprecated -- see top of this file for suggested replacement classes }
function CFURLCreatePropertyFromResource( alloc: CFAllocatorRef; url: CFURLRef; proprty: CFStringRef; var errorCode: SInt32 ): CFTypeRef; external name '_CFURLCreatePropertyFromResource';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CF_DEPRECATED(10_0, 10_9, 2_0, 7_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


{ Common error codes (returned only by the older APIs that predate CFError) }
type
<<<<<<< HEAD
	CFURLError = CFIndex;
const
	kCFURLUnknownError = -10; (* CF_ENUM_DEPRECATED(10_0, 10_9, 2_0, 7_0) *)
	kCFURLUnknownSchemeError = -11; (* CF_ENUM_DEPRECATED(10_0, 10_9, 2_0, 7_0) *)
	kCFURLResourceNotFoundError = -12; (* CF_ENUM_DEPRECATED(10_0, 10_9, 2_0, 7_0) *)
	kCFURLResourceAccessViolationError = -13; (* CF_ENUM_DEPRECATED(10_0, 10_9, 2_0, 7_0) *)
	kCFURLRemoteHostUnavailableError = -14; (* CF_ENUM_DEPRECATED(10_0, 10_9, 2_0, 7_0) *)
	kCFURLImproperArgumentsError = -15; (* CF_ENUM_DEPRECATED(10_0, 10_9, 2_0, 7_0) *)
	kCFURLUnknownPropertyKeyError = -16; (* CF_ENUM_DEPRECATED(10_0, 10_9, 2_0, 7_0) *)
	kCFURLPropertyKeyUnavailableError = -17; (* CF_ENUM_DEPRECATED(10_0, 10_9, 2_0, 7_0) *)
	kCFURLTimeoutError = -18; (* CF_ENUM_DEPRECATED(10_0, 10_9, 2_0, 7_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> graemeg/cpstrnew
{ Common error codes (returned only by the older APIs that predate CFError) }
type
=======
>>>>>>> origin/cpstrnew
	CFURLError = SIGNEDLONG;
=======
=======
>>>>>>> origin/fixes_2_2
function CFURLWriteDataAndPropertiesToResource( url: CFURLRef; dataToWrite: CFDataRef; propertiesToWrite: CFDictionaryRef; var errorCode: SInt32 ): Boolean; external name '_CFURLWriteDataAndPropertiesToResource';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{ Destroys the resource indicated by url. }
{ Returns success or failure; errorCode set as above. }
function CFURLDestroyResource( url: CFURLRef; var errorCode: SInt32 ): Boolean; external name '_CFURLDestroyResource';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{ Convenience method which calls through to CFURLCreateDataAndPropertiesFromResource(). }
{ Returns NULL on error and sets errorCode accordingly. }
function CFURLCreatePropertyFromResource( alloc: CFAllocatorRef; url: CFURLRef; proprty: CFStringRef; var errorCode: SInt32 ): CFTypeRef; external name '_CFURLCreatePropertyFromResource';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Common error codes (returned only by the older APIs that predate CFError) }
type
<<<<<<< HEAD
	CFURLError = SInt32;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	CFURLError = SIGNEDLONG;
>>>>>>> origin/fixes_2.4
const
	kCFURLUnknownError = -10;
	kCFURLUnknownSchemeError = -11;
	kCFURLResourceNotFoundError = -12;
	kCFURLResourceAccessViolationError = -13;
	kCFURLRemoteHostUnavailableError = -14;
	kCFURLImproperArgumentsError = -15;
	kCFURLUnknownPropertyKeyError = -16;
	kCFURLPropertyKeyUnavailableError = -17;
	kCFURLTimeoutError = -18;
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew

{ Older property keys }

var kCFURLFileExists: CFStringRef; external name '_kCFURLFileExists'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CF_DEPRECATED(10_0, 10_9, 2_0, 7_0) *)
var kCFURLFileDirectoryContents: CFStringRef; external name '_kCFURLFileDirectoryContents'; (* attribute const *)
(* CF_DEPRECATED(10_0, 10_9, 2_0, 7_0) *)
var kCFURLFileLength: CFStringRef; external name '_kCFURLFileLength'; (* attribute const *)
(* CF_DEPRECATED(10_0, 10_9, 2_0, 7_0) *)
var kCFURLFileLastModificationTime: CFStringRef; external name '_kCFURLFileLastModificationTime'; (* attribute const *)
(* CF_DEPRECATED(10_0, 10_9, 2_0, 7_0) *)
var kCFURLFilePOSIXMode: CFStringRef; external name '_kCFURLFilePOSIXMode'; (* attribute const *)
(* CF_DEPRECATED(10_0, 10_9, 2_0, 7_0) *)
var kCFURLFileOwnerID: CFStringRef; external name '_kCFURLFileOwnerID'; (* attribute const *)
(* CF_DEPRECATED(10_0, 10_9, 2_0, 7_0) *)
var kCFURLHTTPStatusCode: CFStringRef; external name '_kCFURLHTTPStatusCode'; (* attribute const *)
(* CF_DEPRECATED(10_0, 10_9, 2_0, 7_0) *)
var kCFURLHTTPStatusLine: CFStringRef; external name '_kCFURLHTTPStatusLine'; (* attribute const *)
(* CF_DEPRECATED(10_0, 10_9, 2_0, 7_0) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
var kCFURLFileDirectoryContents: CFStringRef; external name '_kCFURLFileDirectoryContents'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
var kCFURLFileLength: CFStringRef; external name '_kCFURLFileLength'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
var kCFURLFileLastModificationTime: CFStringRef; external name '_kCFURLFileLastModificationTime'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
var kCFURLFilePOSIXMode: CFStringRef; external name '_kCFURLFilePOSIXMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
var kCFURLFileOwnerID: CFStringRef; external name '_kCFURLFileOwnerID'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
var kCFURLHTTPStatusCode: CFStringRef; external name '_kCFURLHTTPStatusCode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
var kCFURLHTTPStatusLine: CFStringRef; external name '_kCFURLHTTPStatusLine'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
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

{ Older property keys }

var kCFURLFileExists: CFStringRef; external name '_kCFURLFileExists'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
var kCFURLFileDirectoryContents: CFStringRef; external name '_kCFURLFileDirectoryContents'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
var kCFURLFileLength: CFStringRef; external name '_kCFURLFileLength'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
var kCFURLFileLastModificationTime: CFStringRef; external name '_kCFURLFileLastModificationTime'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
var kCFURLFilePOSIXMode: CFStringRef; external name '_kCFURLFilePOSIXMode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
var kCFURLFileOwnerID: CFStringRef; external name '_kCFURLFileOwnerID'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
var kCFURLHTTPStatusCode: CFStringRef; external name '_kCFURLHTTPStatusCode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
var kCFURLHTTPStatusLine: CFStringRef; external name '_kCFURLHTTPStatusLine'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4

{ The value of kCFURLFileExists is a CFBoolean }
{ The value of kCFURLFileDirectoryContents is a CFArray containing CFURLs.  An empty array means the directory exists, but is empty }
{ The value of kCFURLFileLength is a CFNumber giving the file's length in bytes }
{ The value of kCFURLFileLastModificationTime is a CFDate }
{ The value of kCFURLFilePOSIXMode is a CFNumber as given in stat.h }
{ The value of kCFURLFileOwnerID is a CFNumber representing the owner's uid }

{ Properties for the http: scheme.  Except for the common error codes, above, errorCode will be set to the HTTP response status code upon failure.  Any HTTP header name can also be used as a property }
{ The value of kCFURLHTTPStatusCode is a CFNumber }
{ The value of kCFURLHTTPStatusLine is a CFString }

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
