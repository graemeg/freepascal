{ CoreGraphics - CGDataProvider.h
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   Copyright (c) 1999-2011 Apple Inc.
   All rights reserved. }
{       Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, August 2015 }
=======
 * Copyright (c) 1999-2008 Apple Inc.
 * All rights reserved. }
{       Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
 * Copyright (c) 1999-2008 Apple Inc.
 * All rights reserved. }
{       Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
 * Copyright (c) 1999-2008 Apple Inc.
 * All rights reserved. }
{       Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
 * Copyright (c) 1999-2008 Apple Inc.
 * All rights reserved. }
{       Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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
 * Copyright (c) 1999-2004 Apple Computer, Inc.
 * All rights reserved.
 }
=======
 * Copyright (c) 1999-2008 Apple Inc.
 * All rights reserved. }
>>>>>>> origin/fixes_2.4
{       Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
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

unit CGDataProvider;
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
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
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
>>>>>>> origin/cpstrnew
{$ifc defined(iphonesim)}
 	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := TRUE}
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
=======
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
>>>>>>> graemeg/cpstrnew
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_CPU_ARM64 := TRUE}
=======
	{$setc TARGET_CPU_ARM := TRUE}
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
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
uses MacTypes,MacOSXPosix,CFBase,CFData,CGBase,CFURL;
{$endc} {not MACOSALLINCLUDE}

=======
uses MacTypes,CFBase,CFData,CGBase,CFURL;
>>>>>>> graemeg/fixes_2_2
=======
uses MacTypes,CFBase,CFData,CGBase,CFURL;
>>>>>>> origin/fixes_2_2
=======
uses MacTypes,MacOSXPosix,CFBase,CFData,CGBase,CFURL;
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
	CGDataProviderRef = ^OpaqueCGDataProviderRef; { an opaque type }
	OpaqueCGDataProviderRef = record end;
=======
	CGDataProviderRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CGDataProviderRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CGDataProviderRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CGDataProviderRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew


{ This callback is called to copy `count' bytes from the sequential data
   stream to `buffer'. }
=======
=======
>>>>>>> origin/fixes_2_2
	CGDataProviderRef = ^SInt32; { an opaque 32-bit type }


{ This callback is called to copy `count' bytes from the sequential data
 * stream to `buffer'. }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	CGDataProviderRef = ^SInt32; { an opaque type }


{ This callback is called to copy `count' bytes from the sequential data
   stream to `buffer'. }
>>>>>>> origin/fixes_2.4

type
	CGDataProviderGetBytesCallback = function( info: UnivPtr; buffer: UnivPtr; count: size_t ): size_t;

{ This callback is called to skip `count' bytes forward in the sequential
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   data stream. It should return the number of bytes that were actually
   skipped. }

type
	CGDataProviderSkipForwardCallback = function( info: UnivPtr; count: off_t ): off_t;

{ This callback is called to rewind to the beginning of sequential data
   stream. }
=======
=======
>>>>>>> origin/fixes_2_2
 * data stream. }
=======
   data stream. It should return the number of bytes that were actually
   skipped. }
>>>>>>> origin/fixes_2.4

type
	CGDataProviderSkipForwardCallback = function( info: UnivPtr; count: off_t ): off_t;

{ This callback is called to rewind to the beginning of sequential data
<<<<<<< HEAD
 * stream. }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
   stream. }
>>>>>>> origin/fixes_2.4

type
	CGDataProviderRewindCallback = procedure( info: UnivPtr );

{ This callback is called to release the `info' pointer when the data
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   provider is freed. }
=======
 * provider is freed. }
>>>>>>> graemeg/fixes_2_2
=======
 * provider is freed. }
>>>>>>> origin/fixes_2_2
=======
   provider is freed. }
>>>>>>> origin/fixes_2.4

type
	CGDataProviderReleaseInfoCallback = procedure( info: UnivPtr );

{ Callbacks for sequentially accessing data.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
   `version' is the version of this structure. It should be set to 0.
   `getBytes' is called to copy `count' bytes from the sequential data
     stream to `buffer'. It should return the number of bytes copied, or 0
     if there's no more data.
   `skipForward' is called to skip ahead in the sequential data stream by
     `count' bytes.
   `rewind' is called to rewind the sequential data stream to the beginning
     of the data.
   `releaseInfo', if non-NULL, is called to release the `info' pointer when
     the provider is freed. }
<<<<<<< HEAD

type
	CGDataProviderSequentialCallbacks = record
		version: UInt32;
{$ifc TARGET_CPU_64}
		__alignment_dummy: UInt32;
{$endc}
		getBytes: CGDataProviderGetBytesCallback;
		skipForward: CGDataProviderSkipForwardCallback;
		rewind: CGDataProviderRewindCallback;
		releaseInfo: CGDataProviderReleaseInfoCallback;
=======
=======
>>>>>>> origin/fixes_2_2
 * `getBytes' is called to copy `count' bytes from the sequential data
 *   stream to `buffer'.  It should return the number of bytes copied, or 0
 *   if there's no more data.
 * `skipBytes' is called to skip ahead in the sequential data stream by
 *   `count' bytes.
 * `rewind' is called to rewind the sequential data stream to the beginning
 *   of the data.
 * `releaseProvider', if non-NULL, is called to release the `info' pointer
 *   when the provider is freed. }
=======
>>>>>>> origin/fixes_2.4

type
	CGDataProviderSequentialCallbacks = record
		version: UInt32;
{$ifc TARGET_CPU_64}
		__alignment_dummy: UInt32;
{$endc}
		getBytes: CGDataProviderGetBytesCallback;
		skipForward: CGDataProviderSkipForwardCallback;
		rewind: CGDataProviderRewindCallback;
<<<<<<< HEAD
		releaseProvider: CGDataProviderReleaseInfoCallback;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
		releaseInfo: CGDataProviderReleaseInfoCallback;
>>>>>>> origin/fixes_2.4
	end;

{ This callback is called to get a pointer to the entire block of data. }

type
	CGDataProviderGetBytePointerCallback = function( info: UnivPtr ): UnivPtr;

{ This callback is called to release the pointer to entire block of
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   data. }
=======
 * data. }
>>>>>>> graemeg/fixes_2_2
=======
 * data. }
>>>>>>> origin/fixes_2_2
=======
   data. }
>>>>>>> origin/fixes_2.4

type
	CGDataProviderReleaseBytePointerCallback = procedure( info: UnivPtr; pointr: {const} UnivPtr );

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ This callback is called to copy `count' bytes at byte offset `position'
   into `buffer'. }

type
	CGDataProviderGetBytesAtPositionCallback = function( info: UnivPtr; buffer: UnivPtr; position: off_t; count: size_t ): size_t;

{ Callbacks for directly accessing data.
   `version' is the version of this structure. It should be set to 0.
   `getBytePointer', if non-NULL, is called to return a pointer to the
     provider's entire block of data.
   `releaseBytePointer', if non-NULL, is called to release a pointer to the
     provider's entire block of data.
   `getBytesAtPosition', if non-NULL, is called to copy `count' bytes at
     offset `position' from the provider's data to `buffer'. It should
     return the number of bytes copied, or 0 if there's no more data.
   `releaseInfo', if non-NULL, is called to release the `info' pointer when
     the provider is freed.

   At least one of `getBytePointer' or `getBytesAtPosition' must be
   non-NULL. }

type
	CGDataProviderDirectCallbacks = record
		version: UInt32;
{$ifc TARGET_CPU_64}
		__alignment_dummy: UInt32;
{$endc}
		getBytePointer: CGDataProviderGetBytePointerCallback;
		releaseBytePointer: CGDataProviderReleaseBytePointerCallback;
		getBytesAtPosition: CGDataProviderGetBytesAtPositionCallback;
		releaseInfo: CGDataProviderReleaseInfoCallback;
=======
=======
>>>>>>> origin/fixes_2_2
{ This callback is called to copy `count' bytes at byte offset `offset'
 * into `buffer'. }
=======
{ This callback is called to copy `count' bytes at byte offset `position'
   into `buffer'. }
>>>>>>> origin/fixes_2.4

type
	CGDataProviderGetBytesAtPositionCallback = function( info: UnivPtr; buffer: UnivPtr; position: off_t; count: size_t ): size_t;

{ Callbacks for directly accessing data.
   `version' is the version of this structure. It should be set to 0.
   `getBytePointer', if non-NULL, is called to return a pointer to the
     provider's entire block of data.
   `releaseBytePointer', if non-NULL, is called to release a pointer to the
     provider's entire block of data.
   `getBytesAtPosition', if non-NULL, is called to copy `count' bytes at
     offset `position' from the provider's data to `buffer'. It should
     return the number of bytes copied, or 0 if there's no more data.
   `releaseInfo', if non-NULL, is called to release the `info' pointer when
     the provider is freed.

   At least one of `getBytePointer' or `getBytesAtPosition' must be
   non-NULL. }

type
	CGDataProviderDirectCallbacks = record
		version: UInt32;
{$ifc TARGET_CPU_64}
		__alignment_dummy: UInt32;
{$endc}
		getBytePointer: CGDataProviderGetBytePointerCallback;
		releaseBytePointer: CGDataProviderReleaseBytePointerCallback;
<<<<<<< HEAD
		getBytes: CGDataProviderGetBytesAtOffsetCallback;
		releaseProvider: CGDataProviderReleaseInfoCallback;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
		getBytesAtPosition: CGDataProviderGetBytesAtPositionCallback;
		releaseInfo: CGDataProviderReleaseInfoCallback;
>>>>>>> origin/fixes_2.4
	end;

{ Return the CFTypeID for CGDataProviderRefs. }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function CGDataProviderGetTypeID: CFTypeID; external name '_CGDataProviderGetTypeID';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Create a sequential-access data provider using `callbacks' to provide the
   data. `info' is passed to each of the callback functions. }

function CGDataProviderCreateSequential( info: UnivPtr; const (*var*) callbacks: CGDataProviderSequentialCallbacks ): CGDataProviderRef; external name '_CGDataProviderCreateSequential';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Create a direct-access data provider using `callbacks' to supply `size'
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   bytes of data. `info' is passed to each of the callback functions.
   The underlying data must not change for the life of the data provider. }
=======
   bytes of data. `info' is passed to each of the callback functions. }
>>>>>>> graemeg/cpstrnew
=======
   bytes of data. `info' is passed to each of the callback functions. }
>>>>>>> graemeg/cpstrnew
=======
   bytes of data. `info' is passed to each of the callback functions. }
>>>>>>> graemeg/cpstrnew
=======
   bytes of data. `info' is passed to each of the callback functions. }
>>>>>>> origin/cpstrnew

function CGDataProviderCreateDirect( info: UnivPtr; size: off_t; const (*var*) callbacks: CGDataProviderDirectCallbacks ): CGDataProviderRef; external name '_CGDataProviderCreateDirect';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
function CGDataProviderGetTypeID: CFTypeID; external name '_CGDataProviderGetTypeID'; (* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
=======
function CGDataProviderGetTypeID: CFTypeID; external name '_CGDataProviderGetTypeID';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ Create a sequential-access data provider using `callbacks' to provide the
   data. `info' is passed to each of the callback functions. }

function CGDataProviderCreateSequential( info: UnivPtr; const (*var*) callbacks: CGDataProviderSequentialCallbacks ): CGDataProviderRef; external name '_CGDataProviderCreateSequential';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Create a direct-access data provider using `callbacks' to supply `size'
   bytes of data. `info' is passed to each of the callback functions. }

<<<<<<< HEAD
function CGDataProviderCreateDirectAccess( info: UnivPtr; size: size_t; const (*var*) callbacks: CGDataProviderDirectAccessCallbacks ): CGDataProviderRef; external name '_CGDataProviderCreateDirectAccess';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CGDataProviderCreateDirect( info: UnivPtr; size: off_t; const (*var*) callbacks: CGDataProviderDirectCallbacks ): CGDataProviderRef; external name '_CGDataProviderCreateDirect';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ The callback used by `CGDataProviderCreateWithData'. }

type
	CGDataProviderReleaseDataCallback = procedure( info: UnivPtr; data: {const} UnivPtr; size: size_t );

{ Create a direct-access data provider using `data', an array of `size'
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   bytes. `releaseData' is called when the data provider is freed, and is
   passed `info' as its first argument. }

function CGDataProviderCreateWithData( info: UnivPtr; data: {const} UnivPtr; size: size_t; releaseData: CGDataProviderReleaseDataCallback ): CGDataProviderRef; external name '_CGDataProviderCreateWithData';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Create a direct-access data provider which reads from `data'. }

function CGDataProviderCreateWithCFData( data: CFDataRef ): CGDataProviderRef; external name '_CGDataProviderCreateWithCFData';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Create a data provider reading from `url'. }

function CGDataProviderCreateWithURL( url: CFURLRef ): CGDataProviderRef; external name '_CGDataProviderCreateWithURL';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{ Create a data provider reading from `filename'. }

function CGDataProviderCreateWithFilename( filename: ConstCStringPtr ): CGDataProviderRef; external name '_CGDataProviderCreateWithFilename';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======

{ Create a data provider reading from `filename'. }

function CGDataProviderCreateWithFilename( filename: ConstCStringPtr ): CGDataProviderRef; external name '_CGDataProviderCreateWithFilename';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
{ Equivalent to `CFRetain(provider)', but doesn't crash (as CFRetain does)
   if `provider' is NULL. }

function CGDataProviderRetain( provider: CGDataProviderRef ): CGDataProviderRef; external name '_CGDataProviderRetain';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Equivalent to `CFRelease(provider)', but doesn't crash (as CFRelease
   does) if `provider' is NULL. }

procedure CGDataProviderRelease( provider: CGDataProviderRef ); external name '_CGDataProviderRelease';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
<<<<<<< HEAD

{ Return a copy of the data specified by provider. Returns NULL if a
   complete copy of the data can't be obtained (for example, if the
   underlying data is too large to fit in memory). }

function CGDataProviderCopyData( provider: CGDataProviderRef ): CFDataRef; external name '_CGDataProviderCopyData';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc TARGET_OS_MAC}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
{ Deprecated API. }

{ This callback is called to skip `count' bytes forward in the sequential
   data stream. }

type
	CGDataProviderSkipBytesCallback = procedure( info: UnivPtr; count: size_t );

=======

{ Return a copy of the data specified by provider. Returns NULL if a
   complete copy of the data can't be obtained (for example, if the
   underlying data is too large to fit in memory). }

function CGDataProviderCopyData( provider: CGDataProviderRef ): CFDataRef; external name '_CGDataProviderCopyData';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{ Deprecated API. }

{ This callback is called to skip `count' bytes forward in the sequential
   data stream. }

type
	CGDataProviderSkipBytesCallback = procedure( info: UnivPtr; count: size_t );

>>>>>>> origin/cpstrnew
{ Old-style callbacks for sequentially accessing data.
   `getBytes' is called to copy `count' bytes from the sequential data
     stream to `buffer'. It should return the number of bytes copied, or 0
     if there's no more data.
   `skipBytes' is called to skip ahead in the sequential data stream by
     `count' bytes.
   `rewind' is called to rewind the sequential data stream to the beginning
     of the data.
   `releaseProvider', if non-NULL, is called to release the `info' pointer
     when the provider is freed. }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/cpstrnew

type
	CGDataProviderCallbacks = record
		getBytes: CGDataProviderGetBytesCallback;
		skipBytes: CGDataProviderSkipBytesCallback;
		rewind: CGDataProviderRewindCallback;
		releaseProvider: CGDataProviderReleaseInfoCallback;
	end;

{ This callback is called to copy `count' bytes at byte offset `offset'
   into `buffer'. }

type
<<<<<<< HEAD
=======

type
	CGDataProviderCallbacks = record
		getBytes: CGDataProviderGetBytesCallback;
		skipBytes: CGDataProviderSkipBytesCallback;
		rewind: CGDataProviderRewindCallback;
		releaseProvider: CGDataProviderReleaseInfoCallback;
	end;

{ This callback is called to copy `count' bytes at byte offset `offset'
   into `buffer'. }

type
>>>>>>> graemeg/cpstrnew
	CGDataProviderGetBytesAtOffsetCallback = function( info: UnivPtr; buffer: UnivPtr; offset: size_t; count: size_t ): size_t;

{ Callbacks for directly accessing data.
   `getBytePointer', if non-NULL, is called to return a pointer to the
     provider's entire block of data.
   `releaseBytePointer', if non-NULL, is called to release a pointer to the
     provider's entire block of data.
   `getBytes', if non-NULL, is called to copy `count' bytes at offset
     `offset' from the provider's data to `buffer'. It should return the
     number of bytes copied, or 0 if there's no more data.
   `releaseProvider', if non-NULL, is called when the provider is freed.
  
   At least one of `getBytePointer' or `getBytes' must be non-NULL. }

type
	CGDataProviderDirectAccessCallbacks = record
		getBytePointer: CGDataProviderGetBytePointerCallback;
		releaseBytePointer: CGDataProviderReleaseBytePointerCallback;
		getBytes: CGDataProviderGetBytesAtOffsetCallback;
		releaseProvider: CGDataProviderReleaseInfoCallback;
	end;

{ Create a sequential-access data provider using `callbacks' to provide the
   data. `info' is passed to each of the callback functions. }

=======

type
	CGDataProviderCallbacks = record
		getBytes: CGDataProviderGetBytesCallback;
		skipBytes: CGDataProviderSkipBytesCallback;
		rewind: CGDataProviderRewindCallback;
		releaseProvider: CGDataProviderReleaseInfoCallback;
	end;

{ This callback is called to copy `count' bytes at byte offset `offset'
   into `buffer'. }

type
	CGDataProviderGetBytesAtOffsetCallback = function( info: UnivPtr; buffer: UnivPtr; offset: size_t; count: size_t ): size_t;

=======

type
	CGDataProviderCallbacks = record
		getBytes: CGDataProviderGetBytesCallback;
		skipBytes: CGDataProviderSkipBytesCallback;
		rewind: CGDataProviderRewindCallback;
		releaseProvider: CGDataProviderReleaseInfoCallback;
	end;

{ This callback is called to copy `count' bytes at byte offset `offset'
   into `buffer'. }

type
	CGDataProviderGetBytesAtOffsetCallback = function( info: UnivPtr; buffer: UnivPtr; offset: size_t; count: size_t ): size_t;

>>>>>>> graemeg/cpstrnew
=======
	CGDataProviderGetBytesAtOffsetCallback = function( info: UnivPtr; buffer: UnivPtr; offset: size_t; count: size_t ): size_t;

>>>>>>> origin/cpstrnew
{ Callbacks for directly accessing data.
   `getBytePointer', if non-NULL, is called to return a pointer to the
     provider's entire block of data.
   `releaseBytePointer', if non-NULL, is called to release a pointer to the
     provider's entire block of data.
   `getBytes', if non-NULL, is called to copy `count' bytes at offset
     `offset' from the provider's data to `buffer'. It should return the
     number of bytes copied, or 0 if there's no more data.
   `releaseProvider', if non-NULL, is called when the provider is freed.
  
   At least one of `getBytePointer' or `getBytes' must be non-NULL. }

type
	CGDataProviderDirectAccessCallbacks = record
		getBytePointer: CGDataProviderGetBytePointerCallback;
		releaseBytePointer: CGDataProviderReleaseBytePointerCallback;
		getBytes: CGDataProviderGetBytesAtOffsetCallback;
		releaseProvider: CGDataProviderReleaseInfoCallback;
	end;

{ Create a sequential-access data provider using `callbacks' to provide the
   data. `info' is passed to each of the callback functions. }

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function CGDataProviderCreate( info: UnivPtr; const (*var*) callbacks: CGDataProviderCallbacks ): CGDataProviderRef; external name '_CGDataProviderCreate';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)

{ Create a direct-access data provider using `callbacks' to supply `size'
   bytes of data. `info' is passed to each of the callback functions. }

function CGDataProviderCreateDirectAccess( info: UnivPtr; size: size_t; const (*var*) callbacks: CGDataProviderDirectAccessCallbacks ): CGDataProviderRef; external name '_CGDataProviderCreateDirectAccess';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
 * bytes.  `releaseData' is called when the data provider is freed, and is
 * passed `info' as its first argument. }
=======
   bytes. `releaseData' is called when the data provider is freed, and is
   passed `info' as its first argument. }
>>>>>>> origin/fixes_2.4

function CGDataProviderCreateWithData( info: UnivPtr; data: {const} UnivPtr; size: size_t; releaseData: CGDataProviderReleaseDataCallback ): CGDataProviderRef; external name '_CGDataProviderCreateWithData';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Create a direct-access data provider which reads from `data'. }

function CGDataProviderCreateWithCFData( data: CFDataRef ): CGDataProviderRef; external name '_CGDataProviderCreateWithCFData';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Create a data provider reading from `url'. }

function CGDataProviderCreateWithURL( url: CFURLRef ): CGDataProviderRef; external name '_CGDataProviderCreateWithURL';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Create a data provider reading from `filename'. }

function CGDataProviderCreateWithFilename( filename: ConstCStringPtr ): CGDataProviderRef; external name '_CGDataProviderCreateWithFilename';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Equivalent to `CFRetain(provider)', but doesn't crash (as CFRetain does)
   if `provider' is NULL. }

function CGDataProviderRetain( provider: CGDataProviderRef ): CGDataProviderRef; external name '_CGDataProviderRetain';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Equivalent to `CFRelease(provider)', but doesn't crash (as CFRelease
   does) if `provider' is NULL. }

procedure CGDataProviderRelease( provider: CGDataProviderRef ); external name '_CGDataProviderRelease';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return a copy of the data specified by provider. Returns NULL if a
   complete copy of the data can't be obtained (for example, if the
   underlying data is too large to fit in memory). }

function CGDataProviderCopyData( provider: CGDataProviderRef ): CFDataRef; external name '_CGDataProviderCopyData';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{ Deprecated API. }

{ This callback is called to skip `count' bytes forward in the sequential
   data stream. }

type
	CGDataProviderSkipBytesCallback = procedure( info: UnivPtr; count: size_t );

{ Old-style callbacks for sequentially accessing data.
   `getBytes' is called to copy `count' bytes from the sequential data
     stream to `buffer'. It should return the number of bytes copied, or 0
     if there's no more data.
   `skipBytes' is called to skip ahead in the sequential data stream by
     `count' bytes.
   `rewind' is called to rewind the sequential data stream to the beginning
     of the data.
   `releaseProvider', if non-NULL, is called to release the `info' pointer
     when the provider is freed. }

type
	CGDataProviderCallbacks = record
		getBytes: CGDataProviderGetBytesCallback;
		skipBytes: CGDataProviderSkipBytesCallback;
		rewind: CGDataProviderRewindCallback;
		releaseProvider: CGDataProviderReleaseInfoCallback;
	end;

{ This callback is called to copy `count' bytes at byte offset `offset'
   into `buffer'. }

type
	CGDataProviderGetBytesAtOffsetCallback = function( info: UnivPtr; buffer: UnivPtr; offset: size_t; count: size_t ): size_t;

{ Callbacks for directly accessing data.
   `getBytePointer', if non-NULL, is called to return a pointer to the
     provider's entire block of data.
   `releaseBytePointer', if non-NULL, is called to release a pointer to the
     provider's entire block of data.
   `getBytes', if non-NULL, is called to copy `count' bytes at offset
     `offset' from the provider's data to `buffer'. It should return the
     number of bytes copied, or 0 if there's no more data.
   `releaseProvider', if non-NULL, is called when the provider is freed.
  
   At least one of `getBytePointer' or `getBytes' must be non-NULL. }

type
	CGDataProviderDirectAccessCallbacks = record
		getBytePointer: CGDataProviderGetBytePointerCallback;
		releaseBytePointer: CGDataProviderReleaseBytePointerCallback;
		getBytes: CGDataProviderGetBytesAtOffsetCallback;
		releaseProvider: CGDataProviderReleaseInfoCallback;
	end;

{ Create a sequential-access data provider using `callbacks' to provide the
   data. `info' is passed to each of the callback functions. }

function CGDataProviderCreate( info: UnivPtr; const (*var*) callbacks: CGDataProviderCallbacks ): CGDataProviderRef; external name '_CGDataProviderCreate';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)

{ Create a direct-access data provider using `callbacks' to supply `size'
   bytes of data. `info' is passed to each of the callback functions. }

function CGDataProviderCreateDirectAccess( info: UnivPtr; size: size_t; const (*var*) callbacks: CGDataProviderDirectAccessCallbacks ): CGDataProviderRef; external name '_CGDataProviderCreateDirectAccess';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
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
