{
 *  CVOpenGLBufferPool.h
 *  CoreVideo
 *
 *  Copyright 2004 Apple Computer, Inc. All rights reserved.
 *
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{  Pascal Translation:  Gorazd Krosl, <gorazd_1957@yahoo.ca>, 2009 }
{  Pascal Translation Update: Jonas Maebe <jonas@freepascal.org>, October 2012 }
{  Pascal Translation Update: Jonas Maebe <jonas@freepascal.org>, August 2015 }
=======
{	 Pascal Translation:  Gorazd Krosl, <gorazd_1957@yahoo.ca>, 2009 }

>>>>>>> graemeg/cpstrnew
=======
{	 Pascal Translation:  Gorazd Krosl, <gorazd_1957@yahoo.ca>, 2009 }

>>>>>>> graemeg/cpstrnew
=======
{	 Pascal Translation:  Gorazd Krosl, <gorazd_1957@yahoo.ca>, 2009 }

>>>>>>> graemeg/cpstrnew
=======
{	 Pascal Translation:  Gorazd Krosl, <gorazd_1957@yahoo.ca>, 2009 }

>>>>>>> origin/cpstrnew
=======
{	 Pascal Translation:  Gorazd Krosl, <gorazd_1957@yahoo.ca>, 2009 }

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

unit CVOpenGLBufferPool;
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
uses MacTypes,CFBase,CFString,CFDictionary,CVBase,CVReturns,CVOpenGLBuffer;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}

 {! @header CVOpenGLBufferPool.h
	@copyright 2004 Apple Computer, Inc. All rights reserved.
	@availability Mac OS X 10.4 or later
    @discussion CVOpenGLBufferPool is a utility object for managing a set of CVOpenGLBuffer objects that are going to be recycled.
		   
}


type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CVOpenGLBufferPoolRef = ^__CVOpenGLBufferPool; { an opaque type }
	__CVOpenGLBufferPool = record end;
=======
	CVOpenGLBufferPoolRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CVOpenGLBufferPoolRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CVOpenGLBufferPoolRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CVOpenGLBufferPoolRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	CVOpenGLBufferPoolRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4

var kCVOpenGLBufferPoolMinimumBufferCountKey: CFStringRef; external name '_kCVOpenGLBufferPoolMinimumBufferCountKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

// By default, buffers will age out after one second.   If required, setting an age of zero will disable
// the age-out mechanism completely.
var kCVOpenGLBufferPoolMaximumBufferAgeKey: CFStringRef; external name '_kCVOpenGLBufferPoolMaximumBufferAgeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

function CVOpenGLBufferPoolGetTypeID: CFTypeID; external name '_CVOpenGLBufferPoolGetTypeID';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{!
    @function   CVOpenGLBufferPoolRetain
    @abstract   Retains a CVOpenGLBufferPoolRef object
    @discussion Equivalent to CFRetain, but NULL safe
    @param      buffer A CVOpenGLBufferPoolRef object that you want to retain.
    @result     A CVOpenGLBufferPoolRef object that is the same as the passed in buffer.
}
function CVOpenGLBufferPoolRetain( openGLBufferPool: CVOpenGLBufferPoolRef ): CVOpenGLBufferPoolRef; external name '_CVOpenGLBufferPoolRetain';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *) // NULL-safe

{!
    @function   CVOpenGLBufferPoolRelease
    @abstract   Releases a CVOpenGLBufferPoolRef object
    @discussion Equivalent to CFRelease, but NULL safe
    @param      buffer A CVOpenGLBufferPoolRef object that you want to release.
}
procedure CVOpenGLBufferPoolRelease( openGLBufferPool: CVOpenGLBufferPoolRef ); external name '_CVOpenGLBufferPoolRelease';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *) // NULL-safe

{!
    @function   CVOpenGLBufferPoolCreate
    @abstract   Creates a new OpenGL Buffer pool.
    @discussion Equivalent to CFRelease, but NULL safe
    @param      allocator The CFAllocatorRef to use for allocating this buffer pool.  May be NULL.
    @param      poolAttributes   A CFDictionaryRef containing the attributes to be used for the pool itself.
    @param      openGLBufferAttributes   A CFDictionaryRef containing the attributes to be used for creating new OpenGLBuffers within the pool.
    @param      poolOut   The newly created pool will be placed here
    @result     Returns kCVReturnSuccess on success
}
function CVOpenGLBufferPoolCreate( allocator: CFAllocatorRef; poolAttributes: CFDictionaryRef; openGLBufferAttributes: CFDictionaryRef; var poolOut: CVOpenGLBufferPoolRef ): CVReturn; external name '_CVOpenGLBufferPoolCreate';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{!
    @function   CVOpenGLBufferPoolGetOpenGLBufferAttributes
    @abstract   Returns the pool attributes dictionary for a CVOpenGLBufferPool
    @param      pool  The CVOpenGLBufferPoolRef to retrieve the attributes from
    @result     Returns the pool attributes dictionary, or NULL on failure.
}
function CVOpenGLBufferPoolGetAttributes( pool: CVOpenGLBufferPoolRef ): CFDictionaryRef; external name '_CVOpenGLBufferPoolGetAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{!
    @function   CVOpenGLBufferPoolGetOpenGLBufferAttributes
    @abstract   Returns the attributes of OpenGL buffers that will be created from this pool.
    @discussion This function is provided for those cases where you may need to know some information about the buffers that
                will be created up front.
    @param      pool  The CVOpenGLBufferPoolRef to retrieve the attributes from
    @result     Returns the OpenGL buffer attributes dictionary, or NULL on failure.
}
function CVOpenGLBufferPoolGetOpenGLBufferAttributes( pool: CVOpenGLBufferPoolRef ): CFDictionaryRef; external name '_CVOpenGLBufferPoolGetOpenGLBufferAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{!
    @function   CVOpenGLBufferPoolCreateOpenGLBuffer
    @abstract   Creates a new OpenGLBuffer object from the pool.
    @discussion The function creates a new CVOpenGLBuffer with the default attachments using the OpenGL buffer attributes specifed during pool creation.
    @param      allocator The CFAllocatorRef to use for creating the OpenGL buffer.  May be NULL.
    @param      openGLBufferPool      The CVOpenGLBufferPool that should create the new CVOpenGLBuffer.
    @param      openGLBufferOut   The newly created OpenGL buffer will be placed here
    @result     Returns kCVReturnSuccess on success
}
function CVOpenGLBufferPoolCreateOpenGLBuffer( allocator: CFAllocatorRef; openGLBufferPool: CVOpenGLBufferPoolRef; var openGLBufferOut: CVOpenGLBufferRef ): CVReturn; external name '_CVOpenGLBufferPoolCreateOpenGLBuffer';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{$endc}	// TARGET_OS_MAC
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
