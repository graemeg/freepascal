{
 *  IOSurface.h
 *  IOSurface
 *
 *  Copyright 2006-2008 Apple Computer, Inc. All rights reserved.
 *
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, September 2010 }
{  Pascal Translation Update: Jonas Maebe <jonas@freepascal.org>, October 2012 }
=======
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, September 2010 }
>>>>>>> graemeg/cpstrnew
=======
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, September 2010 }
>>>>>>> graemeg/cpstrnew
=======
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, September 2010 }
>>>>>>> graemeg/cpstrnew
=======
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, September 2010 }
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

unit IOSurfaceAPI;
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
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
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
>>>>>>> origin/cpstrnew
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
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
uses MacTypes,MacOSXPosix,CFBase,CFDictionary,IOKitReturn;
{$endc} {not MACOSALLINCLUDE}


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
{$ifc TARGET_OS_MAC}

>>>>>>> graemeg/cpstrnew
=======
{$ifc TARGET_OS_MAC}

>>>>>>> graemeg/cpstrnew
=======
{$ifc TARGET_OS_MAC}

>>>>>>> graemeg/cpstrnew
=======
{$ifc TARGET_OS_MAC}

>>>>>>> origin/cpstrnew
type
	IOSurfaceRef = ^__IOSurface; { an opaque type }
	__IOSurface = record end;

type
	IOSurfaceID = UInt32;

<<<<<<< HEAD
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
=======
>>>>>>> origin/cpstrnew
{ The following list of properties are used with the CFDictionary passed to IOSurfaceCreate(). }

{ kIOSurfaceAllocSize    - CFNumber of the total allocation size of the buffer including all planes.    
				    Defaults to BufferHeight * BytesPerRow if not specified.   Must be specified for
				    dimensionless buffers. }
var kIOSurfaceAllocSize: CFStringRef; external name '_kIOSurfaceAllocSize'; (* attribute const *)
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ kIOSurfaceWidth  - CFNumber for the width of the IOSurface buffer in pixels.  Required for planar IOSurfaces. }
var kIOSurfaceWidth: CFStringRef; external name '_kIOSurfaceWidth'; (* attribute const *)
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ kIOSurfaceHeight - CFNumber for the height of the IOSurface buffer in pixels. Required for planar IOSurfaces. }
var kIOSurfaceHeight: CFStringRef; external name '_kIOSurfaceHeight'; (* attribute const *)
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ kIOSurfaceBytesPerRow - CFNumber for the bytes per row of the buffer.   If not specified, IOSurface will first calculate
                                   the number full elements required on each row (by rounding up), multiplied by the bytes per element
				   for this buffer.   That value will then be appropriately aligned. }
var kIOSurfaceBytesPerRow: CFStringRef; external name '_kIOSurfaceBytesPerRow'; (* attribute const *)
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ Optional properties for non-planar two dimensional images }
 
{ kIOSurfaceBitsPerElement - CFNumber for the total number of bytes in an element.  Default to 1. }
var kIOSurfaceBytesPerElement: CFStringRef; external name '_kIOSurfaceBytesPerElement'; (* attribute const *)
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ kIOSurfaceElementWidth   - CFNumber for how many pixels wide each element is.   Defaults to 1. } 
var kIOSurfaceElementWidth: CFStringRef; external name '_kIOSurfaceElementWidth'; (* attribute const *)
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ kIOSurfaceElementHeight  - CFNumber for how many pixels high each element is.   Defaults to 1. } 
var kIOSurfaceElementHeight: CFStringRef; external name '_kIOSurfaceElementHeight'; (* attribute const *)
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ kIOSurfaceOffset - CFNumber for the starting offset into the buffer.  Defaults to 0. }
var kIOSurfaceOffset: CFStringRef; external name '_kIOSurfaceOffset'; (* attribute const *)
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ Properties for planar surface buffers }

{ kIOSurfacePlaneInfo    - CFArray describing each image plane in the buffer as a CFDictionary.   The CFArray must have at least one entry. }
var kIOSurfacePlaneInfo: CFStringRef; external name '_kIOSurfacePlaneInfo'; (* attribute const *)
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ kIOSurfacePlaneWidth  - CFNumber for the width of this plane in pixels.  Required for image planes. }
var kIOSurfacePlaneWidth: CFStringRef; external name '_kIOSurfacePlaneWidth'; (* attribute const *)
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ kIOSurfacePlaneHeight  - CFNumber for the height of this plane in pixels.  Required for image planes. }
var kIOSurfacePlaneHeight: CFStringRef; external name '_kIOSurfacePlaneHeight'; (* attribute const *)
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ kIOSurfacePlaneBytesPerRow    - CFNumber for the bytes per row of this plane.  If not specified, IOSurface will first calculate
                                   the number full elements required on each row (by rounding up), multiplied by the bytes per element
				   for this plane.   That value will then be appropriately aligned. }
var kIOSurfacePlaneBytesPerRow: CFStringRef; external name '_kIOSurfacePlaneBytesPerRow'; (* attribute const *)
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ kIOSurfacePlaneOffset  - CFNumber for the offset into the buffer for this plane.  If not specified then IOSurface
   will lay out each plane sequentially based on the previous plane's allocation size. }
var kIOSurfacePlaneOffset: CFStringRef; external name '_kIOSurfacePlaneOffset'; (* attribute const *)
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ kIOSurfacePlaneSize    - CFNumber for the total data size of this plane.  Defaults to plane height * plane bytes per row if not specified. }
var kIOSurfacePlaneSize: CFStringRef; external name '_kIOSurfacePlaneSize'; (* attribute const *)
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ Optional properties for planar surface buffers }

{ kIOSurfacePlaneBase    - CFNumber for the base offset into the buffer for this plane. Optional, defaults to the plane offset }
var kIOSurfacePlaneBase: CFStringRef; external name '_kIOSurfacePlaneBase'; (* attribute const *)
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ kIOSurfacePlaneBytesPerElement    - CFNumber for the bytes per element of this plane.  Optional, default is 1. }
var kIOSurfacePlaneBytesPerElement: CFStringRef; external name '_kIOSurfacePlaneBytesPerElement'; (* attribute const *)
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ kIOSurfacePlaneElementWidth    - CFNumber for the element width of this plane.  Optional, default is 1. }
var kIOSurfacePlaneElementWidth: CFStringRef; external name '_kIOSurfacePlaneElementWidth'; (* attribute const *)
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ kIOSurfacePlaneElementHeight   - CFNumber for the element height of this plane.  Optional, default is 1. }
var kIOSurfacePlaneElementHeight: CFStringRef; external name '_kIOSurfacePlaneElementHeight'; (* attribute const *)
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ Optional properties global to the entire IOSurface }

{ kIOSurfaceCacheMode		- CFNumber for the CPU cache mode to be used for the allocation.  Default is kIOMapDefaultCache. }
var kIOSurfaceCacheMode: CFStringRef; external name '_kIOSurfaceCacheMode'; (* attribute const *)
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ kIOSurfaceIsGlobal - CFBoolean     If true, the IOSurface may be looked up by any task in the system by its ID. }
var kIOSurfaceIsGlobal: CFStringRef; external name '_kIOSurfaceIsGlobal'; (* attribute const *)
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ kIOSurfacePixelFormat - CFNumber	A 32-bit unsigned integer that stores the traditional Mac OS X buffer format  }
var kIOSurfacePixelFormat: CFStringRef; external name '_kIOSurfacePixelFormat'; (* attribute const *)
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

function IOSurfaceGetTypeID: CFTypeID; external name '_IOSurfaceGetTypeID';
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ Create a brand new IOSurface object }
function IOSurfaceCreate( properties: CFDictionaryRef ): IOSurfaceRef; external name '_IOSurfaceCreate';
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ Perform an atomic lookup and retain of a IOSurface by its IOSurfaceID.
   Note: Performing multiple lookups of the same IOSurface will *NOT* return
   the same IOSurfaceRef.   If you need to compare two IOSurface objects
   for equality, you must either do so by comparing their IOSurfaceIDs, or by 
   using CFEqual(). }
function IOSurfaceLookup( csid: IOSurfaceID ): IOSurfaceRef; external name '_IOSurfaceLookup';
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ Retrieve the unique IOSurfaceID value for a IOSurface }
function IOSurfaceGetID( buffer: IOSurfaceRef ): IOSurfaceID; external name '_IOSurfaceGetID';
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)
	
const
// If you are not going to modify the data while you hold the lock, you should set this flag to avoid invalidating
	// any existing caches of the buffer contents.  This flag should be passed both to the lock and unlock functions.
	// Non-symmentrical usage of this flag will result in undefined behavior.
	kIOSurfaceLockReadOnly = $00000001;
	
	// If you want to detect/avoid a potentially expensive paging operation (such as readback from a GPU to system memory)
	// when you lock the buffer, you may include this flag.   If locking the buffer requires a readback, the lock will
	// fail with an error return of kIOReturnCannotLock.
	kIOSurfaceLockAvoidSync = $00000002;
			
{ "Lock" or "Unlock" a IOSurface for reading or writing.

    The term "lock" is used loosely in this context, and is simply used along with the
    "unlock" information to put a bound on CPU access to the raw IOSurface data.
    
    If the seed parameter is non-NULL, IOSurfaceLock() will store the buffer's
    internal modification seed value at the time you made the lock call.   You can compare
    this value to a value returned previously to determine of the contents of the buffer
    has been changed since the last lock.
    
    In the case of IOSurfaceUnlock(), the seed value returned will be the internal
    seed value at the time of the unlock.  If you locked the buffer for writing, this value
    will be incremented as the unlock is performed and the new value will be returned.
    
    See the kIOSurfaceLock enums for more information.
    
    Note: Locking and unlocking a IOSurface is not a particularly cheap operation,
    so care should be taken to avoid the calls whenever possible.   The seed values are 
    particularly useful for keeping a cache of the buffer contents.
}
function IOSurfaceLock( buffer: IOSurfaceRef; options: UInt32; var seed: UInt32 ): IOReturn; external name '_IOSurfaceLock';
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)	
function IOSurfaceUnlock( buffer: IOSurfaceRef; options: UInt32; var seed: UInt32 ): IOReturn; external name '_IOSurfaceUnlock';
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ These routines are all fairly self explanatory.  0 is returned if buffer is invalid or NULL }
function IOSurfaceGetAllocSize( buffer: IOSurfaceRef ): size_t; external name '_IOSurfaceGetAllocSize';
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

function IOSurfaceGetWidth( buffer: IOSurfaceRef ): size_t; external name '_IOSurfaceGetWidth';
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)
	
function IOSurfaceGetHeight( buffer: IOSurfaceRef ): size_t; external name '_IOSurfaceGetHeight';
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

function IOSurfaceGetBytesPerElement( buffer: IOSurfaceRef ): size_t; external name '_IOSurfaceGetBytesPerElement';
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

function IOSurfaceGetBytesPerRow( buffer: IOSurfaceRef ): size_t; external name '_IOSurfaceGetBytesPerRow';
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

function IOSurfaceGetBaseAddress( buffer: IOSurfaceRef ): UnivPtr; external name '_IOSurfaceGetBaseAddress';
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

function IOSurfaceGetElementWidth( buffer: IOSurfaceRef ): size_t; external name '_IOSurfaceGetElementWidth';
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

function IOSurfaceGetElementHeight( buffer: IOSurfaceRef ): size_t; external name '_IOSurfaceGetElementHeight';
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

function IOSurfaceGetPixelFormat( buffer: IOSurfaceRef ): OSType; external name '_IOSurfaceGetPixelFormat';
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ This will return the current seed value of the buffer and is a cheap call to make to see
   if the contents of the buffer have changed since the last lock/unlock. }
function IOSurfaceGetSeed( buffer: IOSurfaceRef ): UInt32; external name '_IOSurfaceGetSeed';
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ Return the number of planes in this buffer.  May be 0.   Returns 0 for an invalid or NULL buffer pointer. }
function IOSurfaceGetPlaneCount( buffer: IOSurfaceRef ): size_t; external name '_IOSurfaceGetPlaneCount';
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ These routines return information about a particular plane of a IOSurface.   

   If the planeIndex is greater than or equal to the plane count of the IOSurface, zero
   is returned.... with one exception.   If this IOSurface has zero planes and a planeIndex
   of zero is passed in, the routines function just like the non-planar APIs.  This is to allow
   higher level code to treat planar and non-planar buffers is a more uniform fashion. }

function IOSurfaceGetWidthOfPlane( buffer: IOSurfaceRef; planeIndex: size_t ): size_t; external name '_IOSurfaceGetWidthOfPlane';
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

function IOSurfaceGetHeightOfPlane( buffer: IOSurfaceRef; planeIndex: size_t ): size_t; external name '_IOSurfaceGetHeightOfPlane';
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

function IOSurfaceGetBytesPerElementOfPlane( buffer: IOSurfaceRef; planeIndex: size_t ): size_t; external name '_IOSurfaceGetBytesPerElementOfPlane';
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

function IOSurfaceGetBytesPerRowOfPlane( buffer: IOSurfaceRef; planeIndex: size_t ): size_t; external name '_IOSurfaceGetBytesPerRowOfPlane';
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

function IOSurfaceGetBaseAddressOfPlane( buffer: IOSurfaceRef; planeIndex: size_t ): UnivPtr; external name '_IOSurfaceGetBaseAddressOfPlane';
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

function IOSurfaceGetElementWidthOfPlane( buffer: IOSurfaceRef; planeIndex: size_t ): size_t; external name '_IOSurfaceGetElementWidthOfPlane';
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

function IOSurfaceGetElementHeightOfPlane( buffer: IOSurfaceRef; planeIndex: size_t ): size_t; external name '_IOSurfaceGetElementHeightOfPlane';
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ These calls let you attach CF property list types to a IOSurface buffer.  These calls are 
   expensive (they essentially must serialize the data into the kernel) and thus should be avoided whenever
   possible.   Note:  These functions can not be used to change the underlying surface properties. }
procedure IOSurfaceSetValue( buffer: IOSurfaceRef; key: CFStringRef; value: CFTypeRef ); external name '_IOSurfaceSetValue';
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

function IOSurfaceCopyValue( buffer: IOSurfaceRef; key: CFStringRef ): CFTypeRef; external name '_IOSurfaceCopyValue';
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

procedure IOSurfaceRemoveValue( buffer: IOSurfaceRef; key: CFStringRef ); external name '_IOSurfaceRemoveValue';
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ This call lets you get a mach_port_t that holds a reference to the IOSurface. This is useful 
   if you need to atomically or securely pass an IOSurface to another task without making the surface global to
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   the entire system.  The returned port must be deallocated with mach_port_deallocate or the equivalent.  
   Note: Any live mach ports created from an IOSurfaceRef implicity increase the IOSurface's global use
   count by one until the port is deleted. }
=======
   the entire system.  The returned port must be deallocated with mach_port_deallocate or the equivalent.  }
>>>>>>> graemeg/cpstrnew
=======
   the entire system.  The returned port must be deallocated with mach_port_deallocate or the equivalent.  }
>>>>>>> graemeg/cpstrnew
=======
   the entire system.  The returned port must be deallocated with mach_port_deallocate or the equivalent.  }
>>>>>>> graemeg/cpstrnew
=======
   the entire system.  The returned port must be deallocated with mach_port_deallocate or the equivalent.  }
>>>>>>> origin/cpstrnew
function IOSurfaceCreateMachPort( buffer: IOSurfaceRef ): mach_port_t; external name '_IOSurfaceCreateMachPort';
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ This call lets you take a mach_port_t created via IOSurfaceCreatePort() and recreate an IOSurfaceRef from it.
   Note: This call does NOT destroy the port. }
function IOSurfaceLookupFromMachPort( port: mach_port_t ): IOSurfaceRef; external name '_IOSurfaceLookupFromMachPort';
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef XPC_TRANSLATED}
{ This call lets you get an xpc_object_t that holds a reference to the IOSurface.
   Note: Any live XPC objects created from an IOSurfaceRef implicity increase the IOSurface's global use
   count by one until the object is destroyed. }
function IOSurfaceCreateXPCObject( aSurface: IOSurfaceRef ): xpc_object_t; external name '_IOSurfaceCreateXPCObject';
(* IOSFC_AVAILABLE_STARTING(__MAC_10_7, __IPHONE_NA) *)

{ This call lets you take an xpc_object_t created via IOSurfaceCreatePort() and recreate an IOSurfaceRef from it. }
function IOSurfaceLookupFromXPCObject( xobj: xpc_object_t ): IOSurfaceRef; external name '_IOSurfaceLookupFromXPCObject';
(* IOSFC_AVAILABLE_STARTING(__MAC_10_7, __IPHONE_NA) *)
{$endif} {XPC_TRANSLATED}

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{ 
   IOSurfaceGetPropertyMaximum() will return the maximum of a given property that is guaranteed to be 
   compatible with all of the current devices (GPUs, etc.) in the system.   The most important ones being:
   
   kIOSurfaceBytesPerRow
   kIOSurfaceWidth
   kIOSurfaceHeight
   kIOSurfacePlaneBytesPerRow
   kIOSurfacePlaneWidth
   kIOSurfacePlaneHeight
   
   For the width and height properties, the maximum values are the largest that are guaranteed to work
   for both reading and writing.   In OpenGL terms this translates into the largest size that will work
   for both textures and render targets.
   
   This function returns 0 for properties that have no predefined limit or where the concept of a limit
   would be considered invalid (such as kIOSurfacePixelFormat).
      
}   
function IOSurfaceGetPropertyMaximum( property: CFStringRef ): size_t; external name '_IOSurfaceGetPropertyMaximum';
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
{ 
   If a property has a particular alignment requirement, then IOSurfaceGetPropertyAlignment() will return it.  
   If the property has no alignment requirement then 1 will be returned.   The following properties 
   should always be aligned if you choose calculate them yourself:
   
   kIOSurfaceBytesPerRow
   kIOSurfaceOffset
   
   kIOSurfacePlaneBase
   kIOSurfacePlaneOffset
   kIOSurfacePlaneBytesPerRow
   
}   
function IOSurfaceGetPropertyAlignment( property: CFStringRef ): size_t; external name '_IOSurfaceGetPropertyAlignment';
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{ This is a convenience function to automatically align property values.  For properties with no alignment
   requirements, the original value will be returned. }
function IOSurfaceAlignProperty( property: CFStringRef; value: size_t ): size_t; external name '_IOSurfaceAlignProperty';
(* IOSFC_AVAILABLE_STARTING(__MAC_10_6, __IPHONE_NA) *)

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
