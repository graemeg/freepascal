{
 * Copyright (c) 1998-2009 Apple Inc. All Rights Reserved.
 *
 * @APPLE_LICENSE_HEADER_START@
 * 
 * This file contains Original Code and/or Modifications of Original Code
 * as defined in and that are subject to the Apple Public Source License
 * Version 2.0 (the 'License'). You may not use this file except in
 * compliance with the License. Please obtain a copy of the License at
 * http://www.opensource.apple.com/apsl/ and read it before using this
 * file.
 * 
 * The Original Code and all software distributed under the License are
 * distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER
 * EXPRESS OR IMPLIED, AND APPLE HEREBY DISCLAIMS ALL SUCH WARRANTIES,
 * INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT.
 * Please see the License for the specific language governing rights and
 * limitations under the License.
 * 
 * @APPLE_LICENSE_HEADER_END@
 }
{  Initial Pascal Translation:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit DADisk;
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
uses MacTypes,DASession,CFBase,CFDictionary,CFString;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}


var kDADiskDescriptionVolumeKindKey: CFStringRef; external name '_kDADiskDescriptionVolumeKindKey'; (* attribute const *)      { ( CFString     ) }
var kDADiskDescriptionVolumeMountableKey: CFStringRef; external name '_kDADiskDescriptionVolumeMountableKey'; (* attribute const *) { ( CFBoolean    ) }
var kDADiskDescriptionVolumeNameKey: CFStringRef; external name '_kDADiskDescriptionVolumeNameKey'; (* attribute const *)      { ( CFString     ) }
var kDADiskDescriptionVolumeNetworkKey: CFStringRef; external name '_kDADiskDescriptionVolumeNetworkKey'; (* attribute const *)   { ( CFBoolean    ) }
var kDADiskDescriptionVolumePathKey: CFStringRef; external name '_kDADiskDescriptionVolumePathKey'; (* attribute const *)      { ( CFURL        ) }
var kDADiskDescriptionVolumeUUIDKey: CFStringRef; external name '_kDADiskDescriptionVolumeUUIDKey'; (* attribute const *)      { ( CFUUID       ) }

var kDADiskDescriptionMediaBlockSizeKey: CFStringRef; external name '_kDADiskDescriptionMediaBlockSizeKey'; (* attribute const *)  { ( CFNumber     ) }
var kDADiskDescriptionMediaBSDMajorKey: CFStringRef; external name '_kDADiskDescriptionMediaBSDMajorKey'; (* attribute const *)   { ( CFNumber     ) }
var kDADiskDescriptionMediaBSDMinorKey: CFStringRef; external name '_kDADiskDescriptionMediaBSDMinorKey'; (* attribute const *)   { ( CFNumber     ) }
var kDADiskDescriptionMediaBSDNameKey: CFStringRef; external name '_kDADiskDescriptionMediaBSDNameKey'; (* attribute const *)    { ( CFString     ) }
var kDADiskDescriptionMediaBSDUnitKey: CFStringRef; external name '_kDADiskDescriptionMediaBSDUnitKey'; (* attribute const *)    { ( CFNumber     ) }
var kDADiskDescriptionMediaContentKey: CFStringRef; external name '_kDADiskDescriptionMediaContentKey'; (* attribute const *)    { ( CFString     ) }
var kDADiskDescriptionMediaEjectableKey: CFStringRef; external name '_kDADiskDescriptionMediaEjectableKey'; (* attribute const *)  { ( CFBoolean    ) }
var kDADiskDescriptionMediaIconKey: CFStringRef; external name '_kDADiskDescriptionMediaIconKey'; (* attribute const *)       { ( CFDictionary ) }
var kDADiskDescriptionMediaKindKey: CFStringRef; external name '_kDADiskDescriptionMediaKindKey'; (* attribute const *)       { ( CFString     ) }
var kDADiskDescriptionMediaLeafKey: CFStringRef; external name '_kDADiskDescriptionMediaLeafKey'; (* attribute const *)       { ( CFBoolean    ) }
var kDADiskDescriptionMediaNameKey: CFStringRef; external name '_kDADiskDescriptionMediaNameKey'; (* attribute const *)       { ( CFString     ) }
var kDADiskDescriptionMediaPathKey: CFStringRef; external name '_kDADiskDescriptionMediaPathKey'; (* attribute const *)       { ( CFString     ) }
var kDADiskDescriptionMediaRemovableKey: CFStringRef; external name '_kDADiskDescriptionMediaRemovableKey'; (* attribute const *)  { ( CFBoolean    ) }
var kDADiskDescriptionMediaSizeKey: CFStringRef; external name '_kDADiskDescriptionMediaSizeKey'; (* attribute const *)       { ( CFNumber     ) }
var kDADiskDescriptionMediaTypeKey: CFStringRef; external name '_kDADiskDescriptionMediaTypeKey'; (* attribute const *)       { ( CFString     ) }
var kDADiskDescriptionMediaUUIDKey: CFStringRef; external name '_kDADiskDescriptionMediaUUIDKey'; (* attribute const *)       { ( CFUUID       ) }
var kDADiskDescriptionMediaWholeKey: CFStringRef; external name '_kDADiskDescriptionMediaWholeKey'; (* attribute const *)      { ( CFBoolean    ) }
var kDADiskDescriptionMediaWritableKey: CFStringRef; external name '_kDADiskDescriptionMediaWritableKey'; (* attribute const *)   { ( CFBoolean    ) }

var kDADiskDescriptionDeviceGUIDKey: CFStringRef; external name '_kDADiskDescriptionDeviceGUIDKey'; (* attribute const *)      { ( CFData       ) }
var kDADiskDescriptionDeviceInternalKey: CFStringRef; external name '_kDADiskDescriptionDeviceInternalKey'; (* attribute const *)  { ( CFBoolean    ) }
var kDADiskDescriptionDeviceModelKey: CFStringRef; external name '_kDADiskDescriptionDeviceModelKey'; (* attribute const *)     { ( CFString     ) }
var kDADiskDescriptionDevicePathKey: CFStringRef; external name '_kDADiskDescriptionDevicePathKey'; (* attribute const *)      { ( CFString     ) }
var kDADiskDescriptionDeviceProtocolKey: CFStringRef; external name '_kDADiskDescriptionDeviceProtocolKey'; (* attribute const *)  { ( CFString     ) }
var kDADiskDescriptionDeviceRevisionKey: CFStringRef; external name '_kDADiskDescriptionDeviceRevisionKey'; (* attribute const *)  { ( CFString     ) }
var kDADiskDescriptionDeviceUnitKey: CFStringRef; external name '_kDADiskDescriptionDeviceUnitKey'; (* attribute const *)      { ( CFNumber     ) }
var kDADiskDescriptionDeviceVendorKey: CFStringRef; external name '_kDADiskDescriptionDeviceVendorKey'; (* attribute const *)    { ( CFString     ) }

var kDADiskDescriptionBusNameKey: CFStringRef; external name '_kDADiskDescriptionBusNameKey'; (* attribute const *)         { ( CFString     ) }
var kDADiskDescriptionBusPathKey: CFStringRef; external name '_kDADiskDescriptionBusPathKey'; (* attribute const *)         { ( CFString     ) }


{!
 * @typedef    DADiskRef
 * Type of a reference to DADisk instances.
 }

type
	DADiskRef = ^SInt32; { an opaque type }

{!
 * @function   DADiskGetTypeID
 * @abstract   Returns the type identifier of all DADisk instances.
 }

function DADiskGetTypeID: CFTypeID; external name '_DADiskGetTypeID';

{!
 * @function   DADiskCreateFromBSDName
 * @abstract   Creates a new disk object.
 * @param      allocator The allocator object to be used to allocate memory.
 * @param      session   The DASession in which to contact Disk Arbitration.
 * @param      name      The BSD device name.
 * @result     A reference to a new DADisk.
 * @discussion
 * The caller of this function receives a reference to the returned object.  The
 * caller also implicitly retains the object and is responsible for releasing it
 * with CFRelease().
 }

function DADiskCreateFromBSDName( allocator: CFAllocatorRef; session: DASessionRef; name: ConstCStringPtr ): DADiskRef; external name '_DADiskCreateFromBSDName';

(*
Requires IOKit translation

{!
 * @function   DADiskCreateFromIOMedia
 * @abstract   Creates a new disk object.
 * @param      allocator The allocator object to be used to allocate memory.
 * @param      session   The DASession in which to contact Disk Arbitration.
 * @param      media     The I/O Kit media object.
 * @result     A reference to a new DADisk.
 * @discussion
 * The caller of this function receives a reference to the returned object.  The
 * caller also implicitly retains the object and is responsible for releasing it
 * with CFRelease().
 }

function DADiskCreateFromIOMedia( allocator: CFAllocatorRef; session: DASessionRef; media: io_service_t ): DADiskRef; external name '_DADiskCreateFromIOMedia';
*)

{!
 * @function   DADiskGetBSDName
 * @abstract   Obtains the BSD device name for the specified disk.
 * @param      disk The DADisk for which to obtain the BSD device name.
 * @result     The disk's BSD device name.
 * @discussion
 * The BSD device name can be used with opendev() to open the BSD device.
 }

function DADiskGetBSDName( disk: DADiskRef ): CStringPtr; external name '_DADiskGetBSDName';

(*
Requires IOKit translation

{!
 * @function   DADiskCopyIOMedia
 * @abstract   Obtains the I/O Kit media object for the specified disk.
 * @param      disk The DADisk for which to obtain the I/O Kit media object.
 * @result     The disk's I/O Kit media object.
 * @discussion
 * The caller of this function receives a reference to the returned object.  The
 * caller also implicitly retains the object and is responsible for releasing it
 * with IOObjectRelease().
 }

function DADiskCopyIOMedia( disk: DADiskRef ): io_service_t; external name '_DADiskCopyIOMedia';
*)

{!
 * @function   DADiskCopyDescription
 * @abstract   Obtains the Disk Arbitration description of the specified disk.
 * @param      disk The DADisk for which to obtain the Disk Arbitration description.
 * @result     The disk's Disk Arbitration description.
 * @discussion
 * This function will contact Disk Arbitration to acquire the latest description
 * of the specified disk, unless this function is called on a disk object passed
 * within the context of a registered callback, in which case the description is
 * current as of that callback event.
 *
 * The caller of this function receives a reference to the returned object.  The
 * caller also implicitly retains the object and is responsible for releasing it
 * with CFRelease().
 }

function DADiskCopyDescription( disk: DADiskRef ): CFDictionaryRef; external name '_DADiskCopyDescription';

{!
 * @function   DADiskCopyWholeDisk
 * @abstract   Obtain the associated whole disk object for the specified disk.
 * @param      disk The disk object.
 * @result     The disk's associated whole disk object.
 * @discussion
 * The caller of this function receives a reference to the returned object.  The
 * caller also implicitly retains the object and is responsible for releasing it
 * with CFRelease().
 }

function DADiskCopyWholeDisk( disk: DADiskRef ): DADiskRef; external name '_DADiskCopyWholeDisk';

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
