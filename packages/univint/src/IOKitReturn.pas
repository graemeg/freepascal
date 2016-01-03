{
 * Copyright (c) 1998-2002 Apple Computer, Inc. All rights reserved.
 *
 * @APPLE_OSREFERENCE_LICENSE_HEADER_START@
 * 
 * This file contains Original Code and/or Modifications of Original Code
 * as defined in and that are subject to the Apple Public Source License
 * Version 2.0 (the 'License'). You may not use this file except in
 * compliance with the License. The rights granted to you under the License
 * may not be used to create, or enable the creation or redistribution of,
 * unlawful or unlicensed copies of an Apple operating system, or to
 * circumvent, violate, or enable the circumvention or violation of, any
 * terms of an Apple operating system software license agreement.
 * 
 * Please obtain a copy of the License at
 * http://www.opensource.apple.com/apsl/ and read it before using this file.
 * 
 * The Original Code and all software distributed under the License are
 * distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER
 * EXPRESS OR IMPLIED, AND APPLE HEREBY DISCLAIMS ALL SUCH WARRANTIES,
 * INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT.
 * Please see the License for the specific language governing rights and
 * limitations under the License.
 * 
 * @APPLE_OSREFERENCE_LICENSE_HEADER_END@
 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, September 2010 }
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

unit IOKitReturn;
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
uses MacTypes,kern_return,mach_error;
{$endc} {not MACOSALLINCLUDE}


{
 * HISTORY
 }
 
{
 * Core IOReturn values. Others may be family defined.
 }

type
	IOReturn = kern_return_t;

const
  sys_iokit = ((($38) and $3f) shl 26);
  sub_iokit_common = (((0) and $fff) shl 14);
  sub_iokit_usb = (((1) and $fff) shl 14);
  sub_iokit_firewire = (((2) and $fff) shl 14);
  sub_iokit_block_storage = (((4) and $fff) shl 14);
  sub_iokit_graphics = (((5) and $fff) shl 14);
  sub_iokit_networking = (((6) and $fff) shl 14);
  sub_iokit_bluetooth = (((8) and $fff) shl 14);
  sub_iokit_pmu = (((9) and $fff) shl 14);
  sub_iokit_acpi = (((10) and $fff) shl 14);
  sub_iokit_smbus = (((11) and $fff) shl 14);
  sub_iokit_ahci = (((12) and $fff) shl 14);
  sub_iokit_powermanagement = (((13) and $fff) shl 14);
//#define sub_iokit_hidsystem             (((14) and $fff) shl 14)
//#define sub_iokit_pccard                (((21) and $fff) shl 14)

  sub_iokit_vendor_specific = (((-2) and $fff) shl 14);
  sub_iokit_reserved = (((-1) and $fff) shl 14);


function iokit_common_err(ret: IOReturn): IOReturn; inline;
function iokit_family_err(sub, ret: IOReturn): IOReturn; inline;
function iokit_vendor_specific_err(ret: IOReturn): IOReturn; inline;

const
  kIOReturnSuccess = KERN_SUCCESS;            // OK
  kIOReturnError = (sys_iokit or sub_iokit_common or $2bc); // general error 	
  kIOReturnNoMemory = (sys_iokit or sub_iokit_common or $2bd); // can't allocate memory 
  kIOReturnNoResources = (sys_iokit or sub_iokit_common or $2be); // resource shortage 
  kIOReturnIPCError = (sys_iokit or sub_iokit_common or $2bf); // error during IPC 
  kIOReturnNoDevice = (sys_iokit or sub_iokit_common or $2c0); // no such device 
  kIOReturnNotPrivileged = (sys_iokit or sub_iokit_common or $2c1); // privilege violation 
  kIOReturnBadArgument = (sys_iokit or sub_iokit_common or $2c2); // invalid argument 
  kIOReturnLockedRead = (sys_iokit or sub_iokit_common or $2c3); // device read locked 
  kIOReturnLockedWrite = (sys_iokit or sub_iokit_common or $2c4); // device write locked 
  kIOReturnExclusiveAccess = (sys_iokit or sub_iokit_common or $2c5); // exclusive access and
                                                         //   device already open 
  kIOReturnBadMessageID = (sys_iokit or sub_iokit_common or $2c6); // sent/received messages
                                                         //   had different msg_id
  kIOReturnUnsupported = (sys_iokit or sub_iokit_common or $2c7); // unsupported function 
  kIOReturnVMError = (sys_iokit or sub_iokit_common or $2c8); // misc. VM failure 
  kIOReturnInternalError = (sys_iokit or sub_iokit_common or $2c9); // internal error 
  kIOReturnIOError = (sys_iokit or sub_iokit_common or $2ca); // General I/O error 
//#define kIOReturn???Error      (sys_iokit or sub_iokit_common or $2cb) // ??? 
  kIOReturnCannotLock = (sys_iokit or sub_iokit_common or $2cc); // can't acquire lock
  kIOReturnNotOpen = (sys_iokit or sub_iokit_common or $2cd); // device not open 
  kIOReturnNotReadable = (sys_iokit or sub_iokit_common or $2ce); // read not supported 
  kIOReturnNotWritable = (sys_iokit or sub_iokit_common or $2cf); // write not supported 
  kIOReturnNotAligned = (sys_iokit or sub_iokit_common or $2d0); // alignment error 
  kIOReturnBadMedia = (sys_iokit or sub_iokit_common or $2d1); // Media Error 
  kIOReturnStillOpen = (sys_iokit or sub_iokit_common or $2d2); // device(s) still open 
  kIOReturnRLDError = (sys_iokit or sub_iokit_common or $2d3); // rld failure 
  kIOReturnDMAError = (sys_iokit or sub_iokit_common or $2d4); // DMA failure 
  kIOReturnBusy = (sys_iokit or sub_iokit_common or $2d5); // Device Busy 
  kIOReturnTimeout = (sys_iokit or sub_iokit_common or $2d6); // I/O Timeout 
  kIOReturnOffline = (sys_iokit or sub_iokit_common or $2d7); // device offline 
  kIOReturnNotReady = (sys_iokit or sub_iokit_common or $2d8); // not ready 
  kIOReturnNotAttached = (sys_iokit or sub_iokit_common or $2d9); // device not attached 
  kIOReturnNoChannels = (sys_iokit or sub_iokit_common or $2da); // no DMA channels left
  kIOReturnNoSpace = (sys_iokit or sub_iokit_common or $2db); // no space for data 
//#define kIOReturn???Error      (sys_iokit or sub_iokit_common or $2dc) // ??? 
  kIOReturnPortExists = (sys_iokit or sub_iokit_common or $2dd); // port already exists
  kIOReturnCannotWire = (sys_iokit or sub_iokit_common or $2de); // can't wire down 
                                                         //   physical memory
  kIOReturnNoInterrupt = (sys_iokit or sub_iokit_common or $2df); // no interrupt attached
  kIOReturnNoFrames = (sys_iokit or sub_iokit_common or $2e0); // no DMA frames enqueued
  kIOReturnMessageTooLarge = (sys_iokit or sub_iokit_common or $2e1); // oversized msg received
                                                         //   on interrupt port
  kIOReturnNotPermitted = (sys_iokit or sub_iokit_common or $2e2); // not permitted
  kIOReturnNoPower = (sys_iokit or sub_iokit_common or $2e3); // no power to device
  kIOReturnNoMedia = (sys_iokit or sub_iokit_common or $2e4); // media not present
  kIOReturnUnformattedMedia = (sys_iokit or sub_iokit_common or $2e5); // media not formatted
  kIOReturnUnsupportedMode = (sys_iokit or sub_iokit_common or $2e6); // no such mode
  kIOReturnUnderrun = (sys_iokit or sub_iokit_common or $2e7); // data underrun
  kIOReturnOverrun = (sys_iokit or sub_iokit_common or $2e8); // data overrun
  kIOReturnDeviceError = (sys_iokit or sub_iokit_common or $2e9); // the device is not working properly!
  kIOReturnNoCompletion = (sys_iokit or sub_iokit_common or $2ea); // a completion routine is required
  kIOReturnAborted = (sys_iokit or sub_iokit_common or $2eb); // operation aborted
  kIOReturnNoBandwidth = (sys_iokit or sub_iokit_common or $2ec); // bus bandwidth would be exceeded
  kIOReturnNotResponding = (sys_iokit or sub_iokit_common or $2ed); // device not responding
  kIOReturnIsoTooOld = (sys_iokit or sub_iokit_common or $2ee); // isochronous I/O request for distant past!
  kIOReturnIsoTooNew = (sys_iokit or sub_iokit_common or $2ef); // isochronous I/O request for distant future
  kIOReturnNotFound = (sys_iokit or sub_iokit_common or $2f0); // data was not found
  kIOReturnInvalid = (sys_iokit or sub_iokit_common or $1);   // should never be seen

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
implementation


{$push}
{$R-,Q-}

function iokit_common_err(ret: IOReturn): IOReturn; inline;
begin
  iokit_common_err:=(sys_iokit or sub_iokit_common or (ret))
end;

function iokit_family_err(sub, ret: IOReturn): IOReturn; inline;
begin
  iokit_family_err:=(sys_iokit or (sub) or (ret))
end;

function iokit_vendor_specific_err(ret: IOReturn): IOReturn; inline;
begin
  iokit_vendor_specific_err:=(sys_iokit or sub_iokit_vendor_specific or (ret))
end;

{$pop}

end.

{$endc} {not MACOSALLINCLUDE}
