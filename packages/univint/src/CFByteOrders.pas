{	CFByteOrder.h
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	Copyright (c) 1995-2009, Apple Inc. All rights reserved.
}
{   Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{   Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, September 2005 }
{   Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, April 2006, February 2008, October 2009 }
{
    Modified for use with Free Pascal
    Version 308
    Please report any bugs to <gpc@microbizz.nl>
}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
	Copyright (c) 1995-2005, Apple, Inc. All rights reserved.
=======
	Copyright (c) 1995-2009, Apple Inc. All rights reserved.
>>>>>>> origin/fixes_2.4
}
{   Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{   Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, September 2005 }
{   Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, April 2006, February 2008, October 2009 }
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

unit CFByteOrders;
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
=======
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
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
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
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
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
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
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
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


type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CFByteOrder = SIGNEDLONG;
=======
	CFByteOrder = SInt32;
>>>>>>> graemeg/fixes_2_2
=======
	CFByteOrder = SInt32;
>>>>>>> origin/fixes_2_2
=======
	CFByteOrder = SIGNEDLONG;
>>>>>>> origin/fixes_2.4
const
	CFByteOrderUnknown = 0;
	CFByteOrderLittleEndian = 1;
	CFByteOrderBigEndian = 2;

function CFByteOrderGetCurrent: CFByteOrder; inline;
function CFSwapInt16( arg: UInt16 ): UInt16; inline;
function CFSwapInt32( arg: UInt32 ): UInt32; inline;
function CFSwapInt64( arg: UInt64 ): UInt64; inline;

function CFSwapInt16BigToHost( arg: UInt16 ): UInt16; inline;
function CFSwapInt32BigToHost( arg: UInt32 ): UInt32; inline;
function CFSwapInt64BigToHost( arg: UInt64 ): UInt64; inline;
function CFSwapInt16HostToBig( arg: UInt16 ): UInt16; inline;
function CFSwapInt32HostToBig( arg: UInt32 ): UInt32; inline;
function CFSwapInt64HostToBig( arg: UInt64 ): UInt64; inline;

{$ifc TARGET_RT_BIG_ENDIAN}


{$elsec}



{$endc}

function CFSwapInt16LittleToHost( arg: UInt16 ): UInt16; inline;
function CFSwapInt32LittleToHost( arg: UInt32 ): UInt32; inline;
function CFSwapInt64LittleToHost( arg: UInt64 ): UInt64; inline;
function CFSwapInt16HostToLittle( arg: UInt16 ): UInt16; inline;
function CFSwapInt32HostToLittle( arg: UInt32 ): UInt32; inline;
function CFSwapInt64HostToLittle( arg: UInt64 ): UInt64; inline;

{$ifc TARGET_RT_LITTLE_ENDIAN}


{$elsec}



{$endc}

type
	CFSwappedFloat32 = record
		v: UInt32;
	end;
type
	CFSwappedFloat64 = record
		v: UInt64;
	end;

function CFConvertFloat32HostToSwapped( arg: Float32 ): CFSwappedFloat32; inline;
function CFConvertFloat32SwappedToHost( arg: CFSwappedFloat32 ): Float32; inline;
function CFConvertFloat64HostToSwapped( arg: Float64 ): CFSwappedFloat64; inline;
function CFConvertFloat64SwappedToHost( arg: CFSwappedFloat64 ): Float64; inline;
function CFConvertFloatHostToSwapped( arg: Float32 ): CFSwappedFloat32; inline;
function CFConvertFloatSwappedToHost( arg: CFSwappedFloat32 ): Float32; inline;
function CFConvertDoubleHostToSwapped( arg: Float64 ): CFSwappedFloat64; inline;
function CFConvertDoubleSwappedToHost( arg: CFSwappedFloat64 ): Float64; inline;

{$ifc TARGET_RT_LITTLE_ENDIAN}



{$elsec}


{$endc}

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


{$R-}

function CFByteOrderGetCurrent: CFByteOrder; inline;
	var
		x: UInt32 = (CFByteOrderBigEndian shl 24) or CFByteOrderLittleEndian;
begin
	CFByteOrderGetCurrent := CFByteOrder(UInt8Ptr(@x)^);
end;

function CFSwapInt16( arg: UInt16 ): UInt16; inline;
begin
	CFSwapInt16 := (( arg shl 8) and $0FF00) or (( arg shr 8) and $00FF);
end;

function CFSwapInt32( arg: UInt32 ): UInt32; inline;
begin
    CFSwapInt32 := ((arg and $FF) shl 24) or ((arg and $0FF00) shl 8) or ((arg shr 8) and $0FF00) or ((arg shr 24) and $FF);
end;

function CFSwapInt64( arg: UInt64 ): UInt64; inline;
begin
	CFSwapInt64 := (UInt64(CFSwapInt32( arg and $FFFFFFFF )) shl 32) or CFSwapInt32( (arg shr 32) and $FFFFFFFF );
end;

{$ifc TARGET_RT_BIG_ENDIAN}
function CFSwapInt16BigToHost( arg: UInt16 ): UInt16; inline;
begin
  CFSwapInt16BigToHost := arg;
end;

function CFSwapInt32BigToHost( arg: UInt32 ): UInt32; inline;
begin
  CFSwapInt32BigToHost := arg;
end;

function CFSwapInt64BigToHost( arg: UInt64 ): UInt64; inline;
begin
  CFSwapInt64BigToHost := arg;
end;

function CFSwapInt16HostToBig( arg: UInt16 ): UInt16; inline;
begin
  CFSwapInt16HostToBig := arg;
end;

function CFSwapInt32HostToBig( arg: UInt32 ): UInt32; inline;
begin
  CFSwapInt32HostToBig := arg;
end;

function CFSwapInt64HostToBig( arg: UInt64 ): UInt64; inline;
begin
  CFSwapInt64HostToBig := arg;
end;

function CFSwapInt16LittleToHost( arg: UInt16 ): UInt16; inline;
begin
  CFSwapInt16LittleToHost := CFSwapInt16(arg);
end;

function CFSwapInt32LittleToHost( arg: UInt32 ): UInt32; inline;
begin
  CFSwapInt32LittleToHost := CFSwapInt32(arg);
end;

function CFSwapInt64LittleToHost( arg: UInt64 ): UInt64; inline;
begin
  CFSwapInt64LittleToHost := CFSwapInt64(arg);
end;

function CFSwapInt16HostToLittle( arg: UInt16 ): UInt16; inline;
begin
  CFSwapInt16HostToLittle := CFSwapInt16(arg);
end;

function CFSwapInt32HostToLittle( arg: UInt32 ): UInt32; inline;
begin
  CFSwapInt32HostToLittle := CFSwapInt32(arg);
end;

function CFSwapInt64HostToLittle( arg: UInt64 ): UInt64; inline;
begin
  CFSwapInt64HostToLittle := CFSwapInt64(arg);
end;

function CFConvertFloat32HostToSwapped( arg: Float32 ): CFSwappedFloat32; inline;
begin
  CFConvertFloat32HostToSwapped := CFSwappedFloat32(arg);
end;

function CFConvertFloat32SwappedToHost( arg: CFSwappedFloat32 ): Float32; inline;
begin
  CFConvertFloat32SwappedToHost := Float32(arg);
end;

function CFConvertFloat64HostToSwapped( arg: Float64 ): CFSwappedFloat64; inline;
begin
  CFConvertFloat64HostToSwapped := CFSwappedFloat64(arg);
end;

function CFConvertFloat64SwappedToHost( arg: CFSwappedFloat64 ): Float64; inline;
begin
  CFConvertFloat64SwappedToHost := Float64(arg);
end;

function CFConvertFloatHostToSwapped( arg: Float32 ): CFSwappedFloat32; inline;
begin
  CFConvertFloatHostToSwapped := CFSwappedFloat32(arg);
end;

function CFConvertFloatSwappedToHost( arg: CFSwappedFloat32 ): Float32; inline;
begin
  CFConvertFloatSwappedToHost := Float32(arg);
end;

function CFConvertDoubleHostToSwapped( arg: Float64): CFSwappedFloat64; inline;
begin
  CFConvertDoubleHostToSwapped := CFSwappedFloat64(arg);
end;

function CFConvertDoubleSwappedToHost( arg: CFSwappedFloat64 ): Float64; inline;
begin
  CFConvertDoubleSwappedToHost := Float64(arg);
end;

{$elsec}

function CFSwapInt16LittleToHost( arg: UInt16 ): UInt16; inline;
begin
  CFSwapInt16LittleToHost := arg;
end;

function CFSwapInt32LittleToHost( arg: UInt32 ): UInt32; inline;
begin
  CFSwapInt32LittleToHost := arg;
end;

function CFSwapInt64LittleToHost( arg: UInt64 ): UInt64; inline;
begin
  CFSwapInt64LittleToHost := arg;
end;

function CFSwapInt16HostToLittle( arg: UInt16 ): UInt16; inline;
begin
  CFSwapInt16HostToLittle := arg;
end;

function CFSwapInt32HostToLittle( arg: UInt32 ): UInt32; inline;
begin
  CFSwapInt32HostToLittle := arg;
end;

function CFSwapInt64HostToLittle( arg: UInt64 ): UInt64; inline;
begin
  CFSwapInt64HostToLittle := arg;
end;

function CFSwapInt16BigToHost( arg: UInt16 ): UInt16; inline;
begin
  CFSwapInt16BigToHost := CFSwapInt16(arg);
end;

function CFSwapInt32BigToHost( arg: UInt32 ): UInt32; inline;
begin
  CFSwapInt32BigToHost := CFSwapInt32(arg);
end;

function CFSwapInt64BigToHost( arg: UInt64 ): UInt64; inline;
begin
  CFSwapInt64BigToHost := CFSwapInt64(arg);
end;

function CFSwapInt16HostToBig( arg: UInt16 ): UInt16; inline;
begin
  CFSwapInt16HostToBig := CFSwapInt16(arg);
end;

function CFSwapInt32HostToBig( arg: UInt32 ): UInt32; inline;
begin
  CFSwapInt32HostToBig := CFSwapInt32(arg);
end;

function CFSwapInt64HostToBig( arg: UInt64 ): UInt64; inline;
begin
  CFSwapInt64HostToBig := CFSwapInt64(arg);
end;

function CFConvertFloat32HostToSwapped( arg: Float32 ): CFSwappedFloat32; inline;
begin
  CFConvertFloat32HostToSwapped.v := CFSwapInt32(CFSwappedFloat32(arg).v);
end;

function CFConvertFloat32SwappedToHost( arg: CFSwappedFloat32 ): Float32; inline;
begin
  CFConvertFloat32SwappedToHost := Float32(CFSwappedFloat32(CFSwapInt32(arg.v)));
end;

function CFConvertFloat64HostToSwapped( arg: Float64 ): CFSwappedFloat64; inline;
begin
  CFConvertFloat64HostToSwapped.v := CFSwapInt64(CFSwappedFloat64(arg).v);
end;

function CFConvertFloat64SwappedToHost( arg: CFSwappedFloat64 ): Float64; inline;
begin
  CFConvertFloat64SwappedToHost := Float64(CFSwappedFloat64(CFSwapInt64(arg.v)));
end;

function CFConvertFloatHostToSwapped( arg: Float32 ): CFSwappedFloat32; inline;
begin
  CFConvertFloatHostToSwapped.v := CFSwapInt32(CFSwappedFloat32(arg).v);
end;

function CFConvertFloatSwappedToHost( arg: CFSwappedFloat32 ): Float32; inline;
begin
  CFConvertFloatSwappedToHost := Float32(CFSwappedFloat32(CFSwapInt32(arg.v)));
end;

function CFConvertDoubleHostToSwapped( arg: Float64 ): CFSwappedFloat64; inline;
begin
  CFConvertDoubleHostToSwapped.v := CFSwapInt64(CFSwappedFloat64(arg).v);
end;

function CFConvertDoubleSwappedToHost( arg: CFSwappedFloat64 ): Float64; inline;
begin
  CFConvertDoubleSwappedToHost := Float64(CFSwappedFloat64(CFSwapInt64(arg.v)));
end;
{$endc}


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
