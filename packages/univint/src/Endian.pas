{
     File:       CarbonCore/Endian.h
 
     Contains:   Endian swapping utilties
<<<<<<< HEAD
<<<<<<< HEAD
                 The contents of this header file are deprecated.
                 Use CFByteOrder API instead.
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Copyright:  © 1997-2011 by Apple Inc. All rights reserved.
}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
     Version:    CarbonCore-859.2~1
 
     Copyright:  © 1997-2008 by Apple Computer, Inc., all rights reserved
=======
=======
>>>>>>> origin/fixes_2_2
 
     Version:    CarbonCore-859.2~1
 
<<<<<<< HEAD
     Copyright:  © 1997-2005 by Apple Computer, Inc., all rights reserved
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  © 1997-2008 by Apple Computer, Inc., all rights reserved
>>>>>>> origin/fixes_2.4
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
                     http://www.freepascal.org/bugs.html
 
}
{   Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, November 2005 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
{   Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> origin/fixes_2.4
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

unit Endian;
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
=======
{$elsec}
>>>>>>> origin/cpstrnew
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
<<<<<<< HEAD
<<<<<<< HEAD
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
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
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
uses MacTypes;
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


{$ALIGN MAC68K}

{
    This file provides Endian Flipping routines for dealing with converting data
    between Big-Endian and Little-Endian machines.  These routines are useful
    when writing code to compile for both Big and Little Endian machines and  
    which must handle other endian number formats, such as reading or writing 
    to a file or network packet.
    
    These routines are named as follows:
    
        Endian<U><W>_<S>to<D>

    where
        <U> is whether the integer is signed ('S') or unsigned ('U')
        <W> is integer bit width: 16, 32, or 64 
        <S> is the source endian format: 'B' for big, 'L' for little, or 'N' for native
        <D> is the destination endian format: 'B' for big, 'L' for little, or 'N' for native
    
    For example, to convert a Big Endian 32-bit unsigned integer to the current native format use:
        
        long i = EndianU32_BtoN(data);
        
    This file is set up so that the function macro to nothing when the target runtime already
    is the desired format (e.g. on Big Endian machines, EndianU32_BtoN() macros away).
            
    If long long's are not supported, you cannot get 64-bit quantities as a single value.
    The macros are not defined in that case.

    For gcc, the macros build on top of the inline byte swapping
    routines from <libkern/OSByteOrder.h>, which may have better performance.
    
    
                                <<< W A R N I N G >>>
    
    It is very important not to put any autoincrements inside the macros.  This 
    will produce erroneous results because each time the address is accessed in the macro, 
    the increment occurs.
    
 }
 // Macros might be better solutions
function Endian16_Swap( arg: UInt16 ): UInt16; inline;
function Endian32_Swap( arg: UInt32 ): UInt32; inline;
function Endian64_Swap_Pascal( arg: UInt64 ): UInt64; inline;
function EndianS16_Swap( arg: SInt16 ): SInt16; inline;
function EndianS32_Swap( arg: SInt32 ): SInt32; inline;
function EndianS64_Swap( arg: SInt64 ): SInt64; inline;


function Endian64_Swap( arg: UInt64 ): UInt64; inline;
//  Macro away no-op functions

{$ifc TARGET_RT_BIG_ENDIAN}

function EndianS16_BtoN( arg: SInt16 ): SInt16; inline;
function EndianS16_NtoB( arg: SInt16 ): SInt16; inline;
function EndianU16_BtoN( arg: UInt16 ): UInt16; inline;
function EndianU16_NtoB( arg: UInt16 ): UInt16; inline;
function EndianS32_BtoN( arg: SInt32 ): SInt32; inline;
function EndianS32_NtoB( arg: SInt32 ): SInt32; inline;
function EndianU32_BtoN( arg: UInt32 ): UInt32; inline;
function EndianU32_NtoB( arg: UInt32 ): UInt32; inline;
function EndianS64_BtoN( arg: SInt64 ): SInt64; inline;
function EndianS64_NtoB( arg: SInt64 ): SInt64; inline;
function EndianU64_BtoN( arg: UInt64 ): UInt64; inline;
function EndianU64_NtoB( arg: UInt64 ): UInt64; inline;

{$elsec}

function EndianS16_LtoN( arg: SInt16 ): SInt16; inline;
function EndianS16_NtoL( arg: SInt16 ): SInt16; inline;
function EndianU16_LtoN( arg: UInt16 ): UInt16; inline;
function EndianU16_NtoL( arg: UInt16 ): UInt16; inline;
function EndianS32_LtoN( arg: SInt32 ): SInt32; inline;
function EndianS32_NtoL( arg: SInt32 ): SInt32; inline;
function EndianU32_LtoN( arg: UInt32 ): UInt32; inline;
function EndianU32_NtoL( arg: UInt32 ): UInt32; inline;
function EndianS64_LtoN( arg: SInt64 ): SInt64; inline;
function EndianS64_NtoL( arg: SInt64 ): SInt64; inline;
function EndianU64_LtoN( arg: UInt64 ): UInt64; inline;
function EndianU64_NtoL( arg: UInt64 ): UInt64; inline;

{$endc}

//  Map native to actual

{$ifc TARGET_RT_BIG_ENDIAN}

function EndianS16_LtoN( arg: SInt16 ): SInt16; inline;
function EndianS16_NtoL( arg: SInt16 ): SInt16; inline;
function EndianU16_LtoN( arg: UInt16 ): UInt16; inline;
function EndianU16_NtoL( arg: UInt16 ): UInt16; inline;
function EndianS32_LtoN( arg: SInt32 ): SInt32; inline;
function EndianS32_NtoL( arg: SInt32 ): SInt32; inline;
function EndianU32_LtoN( arg: UInt32 ): UInt32; inline;
function EndianU32_NtoL( arg: UInt32 ): UInt32; inline;
function EndianS64_LtoN( arg: SInt64 ): SInt64; inline;
function EndianS64_NtoL( arg: SInt64 ): SInt64; inline;
function EndianU64_LtoN( arg: UInt64 ): UInt64; inline;
function EndianU64_NtoL( arg: UInt64 ): UInt64; inline;

{$elsec}

function EndianS16_BtoN( arg: SInt16 ): SInt16; inline;
function EndianS16_NtoB( arg: SInt16 ): SInt16; inline;
function EndianU16_BtoN( arg: UInt16 ): UInt16; inline;
function EndianU16_NtoB( arg: UInt16 ): UInt16; inline;
function EndianS32_BtoN( arg: SInt32 ): SInt32; inline;
function EndianS32_NtoB( arg: SInt32 ): SInt32; inline;
function EndianU32_BtoN( arg: UInt32 ): UInt32; inline;
function EndianU32_NtoB( arg: UInt32 ): UInt32; inline;
function EndianS64_BtoN( arg: SInt64 ): SInt64; inline;
function EndianS64_NtoB( arg: SInt64 ): SInt64; inline;
function EndianU64_BtoN( arg: UInt64 ): UInt64; inline;
function EndianU64_NtoB( arg: UInt64 ): UInt64; inline;

{$endc}

//     Implement *LtoB and *BtoL

function EndianS16_LtoB( arg: SInt16 ): SInt16; inline;
function EndianS16_BtoL( arg: SInt16 ): SInt16; inline;
function EndianU16_LtoB( arg: UInt16 ): UInt16; inline;
function EndianU16_BtoL( arg: UInt16 ): UInt16; inline;
function EndianS32_LtoB( arg: SInt32 ): SInt32; inline;
function EndianS32_BtoL( arg: SInt32 ): SInt32; inline;
function EndianU32_LtoB( arg: UInt32 ): UInt32; inline;
function EndianU32_BtoL( arg: UInt32 ): UInt32; inline;
function EndianS64_LtoB( arg: SInt64 ): SInt64; inline;
function EndianS64_BtoL( arg: SInt64 ): SInt64; inline;
function EndianU64_LtoB( arg: UInt64 ): UInt64; inline;
function EndianU64_BtoL( arg: UInt64 ): UInt64; inline;

{
   These types are used for structures that contain data that is
   always in BigEndian format.  This extra typing prevents little
   endian code from directly changing the data, thus saving much
   time in the debugger.
}

{$ifc TARGET_RT_LITTLE_ENDIAN}

type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
	BigEndianUInt32 = record
		bigEndianValue: UInt32;
	end;
type
<<<<<<< HEAD
	BigEndianLong = record
		bigEndianValue: SIGNEDLONG;
	end;
type
	BigEndianUnsignedLong = record
		bigEndianValue: UNSIGNEDLONG;
	end;
type
	BigEndianShort = record
		bigEndianValue: SInt16;
	end;
type
	BigEndianUnsignedShort = record
		bigEndianValue: UInt16;
	end;
type
	BigEndianFixed = record
		bigEndianValue: Fixed;
	end;
type
	BigEndianUnsignedFixed = record
		bigEndianValue: UnsignedFixed;
	end;
type
=======
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	BigEndianLong = record
		bigEndianValue: SIGNEDLONG;
	end;
type
	BigEndianUnsignedLong = record
		bigEndianValue: UNSIGNEDLONG;
	end;
type
	BigEndianShort = record
		bigEndianValue: SInt16;
	end;
type
	BigEndianUnsignedShort = record
		bigEndianValue: UInt16;
	end;
type
	BigEndianFixed = record
		bigEndianValue: Fixed;
	end;
type
	BigEndianUnsignedFixed = record
		bigEndianValue: UnsignedFixed;
	end;
<<<<<<< HEAD

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
type
>>>>>>> origin/fixes_2.4
	BigEndianOSType = record
		bigEndianValue: OSType;
	end;

{$elsec}

type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
	BigEndianUInt32 = UInt32;
	BigEndianLong = SIGNEDLONG;
	BigEndianUnsignedLong = UNSIGNEDLONG;
	BigEndianShort = SInt16;
	BigEndianUnsignedShort = UInt16;
	BigEndianFixed = Fixed;
	BigEndianUnsignedFixed = UnsignedFixed;
	BigEndianOSType = OSType;
<<<<<<< HEAD
{$endc}  {TARGET_RT_LITTLE_ENDIAN}

type
	BigEndianUInt32Ptr = ^BigEndianUInt32;
=======
=======
>>>>>>> origin/fixes_2_2
	BigEndianLong						= SInt32;
	BigEndianUnsignedLong				= UInt32;
	BigEndianShort						= SInt16;
	BigEndianUnsignedShort				= UInt16;
	BigEndianFixed						= Fixed;
	BigEndianUnsignedFixed				= UnsignedFixed;
	BigEndianOSType						= OSType;
{$endc}  {TARGET_RT_LITTLE_ENDIAN}

type
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$endc}  {TARGET_RT_LITTLE_ENDIAN}

type
	BigEndianUInt32Ptr = ^BigEndianUInt32;
>>>>>>> origin/fixes_2.4
	BigEndianLongPtr = ^BigEndianLong;
	BigEndianUnsignedLongPtr = ^BigEndianUnsignedLong;
	BigEndianShortPtr = ^BigEndianShort;
	BigEndianUnsignedShortPtr = ^BigEndianUnsignedShort;
	BigEndianFixedPtr = ^BigEndianFixed;
	BigEndianUnsignedFixedPtr = ^BigEndianUnsignedFixed;
	BigEndianOSTypePtr = ^BigEndianOSType;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc TARGET_API_MAC_OSX}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$ifc TARGET_API_MAC_OSX}
>>>>>>> origin/fixes_2.4
{
        CoreEndian flipping API.

        This API is used to generically massage data buffers, in
        place, from one endian architecture to another.  In effect,
        the API supports registering a set of callbacks that can
        effect this translation.  

        The data types have specific meanings within their domain,
        although some data types can be registered with the same
        callback in several domains.  There is no wildcard domain.

        A set of pre-defined flippers are implemented by the Carbon
        frameworks for most common resource manager and AppleEvent data
        types.
  }
const
	kCoreEndianResourceManagerDomain = FourCharCode('rsrc');
	kCoreEndianAppleEventManagerDomain = FourCharCode('aevt');


{
 *  CoreEndianFlipProc
 *  
 *  Discussion:
 *    Callback use to flip endian-ness of typed data
 *  
 *  Parameters:
 *    
 *    dataDomain:
 *      Domain of the data type
 *    
 *    dataType:
 *      Type of data being flipped
 *    
 *    id:
 *      resource id (if being flipped on behalf of the resource
 *      manager, otherwise will be zero)
 *    
 *    dataPtr:
 *      Pointer to the data
 *    
 *    dataSize:
 *      Length of the data
 *    
 *    currentlyNative:
 *      Boolean indicating which direction to flip: false means flip
 *      from disk big endian to native (from disk), true means flip
 *      from native to disk big endian (to disk)
 *    
 *    refcon:
 *      An optional user reference supplied when the flipper is
 *      installed
 *  
 *  Result:
 *    Error code indicating whether the data was flipped.  noErr would
 *    indicate that the data was flipped as appropriate; any other
 *    error will be propagated back to the caller.
 }
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CoreEndianFlipProc = function( dataDomain: OSType; dataType: OSType; id: SInt16; dataPtr: UnivPtr; dataSize: ByteCount; currentlyNative: Boolean; refcon: UnivPtr ): OSStatus;
=======
	CoreEndianFlipProc = function( dataDomain: OSType; dataType: OSType; id: SInt16; dataPtr: UnivPtr; dataSize: UInt32; currentlyNative: Boolean; refcon: UnivPtr ): OSStatus;
>>>>>>> graemeg/fixes_2_2
=======
	CoreEndianFlipProc = function( dataDomain: OSType; dataType: OSType; id: SInt16; dataPtr: UnivPtr; dataSize: UInt32; currentlyNative: Boolean; refcon: UnivPtr ): OSStatus;
>>>>>>> origin/fixes_2_2
=======
	CoreEndianFlipProc = function( dataDomain: OSType; dataType: OSType; id: SInt16; dataPtr: UnivPtr; dataSize: ByteCount; currentlyNative: Boolean; refcon: UnivPtr ): OSStatus;
>>>>>>> origin/fixes_2.4
{
 * Install a flipper for this application
 }
{
 *  CoreEndianInstallFlipper()
 *  
 *  Summary:
 *    Installs a flipper proc for the given data type.  If the flipper
 *    is already registered, this flipper will take replace it.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    dataDomain:
 *      Domain of the data type
 *    
 *    dataType:
 *      Type of data for which this flipper should be installed
 *    
 *    proc:
 *      Flipper callback to be called for data of this type
 *    
 *    refcon:
 *      Optional user reference for the flipper
 *  
 *  Result:
 *    Error code indicating whether or not the flipper could be
 *    installed
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function CoreEndianInstallFlipper( dataDomain: OSType; dataType: OSType; proc: CoreEndianFlipProc; refcon: UnivPtr { can be NULL } ): OSStatus; external name '_CoreEndianInstallFlipper';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_3, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/fixes_2_2


{
 *  CoreEndianGetFlipper()
 *  
 *  Summary:
 *    Gets an existing data flipper proc for the given data type
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    dataDomain:
 *      Domain of the data type
 *    
 *    dataType:
 *      Type of the data for which this flipper should be installed
 *    
 *    proc:
 *      Pointer to a flipper callback
 *    
 *    refcon:
 *      Pointer to the callback refcon
 *  
 *  Result:
 *    noErr if the given flipper could be found; otherwise
 *    handlerNotFoundErr will be returned.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function CoreEndianGetFlipper( dataDomain: OSType; dataType: OSType; var proc: CoreEndianFlipProc; refcon: UnivPtrPtr ): OSStatus; external name '_CoreEndianGetFlipper';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_3, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/fixes_2_2


{
 *  CoreEndianFlipData()
 *  
 *  Summary:
 *    Calls the flipper for the given data type with the associated data
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    dataDomain:
 *      Domain of the data type
 *    
 *    dataType:
 *      type of the data
 *    
 *    id:
 *      resource id (if not a resource, pass zero)
 *    
 *    data:
 *      a pointer to the data to be flipped (in place)
 *    
 *    dataLen:
 *      length of the data to flip
 *    
 *    currentlyNative:
 *      a boolean indicating the direction to flip (whether the data is
 *      currently native endian or big-endian)
 *  
 *  Result:
 *    Error code indicating whether the data was flipped.  If
 *    handlerNotFound is returned, then no flipping took place (which
 *    is not necessarily an error condtion)
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function CoreEndianFlipData( dataDomain: OSType; dataType: OSType; id: SInt16; data: UnivPtr; dataLen: ByteCount; currentlyNative: Boolean ): OSStatus; external name '_CoreEndianFlipData';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_3, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)

{$endc} {TARGET_API_MAC_OSX}

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{$endc} {TARGET_API_MAC_OSX}

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{$endc} {TARGET_API_MAC_OSX}

>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
function CoreEndianFlipData( dataDomain: OSType; dataType: OSType; id: SInt16; data: UnivPtr; dataLen: UInt32; currentlyNative: Boolean ): OSStatus; external name '_CoreEndianFlipData';
=======
function CoreEndianFlipData( dataDomain: OSType; dataType: OSType; id: SInt16; data: UnivPtr; dataLen: ByteCount; currentlyNative: Boolean ): OSStatus; external name '_CoreEndianFlipData';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{$endc} {TARGET_API_MAC_OSX}

<<<<<<< HEAD

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4
implementation

{$R-}

function Endian16_Swap( arg: UInt16 ): UInt16; inline;
begin
	Endian16_Swap := (( arg shl 8) and $0FF00) or (( arg shr 8) and $00FF);
end;

function Endian32_Swap( arg: UInt32 ): UInt32; inline;
begin
    Endian32_Swap := ((arg and $FF) shl 24) or ((arg and $0FF00) shl 8) or ((arg shr 8) and $0FF00) or ((arg shr 24) and $FF);
end;

function Endian64_Swap_Pascal( arg: UInt64 ): UInt64; inline;
begin
	Endian64_Swap_Pascal := (Endian32_Swap( arg and $FFFFFFFF ) shl 32) or Endian32_Swap( (arg shr 32) and $FFFFFFFF );
end;

function Endian64_Swap( arg: UInt64 ): UInt64; inline;
begin
	Endian64_Swap := Endian64_Swap_Pascal(arg);
end;

function EndianS16_Swap( arg: SInt16 ): SInt16; inline;
begin
	EndianS16_Swap := (( arg shl 8) and $0FF00) or (( arg shr 8) and $00FF);
end;

function EndianS32_Swap( arg: SInt32 ): SInt32; inline;
begin
    EndianS32_Swap := ((arg and $FF) shl 24) or ((arg and $0FF00) shl 8) or ((arg shr 8) and $0FF00) or ((arg shr 24) and $FF);
end;

function EndianS64_Swap( arg: SInt64 ): SInt64; inline;
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	EndianS64_Swap := (SInt64( Endian32_Swap( arg and $FFFFFFFF ) ) shl 32) or Endian32_Swap( (arg shr 32) and $FFFFFFFF );
=======
	EndianS64_Swap := (Endian32_Swap( arg and $FFFFFFFF ) shl 32) or Endian32_Swap( (arg shr 32) and $FFFFFFFF );
>>>>>>> graemeg/fixes_2_2
=======
	EndianS64_Swap := (Endian32_Swap( arg and $FFFFFFFF ) shl 32) or Endian32_Swap( (arg shr 32) and $FFFFFFFF );
>>>>>>> origin/fixes_2_2
=======
	EndianS64_Swap := (SInt64( Endian32_Swap( arg and $FFFFFFFF ) ) shl 32) or Endian32_Swap( (arg shr 32) and $FFFFFFFF );
>>>>>>> origin/fixes_2.4
end;

{$ifc TARGET_RT_BIG_ENDIAN}
function EndianS16_BtoN( arg: SInt16 ): SInt16; inline;
begin
  EndianS16_BtoN := arg;
end;

function EndianS16_NtoB( arg: SInt16 ): SInt16; inline;
begin
  EndianS16_NtoB := arg;
end;

function EndianU16_BtoN( arg: UInt16 ): UInt16; inline;
begin
  EndianU16_BtoN := arg;
end;

function EndianU16_NtoB( arg: UInt16 ): UInt16; inline;
begin
  EndianU16_NtoB := arg;
end;

function EndianS32_BtoN( arg: SInt32 ): SInt32; inline;
begin
  EndianS32_BtoN := arg;
end;

function EndianS32_NtoB( arg: SInt32 ): SInt32; inline;
begin
  EndianS32_NtoB := arg;
end;

function EndianU32_BtoN( arg: UInt32 ): UInt32; inline;
begin
  EndianU32_BtoN := arg;
end;

function EndianU32_NtoB( arg: UInt32 ): UInt32; inline;
begin
  EndianU32_NtoB := arg;
end;

function EndianS64_BtoN( arg: SInt64 ): SInt64; inline;
begin
  EndianS64_BtoN := arg;
end;

function EndianS64_NtoB( arg: SInt64 ): SInt64; inline;
begin
  EndianS64_NtoB := arg;
end;

function EndianU64_BtoN( arg: UInt64 ): UInt64; inline;
begin
  EndianU64_BtoN := arg;
end;

function EndianU64_NtoB( arg: UInt64 ): UInt64; inline;
begin
  EndianU64_NtoB := arg;
end;

function EndianS16_LtoN( arg: SInt16 ): SInt16; inline;
begin
  EndianS16_LtoN := EndianS16_Swap(arg);
end;

function EndianS16_NtoL( arg: SInt16 ): SInt16; inline;
begin
  EndianS16_NtoL := EndianS16_Swap(arg);
end;

function EndianU16_LtoN( arg: UInt16 ): UInt16; inline;
begin
  EndianU16_LtoN := Endian16_Swap(arg);
end;

function EndianU16_NtoL( arg: UInt16 ): UInt16; inline;
begin
  EndianU16_NtoL := Endian16_Swap(arg);
end;

function EndianS32_LtoN( arg: SInt32 ): SInt32; inline;
begin
  EndianS32_LtoN := EndianS32_Swap(arg);
end;

function EndianS32_NtoL( arg: SInt32 ): SInt32; inline;
begin
  EndianS32_NtoL := EndianS32_Swap(arg);
end;

function EndianU32_LtoN( arg: UInt32 ): UInt32; inline;
begin
  EndianU32_LtoN := Endian32_Swap(arg);
end;

function EndianU32_NtoL( arg: UInt32 ): UInt32; inline;
begin
  EndianU32_NtoL := Endian32_Swap(arg);
end;


function EndianS64_LtoN( arg: SInt64 ): SInt64; inline;
begin
  EndianS64_LtoN := EndianS64_Swap(arg);
end;

function EndianS64_NtoL( arg: SInt64 ): SInt64; inline;
begin
  EndianS64_NtoL := EndianS64_Swap(arg);
end;

function EndianU64_LtoN( arg: UInt64 ): UInt64; inline;
begin
  EndianU64_LtoN := Endian64_Swap(arg);
end;

function EndianU64_NtoL( arg: UInt64 ): UInt64; inline;
begin
  EndianU64_NtoL := Endian64_Swap(arg);
end;

{$elsec}
function EndianS16_BtoN( arg: SInt16 ): SInt16; inline;
begin
  EndianS16_BtoN := EndianS16_Swap(arg);
end;

function EndianS16_NtoB( arg: SInt16 ): SInt16; inline;
begin
  EndianS16_NtoB := EndianS16_Swap(arg);
end;

function EndianU16_BtoN( arg: UInt16 ): UInt16; inline;
begin
  EndianU16_BtoN := Endian16_Swap(arg);
end;

function EndianU16_NtoB( arg: UInt16 ): UInt16; inline;
begin
  EndianU16_NtoB := Endian16_Swap(arg);
end;

function EndianS32_BtoN( arg: SInt32 ): SInt32; inline;
begin
  EndianS32_BtoN := EndianS32_Swap(arg);
end;

function EndianS32_NtoB( arg: SInt32 ): SInt32; inline;
begin
  EndianS32_NtoB := EndianS32_Swap(arg);
end;

function EndianU32_BtoN( arg: UInt32 ): UInt32; inline;
begin
  EndianU32_BtoN := Endian32_Swap(arg);
end;

function EndianU32_NtoB( arg: UInt32 ): UInt32; inline;
begin
  EndianU32_NtoB := Endian32_Swap(arg);
end;


function EndianS64_BtoN( arg: SInt64 ): SInt64; inline;
begin
  EndianS64_BtoN := EndianS64_Swap(arg);
end;

function EndianS64_NtoB( arg: SInt64 ): SInt64; inline;
begin
  EndianS64_NtoB := EndianS64_Swap(arg);
end;

function EndianU64_BtoN( arg: UInt64 ): UInt64; inline;
begin
  EndianU64_BtoN := Endian64_Swap(arg);
end;

function EndianU64_NtoB( arg: UInt64 ): UInt64; inline;
begin
  EndianU64_NtoB := Endian64_Swap(arg);
end;

function EndianS16_LtoN( arg: SInt16 ): SInt16; inline;
begin
  EndianS16_LtoN := arg;
end;

function EndianS16_NtoL( arg: SInt16 ): SInt16; inline;
begin
  EndianS16_NtoL := arg;
end;

function EndianU16_LtoN( arg: UInt16 ): UInt16; inline;
begin
  EndianU16_LtoN := arg;
end;

function EndianU16_NtoL( arg: UInt16 ): UInt16; inline;
begin
  EndianU16_NtoL := arg;
end;

function EndianS32_LtoN( arg: SInt32 ): SInt32; inline;
begin
  EndianS32_LtoN := arg;
end;

function EndianS32_NtoL( arg: SInt32 ): SInt32; inline;
begin
  EndianS32_NtoL := arg;
end;

function EndianU32_LtoN( arg: UInt32 ): UInt32; inline;
begin
  EndianU32_LtoN := arg;
end;

function EndianU32_NtoL( arg: UInt32 ): UInt32; inline;
begin
  EndianU32_NtoL := arg;
end;

function EndianS64_LtoN( arg: SInt64 ): SInt64; inline;
begin
  EndianS64_LtoN := arg;
end;

function EndianS64_NtoL( arg: SInt64 ): SInt64; inline;
begin
  EndianS64_NtoL := arg;
end;

function EndianU64_LtoN( arg: UInt64 ): UInt64; inline;
begin
  EndianU64_LtoN := arg;
end;

function EndianU64_NtoL( arg: UInt64 ): UInt64; inline;
begin
  EndianU64_NtoL := arg;
end;

{$endc}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
function EndianS16_LtoB( arg: SInt16 ): SInt16; inline;
begin
  EndianS16_LtoB:=EndianS16_Swap(arg);
end;

function EndianS16_BtoL( arg: SInt16 ): SInt16; inline;
begin
  EndianS16_BtoL:=EndianS16_Swap(arg);
end;

function EndianU16_LtoB( arg: UInt16 ): UInt16; inline;
begin
  EndianU16_LtoB:=Endian16_Swap(arg);
end;

function EndianU16_BtoL( arg: UInt16 ): UInt16; inline;
begin
  EndianU16_BtoL:=Endian16_Swap(arg);
end;

function EndianS32_LtoB( arg: SInt32 ): SInt32; inline;
begin
  EndianS32_LtoB:=EndianS32_Swap(arg);
end;

function EndianS32_BtoL( arg: SInt32 ): SInt32; inline;
begin
  EndianS32_BtoL:=EndianS32_Swap(arg);
end;

function EndianU32_LtoB( arg: UInt32 ): UInt32; inline;
begin
  EndianU32_LtoB:=Endian32_Swap(arg);
end;

function EndianU32_BtoL( arg: UInt32 ): UInt32; inline;
begin
  EndianU32_BtoL:=Endian32_Swap(arg);
end;

function EndianS64_LtoB( arg: SInt64 ): SInt64; inline;
begin
  EndianS64_LtoB:=EndianS64_Swap(arg);
end;

function EndianS64_BtoL( arg: SInt64 ): SInt64; inline;
begin
  EndianS64_BtoL:=EndianS64_Swap(arg);
end;

function EndianU64_LtoB( arg: UInt64 ): UInt64; inline;
begin
  EndianU64_LtoB:=Endian64_Swap_Pascal(arg);
end;

function EndianU64_BtoL( arg: UInt64 ): UInt64; inline;
begin
  EndianU64_BtoL:=Endian64_Swap_Pascal(arg);
end;


<<<<<<< HEAD

end.

{$endc} {not MACOSALLINCLUDE}
=======

end.
>>>>>>> graemeg/fixes_2_2
=======

end.
>>>>>>> origin/fixes_2_2
=======

end.

{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4
