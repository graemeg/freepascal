{
<<<<<<< HEAD
<<<<<<< HEAD
     File:       CarbonCore/fp.h
 
     Contains:   FPCE Floating-Point Definitions and Declarations.
                 The contents of this header file are deprecated.
                 Use math.h instead.
 
     Copyright:  © 1987-2011 by Apple Inc. All rights reserved.
}
{
    Modified for use with Free Pascal
    Version 308
    Please report any bugs to <gpc@microbizz.nl>
}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
     File:       fp.p
 
     Contains:   FPCE Floating-Point Definitions and Declarations.
 
     Version:    Technology: MathLib v2
                 Release:    Universal Interfaces 3.4.2
 
     Copyright:  © 1987-2002 by Apple Computer, Inc., all rights reserved.
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
                     http://www.freepascal.org/bugs.html
 
}

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

unit fp;
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
>>>>>>> origin/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
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
uses MacTypes,ConditionalMacros;
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

{*******************************************************************************
*                                                                               *
*    A collection of numerical functions designed to facilitate a wide          *
*    range of numerical programming as required by C9X.                         *
*                                                                               *
*    The <fp.h> declares many functions in support of numerical programming.    *
*    It provides a superset of <math.h> and <SANE.h> functions.  Some           *
*    functionality previously found in <SANE.h> and not in the FPCE <fp.h>      *
*    can be found in this <fp.h> under the heading "__NOEXTENSIONS__".          *
*                                                                               *
*    All of these functions are IEEE 754 aware and treat exceptions, NaNs,      *
*    positive and negative zero and infinity consistent with the floating-      *
*    point standard.                                                            *
*                                                                               *
*******************************************************************************}


{$ALIGN MAC68K}

{*******************************************************************************
*                                                                               *
*                            Efficient types                                    *
*                                                                               *
*    float_t         Most efficient type at least as wide as float              *
*    double_t        Most efficient type at least as wide as double             *
*                                                                               *
*      CPU            float_t(bits)                double_t(bits)               *
*    --------        -----------------            -----------------             *
*    PowerPC          float(32)                    double(64)                   *
*    68K              long double(80/96)           long double(80/96)           *
*    x86              double(64)                   double(64)                   *
*                                                                               *
*******************************************************************************}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc TARGET_CPU_PPC or TARGET_CPU_PPC64 or TARGET_CPU_ARM or TARGET_CPU_ARM64}
=======
{$ifc TARGET_CPU_PPC or TARGET_CPU_PPC64 or TARGET_CPU_ARM}
>>>>>>> graemeg/cpstrnew
=======
{$ifc TARGET_CPU_PPC or TARGET_CPU_PPC64 or TARGET_CPU_ARM}
>>>>>>> graemeg/cpstrnew
=======
{$ifc TARGET_CPU_PPC or TARGET_CPU_PPC64 or TARGET_CPU_ARM}
>>>>>>> graemeg/cpstrnew
=======
{$ifc TARGET_CPU_PPC or TARGET_CPU_PPC64 or TARGET_CPU_ARM}
>>>>>>> origin/cpstrnew
=======
{$ifc TARGET_CPU_PPC}
>>>>>>> graemeg/fixes_2_2
=======
{$ifc TARGET_CPU_PPC}
>>>>>>> origin/fixes_2_2
=======
{$ifc TARGET_CPU_PPC or TARGET_CPU_PPC64 or TARGET_CPU_ARM}
>>>>>>> origin/fixes_2.4

type
	float_t								= Float32;
	double_t							= Float64;
{$elsec}
  {$ifc TARGET_CPU_68K}
type
    float_t                             = extended;
    double_t                            = extended;
  {$elsec}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    {$ifc TARGET_CPU_X86 or TARGET_CPU_X86_64}
=======
    {$ifc TARGET_CPU_X86}
>>>>>>> graemeg/fixes_2_2
=======
    {$ifc TARGET_CPU_X86}
>>>>>>> origin/fixes_2_2
=======
    {$ifc TARGET_CPU_X86 or TARGET_CPU_X86_64}
>>>>>>> origin/fixes_2.4

type
	float_t								= Float64;
	double_t							= Float64;
    {$elsec}
      {$ifc TARGET_CPU_MIPS}

type
	float_t								= Double;
	double_t							= Double;
      {$elsec}
        {$ifc TARGET_CPU_ALPHA}

type
	float_t								= Double;
	double_t							= Double;
        {$elsec}
          {$ifc TARGET_CPU_SPARC}

type
	float_t								= Double;
	double_t							= Double;
          {$elsec}
{ Unsupported CPU }
          {$endc}
        {$endc}
      {$endc}
    {$endc}
  {$endc}
{$endc}

{*******************************************************************************
*                                                                               *
*                              Define some constants.                           *
*                                                                               *
*    HUGE_VAL            IEEE 754 value of infinity.                            *
*    INFINITY            IEEE 754 value of infinity.                            *
*    NAN                 A generic NaN (Not A Number).                          *
*    DECIMAL_DIG         Satisfies the constraint that the conversion from      *
*                        double to decimal and back is the identity function.   *
*                                                                               *
*******************************************************************************}
const
{$ifc TARGET_CPU_PPC}
    DECIMAL_DIG                         = 17; 
{$elsec}
    DECIMAL_DIG                         = 21;
{$endc}
{$ifc TARGET_OS_MAC}
{*******************************************************************************
*                                                                               *
*                            Trigonometric functions                            *
*                                                                               *
*   acos        result is in [0,pi].                                            *
*   asin        result is in [-pi/2,pi/2].                                      *
*   atan        result is in [-pi/2,pi/2].                                      *
*   atan2       Computes the arc tangent of y/x in [-pi,pi] using the sign of   *
*               both arguments to determine the quadrant of the computed value. *
*                                                                               *
*******************************************************************************}
{
 *  cos()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function cos( x: double_t ): double_t; external name '_cos';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function cos(x: double_t): double_t; external name '_cos';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  sin()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function sin( x: double_t ): double_t; external name '_sin';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function sin(x: double_t): double_t; external name '_sin';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  tan()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function tan( x: double_t ): double_t; external name '_tan';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function tan(x: double_t): double_t; external name '_tan';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  acos()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function acos( x: double_t ): double_t; external name '_acos';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function acos(x: double_t): double_t; external name '_acos';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  asin()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function asin( x: double_t ): double_t; external name '_asin';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function asin(x: double_t): double_t; external name '_asin';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  atan()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function atan( x: double_t ): double_t; external name '_atan';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function atan(x: double_t): double_t; external name '_atan';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  atan2()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function atan2( y: double_t; x: double_t ): double_t; external name '_atan2';
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function atan2(y: double_t; x: double_t): double_t; external name '_atan2';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2


{*******************************************************************************
*                                                                               *
*                              Hyperbolic functions                             *
*                                                                               *
*******************************************************************************}
{
 *  cosh()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function cosh( x: double_t ): double_t; external name '_cosh';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function cosh(x: double_t): double_t; external name '_cosh';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  sinh()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function sinh( x: double_t ): double_t; external name '_sinh';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function sinh(x: double_t): double_t; external name '_sinh';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  tanh()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function tanh( x: double_t ): double_t; external name '_tanh';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function tanh(x: double_t): double_t; external name '_tanh';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  acosh()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function acosh( x: double_t ): double_t; external name '_acosh';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function acosh(x: double_t): double_t; external name '_acosh';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  asinh()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function asinh( x: double_t ): double_t; external name '_asinh';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function asinh(x: double_t): double_t; external name '_asinh';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  atanh()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function atanh( x: double_t ): double_t; external name '_atanh';
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function atanh(x: double_t): double_t; external name '_atanh';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2


{*******************************************************************************
*                                                                               *
*                              Exponential functions                            *
*                                                                               *
*   expm1       expm1(x) = exp(x) - 1.  But, for small enough arguments,        *
*               expm1(x) is expected to be more accurate than exp(x) - 1.       *
*   frexp       Breaks a floating-point number into a normalized fraction       *
*               and an integral power of 2.  It stores the SInt16 in the       *
*               object pointed by *exponent.                                    *
*   ldexp       Multiplies a floating-point number by an SInt16 power of 2.    *
*   log1p       log1p = log(1 + x). But, for small enough arguments,            *
*               log1p is expected to be more accurate than log(1 + x).          *
*   logb        Extracts the exponent of its argument, as a signed integral     *
*               value. A subnormal argument is treated as though it were first  *
*               normalized. Thus:                                               *
*                                  1   <=   x * 2^(-logb(x))   <   2            *
*   modf        Returns fractional part of x as function result and returns     *
*               integral part of x via iptr. Note C9X uses double not double_t. *
*   scalb       Computes x * 2^n efficently.  This is not normally done by      *
*               computing 2^n explicitly.                                       *
*                                                                               *
*******************************************************************************}
{
 *  exp()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function exp( x: double_t ): double_t; external name '_exp';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function exp(x: double_t): double_t; external name '_exp';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  expm1()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function expm1( x: double_t ): double_t; external name '_expm1';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function expm1(x: double_t): double_t; external name '_expm1';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  exp2()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function exp2( x: double_t ): double_t; external name '_exp2';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function exp2(x: double_t): double_t; external name '_exp2';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  frexp()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function frexp( x: double_t; var exponent: SInt32 ): double_t; external name '_frexp';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function frexp(x: double_t; var exponent: SInt32): double_t; external name '_frexp';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  ldexp()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function ldexp( x: double_t; n: SInt32 ): double_t; external name '_ldexp';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function ldexp(x: double_t; n: SInt32): double_t; external name '_ldexp';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  log()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function log( x: double_t ): double_t; external name '_log';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function log(x: double_t): double_t; external name '_log';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  log2()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function log2( x: double_t ): double_t; external name '_log2';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function log2(x: double_t): double_t; external name '_log2';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  log1p()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function log1p( x: double_t ): double_t; external name '_log1p';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function log1p(x: double_t): double_t; external name '_log1p';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  log10()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function log10( x: double_t ): double_t; external name '_log10';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function log10(x: double_t): double_t; external name '_log10';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  logb()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function logb( x: double_t ): double_t; external name '_logb';
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function logb(x: double_t): double_t; external name '_logb';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  modf()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function modf( x: double_t; var iptr: double_t ): double_t; external name '_modf';
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function modf(x: double_t; var iptr: double_t): double_t; external name '_modf';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  modff()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function modff( x: Float32; var iptrf: Float32 ): Float32; external name '_modff';
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function modff(x: Single; var iptrf: Single): Single; external name '_modff';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2


{
    Note: For compatiblity scalb(x,n) has n of type
            int  on Mac OS X 
            long on Mac OS
}
<<<<<<< HEAD
<<<<<<< HEAD
type
	_scalb_n_type = SInt32;
{
 *  scalb()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function scalb( x: double_t; n: _scalb_n_type ): double_t; external name '_scalb';
=======
=======
>>>>>>> origin/fixes_2_2

type
	_scalb_n_type						= SInt32;
	{
	 *  scalb()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   in MathLib 1.0 and later
	 *    CarbonLib:        in CarbonLib 1.0 and later
	 *    Mac OS X:         in version 10.0 and later
	 	}
function scalb(x: double_t; n: _scalb_n_type): double_t; external name '_scalb';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2


{*******************************************************************************
*                                                                               *
*                     Power and absolute value functions                        *
*                                                                               *
*   hypot       Computes the square root of the sum of the squares of its       *
*               arguments, without undue overflow or underflow.                 *
*   pow         Returns x raised to the power of y.  Result is more accurate    *
*               than using exp(log(x)*y).                                       *
*                                                                               *
*******************************************************************************}
{
 *  fabs()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function fabs( x: double_t ): double_t; external name '_fabs';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function fabs(x: double_t): double_t; external name '_fabs';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  hypot()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function hypot( x: double_t; y: double_t ): double_t; external name '_hypot';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function hypot(x: double_t; y: double_t): double_t; external name '_hypot';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  pow()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 2.0 and later
 }
function pow( x: double_t; y: double_t ): double_t; external name '_pow';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 2.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function pow(x: double_t; y: double_t): double_t; external name '_pow';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  sqrt()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function sqrt( x: double_t ): double_t; external name '_sqrt';
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function sqrt(x: double_t): double_t; external name '_sqrt';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2


{*******************************************************************************
*                                                                               *
*                        Gamma and Error functions                              *
*                                                                               *
*   erf         The error function.                                             *
*   erfc        Complementary error function.                                   *
*   gamma       The gamma function.                                             *
*   lgamma      Computes the base-e logarithm of the absolute value of          *
*               gamma of its argument x, for x > 0.                             *
*                                                                               *
*******************************************************************************}
{
 *  erf()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function erf( x: double_t ): double_t; external name '_erf';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function erf(x: double_t): double_t; external name '_erf';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  erfc()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function erfc( x: double_t ): double_t; external name '_erfc';
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function erfc(x: double_t): double_t; external name '_erfc';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  gamma()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function gamma( x: double_t ): double_t; external name '_gamma';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function gamma(x: double_t): double_t; external name '_gamma';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  lgamma()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function lgamma( x: double_t ): double_t; external name '_lgamma';
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function lgamma(x: double_t): double_t; external name '_lgamma';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2


{*******************************************************************************
*                                                                               *
*                        Nearest SInt16 functions                              *
*                                                                               *
*   rint        Rounds its argument to an integral value in floating point      *
*               format, honoring the current rounding direction.                *
*                                                                               *
*   nearbyint   Differs from rint only in that it does not raise the inexact    *
*               exception. It is the nearbyint function recommended by the      *
*               IEEE floating-point standard 854.                               *
*                                                                               *
*   rinttol     Rounds its argument to the nearest long int using the current   *
*               rounding direction.  NOTE: if the rounded value is outside      *
*               the range of long int, then the result is undefined.            *
*                                                                               *
*   round       Rounds the argument to the nearest integral value in floating   *
*               point format similar to the Fortran "anint" function. That is:  *
*               add half to the magnitude and chop.                             *
*                                                                               *
*   roundtol    Similar to the Fortran function nint or to the Pascal round.    *
*               NOTE: if the rounded value is outside the range of long int,    *
*               then the result is undefined.                                   *
*                                                                               *
*   trunc       Computes the integral value, in floating format, nearest to     *
*               but no larger in magnitude than its argument.   NOTE: on 68K    *
*               compilers when using -elems881, trunc must return an int        *
*                                                                               *
*******************************************************************************}
{
 *  ceil()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function ceil( x: double_t ): double_t; external name '_ceil';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function ceil(x: double_t): double_t; external name '_ceil';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  floor()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function floor( x: double_t ): double_t; external name '_floor';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function floor(x: double_t): double_t; external name '_floor';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  rint()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function rint( x: double_t ): double_t; external name '_rint';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function rint(x: double_t): double_t; external name '_rint';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  nearbyint()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function nearbyint( x: double_t ): double_t; external name '_nearbyint';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function nearbyint(x: double_t): double_t; external name '_nearbyint';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  rinttol()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function rinttol( x: double_t ): SIGNEDLONG; external name '_rinttol';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function rinttol(x: double_t): SInt32; external name '_rinttol';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  round()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function roundd( x: double_t ): double_t; external name '_round';
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function roundd(x: double_t): double_t; external name '_round';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  roundtol()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function roundtol( round: double_t ): SIGNEDLONG; external name '_roundtol';
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function roundtol(round: double_t): SInt32; external name '_roundtol';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
    Note: For compatiblity trunc(x) has a return type of
            int       for classic 68K with FPU enabled
            double_t  everywhere else
}
{$ifc TARGET_RT_MAC_68881}

type
	_trunc_return_type					= SInt32;
{$elsec}

type
	_trunc_return_type					= double_t;
{$endc}  {TARGET_RT_MAC_68881}
<<<<<<< HEAD
<<<<<<< HEAD
{
 *  trunc()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function truncd( x: double_t ): _trunc_return_type; external name '_trunc';
=======
=======
>>>>>>> origin/fixes_2_2
	{
	 *  trunc()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   in MathLib 1.0 and later
	 *    CarbonLib:        in CarbonLib 1.0 and later
	 *    Mac OS X:         in version 10.0 and later
	 	}
function truncd(x: double_t): _trunc_return_type; external name '_trunc';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2


{*******************************************************************************
*                                                                               *
*                            Remainder functions                                *
*                                                                               *
*   remainder       IEEE 754 floating point standard for remainder.             *
*   remquo          SANE remainder.  It stores into 'quotient' the 7 low-order  *
*                   bits of the SInt16 quotient x/y, such that:                *
*                       -127 <= quotient <= 127.                                *
*                                                                               *
*******************************************************************************}
{
 *  fmod()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function fmod( x: double_t; y: double_t ): double_t; external name '_fmod';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function fmod(x: double_t; y: double_t): double_t; external name '_fmod';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  remainder()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function remainder( x: double_t; y: double_t ): double_t; external name '_remainder';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function remainder(x: double_t; y: double_t): double_t; external name '_remainder';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  remquo()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function remquo( x: double_t; y: double_t; var quo: SInt32 ): double_t; external name '_remquo';
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function remquo(x: double_t; y: double_t; var quo: SInt32): double_t; external name '_remquo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2


{*******************************************************************************
*                                                                               *
*                             Auxiliary functions                               *
*                                                                               *
*   copysign        Produces a value with the magnitude of its first argument   *
*                   and sign of its second argument.  NOTE: the order of the    *
*                   arguments matches the recommendation of the IEEE 754        *
*                   floating point standard,  which is opposite from the SANE   *
*                   copysign function.                                          *
*                                                                               *
*   nan             The call 'nan("n-char-sequence")' returns a quiet NaN       *
*                   with content indicated through tagp in the selected         *
*                   data type format.                                           *
*                                                                               *
*   nextafter       Computes the next representable value after 'x' in the      *
*                   direction of 'y'.  if x == y, then y is returned.           *
*                                                                               *
*******************************************************************************}
{
 *  copysign()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function copysign( x: double_t; y: double_t ): double_t; external name '_copysign';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function copysign(x: double_t; y: double_t): double_t; external name '_copysign';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  nan()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function nan( tagp: ConstCStringPtr ): Float64; external name '_nan';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function nan(tagp: ConstCStringPtr): Double; external name '_nan';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  nanf()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function nanf( tagp: ConstCStringPtr ): Float32; external name '_nanf';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function nanf(tagp: ConstCStringPtr): Single; external name '_nanf';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  nextafterd()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function nextafterd( x: Float64; y: Float64 ): Float64; external name '_nextafterd';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function nextafterd(x: Double; y: Double): Double; external name '_nextafterd';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  nextafterf()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function nextafterf( x: Float32; y: Float32 ): Float32; external name '_nextafterf';
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function nextafterf(x: Single; y: Single): Single; external name '_nextafterf';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2


{
 *  __fpclassifyd()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function __fpclassifyd( x: Float64 ): SIGNEDLONG; external name '___fpclassifyd';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function __fpclassifyd(x: Double): SInt32; external name '___fpclassifyd';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  __fpclassifyf()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function __fpclassifyf( x: Float32 ): SIGNEDLONG; external name '___fpclassifyf';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function __fpclassifyf(x: Single): SInt32; external name '___fpclassifyf';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  __isnormald()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function __isnormald( x: Float64 ): SIGNEDLONG; external name '___isnormald';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function __isnormald(x: Double): SInt32; external name '___isnormald';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  __isnormalf()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function __isnormalf( x: Float32 ): SIGNEDLONG; external name '___isnormalf';
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function __isnormalf(x: Single): SInt32; external name '___isnormalf';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  __isfinited()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function __isfinited( x: Float64 ): SIGNEDLONG; external name '___isfinited';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function __isfinited(x: Double): SInt32; external name '___isfinited';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  __isfinitef()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function __isfinitef( x: Float32 ): SIGNEDLONG; external name '___isfinitef';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function __isfinitef(x: Single): SInt32; external name '___isfinitef';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  __isnand()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function __isnand( x: Float64 ): SIGNEDLONG; external name '___isnand';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function __isnand(x: Double): SInt32; external name '___isnand';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  __isnanf()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function __isnanf( x: Float32 ): SIGNEDLONG; external name '___isnanf';
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function __isnanf(x: Single): SInt32; external name '___isnanf';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  __signbitd()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function __signbitd( x: Float64 ): SIGNEDLONG; external name '___signbitd';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function __signbitd(x: Double): SInt32; external name '___signbitd';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  __signbitf()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function __signbitf( x: Float32 ): SIGNEDLONG; external name '___signbitf';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function __signbitf(x: Single): SInt32; external name '___signbitf';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  __inf()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
>>>>>>> origin/fixes_2_2
 }
function __inf: double_t; external name '___inf';


{*******************************************************************************
*                                                                               *
*                              Inquiry macros                                   *
*                                                                               *
<<<<<<< HEAD
<<<<<<< HEAD
*   fpclassify      Returns one of the FP_Å values.                             *
=======
*   fpclassify      Returns one of the FP_≈ values.                             *
>>>>>>> graemeg/fixes_2_2
=======
*   fpclassify      Returns one of the FP_≈ values.                             *
>>>>>>> origin/fixes_2_2
*   isnormal        Non-zero if and only if the argument x is normalized.       *
*   isfinite        Non-zero if and only if the argument x is finite.           *
*   isnan           Non-zero if and only if the argument x is a NaN.            *
*   signbit         Non-zero if and only if the sign of the argument x is       *
*                   negative.  This includes, NaNs, infinities and zeros.       *
*                                                                               *
*******************************************************************************}

const
	FP_SNAN						= 0;							{       signaling NaN                          }
	FP_QNAN						= 1;							{       quiet NaN                              }
	FP_INFINITE					= 2;							{       + or - infinity                        }
	FP_ZERO						= 3;							{       + or - zero                            }
	FP_NORMAL					= 4;							{       all normal numbers                     }
	FP_SUBNORMAL				= 5;							{       denormal numbers                       }


<<<<<<< HEAD
<<<<<<< HEAD
{*******************************************************************************
*                                                                               *
*                      Max, Min and Positive Difference                         *
*                                                                               *
*   fdim        Determines the 'positive difference' between its arguments:     *
*               ( x - y, if x > y ), ( +0, if x <= y ).  If one argument is     *
*               NaN, then fdim returns that NaN.  if both arguments are NaNs,   *
*               then fdim returns the first argument.                           *
*                                                                               *
*   fmax        Returns the maximum of the two arguments.  Corresponds to the   *
*               max function in FORTRAN.  NaN arguments are treated as missing  *
*               data.  If one argument is NaN and the other is a number, then   *
*               the number is returned.  If both are NaNs then the first        *
*               argument is returned.                                           *
*                                                                               *
*   fmin        Returns the minimum of the two arguments.  Corresponds to the   *
*               min function in FORTRAN.  NaN arguments are treated as missing  *
*               data.  If one argument is NaN and the other is a number, then   *
*               the number is returned.  If both are NaNs then the first        *
*               argument is returned.                                           *
*                                                                               *
*******************************************************************************}
{
 *  fdim()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function fdim( x: double_t; y: double_t ): double_t; external name '_fdim';

=======
=======
>>>>>>> origin/fixes_2_2
	{	*******************************************************************************
	*                                                                               *
	*                      Max, Min and Positive Difference                         *
	*                                                                               *
	*   fdim        Determines the 'positive difference' between its arguments:     *
	*               ( x - y, if x > y ), ( +0, if x <= y ).  If one argument is     *
	*               NaN, then fdim returns that NaN.  if both arguments are NaNs,   *
	*               then fdim returns the first argument.                           *
	*                                                                               *
	*   fmax        Returns the maximum of the two arguments.  Corresponds to the   *
	*               max function in FORTRAN.  NaN arguments are treated as missing  *
	*               data.  If one argument is NaN and the other is a number, then   *
	*               the number is returned.  If both are NaNs then the first        *
	*               argument is returned.                                           *
	*                                                                               *
	*   fmin        Returns the minimum of the two arguments.  Corresponds to the   *
	*               min function in FORTRAN.  NaN arguments are treated as missing  *
	*               data.  If one argument is NaN and the other is a number, then   *
	*               the number is returned.  If both are NaNs then the first        *
	*               argument is returned.                                           *
	*                                                                               *
	*******************************************************************************	}
	{
	 *  fdim()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   in MathLib 1.0 and later
	 *    CarbonLib:        in CarbonLib 1.0 and later
	 *    Mac OS X:         in version 10.0 and later
	 	}
function fdim(x: double_t; y: double_t): double_t; external name '_fdim';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  fmax()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function fmax( x: double_t; y: double_t ): double_t; external name '_fmax';

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function fmax(x: double_t; y: double_t): double_t; external name '_fmax';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  fmin()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function fmin( x: double_t; y: double_t ): double_t; external name '_fmin';
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function fmin(x: double_t; y: double_t): double_t; external name '_fmin';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2


{******************************************************************************
*                                Constants                                     *
******************************************************************************}

const
  pi = system.pi;

{*******************************************************************************
*                                                                               *
*                              Non NCEG extensions                              *
*                                                                               *
*******************************************************************************}
{$ifc undefined __NOEXTENSIONS__}
{*******************************************************************************
*                                                                               *
*                              Financial functions                              *
*                                                                               *
*   compound        Computes the compound interest factor "(1 + rate)^periods"  *
*                   more accurately than the straightforward computation with   *
*                   the Power function.  This is SANE's compound function.      *
*                                                                               *
*   annuity         Computes the present value factor for an annuity            *
*                   "(1 - (1 + rate)^(-periods)) /rate" more accurately than    *
*                   the straightforward computation with the Power function.    *
*                   This is SANE's annuity function.                            *
*                                                                               *
*******************************************************************************}
{
 *  compound()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function compound( rate: Float64; periods: Float64 ): Float64; external name '_compound';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function compound(rate: double_t; periods: double_t): double_t; external name '_compound';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  annuity()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function annuity( rate: Float64; periods: Float64 ): Float64; external name '_annuity';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function annuity(rate: double_t; periods: double_t): double_t; external name '_annuity';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2


{*******************************************************************************
*                                                                               *
*                              Random function                                  *
*                                                                               *
*   randomx         A pseudorandom number generator.  It uses the iteration:    *
*                               (7^5*x)mod(2^31-1)                              *
*                                                                               *
*******************************************************************************}
{
 *  randomx()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function randomx( var x: double_t ): double_t; external name '_randomx';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function randomx(var x: double_t): double_t; external name '_randomx';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2


{******************************************************************************
*                              Relational operator                             *
******************************************************************************}
{      relational operator      }
<<<<<<< HEAD
<<<<<<< HEAD
type
	relop = SInt16;
const
	GREATERTHAN = 0;
	LESSTHAN = 1;
	EQUALTO = 2;
	UNORDERED = 3;

{
 *  relation()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function relation( x: double_t; y: double_t ): relop; external name '_relation';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)


=======
=======
>>>>>>> origin/fixes_2_2

type
	relop								= SInt16;

const
	GREATERTHAN					= 0;
	LESSTHAN					= 1;
	EQUALTO						= 2;
	UNORDERED					= 3;

	{
	 *  relation()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   in MathLib 1.0 and later
	 *    CarbonLib:        in CarbonLib 1.0 and later
	 *    Mac OS X:         in version 10.0 and later
	 	}
function relation(x: double_t; y: double_t): relop; external name '_relation';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2


{*******************************************************************************
*                                                                               *
*                         Binary to decimal conversions                         *
*                                                                               *
*   SIGDIGLEN   Significant decimal digits.                                     *
*                                                                               *
*   decimal     A record which provides an intermediate unpacked form for       *
*               programmers who wish to do their own parsing of numeric input   *
*               or formatting of numeric output.                                *
*                                                                               *
*   decform     Controls each conversion to a decimal string.  The style field  *
*               is either FLOATDECIMAL or FIXEDDECIMAL. If FLOATDECIMAL, the    *
*               value of the field digits is the number of significant digits.  *
*               If FIXEDDECIMAL value of the field digits is the number of      *
*               digits to the right of the decimal point.                       *
*                                                                               *
*   num2dec     Converts a double_t to a decimal record using a decform.        *
*   dec2num     Converts a decimal record d to a double_t value.                *
*   dec2str     Converts a decform and decimal to a string using a decform.     *
*   str2dec     Converts a string to a decimal struct.                          *
*   dec2d       Similar to dec2num except a double is returned (68k only).      *
*   dec2f       Similar to dec2num except a float is returned.                  *
*   dec2s       Similar to dec2num except a short is returned.                  *
*   dec2l       Similar to dec2num except a long is returned.                   *
*                                                                               *
*******************************************************************************}
const
    SIGDIGLEN                   = 36;
    DECSTROUTLEN                = 80;
type
    DecimalKindItem = (FLOATDECIMAL, FIXEDDECIMAL);
    DecimalKind = DecimalKindItem;

    decimal = record
        sgn:    0..1;           { sign 0 for +, 1 for -  }
        exp:    SInt16;
        sig:    Str36;
    end;

    decform = record
        style:  DecimalKind;
        digits: SInt16;
    end;

{
 *  num2dec()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
procedure num2dec( const (*var*) f: decform; x: double_t; var d: decimal ); external name '_num2dec';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure num2dec(const (*var*) f: decform; x: double_t; var d: decimal); external name '_num2dec';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  dec2num()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function dec2num( const (*var*) d: decimal ): double_t; external name '_dec2num';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function dec2num(const (*var*) d: decimal): double_t; external name '_dec2num';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  dec2str()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
procedure dec2str( const (*var*) f: decform; const (*var*) d: decimal; var s: char ); external name '_dec2str';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure dec2str(const (*var*) f: decform; const (*var*) d: decimal; s: CStringPtr); external name '_dec2str';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  str2dec()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
procedure str2dec( s: ConstCStringPtr; var ix: SInt16; var d: decimal; var vp: SInt16 ); external name '_str2dec';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure str2dec(s: ConstCStringPtr; var ix: SInt16; var d: decimal; var vp: SInt16); external name '_str2dec';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{$ifc TARGET_CPU_68K}
{$ifc CALL_NOT_IN_CARBON}
{
 *  dec2d()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function dec2d(const (*var*) d: decimal): Double; external name '_dec2d';

{$endc}  {CALL_NOT_IN_CARBON}
{$endc}  {TARGET_CPU_68K}
{
 *  dec2f()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function dec2f( const (*var*) d: decimal ): Float32; external name '_dec2f';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function dec2f(const (*var*) d: decimal): Single; external name '_dec2f';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  dec2s()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function dec2s( const (*var*) d: decimal ): SInt16; external name '_dec2s';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function dec2s(const (*var*) d: decimal): SInt16; external name '_dec2s';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  dec2l()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 }
function dec2l( const (*var*) d: decimal ): SIGNEDLONG; external name '_dec2l';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function dec2l(const (*var*) d: decimal): SInt32; external name '_dec2l';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2


{*******************************************************************************
*                                                                               *
*                         68k-only Transfer Function Prototypes                 *
*                                                                               *
*******************************************************************************}
{$ifc TARGET_CPU_68K}
{$ifc CALL_NOT_IN_CARBON}
{
 *  x96tox80()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure x96tox80(const (*var*) x: extended96; var x80: extended80); external name '_x96tox80';

{
 *  x80tox96()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
procedure x80tox96(const (*var*) x80: extended80; var x: extended96); external name '_x80tox96';

{$endc}  {CALL_NOT_IN_CARBON}
{$endc}  {TARGET_CPU_68K}
{$endc}
{*******************************************************************************
*                                                                               *
*                         PowerPC-only Function Prototypes                      *
*                                                                               *
*******************************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
{$ifc TARGET_CPU_PPC or TARGET_CPU_PPC64}
=======
{$ifc TARGET_CPU_PPC}
>>>>>>> graemeg/fixes_2_2
=======
{$ifc TARGET_CPU_PPC}
>>>>>>> origin/fixes_2_2

{
 *  cosl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function cosl(x: LongDouble): LongDouble; external name '_cosl';


{
 *  sinl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function sinl(x: LongDouble): LongDouble; external name '_sinl';


{
 *  tanl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function tanl(x: LongDouble): LongDouble; external name '_tanl';


{
 *  acosl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function acosl(x: LongDouble): LongDouble; external name '_acosl';


{
 *  asinl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function asinl(x: LongDouble): LongDouble; external name '_asinl';


{
 *  atanl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function atanl(x: LongDouble): LongDouble; external name '_atanl';


{
 *  atan2l()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function atan2l(y: LongDouble; x: LongDouble): LongDouble; external name '_atan2l';


{
 *  coshl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function coshl(x: LongDouble): LongDouble; external name '_coshl';


{
 *  sinhl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function sinhl(x: LongDouble): LongDouble; external name '_sinhl';


{
 *  tanhl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function tanhl(x: LongDouble): LongDouble; external name '_tanhl';


{
 *  acoshl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function acoshl(x: LongDouble): LongDouble; external name '_acoshl';


{
 *  asinhl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function asinhl(x: LongDouble): LongDouble; external name '_asinhl';


{
 *  atanhl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function atanhl(x: LongDouble): LongDouble; external name '_atanhl';


{
 *  expl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function expl(x: LongDouble): LongDouble; external name '_expl';


{
 *  expm1l()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function expm1l(x: LongDouble): LongDouble; external name '_expm1l';


{
 *  exp2l()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function exp2l(x: LongDouble): LongDouble; external name '_exp2l';


{
 *  frexpl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function frexpl(x: LongDouble; var exponent: SInt32): LongDouble; external name '_frexpl';


{
 *  ldexpl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function ldexpl(x: LongDouble; n: SInt32): LongDouble; external name '_ldexpl';


{
 *  logl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function logl(x: LongDouble): LongDouble; external name '_logl';


{
 *  log1pl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function log1pl(x: LongDouble): LongDouble; external name '_log1pl';


{
 *  log10l()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function log10l(x: LongDouble): LongDouble; external name '_log10l';


{
 *  log2l()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function log2l(x: LongDouble): LongDouble; external name '_log2l';


{
 *  logbl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function logbl(x: LongDouble): LongDouble; external name '_logbl';


{
 *  scalbl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function scalbl(x: LongDouble; n: SInt32): LongDouble; external name '_scalbl';


{
 *  fabsl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function fabsl(x: LongDouble): LongDouble; external name '_fabsl';


{
 *  hypotl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function hypotl(x: LongDouble; y: LongDouble): LongDouble; external name '_hypotl';


{
 *  powl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function powl(x: LongDouble; y: LongDouble): LongDouble; external name '_powl';


{
 *  sqrtl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function sqrtl(x: LongDouble): LongDouble; external name '_sqrtl';


{
 *  erfl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function erfl(x: LongDouble): LongDouble; external name '_erfl';


{
 *  erfcl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function erfcl(x: LongDouble): LongDouble; external name '_erfcl';


{
 *  gammal()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function gammal(x: LongDouble): LongDouble; external name '_gammal';


{
 *  lgammal()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function lgammal(x: LongDouble): LongDouble; external name '_lgammal';


{
 *  ceill()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 2.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 2.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 2.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function ceill(x: LongDouble): LongDouble; external name '_ceill';


{
 *  floorl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function floorl(x: LongDouble): LongDouble; external name '_floorl';


{
 *  rintl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function rintl(x: LongDouble): LongDouble; external name '_rintl';


{
 *  nearbyintl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function nearbyintl(x: LongDouble): LongDouble; external name '_nearbyintl';


{
 *  rinttoll()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function rinttoll(x: LongDouble): SInt32; external name '_rinttoll';


{
 *  roundl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function roundl(x: LongDouble): LongDouble; external name '_roundl';


{
 *  roundtoll()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function roundtoll(x: LongDouble): SInt32; external name '_roundtoll';


{
 *  truncl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function truncl(x: LongDouble): LongDouble; external name '_truncl';


{
 *  remainderl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function remainderl(x: LongDouble; y: LongDouble): LongDouble; external name '_remainderl';


{
 *  remquol()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function remquol(x: LongDouble; y: LongDouble; var quo: SInt32): LongDouble; external name '_remquol';


{
 *  copysignl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function copysignl(x: LongDouble; y: LongDouble): LongDouble; external name '_copysignl';


{
 *  fdiml()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function fdiml(x: LongDouble; y: LongDouble): LongDouble; external name '_fdiml';


{
 *  fmaxl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function fmaxl(x: LongDouble; y: LongDouble): LongDouble; external name '_fmaxl';


{
 *  fminl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function fminl(x: LongDouble; y: LongDouble): LongDouble; external name '_fminl';

{$ifc undefined __NOEXTENSIONS__}
{
 *  relationl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function relationl(x: LongDouble; y: LongDouble): relop; external name '_relationl';


{
 *  num2decl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
procedure num2decl(const (*var*) f: decform; x: LongDouble; var d: decimal); external name '_num2decl';


{
 *  dec2numl()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         not available
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
function dec2numl(const (*var*) d: decimal): LongDouble; external name '_dec2numl';

{$endc}
{$endc}  {TARGET_CPU_PPC}
{$endc}  {TARGET_OS_MAC}

{$ifc undefined __NOEXTENSIONS__}
{    
        MathLib v2 has two new transfer functions: x80tod and dtox80.  They can 
        be used to directly transform 68k 80-bit extended data types to double
        and back for PowerPC based machines without using the functions
        x80told or ldtox80.  Double rounding may occur. 
    }
{
 *  x80tod()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 2.0 and later
 }
function x80tod( const (*var*) x80: extended80 ): Float64; external name '_x80tod';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 2.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function x80tod(const (*var*) x80: extended80): Double; external name '_x80tod';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{
 *  dtox80()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 2.0 and later
 }
procedure dtox80( const (*var*) x: Float64; var x80: extended80 ); external name '_dtox80';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 2.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure dtox80((*const*) var x: Double; var x80: extended80); external name '_dtox80';

{$ifc TARGET_CPU_PPC}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{
 *  x80told()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
>>>>>>> origin/fixes_2_2
 }
procedure x80told(const (*var*) x80: extended80; var x: LongDouble); external name '_x80told';


{
 *  ldtox80()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 }
procedure ldtox80((*const*) var x: LongDouble; var x80: extended80); external name '_ldtox80';

{$endc}


{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in MathLib 1.0 and later or as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         not available
 }
procedure ldtox80((*const*) var x: LongDouble; var x80: extended80); external name '_ldtox80';

{$endc} {TARGET_CPU_PPC}
{$endc}

{$ALIGN MAC68K}

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
