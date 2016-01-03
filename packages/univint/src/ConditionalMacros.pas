{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 * Copyright (c) 1993-2011 by Apple Inc.. All rights reserved.
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
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
     File:       CarbonCore/ConditionalMacros.h
 
     Contains:   Set up for compiler independent conditionals
 
     Version:    CarbonCore-859.2~1
 
     Copyright:  © 1993-2008 by Apple Computer, Inc., all rights reserved
=======
=======
>>>>>>> origin/fixes_2_2
     File:       ConditionalMacros.p
=======
     File:       CarbonCore/ConditionalMacros.h
>>>>>>> origin/fixes_2.4
 
     Contains:   Set up for compiler independent conditionals
 
     Version:    CarbonCore-859.2~1
 
<<<<<<< HEAD
     Copyright:  © 1993-2002 by Apple Computer, Inc., all rights reserved
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  © 1993-2008 by Apple Computer, Inc., all rights reserved
>>>>>>> origin/fixes_2.4
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
                     http://www.freepascal.org/bugs.html
 
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{    Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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
{    Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit ConditionalMacros;
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
=======
>>>>>>> graemeg/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
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
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := FALSE}
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
>>>>>>> graemeg/cpstrnew
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
{$endc} {not MACOSALLINCLUDE}
{$ALIGN POWER}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD


{
     File:       ConditionalMacros.h
 
     Contains:   Set up for compiler independent conditionals
 
     Version:    CarbonCore-769~1
  
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
                     http://developer.apple.com/bugreporter/
 
}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
{$ALIGN MAC68K}
>>>>>>> graemeg/fixes_2_2
=======
{$ALIGN MAC68K}
>>>>>>> origin/fixes_2_2
=======
{$endc} {not MACOSALLINCLUDE}
{$ALIGN POWER}
>>>>>>> origin/fixes_2.4

{***************************************************************************************************
    UNIVERSAL_INTERFACES_VERSION
    
        0x0400 --> version 4.0 (Mac OS X only)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        0x0335 --> version 3.4 
=======
        0x0341 --> version 3.4.1 
        0x0340 --> version 3.4 
>>>>>>> graemeg/fixes_2_2
=======
        0x0341 --> version 3.4.1 
        0x0340 --> version 3.4 
>>>>>>> origin/fixes_2_2
=======
        0x0335 --> version 3.4 
>>>>>>> origin/fixes_2.4
        0x0331 --> version 3.3.1
        0x0330 --> version 3.3
        0x0320 --> version 3.2
        0x0310 --> version 3.1
        0x0301 --> version 3.0.1
        0x0300 --> version 3.0
        0x0210 --> version 2.1
        This conditional did not exist prior to version 2.1
***************************************************************************************************}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
// defined in the conversion script as a macro }
// const
//   UNIVERSAL_INTERFACES_VERSION = $0400;
{***************************************************************************************************

    All TARGET_* condtionals are set up by TargetConditionals.h
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

***************************************************************************************************}
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======

***************************************************************************************************}
>>>>>>> graemeg/cpstrnew
=======

***************************************************************************************************}
>>>>>>> origin/cpstrnew

***************************************************************************************************}

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
{***************************************************************************************************

    PRAGMA_*
=======
=======
>>>>>>> origin/fixes_2_2


=======
// defined in the conversion script as a macro }
// const
//   UNIVERSAL_INTERFACES_VERSION = $0400;
>>>>>>> origin/fixes_2.4
{***************************************************************************************************

    All TARGET_* condtionals are set up by TargetConditionals.h

***************************************************************************************************}


{***************************************************************************************************

<<<<<<< HEAD
    PRAGMA_Å
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
    PRAGMA_*
>>>>>>> origin/fixes_2.4
    These conditionals specify whether the compiler supports particular #pragma's
    
        PRAGMA_IMPORT           - Compiler supports: #pragma import on/off/reset
        PRAGMA_ONCE             - Compiler supports: #pragma once
        PRAGMA_STRUCT_ALIGN     - Compiler supports: #pragma options align=mac68k/power/reset
        PRAGMA_STRUCT_PACK      - Compiler supports: #pragma pack(n)
        PRAGMA_STRUCT_PACKPUSH  - Compiler supports: #pragma pack(push, n)/pack(pop)
        PRAGMA_ENUM_PACK        - Compiler supports: #pragma options(!pack_enums)
        PRAGMA_ENUM_ALWAYSINT   - Compiler supports: #pragma enumsalwaysint on/off/reset
        PRAGMA_ENUM_OPTIONS     - Compiler supports: #pragma options enum=int/small/reset


    FOUR_CHAR_CODE
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    This conditional is deprecated.  It was used to work around a bug in one obscure compiler that did not pack multiple characters in single quotes rationally.
    It was never intended for endian swapping.
=======
    This conditional does the proper byte swapping to assue that a four character code (e.g. 'TEXT')
    is compiled down to the correct value on all compilers.
>>>>>>> graemeg/fixes_2_2
=======
    This conditional does the proper byte swapping to assue that a four character code (e.g. 'TEXT')
    is compiled down to the correct value on all compilers.
>>>>>>> origin/fixes_2_2
=======
    This conditional is deprecated.  It was used to work around a bug in one obscure compiler that did not pack multiple characters in single quotes rationally.
    It was never intended for endian swapping.
>>>>>>> origin/fixes_2.4

        FourCharCode('abcd')  - Convert a four-char-code to the correct 32-bit value


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    TYPE_*
=======
    TYPE_Å
>>>>>>> graemeg/fixes_2_2
=======
    TYPE_Å
>>>>>>> origin/fixes_2_2
=======
    TYPE_*
>>>>>>> origin/fixes_2.4
    These conditionals specify whether the compiler supports particular types.

        TYPE_LONGLONG               - Compiler supports "long long" 64-bit integers
        TYPE_BOOL                   - Compiler supports "bool"
        TYPE_EXTENDED               - Compiler supports "extended" 80/96 bit floating point
        TYPE_LONGDOUBLE_IS_DOUBLE   - Compiler implements "long double" same as "double"


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FUNCTION_*
=======
    FUNCTION_Å
>>>>>>> graemeg/fixes_2_2
=======
    FUNCTION_Å
>>>>>>> origin/fixes_2_2
=======
    FUNCTION_*
>>>>>>> origin/fixes_2.4
    These conditionals specify whether the compiler supports particular language extensions
    to function prototypes and definitions.

        FUNCTION_PASCAL         - Compiler supports "pascal void Foo()"
        FUNCTION_DECLSPEC       - Compiler supports "__declspec(xxx) void Foo()"
        FUNCTION_WIN32CC        - Compiler supports "void __cdecl Foo()" and "void __stdcall Foo()"

***************************************************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{***************************************************************************************************

    Under MacOS, the classic 68k runtime has two calling conventions: pascal or C
    Under Win32, there are two calling conventions: __cdecl or __stdcall
    Headers and implementation files can use the following macros to make their
    source more portable by hiding the calling convention details:

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    EXTERN_API*
=======
    EXTERN_APIÅ 
>>>>>>> graemeg/cpstrnew
=======
    EXTERN_APIÅ 
>>>>>>> graemeg/cpstrnew
=======
    EXTERN_APIÅ 
>>>>>>> graemeg/cpstrnew
=======
    EXTERN_APIÅ 
>>>>>>> origin/cpstrnew
=======
    EXTERN_APIÅ 
>>>>>>> origin/fixes_2.4
    These macros are used to specify the calling convention on a function prototype.

        EXTERN_API              - Classic 68k: pascal, Win32: __cdecl
        EXTERN_API_C            - Classic 68k: C,      Win32: __cdecl
        EXTERN_API_STDCALL      - Classic 68k: pascal, Win32: __stdcall
        EXTERN_API_C_STDCALL    - Classic 68k: C,      Win32: __stdcall


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    DEFINE_API*
=======
    DEFINE_APIÅ 
>>>>>>> graemeg/cpstrnew
=======
    DEFINE_APIÅ 
>>>>>>> graemeg/cpstrnew
=======
    DEFINE_APIÅ 
>>>>>>> graemeg/cpstrnew
=======
    DEFINE_APIÅ 
>>>>>>> origin/cpstrnew
=======
    DEFINE_APIÅ 
>>>>>>> origin/fixes_2.4
    These macros are used to specify the calling convention on a function definition.

        DEFINE_API              - Classic 68k: pascal, Win32: __cdecl
        DEFINE_API_C            - Classic 68k: C,      Win32: __cdecl
        DEFINE_API_STDCALL      - Classic 68k: pascal, Win32: __stdcall
        DEFINE_API_C_STDCALL    - Classic 68k: C,      Win32: __stdcall


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    CALLBACK_API*
=======
    CALLBACK_APIÅ   
>>>>>>> graemeg/cpstrnew
=======
    CALLBACK_APIÅ   
>>>>>>> graemeg/cpstrnew
=======
    CALLBACK_APIÅ   
>>>>>>> graemeg/cpstrnew
=======
    CALLBACK_APIÅ   
>>>>>>> origin/cpstrnew
=======
    CALLBACK_APIÅ   
>>>>>>> origin/fixes_2.4
    These macros are used to specify the calling convention of a function pointer.

        CALLBACK_API            - Classic 68k: pascal, Win32: __stdcall
        CALLBACK_API_C          - Classic 68k: C,      Win32: __stdcall
        CALLBACK_API_STDCALL    - Classic 68k: pascal, Win32: __cdecl
        CALLBACK_API_C_STDCALL  - Classic 68k: C,      Win32: __cdecl

***************************************************************************************************}

{***************************************************************************************************
    
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Set up TARGET_API_*_* values
=======
    Set up TARGET_API_Å_Å values
>>>>>>> graemeg/cpstrnew
=======
    Set up TARGET_API_Å_Å values
>>>>>>> graemeg/cpstrnew
=======
    Set up TARGET_API_Å_Å values
>>>>>>> graemeg/cpstrnew
=======
    Set up TARGET_API_Å_Å values
>>>>>>> origin/cpstrnew

***************************************************************************************************}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
    Set up TARGET_API_Å_Å values

***************************************************************************************************}
>>>>>>> origin/fixes_2.4

{***************************************************************************************************
    Backward compatibility for clients expecting 2.x version on ConditionalMacros.h

    GENERATINGPOWERPC       - Compiler is generating PowerPC instructions
    GENERATING68K           - Compiler is generating 68k family instructions
    GENERATING68881         - Compiler is generating mc68881 floating point instructions
    GENERATINGCFM           - Code being generated assumes CFM calling conventions
    CFMSYSTEMCALLS          - No A-traps.  Systems calls are made using CFM and UPP's
    PRAGMA_ALIGN_SUPPORTED  - Compiler supports: #pragma options align=mac68k/power/reset
    PRAGMA_IMPORT_SUPPORTED - Compiler supports: #pragma import on/off/reset
    CGLUESUPPORTED          - Clients can use all lowercase toolbox functions that take C strings instead of pascal strings

***************************************************************************************************}


{***************************************************************************************************

    OLDROUTINENAMES         - "Old" names for Macintosh system calls are allowed in source code.
                              (e.g. DisposPtr instead of DisposePtr). The names of system routine
                              are now more sensitive to change because CFM binds by name.  In the 
                              past, system routine names were compiled out to just an A-Trap.  
                              Macros have been added that each map an old name to its new name.  
                              This allows old routine names to be used in existing source files,
                              but the macros only work if OLDROUTINENAMES is true.  This support
                              will be removed in the near future.  Thus, all source code should 
                              be changed to use the new names! You can set OLDROUTINENAMES to false
                              to see if your code has any old names left in it.
    
***************************************************************************************************}


{***************************************************************************************************
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 The following macros isolate the use of 68K inlines in function prototypes.
    On the Mac OS under the Classic 68K runtime, function prototypes were followed
 by a list of 68K opcodes which the compiler inserted in the generated code instead
 of a JSR.  Under Classic 68K on the Mac OS, this macro will put the opcodes
    in the right syntax.  For all other OS's and runtimes the macro suppress the opcodes.
  Example:
   
       EXTERN_P void DrawPicture(PicHandle myPicture, const Rect *dstRect)
            ONEWORDINLINE(0xA8F6);
 
***************************************************************************************************}

{***************************************************************************************************
<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

    TARGET_CARBON                   - default: false. Switches all of the above as described.  Overrides all others
                                    - NOTE: If you set TARGET_CARBON to 1, then the other switches will be setup by
                                            ConditionalMacros, and should not be set manually.

    If you wish to do development for pre-Carbon Systems, you can set the following:

    OPAQUE_TOOLBOX_STRUCTS          - default: false. True for Carbon builds, hides struct fields.
    OPAQUE_UPP_TYPES                - default: false. True for Carbon builds, UPP types are unique and opaque.
    ACCESSOR_CALLS_ARE_FUNCTIONS    - default: false. True for Carbon builds, enables accessor functions.
    CALL_NOT_IN_CARBON              - default: true.  False for Carbon builds, hides calls not supported in Carbon.
    
    Specifically, if you are building a non-Carbon application (one that links against InterfaceLib)
    but you wish to use some of the accessor functions, you can set ACCESSOR_CALLS_ARE_FUNCTIONS to 1
    and link with CarbonAccessors.o, which implements just the accessor functions. This will help you
    preserve source compatibility between your Carbon and non-Carbon application targets.
    
    MIXEDMODE_CALLS_ARE_FUNCTIONS   - deprecated.

***************************************************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{
     * It's possible to have ACCESSOR_CALLS_ARE_FUNCTIONS set to true and OPAQUE_TOOLBOX_STRUCTS
     * set to false, but not the other way around, so make sure the defines are not set this way.
     }
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
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
