<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{
<<<<<<< HEAD
     File:       UTType.h
 
     Contains:   Public interfaces for uniform type identifiers
 
     Copyright:  (c) 2003-2012 by Apple Inc. All rights reserved.
=======
     File:       LaunchServices/UTType.h
 
     Contains:   Public interfaces for Uniform Type Identification
 
     Version:    LaunchServices-360.3~1
 
     Copyright:  © 2003-2008 by Apple Computer, Inc., all rights reserved.
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

{     File:       UTType.p(.pas)																	    }
{ 																										}
{     Contains:   CodeWarrior Pascal(GPC) translation of Apple's Mac OS X 10.3 introduced UTType.h	    }
{				  Translation compatible with make-gpc-interfaces.pl generated MWPInterfaces            }
{                 (GPCPInterfaces) and Mac OS X 10.3.x or higher.  The GPC translation is linkable with }
{                 Mac OS X 10.3.x or higher Mach-O ApplicationServices.framework.                       }  
{                 (Note:  ApplicationServices is the “umbrella” framework encapsulating the             }
{                 LaunchServices subframework which contains the UTType declared data and function      }
{                 symbols.)                                                                             }
{                                                                                                       }
{                The CodeWarrior Pascal translation is NOT linkable with Mac OS X 10.3.x or higher      }
{                CFM CarbonLib.  The declared symbols are exported only from the Mach-O                 }
{				 ApplicationServices “umbrella” framework (encapsulates the LaunchServices subframework)}
{				 and references to symbols used by CFM code must be established at runtime.  Individual }
{				 symbol references can be resolved using CFBundleGetFunctionPointerForName and          }
{				 CFBundleGetDataPointerForName (see CFBundle.p) respectively for function and data      }
{				 symbols.  For bulk resolution of Mach-O symbol references,  CFMLateImport technology   }
{				 can be used.                                                                           }
{ 																										}
{     Version:    1.0																					}
{ 																										}
{	  Pascal Translation:  Gale Paeper, <gpaeper@empirenet.com>, 2004									}
{ 																										}
{     Copyright:  Subject to the constraints of Apple's original rights, you're free to use this		}
{				  translation as you deem fit.															}
{ 																										}
{     Bugs?:      This is an AS IS translation with no express guarentees of any kind.					}
{                 If you do find a bug, please help out the Macintosh Pascal programming community by   }
{				  reporting your bug finding and possible fix to either personal e-mail to Gale Paeper	}
{				  or a posting to the MacPascal mailing list.											}
{                                                                                                       }
{
      Change History (most recent first) (DD/MM/YY):

         <1>      06/12/04    GRP     First Pascal translation of UTType.h, version LaunchServices-98~1.
}
{     Translation assisted by:                                                                          }
{This file was processed by Dan's Source Converter}
{version 1.3 (this version modified by Ingemar Ragnemalm)}

{The original source on which this file is based: }
=======
>>>>>>> origin/fixes_2.4
{
     File:       LaunchServices/UTType.h
 
     Contains:   Public interfaces for Uniform Type Identification
 
     Version:    LaunchServices-360.3~1
 
<<<<<<< HEAD
     Copyright:  © 2003 by Apple Computer, Inc., all rights reserved.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  © 2003-2008 by Apple Computer, Inc., all rights reserved.
>>>>>>> origin/fixes_2.4
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
{	 Pascal Translation Update: Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }

>>>>>>> graemeg/cpstrnew
=======
{	 Pascal Translation Update: Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }

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
                     http://www.freepascal.org/bugs.html
 
}

{	 Pascal Translation Update: Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }

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

unit UTType;
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
>>>>>>> origin/cpstrnew
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
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
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
uses MacTypes,CFBase,CFArray,CFDictionary,CFURL;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not MACOSALLINCLUDE}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

{ header available as of iPhoneOS 3.0, but can't add iPhoneOS markers yet
  because they're not yet present in the Mac OS X version of the header }
>>>>>>> graemeg/cpstrnew
=======

{ header available as of iPhoneOS 3.0, but can't add iPhoneOS markers yet
  because they're not yet present in the Mac OS X version of the header }
>>>>>>> graemeg/cpstrnew
=======

{ header available as of iPhoneOS 3.0, but can't add iPhoneOS markers yet
  because they're not yet present in the Mac OS X version of the header }
>>>>>>> graemeg/cpstrnew
=======

{ header available as of iPhoneOS 3.0, but can't add iPhoneOS markers yet
  because they're not yet present in the Mac OS X version of the header }
>>>>>>> origin/cpstrnew

{$ALIGN POWER}


=======

{$ALIGN POWER}

>>>>>>> graemeg/fixes_2_2
=======

{$ALIGN POWER}

>>>>>>> origin/fixes_2_2
=======
{$endc} {not MACOSALLINCLUDE}


{ header available as of iPhoneOS 3.0, but can't add iPhoneOS markers yet
  because they're not yet present in the Mac OS X version of the header }

{$ALIGN POWER}


>>>>>>> origin/fixes_2.4
{ ======================================================================================================== }
{ Uniform Type Identification API                                                                          }
{ ======================================================================================================== }
{
    Uniform Type Identification Primer

    Uniform Type Identifiers (or UTIs) are strings which uniquely identify
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    abstract types. They can be used to describe a file format or an
    in-memory data type, but can also be used to describe the type of
=======
    abstract types. They can be used to describe a file format or
    data type, but can also be used to describe type information for
>>>>>>> graemeg/fixes_2_2
=======
    abstract types. They can be used to describe a file format or
    data type, but can also be used to describe type information for
>>>>>>> origin/fixes_2_2
=======
    abstract types. They can be used to describe a file format or an
    in-memory data type, but can also be used to describe the type of
>>>>>>> origin/fixes_2.4
    other sorts of entities, such as directories, volumes, or packages.

    The syntax of a uniform type identifier looks like a bundle identifier.
    It has the form of a reversed DNS name, although some special top-level 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    UTI domains are reserved by Apple and are outside the current IANA 
=======
    UTI domains are reserved by Apple and are outside the currect IANA 
>>>>>>> graemeg/fixes_2_2
=======
    UTI domains are reserved by Apple and are outside the currect IANA 
>>>>>>> origin/fixes_2_2
=======
    UTI domains are reserved by Apple and are outside the current IANA 
>>>>>>> origin/fixes_2.4
    top-level Internet domain name space.

    Examples:

        public.jpeg
        public.utf16-plain-text
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        com.apple.xml-property-list
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        com.apple.appleworks.doc
>>>>>>> graemeg/cpstrnew
=======
        com.apple.xml-plist
        com.apple.appleworks.doc
>>>>>>> graemeg/fixes_2_2
=======
        com.apple.xml-plist
=======
        com.apple.xml-property-list
>>>>>>> origin/fixes_2.4
        com.apple.appleworks.doc
>>>>>>> origin/fixes_2_2

    Types which are standard or not controlled by any one organization 
    are declared in the "public" domain. Currently, public types may  
    be declared only by Apple.

    Types specific to Mac OS are declared with identifiers in the 
    com.apple.macos domain.

    Third parties should declare their own uniform type identifiers
    in their respective registered Internet domain spaces.

    Type declarations appear in bundle property lists and tell
    the system several things about a type, including the following:

    Conformance

    A type may "conform" to one or more other types. For example, the
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    type com.apple.xml-property-list conforms to both the
    com.apple.property-list and public.xml types. The public.xml 
=======
    type com.apple.macos.xml-property-list conforms to both the
    com.apple.macos.property-list and public.xml types. The public.xml 
>>>>>>> graemeg/fixes_2_2
=======
    type com.apple.macos.xml-property-list conforms to both the
    com.apple.macos.property-list and public.xml types. The public.xml 
>>>>>>> origin/fixes_2_2
=======
    type com.apple.xml-property-list conforms to both the
    com.apple.property-list and public.xml types. The public.xml 
>>>>>>> origin/fixes_2.4
    type in turn conforms to type public.text. Finally, type public.text  
    conforms to public.data, which is the base type for all types 
    describing bytes stream formats. Conformance relationships between 
    types are established in type declarations.

    Conformance relationships establish a multiple inheritanace hierarchy
    between types. Type property values may be inherited at runtime
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    according to the conformance relationships for each type. When a type's 
=======
    according to the conformance relationships each type. When a type's 
>>>>>>> graemeg/fixes_2_2
=======
    according to the conformance relationships each type. When a type's 
>>>>>>> origin/fixes_2_2
=======
    according to the conformance relationships for each type. When a type's 
>>>>>>> origin/fixes_2.4
    declaration does not include a value for particular type property, 
    then the type's supertypes are searched for a value. Supertypes are 
    searched depth-first, in the order given in the type declaration. 
    This is the only way in which the declared order of the conforms-to 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    supertypes is significant.
=======
    supertypes is signitificant.
>>>>>>> graemeg/fixes_2_2
=======
    supertypes is signitificant.
>>>>>>> origin/fixes_2_2
=======
    supertypes is significant.
>>>>>>> origin/fixes_2.4

    Tags

    A "tag" is a string which indicates a type in some other type 
    identification space, such as a filename extension, MIME Type,
    or NSPboardType. Each type declaration may include a 
    "tag specification", which is a dictionary listing all of the 
    tags associated with the type.

    A tag's "class" is the namespace of a tag: filename extension, 
    MIME type, OSType, etc. Tag classes are themselves identified by 
    uniform type identifiers so that the set of valid tag classes is 
    easily extendend in the future.

    Other Type Properties

    Type declarations may include several other properties: a localizable
    user description of the type, the name of an icon resource in
    the declaring bundle, a reference URL identifying technical 
    documentation about the type itself, and finally a version number, 
    which can be incremented as a type evolves. All of these properties
    are optional.

    Exported vs. Imported Type Declarations
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
    
>>>>>>> graemeg/fixes_2_2
=======
    
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
    Type declarations are either exported or imported. An exported
    type declaration means that the type itself is defined or owned 
    by the organization making the declaration. For example, a propietary
    document type declaration should only be exported by the application
    which controls the document format.

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    An imported declaration is for applications which depend on the
=======
    An imported declatation is for applications which depend on the
>>>>>>> graemeg/fixes_2_2
=======
    An imported declatation is for applications which depend on the
>>>>>>> origin/fixes_2_2
=======
    An imported declaration is for applications which depend on the
>>>>>>> origin/fixes_2.4
    existence of someone else's type declaration. If application A can
    open application B's document format, then application A makes
    an imported declaration of application B's document type so that
    even if application B is not present on the system, there is an
    acessible declaration of its document type.

    An exported declaration of a particular type identifier is always
    preferred over an imported declaration.

    Example XML Type Declaration

    Appearing below is an XML excerpt from a bundle Info.plist file which 
    declares the public type "public.jpeg":
    
        <key>UTExportedTypeDeclarations</key>
        <array>
            <dict>
                <key>UTTypeIdentifier</key>
                <string>public.jpeg</string>
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                <key>UTTypeDescription</key>
                <string>JPEG image</string>
                <key>UTTypeIconFile</key>
=======
=======
>>>>>>> origin/fixes_2_2
                <key>UTTypeReferenceURL</key>
                <string>http://www.w3.org/Graphics/JPEG/</string>
                <key>UTTypeDescription</key>
                <string>JPEG image</string>
                <key>UTTypeIconName</key>
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
                <key>UTTypeDescription</key>
                <string>JPEG image</string>
                <key>UTTypeIconFile</key>
>>>>>>> origin/fixes_2.4
                <string>public.jpeg.icns</string>
                <key>UTTypeConformsTo</key>
                <array>
                    <string>public.image</string>
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                </array>
                <key>UTTypeTagSpecification</key>
                <dict>
                    <key>com.apple.ostype</key>
=======
=======
>>>>>>> origin/fixes_2_2
                    <string>public.data</string>
                </array>
                <key>UTTypeTagSpecification</key>
                <dict>
                    <key>com.apple.macos.ostype</key>
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
                </array>
                <key>UTTypeTagSpecification</key>
                <dict>
                    <key>com.apple.ostype</key>
>>>>>>> origin/fixes_2.4
                    <string>JPEG</string>
                    <key>public.filename-extension</key>
                    <array>
                        <string>jpeg</string>
                        <string>jpg</string>
                    </array>
                    <key>public.mime-type</key>
                    <string>image/jpeg</string>
                </dict>
            </dict>
        </array>


    Dynamic Type Identifiers

    Uniform Type Identifiation uses dynamic type identifiers to
    represent types for which no identifier has been declared. A
    dynamic type identifier is syntactially a regular uniform
    type identifier in the "dyn" domain. However, after the
    initial domain label, a dynamic type identifier is an 
    opaque encoding of a tag specification. Dynamic type 
    identifiers cannot be declared. They are generated on-demand
    with whatever type information is available at the time, often 
    a single (otherwise unknown) type tag.

    A dynamic identifier therefore carries within it a minimal
    amount of type information, but enough to work well with the
    Uniform Type Identification API. For example, a client can
    extract from a dynamic type identifier the original tag
    specification with which it was created. A client can also
    test a dynamic type identifier for equality to another
    uniform type identifier. If the dynamic identifier's
    tag specification is a subset of the other identifier's
    tags, the two are considered equal.

    Dynamic type identifiers do not express the full richness
    of type information associated with a declared type 
    identifier, but dynamic type identifiers allow the behavior
    to degrade gracefully in the presence of incomplete 
    declared type information.

    A dynamic type identifier may be transmitted across processes
    on a given system, but it should never be stored persistently
    or transmitted over the wire to another system. In particular,
    dynamic identifiers should not appear in bundle info property
    lists, and they will generally be ignored when they do. Apple 
    reserves the right to change the opaque format of dynamic
    identifiers in future versions of Mac OS X.
}

{
    Type Declaration Dictionary Keys

    The following keys are used in type declarations
}
{
 *  kUTExportedTypeDeclarationsKey
<<<<<<< HEAD
<<<<<<< HEAD
 }
var kUTExportedTypeDeclarationsKey: CFStringRef; external name '_kUTExportedTypeDeclarationsKey'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_3_0) *)
{
 *  kUTImportedTypeDeclarationsKey
 }
var kUTImportedTypeDeclarationsKey: CFStringRef; external name '_kUTImportedTypeDeclarationsKey'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_3_0) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
=======
=======
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
var kUTExportedTypeDeclarationsKey: CFStringRef; external name '_kUTExportedTypeDeclarationsKey'; (* attribute const *)
<<<<<<< HEAD

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/fixes_2.4
{
 *  kUTImportedTypeDeclarationsKey
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 }
var kUTImportedTypeDeclarationsKey: CFStringRef; external name '_kUTImportedTypeDeclarationsKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
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
 *  kUTTypeIdentifierKey
 }
var kUTTypeIdentifierKey: CFStringRef; external name '_kUTTypeIdentifierKey'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_3_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/cpstrnew
{
 *  kUTTypeTagSpecificationKey
 }
var kUTTypeTagSpecificationKey: CFStringRef; external name '_kUTTypeTagSpecificationKey'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_3_0) *)
{
 *  kUTTypeConformsToKey
 }
var kUTTypeConformsToKey: CFStringRef; external name '_kUTTypeConformsToKey'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_3_0) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
=======
=======
>>>>>>> origin/fixes_2_2
}

// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
=======
 }
>>>>>>> origin/fixes_2.4
var kUTImportedTypeDeclarationsKey: CFStringRef; external name '_kUTImportedTypeDeclarationsKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
{
 *  kUTTypeIdentifierKey
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
var kUTTypeIdentifierKey: CFStringRef; external name '_kUTTypeIdentifierKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
{
 *  kUTTypeTagSpecificationKey
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
var kUTTypeTagSpecificationKey: CFStringRef; external name '_kUTTypeTagSpecificationKey'; (* attribute const *)
<<<<<<< HEAD

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/fixes_2.4
{
 *  kUTTypeConformsToKey
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 }
var kUTTypeConformsToKey: CFStringRef; external name '_kUTTypeConformsToKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
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
 *  kUTTypeDescriptionKey
 }
var kUTTypeDescriptionKey: CFStringRef; external name '_kUTTypeDescriptionKey'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_3_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/cpstrnew
{
 *  kUTTypeIconFileKey
 }
var kUTTypeIconFileKey: CFStringRef; external name '_kUTTypeIconFileKey'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_3_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/cpstrnew
{
 *  kUTTypeReferenceURLKey
 }
var kUTTypeReferenceURLKey: CFStringRef; external name '_kUTTypeReferenceURLKey'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_3_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/cpstrnew
{
 *  kUTTypeVersionKey
 }
var kUTTypeVersionKey: CFStringRef; external name '_kUTTypeVersionKey'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_3_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 } 
 
// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
=======
 }
>>>>>>> origin/fixes_2.4
var kUTTypeConformsToKey: CFStringRef; external name '_kUTTypeConformsToKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
{
 *  kUTTypeDescriptionKey
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
var kUTTypeDescriptionKey: CFStringRef; external name '_kUTTypeDescriptionKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
{
 *  kUTTypeIconFileKey
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
var kUTTypeIconFileKey: CFStringRef; external name '_kUTTypeIconFileKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
{
 *  kUTTypeReferenceURLKey
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
var kUTTypeReferenceURLKey: CFStringRef; external name '_kUTTypeReferenceURLKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
{
 *  kUTTypeVersionKey
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
var kUTTypeVersionKey: CFStringRef; external name '_kUTTypeVersionKey'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
    Type Tag Classes

    The following constant strings identify tag classes for use 
    when converting uniform type identifiers to and from
    equivalent tags.
}
{
 *  kUTTagClassFilenameExtension
<<<<<<< HEAD
<<<<<<< HEAD
 }
var kUTTagClassFilenameExtension: CFStringRef; external name '_kUTTagClassFilenameExtension'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_3_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/cpstrnew
{
 *  kUTTagClassMIMEType
 }
var kUTTagClassMIMEType: CFStringRef; external name '_kUTTagClassMIMEType'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_3_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/cpstrnew
{
 *  kUTTagClassNSPboardType
 }
var kUTTagClassNSPboardType: CFStringRef; external name '_kUTTagClassNSPboardType'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/cpstrnew
{
 *  kUTTagClassOSType
 }
var kUTTagClassOSType: CFStringRef; external name '_kUTTagClassOSType'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
var kUTTagClassFilenameExtension: CFStringRef; external name '_kUTTagClassFilenameExtension'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
{
 *  kUTTagClassMIMEType
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
var kUTTagClassMIMEType: CFStringRef; external name '_kUTTagClassMIMEType'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
{
 *  kUTTagClassNSPboardType
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
var kUTTagClassNSPboardType: CFStringRef; external name '_kUTTagClassNSPboardType'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
{
 *  kUTTagClassOSType
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
var kUTTagClassOSType: CFStringRef; external name '_kUTTagClassOSType'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/fixes_2.4

{
 *  UTTypeCreatePreferredIdentifierForTag()
 *  
 *  Discussion:
 *    Creates a uniform type identifier for the type indicated by the
 *    specified tag. This is the primary function to use for going from
 *    tag (extension/MIMEType/OSType) to uniform type identifier.
 *    Optionally, the returned type identifiers must conform to the
 *    identified "conforming-to" type argument. This is a hint to the
 *    implementation to constrain the search to a particular tree of
 *    types. For example, the client may want to know the type
 *    indicated by a particular extension tag. If the client knows that
 *    the extension is associated with a directory (rather than a
 *    file), the client may specify "public.directory" for the
 *    conforming-to argument. This will allow the implementation to
 *    ignore all types associated with byte data formats (public.data
 *    base type). If more than one type is indicated, preference is
 *    given to a public type over a non-public type on the theory that
 *    instances of public types are more common, and therefore more
 *    likely to be correct. When there a choice must be made between
 *    multiple public types or multiple non-public types, the selection
 *    rules are undefined. Clients needing finer control should use
<<<<<<< HEAD
<<<<<<< HEAD
 *    UTTypeCreateAllIdentifiersForTag. If no declared type is
=======
 *    UTTypeCreateAllIdentifiersWithTag. If no declared type is
>>>>>>> graemeg/fixes_2_2
=======
 *    UTTypeCreateAllIdentifiersWithTag. If no declared type is
>>>>>>> origin/fixes_2_2
 *    indicated, a dynamic type identifier is generated which satisfies
 *    the parameters.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    inTagClass:
 *      the class identifier of the tag argument
 *    
 *    inTag:
 *      the tag string
 *    
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    inConformingToUTI:
=======
 *    inConformingToTypeIdentifier:
>>>>>>> graemeg/fixes_2_2
=======
 *    inConformingToTypeIdentifier:
>>>>>>> origin/fixes_2_2
=======
 *    inConformingToUTI:
>>>>>>> origin/fixes_2.4
 *      the identifier of a type to which the result must conform
 *  
 *  Result:
 *    a new CFStringRef containing the type identifier, or NULL if
 *    inTagClass is not a known tag class
<<<<<<< HEAD
<<<<<<< HEAD
 }
function UTTypeCreatePreferredIdentifierForTag( inTagClass: CFStringRef; inTag: CFStringRef; inConformingToUTI: CFStringRef { can be NULL } ): CFStringRef; external name '_UTTypeCreatePreferredIdentifierForTag';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_3_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD

// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER 
function UTTypeCreatePreferredIdentifierForTag(
  inTagClass: CFStringRef;
  inTag: CFStringRef;
  inConformingToTypeIdentifier: CFStringRef): CFStringRef; external name '_UTTypeCreatePreferredIdentifierForTag'; { can be NULL }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function UTTypeCreatePreferredIdentifierForTag( inTagClass: CFStringRef; inTag: CFStringRef; inConformingToUTI: CFStringRef { can be NULL } ): CFStringRef; external name '_UTTypeCreatePreferredIdentifierForTag';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  UTTypeCreateAllIdentifiersForTag()
 *  
 *  Discussion:
 *    Creates an array of all uniform type identifiers indicated by the
 *    specified tag. An overloaded tag (e.g., an extension used by
 *    several applications for different file formats) may indicate
 *    multiple types. If no declared type identifiers have the
 *    specified tag, then a single dynamic type identifier will be
 *    created for the tag. Optionally, the returned type identifiers
 *    must conform to the identified "conforming-to" type argument.
 *    This is a hint to the implementation to constrain the search to a
 *    particular tree of types. For example, the client may want to
 *    know the type indicated by a particular extension tag. If the
 *    client knows that the extension is associated with a directory
 *    (rather than a file), the client may specify "public.directory"
 *    for the conforming-to argument. This will allow the
 *    implementation to ignore all types associated with byte data
 *    formats (public.data base type).
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    inTagClass:
 *      the class identifier of the tag argument
 *    
 *    inTag:
 *      the tag string
 *    
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    inConformingToUTI:
=======
 *    inConformingToTypeIdentifier:
>>>>>>> graemeg/fixes_2_2
=======
 *    inConformingToTypeIdentifier:
>>>>>>> origin/fixes_2_2
=======
 *    inConformingToUTI:
>>>>>>> origin/fixes_2.4
 *      the identifier of a type to which the results must conform
 *  
 *  Result:
 *    An array of uniform type identifiers, or NULL if inTagClass is
 *    not a known tag class
<<<<<<< HEAD
<<<<<<< HEAD
 }
function UTTypeCreateAllIdentifiersForTag( inTagClass: CFStringRef; inTag: CFStringRef; inConformingToUTI: CFStringRef { can be NULL } ): CFArrayRef; external name '_UTTypeCreateAllIdentifiersForTag';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_3_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD

// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function UTTypeCreateAllIdentifiersForTag(
  inTagClass: CFStringRef;
  inTag: CFStringRef;
  inConformingToTypeIdentifier: CFStringRef): CFArrayRef; external name '_UTTypeCreateAllIdentifiersForTag'; { can be NULL } 
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function UTTypeCreateAllIdentifiersForTag( inTagClass: CFStringRef; inTag: CFStringRef; inConformingToUTI: CFStringRef { can be NULL } ): CFArrayRef; external name '_UTTypeCreateAllIdentifiersForTag';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  UTTypeCopyPreferredTagWithClass()
 *  
 *  Discussion:
 *    Returns the identified type's preferred tag with the specified
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    tag class as a CFString. This is the primary function to use for
=======
 *    tag class as a CSString. This is the primary function to use for
>>>>>>> graemeg/fixes_2_2
=======
 *    tag class as a CSString. This is the primary function to use for
>>>>>>> origin/fixes_2_2
=======
 *    tag class as a CFString. This is the primary function to use for
>>>>>>> origin/fixes_2.4
 *    going from uniform type identifier to tag. If the type
 *    declaration included more than one tag with the specified class,
 *    the first tag in the declared tag array is the preferred tag.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    inUTI:
=======
 *    inTypeIdentifier:
>>>>>>> graemeg/fixes_2_2
=======
 *    inTypeIdentifier:
>>>>>>> origin/fixes_2_2
=======
 *    inUTI:
>>>>>>> origin/fixes_2.4
 *      the uniform type identifier
 *    
 *    inTagClass:
 *      the class of tags to return
 *  
 *  Result:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    the tag string, or NULL if there is no tag of the specified class.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 }
function UTTypeCopyPreferredTagWithClass( inUTI: CFStringRef; inTagClass: CFStringRef ): CFStringRef; external name '_UTTypeCopyPreferredTagWithClass';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_3_0) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
 *    an array of tags (as CFStrings), or NULL if there are no tags
 *    with the specified class.
>>>>>>> graemeg/fixes_2_2
=======
 *    an array of tags (as CFStrings), or NULL if there are no tags
 *    with the specified class.
>>>>>>> origin/fixes_2_2
=======
 *    the tag string, or NULL if there is no tag of the specified class.
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function UTTypeCopyPreferredTagWithClass( inUTI: CFStringRef; inTagClass: CFStringRef ): CFStringRef; external name '_UTTypeCopyPreferredTagWithClass';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
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

// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function UTTypeCopyPreferredTagWithClass(
  inTypeIdentifier: CFStringRef;
  inTagClass: CFStringRef): CFStringRef; external name '_UTTypeCopyPreferredTagWithClass';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function UTTypeCopyPreferredTagWithClass( inUTI: CFStringRef; inTagClass: CFStringRef ): CFStringRef; external name '_UTTypeCopyPreferredTagWithClass';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  UTTypeEqual()
 *  
 *  Discussion:
 *    Compares two identified types for equality. Types are equal if
 *    their identifier strings are equal using a case-insensitive
 *    comparison. In addition, if one or both of the identifiers is a
 *    dynamic identifier, then the types are equal if either
 *    identifier's tag specification is a subset of the other
 *    identifier's tag specification.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    inUTI1:
 *      a uniform type identifier
 *    
 *    inUTI2:
 *      another uniform type identifier
 }
function UTTypeEqual( inUTI1: CFStringRef; inUTI2: CFStringRef ): Boolean; external name '_UTTypeEqual';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_3_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *    inTypeIdentifier1:
=======
 *    inUTI1:
>>>>>>> origin/fixes_2.4
 *      a uniform type identifier
 *    
 *    inUTI2:
 *      another uniform type identifier
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD

// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function UTTypeEqual(
  inTypeIdentifier1: CFStringRef;
  inTypeIdentifier2: CFStringRef): Boolean; external name '_UTTypeEqual';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function UTTypeEqual( inUTI1: CFStringRef; inUTI2: CFStringRef ): Boolean; external name '_UTTypeEqual';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  UTTypeConformsTo()
 *  
 *  Discussion:
 *    Tests for a conformance relationship between the two identified
 *    types. Returns true if the types are equal, or if the first type
 *    conforms, directly or indirectly, to the second type.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    inUTI:
 *      the uniform type identifier to test
 *    
 *    inConformsToUTI:
 *      the uniform type identifier against which to test conformance.
 }
function UTTypeConformsTo( inUTI: CFStringRef; inConformsToUTI: CFStringRef ): Boolean; external name '_UTTypeConformsTo';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_3_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *    inTypeIdentifier:
=======
 *    inUTI:
>>>>>>> origin/fixes_2.4
 *      the uniform type identifier to test
 *    
 *    inConformsToUTI:
 *      the uniform type identifier against which to test conformance.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD

// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function UTTypeConformsTo(
  inTypeIdentifier: CFStringRef;
  inConformsToTypeIdentifier: CFStringRef): Boolean; external name '_UTTypeConformsTo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function UTTypeConformsTo( inUTI: CFStringRef; inConformsToUTI: CFStringRef ): Boolean; external name '_UTTypeConformsTo';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  UTTypeCopyDescription()
 *  
 *  Discussion:
 *    Returns the localized, user-readable type description string
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    inUTI:
=======
 *    inTypeIdentifier:
>>>>>>> graemeg/fixes_2_2
=======
 *    inTypeIdentifier:
>>>>>>> origin/fixes_2_2
=======
 *    inUTI:
>>>>>>> origin/fixes_2.4
 *      the uniform type identifier
 *  
 *  Result:
 *    a localized string, or NULL of no type description is available
<<<<<<< HEAD
<<<<<<< HEAD
 }
function UTTypeCopyDescription( inUTI: CFStringRef ): CFStringRef; external name '_UTTypeCopyDescription';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_3_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD

// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function UTTypeCopyDescription(inTypeIdentifier: CFStringRef): CFStringRef; external name '_UTTypeCopyDescription';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function UTTypeCopyDescription( inUTI: CFStringRef ): CFStringRef; external name '_UTTypeCopyDescription';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  UTTypeCopyDeclaration()
 *  
 *  Discussion:
 *    Returns the identified type's declaration dictionary, as it
 *    appears in the declaring bundle's info property list. This the
 *    access path to other type properties for which direct access is
 *    rarely needed.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    inUTI:
=======
 *    inTypeIdentifier:
>>>>>>> graemeg/fixes_2_2
=======
 *    inTypeIdentifier:
>>>>>>> origin/fixes_2_2
=======
 *    inUTI:
>>>>>>> origin/fixes_2.4
 *      the uniform type identifier
 *  
 *  Result:
 *    a tag declaration dictionary, or NULL if the type is not declared
<<<<<<< HEAD
<<<<<<< HEAD
 }
function UTTypeCopyDeclaration( inUTI: CFStringRef ): CFDictionaryRef; external name '_UTTypeCopyDeclaration';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_3_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD

// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function UTTypeCopyDeclaration(inTypeIdentifier: CFStringRef): CFDictionaryRef; external name '_UTTypeCopyDeclaration';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function UTTypeCopyDeclaration( inUTI: CFStringRef ): CFDictionaryRef; external name '_UTTypeCopyDeclaration';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  UTTypeCopyDeclaringBundleURL()
 *  
 *  Discussion:
 *    Returns the URL of the bundle containing the type declaration of
 *    the identified type.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    inUTI:
=======
 *    inTypeIdentifier:
>>>>>>> graemeg/fixes_2_2
=======
 *    inTypeIdentifier:
>>>>>>> origin/fixes_2_2
=======
 *    inUTI:
>>>>>>> origin/fixes_2.4
 *      the uniform type identifier
 *  
 *  Result:
 *    a URL, or NULL if the bundle cannot be located.
<<<<<<< HEAD
<<<<<<< HEAD
 }
function UTTypeCopyDeclaringBundleURL( inUTI: CFStringRef ): CFURLRef; external name '_UTTypeCopyDeclaringBundleURL';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_3_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD

// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function UTTypeCopyDeclaringBundleURL(inTypeIdentifier: CFStringRef): CFURLRef; external name '_UTTypeCopyDeclaringBundleURL';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function UTTypeCopyDeclaringBundleURL( inUTI: CFStringRef ): CFURLRef; external name '_UTTypeCopyDeclaringBundleURL';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  UTCreateStringForOSType()
 *  
 *  Discussion:
 *    A helper function to canonically encode an OSType as a CFString
 *    suitable for use as a tag argument.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    inOSType:
 *      the OSType value to encode
 *  
 *  Result:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    a new CFString representing the OSType
 }
function UTCreateStringForOSType( inOSType: OSType ): CFStringRef; external name '_UTCreateStringForOSType';
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_NA) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 *    a new CFString representing the OSType. Returns the empty string
 *    when the argument is 0
=======
 *    a new CFString representing the OSType, or NULL if the argument
 *    is 0 or '????'
>>>>>>> graemeg/fixes_2_2
=======
 *    a new CFString representing the OSType, or NULL if the argument
 *    is 0 or '????'
>>>>>>> origin/fixes_2_2
=======
 *    a new CFString representing the OSType. Returns the empty string
 *    when the argument is 0
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function UTCreateStringForOSType( inOSType: OSType ): CFStringRef; external name '_UTCreateStringForOSType';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
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

// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function UTCreateStringForOSType(inOSType: OSType): CFStringRef; external name '_UTCreateStringForOSType';
>>>>>>> graemeg/fixes_2_2
=======

// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function UTCreateStringForOSType(inOSType: OSType): CFStringRef; external name '_UTCreateStringForOSType';
>>>>>>> origin/fixes_2_2
=======
function UTCreateStringForOSType( inOSType: OSType ): CFStringRef; external name '_UTCreateStringForOSType';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  UTGetOSTypeFromString()
 *  
 *  Discussion:
 *    A helper function to canonically decode a string-encoded OSType
 *    back to the original OSType value.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    inString:
 *      the string to decode
 *  
 *  Result:
 *    the OSType value encoded in the string, or 0 if the string is not
 *    a valid encoding of an OSType
<<<<<<< HEAD
<<<<<<< HEAD
 }
function UTGetOSTypeFromString( inString: CFStringRef ): OSType; external name '_UTGetOSTypeFromString';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_3,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/cpstrnew

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function UTGetOSTypeFromString( inString: CFStringRef ): OSType; external name '_UTGetOSTypeFromString';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

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
