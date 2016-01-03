{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       HIServices/HIShape.h
 
     Contains:   Generic Abstract Shape API
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Version:    HIServices-416~44
=======
     Version:    HIServices-308~1
>>>>>>> graemeg/cpstrnew
=======
     Version:    HIServices-308~1
>>>>>>> graemeg/cpstrnew
=======
     Version:    HIServices-308~1
>>>>>>> graemeg/cpstrnew
=======
     Version:    HIServices-308~1
>>>>>>> origin/cpstrnew
 
     Copyright:  © 2001-2008 by Apple Computer, Inc., all rights reserved.
=======
=======
>>>>>>> origin/fixes_2_2
     File:       HIToolbox/HIShape.h
=======
     File:       HIServices/HIShape.h
>>>>>>> origin/fixes_2.4
 
     Contains:   Generic Abstract Shape API
 
     Version:    HIServices-308~1
 
<<<<<<< HEAD
     Copyright:  © 2001-2005 by Apple Computer, Inc., all rights reserved.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  © 2001-2008 by Apple Computer, Inc., all rights reserved.
>>>>>>> origin/fixes_2.4
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
=======
                     http://www.freepascal.org/bugs.html
>>>>>>> graemeg/fixes_2_2
=======
                     http://www.freepascal.org/bugs.html
>>>>>>> origin/fixes_2_2
 
}
{     File:       HIShape.p(.pas)																		}
{ 																										}
{     Contains:   CodeWarrior Pascal(GPC) translation of Apple's Mac OS X 10.2 introduced HIShape.h.	}
{				  Translation compatible with make-gpc-interfaces.pl generated MWPInterfaces            }
{                 (GPCPInterfaces) and Mac OS X 10.2.x or higher.  The CodeWarrior Pascal translation   }
{                 is linkable with Mac OS X 10.2.x or higher CFM CarbonLib.  The GPC translation is     }
{                 linkable with Mac OS X 10.2.x or higher Mach-O Carbon.framework.                      }
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
{     Translation assisted by:                                                                          }
{This file was processed by Dan's Source Converter}
{version 1.3 (this version modified by Ingemar Ragnemalm)}
{       Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
=======
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
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit HIShape;
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
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
>>>>>>> graemeg/cpstrnew
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
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
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
<<<<<<< HEAD
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
=======
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
<<<<<<< HEAD
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
>>>>>>> graemeg/cpstrnew
=======
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
uses MacTypes,CFBase,CGBase,CGContext,CGGeometry,Drag,QuickdrawTypes,CarbonEvents,HIGeometry;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

<<<<<<< HEAD
=======
uses MacTypes,CFBase,CGContext,Drag,Quickdraw,CarbonEvents,HIGeometry;
>>>>>>> graemeg/fixes_2_2
=======
uses MacTypes,CFBase,CGContext,Drag,Quickdraw,CarbonEvents,HIGeometry;
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{$ALIGN POWER}


{
 *  HIShape
 *  
 *  Discussion:
 *    HIShape is an abstract shape object for use with some of the
 *    HIToolbox APIs. It is designed as a replacement for RgnHandles
 *    (though it is currently implemented in terms of them at the time
 *    of this writing). This abstraction will allow us to avoid using
 *    QD types in our APIs, particularly in HIView. 
 *    
 *    One of the biggest benefits of HIShape is that we have mutable
 *    and immutable variants. This means that immutable shapes can be
 *    created and passed around and 'copied' very quickly, since they
 *    are actually refcounted when copied. This avoids needing to do
 *    the handle-to-handle copies that occur right now with
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    RgnHandle-based APIs. 
 *    
 *    Thread Safety Information 
 *    
 *    On Mac OS X 10.5 and later, all of the HIShape APIs are thread
 *    safe. However, only the APIs are thread safe; the HIShapeRefs
 *    themselves are not. It is safe to call HIShape APIs on multiple
 *    threads, so long as no two threads try to operate on the same
 *    HIShapeRef at the same time. 
 *    
 *    If you need multiple threads to operate on a single HIShapeRef at
 *    the same time, you must implement your own locking mechanism.
<<<<<<< HEAD
 }
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	HIShapeRef = ^__HIShape; { an opaque type }
	__HIShape = record end;
	HIMutableShapeRef = ^__HIShape; { an opaque type }
=======
	HIShapeRef = ^SInt32; { an opaque type }
	HIMutableShapeRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	HIShapeRef = ^SInt32; { an opaque type }
	HIMutableShapeRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	HIShapeRef = ^SInt32; { an opaque type }
	HIMutableShapeRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	HIShapeRef = ^SInt32; { an opaque type }
	HIMutableShapeRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew

{
 *  Summary:
 *    Messages passed to an HIShapeEnumeratorProcPtr callback.
 }
const
{
   * The callback receives this message at the beginning of enumeration.
   }
	kHIShapeEnumerateInit = 1;

  {
   * The callback receives this message when it is passed a rectangular
   * portion of the shape.
   }
	kHIShapeEnumerateRect = 2;
<<<<<<< HEAD
=======

  {
   * The callback receives this message at the end of enumeration.
   }
	kHIShapeEnumerateTerminate = 3;
>>>>>>> origin/cpstrnew

  {
   * The callback receives this message at the end of enumeration.
   }
	kHIShapeEnumerateTerminate = 3;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD


{
 *  Summary:
 *    Options for HIShapeEnumerate.
 }
const
{
   * Enumeration should begin at the top of the shape. This is the
   * default behavior.
   }
	kHIShapeParseFromTop = 0;

  {
   * Enumeration should begin at the bottom of the shape.
   }
	kHIShapeParseFromBottom = 1 shl 0;

  {
   * Enumeration should begin at the left side of the shape. This is
   * the default behavior.
   }
	kHIShapeParseFromLeft = 0;

  {
   * Enumeration should begin at the right side of the shape.
   }
	kHIShapeParseFromRight = 1 shl 1;

  {
   * Enumeration should begin at the top left corner of the shape. This
   * is the default behavior.
   }
	kHIShapeParseFromTopLeft = kHIShapeParseFromTop or kHIShapeParseFromLeft;

  {
   * Enumeration should begin at the bottom right corner of the shape.
   }
	kHIShapeParseFromBottomRight = kHIShapeParseFromBottom or kHIShapeParseFromRight;


{
=======


{
 *  Summary:
 *    Options for HIShapeEnumerate.
 }
const
{
   * Enumeration should begin at the top of the shape. This is the
   * default behavior.
   }
	kHIShapeParseFromTop = 0;

  {
   * Enumeration should begin at the bottom of the shape.
   }
	kHIShapeParseFromBottom = 1 shl 0;

  {
   * Enumeration should begin at the left side of the shape. This is
   * the default behavior.
   }
	kHIShapeParseFromLeft = 0;

  {
   * Enumeration should begin at the right side of the shape.
   }
	kHIShapeParseFromRight = 1 shl 1;

  {
   * Enumeration should begin at the top left corner of the shape. This
   * is the default behavior.
   }
	kHIShapeParseFromTopLeft = kHIShapeParseFromTop or kHIShapeParseFromLeft;

  {
   * Enumeration should begin at the bottom right corner of the shape.
   }
	kHIShapeParseFromBottomRight = kHIShapeParseFromBottom or kHIShapeParseFromRight;


{
>>>>>>> graemeg/cpstrnew
=======


{
=======


{
>>>>>>> graemeg/cpstrnew
 *  Summary:
 *    Options for HIShapeEnumerate.
 }
const
{
   * Enumeration should begin at the top of the shape. This is the
   * default behavior.
   }
	kHIShapeParseFromTop = 0;

  {
   * Enumeration should begin at the bottom of the shape.
   }
	kHIShapeParseFromBottom = 1 shl 0;

  {
   * Enumeration should begin at the left side of the shape. This is
   * the default behavior.
   }
	kHIShapeParseFromLeft = 0;

  {
   * Enumeration should begin at the right side of the shape.
   }
	kHIShapeParseFromRight = 1 shl 1;

  {
   * Enumeration should begin at the top left corner of the shape. This
   * is the default behavior.
   }
	kHIShapeParseFromTopLeft = kHIShapeParseFromTop or kHIShapeParseFromLeft;

  {
   * Enumeration should begin at the bottom right corner of the shape.
   }
	kHIShapeParseFromBottomRight = kHIShapeParseFromBottom or kHIShapeParseFromRight;


{
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
 *  HIShapeEnumerateProcPtr
 *  
 *  Summary:
 *    Callback function that receives rectangles parsed from an HIShape.
 *  
 *  Parameters:
 *    
 *    inMessage:
 *      One of kHIShapeEnumerateInit, kHIShapeEnumerateRect, or
 *      kHIShapeEnumerateTerminate.
 *    
 *    inShape:
 *      The shape being enumerated.
 *    
 *    inRect:
 *      If inMessage is kHIShapeEnumerateRect, this parameter is a
 *      rectangle that forms part of the shape. If inMessage is
 *      kHIShapeEnumerateInit or kHIShapeEnumerateTerminate, the value
 *      of this parameter is undefined.
 *    
 *    inRefcon:
 *      Client-provided data that was passed to HIShapeEnumerate.
 *  
 *  Result:
 *    The callback should return noErr to continue enumeration, or any
 *    other error code to stop enumeration. If the callback returns a
 *    value other than noErr in response to the Init message, then the
 *    callback will not be called with the Rect or Terminate messages.
 *    If the callback returns a value other than noErr in response to a
 *    Rect message, then the callback will still be called with a
 *    Terminate message. The first value other than noErr returned by
 *    the callback is also returned from HIShapeEnumerate.
 }
type
	HIShapeEnumerateProcPtr = function( inMessage: SInt32; inShape: HIShapeRef; const (*var*) inRect: CGRect; inRefcon: UnivPtr ): OSStatus;
{
 *  Summary:
 *    Options for HIShapeEnumerate.
 }
const
{
   * Enumeration should begin at the top of the shape. This is the
   * default behavior.
   }
	kHIShapeParseFromTop = 0;

  {
   * Enumeration should begin at the bottom of the shape.
   }
	kHIShapeParseFromBottom = 1 shl 0;

  {
   * Enumeration should begin at the left side of the shape. This is
   * the default behavior.
   }
	kHIShapeParseFromLeft = 0;

  {
   * Enumeration should begin at the right side of the shape.
   }
	kHIShapeParseFromRight = 1 shl 1;

  {
   * Enumeration should begin at the top left corner of the shape. This
   * is the default behavior.
   }
	kHIShapeParseFromTopLeft = kHIShapeParseFromTop or kHIShapeParseFromLeft;

  {
   * Enumeration should begin at the bottom right corner of the shape.
   }
	kHIShapeParseFromBottomRight = kHIShapeParseFromBottom or kHIShapeParseFromRight;


{
 *  HIShapeEnumerateProcPtr
 *  
 *  Summary:
 *    Callback function that receives rectangles parsed from an HIShape.
 *  
 *  Parameters:
 *    
 *    inMessage:
 *      One of kHIShapeEnumerateInit, kHIShapeEnumerateRect, or
 *      kHIShapeEnumerateTerminate.
 *    
 *    inShape:
 *      The shape being enumerated.
 *    
 *    inRect:
 *      If inMessage is kHIShapeEnumerateRect, this parameter is a
 *      rectangle that forms part of the shape. If inMessage is
 *      kHIShapeEnumerateInit or kHIShapeEnumerateTerminate, the value
 *      of this parameter is undefined.
 *    
 *    inRefcon:
 *      Client-provided data that was passed to HIShapeEnumerate.
 *  
 *  Result:
 *    The callback should return noErr to continue enumeration, or any
 *    other error code to stop enumeration. If the callback returns a
 *    value other than noErr in response to the Init message, then the
 *    callback will not be called with the Rect or Terminate messages.
 *    If the callback returns a value other than noErr in response to a
 *    Rect message, then the callback will still be called with a
 *    Terminate message. The first value other than noErr returned by
 *    the callback is also returned from HIShapeEnumerate.
 }
type
	HIShapeEnumerateProcPtr = function( inMessage: SInt32; inShape: HIShapeRef; const (*var*) inRect: CGRect; inRefcon: UnivPtr ): OSStatus;
{
 *  HIShapeGetTypeID()
 *  
 *  Discussion:
 *    Returns the CoreFoundation type ID for the HIShape class.
 *    
 *    
 *    This API is thread safe only on Mac OS X 10.5 and later; see the
 *    Thread Safety Information section at the top of the header for
 *    other important details.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Result:
 *    A CoreFoundation type ID.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
=======
=======
>>>>>>> origin/fixes_2_2
 *    RgnHandle-based APIs.
=======
>>>>>>> origin/fixes_2.4
 }
type
	HIShapeRef = ^SInt32; { an opaque type }
	HIMutableShapeRef = ^SInt32; { an opaque type }

{
 *  Summary:
 *    Messages passed to an HIShapeEnumeratorProcPtr callback.
 }
const
{
   * The callback receives this message at the beginning of enumeration.
   }
	kHIShapeEnumerateInit = 1;

  {
   * The callback receives this message when it is passed a rectangular
   * portion of the shape.
   }
	kHIShapeEnumerateRect = 2;

  {
   * The callback receives this message at the end of enumeration.
   }
	kHIShapeEnumerateTerminate = 3;


{
 *  Summary:
 *    Options for HIShapeEnumerate.
 }
const
{
   * Enumeration should begin at the top of the shape. This is the
   * default behavior.
   }
	kHIShapeParseFromTop = 0;

  {
   * Enumeration should begin at the bottom of the shape.
   }
	kHIShapeParseFromBottom = 1 shl 0;

  {
   * Enumeration should begin at the left side of the shape. This is
   * the default behavior.
   }
	kHIShapeParseFromLeft = 0;

  {
   * Enumeration should begin at the right side of the shape.
   }
	kHIShapeParseFromRight = 1 shl 1;

  {
   * Enumeration should begin at the top left corner of the shape. This
   * is the default behavior.
   }
	kHIShapeParseFromTopLeft = kHIShapeParseFromTop or kHIShapeParseFromLeft;

  {
   * Enumeration should begin at the bottom right corner of the shape.
   }
	kHIShapeParseFromBottomRight = kHIShapeParseFromBottom or kHIShapeParseFromRight;


{
 *  HIShapeEnumerateProcPtr
 *  
 *  Summary:
 *    Callback function that receives rectangles parsed from an HIShape.
 *  
 *  Parameters:
 *    
 *    inMessage:
 *      One of kHIShapeEnumerateInit, kHIShapeEnumerateRect, or
 *      kHIShapeEnumerateTerminate.
 *    
 *    inShape:
 *      The shape being enumerated.
 *    
 *    inRect:
 *      If inMessage is kHIShapeEnumerateRect, this parameter is a
 *      rectangle that forms part of the shape. If inMessage is
 *      kHIShapeEnumerateInit or kHIShapeEnumerateTerminate, the value
 *      of this parameter is undefined.
 *    
 *    inRefcon:
 *      Client-provided data that was passed to HIShapeEnumerate.
 *  
 *  Result:
 *    The callback should return noErr to continue enumeration, or any
 *    other error code to stop enumeration. If the callback returns a
 *    value other than noErr in response to the Init message, then the
 *    callback will not be called with the Rect or Terminate messages.
 *    If the callback returns a value other than noErr in response to a
 *    Rect message, then the callback will still be called with a
 *    Terminate message. The first value other than noErr returned by
 *    the callback is also returned from HIShapeEnumerate.
 }
type
	HIShapeEnumerateProcPtr = function( inMessage: SInt32; inShape: HIShapeRef; const (*var*) inRect: CGRect; inRefcon: UnivPtr ): OSStatus;
{
 *  HIShapeGetTypeID()
 *  
 *  Discussion:
 *    Returns the CoreFoundation type ID for the HIShape class.
 *    
 *    
 *    This API is thread safe only on Mac OS X 10.5 and later; see the
 *    Thread Safety Information section at the top of the header for
 *    other important details.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Result:
 *    A CoreFoundation type ID.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIShapeGetTypeID: CFTypeID; external name '_HIShapeGetTypeID';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{======================================================================================}
{  IMMUTABLE FUNCTIONS                                                                 }
{======================================================================================}
{
 *  HIShapeCreateEmpty()
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    Creates an immutable empty shape. Useful at times. 
 *    
 *    This API is thread safe only on Mac OS X 10.5 and later; see the
 *    Thread Safety Information section at the top of the header for
 *    other important details.
<<<<<<< HEAD
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
=======
=======
>>>>>>> origin/fixes_2_2
 *    Creates an immutable empty shape. Useful at times.
 *  
 *  Mac OS X threading:
 *    Not thread safe
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
>>>>>>> origin/fixes_2.4
 *  
 *  Result:
 *    An immutable, empty HIShape reference.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function HIShapeCreateEmpty: HIShapeRef; external name '_HIShapeCreateEmpty';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  HIShapeCreateWithQDRgn()
 *  
 *  Discussion:
 *    Creates an immutable shape based on an existing Quickdraw region
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    handle. 
 *    
 *    This API is thread safe only on Mac OS X 10.5 and later; see the
 *    Thread Safety Information section at the top of the header for
 *    other important details.
<<<<<<< HEAD
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
=======
=======
>>>>>>> origin/fixes_2_2
 *    handle.
 *  
 *  Mac OS X threading:
 *    Not thread safe
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
>>>>>>> origin/fixes_2.4
 *  
 *  Parameters:
 *    
 *    inRgn:
 *      The Quickdraw region from which to create the HIShape.
 *  
 *  Result:
 *    An immutable HIShape reference.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIShapeCreateWithQDRgn( inRgn: RgnHandle ): HIShapeRef; external name '_HIShapeCreateWithQDRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  HIShapeCreateWithRect()
 *  
 *  Discussion:
 *    Creates an immutable, rectangular shape based on a given
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    rectangle. 
 *    
 *    This API is thread safe only on Mac OS X 10.5 and later; see the
 *    Thread Safety Information section at the top of the header for
 *    other important details.
<<<<<<< HEAD
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
=======
=======
>>>>>>> origin/fixes_2_2
 *    rectangle.
 *  
 *  Mac OS X threading:
 *    Not thread safe
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
>>>>>>> origin/fixes_2.4
 *  
 *  Parameters:
 *    
 *    inRect:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      The CGRect from which to create the resulting shape.
=======
 *      The HIRect from which to create the resulting shape.
>>>>>>> graemeg/fixes_2_2
=======
 *      The HIRect from which to create the resulting shape.
>>>>>>> origin/fixes_2_2
=======
 *      The CGRect from which to create the resulting shape.
>>>>>>> origin/fixes_2.4
 *  
 *  Result:
 *    An immutable HIShape reference.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIShapeCreateWithRect( const (*var*) inRect: CGRect ): HIShapeRef; external name '_HIShapeCreateWithRect';
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIShapeCreateWithRect( const (*var*) inRect: HIRect ): HIShapeRef; external name '_HIShapeCreateWithRect';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIShapeCreateWithRect( const (*var*) inRect: CGRect ): HIShapeRef; external name '_HIShapeCreateWithRect';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  HIShapeCreateCopy()
 *  
 *  Discussion:
 *    Creates an immutable copy of a mutable or immutable HIShape.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    
 *    
 *    This API is thread safe only on Mac OS X 10.5 and later; see the
 *    Thread Safety Information section at the top of the header for
 *    other important details.
<<<<<<< HEAD
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
=======
 *  
 *  Mac OS X threading:
 *    Not thread safe
>>>>>>> graemeg/fixes_2_2
=======
 *  
 *  Mac OS X threading:
 *    Not thread safe
>>>>>>> origin/fixes_2_2
=======
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
>>>>>>> origin/fixes_2.4
 *  
 *  Parameters:
 *    
 *    inShape:
 *      The existing HIShapeRef you wish to copy.
 *  
 *  Result:
 *    An immutable HIShape reference.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIShapeCreateCopy( inShape: HIShapeRef ): HIShapeRef; external name '_HIShapeCreateCopy';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  HIShapeCreateIntersection()
 *  
 *  Discussion:
 *    Creates a new immutable shape which is the intersection of two
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    others. 
 *    
 *    This API is thread safe only on Mac OS X 10.5 and later; see the
 *    Thread Safety Information section at the top of the header for
 *    other important details.
<<<<<<< HEAD
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
=======
=======
>>>>>>> origin/fixes_2_2
 *    others.
 *  
 *  Mac OS X threading:
 *    Not thread safe
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
>>>>>>> origin/fixes_2.4
 *  
 *  Parameters:
 *    
 *    inShape1:
 *      An existing HIShapeRef.
 *    
 *    inShape2:
 *      An existing HIShapeRef.
 *  
 *  Result:
 *    A new immutable HIShapeRef.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIShapeCreateIntersection( inShape1: HIShapeRef; inShape2: HIShapeRef ): HIShapeRef; external name '_HIShapeCreateIntersection';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  HIShapeCreateDifference()
 *  
 *  Discussion:
 *    Creates a new immutable shape which is the difference of two
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    others. The second shape is subtracted from the first. 
 *    
 *    This API is thread safe only on Mac OS X 10.5 and later; see the
 *    Thread Safety Information section at the top of the header for
 *    other important details.
<<<<<<< HEAD
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
=======
=======
>>>>>>> origin/fixes_2_2
 *    others. The second shape is subtracted from the first.
 *  
 *  Mac OS X threading:
 *    Not thread safe
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
>>>>>>> origin/fixes_2.4
 *  
 *  Parameters:
 *    
 *    inShape1:
 *      An existing HIShapeRef.
 *    
 *    inShape2:
 *      An existing HIShapeRef.
 *  
 *  Result:
 *    A new immutable HIShapeRef.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIShapeCreateDifference( inShape1: HIShapeRef; inShape2: HIShapeRef ): HIShapeRef; external name '_HIShapeCreateDifference';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  HIShapeCreateUnion()
 *  
 *  Discussion:
 *    Creates a new immutable shape which is the union of two others.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    
 *    
 *    This API is thread safe only on Mac OS X 10.5 and later; see the
 *    Thread Safety Information section at the top of the header for
 *    other important details.
<<<<<<< HEAD
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
=======
 *  
 *  Mac OS X threading:
 *    Not thread safe
>>>>>>> graemeg/fixes_2_2
=======
 *  
 *  Mac OS X threading:
 *    Not thread safe
>>>>>>> origin/fixes_2_2
=======
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
>>>>>>> origin/fixes_2.4
 *  
 *  Parameters:
 *    
 *    inShape1:
 *      An existing HIShapeRef.
 *    
 *    inShape2:
 *      An existing HIShapeRef.
 *  
 *  Result:
 *    A new immutable HIShapeRef.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIShapeCreateUnion( inShape1: HIShapeRef; inShape2: HIShapeRef ): HIShapeRef; external name '_HIShapeCreateUnion';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  HIShapeCreateXor()
 *  
 *  Discussion:
 *    Creates a new immutable shape which is the difference between the
 *    union and the intersection of the two others. 
 *    
 *    This API is thread safe only on Mac OS X 10.5 and later; see the
 *    Thread Safety Information section at the top of the header for
 *    other important details.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    inShape1:
 *      An existing HIShapeRef.
 *    
 *    inShape2:
 *      An existing HIShapeRef.
 *  
 *  Result:
 *    A new immutable HIShapeRef.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIShapeCreateXor( inShape1: HIShapeRef; inShape2: HIShapeRef ): HIShapeRef; external name '_HIShapeCreateXor';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  HIShapeIsEmpty()
 *  
 *  Discussion:
 *    Returns true if the given HIShapeRef is empty, i.e. its area is
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    empty. 
 *    
 *    This API is thread safe only on Mac OS X 10.5 and later; see the
 *    Thread Safety Information section at the top of the header for
 *    other important details.
<<<<<<< HEAD
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
=======
=======
>>>>>>> origin/fixes_2_2
 *    empty.
 *  
 *  Mac OS X threading:
 *    Not thread safe
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
>>>>>>> origin/fixes_2.4
 *  
 *  Parameters:
 *    
 *    inShape:
 *      The existing HIShapeRef you wish to test.
 *  
 *  Result:
 *    A boolean result.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIShapeIsEmpty( inShape: HIShapeRef ): Boolean; external name '_HIShapeIsEmpty';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  HIShapeIsRectangular()
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    Returns true if the given HIShapeRef is rectangular. 
 *    
 *    This API is thread safe only on Mac OS X 10.5 and later; see the
 *    Thread Safety Information section at the top of the header for
 *    other important details.
<<<<<<< HEAD
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
=======
=======
>>>>>>> origin/fixes_2_2
 *    Returns true if the given HIShapeRef is rectangular.
 *  
 *  Mac OS X threading:
 *    Not thread safe
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
>>>>>>> origin/fixes_2.4
 *  
 *  Parameters:
 *    
 *    inShape:
 *      The existing HIShapeRef you wish to test.
 *  
 *  Result:
 *    A boolean result.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIShapeIsRectangular( inShape: HIShapeRef ): Boolean; external name '_HIShapeIsRectangular';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  HIShapeContainsPoint()
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    Returns true if the given HIShapeRef contains the point passed
 *    in. 
 *    
 *    This API is thread safe only on Mac OS X 10.5 and later; see the
 *    Thread Safety Information section at the top of the header for
 *    other important details.
<<<<<<< HEAD
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
=======
=======
>>>>>>> origin/fixes_2_2
 *    Returns true if the given HIShapeRef contains the point passed in.
 *  
 *  Mac OS X threading:
 *    Not thread safe
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
>>>>>>> origin/fixes_2.4
 *  
 *  Parameters:
 *    
 *    inShape:
 *      An existing HIShapeRef.
 *    
 *    inPoint:
 *      The point to check.
 *  
 *  Result:
 *    A boolean result.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIShapeContainsPoint( inShape: HIShapeRef; const (*var*) inPoint: CGPoint ): Boolean; external name '_HIShapeContainsPoint';
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIShapeContainsPoint( inShape: HIShapeRef; const (*var*) inPoint: HIPoint ): Boolean; external name '_HIShapeContainsPoint';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIShapeContainsPoint( inShape: HIShapeRef; const (*var*) inPoint: CGPoint ): Boolean; external name '_HIShapeContainsPoint';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  HIShapeIntersectsRect()
 *  
 *  Discussion:
 *    Returns true if the given HIShapeRef intersects the rect passed
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    in. 
 *    
 *    This API is thread safe only on Mac OS X 10.5 and later; see the
 *    Thread Safety Information section at the top of the header for
 *    other important details.
<<<<<<< HEAD
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
=======
=======
>>>>>>> origin/fixes_2_2
 *    in.
 *  
 *  Mac OS X threading:
 *    Not thread safe
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
>>>>>>> origin/fixes_2.4
 *  
 *  Parameters:
 *    
 *    inShape:
 *      An existing HIShapeRef.
 *    
 *    inRect:
 *      The rectangle to check.
 *  
 *  Result:
 *    A boolean result.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function HIShapeIntersectsRect( inShape: HIShapeRef; const (*var*) inRect: CGRect ): Boolean; external name '_HIShapeIntersectsRect';
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function HIShapeIntersectsRect( inShape: HIShapeRef; const (*var*) inRect: HIRect ): Boolean; external name '_HIShapeIntersectsRect';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function HIShapeIntersectsRect( inShape: HIShapeRef; const (*var*) inRect: CGRect ): Boolean; external name '_HIShapeIntersectsRect';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  HIShapeGetBounds()
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    Returns the bounding rectangle of a given HIShapeRef. 
 *    
 *    This API is thread safe only on Mac OS X 10.5 and later; see the
 *    Thread Safety Information section at the top of the header for
 *    other important details.
<<<<<<< HEAD
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
=======
=======
>>>>>>> origin/fixes_2_2
 *    Returns the bounding rectangle of a given HIShapeRef.
 *  
 *  Mac OS X threading:
 *    Not thread safe
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
>>>>>>> origin/fixes_2.4
 *  
 *  Parameters:
 *    
 *    inShape:
 *      An existing HIShapeRef.
 *    
 *    outRect:
 *      Receives the bounding rectangle.
 *  
 *  Result:
 *    A pointer to the rectangle you passed in, for convenience.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIShapeGetBounds( inShape: HIShapeRef; var outRect: CGRect ): CGRectPtr; external name '_HIShapeGetBounds';
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIShapeGetBounds( inShape: HIShapeRef; var outRect: HIRect ): HIRectPtr; external name '_HIShapeGetBounds';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIShapeGetBounds( inShape: HIShapeRef; var outRect: CGRect ): CGRectPtr; external name '_HIShapeGetBounds';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  HIShapeGetAsQDRgn()
 *  
 *  Discussion:
 *    Changes a given Quickdraw region handle to have the same shape as
 *    a given HIShapeRef. Essentially you are converting an HIShapeRef
 *    into a RgnHandle. This conversion may lose fidelity depending on
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    how the shape was created originally. 
 *    
 *    This API is thread safe only on Mac OS X 10.5 and later; see the
 *    Thread Safety Information section at the top of the header for
 *    other important details.
<<<<<<< HEAD
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
=======
=======
>>>>>>> origin/fixes_2_2
 *    how the shape was created originally.
 *  
 *  Mac OS X threading:
 *    Not thread safe
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
>>>>>>> origin/fixes_2.4
 *  
 *  Parameters:
 *    
 *    inShape:
 *      An existing HIShapeRef.
 *    
 *    outRgn:
 *      An existing region to change.
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIShapeGetAsQDRgn( inShape: HIShapeRef; outRgn: RgnHandle ): OSStatus; external name '_HIShapeGetAsQDRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  HIShapeReplacePathInCGContext()
 *  
 *  Discussion:
 *    Given an HIShapeRef and a CGContextRef, make the current path in
 *    the context represent the shape. You might use this to clip to a
 *    shape, for example. You could call this function and then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    immediately call CGContextClip. 
 *    
 *    This API is thread safe only on Mac OS X 10.5 and later; see the
 *    Thread Safety Information section at the top of the header for
 *    other important details.
<<<<<<< HEAD
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
=======
=======
>>>>>>> origin/fixes_2_2
 *    immediately call CGContextClip.
 *  
 *  Mac OS X threading:
 *    Not thread safe
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
>>>>>>> origin/fixes_2.4
 *  
 *  Parameters:
 *    
 *    inShape:
 *      An existing HIShapeRef.
 *    
 *    inContext:
 *      The context to apply the shape to.
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIShapeReplacePathInCGContext( inShape: HIShapeRef; inContext: CGContextRef ): OSStatus; external name '_HIShapeReplacePathInCGContext';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc not TARGET_CPU_64}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$ifc not TARGET_CPU_64}
>>>>>>> origin/fixes_2.4
{
 *  HIShapeSetQDClip()
 *  
 *  Discussion:
 *    Given an HIShapeRef and a Quickdraw port, set the current clip in
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    the port to the shape. 
 *    
 *    This API is thread safe only on Mac OS X 10.5 and later; see the
 *    Thread Safety Information section at the top of the header for
 *    other important details.
<<<<<<< HEAD
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
=======
=======
>>>>>>> origin/fixes_2_2
 *    the port to the shape.
 *  
 *  Mac OS X threading:
 *    Not thread safe
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
>>>>>>> origin/fixes_2.4
 *  
 *  Parameters:
 *    
 *    inShape:
 *      An existing HIShapeRef.
 *    
 *    inPort:
 *      The port to set the clip for.
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIShapeSetQDClip( inShape: HIShapeRef; inPort: CGrafPtr ): OSStatus; external name '_HIShapeSetQDClip';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{$endc} {not TARGET_CPU_64}

{
 *  HIShapeEnumerate()
 *  
 *  Discussion:
 *    Parses a shape into its constituent rectangles and passes each
 *    rectangle to a callback function. 
 *    
 *    This API is thread safe only on Mac OS X 10.5 and later; see the
 *    Thread Safety Information section at the top of the header for
 *    other important details.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    inShape:
 *      The shape to enumerate.
 *    
 *    inOptions:
 *      Options controlling how to enumerate the shape.
 *    
 *    inProc:
 *      The callback function that will be called with each rectangle.
 *    
 *    inRefcon:
 *      Extra data that will be passed to the callback function.
 *  
 *  Result:
 *    The function result is the value returned by the callback
 *    function: noErr if the callback always returns noErr, or the
 *    first non-noErr value returned by the callback.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIShapeEnumerate( inShape: HIShapeRef; inOptions: OptionBits; inProc: HIShapeEnumerateProcPtr; inRefcon: UnivPtr ): OSStatus; external name '_HIShapeEnumerate';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{======================================================================================}
{  MUTABLE FUNCTIONS                                                                   }
{======================================================================================}
{
 *  HIShapeCreateMutable()
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    Creates a new, mutable, empty shape. 
 *    
 *    This API is thread safe only on Mac OS X 10.5 and later; see the
 *    Thread Safety Information section at the top of the header for
 *    other important details.
<<<<<<< HEAD
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
=======
=======
>>>>>>> origin/fixes_2_2
 *    Creates a new, mutable, empty shape.
 *  
 *  Mac OS X threading:
 *    Not thread safe
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
>>>>>>> origin/fixes_2.4
 *  
 *  Result:
 *    A mutable shape reference.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIShapeCreateMutable: HIMutableShapeRef; external name '_HIShapeCreateMutable';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  HIShapeCreateMutableCopy()
 *  
 *  Discussion:
 *    Given an existing HIShapeRef, creates a new mutable copy.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    
 *    
 *    This API is thread safe only on Mac OS X 10.5 and later; see the
 *    Thread Safety Information section at the top of the header for
 *    other important details.
<<<<<<< HEAD
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
=======
 *  
 *  Mac OS X threading:
 *    Not thread safe
>>>>>>> graemeg/fixes_2_2
=======
 *  
 *  Mac OS X threading:
 *    Not thread safe
>>>>>>> origin/fixes_2_2
=======
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
>>>>>>> origin/fixes_2.4
 *  
 *  Parameters:
 *    
 *    inOrig:
 *      The shape to copy.
 *  
 *  Result:
 *    A mutable shape reference.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIShapeCreateMutableCopy( inOrig: HIShapeRef ): HIMutableShapeRef; external name '_HIShapeCreateMutableCopy';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  HIShapeCreateMutableWithRect()
 *  
 *  Discussion:
 *    Creates a mutable HIShapeRef based on a given rectangle. 
 *    
 *    This API is thread safe only on Mac OS X 10.5 and later; see the
 *    Thread Safety Information section at the top of the header for
 *    other important details.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    inRect:
 *      The rectangle from which to create the shape.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIShapeCreateMutableWithRect( const (*var*) inRect: CGRect ): HIMutableShapeRef; external name '_HIShapeCreateMutableWithRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
<<<<<<< HEAD
 *  HIShapeSetEmpty()
 *  
 *  Discussion:
 *    Sets a mutable shape to be an empty shape. 
 *    
 *    This API is thread safe only on Mac OS X 10.5 and later; see the
 *    Thread Safety Information section at the top of the header for
 *    other important details.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
=======
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  HIShapeSetEmpty()
 *  
 *  Discussion:
 *    Sets a mutable shape to be an empty shape. 
 *    
 *    This API is thread safe only on Mac OS X 10.5 and later; see the
 *    Thread Safety Information section at the top of the header for
 *    other important details.
 *  
 *  Mac OS X threading:
<<<<<<< HEAD
 *    Not thread safe
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Thread safe since version 10.5
>>>>>>> origin/fixes_2.4
 *  
 *  Parameters:
 *    
 *    inShape:
 *      The shape to empty.
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIShapeSetEmpty( inShape: HIMutableShapeRef ): OSStatus; external name '_HIShapeSetEmpty';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  HIShapeSetWithShape()
 *  
 *  Discussion:
 *    Sets a mutable shape to have the same contents as another shape.
 *    
 *    
 *    This API is thread safe only on Mac OS X 10.5 and later; see the
 *    Thread Safety Information section at the top of the header for
 *    other important details.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    inDestShape:
 *      The mutable destination shape.
 *    
 *    inSrcShape:
 *      The source shape. This shape's contents will be copied into
 *      inDestShape.
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIShapeSetWithShape( inDestShape: HIMutableShapeRef; inSrcShape: HIShapeRef ): OSStatus; external name '_HIShapeSetWithShape';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  HIShapeIntersect()
 *  
 *  Discussion:
 *    Takes two shapes and sets a third to be their intersection.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    
 *    
 *    This API is thread safe only on Mac OS X 10.5 and later; see the
 *    Thread Safety Information section at the top of the header for
 *    other important details.
<<<<<<< HEAD
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
=======
 *  
 *  Mac OS X threading:
 *    Not thread safe
>>>>>>> graemeg/fixes_2_2
=======
 *  
 *  Mac OS X threading:
 *    Not thread safe
>>>>>>> origin/fixes_2_2
=======
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
>>>>>>> origin/fixes_2.4
 *  
 *  Parameters:
 *    
 *    inShape1:
 *      The first shape.
 *    
 *    inShape2:
 *      The second shape.
 *    
 *    outResult:
 *      The shape to receive the result of the intersection. This can
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      be one of the source shapes. This shape must be mutable.
=======
 *      be one of the source shapes.
>>>>>>> graemeg/fixes_2_2
=======
 *      be one of the source shapes.
>>>>>>> origin/fixes_2_2
=======
 *      be one of the source shapes. This shape must be mutable.
>>>>>>> origin/fixes_2.4
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIShapeIntersect( inShape1: HIShapeRef; inShape2: HIShapeRef; outResult: HIMutableShapeRef ): OSStatus; external name '_HIShapeIntersect';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  HIShapeDifference()
 *  
 *  Discussion:
 *    Takes two shapes and sets a third to be their difference. The
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    second shape is subtracted from the first. 
 *    
 *    This API is thread safe only on Mac OS X 10.5 and later; see the
 *    Thread Safety Information section at the top of the header for
 *    other important details.
<<<<<<< HEAD
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
=======
=======
>>>>>>> origin/fixes_2_2
 *    second shape is subtracted from the first.
 *  
 *  Mac OS X threading:
 *    Not thread safe
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
>>>>>>> origin/fixes_2.4
 *  
 *  Parameters:
 *    
 *    inShape1:
 *      The first shape.
 *    
 *    inShape2:
 *      The second shape.
 *    
 *    outResult:
 *      The shape to receive the result of the intersection. This can
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      be one of the source shapes. This shape must be mutable.
=======
 *      be one of the source shapes.
>>>>>>> graemeg/fixes_2_2
=======
 *      be one of the source shapes.
>>>>>>> origin/fixes_2_2
=======
 *      be one of the source shapes. This shape must be mutable.
>>>>>>> origin/fixes_2.4
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIShapeDifference( inShape1: HIShapeRef; inShape2: HIShapeRef; outResult: HIMutableShapeRef ): OSStatus; external name '_HIShapeDifference';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  HIShapeUnion()
 *  
 *  Discussion:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    Takes two shapes and sets a third to be their union. 
 *    
 *    On Mac OS X 10.2 and 10.3, this API incorrectly required that the
 *    result shape be immutable. On Mac OS X 10.4 and later, this API
 *    correctly requires that the result shape be mutable. If you need
 *    to run on both 10.4 and earlier releases, you will need to
 *    account for this difference. 
 *    
 *    This API is thread safe only on Mac OS X 10.5 and later; see the
 *    Thread Safety Information section at the top of the header for
 *    other important details.
<<<<<<< HEAD
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
=======
=======
>>>>>>> origin/fixes_2_2
 *    Takes two shapes and sets a third to be their union.
 *  
 *  Mac OS X threading:
 *    Not thread safe
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
>>>>>>> origin/fixes_2.4
 *  
 *  Parameters:
 *    
 *    inShape1:
 *      The first shape.
 *    
 *    inShape2:
 *      The second shape.
 *    
 *    outResult:
 *      The shape to receive the result of the union. This can be one
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      of the source shapes. On Mac OS X 10.2 and 10.3, this shape
 *      must be immutable, but it will be set to the unioned shape
 *      anyways. On Mac OS X 10.4 and later, this shape must be mutable.
=======
 *      of the source shapes.
>>>>>>> graemeg/fixes_2_2
=======
 *      of the source shapes.
>>>>>>> origin/fixes_2_2
=======
 *      of the source shapes. On Mac OS X 10.2 and 10.3, this shape
 *      must be immutable, but it will be set to the unioned shape
 *      anyways. On Mac OS X 10.4 and later, this shape must be mutable.
>>>>>>> origin/fixes_2.4
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIShapeUnion( inShape1: HIShapeRef; inShape2: HIShapeRef; outResult: HIMutableShapeRef ): OSStatus; external name '_HIShapeUnion';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  HIShapeXor()
 *  
 *  Discussion:
 *    Takes two shapes and sets a third to be the difference between
 *    the union and the intersection of the two shapes. 
 *    
 *    This API is thread safe only on Mac OS X 10.5 and later; see the
 *    Thread Safety Information section at the top of the header for
 *    other important details.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    inShape1:
 *      The first shape.
 *    
 *    inShape2:
 *      The second shape.
 *    
 *    outResult:
 *      The shape to receive the result of the xor operation. This can
 *      be one of the source shapes. This shape must be mutable.
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIShapeXor( inShape1: HIShapeRef; inShape2: HIShapeRef; outResult: HIMutableShapeRef ): OSStatus; external name '_HIShapeXor';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
<<<<<<< HEAD
 *  HIShapeOffset()
 *  
 *  Discussion:
 *    Offsets a shape by some delta. 
 *    
 *    This API is thread safe only on Mac OS X 10.5 and later; see the
 *    Thread Safety Information section at the top of the header for
 *    other important details.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
=======
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  HIShapeOffset()
 *  
 *  Discussion:
 *    Offsets a shape by some delta. 
 *    
 *    This API is thread safe only on Mac OS X 10.5 and later; see the
 *    Thread Safety Information section at the top of the header for
 *    other important details.
 *  
 *  Mac OS X threading:
<<<<<<< HEAD
 *    Not thread safe
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Thread safe since version 10.5
>>>>>>> origin/fixes_2.4
 *  
 *  Parameters:
 *    
 *    inShape:
 *      The shape to offset.
 *    
 *    inDX:
 *      The delta to move the shape on the X axis.
 *    
 *    inDY:
 *      The delta to move the shape on the Y axis.
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIShapeOffset( inShape: HIMutableShapeRef; inDX: CGFloat; inDY: CGFloat ): OSStatus; external name '_HIShapeOffset';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  HIShapeInset()
 *  
 *  Discussion:
 *    Contracts or expands a shape by some delta. 
 *    
 *    This API is thread safe only on Mac OS X 10.5 and later; see the
 *    Thread Safety Information section at the top of the header for
 *    other important details.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    inShape:
 *      The shape to inset.
 *    
 *    inDX:
 *      The delta to contract the shape on the X axis. Can be negative
 *      to expand the shape.
 *    
 *    inDY:
 *      The delta to contract the shape on the Y axis. Can be negative
 *      to expand the shape.
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIShapeInset( inShape: HIMutableShapeRef; inDX: CGFloat; inDY: CGFloat ): OSStatus; external name '_HIShapeInset';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  HIShapeUnionWithRect()
 *  
 *  Discussion:
 *    Unions a shape with a rectangle. 
 *    
 *    This API is thread safe only on Mac OS X 10.5 and later; see the
 *    Thread Safety Information section at the top of the header for
 *    other important details.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    inShape:
 *      The shape to combine with the rectangle.
 *    
 *    inRect:
 *      The rectangle to combine with the shape.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIShapeUnionWithRect( inShape: HIMutableShapeRef; const (*var*) inRect: CGRect ): OSStatus; external name '_HIShapeUnionWithRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{$endc} {TARGET_OS_MAC}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function HIShapeOffset( inShape: HIMutableShapeRef; inDX: CGFloat; inDY: CGFloat ): OSStatus; external name '_HIShapeOffset';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  HIShapeInset()
 *  
 *  Discussion:
 *    Contracts or expands a shape by some delta. 
 *    
 *    This API is thread safe only on Mac OS X 10.5 and later; see the
 *    Thread Safety Information section at the top of the header for
 *    other important details.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    inShape:
 *      The shape to inset.
 *    
 *    inDX:
 *      The delta to contract the shape on the X axis. Can be negative
 *      to expand the shape.
 *    
 *    inDY:
 *      The delta to contract the shape on the Y axis. Can be negative
 *      to expand the shape.
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIShapeInset( inShape: HIMutableShapeRef; inDX: CGFloat; inDY: CGFloat ): OSStatus; external name '_HIShapeInset';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  HIShapeUnionWithRect()
 *  
 *  Discussion:
 *    Unions a shape with a rectangle. 
 *    
 *    This API is thread safe only on Mac OS X 10.5 and later; see the
 *    Thread Safety Information section at the top of the header for
 *    other important details.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    inShape:
 *      The shape to combine with the rectangle.
 *    
 *    inRect:
 *      The rectangle to combine with the shape.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function HIShapeUnionWithRect( inShape: HIMutableShapeRef; const (*var*) inRect: CGRect ): OSStatus; external name '_HIShapeUnionWithRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{$endc} {TARGET_OS_MAC}

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
