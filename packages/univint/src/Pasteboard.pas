
{     File:       Pasteboard.p(.pas)																	}
{ 																										}
{     Contains:   CodeWarrior Pascal(GPC) translation of Apple's Mac OS X 10.3 introduced Pasteboard.h	}
{				  Translation compatible with make-gpc-interfaces.pl generated MWPInterfaces            }
{                 (GPCPInterfaces) and Mac OS X 10.3.x or higher.  The CodeWarrior Pascal translation   }
{                 is linkable with Mac OS X 10.3.x or higher CFM CarbonLib.  The GPC translation is     }
{                 linkable with Mac OS X 10.3.x or higher Mach-O Carbon.framework.                      }
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

         <1>      05/12/04    GRP     First Pascal translation of Pasteboard.h, version HIServices-125.6~1.
}
{     Translation assisted by:                                                                          }
{This file was processed by Dan's Source Converter}
{version 1.3 (this version modified by Ingemar Ragnemalm)}

{The original source on which this file is based: }
{  
     File:       HIServices/Pasteboard.h
 
     Contains:   Pasteboard Manager Interfaces.
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
 
     Copyright:  © 2003-2008 by Apple Computer, Inc., all rights reserved.
=======
     Version:    HIServices-125.6~1
 
     Copyright:  © 2003 by Apple Computer, Inc., all rights reserved.
>>>>>>> graemeg/fixes_2_2
=======
     Version:    HIServices-125.6~1
 
     Copyright:  © 2003 by Apple Computer, Inc., all rights reserved.
>>>>>>> origin/fixes_2_2
=======
     Version:    HIServices-308~1
 
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
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
=======
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit Pasteboard;
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
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
=======
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
>>>>>>> origin/cpstrnew
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
>>>>>>> origin/cpstrnew
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
<<<<<<< HEAD
{$elsec}
=======
=======
>>>>>>> graemeg/cpstrnew
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
>>>>>>> graemeg/cpstrnew
=======
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
uses MacTypes,CFBase,CFArray,CFData,CFURL;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}
<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{$ALIGN POWER}


{
 *  Pasteboard Manager
 *  
 *  Discussion:
 *    Pasteboard Manager is the replacement of the Scrap and Drag
 *    flavor APIs. Like the Scrap Manager, it is a cross process data
 *    transport mechanism. Unlike the Drag Manager, it is not bound to
 *    particular high level user actions to initiate and resolve the
 *    data transport. This allows the Pasteboard Manager to be used as
 *    the data transport mechanism for copy and paste, drag and drop,
 *    services, as well as generic cross process communication.
 *    
 *    
 *    Like scraps, pasteboards are local references to global, cross
 *    process, resources. Although, where scraps exist forever,
 *    pasteboard are CFTypes and should be released at the end of their
 *    usefulness. Multiple pasteboard references in a single
 *    application may point to the same global pasteboard resource.
 *    Once the global pasteboard resource has been cleared, all
 *    pasteboard references to that global resource in the application
 *    share ownership allowing any of them to add data to the
 *    pasteboard. Of course, this is not a free for all. Once a flavor
 *    has been added it can not be overwritten by another local
 *    reference. Calling CFRelease() on any one of these references
 *    will implicitly call in promises, but only those made through the
 *    particular pasteboard reference being released. In this way,
 *    local pasteboard references in various components of the
 *    application can add data, make promises, keep those promises, and
 *    release their local pasteboard reference independently of the
 *    other components. 
 *    
 *    For long lived pasteboards, like the clipboard or find
 *    pasteboards, the suggested usage model is for a component to
 *    create a local reference to a pasteboard at the beginning of its
 *    life cycle, hold on to it, and release it at the end of its life
 *    cycle. As long as a pasteboard reference in any application still
 *    points to the global pasteboard resource, the global resource
 *    will not be released. When the final reference is released,
 *    promises will not be called in as the global pasteboard resource
 *    will be released. Only if you want to guarantee a global
 *    pasteboard resource continues to exist beyond the life of your
 *    application should you refrain from calling CFRelease(). In this
 *    case, be sure to call PasteboardResolvePromises() to make sure
 *    any promises are called in. This is not necessary for the
 *    clipboard and find pasteboards as they are retained by the system
 *    and never released. 
 *    
 *    Like the Drag Manager, the Pasteboard Manager allows the use of
 *    multiple items each with its own set of flavors. When using the
 *    clipboard or find pasteboards, it's traditionally been common to
 *    only use the first item, but now multiple items can be copied as
 *    is the case in the Finder. This also opens up the ability for
 *    services to act on multiple items. Another difference from both
 *    the Scrap and Drag Managers is the use of Uniform Type Identifier
 *    based flavor types rather than four character code OSTypes. These
 *    have several advantages. They allow compatibility with Cocoa's
 *    NSPasteboard, more accurately describe the data being
 *    transported, provide a type inheritance mechanism and allow
 *    namespacing with a reverse DNS scheme.
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	PasteboardRef = ^OpaquePasteboardRef; { an opaque type }
	OpaquePasteboardRef = record end;
=======
	PasteboardRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	PasteboardRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	PasteboardRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	PasteboardRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
	PasteboardItemID = UnivPtr;
{ Pasteboard Manager error codes}
const
	badPasteboardSyncErr = -25130; { pasteboard has been modified and must be synchronized}
	badPasteboardIndexErr = -25131; { item index does not exist}
	badPasteboardItemErr = -25132; { item reference does not exist}
	badPasteboardFlavorErr = -25133; { item flavor does not exist}
	duplicatePasteboardFlavorErr = -25134; { item flavor already exists}
	notPasteboardOwnerErr = -25135; { client did not clear the pasteboard}
	noPasteboardPromiseKeeperErr = -25136; { a promise is being added without a promise keeper}
=======
=======
>>>>>>> origin/fixes_2_2
 
=======
>>>>>>> origin/fixes_2.4
type
	PasteboardRef = ^SInt32; { an opaque type }
	PasteboardItemID = UnivPtr;
{ Pasteboard Manager error codes}
const
<<<<<<< HEAD
  badPasteboardSyncErr          = -25130; { pasteboard has been modified and must be synchronized }
  badPasteboardIndexErr         = -25131; { item index does not exist }
  badPasteboardItemErr          = -25132; { item reference does not exist }
  badPasteboardFlavorErr        = -25133; { item flavor does not exist }
  duplicatePasteboardFlavorErr  = -25134; { item flavor already exists }
  notPasteboardOwnerErr         = -25135; { client did not clear the pasteboard }
  noPasteboardPromiseKeeperErr  = -25136; { a promise is being added without a promise keeper }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	badPasteboardSyncErr = -25130; { pasteboard has been modified and must be synchronized}
	badPasteboardIndexErr = -25131; { item index does not exist}
	badPasteboardItemErr = -25132; { item reference does not exist}
	badPasteboardFlavorErr = -25133; { item flavor does not exist}
	duplicatePasteboardFlavorErr = -25134; { item flavor already exists}
	notPasteboardOwnerErr = -25135; { client did not clear the pasteboard}
	noPasteboardPromiseKeeperErr = -25136; { a promise is being added without a promise keeper}
>>>>>>> origin/fixes_2.4


{
 *  PasteboardSyncFlags
 *  
 *  Summary:
 *    The following constants are used by the PasteboardSynchronize()
 *    routine to indicate the status of the local pasteboard reference
 *    in relation to the global, cross process pasteboard resource.
 }
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	PasteboardSyncFlags = OptionBits;
const
{
=======
=======
>>>>>>> origin/fixes_2_2
    PasteboardSyncFlags	= UInt32;

const

  {
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	PasteboardSyncFlags = OptionBits;
const
{
>>>>>>> origin/fixes_2.4
   * Indicates that the global pasteboard resource has been modified
   * since the last time it was accessed via the local pasteboard
   * reference. The call to PasteboardSynchronize() has updated the
   * local pasteboard reference to sync it up with the global resource.
   * This is a good time to see what new information has been placed on
   * the pasteboard to determine whether any tasty flavors have been
   * added and possibly enable pasting.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kPasteboardModified = 1 shl 0;

  {
=======
    kPasteboardModified           = $00000001; {(1 << 0)}

 {
>>>>>>> graemeg/fixes_2_2
=======
    kPasteboardModified           = $00000001; {(1 << 0)}

 {
>>>>>>> origin/fixes_2_2
=======
	kPasteboardModified = 1 shl 0;

  {
>>>>>>> origin/fixes_2.4
   * Indicates that the global pasteboard resource was most recently
   * cleared by the this application. Any local pasteboard reference in
   * the client application may add data to the global pasteboard
   * resource.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kPasteboardClientIsOwner = 1 shl 1;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
	kPasteboardClientIsOwner = 1 shl 1;
>>>>>>> origin/fixes_2.4

{
 *  Pasteboard File Promising
 *  
 *  Summary:
 *    With the FSSpec type being deprecated and removed for 64 bit it is necessary
 *    to introduce a replacement for kDragFlavorTypePromiseHFS. The replacement comes
 *    in the form of two new Uniform Type Identifiers specifically for use with the
 *    pasteboard and promised files. Like the old HFS promise mechanism, the new UTI
 *    based method still requires a multistage handshake between sender and receiver
 *    but the process is somewhat simplified.
 *    
 *    Order of operations on copy or drag
 *    
 *    1) The sender promises kPasteboardTypeFileURLPromise for a file yet to be created.
 *    2) The sender adds kPasteboardTypeFilePromiseContent containing the UTI describing
 *          the file's content.
 *    
 *    Order of operations on paste or drop
 *    
 *    3) The receiver asks for kPasteboardTypeFilePromiseContent to decide if it wants the file.
 *    4) The receiver sets the paste location with PasteboardSetPasteLocation.
 *    5) The receiver asks for kPasteboardTypeFileURLPromise.
 *    6) The sender's promise callback for kPasteboardTypeFileURLPromise is called.
 *    7) The sender uses PasteboardCopyPasteLocation to retrieve the paste location, creates the file
 *          and keeps its kPasteboardTypeFileURLPromise promise.
 *
 *    Automatic translation support has been added so clients operating in the modern
 *    kPasteboardTypeFileURLPromise and kPasteboardTypeFilePromiseContent world can continue
 *    to communicate properly with clients using the traditional kDragFlavorTypePromiseHFS and
 *    kDragPromisedFlavor model.
 }

{
 *  kPasteboardTypeFileURLPromise
 *  
 *  Discussion:
 *    A UTF-8 encoded promised file url on the pasteboard to a file
 *    which does not yet exist.
 }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPasteboardTypeFileURLPromise CFSTRP('com.apple.pasteboard.promised-file-url')}
{$endc}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{
=======

{
 *  Pasteboard File Promising
 *  
 *  Summary:
 *    With the FSSpec type being deprecated and removed for 64 bit it is necessary
 *    to introduce a replacement for kDragFlavorTypePromiseHFS. The replacement comes
 *    in the form of two new Uniform Type Identifiers specifically for use with the
 *    pasteboard and promised files. Like the old HFS promise mechanism, the new UTI
 *    based method still requires a multistage handshake between sender and receiver
 *    but the process is somewhat simplified.
 *    
 *    Order of operations on copy or drag
 *    
 *    1) The sender promises kPasteboardTypeFileURLPromise for a file yet to be created.
 *    2) The sender adds kPasteboardTypeFilePromiseContent containing the UTI describing
 *          the file's content.
 *    
 *    Order of operations on paste or drop
 *    
 *    3) The receiver asks for kPasteboardTypeFilePromiseContent to decide if it wants the file.
 *    4) The receiver sets the paste location with PasteboardSetPasteLocation.
 *    5) The receiver asks for kPasteboardTypeFileURLPromise.
 *    6) The sender's promise callback for kPasteboardTypeFileURLPromise is called.
 *    7) The sender uses PasteboardCopyPasteLocation to retrieve the paste location, creates the file
 *          and keeps its kPasteboardTypeFileURLPromise promise.
 *
 *    Automatic translation support has been added so clients operating in the modern
 *    kPasteboardTypeFileURLPromise and kPasteboardTypeFilePromiseContent world can continue
 *    to communicate properly with clients using the traditional kDragFlavorTypePromiseHFS and
 *    kDragPromisedFlavor model.
 }

{
 *  kPasteboardTypeFileURLPromise
 *  
 *  Discussion:
 *    A UTF-8 encoded promised file url on the pasteboard to a file
 *    which does not yet exist.
 }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPasteboardTypeFileURLPromise CFSTRP('com.apple.pasteboard.promised-file-url')}
{$endc}

{
>>>>>>> graemeg/cpstrnew
=======

{
>>>>>>> graemeg/cpstrnew
=======

{
>>>>>>> origin/cpstrnew
 *  kPasteboardTypeFilePromiseContent
 *  
 *  Discussion:
 *    A UTF-8 encoded UTI describing the type of data to be contained
 *    within the promised file.
 }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPasteboardTypeFilePromiseContent CFSTRP('com.apple.pasteboard.promised-file-content-type')}
{$endc}
=======
    kPasteboardClientIsOwner      = $00000002; {(1 << 1)}

>>>>>>> graemeg/fixes_2_2
=======
    kPasteboardClientIsOwner      = $00000002; {(1 << 1)}

>>>>>>> origin/fixes_2_2
=======

{
 *  kPasteboardTypeFilePromiseContent
 *  
 *  Discussion:
 *    A UTF-8 encoded UTI describing the type of data to be contained
 *    within the promised file.
 }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPasteboardTypeFilePromiseContent CFSTRP('com.apple.pasteboard.promised-file-content-type')}
{$endc}
>>>>>>> origin/fixes_2.4

{
 *  PasteboardFlavorFlags
 *  
 *  Summary:
 *    Pasteboard Flavor Flags
 *  
 *  Discussion:
 *    The following constants are used to tag pasteboard item flavors
 *    with bits of useful information. The first five are settable by
 *    the client via PasteboardPutItemFlavor(). They may all be
 *    received via PasteboardGetItemFlavorFlags().
 }
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	PasteboardFlavorFlags = OptionBits;
const
{
   * No additional information exists for this flavor.
   }
	kPasteboardFlavorNoFlags = 0;
=======
=======
>>>>>>> origin/fixes_2_2
    PasteboardFlavorFlags	= UInt32;

=======
	PasteboardFlavorFlags = OptionBits;
>>>>>>> origin/fixes_2.4
const
{
   * No additional information exists for this flavor.
   }
<<<<<<< HEAD
    kPasteboardFlavorNoFlags      		= $00000000;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kPasteboardFlavorNoFlags = 0;
>>>>>>> origin/fixes_2.4

  {
   * Only the process which has added this flavor can see it. If the
   * process that owns the pasteboard places this flag on a flavor
   * you'll never see it as the receiver so there's no reason to test
   * for it.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kPasteboardFlavorSenderOnly = 1 shl 0;
=======
    kPasteboardFlavorSenderOnly   		= $00000001; {(1 << 0)}
>>>>>>> graemeg/fixes_2_2
=======
    kPasteboardFlavorSenderOnly   		= $00000001; {(1 << 0)}
>>>>>>> origin/fixes_2_2
=======
	kPasteboardFlavorSenderOnly = 1 shl 0;
>>>>>>> origin/fixes_2.4

  {
   * The data for this flavor was translated in some manner by the
   * sender before adding it to the pasteboard. Flavors marked with
   * this flag are not stored by the Finder in clipping files.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kPasteboardFlavorSenderTranslated = 1 shl 1;
=======
    kPasteboardFlavorSenderTranslated		= $00000002; {(1 << 1)}
>>>>>>> graemeg/fixes_2_2
=======
    kPasteboardFlavorSenderTranslated		= $00000002; {(1 << 1)}
>>>>>>> origin/fixes_2_2
=======
	kPasteboardFlavorSenderTranslated = 1 shl 1;
>>>>>>> origin/fixes_2.4

  {
   * Set by the sender if the flavor data should not be saved by the
   * receiver. The data contained may become stale after the end of the
   * drag. Flavors marked with this flag are not stored by the Finder
   * in clipping files.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kPasteboardFlavorNotSaved = 1 shl 2;
=======
    kPasteboardFlavorNotSaved     			= $00000004; {(1 << 2)}
>>>>>>> graemeg/fixes_2_2
=======
    kPasteboardFlavorNotSaved     			= $00000004; {(1 << 2)}
>>>>>>> origin/fixes_2_2
=======
	kPasteboardFlavorNotSaved = 1 shl 2;
>>>>>>> origin/fixes_2.4

  {
   * Like kPasteboardFlavorSenderOnly, when the sender adds this flag,
   * the presence of this flavor will not be made known when
   * PasteboardCopyItemFlavors() is called by the receiver. However,
   * the flavor flags and data can be accessed when explicitly
   * requested via PasteboardGetItemFlavorFlags() or
   * PasteboardCopyItemFlavorData(). This allows a suite of
   * applications to communicate without exporting their method of
   * communication.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kPasteboardFlavorRequestOnly = 1 shl 3;
=======
    kPasteboardFlavorRequestOnly  			= $00000008; {(1 << 3)}
>>>>>>> graemeg/fixes_2_2
=======
    kPasteboardFlavorRequestOnly  			= $00000008; {(1 << 3)}
>>>>>>> origin/fixes_2_2
=======
	kPasteboardFlavorRequestOnly = 1 shl 3;
>>>>>>> origin/fixes_2.4

  {
   * The data for this flavor is provided by the Translation Manager as
   * a convenience to receivers. Flavors marked with this flag are not
   * stored by the Finder in clipping files. This flag can not be added
   * by clients. It is automatically added by the Pasteboard Manager
   * when appropriate.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kPasteboardFlavorSystemTranslated = 1 shl 8;
=======
    kPasteboardFlavorSystemTranslated = $00000100; {(1 << 8)}
>>>>>>> graemeg/fixes_2_2
=======
    kPasteboardFlavorSystemTranslated = $00000100; {(1 << 8)}
>>>>>>> origin/fixes_2_2
=======
	kPasteboardFlavorSystemTranslated = 1 shl 8;
>>>>>>> origin/fixes_2.4

  {
   * The data for this flavor has not yet been added to the pasteboard
   * by the sender. This may have been done as a performance measure if
   * the data is expensive to generate. A request by the receiver to
   * obtain the data will cause the promise to be kept by the sender.
   * This flag can not be added by clients. It is automatically added
   * by the Pasteboard Manager when appropriate.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
	kPasteboardFlavorPromised = 1 shl 9;


{
 *  PasteboardStandardLocation
 *  
 *  Summary:
 *    Pasteboard Standard Drop Locations
 *  
 *  Discussion:
 *    The following constants define common "meta" paste locations.
 }
type
	PasteboardStandardLocation = OSType;
const
{
   * The paste or drop location was in the trash.  This is set when a
   * drag is dropped on the trash icon or a paste occurs within the
   * trash.  Setting this standard paste location sets the traditional
   * paste location to an alias to the trash folder automatically.
   }
	kPasteboardStandardLocationTrash = FourCharCode('trsh');

  {
   * The receiver did not specify a paste location. This is the default.
   }
	kPasteboardStandardLocationUnknown = FourCharCode('unkn');
<<<<<<< HEAD
=======
    kPasteboardFlavorPromised     = $00000200; {(1 << 9)}
>>>>>>> graemeg/fixes_2_2
=======
    kPasteboardFlavorPromised     = $00000200; {(1 << 9)}
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{
 *  PasteboardGetTypeID()
 *  
 *  Summary:
 *    Returns the CFType identifier for a pasteboard object.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Result:
 *    A CFTypeID unique to pasteboard instances.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function PasteboardGetTypeID: CFTypeID; external name '_PasteboardGetTypeID';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

=======
 
// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function PasteboardGetTypeID: CFTypeID; external name '_PasteboardGetTypeID';
>>>>>>> graemeg/fixes_2_2
=======
 
// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function PasteboardGetTypeID: CFTypeID; external name '_PasteboardGetTypeID';
>>>>>>> origin/fixes_2_2
=======
function PasteboardGetTypeID: CFTypeID; external name '_PasteboardGetTypeID';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

>>>>>>> origin/fixes_2.4

{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPasteboardClipboard CFSTRP('com.apple.pasteboard.clipboard')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPasteboardFind CFSTRP('com.apple.pasteboard.find')}
{$endc}

const
    kPasteboardUniqueName	= NIL;

{
 *  PasteboardCreate()
 *  
 *  Summary:
 *    Creates a local pasteboard reference to the global pasteboard
 *    resource of the same name.
 *  
 *  Discussion:
 *    If the the global pasteboard resource doesn't yet exist,
 *    PasteboardCreate creates a new one with the provided name.
 *    Pasteboard names are typically represented in a reverse DNS
 *    scheme (i.e., com.apple.pasteboard.clipboard). Multiple local
 *    pasteboard references may point to the same global pasteboard
 *    resource within an application. An application, window, plug-in,
 *    or object may each hold separate references. These should be held
 *    and used for the life time of the objects. Pasteboard references
 *    are CFTypes. Their memory should be released by calling
 *    CFRelease(). CFRelease() automatically resolves all promises made
 *    to the global pasteboard resource through the reference being
 *    released.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inName:
 *      The name of the pasteboard to reference or create. Passing
 *      kPasteboardUniqueName, or NULL, will guarantee that a new
 *      global pasteboard resource is created. kPasteboardClipboard is
 *      the traditional copy and paste pasteboard. kPasteboardFind is
 *      compatible with Cocoa's global find pasteboard.
 *    
 *    outPasteboard:
 *      The created pasteboard reference.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function PasteboardCreate( inName: CFStringRef { can be NULL }; var outPasteboard: PasteboardRef ): OSStatus; external name '_PasteboardCreate';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2

// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function PasteboardCreate(
  inName: CFStringRef;              { can be NULL }
  var outPasteboard: PasteboardRef): OSStatus; external name '_PasteboardCreate';
  
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function PasteboardCreate( inName: CFStringRef { can be NULL }; var outPasteboard: PasteboardRef ): OSStatus; external name '_PasteboardCreate';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  PasteboardSynchronize()
 *  
 *  Summary:
 *    Compares a local pasteboard reference with the global pasteboard
 *    resource to which it refers, determining whether the global
 *    pasteboard resource has been modified. If so, it updates the
 *    local pasteboard reference to reflect the change. The pasteboard
 *    reference provided is always brought up to date. This routine is
 *    lightweight whether a synchronization is required or not.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inPasteboard:
 *      A local pasteboard reference.
 *  
 *  Result:
 *    A set of pasteboard synchronization flags.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function PasteboardSynchronize( inPasteboard: PasteboardRef ): PasteboardSyncFlags; external name '_PasteboardSynchronize';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
=======

// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function PasteboardSynchronize(inPasteboard: PasteboardRef): PasteboardSyncFlags; external name '_PasteboardSynchronize';
>>>>>>> graemeg/fixes_2_2
=======

// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function PasteboardSynchronize(inPasteboard: PasteboardRef): PasteboardSyncFlags; external name '_PasteboardSynchronize';
>>>>>>> origin/fixes_2_2
=======
function PasteboardSynchronize( inPasteboard: PasteboardRef ): PasteboardSyncFlags; external name '_PasteboardSynchronize';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  PasteboardClear()
 *  
 *  Summary:
 *    Clears all data from the global pasteboard resource associated
 *    with the pasteboard reference provided. The application now owns
 *    the pasteboard and data may be placed onto the global pasteboard
 *    resource from all local pasteboards in the client application
 *    which reference it. PasteboardClear must be called before the
 *    pasteboard can be modified.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inPasteboard:
 *      A local pasteboard reference.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function PasteboardClear( inPasteboard: PasteboardRef ): OSStatus; external name '_PasteboardClear';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
=======

// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function PasteboardClear(inPasteboard: PasteboardRef): OSStatus; external name '_PasteboardClear';
>>>>>>> graemeg/fixes_2_2
=======

// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function PasteboardClear(inPasteboard: PasteboardRef): OSStatus; external name '_PasteboardClear';
>>>>>>> origin/fixes_2_2
=======
function PasteboardClear( inPasteboard: PasteboardRef ): OSStatus; external name '_PasteboardClear';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  PasteboardCopyName()
 *  
 *  Summary:
 *    Copies the name of the given pasteboard. Useful for discovering
 *    the name of a uniquely named pasteboard so other processes may
 *    access it.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inPasteboard:
 *      A local pasteboard reference.
 *    
 *    outName:
 *      On return, a CFString reference to the pasteboard's name. This
 *      string must be released by the client.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PasteboardCopyName( inPasteboard: PasteboardRef; var outName: CFStringRef ): OSStatus; external name '_PasteboardCopyName';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  PasteboardGetItemCount()
 *  
 *  Summary:
 *    Returns the number of items on the pasteboard.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inPasteboard:
 *      A local pasteboard reference.
 *    
 *    outItemCount:
 *      An ItemCount reference which receives number of pasteboard
 *      items.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function PasteboardGetItemCount( inPasteboard: PasteboardRef; var outItemCount: ItemCount ): OSStatus; external name '_PasteboardGetItemCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
=======
=======
>>>>>>> origin/fixes_2_2
 
// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function PasteboardGetItemCount(
  inPasteboard: PasteboardRef;
  var outItemCount: ItemCount): OSStatus; external name '_PasteboardGetItemCount';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function PasteboardGetItemCount( inPasteboard: PasteboardRef; var outItemCount: ItemCount ): OSStatus; external name '_PasteboardGetItemCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  PasteboardGetItemIdentifier()
 *  
 *  Summary:
 *    Returns the item identifier for the nth pasteboard item.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inPasteboard:
 *      A local pasteboard reference.
 *    
 *    inIndex:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      A 1-based CFIndex requesting the nth pasteboard item reference.
=======
 *      A 1-based UInt32 index requesting the nth pasteboard item reference.
>>>>>>> graemeg/fixes_2_2
=======
 *      A 1-based UInt32 index requesting the nth pasteboard item reference.
>>>>>>> origin/fixes_2_2
=======
 *      A 1-based CFIndex requesting the nth pasteboard item reference.
>>>>>>> origin/fixes_2.4
 *    
 *    outItem:
 *      A PasteboardItemID which receives the nth pasteboard item
 *      reference.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function PasteboardGetItemIdentifier( inPasteboard: PasteboardRef; inIndex: CFIndex; var outItem: PasteboardItemID ): OSStatus; external name '_PasteboardGetItemIdentifier';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 
// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function PasteboardGetItemIdentifier(
  inPasteboard: PasteboardRef;
  inIndex: UInt32;
  var outItem: PasteboardItemID): OSStatus; external name '_PasteboardGetItemIdentifier';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function PasteboardGetItemIdentifier( inPasteboard: PasteboardRef; inIndex: CFIndex; var outItem: PasteboardItemID ): OSStatus; external name '_PasteboardGetItemIdentifier';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  PasteboardCopyItemFlavors()
 *  
 *  Summary:
 *    Returns the array of flavors for the provided pasteboard
 *    reference.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inPasteboard:
 *      A local pasteboard reference.
 *    
 *    inItem:
 *      A pasteboard item identifier containing the flavors of interest.
 *    
 *    outFlavorTypes:
 *      A CFArrayRef reference which receives the array of Uniform Type
 *      Identifier based flavor types.  It is the client's
 *      responsibility to release the flavor array via CFRelease().
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function PasteboardCopyItemFlavors( inPasteboard: PasteboardRef; inItem: PasteboardItemID; var outFlavorTypes: CFArrayRef ): OSStatus; external name '_PasteboardCopyItemFlavors';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
=======
=======
>>>>>>> origin/fixes_2_2

// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function PasteboardCopyItemFlavors(
  inPasteboard: PasteboardRef;
  inItem: PasteboardItemID;
  var outFlavorTypes: CFArrayRef): OSStatus; external name '_PasteboardCopyItemFlavors';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function PasteboardCopyItemFlavors( inPasteboard: PasteboardRef; inItem: PasteboardItemID; var outFlavorTypes: CFArrayRef ): OSStatus; external name '_PasteboardCopyItemFlavors';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  PasteboardGetItemFlavorFlags()
 *  
 *  Summary:
 *    Returns the array of flags for the provided flavor, including
 *    implicit translations included by the system automatically.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inPasteboard:
 *      A local pasteboard reference.
 *    
 *    inItem:
 *      A pasteboard item identifier containing the flavor of interest.
 *    
 *    inFlavorType:
 *      A Uniform Type Identifier based flavor type whose flags are
 *      being interrogated.
 *    
 *    outFlags:
 *      A PasteboardFlavorFlags reference which receives the flavor
 *      flags.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 }
function PasteboardGetItemFlavorFlags( inPasteboard: PasteboardRef; inItem: PasteboardItemID; inFlavorType: CFStringRef; var outFlags: PasteboardFlavorFlags ): OSStatus; external name '_PasteboardGetItemFlavorFlags';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
=======
=======
>>>>>>> origin/fixes_2_2
}

// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function PasteboardGetItemFlavorFlags(
  inPasteboard: PasteboardRef;
  inItem: PasteboardItemID;
  inFlavorType: CFStringRef;
  var outFlags: PasteboardFlavorFlags): OSStatus; external name '_PasteboardGetItemFlavorFlags';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 }
function PasteboardGetItemFlavorFlags( inPasteboard: PasteboardRef; inItem: PasteboardItemID; inFlavorType: CFStringRef; var outFlags: PasteboardFlavorFlags ): OSStatus; external name '_PasteboardGetItemFlavorFlags';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  PasteboardCopyItemFlavorData()
 *  
 *  Summary:
 *    Returns the data for the provided flavor.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inPasteboard:
 *      A local pasteboard reference.
 *    
 *    inItem:
 *      A pasteboard item identifier containing the flavor of interest.
 *    
 *    inFlavorType:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      A Uniform Type Identifier-based flavor type whose data is being
 *      retrieved.
=======
 *      A Uniform Type Idendtifier based flavor type whose data is
 *      being retrieved.
>>>>>>> graemeg/fixes_2_2
=======
 *      A Uniform Type Idendtifier based flavor type whose data is
 *      being retrieved.
>>>>>>> origin/fixes_2_2
=======
 *      A Uniform Type Identifier-based flavor type whose data is being
 *      retrieved.
>>>>>>> origin/fixes_2.4
 *    
 *    outData:
 *      A CFDataRef reference which receives the flavor data. It is the
 *      client's responsibility to release the data via CFRelease().
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function PasteboardCopyItemFlavorData( inPasteboard: PasteboardRef; inItem: PasteboardItemID; inFlavorType: CFStringRef; var outData: CFDataRef ): OSStatus; external name '_PasteboardCopyItemFlavorData';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2

// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function PasteboardCopyItemFlavorData(
  inPasteboard: PasteboardRef;
  inItem: PasteboardItemID;
  inFlavorType: CFStringRef;
  var outData: CFDataRef): OSStatus; external name '_PasteboardCopyItemFlavorData';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function PasteboardCopyItemFlavorData( inPasteboard: PasteboardRef; inItem: PasteboardItemID; inFlavorType: CFStringRef; var outData: CFDataRef ): OSStatus; external name '_PasteboardCopyItemFlavorData';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

>>>>>>> origin/fixes_2.4

const
    kPasteboardPromisedData = NIL;
	
{
 *  PasteboardPutItemFlavor()
 *  
 *  Summary:
 *    Adds flavor data or a promise to the global pasteboard resource.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inPasteboard:
 *      A local pasteboard reference.
 *    
 *    inItem:
 *      A pasteboard item identifier in which to add the data or
 *      promise. Pasteboard item identifiers are unique values created
 *      by the owning application when adding pasteboard item flavor
 *      information to a drag. Additional flavors may be added to an
 *      existing item by using the same item identifier value.
 *      Depending on the application, it might be easier to use
 *      internal memory addresses as item identifiers (as long as each
 *      item being added has a unique item reference number).
 *      Pasteboard item identifiers should only be interpreted by the
 *      owning application.
 *    
 *    inFlavorType:
 *      A Uniform Type Identifier based flavor type associated with the
 *      data. If multiple flavors are to be added to an item, the
 *      owning application should add them in order of preference or
 *      richness as determined by the owing application.  The ordering
 *      will be retained when viewed by the receiving application.
 *    
 *    inData:
 *      A CFDataRef reference which receives the flavor data. Passing
 *      kPasteboardPromisedData, or NULL, indicates the data is
 *      promised. This is useful if the data is expensive to generate.
 *      Making promises requires the sender to also implement a promise
 *      keeper which must be set before the promise is made.
 *    
 *    inFlags:
 *      A PasteboardFlavorFlags set of flags to attach to the data.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 }
function PasteboardPutItemFlavor( inPasteboard: PasteboardRef; inItem: PasteboardItemID; inFlavorType: CFStringRef; inData: CFDataRef { can be NULL }; inFlags: PasteboardFlavorFlags ): OSStatus; external name '_PasteboardPutItemFlavor';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
=======
=======
>>>>>>> origin/fixes_2_2
}

// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function PasteboardPutItemFlavor(
  inPasteboard: PasteboardRef;
  inItem: PasteboardItemID;
  inFlavorType: CFStringRef;
  inData: CFDataRef;             { can be NULL }
  inFlags: PasteboardFlavorFlags): OSStatus; external name '_PasteboardPutItemFlavor';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 }
function PasteboardPutItemFlavor( inPasteboard: PasteboardRef; inItem: PasteboardItemID; inFlavorType: CFStringRef; inData: CFDataRef { can be NULL }; inFlags: PasteboardFlavorFlags ): OSStatus; external name '_PasteboardPutItemFlavor';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  PasteboardCopyPasteLocation()
 *  
 *  Summary:
 *    Called by the owner of a pasteboard while providing promised data
 *    to determine the paste location set by the pasteboard receiver.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inPasteboard:
 *      A local pasteboard reference.
 *    
 *    outPasteLocation:
 *      A CFURL reference describing the paste location. It is the
 *      client's responsibility to release the data via CFRelease().
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function PasteboardCopyPasteLocation( inPasteboard: PasteboardRef; var outPasteLocation: CFURLRef ): OSStatus; external name '_PasteboardCopyPasteLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
=======
=======
>>>>>>> origin/fixes_2_2
 
// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function PasteboardCopyPasteLocation(
  inPasteboard: PasteboardRef;
  var outPasteLocation: CFURLRef): OSStatus; external name '_PasteboardCopyPasteLocation';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function PasteboardCopyPasteLocation( inPasteboard: PasteboardRef; var outPasteLocation: CFURLRef ): OSStatus; external name '_PasteboardCopyPasteLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  PasteboardSetPasteLocation()
 *  
 *  Summary:
 *    Called by the receiver of a pasteboard before requesting any item
 *    flavor data via PasteboardCopyItemFlavorData.  When a sending
 *    application's pasteboard promise keeper is called to provide data
 *    to the receiver, PasteboardGetDropLocation can be called to
 *    determine the paste location while providing data.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inPasteboard:
 *      A local pasteboard reference.
 *    
 *    inPasteLocation:
 *      A CFURL describing the paste location.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function PasteboardSetPasteLocation( inPasteboard: PasteboardRef; inPasteLocation: CFURLRef ): OSStatus; external name '_PasteboardSetPasteLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
=======
=======
>>>>>>> origin/fixes_2_2
 
// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function PasteboardSetPasteLocation(
  inPasteboard: PasteboardRef;
  inPasteLocation: CFURLRef): OSStatus; external name '_PasteboardSetPasteLocation';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function PasteboardSetPasteLocation( inPasteboard: PasteboardRef; inPasteLocation: CFURLRef ): OSStatus; external name '_PasteboardSetPasteLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  PasteboardPromiseKeeperProcPtr
 *  
 *  Summary:
 *    Callback for providing data previously promised on the pasteboard.
 *  
 *  Parameters:
 *    
 *    pasteboard:
 *      The local pasteboard reference on which the promise was made.
 *    
 *    item:
 *      The pasteboard item identifier containing the promised flavor.
 *    
 *    flavorType:
 *      The Uniform Type Identifier based flavor type for which the
 *      promised data is being requested.
 *    
 *    context:
 *      The value passed as the context in PasteboardSetPromiseKeeper().
 *  
 *  Result:
 *    An operating system result code.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 }
type
	PasteboardPromiseKeeperProcPtr = function( pasteboard: PasteboardRef; item: PasteboardItemID; flavorType: CFStringRef; context: UnivPtr ): OSStatus;
=======
=======
>>>>>>> origin/fixes_2_2
}

type
{$ifc TYPED_FUNCTION_POINTERS}
	PasteboardPromiseKeeperProcPtr = function(pasteboard: PasteboardRef; item: PasteboardItemID; flavorType: CFStringRef; context: Ptr): OSStatus;
{$elsec}
	PasteboardPromiseKeeperProcPtr = ProcPtr;
{$endc}

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 }
type
	PasteboardPromiseKeeperProcPtr = function( pasteboard: PasteboardRef; item: PasteboardItemID; flavorType: CFStringRef; context: UnivPtr ): OSStatus;
>>>>>>> origin/fixes_2.4
{
 *  PasteboardSetPromiseKeeper()
 *  
 *  Summary:
 *    Associates a promise keeper callback with a local pasteboard
 *    reference. The promise keeper must be set before a promise is
 *    made.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inPasteboard:
 *      The local pasteboard reference on which promises will be made.
 *    
 *    inPromiseKeeper:
 *      A PasteboardPromiseKeeperProcPtr promise keeper proc.
 *    
 *    inContext:
 *      The value passed in this parameter is passed on to your promise
 *      keeper proc when it is called.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function PasteboardSetPromiseKeeper( inPasteboard: PasteboardRef; inPromiseKeeper: PasteboardPromiseKeeperProcPtr; inContext: UnivPtr ): OSStatus; external name '_PasteboardSetPromiseKeeper';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2

// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function PasteboardSetPromiseKeeper(
  inPasteboard: PasteboardRef;
  inPromiseKeeper: PasteboardPromiseKeeperProcPtr;
  inContext: Ptr): OSStatus; external name '_PasteboardSetPromiseKeeper';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function PasteboardSetPromiseKeeper( inPasteboard: PasteboardRef; inPromiseKeeper: PasteboardPromiseKeeperProcPtr; inContext: UnivPtr ): OSStatus; external name '_PasteboardSetPromiseKeeper';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

>>>>>>> origin/fixes_2.4

const
    kPasteboardResolveAllPromises = NIL;
	
{
 *  PasteboardResolvePromises()
 *  
 *  Summary:
 *    Resolves promises on the provided local pasteboard reference. If
 *    multiple local pasteboard references to the same global
 *    pasteboard resource exist, only those promises made through the
 *    provided reference are resolved.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inPasteboard:
 *      The local pasteboard reference for which promises will be
 *      resolved. Passing kPasteboardResolveAllPromises, or NULL, will
 *      cause all promises on all global pasteboard resources currently
 *      owned by this application to be resolved.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function PasteboardResolvePromises( inPasteboard: PasteboardRef ): OSStatus; external name '_PasteboardResolvePromises';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
 
// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER;
function PasteboardResolvePromises(inPasteboard: PasteboardRef): OSStatus; external name '_PasteboardResolvePromises';

{$ALIGN MAC68K}
=======
function PasteboardResolvePromises( inPasteboard: PasteboardRef ): OSStatus; external name '_PasteboardResolvePromises';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/fixes_2.4

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
