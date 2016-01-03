{
<<<<<<< HEAD
<<<<<<< HEAD
     File:       UTCoreTypes.h
 
     Contains:   String constants for core uniform type identifiers
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Copyright:  (c) 2004-2012 by Apple Inc. All rights reserved.
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
=======
     Version:    LaunchServices-360.3~1
=======
     Version:    LaunchServices-360.3~1
=======
     Version:    LaunchServices-360.3~1
=======
     Version:    LaunchServices-360.3~1
 
     Copyright:  © 2004-2008 by Apple Computer, Inc., all rights reserved.
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
                     http://www.freepascal.org/bugs.html
>>>>>>> origin/cpstrnew
 
     Copyright:  © 2004-2008 by Apple Computer, Inc., all rights reserved.
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
                     http://www.freepascal.org/bugs.html
>>>>>>> graemeg/cpstrnew
 
     Copyright:  © 2004-2008 by Apple Computer, Inc., all rights reserved.
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
                     http://www.freepascal.org/bugs.html
>>>>>>> graemeg/cpstrnew
 
     Copyright:  © 2004-2008 by Apple Computer, Inc., all rights reserved.
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
                     http://www.freepascal.org/bugs.html
>>>>>>> graemeg/cpstrnew
 
                     http://bugs.freepascal.org
}

<<<<<<< HEAD
=======
{	 Pascal Translation:  Gale R Paeper, <gpaeper@empirenet.com>, 2006 }
{	 Pascal Translation Update: Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }

>>>>>>> graemeg/cpstrnew
{
    Modified for use with Free Pascal
    Version 308
    Please report any bugs to <gpc@microbizz.nl>
}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
     File:       LaunchServices/UTCoreTypes.h
 
     Contains:   String constants for core uniform type identifiers
 
     Version:    LaunchServices-360.3~1
 
     Copyright:  © 2004-2008 by Apple Computer, Inc., all rights reserved.
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
                     http://www.freepascal.org/bugs.html
 
}

{	 Pascal Translation:  Gale R Paeper, <gpaeper@empirenet.com>, 2006 }
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

unit UTCoreTypes;
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
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
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
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
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
<<<<<<< HEAD
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
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
=======
=======
>>>>>>> origin/cpstrnew
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
=======
{$elsec}
>>>>>>> graemeg/cpstrnew
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
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
uses MacTypes,CFBase;
{$endc} {not MACOSALLINCLUDE}


<<<<<<< HEAD
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
=======
uses CFBase;
>>>>>>> graemeg/fixes_2_2
=======
uses CFBase;
>>>>>>> origin/fixes_2_2
=======
{ header available as of iPhoneOS 3.0, but can't add iPhoneOS markers yet
  because they're not yet present in the Mac OS X version of the header }

>>>>>>> origin/fixes_2.4
{$ALIGN POWER}


{ ================================================================================ *
 *  Abstract base types                                                             *
 * ================================================================================ }
{
 *  kUTTypeItem
 *
 *    generic base type for most things
 *    (files, directories)
 *
 *    UTI: public.item
 *
 *
 *  kUTTypeContent
 *
 *    base type for anything containing user-viewable document content
 *    (documents, pasteboard data, and document packages)
 *
 *    UTI: public.content
 *
 *
 *  kUTTypeCompositeContent
 *
 *    base type for content formats supporting mixed embedded content
 *    (i.e., compound documents)
 *
 *    UTI: public.composite-content
 *    conforms to: public.content
 *
 *
 *  kUTTypeApplication
 *
<<<<<<< HEAD
<<<<<<< HEAD
 *    base type for OS X applications, launchable items
=======
 *    base type for Mac OS X applications, launchable items
>>>>>>> graemeg/fixes_2_2
=======
 *    base type for Mac OS X applications, launchable items
>>>>>>> origin/fixes_2_2
 *
 *    UTI: com.apple.application
 *
 *
 *  kUTTypeMessage
 *
 *    base type for messages (email, IM, etc.)
 *
 *    UTI: public.message
 *
 *
 *  kUTTypeContact
 *
 *    contact information, e.g. for a person, group, organization
 *
 *    UTI: public.contact
 *
 *
 *  kUTTypeArchive
 *
 *    an archive of files and directories
 *
 *    UTI: public.archive
 *
 *
 *  kUTTypeDiskImage
 *
 *    a data item mountable as a volume
 *
 *    UTI: public.disk-image
 }
<<<<<<< HEAD
<<<<<<< HEAD
var kUTTypeItem: CFStringRef; external name '_kUTTypeItem'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeContent: CFStringRef; external name '_kUTTypeContent'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeCompositeContent: CFStringRef; external name '_kUTTypeCompositeContent'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeApplication: CFStringRef; external name '_kUTTypeApplication'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeMessage: CFStringRef; external name '_kUTTypeMessage'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeContact: CFStringRef; external name '_kUTTypeContact'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeArchive: CFStringRef; external name '_kUTTypeArchive'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeDiskImage: CFStringRef; external name '_kUTTypeDiskImage'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)


=======
=======
>>>>>>> origin/fixes_2_2
{
 *  kUTTypeItem
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeItem: CFStringRef; external name '_kUTTypeItem'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeContent
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeContent: CFStringRef; external name '_kUTTypeContent'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeCompositeContent
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeCompositeContent: CFStringRef; external name '_kUTTypeCompositeContent'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeApplication
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeApplication: CFStringRef; external name '_kUTTypeApplication'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeMessage
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeMessage: CFStringRef; external name '_kUTTypeMessage'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeContact
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeContact: CFStringRef; external name '_kUTTypeContact'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeArchive
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeArchive: CFStringRef; external name '_kUTTypeArchive'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeDiskImage
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeDiskImage: CFStringRef; external name '_kUTTypeDiskImage'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{ ================================================================================ *
 *  Concrete base types                                                             *
 * ================================================================================ }
{
 *  kUTTypeData
 *
 *    base type for any sort of simple byte stream,
 *    including files and in-memory data
 *
 *    UTI: public.data
 *    conforms to: public.item
 *
 *
 *  kUTTypeDirectory
 *
 *    file system directory 
 *    (includes packages AND folders)
 *
 *    UTI: public.directory
 *    conforms to: public.item
 *
 *
 *  kUTTypeResolvable
 *
<<<<<<< HEAD
<<<<<<< HEAD
 *    symlink and alias file types conform to this UTI
=======
 *    anything the Alias Manager can resolve
>>>>>>> graemeg/fixes_2_2
=======
 *    anything the Alias Manager can resolve
>>>>>>> origin/fixes_2_2
 *
 *    UTI: com.apple.resolvable
 *
 *
 *  kUTTypeSymLink
 *
 *    a symbolic link
 *
 *    UTI: public.symlink
 *    conforms to: public.item, com.apple.resolvable
 *
 *
 *  kUTTypeMountPoint
 *
 *    a volume mount point (resolvable, resolves to the root dir of a volume)
 *
 *    UTI: com.apple.mount-point
 *    conforms to: public.item, com.apple.resolvable
 *
 *
 *  kUTTypeAliasFile
 *
 *    a fully-formed alias file
 *
 *    UTI: com.apple.alias-file
 *    conforms to: public.data, com.apple.resolvable
 *
 *
 *  kUTTypeAliasRecord
 *
 *    raw alias data
 *
 *    UTI: com.apple.alias-record
 *    conforms to: public.data, com.apple.resolvable
 *
 *
 *  kUTTypeURL
 *
 *    The bytes of a URL
 *    (OSType 'url ')
 *
 *    UTI: public.url
 *    conforms to: public.data
 *
 *
 *  kUTTypeFileURL
 *
 *    The text of a "file:" URL 
 *    (OSType 'furl')
 *
 *    UTI: public.file-url
 *    conforms to: public.url
 }
<<<<<<< HEAD
<<<<<<< HEAD
var kUTTypeData: CFStringRef; external name '_kUTTypeData'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeDirectory: CFStringRef; external name '_kUTTypeDirectory'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeResolvable: CFStringRef; external name '_kUTTypeResolvable'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeSymLink: CFStringRef; external name '_kUTTypeSymLink'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeMountPoint: CFStringRef; external name '_kUTTypeMountPoint'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeAliasFile: CFStringRef; external name '_kUTTypeAliasFile'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeAliasRecord: CFStringRef; external name '_kUTTypeAliasRecord'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeURL: CFStringRef; external name '_kUTTypeURL'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeFileURL: CFStringRef; external name '_kUTTypeFileURL'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)


=======
=======
>>>>>>> origin/fixes_2_2
{
 *  kUTTypeData
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeData: CFStringRef; external name '_kUTTypeData'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeDirectory
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeDirectory: CFStringRef; external name '_kUTTypeDirectory'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeResolvable
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeResolvable: CFStringRef; external name '_kUTTypeResolvable'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeSymLink
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeSymLink: CFStringRef; external name '_kUTTypeSymLink'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeMountPoint
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeMountPoint: CFStringRef; external name '_kUTTypeMountPoint'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeAliasFile
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeAliasFile: CFStringRef; external name '_kUTTypeAliasFile'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeAliasRecord
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeAliasRecord: CFStringRef; external name '_kUTTypeAliasRecord'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeURL
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeURL: CFStringRef; external name '_kUTTypeURL'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeFileURL
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeFileURL: CFStringRef; external name '_kUTTypeFileURL'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{ ================================================================================ *
 *  Text types                                                                      *
 * ================================================================================ }
{
 *  kUTTypeText
 *
 *    base type for all text-encoded data, 
 *    including text with markup (HTML, RTF, etc.)
 *
 *    UTI: public.text
 *    conforms to: public.data, public.content
 *
 *
 *  kUTTypePlainText
 *
 *    text with no markup, unspecified encoding
 *
 *    UTI: public.plain-text
 *    conforms to: public.text
 *
 *
 *  kUTTypeUTF8PlainText
 *
 *    plain text, UTF-8 encoding
 *    (OSType 'utf8', NSPasteboardType "NSStringPBoardType")
 *
 *    UTI: public.utf8-plain-text
 *    conforms to: public.plain-text
 *
 *
 *  kUTTypeUTF16ExternalPlainText
 *
 *    plain text, UTF-16 encoding, with BOM, or if BOM 
 *    is not present, has "external representation" 
 *    byte order (big-endian).
 *    (OSType 'ut16')
 *
 *    UTI: public.utf16-external-plain-text
 *    conforms to: public.plain-text
 *
 *
 *  kUTTypeUTF16PlainText
 *
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    plain text, UTF-16 encoding, native byte order, optional BOM
=======
 *    plain text, UTF-16 encoding, native byte order, no BOM
>>>>>>> graemeg/fixes_2_2
=======
 *    plain text, UTF-16 encoding, native byte order, no BOM
>>>>>>> origin/fixes_2_2
=======
 *    plain text, UTF-16 encoding, native byte order, optional BOM
>>>>>>> origin/fixes_2.4
 *    (OSType 'utxt')
 *
 *    UTI: public.utf16-plain-text
 *    conforms to: public.plain-text
 *
 *
 *  kUTTypeRTF
 *
 *    Rich Text Format
 *
 *    UTI: public.rtf
 *    conforms to: public.text
 *
 *
 *  kUTTypeHTML
 *
 *    HTML, any version
 *
 *    UTI: public.html
 *    conforms to: public.text
 *
 *
 *  kUTTypeXML
 *
 *    generic XML
 *
 *    UTI: public.xml
 *    conforms to: public.text
 *
 *
 *  kUTTypeSourceCode
 *
 *    abstract type for source code (any language)
 *
 *    UTI: public.source-code
 *    conforms to: public.plain-text
 *
 *
 *  kUTTypeCSource
 *
 *    C source code (.c)
 *
 *    UTI: public.c-source
 *    conforms to: public.plain-text
 *
 *
 *  kUTTypeObjectiveCSource
 *
 *    Objective-C source code (.m)
 *
 *    UTI: public.objective-c-source
 *    conforms to: public.plain-text
 *
 *
 *  kUTTypeCPlusPlusSource
 *
 *    C++ source code (.cp, etc.)
 *
 *    UTI: public.c-plus-plus-source
 *    conforms to: public.plain-text
 *
 *
 *  kUTTypeObjectiveCPlusPlusSource
 *
 *    Objective-C++ source code
 *
 *    UTI: public.objective-c-plus-plus-source
 *    conforms to: public.plain-text
 *
 *
 *  kUTTypeCHeader
 *
 *    C header
 *
 *    UTI: public.c-header
 *    conforms to: public.plain-text
 *
 *
 *  kUTTypeCPlusPlusHeader
 *
 *    C++ header
 *
 *    UTI: public.c-plus-plus-header
 *    conforms to: public.plain-text
 *
 *
 *  kUTTypeJavaSource
 *
 *    Java source code
 *
 *    UTI: com.sun.java-source
 *    conforms to: public.plain-text
 }
<<<<<<< HEAD
<<<<<<< HEAD
var kUTTypeText: CFStringRef; external name '_kUTTypeText'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypePlainText: CFStringRef; external name '_kUTTypePlainText'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeUTF8PlainText: CFStringRef; external name '_kUTTypeUTF8PlainText'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeUTF16ExternalPlainText: CFStringRef; external name '_kUTTypeUTF16ExternalPlainText'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeUTF16PlainText: CFStringRef; external name '_kUTTypeUTF16PlainText'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeRTF: CFStringRef; external name '_kUTTypeRTF'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeHTML: CFStringRef; external name '_kUTTypeHTML'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeXML: CFStringRef; external name '_kUTTypeXML'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeSourceCode: CFStringRef; external name '_kUTTypeSourceCode'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeCSource: CFStringRef; external name '_kUTTypeCSource'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeObjectiveCSource: CFStringRef; external name '_kUTTypeObjectiveCSource'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeCPlusPlusSource: CFStringRef; external name '_kUTTypeCPlusPlusSource'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeObjectiveCPlusPlusSource: CFStringRef; external name '_kUTTypeObjectiveCPlusPlusSource'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeCHeader: CFStringRef; external name '_kUTTypeCHeader'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeCPlusPlusHeader: CFStringRef; external name '_kUTTypeCPlusPlusHeader'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeJavaSource: CFStringRef; external name '_kUTTypeJavaSource'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)

=======
=======
>>>>>>> origin/fixes_2_2
{
 *  kUTTypeText
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeText: CFStringRef; external name '_kUTTypeText'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypePlainText
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypePlainText: CFStringRef; external name '_kUTTypePlainText'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeUTF8PlainText
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeUTF8PlainText: CFStringRef; external name '_kUTTypeUTF8PlainText'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeUTF16ExternalPlainText
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeUTF16ExternalPlainText: CFStringRef; external name '_kUTTypeUTF16ExternalPlainText'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeUTF16PlainText
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeUTF16PlainText: CFStringRef; external name '_kUTTypeUTF16PlainText'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeRTF
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeRTF: CFStringRef; external name '_kUTTypeRTF'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeHTML
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeHTML: CFStringRef; external name '_kUTTypeHTML'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeXML
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeXML: CFStringRef; external name '_kUTTypeXML'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeSourceCode
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeSourceCode: CFStringRef; external name '_kUTTypeSourceCode'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeCSource
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeCSource: CFStringRef; external name '_kUTTypeCSource'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeObjectiveCSource
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeObjectiveCSource: CFStringRef; external name '_kUTTypeObjectiveCSource'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeCPlusPlusSource
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeCPlusPlusSource: CFStringRef; external name '_kUTTypeCPlusPlusSource'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeObjectiveCPlusPlusSource
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeObjectiveCPlusPlusSource: CFStringRef; external name '_kUTTypeObjectiveCPlusPlusSource'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeCHeader
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeCHeader: CFStringRef; external name '_kUTTypeCHeader'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeCPlusPlusHeader
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeCPlusPlusHeader: CFStringRef; external name '_kUTTypeCPlusPlusHeader'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeJavaSource
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeJavaSource: CFStringRef; external name '_kUTTypeJavaSource'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{ ================================================================================ *
 *  Composite content types                                                         *
 * ================================================================================ }
{
 *  kUTTypePDF
 *
 *    Adobe PDF
 *
 *    UTI: com.adobe.pdf
 *    conforms to: public.data, public.composite-content
 *
 *
 *  kUTTypeRTFD
 *
 *    Rich Text Format Directory 
 *    (RTF with content embedding, on-disk format)
 *
 *    UTI: com.apple.rtfd
 *    conforms to: com.apple.package, public.composite-content
 *
 *
 *  kUTTypeFlatRTFD
 *
 *    Flattened RTFD (pasteboard format)
 *
 *    UTI: com.apple.flat-rtfd
 *    conforms to: public.data, public.composite-content
 *
 *
 *  kUTTypeTXNTextAndMultimediaData
 *
 *    MLTE (Textension) format for mixed text & multimedia data
 *    (OSType 'txtn')
 *
 *    UTI: com.apple.txn.text-multimedia-data
 *    conforms to: public.data, public.composite-content
 *
 *
 *  kUTTypeWebArchive
 *
 *    The WebKit webarchive format
 *
 *    UTI: com.apple.webarchive
 *    conforms to: public.data, public.composite-content
 }
<<<<<<< HEAD
<<<<<<< HEAD
var kUTTypePDF: CFStringRef; external name '_kUTTypePDF'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeRTFD: CFStringRef; external name '_kUTTypeRTFD'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeFlatRTFD: CFStringRef; external name '_kUTTypeFlatRTFD'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeTXNTextAndMultimediaData: CFStringRef; external name '_kUTTypeTXNTextAndMultimediaData'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeWebArchive: CFStringRef; external name '_kUTTypeWebArchive'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)

=======
=======
>>>>>>> origin/fixes_2_2
{
 *  kUTTypePDF
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypePDF: CFStringRef; external name '_kUTTypePDF'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeRTFD
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeRTFD: CFStringRef; external name '_kUTTypeRTFD'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeFlatRTFD
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeFlatRTFD: CFStringRef; external name '_kUTTypeFlatRTFD'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeTXNTextAndMultimediaData
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeTXNTextAndMultimediaData: CFStringRef; external name '_kUTTypeTXNTextAndMultimediaData'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeWebArchive
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeWebArchive: CFStringRef; external name '_kUTTypeWebArchive'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{ ================================================================================ *
 *  Image content types                                                             *
 * ================================================================================ }
{
 *  kUTTypeImage
 *
 *    abstract image data
 *
 *    UTI: public.image
 *    conforms to: public.data, public.content
 *
 *
 *  kUTTypeJPEG
 *
 *    JPEG image
 *
 *    UTI: public.jpeg
 *    conforms to: public.image
 *
 *
 *  kUTTypeJPEG2000
 *
 *    JPEG-2000 image
 *
 *    UTI: public.jpeg-2000
 *    conforms to: public.image
 *
 *
 *  kUTTypeTIFF
 *
 *    TIFF image
 *
 *    UTI: public.tiff
 *    conforms to: public.image
 *
 *
 *  kUTTypePICT
 *
 *    Quickdraw PICT format
 *
 *    UTI: com.apple.pict
 *    conforms to: public.image
 *
 *
 *  kUTTypeGIF
 *
 *    GIF image
 *
 *    UTI: com.compuserve.gif
 *    conforms to: public.image
 *
 *
 *  kUTTypePNG
 *
 *    PNG image
 *
 *    UTI: public.png
 *    conforms to: public.image
 *
 *
 *  kUTTypeQuickTimeImage
 *
 *    QuickTime image format (OSType 'qtif')
 *
 *    UTI: com.apple.quicktime-image
 *    conforms to: public.image
 *
 *
 *  kUTTypeAppleICNS
 *
 *    Apple icon data
 *
 *    UTI: com.apple.icns
 *    conforms to: public.image
 *
 *
 *  kUTTypeBMP
 *
 *    Windows bitmap
 *
 *    UTI: com.microsoft.bmp
 *    conforms to: public.image
 *
 *
 *  kUTTypeICO
 *
 *    Windows icon data
 *
 *    UTI: com.microsoft.ico
 *    conforms to: public.image
 }
<<<<<<< HEAD
<<<<<<< HEAD
var kUTTypeImage: CFStringRef; external name '_kUTTypeImage'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeJPEG: CFStringRef; external name '_kUTTypeJPEG'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeJPEG2000: CFStringRef; external name '_kUTTypeJPEG2000'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeTIFF: CFStringRef; external name '_kUTTypeTIFF'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypePICT: CFStringRef; external name '_kUTTypePICT'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeGIF: CFStringRef; external name '_kUTTypeGIF'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypePNG: CFStringRef; external name '_kUTTypePNG'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeQuickTimeImage: CFStringRef; external name '_kUTTypeQuickTimeImage'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeAppleICNS: CFStringRef; external name '_kUTTypeAppleICNS'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeBMP: CFStringRef; external name '_kUTTypeBMP'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeICO: CFStringRef; external name '_kUTTypeICO'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)

=======
=======
>>>>>>> origin/fixes_2_2
{
 *  kUTTypeImage
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeImage: CFStringRef; external name '_kUTTypeImage'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeJPEG
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeJPEG: CFStringRef; external name '_kUTTypeJPEG'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeJPEG2000
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeJPEG2000: CFStringRef; external name '_kUTTypeJPEG2000'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeTIFF
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeTIFF: CFStringRef; external name '_kUTTypeTIFF'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypePICT
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypePICT: CFStringRef; external name '_kUTTypePICT'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeGIF
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeGIF: CFStringRef; external name '_kUTTypeGIF'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypePNG
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypePNG: CFStringRef; external name '_kUTTypePNG'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeQuickTimeImage
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeQuickTimeImage: CFStringRef; external name '_kUTTypeQuickTimeImage'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeAppleICNS
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeAppleICNS: CFStringRef; external name '_kUTTypeAppleICNS'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeBMP
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeBMP: CFStringRef; external name '_kUTTypeBMP'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeICO
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeICO: CFStringRef; external name '_kUTTypeICO'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{ ================================================================================ *
 *  Audiovisual content types                                                       *
 * ================================================================================ }
{
 *  kUTTypeAudiovisualContent
 *
 *    audio and/or video content
 *
 *    UTI: public.audiovisual-content
 *    conforms to: public.data, public.content
 *
 *
 *  kUTTypeMovie
 *
 *    A media format which may contain both video and audio
 *    Corresponds to what users would label a "movie"
 *
 *    UTI: public.movie
 *    conforms to: public.audiovisual-content
 *
 *
 *  kUTTypeVideo
 *
 *    pure video (no audio)
 *
 *    UTI: public.video
 *    conforms to: public.movie
 *
 *
 *  kUTTypeAudio
 *
 *    pure audio (no video)
 *
 *    UTI: public.audio
 *    conforms to: public.audiovisual-content
 *
 *
 *  kUTTypeQuickTimeMovie
 *
 *    QuickTime movie
 *
 *    UTI: com.apple.quicktime-movie
 *    conforms to: public.movie
 *
 *
 *  kUTTypeMPEG
 *
 *    MPEG-1 or MPEG-2 movie
 *
 *    UTI: public.mpeg
 *    conforms to: public.movie
 *
 *
 *  kUTTypeMPEG4
 *
 *    MPEG-4 movie
 *
 *    UTI: public.mpeg-4
 *    conforms to: public.movie
 *
 *
 *  kUTTypeMP3
 *
 *    MP3 audio
 *
 *    UTI: public.mp3
 *    conforms to: public.audio
 *
 *
 *  kUTTypeMPEG4Audio
 *
 *    MPEG-4 audio layer
 *    (.m4a)
 *
 *    UTI: public.mpeg-4-audio
 *    conforms to: public.mpeg-4, public.audio
 *
 *
 *  kUTTypeAppleProtectedMPEG4Audio
 *
 *    Apple protected MPEG4 format
 *    (.m4p, iTunes music store format)
 *
 *    UTI: com.apple.protected-mpeg-4-audio
 *    conforms to: public.audio
 }
<<<<<<< HEAD
<<<<<<< HEAD
var kUTTypeAudiovisualContent: CFStringRef; external name '_kUTTypeAudiovisualContent'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeMovie: CFStringRef; external name '_kUTTypeMovie'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeVideo: CFStringRef; external name '_kUTTypeVideo'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeAudio: CFStringRef; external name '_kUTTypeAudio'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeQuickTimeMovie: CFStringRef; external name '_kUTTypeQuickTimeMovie'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeMPEG: CFStringRef; external name '_kUTTypeMPEG'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeMPEG4: CFStringRef; external name '_kUTTypeMPEG4'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeMP3: CFStringRef; external name '_kUTTypeMP3'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeMPEG4Audio: CFStringRef; external name '_kUTTypeMPEG4Audio'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeAppleProtectedMPEG4Audio: CFStringRef; external name '_kUTTypeAppleProtectedMPEG4Audio'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)


=======
=======
>>>>>>> origin/fixes_2_2
{
 *  kUTTypeAudiovisualContent
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeAudiovisualContent: CFStringRef; external name '_kUTTypeAudiovisualContent'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeMovie
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeMovie: CFStringRef; external name '_kUTTypeMovie'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeVideo
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeVideo: CFStringRef; external name '_kUTTypeVideo'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeAudio
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeAudio: CFStringRef; external name '_kUTTypeAudio'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeQuickTimeMovie
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeQuickTimeMovie: CFStringRef; external name '_kUTTypeQuickTimeMovie'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeMPEG
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeMPEG: CFStringRef; external name '_kUTTypeMPEG'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeMPEG4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeMPEG4: CFStringRef; external name '_kUTTypeMPEG4'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeMP3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeMP3: CFStringRef; external name '_kUTTypeMP3'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeMPEG4Audio
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeMPEG4Audio: CFStringRef; external name '_kUTTypeMPEG4Audio'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeAppleProtectedMPEG4Audio
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeAppleProtectedMPEG4Audio: CFStringRef; external name '_kUTTypeAppleProtectedMPEG4Audio'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{ ================================================================================ *
 *  Directory types                                                                 *
 * ================================================================================ }
{
 *  kUTTypeFolder
 *
 *    a user-browsable directory (i.e., not a package)
 *
 *    UTI: public.folder
 *    conforms to: public.directory
 *
 *
 *  kUTTypeVolume
 *
 *    the root folder of a volume/mount point
 *
 *    UTI: public.volume
 *    conforms to: public.folder
 *
 *
 *  kUTTypePackage
 *
 *    a packaged directory
 *
 *    UTI: com.apple.package
 *    conforms to: public.directory
 *
 *
 *  kUTTypeBundle
 *
 *    a directory conforming to one of the CFBundle layouts
 *
 *    UTI: com.apple.bundle
 *    conforms to: public.directory
 *
 *
 *  kUTTypeFramework
 *
 *    a Mac OS X framework
 *
 *    UTI: com.apple.framework
 *    conforms to: com.apple.bundle
 }
<<<<<<< HEAD
<<<<<<< HEAD
var kUTTypeFolder: CFStringRef; external name '_kUTTypeFolder'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeVolume: CFStringRef; external name '_kUTTypeVolume'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypePackage: CFStringRef; external name '_kUTTypePackage'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeBundle: CFStringRef; external name '_kUTTypeBundle'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeFramework: CFStringRef; external name '_kUTTypeFramework'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)


=======
=======
>>>>>>> origin/fixes_2_2
{
 *  kUTTypeFolder
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeFolder: CFStringRef; external name '_kUTTypeFolder'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeVolume
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeVolume: CFStringRef; external name '_kUTTypeVolume'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypePackage
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypePackage: CFStringRef; external name '_kUTTypePackage'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeBundle
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeBundle: CFStringRef; external name '_kUTTypeBundle'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeFramework
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeFramework: CFStringRef; external name '_kUTTypeFramework'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{ ================================================================================ *
 *  Application types                                                               *
 * ================================================================================ }
{
 *  kUTTypeApplicationBundle
 *
 *    a bundled application
 *
 *    UTI: com.apple.application-bundle
 *    conforms to: com.apple.application, com.apple.bundle, com.apple.package
 *
 *
 *  kUTTypeApplicationFile
 *
 *    a single-file Carbon/Classic application 
 *
 *    UTI: com.apple.application-file
 *    conforms to: com.apple.application, public.data
 }
<<<<<<< HEAD
<<<<<<< HEAD
var kUTTypeApplicationBundle: CFStringRef; external name '_kUTTypeApplicationBundle'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)
var kUTTypeApplicationFile: CFStringRef; external name '_kUTTypeApplicationFile'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)


=======
=======
>>>>>>> origin/fixes_2_2
{
 *  kUTTypeApplicationBundle
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeApplicationBundle: CFStringRef; external name '_kUTTypeApplicationBundle'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kUTTypeApplicationFile
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeApplicationFile: CFStringRef; external name '_kUTTypeApplicationFile'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{ ================================================================================ *
 *  Contact types                                                                   *
 * ================================================================================ }
{
 *  kUTTypeVCard
 *
 *    VCard format
 *
 *    UTI: public.vcard
 *    conforms to: public.data, public.contact
 }
<<<<<<< HEAD
<<<<<<< HEAD
var kUTTypeVCard: CFStringRef; external name '_kUTTypeVCard'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)


=======
=======
>>>>>>> origin/fixes_2_2
{
 *  kUTTypeVCard
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeVCard: CFStringRef; external name '_kUTTypeVCard'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{ ================================================================================ *
 *  Misc. types                                                                     *
 * ================================================================================ }
{
 *  kUTTypeInkText
 *
 *    Opaque InkText data
 *
 *    UTI: com.apple.ink.inktext
 *    conforms to: public.data
 }
<<<<<<< HEAD
<<<<<<< HEAD
var kUTTypeInkText: CFStringRef; external name '_kUTTypeInkText'; (* attribute const *)
(* __OSX_AVAILABLE_STARTING(__MAC_10_4,__IPHONE_3_0) *)

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
{
 *  kUTTypeInkText
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kUTTypeInkText: CFStringRef; external name '_kUTTypeInkText'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

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
