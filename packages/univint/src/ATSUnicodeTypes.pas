{
     File:       QD/ATSUnicodeTypes.h
 
     Contains:   ATSUI types and constants.
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Version:    Quickdraw-285~150
=======
     Version:    Quickdraw-262~1
>>>>>>> graemeg/cpstrnew
=======
     Version:    Quickdraw-262~1
>>>>>>> graemeg/cpstrnew
=======
     Version:    Quickdraw-262~1
>>>>>>> graemeg/cpstrnew
=======
     Version:    Quickdraw-262~1
>>>>>>> origin/cpstrnew
 
     Copyright:  © 2003-2008 by Apple Inc. all rights reserved.
=======
     Version:    Quickdraw-150~1
 
     Copyright:  © 2003 by Apple Computer, Inc., all rights reserved.
>>>>>>> graemeg/fixes_2_2
=======
     Version:    Quickdraw-150~1
 
     Copyright:  © 2003 by Apple Computer, Inc., all rights reserved.
>>>>>>> origin/fixes_2_2
=======
     Version:    Quickdraw-262~1
 
     Copyright:  © 2003-2008 by Apple Inc. all rights reserved.
>>>>>>> origin/fixes_2.4
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}
<<<<<<< HEAD
{  Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, 2004 }
{   Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
=======
{	  Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, 2004 }
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
                     http://www.freepascal.org/bugs.html
 
}
{	  Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, 2004 }
{   Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit ATSUnicodeTypes;
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
=======
>>>>>>> graemeg/cpstrnew
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
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
<<<<<<< HEAD
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
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
<<<<<<< HEAD
{$elsec}
<<<<<<< HEAD
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
=======
>>>>>>> origin/fixes_2.4
uses MacTypes,MacMemory,ATSLayoutTypes,Fonts,QuickdrawTypes,SFNTTypes,SFNTLayoutTypes,ATSTypes,TextCommon;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}
<<<<<<< HEAD
=======
uses MacTypes,MacMemory,ATSLayoutTypes,Fonts,Quickdraw,SFNTTypes,SFNTLayoutTypes,ATSTypes,TextCommon;
{$ALIGN MAC68K}
>>>>>>> graemeg/fixes_2_2
=======
uses MacTypes,MacMemory,ATSLayoutTypes,Fonts,Quickdraw,SFNTTypes,SFNTLayoutTypes,ATSTypes,TextCommon;
{$ALIGN MAC68K}
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{ See also ATSLayoutTypes.h for more ATSUI-related types and constants }
{ ---------------------------------------------------------------------------- }
{ ATSUI types and related constants                                            }
{ ---------------------------------------------------------------------------- }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4

{$ALIGN MAC68K}


<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{
 *  ATSUTextLayout
 *  
 *  Discussion:
 *    Text layout objects are the heart of ATSUI. These opaque objects
 *    associate Unicode text with style runs, store information about
 *    softbreaks, line and layout controls, and other information about
 *    the text. Text drawing is done by passing a valid layout object
 *    and a range of text to draw to the function ATSUDrawText, after
 *    initial setup work on the layout has been done. See the
 *    definitions of the functions ATSUCreateTextLayout and
 *    ATSUCreateTextLayoutWithTextPtr for more information about
 *    creating layouts.
 }
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	ATSUTextLayout = ^OpaqueATSUTextLayout; { an opaque type }
	OpaqueATSUTextLayout = record end;
=======
	ATSUTextLayout = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	ATSUTextLayout = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	ATSUTextLayout = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	ATSUTextLayout = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	ATSUTextLayout    = ^SInt32; { an opaque 32-bit type }
>>>>>>> graemeg/fixes_2_2
=======
	ATSUTextLayout    = ^SInt32; { an opaque 32-bit type }
>>>>>>> origin/fixes_2_2
=======
	ATSUTextLayout = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4

{
 *  ATSUStyle
 *  
 *  Discussion:
 *    Style objects retain information about text such as font, point
 *    size, color and other attributes. Styles are associated with text
 *    through a layout object. See the definitions of the functions
 *    ATSUSetRunStyle and ATSUCreateTextLayoutWithTextPtr for more
 *    information about assigning styles to runs of text in a layout
 *    object. See the definitions of the functions ATSUCreateStyle and
 *    ATSUSetAttributes for more information on creating and
 *    manipulating styles.
 }
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	ATSUStyle = ^OpaqueATSUStyle; { an opaque type }
	OpaqueATSUStyle = record end;
=======
	ATSUStyle = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	ATSUStyle = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	ATSUStyle = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	ATSUStyle = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	ATSUStyle    = ^SInt32; { an opaque 32-bit type }
>>>>>>> graemeg/fixes_2_2
=======
	ATSUStyle    = ^SInt32; { an opaque 32-bit type }
>>>>>>> origin/fixes_2_2
=======
	ATSUStyle = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4
	ATSUStylePtr = ^ATSUStyle;

{
 *  ATSUFontFallbacks
 *  
 *  Discussion:
 *    ATSUFontFallbacks objects are used to store the desired font
 *    fallback mode, list, and associated cache information. See the
 *    definitions of ATSUFontFallbackMethod and ATSUSetObjFontFallbacks
 *    for more information about setting up font fallbacks.
 }
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	ATSUFontFallbacks = ^OpaqueATSUFontFallbacks; { an opaque type }
	OpaqueATSUFontFallbacks = record end;
=======
	ATSUFontFallbacks = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	ATSUFontFallbacks = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	ATSUFontFallbacks = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	ATSUFontFallbacks = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	ATSUFontFallbacks    = ^SInt32; { an opaque 32-bit type }
>>>>>>> graemeg/fixes_2_2
=======
	ATSUFontFallbacks    = ^SInt32; { an opaque 32-bit type }
>>>>>>> origin/fixes_2_2
=======
	ATSUFontFallbacks = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4

{
 *  ATSUTextMeasurement
 *  
 *  Discussion:
 *    ATSUTextMeasurement is specific to ATSUI in that measurement
 *    quantities are in fractional Fixed format instead of shorts used
 *    in QuickDraw Text. This provides exact outline metrics and line
 *    specifications such as line width, ascent, descent, and so on.
 *    See FixMath.h for conversion functions for Fixed numbers.
 }
type
	ATSUTextMeasurement = Fixed;
	ATSUTextMeasurementPtr = ^ATSUTextMeasurement;

{
 *  ATSUFontID
 *  
 *  Discussion:
 *    ATSUFontID indicates a particular font family and face.
 *    ATSUFontID's are not guaranteed to remain constant across
 *    sessions. Clients should use the font's unique name to get a font
 *    token to store in documents which is guaranteed to remain
 *    constant across sessions.
 }
type
	ATSUFontID = FMFont;
	ATSUFontIDPtr = ^ATSUFontID;

{
 *  ATSUFontFeatureType
 *  
 *  Discussion:
 *    Used to identify a font feature type for a particular font. See
 *    the definitions of the functions ATSUGetAllFontFeatures and
 *    ATSUGetFontFeature for more information about font features.
 }
type
	ATSUFontFeatureType = UInt16;
	ATSUFontFeatureTypePtr = ^ATSUFontFeatureType;

{
 *  ATSUFontFeatureSelector
 *  
 *  Discussion:
 *    Used to identify a font feature selector for a particular font.
 *    See the definitions of the functions ATSUGetAllFontFeatures and
 *    ATSUGetFontFeature for more information about font features.
 }
type
	ATSUFontFeatureSelector = UInt16;
	ATSUFontFeatureSelectorPtr = ^ATSUFontFeatureSelector;

{
 *  ATSUFontVariationAxis
 *  
 *  Discussion:
 *    Used to identify a font variation axis for a particular font. See
 *    the definitions of the functions ATSUGetAllFontVariations and
 *    ATSUGetFontVariation for more information about font variations.
 }
type
	ATSUFontVariationAxis = FourCharCode;
	ATSUFontVariationAxisPtr = ^ATSUFontVariationAxis;

{
 *  ATSUFontVariationValue
 *  
 *  Discussion:
 *    Used to identify a font variation value for a particular font.
 *    See the definitions of the functions ATSUGetAllFontVariations and
 *    ATSUGetFontVariation for more information about font variations.
 }
type
	ATSUFontVariationValue = Fixed;
	ATSUFontVariationValuePtr = ^ATSUFontVariationValue;

{
 *  ATSUAttributeTag
 *  
 *  Summary:
 *    Constants used for style attributes, line controls, and layout
 *    controls.
 *  
 *  Discussion:
 *    The following constants are used to change settings in text
 *    layout and style objects. Use the functions ATSUSetLineControls,
 *    ATSUSetLayoutControls, and ATSUSetAttributes to set these values
 *    in lines, layouts, and styles, respectively. Apple reserves tag
 *    values 0 to 65535 (0 to 0x0000FFFF). ATSUI clients may create
 *    their own tags with any other value.
 }
type
	ATSUAttributeTag = UInt32;
	ATSUAttributeTagPtr = ^ATSUAttributeTag;
const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{
   * (Type: ATSUTextMeasurement) (Default value: 0) Must not be less
   * than zero. May be set as a line or layout control.
   }
	kATSULineWidthTag = 1;
=======
=======
>>>>>>> origin/fixes_2_2

  {
   * (Type: ATSUTextMeasurement) (Default value: 0) Must not be less
   * than zero. May be set as a line or layout control.
   }
  kATSULineWidthTag             = 1;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{
   * (Type: ATSUTextMeasurement) (Default value: 0) Must not be less
   * than zero. May be set as a line or layout control.
   }
	kATSULineWidthTag = 1;
>>>>>>> origin/fixes_2.4

  {
   * (Type: Fixed) (Default value: 0) Angle is specified in degrees in
   * right-handed coordinate system. May be set as a line control.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSULineRotationTag = 2;

  {
   * (Type: Boolean) (Default value: GetSysDirection()) Must be 0 or 1.
   * See below for convenience constants. May be set as a layout
   * control.
   }
	kATSULineDirectionTag = 3;
=======
=======
>>>>>>> origin/fixes_2_2
  kATSULineRotationTag          = 2;
=======
	kATSULineRotationTag = 2;
>>>>>>> origin/fixes_2.4

  {
   * (Type: Boolean) (Default value: GetSysDirection()) Must be 0 or 1.
   * See below for convenience constants. May be set as a layout
   * control.
   }
<<<<<<< HEAD
  kATSULineDirectionTag         = 3;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kATSULineDirectionTag = 3;
>>>>>>> origin/fixes_2.4

  {
   * (Type: Fract) (Default value: kATSUNoJustification) May be set as
   * a line or layout control.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSULineJustificationFactorTag = 4;
=======
  kATSULineJustificationFactorTag = 4;
>>>>>>> graemeg/fixes_2_2
=======
  kATSULineJustificationFactorTag = 4;
>>>>>>> origin/fixes_2_2
=======
	kATSULineJustificationFactorTag = 4;
>>>>>>> origin/fixes_2.4

  {
   * (Type: Fract) (Default value: kATSUStartAlignment) May be set as a
   * line or layout control.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSULineFlushFactorTag = 5;
=======
  kATSULineFlushFactorTag       = 5;
>>>>>>> graemeg/fixes_2_2
=======
  kATSULineFlushFactorTag       = 5;
>>>>>>> origin/fixes_2_2
=======
	kATSULineFlushFactorTag = 5;
>>>>>>> origin/fixes_2.4

  {
   * (Type: BslnBaselineRecord) (Default value: all zeros) Calculated
   * from other style attributes (e.g., font and point size). May be
   * set as a line or layout control.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSULineBaselineValuesTag = 6;
=======
  kATSULineBaselineValuesTag    = 6;
>>>>>>> graemeg/fixes_2_2
=======
  kATSULineBaselineValuesTag    = 6;
>>>>>>> origin/fixes_2_2
=======
	kATSULineBaselineValuesTag = 6;
>>>>>>> origin/fixes_2.4

  {
   * (Type: ATSLineLayoutOptions) (Default value: all zeros) See
   * ATSLayoutTypes.h for a definition of the ATSLineLayoutOptions type
   * and a list of possible values. May be set as a line or layout
   * control.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSULineLayoutOptionsTag = 7;
=======
  kATSULineLayoutOptionsTag     = 7;
>>>>>>> graemeg/fixes_2_2
=======
  kATSULineLayoutOptionsTag     = 7;
>>>>>>> origin/fixes_2_2
=======
	kATSULineLayoutOptionsTag = 7;
>>>>>>> origin/fixes_2.4

  {
   * (Type: ATSUTextMeasurement) (Default value: determined by font(s))
   * Must not be less than zero. Starting in Mac OS X 10.2, you can
   * retrieve this value as a line or layout control, even if you have
   * not explicitly set it. This makes it easy to calculate line
   * height. May be set as a line or layout control.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSULineAscentTag = 8;
=======
  kATSULineAscentTag            = 8;
>>>>>>> graemeg/fixes_2_2
=======
  kATSULineAscentTag            = 8;
>>>>>>> origin/fixes_2_2
=======
	kATSULineAscentTag = 8;
>>>>>>> origin/fixes_2.4

  {
   * (Type: ATSUTextMeasurement) (Default value: determined by font(s))
   * Must not be less than zero. Starting in Mac OS X, you can retrieve
   * this value as a line or layout control, even if you have not
   * explicitly set it. This makes it easy to calculate line height.
   * May be set as a line or layout control.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSULineDescentTag = 9;
=======
  kATSULineDescentTag           = 9;
>>>>>>> graemeg/fixes_2_2
=======
  kATSULineDescentTag           = 9;
>>>>>>> origin/fixes_2_2
=======
	kATSULineDescentTag = 9;
>>>>>>> origin/fixes_2.4

  {
   * (Type: RegionCode) (Default value: kTextRegionDontCare) See
   * Script.h for possible values. May be set as a line or layout
   * control.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSULineLangRegionTag = 10;
=======
  kATSULineLangRegionTag        = 10;
>>>>>>> graemeg/fixes_2_2
=======
  kATSULineLangRegionTag        = 10;
>>>>>>> origin/fixes_2_2
=======
	kATSULineLangRegionTag = 10;
>>>>>>> origin/fixes_2.4

  {
   * (Type: TextBreakLocatorRef) (Default value: NULL) See
   * UnicodeUtilities.h for more information on creating a
   * TextBreakLocator. May be set as a line or layout control.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSULineTextLocatorTag = 11;
=======
  kATSULineTextLocatorTag       = 11;
>>>>>>> graemeg/fixes_2_2
=======
  kATSULineTextLocatorTag       = 11;
>>>>>>> origin/fixes_2_2
=======
	kATSULineTextLocatorTag = 11;
>>>>>>> origin/fixes_2.4

  {
   * (Type: ATSULineTruncation) (Default value: kATSUTruncateNone) See
   * the definition of ATSULineTruncation for possible values. May be
   * set as a line or layout control.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSULineTruncationTag = 12;
=======
  kATSULineTruncationTag        = 12;
>>>>>>> graemeg/fixes_2_2
=======
  kATSULineTruncationTag        = 12;
>>>>>>> origin/fixes_2_2
=======
	kATSULineTruncationTag = 12;
>>>>>>> origin/fixes_2.4

  {
   * (Type: ATSUFontFallbacks) (Default value: current global fallback
   * state) The current global fallback state is determined using the
   * ATSUSetFontFallbacks function. The use of this function is not
   * recommended. Instead, use the functions ATSUCreateFontFallbacks
   * and ATSUSetObjFontFallbacks to create a ATSUFontFallbacks object,
   * and then use the kATSULineFontFallbacksTag attribute to set the
   * font fallbacks object as a layout control. See the definition of
   * ATSUFontFallbacks for more information. May be set as a layout
   * control.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSULineFontFallbacksTag = 13;
=======
  kATSULineFontFallbacksTag     = 13;
>>>>>>> graemeg/fixes_2_2
=======
  kATSULineFontFallbacksTag     = 13;
>>>>>>> origin/fixes_2_2
=======
	kATSULineFontFallbacksTag = 13;
>>>>>>> origin/fixes_2.4

  {
   * (Type: CFStringRef) (Default value: user setting in System
   * Preferences) Indicates current setting for the decimal separator.
   * This affects the behavior of decimal tabs. May be set as a line or
   * layout control.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSULineDecimalTabCharacterTag = 14;

  {
   * (Type: ATSULayoutOperationOverrideSpecifier) (Default value: NULL)
   * See ATSLayoutTypes.h for a definition of the
   * ATSULayoutOperationOverrideSpecifier structure. May be set as a
   * layout control.
   }
	kATSULayoutOperationOverrideTag = 15;
=======
=======
>>>>>>> origin/fixes_2_2
  kATSULineDecimalTabCharacterTag = 14;
=======
	kATSULineDecimalTabCharacterTag = 14;
>>>>>>> origin/fixes_2.4

  {
   * (Type: ATSULayoutOperationOverrideSpecifier) (Default value: NULL)
   * See ATSLayoutTypes.h for a definition of the
   * ATSULayoutOperationOverrideSpecifier structure. May be set as a
   * layout control.
   }
<<<<<<< HEAD
  kATSULayoutOperationOverrideTag = 15;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kATSULayoutOperationOverrideTag = 15;
>>>>>>> origin/fixes_2.4

  {
   * (Type: CGColorRef) (Default value: user setting in System
   * Preferences) Indicates current setting for the highlight color.
   * May be set as a line or layout control.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSULineHighlightCGColorTag = 17;
=======
  kATSULineHighlightCGColorTag  = 17;
>>>>>>> graemeg/fixes_2_2
=======
  kATSULineHighlightCGColorTag  = 17;
>>>>>>> origin/fixes_2_2
=======
	kATSULineHighlightCGColorTag = 17;
>>>>>>> origin/fixes_2.4

  {
   * This is just for convenience. It is the upper limit of the line
   * and layout tags.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUMaxLineTag = 18;
=======
  kATSUMaxLineTag               = 18;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUMaxLineTag               = 18;
>>>>>>> origin/fixes_2_2
=======
	kATSUMaxLineTag = 18;
>>>>>>> origin/fixes_2.4

  {
   * This tag is obsolete. Please use kATSULineLangRegionTag instead.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSULineLanguageTag = 10;
=======
  kATSULineLanguageTag          = 10;
>>>>>>> graemeg/fixes_2_2
=======
  kATSULineLanguageTag          = 10;
>>>>>>> origin/fixes_2_2
=======
	kATSULineLanguageTag = 10;
>>>>>>> origin/fixes_2.4

  {
   * (Type: CGContextRef) (Default value: NULL) Use this tag to produce
   * Quartz rendering with ATSUI. See the definitions of the functions
   * QDBeginCGContext and QDEndCGContext in Quickdraw.h for more
   * information about creating a CGContext from a graphics port. May
   * be set as a layout control.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUCGContextTag = 32767;
=======
  kATSUCGContextTag             = 32767;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUCGContextTag             = 32767;
>>>>>>> origin/fixes_2_2
=======
	kATSUCGContextTag = 32767;
>>>>>>> origin/fixes_2.4

  {
   * (Type: Boolean) (Default value: false) For compatability purposes
   * only. Choosing typographic styles from font families is preferred.
   * Note this tag will produce a synthetic style for fonts that do not
   * have a typographic style. May be set as a style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUQDBoldfaceTag = 256;
=======
  kATSUQDBoldfaceTag            = 256;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUQDBoldfaceTag            = 256;
>>>>>>> origin/fixes_2_2
=======
	kATSUQDBoldfaceTag = 256;
>>>>>>> origin/fixes_2.4

  {
   * (Type: Boolean) (Default value: false) For compatability purposes
   * only. Choosing typographic styles from font families is preferred.
   * Note this tag will produce a synthetic style for fonts that do not
   * have a typographic style. May be set as a style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUQDItalicTag = 257;
=======
  kATSUQDItalicTag              = 257;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUQDItalicTag              = 257;
>>>>>>> origin/fixes_2_2
=======
	kATSUQDItalicTag = 257;
>>>>>>> origin/fixes_2.4

  {
   * (Type: Boolean) (Default value: false) For compatability purposes
   * only. May be set as a style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUQDUnderlineTag = 258;
=======
  kATSUQDUnderlineTag           = 258;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUQDUnderlineTag           = 258;
>>>>>>> origin/fixes_2_2
=======
	kATSUQDUnderlineTag = 258;
>>>>>>> origin/fixes_2.4

  {
   * (Type: Boolean) (Default value: false) For compatability purposes
   * only. May be set as a style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUQDCondensedTag = 259;
=======
  kATSUQDCondensedTag           = 259;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUQDCondensedTag           = 259;
>>>>>>> origin/fixes_2_2
=======
	kATSUQDCondensedTag = 259;
>>>>>>> origin/fixes_2.4

  {
   * (Type: Boolean) (Default value: false) For compatability purposes
   * only. May be set as a style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUQDExtendedTag = 260;
=======
  kATSUQDExtendedTag            = 260;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUQDExtendedTag            = 260;
>>>>>>> origin/fixes_2_2
=======
	kATSUQDExtendedTag = 260;
>>>>>>> origin/fixes_2.4

  {
   * (Type: ATSUFontID) (Default value: LMGetApFontID() or if not
   * valid, LMGetSysFontFam()) May be set as a style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUFontTag = 261;
=======
  kATSUFontTag                  = 261;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUFontTag                  = 261;
>>>>>>> origin/fixes_2_2
=======
	kATSUFontTag = 261;
>>>>>>> origin/fixes_2.4

  {
   * (Type: Fixed) (Default value: Long2Fix(LMGetSysFontSize())) May be
   * set as a style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUSizeTag = 262;
=======
  kATSUSizeTag                  = 262;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUSizeTag                  = 262;
>>>>>>> origin/fixes_2_2
=======
	kATSUSizeTag = 262;
>>>>>>> origin/fixes_2.4

  {
   * (Type: RGBColor) (Default value: (0, 0, 0)) May be set as a style
   * attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUColorTag = 263;
=======
  kATSUColorTag                 = 263;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUColorTag                 = 263;
>>>>>>> origin/fixes_2_2
=======
	kATSUColorTag = 263;
>>>>>>> origin/fixes_2.4

  {
   * (Type: RegionCode) (Default value:
   * GetScriptManagerVariable(smRegionCode)) See Script.h for a list of
   * possible values. May be set as a style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSULangRegionTag = 264;
=======
  kATSULangRegionTag            = 264;
>>>>>>> graemeg/fixes_2_2
=======
  kATSULangRegionTag            = 264;
>>>>>>> origin/fixes_2_2
=======
	kATSULangRegionTag = 264;
>>>>>>> origin/fixes_2.4

  {
   * (Type: ATSUVerticalCharacterType) (Default value:
   * kATSUStronglyHorizontal) See the definition of
   * ATSUVerticalCharacterType for a list of possible values. May be
   * set as a style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUVerticalCharacterTag = 265;
=======
  kATSUVerticalCharacterTag     = 265;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUVerticalCharacterTag     = 265;
>>>>>>> origin/fixes_2_2
=======
	kATSUVerticalCharacterTag = 265;
>>>>>>> origin/fixes_2.4

  {
   * (Type: ATSUTextMeasurement) (Default value: kATSUseGlyphAdvance)
   * Must not be less than zero. May be set as a style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUImposeWidthTag = 266;
=======
  kATSUImposeWidthTag           = 266;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUImposeWidthTag           = 266;
>>>>>>> origin/fixes_2_2
=======
	kATSUImposeWidthTag = 266;
>>>>>>> origin/fixes_2.4

  {
   * (Type: Fixed) (Default value: 0) May be set as a style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUBeforeWithStreamShiftTag = 267;
=======
  kATSUBeforeWithStreamShiftTag = 267;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUBeforeWithStreamShiftTag = 267;
>>>>>>> origin/fixes_2_2
=======
	kATSUBeforeWithStreamShiftTag = 267;
>>>>>>> origin/fixes_2.4

  {
   * (Type: Fixed) (Default value: 0) May be set as a style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUAfterWithStreamShiftTag = 268;
=======
  kATSUAfterWithStreamShiftTag  = 268;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUAfterWithStreamShiftTag  = 268;
>>>>>>> origin/fixes_2_2
=======
	kATSUAfterWithStreamShiftTag = 268;
>>>>>>> origin/fixes_2.4

  {
   * (Type: Fixed) (Default value: 0) May be set as a style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUCrossStreamShiftTag = 269;
=======
  kATSUCrossStreamShiftTag      = 269;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUCrossStreamShiftTag      = 269;
>>>>>>> origin/fixes_2_2
=======
	kATSUCrossStreamShiftTag = 269;
>>>>>>> origin/fixes_2.4

  {
   * (Type: Fixed) (Default value: kATSNoTracking) May be set as a
   * style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUTrackingTag = 270;
=======
  kATSUTrackingTag              = 270;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUTrackingTag              = 270;
>>>>>>> origin/fixes_2_2
=======
	kATSUTrackingTag = 270;
>>>>>>> origin/fixes_2.4

  {
   * (Type: Fract) (Default value: 0) May be set as a style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUHangingInhibitFactorTag = 271;
=======
  kATSUHangingInhibitFactorTag  = 271;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUHangingInhibitFactorTag  = 271;
>>>>>>> origin/fixes_2_2
=======
	kATSUHangingInhibitFactorTag = 271;
>>>>>>> origin/fixes_2.4

  {
   * (Type: Fract) (Default value: 0) May be set as a style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUKerningInhibitFactorTag = 272;
=======
  kATSUKerningInhibitFactorTag  = 272;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUKerningInhibitFactorTag  = 272;
>>>>>>> origin/fixes_2_2
=======
	kATSUKerningInhibitFactorTag = 272;
>>>>>>> origin/fixes_2.4

  {
   * (Type: Fixed) (Default value: 0) Must be between -1.0 and 1.0. May
   * be set as a style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUDecompositionFactorTag = 273;
=======
  kATSUDecompositionFactorTag   = 273;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUDecompositionFactorTag   = 273;
>>>>>>> origin/fixes_2_2
=======
	kATSUDecompositionFactorTag = 273;
>>>>>>> origin/fixes_2.4

  {
   * (Type: BslnBaselineClass) (Default value: kBSLNRomanBaseline) See
   * SFNTLayoutTypes.h for more information. Use the constant
   * kBSLNNoBaselineOverride to use intrinsic baselines. May be set as
   * a style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUBaselineClassTag = 274;
=======
  kATSUBaselineClassTag         = 274;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUBaselineClassTag         = 274;
>>>>>>> origin/fixes_2_2
=======
	kATSUBaselineClassTag = 274;
>>>>>>> origin/fixes_2.4

  {
   * (Type: ATSJustPriorityWidthDeltaOverrides) (Default value: all
   * zeros) See ATSLayoutTypes.h for more information. May be set as a
   * style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUPriorityJustOverrideTag = 275;
=======
  kATSUPriorityJustOverrideTag  = 275;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUPriorityJustOverrideTag  = 275;
>>>>>>> origin/fixes_2_2
=======
	kATSUPriorityJustOverrideTag = 275;
>>>>>>> origin/fixes_2.4

  {
   * (Type: Boolean) (Default value: false) When set to true, ligatures
   * and compound characters will not have divisable components. May be
   * set as a style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUNoLigatureSplitTag = 276;
=======
  kATSUNoLigatureSplitTag       = 276;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUNoLigatureSplitTag       = 276;
>>>>>>> origin/fixes_2_2
=======
	kATSUNoLigatureSplitTag = 276;
>>>>>>> origin/fixes_2.4

  {
   * (Type: Boolean) (Default value: false) When set to true, ATSUI
   * will not use a glyph's angularity to determine its boundaries. May
   * be set as a style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUNoCaretAngleTag = 277;
=======
  kATSUNoCaretAngleTag          = 277;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUNoCaretAngleTag          = 277;
>>>>>>> origin/fixes_2_2
=======
	kATSUNoCaretAngleTag = 277;
>>>>>>> origin/fixes_2.4

  {
   * (Type: Boolean) (Default value: false) When set to true, ATSUI
   * will suppress automatic cross kerning (defined by font). May be
   * set as a style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUSuppressCrossKerningTag = 278;
=======
  kATSUSuppressCrossKerningTag  = 278;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUSuppressCrossKerningTag  = 278;
>>>>>>> origin/fixes_2_2
=======
	kATSUSuppressCrossKerningTag = 278;
>>>>>>> origin/fixes_2.4

  {
   * (Type: Boolean) (Default value: false) When set to true, ATSUI
   * will suppress glyphs' automatic optical positional alignment. May
   * be set as a style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUNoOpticalAlignmentTag = 279;
=======
  kATSUNoOpticalAlignmentTag    = 279;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUNoOpticalAlignmentTag    = 279;
>>>>>>> origin/fixes_2_2
=======
	kATSUNoOpticalAlignmentTag = 279;
>>>>>>> origin/fixes_2.4

  {
   * (Type: Boolean) (Default value: false) When set to true, ATSUI
   * will force glyphs to hang beyond the line boundaries. May be set
   * as a style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUForceHangingTag = 280;
=======
  kATSUForceHangingTag          = 280;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUForceHangingTag          = 280;
>>>>>>> origin/fixes_2_2
=======
	kATSUForceHangingTag = 280;
>>>>>>> origin/fixes_2.4

  {
   * (Type: Boolean) (Default value: false) When set to true, ATSUI
   * will not perform post-compensation justification if needed. May be
   * set as a style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUNoSpecialJustificationTag = 281;
=======
  kATSUNoSpecialJustificationTag = 281;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUNoSpecialJustificationTag = 281;
>>>>>>> origin/fixes_2_2
=======
	kATSUNoSpecialJustificationTag = 281;
>>>>>>> origin/fixes_2.4

  {
   * (Type: TextBreakLocatorRef) (Default value: NULL) See
   * UnicodeUtilities.h for more information about creating a
   * TextBreakLocator. May be set as a style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUStyleTextLocatorTag = 282;
=======
  kATSUStyleTextLocatorTag      = 282;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUStyleTextLocatorTag      = 282;
>>>>>>> origin/fixes_2_2
=======
	kATSUStyleTextLocatorTag = 282;
>>>>>>> origin/fixes_2.4

  {
   * (Type: ATSStyleRenderingOptions) (Default value:
   * kATSStyleNoOptions) See ATSLayoutTypes.h for a definition of
   * ATSStyleRenderingOptions and a list of possible values. May be set
   * as a style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUStyleRenderingOptionsTag = 283;
=======
  kATSUStyleRenderingOptionsTag = 283;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUStyleRenderingOptionsTag = 283;
>>>>>>> origin/fixes_2_2
=======
	kATSUStyleRenderingOptionsTag = 283;
>>>>>>> origin/fixes_2.4

  {
   * (Type: ATSUTextMeasurement) (Default value: determined by font)
   * Must not be less than zero. Starting in Mac OS X 10.2, you can
   * retrieve a value for this attribute, even if you have not
   * explicitly set it. This can make calculating line height easier.
   * May be set as a style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUAscentTag = 284;
=======
  kATSUAscentTag                = 284;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUAscentTag                = 284;
>>>>>>> origin/fixes_2_2
=======
	kATSUAscentTag = 284;
>>>>>>> origin/fixes_2.4

  {
   * (Type: ATSUTextMeasurement) (Default value: determined by font)
   * Must not be less than zero. Starting in Mac OS X 10.2, you can
   * retrieve a value for this attribute, even if you have not
   * explicitly set it. This can make calculating line height easier.
   * May be set as a style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUDescentTag = 285;
=======
  kATSUDescentTag               = 285;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUDescentTag               = 285;
>>>>>>> origin/fixes_2_2
=======
	kATSUDescentTag = 285;
>>>>>>> origin/fixes_2.4

  {
   * (Type: ATSUTextMeasurement) (Default value: determined by font)
   * Must not be less than zero. Starting in Mac OS X 10.2, you can
   * retrieve a value for this attribute, even if you have not
   * explicitly set it. This can make calculating line height easier.
   * May be set as a style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSULeadingTag = 286;
=======
  kATSULeadingTag               = 286;
>>>>>>> graemeg/fixes_2_2
=======
  kATSULeadingTag               = 286;
>>>>>>> origin/fixes_2_2
=======
	kATSULeadingTag = 286;
>>>>>>> origin/fixes_2.4

  {
   * (Type: ATSUGlyphSelector) (Default value: 0) See the definition of
   * ATSUGlyphSelector for more information and a list of possible
   * values. May be set as a style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUGlyphSelectorTag = 287;
=======
  kATSUGlyphSelectorTag         = 287;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUGlyphSelectorTag         = 287;
>>>>>>> origin/fixes_2_2
=======
	kATSUGlyphSelectorTag = 287;
>>>>>>> origin/fixes_2.4

  {
   * (Type: ATSURGBAlphaColor) (Default value: (0, 0, 0, 1)) See the
   * definition of ATSURGBAlphaColor for more information. May be set
   * as a style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSURGBAlphaColorTag = 288;
=======
  kATSURGBAlphaColorTag         = 288;
>>>>>>> graemeg/fixes_2_2
=======
  kATSURGBAlphaColorTag         = 288;
>>>>>>> origin/fixes_2_2
=======
	kATSURGBAlphaColorTag = 288;
>>>>>>> origin/fixes_2.4

  {
   * (Type: CGAffineTransform) (Default value:
   * CGAffineTransformIdentity) See the definition of CGAffineTransform
   * in CGAffineTransform.h for more information. May be set as a style
   * attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUFontMatrixTag = 289;
=======
  kATSUFontMatrixTag            = 289;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUFontMatrixTag            = 289;
>>>>>>> origin/fixes_2_2
=======
	kATSUFontMatrixTag = 289;
>>>>>>> origin/fixes_2.4

  {
   * (Type: ATSUStyleLineCountType) (Default value:
   * kATSUStyleSingleLineCount) Used to specify the number of strokes
   * to be drawn for an underline. May be set as a style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUStyleUnderlineCountOptionTag = 290;
=======
  kATSUStyleUnderlineCountOptionTag = 290;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUStyleUnderlineCountOptionTag = 290;
>>>>>>> origin/fixes_2_2
=======
	kATSUStyleUnderlineCountOptionTag = 290;
>>>>>>> origin/fixes_2.4

  {
   * (Type: CGColorRef) (Default value: NULL) Used to specify the color
   * of the strokes to draw for an underlined run of text. If NULL, the
   * text color is used. May be set as a style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUStyleUnderlineColorOptionTag = 291;
=======
  kATSUStyleUnderlineColorOptionTag = 291;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUStyleUnderlineColorOptionTag = 291;
>>>>>>> origin/fixes_2_2
=======
	kATSUStyleUnderlineColorOptionTag = 291;
>>>>>>> origin/fixes_2.4

  {
   * (Type: Boolean) (Default value: false) Used to specify
   * strikethrough style. May be set as a style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUStyleStrikeThroughTag = 292;
=======
  kATSUStyleStrikeThroughTag    = 292;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUStyleStrikeThroughTag    = 292;
>>>>>>> origin/fixes_2_2
=======
	kATSUStyleStrikeThroughTag = 292;
>>>>>>> origin/fixes_2.4

  {
   * (Type: ATSUStyleLineCountType) (Default value:
   * kATSUStyleSingleLineCount) Used to specify the number of strokes
   * to be drawn for a strikethrough. May be set as a style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUStyleStrikeThroughCountOptionTag = 293;
=======
  kATSUStyleStrikeThroughCountOptionTag = 293;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUStyleStrikeThroughCountOptionTag = 293;
>>>>>>> origin/fixes_2_2
=======
	kATSUStyleStrikeThroughCountOptionTag = 293;
>>>>>>> origin/fixes_2.4

  {
   * (Type: CGColorRef) (Default value: NULL) Used to specify the color
   * of the strokes to draw for a strikethrough style. If NULL, the
   * text color is used. May be set as a style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUStyleStrikeThroughColorOptionTag = 294;
=======
  kATSUStyleStrikeThroughColorOptionTag = 294;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUStyleStrikeThroughColorOptionTag = 294;
>>>>>>> origin/fixes_2_2
=======
	kATSUStyleStrikeThroughColorOptionTag = 294;
>>>>>>> origin/fixes_2.4

  {
   * (Type: Boolean) (Default value: false) Used to specify if text
   * should be drawn with a drop shadow. Only takes effect if a
   * CGContext is used for drawing. May be set as a style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUStyleDropShadowTag = 295;
=======
  kATSUStyleDropShadowTag       = 295;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUStyleDropShadowTag       = 295;
>>>>>>> origin/fixes_2_2
=======
	kATSUStyleDropShadowTag = 295;
>>>>>>> origin/fixes_2.4

  {
   * (Type: float) (Default value: 0.0) Used to specify the amount of
   * blur for a dropshadow. May be set as a style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUStyleDropShadowBlurOptionTag = 296;
=======
  kATSUStyleDropShadowBlurOptionTag = 296;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUStyleDropShadowBlurOptionTag = 296;
>>>>>>> origin/fixes_2_2
=======
	kATSUStyleDropShadowBlurOptionTag = 296;
>>>>>>> origin/fixes_2.4

  {
   * (Type: CGSize) (Default value: (3.0, -3.0)) Used to specify the
   * amount of offset from the text to be used when drawing a
   * dropshadow. May be set as a style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUStyleDropShadowOffsetOptionTag = 297;
=======
  kATSUStyleDropShadowOffsetOptionTag = 297;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUStyleDropShadowOffsetOptionTag = 297;
>>>>>>> origin/fixes_2_2
=======
	kATSUStyleDropShadowOffsetOptionTag = 297;
>>>>>>> origin/fixes_2.4

  {
   * (Type: CGColorRef) (Default value: NULL) Used to specify the color
   * of the dropshadow. May be set as a style attribute.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUStyleDropShadowColorOptionTag = 298;
=======
  kATSUStyleDropShadowColorOptionTag = 298;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUStyleDropShadowColorOptionTag = 298;
>>>>>>> origin/fixes_2_2
=======
	kATSUStyleDropShadowColorOptionTag = 298;
>>>>>>> origin/fixes_2.4

  {
   * This is just for convenience. It is the upper limit of the style
   * tags.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUMaxStyleTag = 299;
=======
  kATSUMaxStyleTag              = 299;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUMaxStyleTag              = 299;
>>>>>>> origin/fixes_2_2
=======
	kATSUMaxStyleTag = 299;
>>>>>>> origin/fixes_2.4

  {
   * This tag is obsolete. Please use kATSULangRegionTag instead. This
   * is the maximum Apple ATSUI reserved tag value.  Client defined
   * tags must be larger.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSULanguageTag = 264;
	kATSUMaxATSUITagValue = 65535;
=======
  kATSULanguageTag              = 264;
  kATSUMaxATSUITagValue         = 65535;
>>>>>>> graemeg/fixes_2_2
=======
  kATSULanguageTag              = 264;
  kATSUMaxATSUITagValue         = 65535;
>>>>>>> origin/fixes_2_2
=======
	kATSULanguageTag = 264;
	kATSUMaxATSUITagValue = 65535;
>>>>>>> origin/fixes_2.4


{
 *  ATSUAttributeValuePtr
 *  
 *  Summary:
 *    Used to provide generic access for storage of attribute values,
 *    which vary in size.
 }
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	ATSUAttributeValuePtr = UnivPtr;
	ConstATSUAttributeValuePtr = UnivPtr;
=======
	ATSUAttributeValuePtr = Ptr;
	ConstATSUAttributeValuePtr = Ptr;
>>>>>>> graemeg/fixes_2_2
=======
	ATSUAttributeValuePtr = Ptr;
	ConstATSUAttributeValuePtr = Ptr;
>>>>>>> origin/fixes_2_2
=======
	ATSUAttributeValuePtr = UnivPtr;
	ConstATSUAttributeValuePtr = UnivPtr;
>>>>>>> origin/fixes_2.4
	ATSUAttributeValuePtrPtr = ^ATSUAttributeValuePtr;

{
 *  ATSUAttributeInfo
 *  
 *  Discussion:
 *    ATSUAttributeInfo is used to provide a tag/size pairing. This
 *    makes it possible to provide the client information about all the
 *    attributes for a given range of text.  This structure is only
 *    used to return to the client information about a complete set of
 *    attributes.  An array of ATSUAttributeInfos is passed as a
 *    parameter so that the client can find out what attributes are set
 *    and what their individual sizes are; with that information, they
 *    can then query about the values of the attributes they're
 *    interested in. Because arrays of ATSUAttributeInfos are used as
 *    parameters to functions, they have to be of a fixed size, hence
 *    the value is not included in the structure.
 }
type
	ATSUAttributeInfo = record
		fTag: ATSUAttributeTag;
		fValueSize: ByteCount;
	end;
	ATSUAttributeInfoPtr = ^ATSUAttributeInfo;

{
 *  ATSUCaret
 *  
 *  Discussion:
 *    Contains the complete information needed to render a caret.  fX
 *    and fY is the position of one of the caret's ends relative to the
 *    origin position of the line the caret belongs. fDeltaX and
 *    fDeltaY is the position of the caret's other end. Hence, to draw
 *    a caret, simply call MoveTo(fX, fY) followed by LineTo(fDeltaX,
 *    fDeltaY) or equivalent.  The ATSUCaret will contain the positions
 *    needed to draw carets on angled lines and reflect angled carets
 *    and leading/trailing split caret appearances.
 }
type
	ATSUCaret = record
		fX: Fixed;
		fY: Fixed;
		fDeltaX: Fixed;
		fDeltaY: Fixed;
	end;
	ATSUCaretPtr = ^ATSUCaret;

{
 *  ATSUCursorMovementType
 *  
 *  Discussion:
 *    Used to indicate how much to move the cursor when using the ATSUI
 *    cusor movement routines. Note that kATSUByCharacterCluster is
 *    only available in Mac OS X and in CarbonLib versions 1.3 and
 *    later.
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
	ATSUCursorMovementType = UInt16;
const
{
   * Cursor movement based on individual characters. The cursor will
   * step through individual characters within ligatures.
   }
	kATSUByCharacter = 0;
=======
=======
>>>>>>> origin/fixes_2_2
type ATSUCursorMovementType = UInt16;
=======
type
	ATSUCursorMovementType = UInt16;
>>>>>>> origin/fixes_2.4
const
{
   * Cursor movement based on individual characters. The cursor will
   * step through individual characters within ligatures.
   }
<<<<<<< HEAD
  kATSUByCharacter              = 0;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kATSUByCharacter = 0;
>>>>>>> origin/fixes_2.4

  {
   * Like kATSUByCharacter, but the cursor will treat ligatures as
   * single entities.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUByTypographicCluster = 1;
=======
  kATSUByTypographicCluster     = 1;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUByTypographicCluster     = 1;
>>>>>>> origin/fixes_2_2
=======
	kATSUByTypographicCluster = 1;
>>>>>>> origin/fixes_2.4

  {
   * Cursor movement by whole words.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUByWord = 2;
=======
  kATSUByWord                   = 2;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUByWord                   = 2;
>>>>>>> origin/fixes_2_2
=======
	kATSUByWord = 2;
>>>>>>> origin/fixes_2.4

  {
   * Cursor movement by clusters based on characters only.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUByCharacterCluster = 3;
=======
  kATSUByCharacterCluster       = 3;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUByCharacterCluster       = 3;
>>>>>>> origin/fixes_2_2
=======
	kATSUByCharacterCluster = 3;
>>>>>>> origin/fixes_2.4

  {
   * Obsolete name for kATSUByTypographicCluster; do not use.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUByCluster = 1;
=======
  kATSUByCluster                = 1;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUByCluster                = 1;
>>>>>>> origin/fixes_2_2
=======
	kATSUByCluster = 1;
>>>>>>> origin/fixes_2.4


{
 *  ATSULineTruncation
 *  
 *  Summary:
 *    Constants used with the kATSULineTruncationTag layout and line
 *    control.
 *  
 *  Discussion:
 *    The constants kATSUTruncateNone, kATSUTruncateStart,
 *    kATSUTruncateEnd, and kATSUTruncateMiddle represent different
 *    places in the text where glyphs should be replaced with an
 *    elipsis should the text not fit within the width set by the
 *    kATSULineWidthTag line and layout control. The constant
 *    kATSUTruncFeatNoSquishing is special and can be bitwise OR'd with
 *    any of the other constants. It indicates that ATSUI should not
 *    perform negative justification to make the text fit. This can be
 *    desirable for situations such as live resize, to prevent the text
 *    from "wiggling".
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
	ATSULineTruncation = UInt32;
const
	kATSUTruncateNone = 0;
	kATSUTruncateStart = 1;
	kATSUTruncateEnd = 2;
	kATSUTruncateMiddle = 3;
	kATSUTruncateSpecificationMask = $00000007;
	kATSUTruncFeatNoSquishing = $00000008;
=======
=======
>>>>>>> origin/fixes_2_2
type ATSULineTruncation = UInt32;
const
  kATSUTruncateNone             = 0;
  kATSUTruncateStart            = 1;
  kATSUTruncateEnd              = 2;
  kATSUTruncateMiddle           = 3;
  kATSUTruncateSpecificationMask = $00000007;
  kATSUTruncFeatNoSquishing     = $00000008;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
type
	ATSULineTruncation = UInt32;
const
	kATSUTruncateNone = 0;
	kATSUTruncateStart = 1;
	kATSUTruncateEnd = 2;
	kATSUTruncateMiddle = 3;
	kATSUTruncateSpecificationMask = $00000007;
	kATSUTruncFeatNoSquishing = $00000008;
>>>>>>> origin/fixes_2.4


{
 *  ATSUStyleLineCountType
 *  
 *  Discussion:
 *    ATSUStyleLineCountType is used to designate how many lines will
 *    be drawn for a given style type.  Currently only the underline
 *    and strikethrough styles support this type.
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
	ATSUStyleLineCountType = UInt16;
const
	kATSUStyleSingleLineCount = 1;
	kATSUStyleDoubleLineCount = 2;
=======
=======
>>>>>>> origin/fixes_2_2
type ATSUStyleLineCountType = UInt16;
const
  kATSUStyleSingleLineCount     = 1;
  kATSUStyleDoubleLineCount     = 2;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
type
	ATSUStyleLineCountType = UInt16;
const
	kATSUStyleSingleLineCount = 1;
	kATSUStyleDoubleLineCount = 2;
>>>>>>> origin/fixes_2.4


{
 *  ATSUVerticalCharacterType
 *  
 *  Discussion:
 *    Use these constants along with the kATSUVerticalCharacterTag
 *    layout control to determine whether the vertical or horizontal
 *    forms of glyphs should be used. Note that this is independent of
 *    line rotation.
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
	ATSUVerticalCharacterType = UInt16;
const
	kATSUStronglyHorizontal = 0;
	kATSUStronglyVertical = 1;
=======
=======
>>>>>>> origin/fixes_2_2
type ATSUVerticalCharacterType = UInt16;
const
  kATSUStronglyHorizontal       = 0;
  kATSUStronglyVertical         = 1;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
type
	ATSUVerticalCharacterType = UInt16;
const
	kATSUStronglyHorizontal = 0;
	kATSUStronglyVertical = 1;
>>>>>>> origin/fixes_2.4


{
 *  ATSUStyleComparison
 *  
 *  Discussion:
 *    ATSUStyleComparison is an enumeration with four values, and is
 *    used by ATSUCompareStyles() to indicate if the first style
 *    parameter contains as a proper subset, is equal to, or is
 *    contained by the second style parameter.
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
	ATSUStyleComparison = UInt16;
const
	kATSUStyleUnequal = 0;
	kATSUStyleContains = 1;
	kATSUStyleEquals = 2;
	kATSUStyleContainedBy = 3;
=======
=======
>>>>>>> origin/fixes_2_2
type ATSUStyleComparison = UInt16;
const
  kATSUStyleUnequal             = 0;
  kATSUStyleContains            = 1;
  kATSUStyleEquals              = 2;
  kATSUStyleContainedBy         = 3;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
type
	ATSUStyleComparison = UInt16;
const
	kATSUStyleUnequal = 0;
	kATSUStyleContains = 1;
	kATSUStyleEquals = 2;
	kATSUStyleContainedBy = 3;
>>>>>>> origin/fixes_2.4


{
 *  ATSUFontFallbackMethod
 *  
 *  Discussion:
 *    ATSUFontFallbackMethod type defines the method by which ATSUI
 *    will try to find an appropriate font for a character if the
 *    assigned font does not contain the needed glyph(s) to represent
 *    it.  This affects ATSUMatchFontsToText and font selection during
 *    layout and drawing when ATSUSetTransientFontMatching is set ON.
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
	ATSUFontFallbackMethod = UInt16;
const
{
   * When this constant is specified, all fonts on the system are
   * searched for substitute glyphs.
   }
	kATSUDefaultFontFallbacks = 0;
=======
=======
>>>>>>> origin/fixes_2_2
type ATSUFontFallbackMethod = UInt16;
=======
type
	ATSUFontFallbackMethod = UInt16;
>>>>>>> origin/fixes_2.4
const
{
   * When this constant is specified, all fonts on the system are
   * searched for substitute glyphs.
   }
<<<<<<< HEAD
  kATSUDefaultFontFallbacks     = 0;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kATSUDefaultFontFallbacks = 0;
>>>>>>> origin/fixes_2.4

  {
   * This constant specifies that only the special last resort font be
   * used for substitute glyphs.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSULastResortOnlyFallback = 1;
=======
  kATSULastResortOnlyFallback   = 1;
>>>>>>> graemeg/fixes_2_2
=======
  kATSULastResortOnlyFallback   = 1;
>>>>>>> origin/fixes_2_2
=======
	kATSULastResortOnlyFallback = 1;
>>>>>>> origin/fixes_2.4

  {
   * This constant specifies that a font list you provide should be
   * sequentially searched for substitute glyphs before the ATSUI
   * searches through all available fonts on the system. You specify
   * this list through the iFonts parameter to the
   * ATSUSetObjFontFallbacks function.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUSequentialFallbacksPreferred = 2;
=======
  kATSUSequentialFallbacksPreferred = 2;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUSequentialFallbacksPreferred = 2;
>>>>>>> origin/fixes_2_2
=======
	kATSUSequentialFallbacksPreferred = 2;
>>>>>>> origin/fixes_2.4

  {
   * This constants specifies that only the font list you provide
   * should be sequentially searched for substitute glyphs. All other
   * fonts on the system are ignored, except for the special last
   * resort font. You specify the list of fonts you want ATSUI to use
   * by passing it to the iFonts parameter of the
   * ATSUSetObjFontFallbacks function.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUSequentialFallbacksExclusive = 3;
=======
  kATSUSequentialFallbacksExclusive = 3;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUSequentialFallbacksExclusive = 3;
>>>>>>> origin/fixes_2_2
=======
	kATSUSequentialFallbacksExclusive = 3;
>>>>>>> origin/fixes_2.4


{
 *  ATSUTabType
 *  
 *  Discussion:
 *    ATSUTabType type defines the characteristic of ATSUI tabs. A Left
 *    tab type specifies that the left side of affected text is to be
 *    maintained flush against the tab stop.  A Right tab type
 *    specifies that the right side of affected text is to be
 *    maintained flush against the tab stop.  A Center tab type
 *    specifies that the affected text centered about the tab stop. A
 *    Decimal tab type specifies that the affected text will be aligned
 *    such that the decimal character will by flush against the tab
 *    stop.  The default decimal character is defined in System
 *    Preferences.  A different decimal character can be defined by
 *    using the ATSUAttributeTag kATSULineDecimalTabCharacterTag.
 }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
	ATSUTabType = UInt16;
const
	kATSULeftTab = 0;
	kATSUCenterTab = 1;
	kATSURightTab = 2;
	kATSUDecimalTab = 3;
	kATSUNumberTabTypes = 4;
=======
=======
>>>>>>> origin/fixes_2_2
type ATSUTabType = UInt16;
const
  kATSULeftTab                  = 0;
  kATSUCenterTab                = 1;
  kATSURightTab                 = 2;
  kATSUDecimalTab               = 3;
  kATSUNumberTabTypes           = 4;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
type
	ATSUTabType = UInt16;
const
	kATSULeftTab = 0;
	kATSUCenterTab = 1;
	kATSURightTab = 2;
	kATSUDecimalTab = 3;
	kATSUNumberTabTypes = 4;
>>>>>>> origin/fixes_2.4


{
 *  ATSUTab
 *  
 *  Discussion:
 *    ATSUTab specifies the position and type of tab stop to be applied
 *    to a ATSUTextLayout set through the ATSUI routine ATSUSetTabArray
 *    and returned through ATSUGetTabArray.
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
	ATSUTab = record
		tabPosition: ATSUTextMeasurement;
		tabType: ATSUTabType;
=======
=======
>>>>>>> origin/fixes_2_2
type 
	ATSUTab = record
  	tabPosition: ATSUTextMeasurement;
	  tabType: ATSUTabType;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
type
	ATSUTab = record
		tabPosition: ATSUTextMeasurement;
		tabType: ATSUTabType;
>>>>>>> origin/fixes_2.4
	end;
	ATSUTabPtr = ^ATSUTab;

{
 *  ATSURGBAlphaColor
 *  
 *  Discussion:
 *    Use this structure with the kATSURGBAlphaColorTag attribute to
 *    specify color for your text in an ATSUStyle. All values range
 *    from 0.0 to 1.0.
 }
type
	ATSURGBAlphaColor = record
		red: Float32;
		green: Float32;
		blue: Float32;
		alpha: Float32;
	end;
	ATSURGBAlphaColorPtr = ^ATSURGBAlphaColor;

{
 *  GlyphCollection
 *  
 *  Discussion:
 *    GlyphCollection types represent the specific character
 *    collection.  If the value is zero, kGlyphCollectionGID, then this
 *    indicates that the glyph value represents the actual glyphID of a
 *    specific font. Adobe collections are based on CID, rather than
 *    glyph ID.
 }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
	GlyphCollection = UInt16;
const
	kGlyphCollectionGID = 0;
	kGlyphCollectionAdobeCNS1 = 1;
	kGlyphCollectionAdobeGB1 = 2;
	kGlyphCollectionAdobeJapan1 = 3;
	kGlyphCollectionAdobeJapan2 = 4;
	kGlyphCollectionAdobeKorea1 = 5;
	kGlyphCollectionUnspecified = $FF;
=======
=======
>>>>>>> origin/fixes_2_2
type GlyphCollection = UInt16;
const
  kGlyphCollectionGID           = 0;
  kGlyphCollectionAdobeCNS1     = 1;
  kGlyphCollectionAdobeGB1      = 2;
  kGlyphCollectionAdobeJapan1   = 3;
  kGlyphCollectionAdobeJapan2   = 4;
  kGlyphCollectionAdobeKorea1   = 5;
  kGlyphCollectionUnspecified   = $FF;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
type
	GlyphCollection = UInt16;
const
	kGlyphCollectionGID = 0;
	kGlyphCollectionAdobeCNS1 = 1;
	kGlyphCollectionAdobeGB1 = 2;
	kGlyphCollectionAdobeJapan1 = 3;
	kGlyphCollectionAdobeJapan2 = 4;
	kGlyphCollectionAdobeKorea1 = 5;
	kGlyphCollectionUnspecified = $FF;
>>>>>>> origin/fixes_2.4


{
 *  ATSUGlyphSelector
 *  
 *  Discussion:
 *    ATSUGlyphSelector can direct ATSUI to use a specific glyph
 *    instead of the one that ATSUI normally derives.  The glyph can be
 *    specified either as a glyphID (specific to the font used) or CID
 *    from a specfic collection defined by the collection entry.
 }
type
	ATSUGlyphSelector = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{
=======

  {
>>>>>>> graemeg/fixes_2_2
=======

  {
>>>>>>> origin/fixes_2_2
=======
{
>>>>>>> origin/fixes_2.4
   * A glyph collection constant. See the definition of GlyphCollection
   * for possible values for this field.
   }
		collection: GlyphCollection;

  {
   * The glyph ID of the glyph (when collection is
   * kGlyphCollectionGID). For Adobe glyph collections, this value
   * represents a CID
   }
		glyphID: GlyphID_fix;
	end;
	ATSUGlyphSelectorPtr = ^ATSUGlyphSelector;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
type ATSUCustomAllocFunc = function( refCon: UnivPtr; howMuch: ByteCount ): Ptr;
type ATSUCustomFreeFunc = procedure( refCon: UnivPtr; doomedBlock: UnivPtr );
type ATSUCustomGrowFunc = function( refCon: UnivPtr; oldBlock: UnivPtr; oldSize: ByteCount; newSize: ByteCount ): Ptr;

{
 *  ATSUMemoryCallbacks
 *  
 *  Discussion:
 *    ATSUMemoryCallbacks is a union struct that allows the ATSUI
 *    client to specify a specific heap for ATSUI use or allocation
 *    callbacks of which ATSUI is to use each time ATSUI performs a
 *    memory operation (alloc, grow, free).
 }
type
	ATSUMemoryCallbacks = record
		case SInt16 of
			1: (
				callbacks: record
					Alloc: ATSUCustomAllocFunc;
					Free: ATSUCustomFreeFunc;
					Grow: ATSUCustomGrowFunc;
					memoryRefCon: Ptr;
				end;
			);
			2: (
				heapToUse: THz;
			);
	end;
	ATSUMemoryCallbacksPtr = ^ATSUMemoryCallbacks;

{
 *  ATSUHeapSpec
 *  
 *  Discussion:
 *    ATSUHeapSpec provides the ATSUI client a means of specifying the
 *    heap from which ATSUI should allocate its dynamic memory or
 *    specifying that ATSUI should use the memory callback provided by
 *    the client.
 }
type ATSUHeapSpec = UInt16;
const
  kATSUUseCurrentHeap           = 0;
  kATSUUseAppHeap               = 1;
  kATSUUseSpecificHeap          = 2;
  kATSUUseCallbacks             = 3;


{
 *  ATSUMemorySetting
 *  
 *  Discussion:
 *    ATSUMemorySetting is used to store the results from a
 *    ATSUSetMemoryAlloc or a ATSUGetCurrentMemorySetting call.  It can
 *    also be used to change the current ATSUMemorySetting by passing
 *    it into the ATSUSetCurrentMemorySetting call.
 }
type
	ATSUMemorySetting    = ^SInt32; { an opaque 32-bit type }


<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{
 *  ATSUGlyphInfo
 *  
 *  Summary:
 *    Structure returned by ATSUGetGlyphInfo
 *  
 *  Discussion:
 *    ATSUGetGlyphInfo will return an array of these structs, one for
 *    each glyph in the specified range. You can then make changes to
 *    the data before drawing it with ATSUDrawGlyphInfo. These
 *    functions are no longer recommended; see ATSUnicodeDirectAccess.h
 *    for replacement functions.
 }
type
	ATSUGlyphInfo = record
		glyphID: GlyphID_fix;
		reserved: UInt16;
		layoutFlags: UInt32;
		charIndex: UniCharArrayOffset;
		style: ATSUStyle;
		deltaY: Float32;
		idealX: Float32;
		screenX: SInt16;
		caretX: SInt16;
	end;
	ATSUGlyphInfoPtr = ^ATSUGlyphInfo;

{
 *  ATSUGlyphInfoArray
 *  
 *  Summary:
 *    Structure returned by ATSUGetGlyphInfo
 *  
 *  Discussion:
 *    This data structure is returned by ATSUGetGlyphInfo. layout is
 *    the same layout you pass in to ATSUGetGlyphInfo, numGlyphs is the
 *    number of glyphs stored in the array glyphs. See the definition
 *    of ATSUGlyphInfo for more information about the data structures
 *    contained in the glyphs array. The ATSUGetGlyphInfo function is
 *    no longer recommended; see ATSUnicodeDirectAccess.h for
 *    replacement functions.
 }
type
	ATSUGlyphInfoArray = record
		layout: ATSUTextLayout;
		numGlyphs: ItemCount;
  	glyphs: array[0..0] of ATSUGlyphInfo;
	end;
	ATSUGlyphInfoArrayPtr = ^ATSUGlyphInfoArray;
{*******************************************************************************}
{  ATSUI highlighting method constants and typedefs                             }
{*******************************************************************************}

{
 *  ATSUHighlightMethod
 *  
 *  Discussion:
 *    Use the constants with the function ATSUSetHighlightingMethod to
 *    determine the method of highlighting to use. kInvertHighlighting
 *    will cause ATSUI to perform a simple color inversion on the area
 *    around the text. Although this method requires the least work, it
 *    does not produce the best visual results. kRedrawHighlighting
 *    will cause ATSUI to redraw the text whenever highlighting or
 *    unhighlighting it. This method produces the best visual results,
 *    but it does require you to specify a way for ATSUI to redraw the
 *    background behind the text after an unhighlight operation. See
 *    the definitions of ATSUUnhighlightData, ATSUBackgroundData,
 *    ATSUBackgroundDataType, and RedrawBackgroundProcPtr for more
 *    information.
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
	ATSUHighlightMethod = UInt32;
const
	kInvertHighlighting = 0;
	kRedrawHighlighting = 1;
=======
=======
>>>>>>> origin/fixes_2_2
type ATSUHighlightMethod = UInt32;
const
  kInvertHighlighting           = 0;
  kRedrawHighlighting           = 1;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
type
	ATSUHighlightMethod = UInt32;
const
	kInvertHighlighting = 0;
	kRedrawHighlighting = 1;
>>>>>>> origin/fixes_2.4


{
 *  ATSUBackgroundDataType
 *  
 *  Discussion:
 *    Use these constants for the dataType field in the
 *    ATSUUnhighlightData structure. kATSUBackgroundColor refers to a
 *    solid color background, while kATSUBackgroundCallback refers to a
 *    redrawing callback function. Note that if you specify
 *    kATSUBackgroundCallback, you must provide a callback function.
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
	ATSUBackgroundDataType = UInt32;
const
	kATSUBackgroundColor = 0;
	kATSUBackgroundCallback = 1;
=======
=======
>>>>>>> origin/fixes_2_2
type ATSUBackgroundDataType = UInt32;
const
  kATSUBackgroundColor          = 0;
  kATSUBackgroundCallback       = 1;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
type
	ATSUBackgroundDataType = UInt32;
const
	kATSUBackgroundColor = 0;
	kATSUBackgroundCallback = 1;
>>>>>>> origin/fixes_2.4


{
 *  ATSUBackgroundColor
 *  
 *  Discussion:
 *    A background color used by ATSUI to redraw the background after a
 *    call to ATSUUnhighlight text when the highlighting method is set
 *    to kRedrawHighlighting. See the definitions of ATSUBackgroundData
 *    and ATSUUnhighlightData for more information.
 }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
	ATSUBackgroundColor = ATSURGBAlphaColor;
=======
type ATSUBackgroundColor = ATSURGBAlphaColor;
>>>>>>> graemeg/fixes_2_2
=======
type ATSUBackgroundColor = ATSURGBAlphaColor;
>>>>>>> origin/fixes_2_2
=======
type
	ATSUBackgroundColor = ATSURGBAlphaColor;
>>>>>>> origin/fixes_2.4

{
 *  RedrawBackgroundProcPtr
 *  
 *  Discussion:
 *    RedrawBackgroundProcPtr is a pointer to a client-supplied
 *    callback function (e.g. MyRedrawBackgroundProc) for redrawing
 *    complex backgrounds (and optionally the text as well) that can be
 *    called by ATSUI for highlighting if the client has called
 *    ATSUSetHighlightingMethod with kRedrawHighlighting for the
 *    iMethod parameter. In order for ATSUI to call the client
 *    function, the client must (1) pass a pointer to the client
 *    function to NewRedrawBackgroundUPP() in order to obtain a
 *    RedrawBackgroundUPP, and (2) pass the RedrawBackgroundUPP in the
 *    unhighlightData.backgroundUPP field of the iUnhighlightData
 *    parameter for the ATSUSetHighlightingMethod call. When finished,
 *    the client should call DisposeRedrawBackgroundUPP with the
 *    RedrawBackgroundUPP.
 *  
 *  Parameters:
 *    
 *    iLayout:
 *      The layout to which the highlighting is being applied. The
 *      client function can use this to redraw the text.
 *    
 *    iTextOffset:
 *      The offset of the text that is being highlighted; can be used
 *      by the client function to redraaw the text.
 *    
 *    iTextLength:
 *      The length of the text that is being highlighted; can be used
 *      by the client function to redraaw the text.
 *    
 *    iUnhighlightArea:
 *      An array of ATSTrapezoids that describes the highlight area.
 *      The ATSTrapezoid array is ALWAYS in QD coordinates.
 *    
 *    iTrapezoidCount:
 *      The count of ATSTrapezoids in iUnhighlightArea.
 *  
 *  Result:
 *    A Boolean result indicating whether ATSUI should redraw the text.
 *    If the client function redraws the text, it should return false,
 *    otherwise it should return true to have ATSUI redraw any text
 *    that needs to be redrawn.
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
	RedrawBackgroundProcPtr = function( iLayout: ATSUTextLayout; iTextOffset: UniCharArrayOffset; iTextLength: UniCharCount; iUnhighlightArea: {variable-size-array} ATSTrapezoidPtr; iTrapezoidCount: ItemCount ): Boolean;
	RedrawBackgroundUPP = RedrawBackgroundProcPtr;
=======
=======
>>>>>>> origin/fixes_2_2
type RedrawBackgroundProcPtr = function( iLayout: ATSUTextLayout; iTextOffset: UniCharArrayOffset; iTextLength: UniCharCount; iUnhighlightArea: ATSTrapezoidPtr; iTrapezoidCount: ItemCount ): Boolean;
// typedef STACK_UPP_TYPE(RedrawBackgroundProcPtr)                 RedrawBackgroundUPP;
// Beats me what this translates to.  If someone finds out they can tell me and we'll update it
type RedrawBackgroundUPP = Ptr;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
type
	RedrawBackgroundProcPtr = function( iLayout: ATSUTextLayout; iTextOffset: UniCharArrayOffset; iTextLength: UniCharCount; iUnhighlightArea: {variable-size-array} ATSTrapezoidPtr; iTrapezoidCount: ItemCount ): Boolean;
	RedrawBackgroundUPP = RedrawBackgroundProcPtr;
>>>>>>> origin/fixes_2.4
{
 *  NewRedrawBackgroundUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function NewRedrawBackgroundUPP( userRoutine: RedrawBackgroundProcPtr ): RedrawBackgroundUPP; external name '_NewRedrawBackgroundUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function NewRedrawBackgroundUPP( userRoutine: RedrawBackgroundProcPtr ): RedrawBackgroundUPP; external name '_NewRedrawBackgroundUPP';
>>>>>>> graemeg/fixes_2_2
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function NewRedrawBackgroundUPP( userRoutine: RedrawBackgroundProcPtr ): RedrawBackgroundUPP; external name '_NewRedrawBackgroundUPP';
>>>>>>> origin/fixes_2_2
=======
function NewRedrawBackgroundUPP( userRoutine: RedrawBackgroundProcPtr ): RedrawBackgroundUPP; external name '_NewRedrawBackgroundUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4

{
 *  DisposeRedrawBackgroundUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure DisposeRedrawBackgroundUPP( userUPP: RedrawBackgroundUPP ); external name '_DisposeRedrawBackgroundUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
procedure DisposeRedrawBackgroundUPP( userUPP: RedrawBackgroundUPP ); external name '_DisposeRedrawBackgroundUPP';
>>>>>>> graemeg/fixes_2_2
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
procedure DisposeRedrawBackgroundUPP( userUPP: RedrawBackgroundUPP ); external name '_DisposeRedrawBackgroundUPP';
>>>>>>> origin/fixes_2_2
=======
procedure DisposeRedrawBackgroundUPP( userUPP: RedrawBackgroundUPP ); external name '_DisposeRedrawBackgroundUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4

{
 *  InvokeRedrawBackgroundUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function InvokeRedrawBackgroundUPP( iLayout: ATSUTextLayout; iTextOffset: UniCharArrayOffset; iTextLength: UniCharCount; iUnhighlightArea: {variable-size-array} ATSTrapezoidPtr; iTrapezoidCount: ItemCount; userUPP: RedrawBackgroundUPP ): Boolean; external name '_InvokeRedrawBackgroundUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function InvokeRedrawBackgroundUPP( iLayout: ATSUTextLayout; iTextOffset: UniCharArrayOffset; iTextLength: UniCharCount; iUnhighlightArea: ATSTrapezoidPtr; iTrapezoidCount: ItemCount; userUPP: RedrawBackgroundUPP ): Boolean; external name '_InvokeRedrawBackgroundUPP';
>>>>>>> graemeg/fixes_2_2
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function InvokeRedrawBackgroundUPP( iLayout: ATSUTextLayout; iTextOffset: UniCharArrayOffset; iTextLength: UniCharCount; iUnhighlightArea: ATSTrapezoidPtr; iTrapezoidCount: ItemCount; userUPP: RedrawBackgroundUPP ): Boolean; external name '_InvokeRedrawBackgroundUPP';
>>>>>>> origin/fixes_2_2
=======
function InvokeRedrawBackgroundUPP( iLayout: ATSUTextLayout; iTextOffset: UniCharArrayOffset; iTextLength: UniCharCount; iUnhighlightArea: {variable-size-array} ATSTrapezoidPtr; iTrapezoidCount: ItemCount; userUPP: RedrawBackgroundUPP ): Boolean; external name '_InvokeRedrawBackgroundUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4


{
 *  ATSUBackgroundData
 *  
 *  Summary:
 *    Data describing one of two methods for ATSUI to unhighlight text.
 *  
 *  Discussion:
 *    When you call ATSUUnhighlightText and the highlighting method
 *    used is kRedrawHighlighting, ATSUI must redraw the text,
 *    including the background, when unhighlighting. The two ways of
 *    doing this are by telling ATSUI to (1) use a solid color to
 *    repaint the background (2) use a callback function to repaint the
 *    background. This union provides that information to ATSUI. See
 *    the definitions of ATSUUnhighlightData, RedrawBackgroundProcPtr,
 *    and ATSUSetHighlightingMethod for more information.
 }
type
	ATSUBackgroundData = record
		case SInt16 of
			1: (
				{
				 * Specifies a color for ATSUI to repaint the background.
				 }
					backgroundColor: ATSUBackgroundColor;
			);
			2: (
				{
				 * Specifies a Universal Procedure Pointer for ATSUI to call to
				 * redraw the background.
				 }
					backgroundUPP: RedrawBackgroundUPP;
			);
	end;
	ATSUBackgroundDataPtr = ^ATSUBackgroundData;

{
 *  ATSUUnhighlightData
 *  
 *  Summary:
 *    Struct for defining a method for ATSUI to unhighlight text.
 *  
 *  Discussion:
 *    There are two methods of highlighting available on Mac OS X:
 *    invert and redraw. For the invert method, no unhighlight method
 *    needs to be specified. ATSUI will simply higlight text by
 *    performing a color inversion on the area surrounding the test.
 *    However, for best results, the redraw method is perferred. With
 *    this method, ATSUI will redraw text with a new background when
 *    highlighting, and redraw it again when unhighlighting. When using
 *    the redraw method, ATSUI needs to know how to restore the
 *    backround when unhighlighting text. That is where the unhighlight
 *    data comes in. This struct tells ATSUI how to restore the
 *    background after a highlight. There are two methods for
 *    specifying this information to ATSUI. One is by specifying a
 *    solid color, the other by providing a callback for redrawing part
 *    of the background.
 }
type
	ATSUUnhighlightData = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{
=======

  {
>>>>>>> graemeg/fixes_2_2
=======

  {
>>>>>>> origin/fixes_2_2
=======
{
>>>>>>> origin/fixes_2.4
   * Determines which method to use for restoring the background after
   * a highlight; solid color (kATSUBackgroundColor), or drawing
   * callback (kATSUBackgroundCallback). See also the definition of
   * ATSUBackgroundDataType.
   }
		dataType: ATSUBackgroundDataType;

  {
   * This union provides the actual data for ATSUI to use when
   * redrawing the background. See the definition of ATSUBackgroundData
   * for more information.
   }
		unhighlightData: ATSUBackgroundData;
	end;
	ATSUUnhighlightDataPtr = ^ATSUUnhighlightData;
{******************************************************************************}
{ Other ATSUI constants                                                        }
{******************************************************************************}

{
 *  Summary:
 *    Line direction types
 *  
 *  Discussion:
 *    These constants are used with the kATSULineDirectionTag control
 *    to determine overall line direction.
 }
const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{
   * Imposes left-to-right or top-to-bottom dominant direction.
   }
	kATSULeftToRightBaseDirection = 0;
=======
=======
>>>>>>> origin/fixes_2_2
  {
   * Imposes left-to-right or top-to-bottom dominant direction.
   }
  kATSULeftToRightBaseDirection = 0;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{
   * Imposes left-to-right or top-to-bottom dominant direction.
   }
	kATSULeftToRightBaseDirection = 0;
>>>>>>> origin/fixes_2.4

  {
   * Impose right-to-left or bottom-to-top dominant direction.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSURightToLeftBaseDirection = 1;
=======
  kATSURightToLeftBaseDirection = 1;
>>>>>>> graemeg/fixes_2_2
=======
  kATSURightToLeftBaseDirection = 1;
>>>>>>> origin/fixes_2_2
=======
	kATSURightToLeftBaseDirection = 1;
>>>>>>> origin/fixes_2.4

const
	kATSUStartAlignment    = Fract($00000000);
	kATSUEndAlignment      = Fract($40000000);
	kATSUCenterAlignment   = Fract($20000000);
	kATSUNoJustification   = Fract($00000000);
	kATSUFullJustification = Fract($40000000);

{
 *  Summary:
 *    This constant will be returned from ATSUFindFontFromName if no
 *    valid font can be found. If you pass this constant to
 *    ATSUSetAttributes, it will produce an error.
 }
const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUInvalidFontID = 0;
=======
  kATSUInvalidFontID            = 0;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUInvalidFontID            = 0;
>>>>>>> origin/fixes_2_2
=======
	kATSUInvalidFontID = 0;
>>>>>>> origin/fixes_2.4


{
 *  Summary:
 *    Pass this constant to line breaking functions (i.e.,
 *    ATSUBreakLine, ATSUBatchBreakLines) if you have already set a
 *    line width as a layout control via the kATSULineWidthTag
 *    attribute.
 }
const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUUseLineControlWidth = $7FFFFFFF;
=======
  kATSUUseLineControlWidth      = $7FFFFFFF;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUUseLineControlWidth      = $7FFFFFFF;
>>>>>>> origin/fixes_2_2
=======
	kATSUUseLineControlWidth = $7FFFFFFF;
>>>>>>> origin/fixes_2.4


{
 *  Summary:
 *    Pass this constant to the iSelector parameter of the
 *    ATSUGetFontFeatureNameCode function if you wish to obtain the
 *    name code for a feature type rather than a feature selector.
 }
const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUNoSelector = $0000FFFF;
=======
  kATSUNoSelector               = $0000FFFF;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUNoSelector               = $0000FFFF;
>>>>>>> origin/fixes_2_2
=======
	kATSUNoSelector = $0000FFFF;
>>>>>>> origin/fixes_2.4


{
 *  Summary:
 *    Text buffer convenience constants.
 *  
 *  Discussion:
 *    These constants refer to the beginning and end of a text buffer.
 *    Functions which accept these constants are marked below. Do not
 *    pass these constants to functions which do not explicity state
 *    they will accept them.
 }
const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{
   * Refers to the beginning of a text buffer.
   }
	kATSUFromTextBeginning = $FFFFFFFF;
=======
=======
>>>>>>> origin/fixes_2_2

  {
   * Refers to the beginning of a text buffer.
   }
  kATSUFromTextBeginning        = $FFFFFFFF;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{
   * Refers to the beginning of a text buffer.
   }
	kATSUFromTextBeginning = $FFFFFFFF;
>>>>>>> origin/fixes_2.4

  {
   * Refers to the end of a text buffer.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUToTextEnd = $FFFFFFFF;
=======
  kATSUToTextEnd                = $FFFFFFFF;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUToTextEnd                = $FFFFFFFF;
>>>>>>> origin/fixes_2_2
=======
	kATSUToTextEnd = $FFFFFFFF;
>>>>>>> origin/fixes_2.4

  {
   * Used for bidi cursor movement between paragraphs.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUFromPreviousLayout = $FFFFFFFE;
=======
  kATSUFromPreviousLayout       = $FFFFFFFE;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUFromPreviousLayout       = $FFFFFFFE;
>>>>>>> origin/fixes_2_2
=======
	kATSUFromPreviousLayout = $FFFFFFFE;
>>>>>>> origin/fixes_2.4

  {
   * Used for bidi cursor movement between paragraphs.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUFromFollowingLayout = $FFFFFFFD;
=======
  kATSUFromFollowingLayout      = $FFFFFFFD;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUFromFollowingLayout      = $FFFFFFFD;
>>>>>>> origin/fixes_2_2
=======
	kATSUFromFollowingLayout = $FFFFFFFD;
>>>>>>> origin/fixes_2.4


{
 *  Summary:
 *    Other convenience constants.
 }
const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{
=======

  {
>>>>>>> graemeg/fixes_2_2
=======

  {
>>>>>>> origin/fixes_2_2
=======
{
>>>>>>> origin/fixes_2.4
   * Pass this constant to functions that require a set of coordinates
   * (i.e., ATSUDrawText, ATSUHighlightText) if you want ATSUI to use
   * the current Quickdraw graphics port pen location.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUUseGrafPortPenLoc = $FFFFFFFF;
=======
  kATSUUseGrafPortPenLoc        = $FFFFFFFF;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUUseGrafPortPenLoc        = $FFFFFFFF;
>>>>>>> origin/fixes_2_2
=======
	kATSUUseGrafPortPenLoc = $FFFFFFFF;
>>>>>>> origin/fixes_2.4

  {
   * Pass this constant to functions such as ATSUClearAttributes and
   * ATSUClearLayoutControls if you wish to clear all settings instead
   * of a specific array of settings.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
	kATSUClearAll = $FFFFFFFF;

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
<<<<<<< HEAD

end.
{$endc} {not MACOSALLINCLUDE}
=======
  kATSUClearAll                 = $FFFFFFFF;

end.
>>>>>>> graemeg/fixes_2_2
=======
  kATSUClearAll                 = $FFFFFFFF;

end.
>>>>>>> origin/fixes_2_2
=======

end.
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4
