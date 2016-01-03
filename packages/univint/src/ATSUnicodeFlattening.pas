{
     File:       QD/ATSUnicodeFlattening.h
 
     Contains:   Public interfaces for Apple Type Services for Unicode Imaging
 
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
 
     Copyright:  © 2002-2008 by Apple Inc. all rights reserved.
=======
     Version:    Quickdraw-150~1
 
     Copyright:  © 2002-2003 by Apple Computer, Inc., all rights reserved.
>>>>>>> graemeg/fixes_2_2
=======
     Version:    Quickdraw-150~1
 
     Copyright:  © 2002-2003 by Apple Computer, Inc., all rights reserved.
>>>>>>> origin/fixes_2_2
=======
     Version:    Quickdraw-262~1
 
     Copyright:  © 2002-2008 by Apple Inc. all rights reserved.
>>>>>>> origin/fixes_2.4
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}
<<<<<<< HEAD
{  Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, 2004 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
=======
{	  Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, 2004 }
{	  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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
{	  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit ATSUnicodeFlattening;
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
=======
>>>>>>> origin/cpstrnew
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
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
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
uses MacTypes,ATSUnicodeTypes,SFNTTypes;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}
<<<<<<< HEAD
=======
{$ALIGN MAC68K}
>>>>>>> graemeg/fixes_2_2
=======
{$ALIGN MAC68K}
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{ ---------------------------------------------------------------------------- }
{ Constants                                                                    }
{ ---------------------------------------------------------------------------- }
{
   ATSUFlattenedDataStreamFormat is used to inform the APIs which flatten and
   unflatten style runs exactly what type of data that they should be generating
   or parsing.
}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4

{$ALIGN MAC68K}

type
	ATSUFlattenedDataStreamFormat = UInt32;
<<<<<<< HEAD
const
	kATSUDataStreamUnicodeStyledText = FourCharCode('ustl');
=======
type ATSUFlattenedDataStreamFormat = UInt32;
const
  kATSUDataStreamUnicodeStyledText = FourCharCode('ustl');
>>>>>>> graemeg/fixes_2_2
=======
type ATSUFlattenedDataStreamFormat = UInt32;
const
  kATSUDataStreamUnicodeStyledText = FourCharCode('ustl');
>>>>>>> origin/fixes_2_2
=======
const
	kATSUDataStreamUnicodeStyledText = FourCharCode('ustl');
>>>>>>> origin/fixes_2.4


{
   ATSUFlattenStyleRunOptions is a bitfield list of options that can be passed
   into the ATSUFlattenStyleRunsToStream API. Currently, there are no options. 
   This is here for future expansion.
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
	ATSUFlattenStyleRunOptions = UInt32;
const
	kATSUFlattenOptionNoOptionsMask = $00000000;
=======
type ATSUFlattenStyleRunOptions = UInt32;
const
  kATSUFlattenOptionNoOptionsMask = $00000000;
>>>>>>> graemeg/fixes_2_2
=======
type ATSUFlattenStyleRunOptions = UInt32;
const
  kATSUFlattenOptionNoOptionsMask = $00000000;
>>>>>>> origin/fixes_2_2
=======
type
	ATSUFlattenStyleRunOptions = UInt32;
const
	kATSUFlattenOptionNoOptionsMask = $00000000;
>>>>>>> origin/fixes_2.4

{
   ATSUUnFlattenStyleRunOptions is a bitfield list of options that can be passed
   into the ATSUUnFlattenStyleRunsToStream API. Currently, there are no options. 
   This is here for future expansion.
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
	ATSUUnFlattenStyleRunOptions = UInt32;
const
	kATSUUnFlattenOptionNoOptionsMask = $00000000;
=======
type ATSUUnFlattenStyleRunOptions = UInt32;
const
  kATSUUnFlattenOptionNoOptionsMask = $00000000;
>>>>>>> graemeg/fixes_2_2
=======
type ATSUUnFlattenStyleRunOptions = UInt32;
const
  kATSUUnFlattenOptionNoOptionsMask = $00000000;
>>>>>>> origin/fixes_2_2
=======
type
	ATSUUnFlattenStyleRunOptions = UInt32;
const
	kATSUUnFlattenOptionNoOptionsMask = $00000000;
>>>>>>> origin/fixes_2.4


{ ---------------------------------------------------------------------------- }
{ Data Types                                                                   }
{ ---------------------------------------------------------------------------- }

{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   ATSUStyleRunInfo is a structure that contains an index into an array of 
=======
   ATSUStyleRunInfo is a structure that contrains an index into an array of 
>>>>>>> graemeg/fixes_2_2
=======
   ATSUStyleRunInfo is a structure that contrains an index into an array of 
>>>>>>> origin/fixes_2_2
=======
   ATSUStyleRunInfo is a structure that contains an index into an array of 
>>>>>>> origin/fixes_2.4
   unique ATSUStyle objects as well as the length of the run that the style run 
   object covers. This structure is utilized by ATSUUnflattenStyleRunsFromStream() 
   to return the style run info to the caller. 
}
type
	ATSUStyleRunInfo = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		runLength: UInt32;
		styleObjectIndex: UInt32;
=======
		runLength: UniCharCount;
		styleObjectIndex: ItemCount;
>>>>>>> graemeg/fixes_2_2
=======
		runLength: UniCharCount;
		styleObjectIndex: ItemCount;
>>>>>>> origin/fixes_2_2
=======
		runLength: UInt32;
		styleObjectIndex: UInt32;
>>>>>>> origin/fixes_2.4
	end;
	ATSUStyleRunInfoPtr = ^ATSUStyleRunInfo;
{ ---------------------------------------------------------------------------- }
{ 'ustl' structure data structures and definitions                             }
{ ---------------------------------------------------------------------------- }
{
   The 'ustl' data structure follows this format:
   1. Main Data Structure Block Header
   2. Flattened Text Layout Data
   3. Flattened Style Run Data
   4. Flattened Style Data
   Per the 'ustl' spec, these structures should maintain four-byte alignment. 
   For things that are variable width (such as font names), padding bytes must
   be added to ensure that this alignment is always kept.
}

{
   structure versioning - the version of the 'ustl' that the ATSUI parsing
   and generating functions will handle is version 2 or greater. Earlier
   versions were not completly specified and have been obsoleted.
}
const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
	kATSFlatDataUstlVersion0 = 0;
	kATSFlatDataUstlVersion1 = 1;
	kATSFlatDataUstlVersion2 = 2;
	kATSFlatDataUstlCurrentVersion = kATSFlatDataUstlVersion2;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
  kATSFlatDataUstlVersion0      = 0;
  kATSFlatDataUstlVersion1      = 1;
  kATSFlatDataUstlVersion2      = 2;
  kATSFlatDataUstlCurrentVersion = kATSFlatDataUstlVersion2;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{ ------------------ }
{ Block 1 Structures }
{ ------------------ }

{
   This is the main data structure block header. It describes the rest
   of the data and how it is structured.
}
type
	ATSFlatDataMainHeaderBlock = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ the 'ustl' version number. This needs to be the first item in the}
=======

                                              { the 'ustl' version number. This needs to be the first item in the}
>>>>>>> graemeg/fixes_2_2
=======

                                              { the 'ustl' version number. This needs to be the first item in the}
>>>>>>> origin/fixes_2_2
=======
{ the 'ustl' version number. This needs to be the first item in the}
>>>>>>> origin/fixes_2.4
                                              { data block do as not to confuse parsers of earlier (and possibly}
                                              { later) versions of the spec *|}
		version: UInt32;

                                              { the total size of the stream in bytes, including the four bytes in}
                                              { the version above}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		sizeOfDataBlock: UInt32;

                                              { offset from the beginning of the stream to the flattened text layout data.}
                                              { This can be set to 0 if there are no text layouts stored in the stream.}
		offsetToTextLayouts: UInt32;

                                              { offset from the beginning of the stream to the flattened style run data. }
                                              { This can be set to 0 if there is no flattened style run data in the stream}
		offsetToStyleRuns: UInt32;

                                              { offset to the flattened style list data. This can be set to 0 if there}
                                              { is no flattened style list data}
		offsetToStyleList: UInt32;
=======
=======
>>>>>>> origin/fixes_2_2
		sizeOfDataBlock: ByteCount;
=======
		sizeOfDataBlock: UInt32;
>>>>>>> origin/fixes_2.4

                                              { offset from the beginning of the stream to the flattened text layout data.}
                                              { This can be set to 0 if there are no text layouts stored in the stream.}
		offsetToTextLayouts: UInt32;

                                              { offset from the beginning of the stream to the flattened style run data. }
                                              { This can be set to 0 if there is no flattened style run data in the stream}
		offsetToStyleRuns: UInt32;

                                              { offset to the flattened style list data. This can be set to 0 if there}
                                              { is no flattened style list data}
<<<<<<< HEAD
		offsetToStyleList: ByteCount;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
		offsetToStyleList: UInt32;
>>>>>>> origin/fixes_2.4
	end;
	ATSFlatDataMainHeaderBlockPtr = ^ATSFlatDataMainHeaderBlock;
{ ------------------ }
{ Block 2 Structures }
{ ------------------ }
{
   The Block 2 Structures are not currently used by any of ATSUI's internal parsing
   or packing routines. They are, however, part of the 'ustl' standard and are put
   here for developer conveniance, as well as to properly define the standard.
}

{
   This is the header that is attached to each flattened text layout. The
   number of flattened text layouts in the stucture is specified by the
   ATSFlatDataTextLayoutHeader structure that is below.
}
type
	ATSFlatDataTextLayoutDataHeader = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ the total size of this particular flattened text layout, including any}
                                              { padding bytes and such. }
		sizeOfLayoutData: UInt32;

                                              { the number of characters covered by this flattened text layout}
		textLayoutLength: UInt32;
=======
=======
>>>>>>> origin/fixes_2_2

                                              { the total size of this particular flattened text layout, including any}
=======
{ the total size of this particular flattened text layout, including any}
>>>>>>> origin/fixes_2.4
                                              { padding bytes and such. }
		sizeOfLayoutData: UInt32;

                                              { the number of characters covered by this flattened text layout}
<<<<<<< HEAD
		textLayoutLength: ByteCount;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
		textLayoutLength: UInt32;
>>>>>>> origin/fixes_2.4

                                              { the byte offset relative to the start of this structure to the flattened}
                                              { layout control data. This can be set to zero if there are no layout}
                                              { controls.}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		offsetToLayoutControls: UInt32;
=======
		offsetToLayoutControls: ByteCount;
>>>>>>> graemeg/fixes_2_2
=======
		offsetToLayoutControls: ByteCount;
>>>>>>> origin/fixes_2_2
=======
		offsetToLayoutControls: UInt32;
>>>>>>> origin/fixes_2.4

                                              { the byte offset, relative to the start of this structure to the}
                                              { flattened line info. This can be set to zero if there is no line info }
                                              { in this layout.}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		offsetToLineInfo: UInt32;
=======
		offsetToLineInfo: ByteCount;
>>>>>>> graemeg/fixes_2_2
=======
		offsetToLineInfo: ByteCount;
>>>>>>> origin/fixes_2_2
=======
		offsetToLineInfo: UInt32;
>>>>>>> origin/fixes_2.4

                                              { if the offsetToLayoutControls is non-zero, then following this block}
                                              { there will be a ATSFlattenedLayoutDataFlattenedLayoutControlsHeader}
                                              { followed by an array of ATSFlattenedLayoutDataFlattenedLayoutControls}
                                              { structures. If the offsetToLineInfo is non-zero, then following the}
                                              { flattened layout controls will be a ATSFlatDataLineInfoHeader}
                                              { structure.}
	end;
	ATSFlatDataTextLayoutDataHeaderPtr = ^ATSFlatDataTextLayoutDataHeader;
{
   This is the the main header for block 2. If there is a block 2, then there
   needs to be one of these. This structure is what the offsetToTextLayouts
   points to in block 1.
}
type
	ATSFlatDataTextLayoutHeader = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ the total number of flattened text layouts stored in this block.}
                                              { This must be non-zero, as if there were no flattened text layouts, the}
                                              { entire block 2 would not exist}
		numFlattenedTextLayouts: UInt32;
=======
=======
>>>>>>> origin/fixes_2_2

                                              { the total number of flattened text layouts stored in this block.}
                                              { This must be non-zero, as if there were no flattened text layouts, the}
                                              { entire block 2 would not exist}
		numFlattenedTextLayouts: ItemCount;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ the total number of flattened text layouts stored in this block.}
                                              { This must be non-zero, as if there were no flattened text layouts, the}
                                              { entire block 2 would not exist}
		numFlattenedTextLayouts: UInt32;
>>>>>>> origin/fixes_2.4

                                              { first of possibly many flattened text layouts. There should be one of}
                                              { these for each flattened text layout as determined by the}
                                              { numFlattenedTextLayouts above. }
		flattenedTextLayouts: array[0..0] of ATSFlatDataTextLayoutDataHeader;
	end;
	ATSFlatDataTextLayoutHeaderPtr = ^ATSFlatDataTextLayoutHeader;
{
   This is the header for the flattened layout controls structure. This is
   the structure that a non-zero offsetToLayoutControls points to in the
   ATSFlatDataTextLayoutDataHeader
}
type
	ATSFlatDataLayoutControlsDataHeader = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ the number of flattened layout controls. It is suggested that there be}
                                              { at least one layout control to output the line direction for the layout}
		numberOfLayoutControls: UInt32;
=======
=======
>>>>>>> origin/fixes_2_2

                                              { the number of flattened layout controls. It is suggested that there be}
                                              { at least one layout control to output the line direction for the layout}
		numberOfLayoutControls: ItemCount;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ the number of flattened layout controls. It is suggested that there be}
                                              { at least one layout control to output the line direction for the layout}
		numberOfLayoutControls: UInt32;
>>>>>>> origin/fixes_2.4

                                              { first of possibly many flattened layout controls. There should be one }
                                              { of these for each layout control as determined by the}
                                              { numberOfLayoutControls above. Of course, if there are no layout controls,}
                                              { then this structure shouldn't even exist. Each attribute info structure}
                                              { in the array could be followed by additional padding bytes in order}
                                              { to maintain four-byte alignment. These padding bytes are not to be}
                                              { included in the fValueSize member of each structure. }
		controlArray: array[0..0] of ATSUAttributeInfo;
	end;
	ATSFlatDataLayoutControlsDataHeaderPtr = ^ATSFlatDataLayoutControlsDataHeader;
type
	ATSFlatDataLineInfoData = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ the length of this particular line in UniChars}
		lineLength: UInt32;

                                              { the number of line controls applied to this line. This can be set}
                                              { to zero if there are no special line controls applied to this line.}
		numberOfLineControls: UInt32;
=======
=======
>>>>>>> origin/fixes_2_2

                                              { the length of this particular line in UniChars}
		lineLength: UniCharCount;

                                              { the number of line controls applied to this line. This can be set}
                                              { to zero if there are no special line controls applied to this line.}
		numberOfLineControls: ItemCount;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ the length of this particular line in UniChars}
		lineLength: UInt32;

                                              { the number of line controls applied to this line. This can be set}
                                              { to zero if there are no special line controls applied to this line.}
		numberOfLineControls: UInt32;
>>>>>>> origin/fixes_2.4

                                              { the numberOfLineControls is non-zero, then following this structure}
                                              { must be an array of ATSUAttributeInfo structures. There must be one}
                                              { ATSUAttributeInfo structure for each numberOfLineControls above.}
	end;
	ATSFlatDataLineInfoDataPtr = ^ATSFlatDataLineInfoData;
{
   This structure is the main data header for the flattened line info data. This
   is what a non-zero offsetToLineInfo points to in the 
   ATSFlatDataTextLayoutDataHeader structure above.
}
type
	ATSFlatDataLineInfoHeader = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ the number of flattened line info structures that are stored in this}
                                              { block. This value should really be equal to the number of soft line}
                                              { breaks in the layout + 1. Of course if numberOfLines is zero, then}
                                              { this structure shouldn't even be used.}
		numberOfLines: UInt32;
=======
=======
>>>>>>> origin/fixes_2_2

                                              { the number of flattened line info structures that are stored in this}
                                              { block. This value should really be equal to the number of soft line}
                                              { breaks in the layout + 1. Of course if numberOfLines is zero, then}
                                              { this structure shouldn't even be used.}
		numberOfLines: ItemCount;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ the number of flattened line info structures that are stored in this}
                                              { block. This value should really be equal to the number of soft line}
                                              { breaks in the layout + 1. Of course if numberOfLines is zero, then}
                                              { this structure shouldn't even be used.}
		numberOfLines: UInt32;
>>>>>>> origin/fixes_2.4

                                              { the first in a array of ATSFlatDataLineInfoData structures. There}
                                              { needs to be a ATSFlatDataLineInfoData for each numberOfLines}
                                              { specified above.}
		lineInfoArray: array[0..0] of ATSFlatDataLineInfoData;
	end;
	ATSFlatDataLineInfoHeaderPtr = ^ATSFlatDataLineInfoHeader;
{ ------------------ }
{ Block 3 Structures }
{ ------------------ }
{
   The block 3 structures are used by ATSUI style run flattening and parsing
   functions, ATSUFlattenStyleRunsToStream and ATSUUnflattenStyleRunsFromStream
   to represent flattened style run information. These structures go hand and
   hand with the block 4 structures.
}

{
   This is the data header that appears before the style run data structures.
   This structure is what a non-zero offsetToStyleRuns in the
   ATSFlatDataMainHeaderBlock points to in block 1.
}
type
	ATSFlatDataStyleRunDataHeader = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ the number of style run data structures stored in this block}
		numberOfStyleRuns: UInt32;
=======

                                              { the number of style run data structures stored in this block}
		numberOfStyleRuns: ItemCount;
>>>>>>> graemeg/fixes_2_2
=======

                                              { the number of style run data structures stored in this block}
		numberOfStyleRuns: ItemCount;
>>>>>>> origin/fixes_2_2
=======
{ the number of style run data structures stored in this block}
		numberOfStyleRuns: UInt32;
>>>>>>> origin/fixes_2.4

                                              { the first in an array of ATSUStyleRunInfo structures. There needs to}
                                              { be a ATSUStyleRunInfo structure for each numberOfStyleRuns specified}
                                              { above. This structure is defined in ATSUnicode.h}
		styleRunArray: array[0..0] of ATSUStyleRunInfo;
	end;
	ATSFlatDataStyleRunDataHeaderPtr = ^ATSFlatDataStyleRunDataHeader;
{ ------------------ }
{ Block 4 Structures }
{ ------------------ }
{
   The block 4 structures store flattened ATSUStyle objects. This too, is
   currently used by the ATSUI style run flattening and parsing functions,
   ATSUFlattenStyleRunsToStream and ATSUUnflattenStyleRunsFromStream. 
}

{
   this structure forms the beginning of an individually flattened ATSUStyle
   object. 
}
type
	ATSFlatDataStyleListStyleDataHeader = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ the size of this flattened style object, including these four bytes and}
                                              { any padding bytes at the end of the structure. Basically, this can be}
                                              { used to determine where the next structure in the array begins.}
		sizeOfStyleInfo: UInt32;
=======
=======
>>>>>>> origin/fixes_2_2

                                              { the size of this flattened style object, including these four bytes and}
                                              { any padding bytes at the end of the structure. Basically, this can be}
                                              { used to determine where the next structure in the array begins.}
		sizeOfStyleInfo: ByteCount;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ the size of this flattened style object, including these four bytes and}
                                              { any padding bytes at the end of the structure. Basically, this can be}
                                              { used to determine where the next structure in the array begins.}
		sizeOfStyleInfo: UInt32;
>>>>>>> origin/fixes_2.4

                                              { the number of attributes set in this flattened style object. This should }
                                              { be at least one for the font data, although it can be 0 if this is to be}
                                              { unspecfied.}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		numberOfSetAttributes: UInt32;

                                              { the number of font features set in the flattened style object. This can}
                                              { be set to 0 if there are no font features set in the style object. }
		numberOfSetFeatures: UInt32;

                                              { the number of font variations set in the flattened style object. This}
                                              { can be set to 0 if there are no font variations set in the style object.}
		numberOfSetVariations: UInt32;
=======
=======
>>>>>>> origin/fixes_2_2
		numberOfSetAttributes: ItemCount;
=======
		numberOfSetAttributes: UInt32;
>>>>>>> origin/fixes_2.4

                                              { the number of font features set in the flattened style object. This can}
                                              { be set to 0 if there are no font features set in the style object. }
		numberOfSetFeatures: UInt32;

                                              { the number of font variations set in the flattened style object. This}
                                              { can be set to 0 if there are no font variations set in the style object.}
<<<<<<< HEAD
		numberOfSetVariations: ItemCount;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
		numberOfSetVariations: UInt32;
>>>>>>> origin/fixes_2.4

                                              { after this structure header, there is the following data in this block:}

                                              { 1. if the numberOfSetAttributes is non-zero, then there will be an}
                                              {       array of ATSUAttributeInfo structures immediately following the}
                                              {       above header data to store the style attributes. This is a variable}
                                              {       structure array. There must be one ATSUAttributeInfo for}
                                              {       for each numberOfSetAttributes. If numberOfSetAttributes is zero,}
                                              {       then skip to the next data section 2.}

                                              { 2. if the numberOfSetFeatures is non-zero, then there will be an array}
                                              {       of ATSFlatDataStyleListFeatureData structures immediately after}
                                              {       the ATSUAttributeInfo array above (if any). There must be one}
                                              {       ATSFlatDataStyleListFeatureData structure for each }
                                              {       numberOfSetFeatures set in the header above. If numberOfSetFeatures}
                                              {       is zero, then skip to the next data section 3.}

                                              { 3. if the numberOfSetVariations is non-zero, then there will be an}
                                              {       array of ATSFlatDataStyleListVariationData immediately after the}
                                              {       ATSFlatDataStyleListFeatureData array above (if any). There must be}
                                              {       one ATSFlatDataStyleListVariationData structure for each }
                                              {       numberOfSetVariations set in the header above.}
	end;
	ATSFlatDataStyleListStyleDataHeaderPtr = ^ATSFlatDataStyleListStyleDataHeader;
{
   this structure is the main header for this block. This structure is what a
   non-zero offsetToStyleList in the ATSFlatDataMainHeaderBlock points to in
   block 1.
}
type
	ATSFlatDataStyleListHeader = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ the total number of flattened style objects stored in this block}
		numberOfStyles: UInt32;
=======

                                              { the total number of flattened style objects stored in this block}
		numberOfStyles: ItemCount;
>>>>>>> graemeg/fixes_2_2
=======

                                              { the total number of flattened style objects stored in this block}
		numberOfStyles: ItemCount;
>>>>>>> origin/fixes_2_2
=======
{ the total number of flattened style objects stored in this block}
		numberOfStyles: UInt32;
>>>>>>> origin/fixes_2.4

                                              { the first in an array of flattned style entries. The data stored}
                                              { in them is variably sized, so a simply array access won't do for}
                                              { iterating through these. However, there must be one of these}
                                              { ATSFlatDataStyleListStyleDataHeader structures for each }
                                              { numberOfStyles above.}
		styleDataArray: array[0..0] of ATSFlatDataStyleListStyleDataHeader;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
	ATSFlatDataStyleListHeaderPtr = ^ATSFlatDataStyleListHeader;
{
   this structure stores flattened font feature data. An array of these comes
   after the array of font data attributes (if any) if the numberOfSetFeatures is
   non-zero. There must be one of these structures for each numberOfSetFeatures.
}
type
	ATSFlatDataStyleListFeatureData = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ the font feature type}
=======

                                              { the font feature type}
>>>>>>> graemeg/fixes_2_2
=======

                                              { the font feature type}
>>>>>>> origin/fixes_2_2
=======
{ the font feature type}
>>>>>>> origin/fixes_2.4
		theFeatureType: ATSUFontFeatureType;

                                              { the font feature selector}
		theFeatureSelector: ATSUFontFeatureSelector;
	end;
	ATSFlatDataStyleListFeatureDataPtr = ^ATSFlatDataStyleListFeatureData;
{
   this structure stores the flattened font variation data. An array of these 
   comes after the array of ATSFlatDataStyleListFeatureData structures (if any) 
   if the numberOfSetVariations is non-zero. There must be one of these
   structures for each numberOfSetFeatures.
}
type
	ATSFlatDataStyleListVariationData = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ the variation axis}
=======

                                              { the variation axis}
>>>>>>> graemeg/fixes_2_2
=======

                                              { the variation axis}
>>>>>>> origin/fixes_2_2
=======
{ the variation axis}
>>>>>>> origin/fixes_2.4
		theVariationAxis: ATSUFontVariationAxis;

                                              { the variation value}
		theVariationValue: ATSUFontVariationValue;
	end;
	ATSFlatDataStyleListVariationDataPtr = ^ATSFlatDataStyleListVariationData;
{ ------------------------ }
{ Flattened Font Data Info }
{ ------------------------ }
{
   This is somewhat of an addendum to the 'ustl' structure above. These flattened 
   data structures are stored in block 4 as a font attribute with the attribute 
   tag of kATSUFontTag. They can store font data in a few different ways, such as
   by a FontSync reference or by simple raw font name data. Just as with the 
   'ustl' above, this structure must maintain four byte alignment.
}


{ these are the currenly supported font specifiers. }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
	ATSFlatDataFontSpeciferType = UInt32;
const
{ this specifier allows the storage of font data based on name data. This}
                                        { uses the stuctures below to store the actual data itself.}
	kATSFlattenedFontSpecifierRawNameData = FourCharCode('namd');
=======
=======
>>>>>>> origin/fixes_2_2
type ATSFlatDataFontSpeciferType = UInt32;
=======
type
	ATSFlatDataFontSpeciferType = UInt32;
>>>>>>> origin/fixes_2.4
const
{ this specifier allows the storage of font data based on name data. This}
                                        { uses the stuctures below to store the actual data itself.}
<<<<<<< HEAD
  kATSFlattenedFontSpecifierRawNameData = FourCharCode('namd');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kATSFlattenedFontSpecifierRawNameData = FourCharCode('namd');
>>>>>>> origin/fixes_2.4

{
   this is the main header for the font data. It dictates what type of data
   is stored. The actual data stored must match the type specified by the
   nameSpecType. 
}
type
	ATSFlatDataFontNameDataHeader = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ the type of data that is flattened in this structure}
=======

                                              { the type of data that is flattened in this structure}
>>>>>>> graemeg/fixes_2_2
=======

                                              { the type of data that is flattened in this structure}
>>>>>>> origin/fixes_2_2
=======
{ the type of data that is flattened in this structure}
>>>>>>> origin/fixes_2.4
		nameSpecifierType: ATSFlatDataFontSpeciferType;

                                              { the size of the data that is flattened in this structre, not including }
                                              { any padding bytes that may be necessary to achive the four byte }
                                              { alignment of the data, unless they are specified as part of structure, }
                                              { such as with the ATSFlatDataFontSpecRawNameData structure.}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		nameSpecifierSize: UInt32;
=======
		nameSpecifierSize: ByteCount;
>>>>>>> graemeg/fixes_2_2
=======
		nameSpecifierSize: ByteCount;
>>>>>>> origin/fixes_2_2
=======
		nameSpecifierSize: UInt32;
>>>>>>> origin/fixes_2.4

                                              { after this header comes the flattened font name data which matches}
                                              { the type specified by the nameSpecifierType above. For instance, if }
                                              { the nameSpecType is kATSFlattenedFontNameSpecifierRawNameData, the}
                                              { structure that immediately follows this would be a}
                                              { ATSFlatDataFontNameRawNameDataHeader structure. }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
	ATSFlatDataFontNameDataHeaderPtr = ^ATSFlatDataFontNameDataHeader;
{
   the next two structures are only used when the nameSpecType is set to
   kATSFlattenedFontSpecifierRawNameData. They are setup to store multiple
   font name table entries for the purposes of reconstructing an ATSUFontID
   for (hopefully) the same font some time in the future.
}
{ this is the structure in which raw font name data is actually stored. }
type
	ATSFlatDataFontSpecRawNameData = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ the type of name being specified}
=======

                                              { the type of name being specified}
>>>>>>> graemeg/fixes_2_2
=======

                                              { the type of name being specified}
>>>>>>> origin/fixes_2_2
=======
{ the type of name being specified}
>>>>>>> origin/fixes_2.4
		fontNameType: FontNameCode;

                                              { the platform type of the font name, whether it be Unicode, Mac, etc.  }
                                              { This should be specified if known. If not known, then specify}
                                              { kFontNoPlatform, but then all matching will be done based on the first}
                                              { font in the name table matching the other parameters.}
		fontNamePlatform: FontPlatformCode;

                                              { the script code of the font’s name based on the platform that was passed}
                                              { in above. If set to kFontNoScript, then the name will be matched based}
                                              { on the first font in the name table matching the other font name}
                                              { parameters.}
		fontNameScript: FontScriptCode;

                                              { the language of the font name. If set to kFontNoLanguage, then the name }
                                              { will be matched based on the first font in the name table matching the}
                                              { other font name parameters.}
		fontNameLanguage: FontLanguageCode;

                                              { the length of the font name in bytes, not including any padding bytes}
                                              { added to maintain the four byte alignment}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		fontNameLength: UInt32;

                                              { after the name length comes the actual font name data itself, plus any}
                                              { padding bytes needed to maintain the four byte alignment.}
=======
=======
>>>>>>> origin/fixes_2_2
		fontNameLength: ByteCount;

                                              { after the name length comes the actual font name data itself, plus any}
                                              { padding bytes needed to maintain the four byte alignment.}

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
		fontNameLength: UInt32;

                                              { after the name length comes the actual font name data itself, plus any}
                                              { padding bytes needed to maintain the four byte alignment.}
>>>>>>> origin/fixes_2.4
	end;
	ATSFlatDataFontSpecRawNameDataPtr = ^ATSFlatDataFontSpecRawNameData;
{
   this is a header structure that defines some things constant throughout
   the entire search for the font name, as well as the array of
   ATSFlatDataFontNameData structures. In order to gaurantee that the same font 
   will be used, more than one name specifer should be stored. The standard ATSUI
   style run flattening and parsing functions, ATSUFlattenStyleRunsToStream and
   ATSUUnflattenStyleRunsFromStream. These will store both the font's full name
   (kFontFullName) as well as the font's manufacturer name (kFontManufacturerName)
   and match fonts based on both of
   these. 
}
type
	ATSFlatDataFontSpecRawNameDataHeader = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ the number of flattened font names. There must be at least one flattened }
                                              { font name, otherwise the structure is malformed.}
		numberOfFlattenedNames: UInt32;
=======
=======
>>>>>>> origin/fixes_2_2

                                              { the number of flattened font names. There must be at least one flattened }
                                              { font name, otherwise the structure is malformed.}
		numberOfFlattenedNames: ItemCount;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ the number of flattened font names. There must be at least one flattened }
                                              { font name, otherwise the structure is malformed.}
		numberOfFlattenedNames: UInt32;
>>>>>>> origin/fixes_2.4

                                              { the first in an array of possibly many font name specifiers - depending}
                                              { on how specific the caller wants this. There must be one }
                                              { ATSFlatDataFontNameData structure for each numberOfFlattenedNames}
                                              { above.}
		nameDataArray: array[0..0] of ATSFlatDataFontSpecRawNameData;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	end;
	ATSFlatDataFontSpecRawNameDataHeaderPtr = ^ATSFlatDataFontSpecRawNameDataHeader;
{ ---------------------------------------------------------------------------- }
{ Style Flattening and Parsing Functions                                       }
{ ---------------------------------------------------------------------------- }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc not TARGET_CPU_64}
{
 *  ATSUFlattenStyleRunsToStream()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreFoundation flattening API instead.
=======
{
 *  ATSUFlattenStyleRunsToStream()
>>>>>>> graemeg/fixes_2_2
=======
{
 *  ATSUFlattenStyleRunsToStream()
>>>>>>> origin/fixes_2_2
=======
{$ifc not TARGET_CPU_64}
{
 *  ATSUFlattenStyleRunsToStream()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreFoundation flattening API instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Converts a series of ATSUStyle objects and associated run info to
 *    a flat binary representation.
 *  
 *  Discussion:
 *    This function takes an array of ATSUStyle objects and style run
 *    information and flattens the data to the specified format. The
 *    style runs must all reference the same block of Unicode text
 *    (usually passed separately as text in the 'utxt' format). The
 *    style runs must also be in ascending order relative to the text
 *    in the text block. Typically you use the function
 *    ATSUFlattenStyleRunsFromStream by calling it twice, as follows:
 *    (1) Provide appropriate values for the iStreamFormat,
 *    iFlattenOptions, iNumberOfRunInfo, iRunInfoArray,
 *    iNumberOfStyleObjects, and iStyleArray parameters. Set
 *    iStreamBufferSize to 0, oStreamBuffer to NULL, and pass a valid
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    reference to a UInt32 variable in the oActualStreamBufferSize
=======
 *    reference to a ByteCount variable in the oActualStreamBufferSize
>>>>>>> graemeg/fixes_2_2
=======
 *    reference to a ByteCount variable in the oActualStreamBufferSize
>>>>>>> origin/fixes_2_2
=======
 *    reference to a UInt32 variable in the oActualStreamBufferSize
>>>>>>> origin/fixes_2.4
 *    parameter. Call the function ATSUFlattenStyleRunsToStream. On
 *    return, oActualStreamBufferSize points to the size needed for the
 *    buffer. (2) Allocate an appropriately-sized buffer for the
 *    oStreamBuffer parameter and then call the function
 *    ATSUFlattenStyleRunsToStream a second time.
 *  
 *  Parameters:
 *    
 *    iStreamFormat:
 *      The format of the flattened data. There is only one format
 *      supported at this time ('ustl'), so you must pass the constant
 *      kATSUDataStreamUnicodeStyledText.
 *    
 *    iFlattenOptions:
 *      The options you want to use to flatten the data. There are no
 *      options supported at this time, so you must pass the constant
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      kATSUFlattenOptionNoOptionsMask.
=======
 *      kATSUFlattenOptionsNoOptionsMask.
>>>>>>> graemeg/fixes_2_2
=======
 *      kATSUFlattenOptionsNoOptionsMask.
>>>>>>> origin/fixes_2_2
=======
 *      kATSUFlattenOptionNoOptionsMask.
>>>>>>> origin/fixes_2.4
 *    
 *    iNumberOfRunInfo:
 *      The number of style run information structures passed in the
 *      iRunInfoArray parameter. If you pass 0, ATSUI assumes there is
 *      only one style for the entire text block passed in the
 *      oStreamBuffer parameter. The flattened data format passed to
 *      the iStreamFormat parameter must support the use of one style.
 *    
 *    iRunInfoArray:
 *      An array of ATSUStyleRunInfo structures that describes the
 *      style runs to be flattened. This array must contain
 *      iNumberOfRunInfo entries. An ATSUStyleRunInfo structure
 *      contains an index into an array of unique ATSUStyle objects and
 *      the length of the run to which the style object applies. Each
 *      index in the ATSUStyleRunInfo structure must reference a valid
 *      ATSUStyle object passed in the iStyleArray parameter. You can
 *      pass NULL, only if iNumberOfRunInfo is set to zero.
 *    
 *    iNumberOfStyleObjects:
 *      The number of ATSUStyle objects in the array passed to the
 *      iStyleArray parameter. You must pass a value that is greater
 *      than 0.
 *    
 *    iStyleArray:
 *      An array of ATSUStyle objects to be flattened. You cannot pass
 *      NULL.
 *    
 *    iStreamBufferSize:
 *      The size of the stream buffer, pointed to by the oStreamBuffer
 *      parameter. You can pass 0only if the iStreamBufferSize
 *      parameter is set to NULL. If you are uncertain of the size of
 *      the array, see the Discussion.
 *    
 *    oStreamBuffer:
 *      On input, a pointer to the data you want to flatten. On return,
 *      points to the flattened data. If you pass NULL for this
 *      parameter, no data is flattened. Instead, the size of the
 *      buffer is calculated by ATSUI and returned in oActualStreamSize
 *      parameter. See the Discussion for more details. You are
 *      responsible for allocating the text buffer passed in the
 *      oStreamBuffer parameter.
 *    
 *    oActualStreamBufferSize:
 *      On return, the size of the data written to the oStreamBuffer
 *      parameter. You can pass NULL only if the oStreamBuffer
 *      parameter is not NULL.
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function ATSUFlattenStyleRunsToStream( iStreamFormat: ATSUFlattenedDataStreamFormat; iFlattenOptions: ATSUFlattenStyleRunOptions; iNumberOfRunInfo: ItemCount; {const} iRunInfoArray: {variable-size-array} ATSUStyleRunInfoPtr; iNumberOfStyleObjects: ItemCount; {const} iStyleArray: {variable-size-array} ATSUStylePtr; iStreamBufferSize: ByteCount; oStreamBuffer: UnivPtr; oActualStreamBufferSize: ByteCountPtr ): OSStatus; external name '_ATSUFlattenStyleRunsToStream';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUUnflattenStyleRunsFromStream()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreFoundation flattening API instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function ATSUFlattenStyleRunsToStream( iStreamFormat: ATSUFlattenedDataStreamFormat; iFlattenOptions: ATSUFlattenStyleRunOptions; iNumberOfRunInfo: ItemCount; {const} iRunInfoArray: {variable-size-array} ATSUStyleRunInfoPtr; iNumberOfStyleObjects: ItemCount; {const} iStyleArray: {variable-size-array} ATSUStylePtr; iStreamBufferSize: ByteCount; oStreamBuffer: UnivPtr; oActualStreamBufferSize: ByteCountPtr ): OSStatus; external name '_ATSUFlattenStyleRunsToStream';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUUnflattenStyleRunsFromStream()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUUnflattenStyleRunsFromStream()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreFoundation flattening API instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Creates a series of ATSUStyle objects and associated run
 *    information from a flat binary representation.
 *  
 *  Discussion:
 *    This function extracts the ATSUI style run information from
 *    previously-flattened data. The style objects and style run
 *    information structures are returned in two separate arrays—the
 *    array oStyleArray and the array oRunInfoArray. These arrays are
 *    not parallel. Each ATSUStyle object in the oStyleArray is a
 *    unique ATSUStyle object. To figure out which ATSUStyle object
 *    belongs to which text run, the caller must parse the array of
 *    ATSUStyleRunInfo structures. These structures contain the style
 *    run lengths and an index into the oStyleArray. Typically you use
 *    the function ATSUUnflattenStyleRunsFromStream by calling it
 *    twice, as follows: (1) Provide appropriate values for the
 *    iStreamFormat, iUnflattenOptions, and iStreamBuffer parameters.
 *    Pass 0for the iNumberOfRunInfo and iNumberOfStyleObjects
 *    parameters, NULL for the oRunInfoArray and oStyleArray,
 *    parameters and valid ItemCount references for the
 *    oActualNumberOfRunInfo and oActualNumberOfStyleObjects
 *    parameters. On return, oActualNumberOfRunInfo and
 *    oActualNumberOfStyleObjects point to the sizes needed to allocate
 *    these arrays. (2) Allocate appropriately-sized arrays of
 *    ATSUStyleRunStructures and ATSUStyle object references. Call the
 *    function ATSUUnflattenStyleRunsFromStream a second time, passing
 *    the newly allocated arrays in the oRunInfoArray and oStyleArray
 *    parameters, with the iNumberOfRunInfo and iNumberOfStyleObjects
 *    parameters set to the values you obtained from the first call.
 *  
 *  Parameters:
 *    
 *    iStreamFormat:
 *      The format of the flattened data. There is only one format
 *      supported at this time ('ustl'), so you must pass the constant
 *      kATSUDataStreamUnicodeStyledText.
 *    
 *    iUnflattenOptions:
 *      The options you want to use to unflatten the data. There are no
 *      options supported at this time, so you must pass the constant
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      kATSUUnflattenOptionNoOptionsMask.
=======
 *      kATSUUnflattenOptionsNoOptionsMask.
>>>>>>> graemeg/fixes_2_2
=======
 *      kATSUUnflattenOptionsNoOptionsMask.
>>>>>>> origin/fixes_2_2
=======
 *      kATSUUnflattenOptionNoOptionsMask.
>>>>>>> origin/fixes_2.4
 *    
 *    iStreamBufferSize:
 *      The size of the buffer pointed to by the iStreamBuffer
 *      parameter. You must pass a value greater than 0.
 *    
 *    iStreamBuffer:
 *      A pointer to the buffer that contains the flattened data. The
 *      data must be of the format specified by the iStreamFormat
 *      parameter and must be of size specified by the
 *      iStreamBufferSize parameter. You cannot pass NULL .
 *    
 *    iNumberOfRunInfo:
 *      The number of style run information structures passed in the
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      oRunInfoArray parameter. If you are uncertain of the number of
=======
 *      iRunInfoArray parameter. If you are uncertain of the number of
>>>>>>> graemeg/fixes_2_2
=======
 *      iRunInfoArray parameter. If you are uncertain of the number of
>>>>>>> origin/fixes_2_2
=======
 *      oRunInfoArray parameter. If you are uncertain of the number of
>>>>>>> origin/fixes_2.4
 *      style run information structures, see the Discussion.
 *    
 *    iNumberOfStyleObjects:
 *      The number of ATSUStyle objects in the array passed into the
 *      iStyleArray parameter. If you are uncertain of the number of
 *      ATSUStyle objects, see the Discussion.
 *    
 *    oRunInfoArray:
 *      On return, points to an array of style run information
 *      structures. Each structure contains a style run length and
 *      index into the oStyleArray array. If you are uncertain of how
 *      much memory to allocate for this array, see the Discussion. You
 *      are responsible for disposing of the array when you no longer
 *      need it.
 *    
 *    oStyleArray:
 *      On return, a pointer to an array of the unique ATSUStyle
 *      objects obtained from the flattened data. The indices returned
 *      in the array oRunInfoArray are indices into this array. If you
 *      are uncertain of how much memory to allocate for this array,
 *      see the Discussion. You are responsible for disposing of the
 *      array and the ATSUI style objects in the array when you no
 *      longer need the array.
 *    
 *    oActualNumberOfRunInfo:
 *      On return, points to the actual number of ATSUStyleRunInfo
 *      structures obtained from the flattened data. The actual number
 *      of structures is the number of entries added to the array
 *      oRunInfoArray. You can pass NULL if you to not want to obtain
 *      this value.
 *    
 *    oActualNumberOfStyleObjects:
 *      On return, points to the actual number of unique ATSUStyle
 *      objects obtained from the flattened data. The actual number is
 *      the number of entries added to the oStyleArray array. You can
 *      pass NULL if you do no want to obtain this value.
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function ATSUUnflattenStyleRunsFromStream( iStreamFormat: ATSUFlattenedDataStreamFormat; iUnflattenOptions: ATSUUnFlattenStyleRunOptions; iStreamBufferSize: ByteCount; iStreamBuffer: {const} UnivPtr; iNumberOfRunInfo: ItemCount; iNumberOfStyleObjects: ItemCount; oRunInfoArray: {variable-size-array} ATSUStyleRunInfoPtr; oStyleArray: {variable-size-array} ATSUStylePtr; oActualNumberOfRunInfo: ItemCountPtr; oActualNumberOfStyleObjects: ItemCountPtr ): OSStatus; external name '_ATSUUnflattenStyleRunsFromStream';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
// AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER
function ATSUUnflattenStyleRunsFromStream( iStreamFormat: ATSUFlattenedDataStreamFormat; iUnflattenOptions: ATSUUnFlattenStyleRunOptions; iStreamBufferSize: ByteCount; iStreamBuffer: UnivPtr; iNumberOfRunInfo: ItemCount; iNumberOfStyleObjects: ItemCount; oRunInfoArray: ATSUStyleRunInfoPtr; oStyleArray: ATSUStylePtr; oActualNumberOfRunInfo: ItemCountPtr; oActualNumberOfStyleObjects: ItemCountPtr ): OSStatus; external name '_ATSUUnflattenStyleRunsFromStream';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function ATSUUnflattenStyleRunsFromStream( iStreamFormat: ATSUFlattenedDataStreamFormat; iUnflattenOptions: ATSUUnFlattenStyleRunOptions; iStreamBufferSize: ByteCount; iStreamBuffer: {const} UnivPtr; iNumberOfRunInfo: ItemCount; iNumberOfStyleObjects: ItemCount; oRunInfoArray: {variable-size-array} ATSUStyleRunInfoPtr; oStyleArray: {variable-size-array} ATSUStylePtr; oActualNumberOfRunInfo: ItemCountPtr; oActualNumberOfStyleObjects: ItemCountPtr ): OSStatus; external name '_ATSUUnflattenStyleRunsFromStream';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4


{ Functions listed beyond this point are either deprecated or not recommended }

{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  ATSUCopyToHandle()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreFoundation flattening API instead.
<<<<<<< HEAD
=======
 *  ATSUCopyToHandle()
>>>>>>> graemeg/fixes_2_2
=======
 *  ATSUCopyToHandle()
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  
 *  Discussion:
 *    This function is no longer recommended. Please use
 *    ATSUFlattenStyleRunsToStream instead.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.1
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUCopyToHandle( iStyle: ATSUStyle; oStyleHandle: Handle ): OSStatus; external name '_ATSUCopyToHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_1 *)


{
 *  ATSUPasteFromHandle()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreFoundation flattening API instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.1
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUCopyToHandle( iStyle: ATSUStyle; oStyleHandle: Handle ): OSStatus; external name '_ATSUCopyToHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_1 *)


{
<<<<<<< HEAD
 *  ATSUPasteFromHandle()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUPasteFromHandle()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreFoundation flattening API instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Discussion:
 *    This function is no longer recommended. Please use
 *    ATSUUnflattenStyleRunsFromStream instead.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.1
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUPasteFromHandle( iStyle: ATSUStyle; iStyleHandle: Handle ): OSStatus; external name '_ATSUPasteFromHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_1 *)


{$endc} {not TARGET_CPU_64}


{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.1
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 8.5 and later
 }
function ATSUPasteFromHandle( iStyle: ATSUStyle; iStyleHandle: Handle ): OSStatus; external name '_ATSUPasteFromHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_1 *)


{$endc} {not TARGET_CPU_64}


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
