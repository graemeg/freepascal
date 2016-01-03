{
     File:       HIToolbox/MacTextEditor.h
 
     Contains:   Interfaces for Multilingual Text Engine (MLTE)
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Version:    HIToolbox-624~3
=======
     Version:    HIToolbox-437~1
>>>>>>> graemeg/cpstrnew
=======
     Version:    HIToolbox-437~1
>>>>>>> graemeg/cpstrnew
=======
     Version:    HIToolbox-437~1
>>>>>>> graemeg/cpstrnew
=======
     Version:    HIToolbox-437~1
>>>>>>> origin/cpstrnew
 
     Copyright:  © 1996-2008 by Apple Computer, Inc., all rights reserved.
=======
     Version:    HIToolbox-219.4.81~2
 
     Copyright:  © 1996-2005 by Apple Computer, Inc., all rights reserved.
>>>>>>> graemeg/fixes_2_2
=======
     Version:    HIToolbox-219.4.81~2
 
     Copyright:  © 1996-2005 by Apple Computer, Inc., all rights reserved.
>>>>>>> origin/fixes_2_2
=======
     Version:    HIToolbox-437~1
 
     Copyright:  © 1996-2008 by Apple Computer, Inc., all rights reserved.
>>>>>>> origin/fixes_2.4
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}
{       Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
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
                     http://www.freepascal.org/bugs.html
 
}
{       Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
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

unit MacTextEditor;
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
>>>>>>> graemeg/cpstrnew
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
>>>>>>> origin/cpstrnew
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
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
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
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
uses MacTypes,CFBase,CFURL,CFArray,CFData,CFString,CFDictionary,CGBase,AEDataModel,TextCommon,QuickdrawTypes,QDOffscreen,Menus,ATSUnicodeTypes,ConditionalMacros,Drag,MacWindows,Files,Events,MacErrors,CarbonEvents,HIObject,HIView,HIGeometry;
{$endc} {not MACOSALLINCLUDE}

{$ALIGN POWER}

{$ifc TARGET_OS_MAC}
=======
uses MacTypes,CFBase,CFURL,CFArray,CFData,CFString,CFDictionary,CGBase,AEDataModel,TextCommon,Quickdraw,QDOffscreen,Menus,ATSUnicodeTypes,ConditionalMacros,Drag,MacWindows,Files,Events,MacErrors,CarbonEvents,HIObjectCore,HIView,HIGeometry;
{$ALIGN POWER}

>>>>>>> graemeg/fixes_2_2
=======
uses MacTypes,CFBase,CFURL,CFArray,CFData,CFString,CFDictionary,CGBase,AEDataModel,TextCommon,Quickdraw,QDOffscreen,Menus,ATSUnicodeTypes,ConditionalMacros,Drag,MacWindows,Files,Events,MacErrors,CarbonEvents,HIObjectCore,HIView,HIGeometry;
{$ALIGN POWER}

>>>>>>> origin/fixes_2_2
=======
uses MacTypes,CFBase,CFURL,CFArray,CFData,CFString,CFDictionary,CGBase,AEDataModel,TextCommon,QuickdrawTypes,QDOffscreen,Menus,ATSUnicodeTypes,ConditionalMacros,Drag,MacWindows,Files,Events,MacErrors,CarbonEvents,HIObject,HIView,HIGeometry;
{$endc} {not MACOSALLINCLUDE}

{$ALIGN POWER}

{$ifc TARGET_OS_MAC}
>>>>>>> origin/fixes_2.4

{$ALIGN MAC68K}


{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Various Defs                                                                                      }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	TXNObject = ^OpaqueTXNObject; { an opaque type }
	OpaqueTXNObject = record end;
=======
	TXNObject = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	TXNObject = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	TXNObject = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	TXNObject = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
	TXNObjectPtr = ^TXNObject;  { when a var xx:TXNObject parameter can be nil, it is changed to xx: TXNObjectPtr }
type
=======
=======
>>>>>>> origin/fixes_2_2

=======
>>>>>>> origin/fixes_2.4
type
	TXNObject = ^SInt32; { an opaque type }
	TXNObjectPtr = ^TXNObject;  { when a var xx:TXNObject parameter can be nil, it is changed to xx: TXNObjectPtr }
type
<<<<<<< HEAD
	TXNFontMenuObject = ^SInt32; { an opaque 32-bit type }
	TXNFontMenuObjectPtr = ^TXNFontMenuObject;  { when a var xx:TXNFontMenuObject parameter can be nil, it is changed to xx: TXNFontMenuObjectPtr }
type
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	TXNVersionValue = UInt32;

{
 *  TXNFrameID
 *  
 *  Summary:
 *    Deprecated.  Pass NULL for any API which uses this type as a
 *    parameter.
 }
type
	TXNFrameID = UInt32;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======


>>>>>>> graemeg/fixes_2_2
=======


>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Error Status                                                                                      }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  MLTE status errors assigned is -22000 through -22039.  See MacErrors.h for -22000 to -22018.        }

const
	kTXNDisabledFunctionalityErr = -22019; { This routine's functionality is disabled.}
	kTXNOperationNotAllowedErr = -22020; { Returned by MLTE API if function cannot be applied. }

{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Feature Bits                                                                                      }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  Useful for the TXNVersionInformation API.                                                           }

const
	kTXNWillDefaultToATSUIBit = 0;
	kTXNWillDefaultToCarbonEventBit = 1;   { Ignored - Carbon events are always used in Mac OS X version 10.1 and later.}

type
	TXNFeatureBits = OptionBits;
const
	kTXNWillDefaultToATSUIMask = 1 shl kTXNWillDefaultToATSUIBit;
	kTXNWillDefaultToCarbonEventMask = 1 shl kTXNWillDefaultToCarbonEventBit;

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
{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Initialization Bits                                                                               }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  Useful for the TXNInitTextension API.                                                               }

const
	kTXNWantMoviesBit = 0;
	kTXNWantSoundBit = 1;
	kTXNWantGraphicsBit = 2;
	kTXNAlwaysUseQuickDrawTextBit = 3;
	kTXNUseTemporaryMemoryBit = 4;

type
	TXNInitOptions = OptionBits;
const
	kTXNWantMoviesMask = 1 shl kTXNWantMoviesBit;
	kTXNWantSoundMask = 1 shl kTXNWantSoundBit;
	kTXNWantGraphicsMask = 1 shl kTXNWantGraphicsBit;
	kTXNAlwaysUseQuickDrawTextMask = 1 shl kTXNAlwaysUseQuickDrawTextBit;
	kTXNUseTemporaryMemoryMask = 1 shl kTXNUseTemporaryMemoryBit;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ Default constants}
=======
{ Default constants  }
>>>>>>> graemeg/fixes_2_2
=======
{ Default constants  }
>>>>>>> origin/fixes_2_2
=======
{ Default constants}
>>>>>>> origin/fixes_2.4

const
	kTXNDefaultFontName = nil;

const
	kTXNDefaultFontSize = $000C0000;

const
	kTXNDefaultFontStyle = normal;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not TARGET_CPU_64}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {not TARGET_CPU_64}
>>>>>>> origin/fixes_2.4
{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • FrameOption Bits                                                                                  }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  Useful for the TXNNewObject and TXNCreateObject APIs.                                               }

const
	kTXNDrawGrowIconBit = 0;
	kTXNShowWindowBit = 1;
	kTXNWantHScrollBarBit = 2;
	kTXNWantVScrollBarBit = 3;
	kTXNReadOnlyBit = 5;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
	kTXNNoKeyboardSyncBit = 6;
>>>>>>> graemeg/fixes_2_2
=======
	kTXNNoKeyboardSyncBit = 6;
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	kTXNNoSelectionBit = 7;
	kTXNSaveStylesAsSTYLResourceBit = 8;
	kOutputTextInUnicodeEncodingBit = 9;
	kTXNDoNotInstallDragProcsBit = 10;
	kTXNAlwaysWrapAtViewEdgeBit = 11;
	kTXNDontDrawSelectionWhenInactiveBit = 13;
	kTXNSingleLineOnlyBit = 14;
	kTXNDisableDragAndDropBit = 15;
	kTXNMonostyledTextBit = 17;
	kTXNDoFontSubstitutionBit = 22;


{
 *  TXNFrameOptions
 *  
 *  Summary:
 *    Defines the initial behavior of an MLTE object created with
 *    TXNCreateObject/TXNNewObject.
 *  
 *  Discussion:
 *    These masks can be combined and passed to
 *    TXNCreateObject/TXNNewObject to define the initial behavior of a
 *    new object.
 }
type
	TXNFrameOptions = OptionBits;
const
{
   * Indicates that the frame will have a size box.
   }
	kTXNDrawGrowIconMask = 1 shl kTXNDrawGrowIconBit;

  {
   * Indicates that the window associated with the text object will be
   * displayed when the object is created.  The application no longer
   * needs to call the ShowWindow function from the Window Manager;
   * MLTE will do this for you.
   }
	kTXNShowWindowMask = 1 shl kTXNShowWindowBit;

  {
   * Indicates that the frame will have a horizontal scrollbar. The
   * scrollbar will be enabled upon creation.  If there are multiple
   * MLTE objects in the same window, the client will need to
   * explicitly disable the scrollbars for those inactive MLTE objects.
   *  Use TXNSetScrollbarState to deactivate the scrollbar.
   }
	kTXNWantHScrollBarMask = 1 shl kTXNWantHScrollBarBit;

  {
   * Indicates that the frame will have a vertical scrollbar. The
   * scrollbar will be enabled upon creation.  If there are multiple
   * MLTE objects in the same window, the client will need to
   * explicitly disable the scrollbars for those inactive MLTE objects.
   *  Use TXNSetScrollbarState to deactivate the scrollbar.
   }
	kTXNWantVScrollBarMask = 1 shl kTXNWantVScrollBarBit;

  {
   * Indicates that the text object will be read-only.  It is
   * equivalent to setting the object into NoUserIO mode, via the tag
   * kTXNNoUserIOTag in TXNSetTXNObjectControls. See description in
   * individual API to determine if the API will still work in NoUserIO
   * mode.
   }
	kTXNReadOnlyMask = 1 shl kTXNReadOnlyBit;

  {
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
   * Indicates that keyboard synchronization will not occur.
   }
	kTXNNoKeyboardSyncMask = 1 shl kTXNNoKeyboardSyncBit;

  {
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
   * Indicates that the user shouldn't be able to set the insertion
   * point or make a selection.
   }
	kTXNNoSelectionMask = 1 shl kTXNNoSelectionBit;

  {
   * Indicates that the text style will be saved as a
   * kTXNMultipleStylesPerTextDocumentResType resource.  You can set
   * this to assure compatibility with SimpleText.  If you use
   * kTXNMultipleStylesPerTextDocumentResType resources to save style
   * info, your documents can have as many styles as you'd like. 
   * However tabs are not saved.  If you don't use this mask, plain
   * text files are saved as kTXNSingleStylePerTextDocumentResType
   * resources, and only the first style in the document is saved. 
   * (Your application is expected to apply all style changes to the
   * entire document.)  If you save files with a
   * kTXNSingleStylePerTextDocumentResType resource, their output is
   * similar to those output by CodeWarrior, BBEdit, and MPW.
   }
	kTXNSaveStylesAsSTYLResourceMask = 1 shl kTXNSaveStylesAsSTYLResourceBit;

  {
   * Indicates that plain text will be saved as Unicode.
   }
	kOutputTextInUnicodeEncodingMask = 1 shl kOutputTextInUnicodeEncodingBit;

  {
   * Indicates that MLTE will not install its own drag handler for the
   * text object.  This can be used if the client wants to install
   * their own handler.
   }
	kTXNDoNotInstallDragProcsMask = 1 shl kTXNDoNotInstallDragProcsBit;

  {
   * Indicates that lines will wrap at the edge of the view rectangle.
   }
	kTXNAlwaysWrapAtViewEdgeMask = 1 shl kTXNAlwaysWrapAtViewEdgeBit;

  {
   * Indicates that the selection (if one) shouldn't be drawn when the
   * text object doesn't have focus.
   }
	kTXNDontDrawSelectionWhenInactiveMask = 1 shl kTXNDontDrawSelectionWhenInactiveBit;

  {
   * Indicates that the text object will not scroll vertically,
   * horizontal scrolling will stop when the end of the text is visible
   * (plus any right margin), and there will be no limit to the width
   * of the text.
   }
	kTXNSingleLineOnlyMask = 1 shl kTXNSingleLineOnlyBit;

  {
   * Indicates that drag and drop will not be allowed in the text
   * object.
   }
	kTXNDisableDragAndDropMask = 1 shl kTXNDisableDragAndDropBit;

  {
   * Indicates that the text object will keep in single style no matter
   * what kind of changes made to the object.  Mac OS X only.
   }
	kTXNMonostyledTextMask = 1 shl kTXNMonostyledTextBit;

  {
   * Indicates that ATSUI font substitution will be used.  Mac OS X
   * only.
   }
	kTXNDoFontSubstitutionMask = 1 shl kTXNDoFontSubstitutionBit;

{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • TextBox Option Bits                                                                               }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{  Useful for TXNDrawUnicodeTextBox and TXNDrawCFStringTextBox APIs. 32 bit only                       }
{$ifc not TARGET_CPU_64}
=======
{  Useful for TXNDrawUnicodeTextBox and TXNDrawCFStringTextBox APIs.                                   }

>>>>>>> graemeg/fixes_2_2
=======
{  Useful for TXNDrawUnicodeTextBox and TXNDrawCFStringTextBox APIs.                                   }

>>>>>>> origin/fixes_2_2
=======
{  Useful for TXNDrawUnicodeTextBox and TXNDrawCFStringTextBox APIs. 32 bit only                       }
{$ifc not TARGET_CPU_64}
>>>>>>> origin/fixes_2.4
const
	kTXNSetFlushnessBit = 0;
	kTXNSetJustificationBit = 1;
	kTXNUseFontFallBackBit = 2;
	kTXNRotateTextBit = 3;
	kTXNUseVerticalTextBit = 4;
	kTXNDontUpdateBoxRectBit = 5;
	kTXNDontDrawTextBit = 6;
	kTXNUseCGContextRefBit = 7;
	kTXNDontWrapTextBit = 9;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not TARGET_CPU_64}

{$ifc not TARGET_CPU_64}
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
=======
{$endc} {not TARGET_CPU_64}

{$ifc not TARGET_CPU_64}
>>>>>>> origin/fixes_2.4
{
 *  TXNTextBoxOptions
 *  
 *  Summary:
 *    Defines how text will be drawn by the TXNDrawUnicodeTextBox and
 *    TXNDrawCFStringTextBox APIs.
 *  
 *  Discussion:
 *    These masks can be combined and added to a TXNTextBoxOptionsData
 *    structure to be passed to the TXNDrawUnicodeTextBox and
 *    TXNDrawCFStringTextBox APIs.
 }
type
	TXNTextBoxOptions = OptionBits;
const
{
   * Indicates that the text will be flush according to the line
   * direction.
   }
	kTXNSetFlushnessMask = 1 shl kTXNSetFlushnessBit;

  {
   * Indicates that the text will be justified in the direction that
   * the text is displayed.  Horizontal text will be justified
   * horizontally, but not vertically.  Vertical text will be justified
   * vertically, but not horizontally.
   }
	kTXNSetJustificationMask = 1 shl kTXNSetJustificationBit;

  {
   * Indicates that ATSUI font substitution (that searches for a font
   * that has a matching character) will be used.
   }
	kTXNUseFontFallBackMask = 1 shl kTXNUseFontFallBackBit;

  {
   * Indicates that the text will be rotated.  The amount of rotation
   * is given in the rotation field of the TXNTextBoxOptionsData
   * structure and is in units of degrees (negative values indicate
   * clockwise rotation).
   }
	kTXNRotateTextMask = 1 shl kTXNRotateTextBit;

  {
   * Indicates that the text will be displayed vertically from top to
   * bottom.
   }
	kTXNUseVerticalTextMask = 1 shl kTXNUseVerticalTextBit;

  {
   * Indicates that the specified rectangle will not be updated.  If
   * you use this mask when you call a TXNDrawxxxTextBox function, the
   * function does not update the right coordinate (bottom coordinate
   * if kTXNUseVerticalTextMask is used) of the specified rectangle to
   * accommodate the longest line for text.
   }
	kTXNDontUpdateBoxRectMask = 1 shl kTXNDontUpdateBoxRectBit;

  {
   * Indicates that the size of the text will be returned but the text
   * box will not be drawn.
   }
	kTXNDontDrawTextMask = 1 shl kTXNDontDrawTextBit;

  {
   * Indicates that the client has provided a CGContext to be used for
   * CG imaging inside the text box.  Pass the CGContextRef in the
   * options field of the TXNTextBoxOptionsData structure.  Mac OS X
   * only.
   }
	kTXNUseCGContextRefMask = 1 shl kTXNUseCGContextRefBit;

  {
   * Indicates that text should not be wrapped.  Mac OS X only.
   }
	kTXNDontWrapTextMask = 1 shl kTXNDontWrapTextBit;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not TARGET_CPU_64}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {not TARGET_CPU_64}

>>>>>>> origin/fixes_2.4
{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • TextBox Options Data                                                                              }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

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
type
	TXNTextBoxOptionsDataPtr = ^TXNTextBoxOptionsData;
	TXNTextBoxOptionsData = record
		optionTags: TXNTextBoxOptions;
		flushness: Fract;
		justification: Fract;
		rotation: Fixed;
		options: UnivPtr;
	end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not TARGET_CPU_64}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {not TARGET_CPU_64}
>>>>>>> origin/fixes_2.4

{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • File Types                                                                                        }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

{
 *  TXNFileType
 *  
 *  Discussion:
 *    Useful for TXNNewObject and TXNSave APIs.
 }
type
	TXNFileType = OSType;
const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kTXNTextFile = FourCharCode('TEXT');
	kTXNAIFFFile = FourCharCode('AIFF');
	kTXNUnicodeTextFile = FourCharCode('utxt');
	kTXNTextensionFile = FourCharCode('txtn');

{$ifc not TARGET_CPU_64}
const
	kTXNPictureFile = FourCharCode('PICT');
	kTXNMovieFile = FourCharCode('MooV');
	kTXNSoundFile = FourCharCode('sfil');

{$endc} {not TARGET_CPU_64}
=======
=======
>>>>>>> origin/fixes_2_2
	kTXNTextensionFile = FourCharCode('txtn');
=======
>>>>>>> origin/fixes_2.4
	kTXNTextFile = FourCharCode('TEXT');
	kTXNAIFFFile = FourCharCode('AIFF');
	kTXNUnicodeTextFile = FourCharCode('utxt');
	kTXNTextensionFile = FourCharCode('txtn');

{$ifc not TARGET_CPU_64}
const
	kTXNPictureFile = FourCharCode('PICT');
	kTXNMovieFile = FourCharCode('MooV');
	kTXNSoundFile = FourCharCode('sfil');
<<<<<<< HEAD
	kTXNAIFFFile = FourCharCode('AIFF');
	kTXNUnicodeTextFile = FourCharCode('utxt');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

{$endc} {not TARGET_CPU_64}
>>>>>>> origin/fixes_2.4

{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Text Encoding Types                                                                               }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

{
 *  TXNPermanentTextEncodingType
 *  
 *  Discussion:
 *    Useful for TXNNewObject and TXNSave APIs.
 }
type
	TXNPermanentTextEncodingType = UInt32;
const
	kTXNSystemDefaultEncoding = 0;
	kTXNMacOSEncoding = 1;
	kTXNUnicodeEncoding = 2;

{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Data Types                                                                                        }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

type
	TXNDataTypePtr = ^TXNDataType;
	TXNDataType = OSType;
const
	kTXNTextData = FourCharCode('TEXT');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
	kTXNUnicodeTextData = FourCharCode('utxt');
	kTXNRichTextFormatData = FourCharCode('RTF ');

{$ifc not TARGET_CPU_64}
const
<<<<<<< HEAD
	kTXNPictureData = FourCharCode('PICT');
	kTXNMovieData = FourCharCode('moov');
	kTXNSoundData = FourCharCode('snd ');
	kTXNTextAndMultimediaData = FourCharCode('txtn');

{$endc} {not TARGET_CPU_64}
=======
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	kTXNPictureData = FourCharCode('PICT');
	kTXNMovieData = FourCharCode('moov');
	kTXNSoundData = FourCharCode('snd ');
	kTXNTextAndMultimediaData = FourCharCode('txtn');
<<<<<<< HEAD
	kTXNRichTextFormatData = FourCharCode('RTF ');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

{$endc} {not TARGET_CPU_64}
>>>>>>> origin/fixes_2.4


{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Action constants used in undo/redo functions                                                      }
{  In Mac OS X version 10.4 and later TXNActionKey is deprecated.                                          }
{  The following action constants should be used instead.                                              }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

{
 *  kTXNActionTyping
 *  
 *  Discussion:
 *    Denotes a typing action.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
var kTXNActionTyping: CFStringRef; external name '_kTXNActionTyping'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  kTXNActionCut
 *  
 *  Discussion:
 *    Denotes a cut action.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
var kTXNActionCut: CFStringRef; external name '_kTXNActionCut'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  kTXNActionPaste
 *  
 *  Discussion:
 *    Denotes a paste action.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
var kTXNActionPaste: CFStringRef; external name '_kTXNActionPaste'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  kTXNActionClear
 *  
 *  Discussion:
 *    Denotes a clear action.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
var kTXNActionClear: CFStringRef; external name '_kTXNActionClear'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  kTXNActionChangeFont
 *  
 *  Discussion:
 *    Denotes a change font action.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
var kTXNActionChangeFont: CFStringRef; external name '_kTXNActionChangeFont'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  kTXNActionChangeColor
 *  
 *  Discussion:
 *    Denotes a change color action.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
var kTXNActionChangeColor: CFStringRef; external name '_kTXNActionChangeColor'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  kTXNActionChangeSize
 *  
 *  Discussion:
 *    Denotes a change size action.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
var kTXNActionChangeSize: CFStringRef; external name '_kTXNActionChangeSize'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  kTXNActionChangeStyle
 *  
 *  Discussion:
 *    Denotes a change style action.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
var kTXNActionChangeStyle: CFStringRef; external name '_kTXNActionChangeStyle'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  kTXNActionAlignLeft
 *  
 *  Discussion:
 *    Denotes an align left action.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
var kTXNActionAlignLeft: CFStringRef; external name '_kTXNActionAlignLeft'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  kTXNActionAlignCenter
 *  
 *  Discussion:
 *    Denotes an align center action.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
var kTXNActionAlignCenter: CFStringRef; external name '_kTXNActionAlignCenter'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  kTXNActionAlignRight
 *  
 *  Discussion:
 *    Denotes an align right action.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
var kTXNActionAlignRight: CFStringRef; external name '_kTXNActionAlignRight'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  kTXNActionDrop
 *  
 *  Discussion:
 *    Denotes a drop action.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
var kTXNActionDrop: CFStringRef; external name '_kTXNActionDrop'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  kTXNActionMove
 *  
 *  Discussion:
 *    Denotes a move action.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
var kTXNActionMove: CFStringRef; external name '_kTXNActionMove'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  kTXNActionChangeFontFeature
 *  
 *  Discussion:
 *    Denotes a change font feature action.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
var kTXNActionChangeFontFeature: CFStringRef; external name '_kTXNActionChangeFontFeature'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  kTXNActionChangeFontVariation
 *  
 *  Discussion:
 *    Denotes a change font variation action.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
var kTXNActionChangeFontVariation: CFStringRef; external name '_kTXNActionChangeFontVariation'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  kTXNActionChangeGlyphVariation
 *  
 *  Discussion:
 *    Denotes a change glyph variation action.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
var kTXNActionChangeGlyphVariation: CFStringRef; external name '_kTXNActionChangeGlyphVariation'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  kTXNActionChangeTextPosition
 *  
 *  Discussion:
 *    Denotes a change text's position action, which includes changing
 *    the space before/after characters and shifting the text's
 *    baseline.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
var kTXNActionChangeTextPosition: CFStringRef; external name '_kTXNActionChangeTextPosition'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  kTXNActionUndoLast
 *  
 *  Discussion:
 *    Used in undo/redo functions if none of the other constants apply.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
var kTXNActionUndoLast: CFStringRef; external name '_kTXNActionUndoLast'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Format Setting Constants                                                                          }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

type
	TXNTabType = SInt8;
const
	kTXNRightTab = -1;
	kTXNLeftTab = 0;
	kTXNCenterTab = 1;

type
	TXNTabPtr = ^TXNTab;
	TXNTab = record
		value: SInt16;
		tabType: TXNTabType;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		filler: UInt8;
=======
		filler: SInt8;
>>>>>>> graemeg/fixes_2_2
=======
		filler: SInt8;
>>>>>>> origin/fixes_2_2
=======
		filler: UInt8;
>>>>>>> origin/fixes_2.4
	end;
const
	kTXNLeftToRight = 0;
	kTXNRightToLeft = 1;

const
	kTXNFlushDefault = 0;    { Flush according to the line direction    }
	kTXNFlushLeft = 1;
	kTXNFlushRight = 2;
	kTXNCenter = 4;
	kTXNFullJust = 8;
	kTXNForceFullJust = 16;    { Flush left and right for all lines   }


{
 *  TXNMargins
 *  
 *  Discussion:
 *    In MLTE version 1.2 and later you can change the top, left and
 *    right margins.
 }
type
	TXNMarginsPtr = ^TXNMargins;
	TXNMargins = record
		topMargin: SInt16;
		leftMargin: SInt16;
		bottomMargin: SInt16;

  {
   * The bottom margin is a placeholder for possible future
   * enhancements.
   }
		rightMargin: SInt16;
	end;

{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Control Tags                                                                                      }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

{
 *  TXNControlTag
 *  
 }
type
	TXNControlTag = FourCharCode;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	TXNControlTagPtr = ^TXNControlTag; { when a VAR xx: TXNControlTag parameter can be nil, it is changed to xx: TXNControlTagPtr }
=======
	TXNControlTagPtr			= ^TXNControlTag; { when a VAR xx: TXNControlTag parameter can be nil, it is changed to xx: TXNControlTagPtr }
>>>>>>> graemeg/fixes_2_2
=======
	TXNControlTagPtr			= ^TXNControlTag; { when a VAR xx: TXNControlTag parameter can be nil, it is changed to xx: TXNControlTagPtr }
>>>>>>> origin/fixes_2_2
=======
	TXNControlTagPtr = ^TXNControlTag; { when a VAR xx: TXNControlTag parameter can be nil, it is changed to xx: TXNControlTagPtr }
>>>>>>> origin/fixes_2.4
const
	kTXNLineDirectionTag = FourCharCode('lndr'); { Not functional when userIO is not allowed   }
	kTXNJustificationTag = FourCharCode('just'); { Not functional when userIO is not allowed   }
	kTXNIOPrivilegesTag = FourCharCode('iopv');
	kTXNSelectionStateTag = FourCharCode('slst');
	kTXNInlineStateTag = FourCharCode('inst');
	kTXNWordWrapStateTag = FourCharCode('wwrs');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
	kTXNKeyboardSyncStateTag = FourCharCode('kbsy');
>>>>>>> graemeg/fixes_2_2
=======
	kTXNKeyboardSyncStateTag = FourCharCode('kbsy');
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	kTXNAutoIndentStateTag = FourCharCode('auin');
	kTXNTabSettingsTag = FourCharCode('tabs');
	kTXNRefConTag = FourCharCode('rfcn');
	kTXNMarginsTag = FourCharCode('marg');
	kTXNFlattenMoviesTag = FourCharCode('flat');
	kTXNDoFontSubstitution = FourCharCode('fSub'); { Note: This can degrade performance greatly in the case of large documents }
	kTXNNoUserIOTag = FourCharCode('nuio');

  {
   * In Mac OS X version 10.4 and later this constant is deprecated. 
   * The functions TXNSetEventTarget and TXNGetEventTarget should be
   * used instead.
   }
	kTXNUseCarbonEvents = FourCharCode('cbcb');
	kTXNDrawSelectionWhenInactiveTag = FourCharCode('dsln');
	kTXNDisableDragAndDropTag = FourCharCode('drag');
	kTXNSingleLevelUndoTag = FourCharCode('undo'); { Set this state during creation of the object.  Switching Undo level back and forth is not recommended.}
	kTXNVisibilityTag = FourCharCode('visb'); { Set the visibility state of the object }

  {
   * In Mac OS X version 10.4 or later use this tag to disable and
   * re-enable layout and drawing. It optimizes performance when adding
   * data incrementally to a text object.
   }
	kTXNDisableLayoutAndDrawTag = kTXNVisibilityTag; { Turn on / off layout and drawing. Same as kTXNVisibilityTag but makes clear what the tag does}

  {
   * In Mac OS X version 10.4 or later use this constant to set
   * autoscroll behaviour.  See the discussion below for the enum
   * TXNAutoScrollBehavior for the types of autoscrolling supported.
   }
	kTXNAutoScrollBehaviorTag = FourCharCode('sbev');


{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Convenience Constants for TXNGet/SetTXNControls                                                   }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

const
	kTXNClearThisControl = $FFFFFFFF; { To clear an object control setting }
	kTXNClearTheseFontFeatures = $80000000; { To clear ATSUI font feature(s) }

{ kTXNIOPrivilegesTag}
const
	kTXNReadWrite = false;
	kTXNReadOnly = true;

{ kTXNSelectionStateTag}
const
	kTXNSelectionOn = true;
	kTXNSelectionOff = false;

{ kTXNInlineStateTag}
const
	kTXNUseInline = false;
	kTXNUseBottomline = true;

{ kTXNWordWrapStateTag}
const
	kTXNAutoWrap = false;
	kTXNNoAutoWrap = true;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
{ kTXNKeyboardSyncStateTag}
const
	kTXNSyncKeyboard = false;
	kTXNNoSyncKeyboard = true;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{ kTXNAutoIndentStateTag}
const
	kTXNAutoIndentOff = false;
	kTXNAutoIndentOn = true;

{ kTXNDrawSelectionWhenInactiveTag}
const
	kTXNDontDrawSelectionWhenInactive = false;
	kTXNDrawSelectionWhenInactive = true;

{ kTXNDisableDragAndDropTag}
const
	kTXNEnableDragAndDrop = false;
	kTXNDisableDragAndDrop = true;

{kTXNDisableLayoutAndDraw}
const
	kTXNDisableLayoutAndDraw = true;
	kTXNEnableLayoutAndDraw = false;

type
	TXNControlDataPtr = ^TXNControlData;
	TXNControlData = record
		case SInt16 of
		0: (
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
			uValue: UNSIGNEDLONG;
			);
		1: (
			sValue: SIGNEDLONG;
=======
=======
>>>>>>> origin/fixes_2_2
			uValue: UInt32;
			);
		1: (
			sValue: SInt32;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
			uValue: UNSIGNEDLONG;
			);
		1: (
			sValue: SIGNEDLONG;
>>>>>>> origin/fixes_2.4
			);
		2: (
			tabValue: TXNTab;
			);
		3: (
			marginsPtr: TXNMarginsPtr;
			);
	end;

{
 *  TXNAutoScrollBehavior
 *  
 *  Summary:
 *    Set of values that can be passed to the function
 *    TXNSetTXNObjectControls along with the control tag
 *    kTXNAutoScrollBehaviorTag to control scrolling behavior.
 *  
 *  Discussion:
 *    Beginning with Mac OS X version 10.4, MLTE supports 3 types of
 *    autoscroll behavior.  These are: always scroll a new insertion
 *    into view, only scroll if the insertion point was visible and the
 *    end of the new insertion is out of view, and never auto scroll
 *    under any circumstance.
 }
type
	TXNAutoScrollBehavior = UInt32;
const
{
   * The default auto scrolling behavior. When text is inserted the
   * document is scrolled to show the new insertion.  This was the only
   * type of autoscrolling prior to Mac OS X version 10.4.
   }
	kTXNAutoScrollInsertionIntoView = 0;

  {
   * Never autoscroll.  This includes mouse dragging and text
   * insertion. The only way to scroll the document is for the user to
   * use the scrollbar or to scroll programatically.
   }
	kTXNAutoScrollNever = 1;

  {
   * This type of autoscrolling is best for implementing terminal or
   * log windows. Autoscrolling only happens when the insertion offset
   * is currently in the users view.  This means that if the user is
   * looking at page 1 of a 10 page document and text is inserted at
   * the end of the document than no autoscrolling will occur. However,
   * if the user was looking at page 10 and text was inserted there the
   * document would scroll.
   }
	kTXNAutoScrollWhenInsertionVisible = 2;


{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Offset/Selection Constants                                                                        }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
type
	TXNOffset = UInt32;
	TXNOffsetPtr				= ^TXNOffset; { when a VAR xx: TXNOffset parameter can be nil, it is changed to xx: TXNOffsetPtr }
const
	kTXNUseCurrentSelection = $FFFFFFFF;
	kTXNStartOffset = 0;
	kTXNEndOffset = $7FFFFFFF;

{ Useful for TXNShowSelection API.}
const
	kTXNShowStart = false;
	kTXNShowEnd = true;

{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Resource Constants                                                                                }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{ Useful for saving document.                                                                          }

const
	kTXNSingleStylePerTextDocumentResType = FourCharCode('MPSR');
	kTXNMultipleStylesPerTextDocumentResType = FourCharCode('styl');

{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • URL Constants                                                                                     }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

{
 *  TXNHyperLinkState
 *  
 *  Discussion:
 *    For future use.  Currently not available.
 }
type
	TXNHyperLinkState = UInt32;
const
	kTXNLinkNotPressed = 0;
	kTXNLinkWasPressed = 1;
	kTXNLinkTracking = 3;

{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Type Attributes / ATSUI Features and Variations                                                   }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  Since MLTE currently uses ATSUI by default, the available face types for kTXNQDFontStyleAttribute   }
{  are limited by what's available in ATSUI.  Currently, MLTE supports types defined in MacTypes.h     }
{  which are normal, bold, italic, underline, condensed, and extended.  An alternative is to use       }
{  available QD compatibility tags defined in ATSUnicode.h.                                            }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{  Deprecate all QD TXNTypeRunAttributes/TXNTypeRunAttributeSizes constants. Use kTXNATSUIStyle or     }
{  ATSUI tags ATSUFontTag, kATSUColorTag and kATSUSizeTag to Set/Get the font, color or size           }
type
	TXNTypeRunAttributes = FourCharCode;
const
=======
=======
>>>>>>> origin/fixes_2_2

type
	TXNTypeRunAttributes = FourCharCode;
const
	kTXNQDFontNameAttribute = FourCharCode('fntn');
	kTXNQDFontFamilyIDAttribute = FourCharCode('font');
	kTXNQDFontStyleAttribute = FourCharCode('face');
	kTXNQDFontColorAttribute = FourCharCode('klor');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{  Deprecate all QD TXNTypeRunAttributes/TXNTypeRunAttributeSizes constants. Use kTXNATSUIStyle or     }
{  ATSUI tags ATSUFontTag, kATSUColorTag and kATSUSizeTag to Set/Get the font, color or size           }
type
	TXNTypeRunAttributes = FourCharCode;
const
>>>>>>> origin/fixes_2.4
	kTXNTextEncodingAttribute = FourCharCode('encd');
	kTXNATSUIFontFeaturesAttribute = FourCharCode('atfe');
	kTXNATSUIFontVariationsAttribute = FourCharCode('atva');
	kTXNURLAttribute = FourCharCode('urla');
	kTXNATSUIStyle = FourCharCode('astl');

type
	TXNTypeRunAttributeSizes = ByteCount;
const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kTXNTextEncodingAttributeSize = SizeOf(TextEncoding);
=======
=======
>>>>>>> origin/fixes_2_2
	kTXNQDFontNameAttributeSize = SizeOf(Str255);
	kTXNQDFontFamilyIDAttributeSize = SizeOf(SInt16);
	kTXNQDFontStyleAttributeSize = SizeOf(Style);
	kTXNQDFontColorAttributeSize = SizeOf(RGBColor);
	kTXNTextEncodingAttributeSize = SizeOf(TextEncoding);
	kTXNFontSizeAttributeSize = SizeOf(Fixed);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kTXNTextEncodingAttributeSize = SizeOf(TextEncoding);
>>>>>>> origin/fixes_2.4
	kTXNATSUIStyleSize = SizeOf(ATSUStyle);

type
	TXNATSUIFeaturesPtr = ^TXNATSUIFeatures;
	TXNATSUIFeatures = record
		featureCount: ItemCount;
		featureTypes: ATSUFontFeatureTypePtr;
		featureSelectors: ATSUFontFeatureSelectorPtr;
	end;

type
	TXNATSUIVariationsPtr = ^TXNATSUIVariations;
	TXNATSUIVariations = record
		variationCount: ItemCount;
		variationAxis: ATSUFontVariationAxisPtr;
		variationValues: ATSUFontVariationValuePtr;
	end;

	TXNAttributeDataPtr = ^TXNAttributeData;
	TXNAttributeData = record
		case SInt16 of
		0: (
			dataPtr: UnivPtr;
			);
		1: (
			dataValue: UInt32;
			);
		2: (
			atsuFeatures: TXNATSUIFeaturesPtr;
			);
		3: (
			atsuVariations: TXNATSUIVariationsPtr;
			);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		4: (
			urlReference: CFURLRef;
			);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
		4: (
			urlReference: CFURLRef;
			);
>>>>>>> origin/fixes_2.4
	end;

type
	TXNTypeAttributesPtr = ^TXNTypeAttributes;
	TXNTypeAttributes = record
		tag: TXNTypeRunAttributes;
		size: ByteCount;
		data: TXNAttributeData;
	end;

const
	kTXNDontCareTypeSize = $FFFFFFFF;
	kTXNDontCareTypeStyle = $FF;
	kTXNIncrementTypeSize = $00000001;
	kTXNDecrementTypeSize = $80000000;
	kTXNUseScriptDefaultValue = -1;

  {
   * kTXNNoFontVariations is returned in the dataValue field when the
   * caller as asked to see if the variation is continuous and there
   * was no variation in the continuous range.
   }
	kTXNNoFontVariations = $7FFF;

{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Style Continuous Bits                                                                             }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{ Useful for TXNGetContinuousTypeAttributes API.                                                       }

const
	kTXNFontContinuousBit = 0;
	kTXNSizeContinuousBit = 1;
	kTXNStyleContinuousBit = 2;
	kTXNColorContinuousBit = 3;
	kTXNATSUIStyleContinuousBit = 4;

type
	TXNContinuousFlags = OptionBits;
const
	kTXNFontContinuousMask = 1 shl kTXNFontContinuousBit;
	kTXNSizeContinuousMask = 1 shl kTXNSizeContinuousBit;
	kTXNStyleContinuousMask = 1 shl kTXNStyleContinuousBit;
	kTXNColorContinuousMask = 1 shl kTXNColorContinuousBit;
	kTXNATSUIStyleContinuousMask = 1 shl kTXNATSUIStyleContinuousBit;

{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Match Options Bits                                                                                }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{ Useful for TXNFind API.                                                                              }

const
	kTXNIgnoreCaseBit = 0;
	kTXNEntireWordBit = 1;
	kTXNUseEncodingWordRulesBit = 31;

type
	TXNMatchOptions = OptionBits;
const
	kTXNIgnoreCaseMask = 1 shl kTXNIgnoreCaseBit;
	kTXNEntireWordMask = 1 shl kTXNEntireWordBit;
	kTXNUseEncodingWordRulesMask = 1 shl kTXNUseEncodingWordRulesBit;

type
	TXNMatchTextRecordPtr = ^TXNMatchTextRecord;
	TXNMatchTextRecord = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		iTextPtr: {const} UnivPtr;
		iTextToMatchLength: SIGNEDLONG;
=======
		iTextPtr: UnivPtr;
		iTextToMatchLength: SInt32;
>>>>>>> graemeg/fixes_2_2
=======
		iTextPtr: UnivPtr;
		iTextToMatchLength: SInt32;
>>>>>>> origin/fixes_2_2
=======
		iTextPtr: {const} UnivPtr;
		iTextToMatchLength: SIGNEDLONG;
>>>>>>> origin/fixes_2.4
		iTextEncoding: TextEncoding;
	end;

{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Font Description                                                                                  }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

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
type
	TXNMacOSPreferredFontDescriptionPtr = ^TXNMacOSPreferredFontDescription;
	TXNMacOSPreferredFontDescription = record
		fontID: UInt32;
		pointSize: Fixed;
		encoding: TextEncoding;
		fontStyle: Style;
	end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not TARGET_CPU_64}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {not TARGET_CPU_64}
>>>>>>> origin/fixes_2.4

{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Background                                                                                        }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

{
 *  TXNBackgroundType
 *  
 *  Discussion:
 *    Currently only an RGBColor is supported for the background.
 }
type
	TXNBackgroundType = UInt32;
const
	kTXNBackgroundTypeRGB = 1;


{
 *  TXNBackgroundData
 *  
 *  Discussion:
 *    The TXNBackgroundData is left as a union so that it can be
 *    expanded in the future to support other background types.
 }
type
	TXNBackgroundDataPtr = ^TXNBackgroundData;
	TXNBackgroundData = record
		case SInt16 of
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{
   * Currently only an RGBColor is supported.
   }
		0: (
			color: RGBColor;
=======
		0: (
			color:				RGBColor;
>>>>>>> graemeg/fixes_2_2
=======
		0: (
			color:				RGBColor;
>>>>>>> origin/fixes_2_2
=======
{
   * Currently only an RGBColor is supported.
   }
		0: (
			color: RGBColor;
>>>>>>> origin/fixes_2.4
			);
	end;

type
	TXNBackgroundPtr = ^TXNBackground;
	TXNBackground = record
		bgType: TXNBackgroundType;
		bg: TXNBackgroundData;
	end;


{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Constants passed in iTXNActionName parameter of TXNGet/ClearCountForActionType                    }
{    functions. In Mac OS X version 10.4 and later TXNCountOptions is deprecated.                      }
{    The following action count constants should be used instead.                                      }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

{
 *  kTXNActionCountOfTextChanges
 *  
 *  Discussion:
 *    All text changes other than style changes and custom defined
 *    actions are included in this action count. Includes key presses,
 *    inline sessions, cut/copy/paste, and drop, etc. Undo or redo
 *    events of the kind listed above are also included in this action
 *    count.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
var kTXNActionCountOfTextChanges: CFStringRef; external name '_kTXNActionCountOfTextChanges'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  kTXNActionCountOfStyleChanges
 *  
 *  Discussion:
 *    Any text style change are included in this action count.  Style
 *    changes include changing font/font face/font size/font
 *    feature/font variation/glyph variation/text position, and font
 *    color, etc. Undo or redo events of the kind listed above are also
 *    included in this action count.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
var kTXNActionCountOfStyleChanges: CFStringRef; external name '_kTXNActionCountOfStyleChanges'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  kTXNActionCountOfAllChanges
 *  
 *  Discussion:
 *    A constant used to request the total count of actions including
 *    all text and style changes, as well as custom defined actions.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
var kTXNActionCountOfAllChanges: CFStringRef; external name '_kTXNActionCountOfAllChanges'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Scrolling                                                                                         }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

{
 *  TXNScrollUnit
 *  
 *  Discussion:
 *    Use as a parameter in TXNScroll.
 }
type
	TXNScrollUnit = UInt32;
const
	kTXNScrollUnitsInPixels = 0;
	kTXNScrollUnitsInLines = 1;
	kTXNScrollUnitsInViewRects = 2;


{
 *  TXNScrollBarOrientation
 *  
 *  Discussion:
 *    Use as a parameter in ScrollInfo callback.
 }
type
	TXNScrollBarOrientation = UInt32;
const
	kTXNHorizontal = 0;
	kTXNVertical = 1;


{
 *  TXNScrollBarState
 *  
 *  Discussion:
 *    Use as a parameter in TXNActivate and TXNSetScrollbarState.
 }
type
	TXNScrollBarState = Boolean;
const
	kScrollBarsAlwaysActive = true;
	kScrollBarsSyncWithFocus = false;

{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Draw Item Bits                                                                                    }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  Specifies which element(s) of the text object to render.   Useful in TXNDrawObject API.             }

const
	kTXNDrawItemScrollbarsBit = 0;
	kTXNDrawItemTextBit = 1;
	kTXNDrawItemTextAndSelectionBit = 2;

type
	TXNDrawItems = OptionBits;
const
	kTXNDrawItemScrollbarsMask = 1 shl kTXNDrawItemScrollbarsBit;
	kTXNDrawItemTextMask = 1 shl kTXNDrawItemTextBit;
	kTXNDrawItemTextAndSelectionMask = 1 shl kTXNDrawItemTextAndSelectionBit;
	kTXNDrawItemAllMask = $FFFFFFFF;

{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Rectangle Keys                                                                                    }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  Each key corresponds to a specific bound for the object.  Useful in TXNGetHIRect API.               }

type
	TXNRectKey = UInt32;
const
	kTXNViewRectKey = 0;
	kTXNDestinationRectKey = 1;
	kTXNTextRectKey = 2;
	kTXNVerticalScrollBarRectKey = 3;
	kTXNHorizontalScrollBarRectKey = 4;

{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Carbon Event Info                                                                                 }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

{ Dictionary keys currently supported in the TXNCarbonEventInfo dictionary }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kTXNTextHandlerKey CFSTRP('TextInput')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kTXNWindowEventHandlerKey CFSTRP('WindowEvent')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kTXNWindowResizeEventHandlerKey CFSTRP('WindowResize')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kTXNCommandTargetKey CFSTRP('CommandTarget')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kTXNCommandUpdateKey CFSTRP('CommandUpdate')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
{$definec kTXNFontMenuObjectKey CFSTRP('FontMenuObject')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
>>>>>>> graemeg/fixes_2_2
=======
{$definec kTXNFontMenuObjectKey CFSTRP('FontMenuObject')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{$definec kTXNActionNameMapperKey CFSTRP('ActionNameMapper')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kTXNWheelMouseEventHandlerKey CFSTRP('WheelMouseEvent')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kTXNTSMDocumentAccessHandlerKey CFSTRP('TSMDocumentAccess')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kTXNFontPanelEventHandlerKey CFSTRP('FontPanel')}
{$endc}

{
 *  TXNCarbonEventInfo
 *  
 *  Summary:
 *    Used to pass an EventTargetRef to MLTE via
 *    TXNSetTXNObjectControls API.
 }
type
	TXNCarbonEventInfoPtr = ^TXNCarbonEventInfo;
	TXNCarbonEventInfo = record
{
   * Pass TRUE.  MLTE no longer uses AppleEvents.
   }
		useCarbonEvents: Boolean;

  {
   * Pass NULL.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		filler: UInt8;
=======
		filler: SInt8;
>>>>>>> graemeg/fixes_2_2
=======
		filler: SInt8;
>>>>>>> origin/fixes_2_2
=======
		filler: UInt8;
>>>>>>> origin/fixes_2.4

  {
   * Pass NULL.  AppleEvent flags have been deprecated.
   }
		flags: UInt16;

  {
   * A reference to a Core Foundation dictionary whose keys are the
   * events you want handled and whose values are event target
   * references associated with the events. See the above list of
   * predefined keys you can use to build the dictionary.
   }
		fDictionary: CFDictionaryRef;
	end;


{——————————————————————————————————————————————————————————————————————————————————————————————————————------------}
{  • Definition of dictionary keys for DocumentAttribute dictionary used in TXNWriteRangeToCFURL(), and            }
{    and TXNReadFromCFURL().                                                                                       }
{    When writing data out, document attributes are embedded into the data stream for document formats that        }
{    support them (ie. MLTE native format and RTF); when reading data in document attributes are extracted         }
{    from the data stream if the document format supports them.                                                    }
{———————————————————————————————————————————————————————————————————————————————————————————————————------------———}

{
 *  kTXNDocumentAttributeTitleKey
 *  
 *  Discussion:
 *    CFString containing document title
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
var kTXNDocumentAttributeTitleKey: CFStringRef; external name '_kTXNDocumentAttributeTitleKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  kTXNDocumentAttributeCompanyNameKey
 *  
 *  Discussion:
 *    CFString containing company name
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
var kTXNDocumentAttributeCompanyNameKey: CFStringRef; external name '_kTXNDocumentAttributeCompanyNameKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  kTXNDocumentAttributeSubjectKey
 *  
 *  Discussion:
 *    CFString containing subject
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
var kTXNDocumentAttributeSubjectKey: CFStringRef; external name '_kTXNDocumentAttributeSubjectKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  kTXNDocumentAttributeAuthorKey
 *  
 *  Discussion:
 *    CFString containing author name (not necessarily same as "last
 *    editor," see editor key below)
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
var kTXNDocumentAttributeAuthorKey: CFStringRef; external name '_kTXNDocumentAttributeAuthorKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  kTXNDocumentAttributeKeywordsKey
 *  
 *  Discussion:
 *    CFArray of CFString, containing keywords
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
var kTXNDocumentAttributeKeywordsKey: CFStringRef; external name '_kTXNDocumentAttributeKeywordsKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  kTXNDocumentAttributeCommentKey
 *  
 *  Discussion:
 *    CFString containing comments
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
var kTXNDocumentAttributeCommentKey: CFStringRef; external name '_kTXNDocumentAttributeCommentKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  kTXNDocumentAttributeEditorKey
 *  
 *  Discussion:
 *    CFString containing name of person who last edited the document
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
var kTXNDocumentAttributeEditorKey: CFStringRef; external name '_kTXNDocumentAttributeEditorKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  kTXNDocumentAttributeCreationTimeKey
 *  
 *  Discussion:
 *    CFAbsoluteTime containing document comments; note that this is
 *    not the file system creation date of the file, but of the
 *    document, as it's stored in the document
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
var kTXNDocumentAttributeCreationTimeKey: CFStringRef; external name '_kTXNDocumentAttributeCreationTimeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  kTXNDocumentAttributeModificationTimeKey
 *  
 *  Discussion:
 *    CFAbsoluteTime containing the last modification date of the
 *    document contents
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
var kTXNDocumentAttributeModificationTimeKey: CFStringRef; external name '_kTXNDocumentAttributeModificationTimeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kTXNDocumentAttributeCopyrightKey
 *  
 *  Discussion:
 *    CFString containing the copyright of the document
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
var kTXNDocumentAttributeCopyrightKey: CFStringRef; external name '_kTXNDocumentAttributeCopyrightKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{——————————————————————————————————————————————————————————————————————————————————————————————————————------------}
{  • Key and value definitions for DataOption dictionary used in TXNWriteRangeToCFURL(), and TXNReadFromCFURL().   }
{    Data options are used to specify options for reading in and writing out data.                                 }
{———————————————————————————————————————————————————————————————————————————————————————————————————------------———}

{
 *  kTXNDataOptionDocumentTypeKey
 *  
 *  Discussion:
 *    CFString containing the document format. Supported string values:
 *    kTXNPlainTextDocumentType, kTXNMLTEDocumentType,
 *    kTXNRTFDocumentType, kTXNQuickTimeDocumentType
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
var kTXNDataOptionDocumentTypeKey: CFStringRef; external name '_kTXNDataOptionDocumentTypeKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  kTXNDataOptionCharacterEncodingKey
 *  
 *  Discussion:
 *    CFNumber of type kCFNumberSInt32Type containing the character
 *    encoding as specified in CFString.h and CFStringEncodingExt.h
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
var kTXNDataOptionCharacterEncodingKey: CFStringRef; external name '_kTXNDataOptionCharacterEncodingKey'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{ Document Type constants to be used as values for kTXNDataOptionDocumentTypeKey  }

{
 *  kTXNPlainTextDocumentType
 *  
 *  Discussion:
 *    Plain Text document
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
var kTXNPlainTextDocumentType: CFStringRef; external name '_kTXNPlainTextDocumentType'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  kTXNMLTEDocumentType
 *  
 *  Discussion:
 *    MLTE native document
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
var kTXNMLTEDocumentType: CFStringRef; external name '_kTXNMLTEDocumentType'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  kTXNRTFDocumentType
 *  
 *  Discussion:
 *    RTF document
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
var kTXNRTFDocumentType: CFStringRef; external name '_kTXNRTFDocumentType'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
{
 *  kTXNQuickTimeDocumentType
 *  
 *  Discussion:
 *    Multimedia file that can be opened by QuickTime
 *    importers
 *    <BR>NOTE:Only supported for reading data, not writing.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
var kTXNQuickTimeDocumentType: CFStringRef; external name '_kTXNQuickTimeDocumentType'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======


>>>>>>> graemeg/fixes_2_2
=======


>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Callbacks                                                                                         }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	TXNFindProcPtr = function( const (*var*) matchData: TXNMatchTextRecord; iDataType: TXNDataType; iMatchOptions: TXNMatchOptions; iSearchTextPtr: {const} UnivPtr; encoding: TextEncoding; absStartOffset: TXNOffset; searchTextLength: ByteCount; var oStartMatch: TXNOffset; var oEndMatch: TXNOffset; var ofound: Boolean; refCon: URefCon ): OSStatus;
	TXNActionNameMapperProcPtr = function( actionName: CFStringRef; commandID: UInt32; inUserData: UnivPtr ): CFStringRef;
	TXNContextualMenuSetupProcPtr = procedure( iContextualMenu: MenuRef; objct: TXNObject; inUserData: UnivPtr );
	TXNScrollInfoProcPtr = procedure( iValue: SInt32; iMaximumValue: SInt32; iScrollBarOrientation: TXNScrollBarOrientation; iRefCon: SRefCon );
=======
=======
>>>>>>> origin/fixes_2_2
	TXNFindProcPtr = function( const (*var*) matchData: TXNMatchTextRecord; iDataType: TXNDataType; iMatchOptions: TXNMatchOptions; iSearchTextPtr: {const} UnivPtr; encoding: TextEncoding; absStartOffset: TXNOffset; searchTextLength: ByteCount; var oStartMatch: TXNOffset; var oEndMatch: TXNOffset; var ofound: Boolean; refCon: UInt32 ): OSStatus;
type
=======
	TXNFindProcPtr = function( const (*var*) matchData: TXNMatchTextRecord; iDataType: TXNDataType; iMatchOptions: TXNMatchOptions; iSearchTextPtr: {const} UnivPtr; encoding: TextEncoding; absStartOffset: TXNOffset; searchTextLength: ByteCount; var oStartMatch: TXNOffset; var oEndMatch: TXNOffset; var ofound: Boolean; refCon: URefCon ): OSStatus;
>>>>>>> origin/fixes_2.4
	TXNActionNameMapperProcPtr = function( actionName: CFStringRef; commandID: UInt32; inUserData: UnivPtr ): CFStringRef;
	TXNContextualMenuSetupProcPtr = procedure( iContextualMenu: MenuRef; objct: TXNObject; inUserData: UnivPtr );
<<<<<<< HEAD
type
	TXNScrollInfoProcPtr = procedure( iValue: SInt32; iMaximumValue: SInt32; iScrollBarOrientation: TXNScrollBarOrientation; iRefCon: SInt32 );
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	TXNScrollInfoProcPtr = procedure( iValue: SInt32; iMaximumValue: SInt32; iScrollBarOrientation: TXNScrollBarOrientation; iRefCon: SRefCon );
>>>>>>> origin/fixes_2.4
type
	TXNFindUPP = TXNFindProcPtr;
type
	TXNActionNameMapperUPP = TXNActionNameMapperProcPtr;
type
	TXNContextualMenuSetupUPP = TXNContextualMenuSetupProcPtr;
type
	TXNScrollInfoUPP = TXNScrollInfoProcPtr;
{
 *  NewTXNFindUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewTXNFindUPP( userRoutine: TXNFindProcPtr ): TXNFindUPP; external name '_NewTXNFindUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  NewTXNActionNameMapperUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewTXNActionNameMapperUPP( userRoutine: TXNActionNameMapperProcPtr ): TXNActionNameMapperUPP; external name '_NewTXNActionNameMapperUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  NewTXNContextualMenuSetupUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewTXNContextualMenuSetupUPP( userRoutine: TXNContextualMenuSetupProcPtr ): TXNContextualMenuSetupUPP; external name '_NewTXNContextualMenuSetupUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  NewTXNScrollInfoUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewTXNScrollInfoUPP( userRoutine: TXNScrollInfoProcPtr ): TXNScrollInfoUPP; external name '_NewTXNScrollInfoUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

{
 *  DisposeTXNFindUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeTXNFindUPP( userUPP: TXNFindUPP ); external name '_DisposeTXNFindUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeTXNActionNameMapperUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeTXNActionNameMapperUPP( userUPP: TXNActionNameMapperUPP ); external name '_DisposeTXNActionNameMapperUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  DisposeTXNContextualMenuSetupUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeTXNContextualMenuSetupUPP( userUPP: TXNContextualMenuSetupUPP ); external name '_DisposeTXNContextualMenuSetupUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  DisposeTXNScrollInfoUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeTXNScrollInfoUPP( userUPP: TXNScrollInfoUPP ); external name '_DisposeTXNScrollInfoUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

{
 *  InvokeTXNFindUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function InvokeTXNFindUPP( const (*var*) matchData: TXNMatchTextRecord; iDataType: TXNDataType; iMatchOptions: TXNMatchOptions; iSearchTextPtr: {const} UnivPtr; encoding: TextEncoding; absStartOffset: TXNOffset; searchTextLength: ByteCount; var oStartMatch: TXNOffset; var oEndMatch: TXNOffset; var ofound: Boolean; refCon: URefCon; userUPP: TXNFindUPP ): OSStatus; external name '_InvokeTXNFindUPP';
=======
function InvokeTXNFindUPP( const (*var*) matchData: TXNMatchTextRecord; iDataType: TXNDataType; iMatchOptions: TXNMatchOptions; iSearchTextPtr: {const} UnivPtr; encoding: TextEncoding; absStartOffset: TXNOffset; searchTextLength: ByteCount; var oStartMatch: TXNOffset; var oEndMatch: TXNOffset; var ofound: Boolean; refCon: UInt32; userUPP: TXNFindUPP ): OSStatus; external name '_InvokeTXNFindUPP';
>>>>>>> graemeg/fixes_2_2
=======
function InvokeTXNFindUPP( const (*var*) matchData: TXNMatchTextRecord; iDataType: TXNDataType; iMatchOptions: TXNMatchOptions; iSearchTextPtr: {const} UnivPtr; encoding: TextEncoding; absStartOffset: TXNOffset; searchTextLength: ByteCount; var oStartMatch: TXNOffset; var oEndMatch: TXNOffset; var ofound: Boolean; refCon: UInt32; userUPP: TXNFindUPP ): OSStatus; external name '_InvokeTXNFindUPP';
>>>>>>> origin/fixes_2_2
=======
function InvokeTXNFindUPP( const (*var*) matchData: TXNMatchTextRecord; iDataType: TXNDataType; iMatchOptions: TXNMatchOptions; iSearchTextPtr: {const} UnivPtr; encoding: TextEncoding; absStartOffset: TXNOffset; searchTextLength: ByteCount; var oStartMatch: TXNOffset; var oEndMatch: TXNOffset; var ofound: Boolean; refCon: URefCon; userUPP: TXNFindUPP ): OSStatus; external name '_InvokeTXNFindUPP';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeTXNActionNameMapperUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeTXNActionNameMapperUPP( actionName: CFStringRef; commandID: UInt32; inUserData: UnivPtr; userUPP: TXNActionNameMapperUPP ): CFStringRef; external name '_InvokeTXNActionNameMapperUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  InvokeTXNContextualMenuSetupUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeTXNContextualMenuSetupUPP( iContextualMenu: MenuRef; objct: TXNObject; inUserData: UnivPtr; userUPP: TXNContextualMenuSetupUPP ); external name '_InvokeTXNContextualMenuSetupUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
 *  InvokeTXNScrollInfoUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure InvokeTXNScrollInfoUPP( iValue: SInt32; iMaximumValue: SInt32; iScrollBarOrientation: TXNScrollBarOrientation; iRefCon: SRefCon; userUPP: TXNScrollInfoUPP ); external name '_InvokeTXNScrollInfoUPP';
=======
procedure InvokeTXNScrollInfoUPP( iValue: SInt32; iMaximumValue: SInt32; iScrollBarOrientation: TXNScrollBarOrientation; iRefCon: SInt32; userUPP: TXNScrollInfoUPP ); external name '_InvokeTXNScrollInfoUPP';
>>>>>>> graemeg/fixes_2_2
=======
procedure InvokeTXNScrollInfoUPP( iValue: SInt32; iMaximumValue: SInt32; iScrollBarOrientation: TXNScrollBarOrientation; iRefCon: SInt32; userUPP: TXNScrollInfoUPP ); external name '_InvokeTXNScrollInfoUPP';
>>>>>>> origin/fixes_2_2
=======
procedure InvokeTXNScrollInfoUPP( iValue: SInt32; iMaximumValue: SInt32; iScrollBarOrientation: TXNScrollBarOrientation; iRefCon: SRefCon; userUPP: TXNScrollInfoUPP ); external name '_InvokeTXNScrollInfoUPP';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


{ **************************************************************************************************** }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{                                  •  MLTE APIs •                                                      }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{ **************************************************************************************************** }

{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Creating and Destroying Object                                                                    }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

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
 *  TXNCreateObject()
 *  
 *  Summary:
 *    Creates a new text object of type TXNObject, which is an opaque
 *    structure that handles text formatting.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iFrameRect:
 *      A pointer to a variable of type HIRect. The rectangle is used
 *      to specify the destination and view rectangles for the new MLTE
 *      object. A value of NULL indicates that the rectangle for the
 *      window port will be used as view and destination rectangles
 *      when the object is attached later on to the window. See
 *      TXNAttachObjectToWindowRef below.
 *    
 *    iFrameOptions:
 *      A value of type TXNFrameOptions that specifies the options you
 *      want the object to support. “See Frame Options” in the MLTE
 *      Reference for a description of the options.
 *    
 *    oTXNObject:
 *      A pointer to a structure of type TXNObject. On return, this
 *      points to the opaque text object data structure allocated by
 *      the function. You need to pass this object to most MLTE
 *      functions.
 *  
 *  Result:
 *    A result code indicating success or failure.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function TXNCreateObject( const (*var*) iFrameRect: HIRect; iFrameOptions: TXNFrameOptions; var oTXNObject: TXNObject ): OSStatus; external name '_TXNCreateObject';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  TXNDeleteObject()
 *  
 *  Summary:
 *    Delete a previously allocated TXNObject and all associated data
 *    structures. If the frameType is multiple frames all frames are
 *    released.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.  The text
 *      object to free.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
procedure TXNDeleteObject( iTXNObject: TXNObject ); external name '_TXNDeleteObject';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  TXNInitTextension()
 *  
 *  Summary:
 *    Initialize the Textension library.  Should be called as soon as
 *    possible after the Macintosh toolbox is initialized.  On Mac OS X
 *    version 10.3 or later, it's not necessary to call this routine.
 *    The cases where you may want to call this routine are: 1) A set
 *    of default fonts different from the system default is desired. 2)
 *    Want to have multimedia support 3) Want to use QuickdrawText
 *    instead of ATSUI to render the text.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iDefaultFonts:
 *      A table of font information including fontFamily ID, point
 *      size, style, and script code. The table can be NULL or can have
 *      an entry for any script for which you would like to to
 *      designate a default font.  Only a valid script number is
 *      required.  You can designate that Textension should use the
 *      default for a give script by setting the field to
 *      kTXNUseScriptDefaultValue (-1).
 *    
 *    iCountDefaultFonts:
 *      Count of entries in the iDefaultFonts parameter.
 *    
 *    iUsageFlags:
 *      Specify whether multimeida should be supported.
 *  
 *  Result:
 *    A result code indicating success or failure. Various MacOS errors
 *    are possible if something is wrong.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNInitTextension( {const} iDefaultFonts: {variable-size-array} TXNMacOSPreferredFontDescriptionPtr { can be NULL }; iCountDefaultFonts: ItemCount; iUsageFlags: TXNInitOptions ): OSStatus; external name '_TXNInitTextension';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  TXNVersionInformation()
 *  
 *  Summary:
 *    Get the version number and a set of feature bits. 
 *    TXNVersionValue uses a NumVersion structure. See MacTypes.h for
 *    the format of the version.  Currently there are two feature bits:
 *     one for ATSUI default, another one for CarbonEvent default.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    oFeatureFlags:
 *      Pointer to a bit mask.  See TXNFeatureMask enum above. If
 *      kTXNWillDefaultToATSUIBit is set it means that by default MLTE
 *      will use ATSUI to image and measure text and will default to
 *      using Unicode to store characters.
 *  
 *  Result:
 *    TXNVersionValue: Current version.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNVersionInformation( var oFeatureFlags: TXNFeatureBits ): TXNVersionValue; external name '_TXNVersionInformation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Set/Get Window Associated with the Object                                                         }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{
 *  TXNAttachObjectToWindowRef()
 *  
 *  Summary:
 *    Attaches a text object to a window.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.  The text
 *      object you want to attach to the input window.
 *    
 *    iWindowRef:
 *      A WindowRef for the window you want to attach the object to.
 *  
 *  Result:
 *    A result code indicating success or failure.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function TXNAttachObjectToWindowRef( iTXNObject: TXNObject; iWindowRef: WindowRef ): OSStatus; external name '_TXNAttachObjectToWindowRef';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  TXNGetWindowRef()
 *  
 *  Summary:
 *    Returns the window that the input object is attached to.
 *  
 *  Discussion:
 *    If no window is attached to the object it returns NULL.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.  The text
 *      object you want to attach to the input window.
 *  
 *  Result:
 *    The windowRef for the current window attached to the the text
 *    object.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function TXNGetWindowRef( iTXNObject: TXNObject ): WindowRef; external name '_TXNGetWindowRef';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Events                                                                                            }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

{
 *  TXNKeyDown()
 *  
 *  Summary:
 *    Process a keydown event. Note that if CJK script is installed and
 *    current font is CJK inline input will take place. This is always
 *    the case unless the application has requested the bottomline
 *    window or has turned off TSM (see initialization options above).
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque struct to apply keydown to.
 *    
 *    iEvent:
 *      The keydown event.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
procedure TXNKeyDown( iTXNObject: TXNObject; const (*var*) iEvent: EventRecord ); external name '_TXNKeyDown';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  TXNAdjustCursor()
 *  
 *  Summary:
 *    Handle switching the cursor.  If over text area set to i-beam. 
 *    Over graphics, sound, movie, scrollbar or outside of window set
 *    to arrow.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque struct obtained from TXNCreateObject.
 *    
 *    ioCursorRgn:
 *      Region to be passed to WaitNextEvent.  Resized accordingly by
 *      TXNAdjustCursor.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
procedure TXNAdjustCursor( iTXNObject: TXNObject; ioCursorRgn: RgnHandle ); external name '_TXNAdjustCursor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  TXNClick()
 *  
 *  Summary:
 *    Process click in content region.  Takes care of scrolling,
 *    selecting text,  playing sound and movies, drag & drop, and
 *    double-clicks.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque struct obtained from TXNCreateObject.
 *    
 *    iEvent:
 *      The mousedown event.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
procedure TXNClick( iTXNObject: TXNObject; const (*var*) iEvent: EventRecord ); external name '_TXNClick';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  TXNSelectAll()
 *  
 *  Summary:
 *    Selects everything in a frame.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque struct obtained from TXNCreateObject.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
procedure TXNSelectAll( iTXNObject: TXNObject ); external name '_TXNSelectAll';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  TXNFocus()
 *  
 *  Summary:
 *    Focus the TXNObject.  Scrollbars and insertion point are made
 *    active  if iBecomingFocused is true, and inactive if false.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque struct obtained from TXNCreateObject.
 *    
 *    iBecomingFocused:
 *      true if becoming active.  false otherwise.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
procedure TXNFocus( iTXNObject: TXNObject; iBecomingFocused: Boolean ); external name '_TXNFocus';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  TXNUpdate()
 *  
 *  Summary:
 *    Handle update event (i.e. draw everything in a frame.) This
 *    function calls the Toolbox BeginUpdate - EndUpdate functions for
 *    the window that was passed to TXNCreateObject.  This makes it
 *    inappropriate for windows that contain something else besides the
 *    TXNObject.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque struct obtained from TXNCreateObject.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
procedure TXNUpdate( iTXNObject: TXNObject ); external name '_TXNUpdate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  TXNDrawObject()
 *  
 *  Summary:
 *    Renders the current content of the TXNObject on the screen.
 *  
 *  Discussion:
 *    Redraws the object element(s) specified in the TXNDrawItems
 *    flags.  Drawing is limited to the input clip rectangle.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject. Draw into this
 *      text object.
 *    
 *    iClipRect:
 *      A pointer to a HIRect. If the rectangle is NULL, MLTE uses its
 *      view rectangle when drawing. If the rectangle is not NULL, MLTE
 *      will intersect iClipRect with the view rectangle to determine
 *      the rectangle to draw.  MLTE will not draw in area not covered
 *      by the port's clip region.  Therefore, a given clipRect larger
 *      than the port's clip region will be trimmed down.
 *    
 *    iDrawItems:
 *      A value of type TXNDrawItems. Indicates what element(s) of the
 *      object are to be drawn.
 *  
 *  Result:
 *    A result code indicating success or failure.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function TXNDrawObject( iTXNObject: TXNObject; const (*var*) iClipRect: HIRect; iDrawItems: TXNDrawItems ): OSStatus; external name '_TXNDrawObject';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  TXNForceUpdate()
 *  
 *  Summary:
 *    Force a frame to be updated.  Very much like toolbox call
 *    InvalRect.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
procedure TXNForceUpdate( iTXNObject: TXNObject ); external name '_TXNForceUpdate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  TXNGetSleepTicks()
 *  
 *  Summary:
 *    Depending on state of window get the appropriate sleep time to be
 *    passed to WaitNextEvent.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *  
 *  Result:
 *    A UInt32 value of the appropriate sleep time.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNGetSleepTicks( iTXNObject: TXNObject ): UInt32; external name '_TXNGetSleepTicks';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  TXNIdle()
 *  
 *  Summary:
 *    Do necessary Idle time processing. Typically flash the cursor. If
 *    a TextService is active pass a NULL event to the Text Service so
 *    it gets  time.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
procedure TXNIdle( iTXNObject: TXNObject ); external name '_TXNIdle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  TXNGrowWindow()
 *  
 *  Summary:
 *    Handle mouse-down in grow region.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    iEvent:
 *      The mousedown event
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
procedure TXNGrowWindow( iTXNObject: TXNObject; const (*var*) iEvent: EventRecord ); external name '_TXNGrowWindow';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  TXNZoomWindow()
 *  
 *  Summary:
 *    Handle mouse-down in zoom.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    iPart:
 *      Value returned by FindWindow
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
procedure TXNZoomWindow( iTXNObject: TXNObject; iPart: SInt16 ); external name '_TXNZoomWindow';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Redo/Undo                                                                                         }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

{
 *  TXNBeginActionGroup()
 *  
 *  Summary:
 *    Starts an action group. Every supported edit action after
 *    TXNBeginActionGroup is called is added to the group until
 *    TXNEndActionGroup (see below) is called. When MLTE receives an
 *    undo/redo command, it will treat all actions added to the group
 *    as a single operation to undo/redo. Nesting of groups is not
 *    allowed. Calling TXNBeginActionGroup twice without calling
 *    TXNEndActionGroup in between will result in an error.
 *    TXNCanUndoAction and TXNCanRedoAction return false if there is an
 *    active action group.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    iActionGroupName:
 *      A client supplied string used to describe the action group.
 *  
 *  Result:
 *    An operating system status code. The error
 *    kTXNOperationNotAllowedErr is returned if an undo action group
 *    has already been started but not terminated.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function TXNBeginActionGroup( iTXNObject: TXNObject; iActionGroupName: CFStringRef ): OSStatus; external name '_TXNBeginActionGroup';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  TXNEndActionGroup()
 *  
 *  Summary:
 *    Ends the current action group. The call is ignored is there is no
 *    active action group.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function TXNEndActionGroup( iTXNObject: TXNObject ): OSStatus; external name '_TXNEndActionGroup';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  TXNCanUndoAction()
 *  
 *  Summary:
 *    Tells the client whether the last action is undoable or not, and
 *    if requested it returns a string that identifies the last action.
 *    Usually, this function is used by clients to determine whether
 *    the Undo item in the Edit menu should be enabled or not, and to
 *    obtain the action name that should be used in that item. When the
 *    last action is an action group (See
 *    TXNBeginActionGroup/TXNEndActionGroup), the the string used to
 *    name the group is returned. If you have asked MLTE to handling
 *    updating for the Redo and Undo edit commands you should call
 *    TXNSetActionNameMapper after calling TXNCanUndoAction so that
 *    MLTE can callback to you to get the correct strings for those
 *    items.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    oActionName:
 *      Pointer to the string that denotes the last action. Pass in
 *      NULL if the string is not needed. The returned string is either
 *      a string defined by MLTE, or a string passed by the client to
 *      the TXNBeginActionGroup when a new action group is created. The
 *      client is responsible to retain and release the string.
 *  
 *  Result:
 *    Returns a Boolean value. If true, the last action is undoable,
 *    and the Undo item in the Edit menu, if there is one, should be
 *    enabled. If false, the last action cannot be undone and Undo item
 *    in the Edit menu should be grayed out.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function TXNCanUndoAction( iTXNObject: TXNObject; var oActionName: CFStringRef ): Boolean; external name '_TXNCanUndoAction';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  TXNCanRedoAction()
 *  
 *  Summary:
 *    Tells the client whether the current item on the undo stack is
 *    redoable or not. Usually, this function is used by clients to
 *    determine whether the Redo item in the Edit menu should be
 *    enabled or not, and to obtain the action name that should be used
 *    in that item. When the current undo item is an action group (See
 *    TXNBeginActionGroup/TXNEndActionGroup), the string used to name
 *    the group is returned.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    oActionName:
 *      Pointer to the string that denotes the current item in the undo
 *      stack. Pass in NULL if the string is not needed. The returned
 *      string is either a string defined by MLTE, or the string passed
 *      by the client to TXNBeginActionGroup when a new action group is
 *      created. The client is responsible to retain and release the
 *      string.
 *  
 *  Result:
 *    Returns a Boolean value. If true, the last action is redoable,
 *    and the Redo item in the Edit menu, if there is one, should be
 *    enabled. If false, the last action cannot be redone, and the Redo
 *    item in the Edit menu should be grayed out.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function TXNCanRedoAction( iTXNObject: TXNObject; var oActionName: CFStringRef ): Boolean; external name '_TXNCanRedoAction';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  TXNSetActionNameMapper()
 *  
 *  Summary:
 *    Provides MLTE with a callback that is used to obtain the
 *    appropriate localized string, which represent either a single
 *    action or an action group (See
 *    TXNBeginActionGroup/TXNEndActionGroup), for updating the Redo and
 *    Undo items in the Edit menu.
 *  
 *  Discussion:
 *    If you have asked MLTE to handling updating for the Redo and Undo
 *    edit commands you should call this function so that MLTE can
 *    callback to you to get the correct strings for those items. When
 *    MLTE's handler for kEventClassCommand/kEventCommandUpdateStatus
 *    is called for the Redo or Undo command MLTE will check to see if
 *    a TXNActionNameMapperProc has been installed. If it has the is
 *    called to get the correct string to update the menu item.  The
 *    client can used the action name and the command ID to determine
 *    the appropriate string.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      The TXNObject.
 *    
 *    iStringForKeyProc:
 *      The callback.
 *    
 *    iUserData:
 *      A pointer to anything.  Of course its a good idea to point at
 *      something that will help you map the action key to a string.
 *  
 *  Result:
 *    OSStatus.  noErr if the operation is successful.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function TXNSetActionNameMapper( iTXNObject: TXNObject; iStringForKeyProc: TXNActionNameMapperUPP; iUserData: {const} UnivPtr ): OSStatus; external name '_TXNSetActionNameMapper';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  TXNUndo()
 *  
 *  Summary:
 *    Undo the last command.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
procedure TXNUndo( iTXNObject: TXNObject ); external name '_TXNUndo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  TXNRedo()
 *  
 *  Summary:
 *    Redo the last command.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
procedure TXNRedo( iTXNObject: TXNObject ); external name '_TXNRedo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  TXNClearUndo()
 *  
 *  Summary:
 *    Purge the undo stack
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   in Textension not yet available
 }
function TXNClearUndo( iTXNObject: TXNObject ): OSStatus; external name '_TXNClearUndo';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Editing                                                                                           }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

{
 *  TXNCut()
 *  
 *  Summary:
 *    Cut the current selection to the clipboard.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *  
 *  Result:
 *    A result code indicating success or failure.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNCut( iTXNObject: TXNObject ): OSStatus; external name '_TXNCut';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  TXNCopy()
 *  
 *  Summary:
 *    Copy current selection to the clipboard.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *  
 *  Result:
 *    A result code indicating success or failure.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNCopy( iTXNObject: TXNObject ): OSStatus; external name '_TXNCopy';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  TXNPaste()
 *  
 *  Summary:
 *    Paste from the clipboard.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *  
 *  Result:
 *    A result code indicating success or failure.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNPaste( iTXNObject: TXNObject ): OSStatus; external name '_TXNPaste';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  TXNClear()
 *  
 *  Summary:
 *    Clear the current selection.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *  
 *  Result:
 *    A result code indicating success or failure.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNClear( iTXNObject: TXNObject ): OSStatus; external name '_TXNClear';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  TXNIsScrapPastable()
 *  
 *  Summary:
 *    Test to see if the current scrap contains data that is supported
 *    by Textension.  Used to determine if Paste item in Edit menu
 *    should be active or inactive. The types of data supported depends
 *    on what data types were specified in the TXNInitTextension
 *    options.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Result:
 *    Boolean: True if data type in Clipboard is supported.  False if
 *    not a supported data type.  If result is True the Paste item in
 *    the menu can be highlighted.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNIsScrapPastable: Boolean; external name '_TXNIsScrapPastable';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Selection                                                                                         }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

{
 *  TXNGetSelection()
 *  
 *  Summary:
 *    Get the absolute offsets of the current selection.  Embedded
 *    graphics, sound, etc. each count as one character.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    oStartOffset:
 *      Absolute beginning of the current selection.
 *    
 *    oEndOffset:
 *      End of current selection.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
procedure TXNGetSelection( iTXNObject: TXNObject; var oStartOffset: TXNOffset; var oEndOffset: TXNOffset ); external name '_TXNGetSelection';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  TXNShowSelection()
 *  
 *  Summary:
 *    Scroll the current selection into view.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    iShowEnd:
 *      If true the end of the selection is scrolled into view. If
 *      false the beginning of selection is scrolled into view.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
procedure TXNShowSelection( iTXNObject: TXNObject; iShowEnd: Boolean ); external name '_TXNShowSelection';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  TXNShowOffset()
 *  
 *  Summary:
 *    Scroll the text at a specified offset into view.
 *  
 *  Discussion:
 *    TXNShowOffset can be used to reveal the text at any offset.
 *    TXNShowSelection, in contrast, can only reveal the current
 *    selection.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    inOffset:
 *      The offset which will be scrolled into view.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
procedure TXNShowOffset( iTXNObject: TXNObject; inOffset: TXNOffset ); external name '_TXNShowOffset';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  TXNIsSelectionEmpty()
 *  
 *  Summary:
 *    Call to find out if the current selection is empty. Use this to
 *    determine if Paste, Cut, Copy, Clear should be highlighted in
 *    Edit menu.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *  
 *  Result:
 *    Boolean: True if current selection is empty (i.e. start offset ==
 *    end offset). False if selection is not empty.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNIsSelectionEmpty( iTXNObject: TXNObject ): Boolean; external name '_TXNIsSelectionEmpty';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  TXNSetSelection()
 *  
 *  Summary:
 *    Set the current selection.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    iStartOffset:
 *      New beginning.
 *    
 *    iEndOffset:
 *      New end.
 *  
 *  Result:
 *    A result code indicating success or failure.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNSetSelection( iTXNObject: TXNObject; iStartOffset: TXNOffset; iEndOffset: TXNOffset ): OSStatus; external name '_TXNSetSelection';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Set/Get Type Attributes                                                                           }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

{
 *  TXNGetContinuousTypeAttributes()
 *  
 *  Summary:
 *    Test the current selection to see if type size, style, color
 *    and/or font are continuous. That is is the current selection made
 *    up of one font, one font size, one Style, and/or one color.  On
 *    return examine the flags to see if the attributes specified were
 *    continuous.  If an attribute is continuous then the dataValue
 *    field in the TXNTypeAttributes can be examined to get the
 *    continous value.  Remember that for color you pass a ptr to an
 *    RGBColor in attr[0].data.dataPtr.
 *  
 *  Discussion:
 *    If examining kTXNATSUIStyleContinuous bit, be sure to call
 *    ATSUDisposeStyle to dispose the style that is returned from MLTE.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    oContinuousFlags:
 *      Bits which can be examined to see if type size, style, color,
 *      and/or font are continuous. Example: if ( 
 *      TXNGetContinuousTypeAttributes( txnObject, &flags, 1, &attr )
 *      == noErr ) ( if ( flags & kTXNFontContinuousMask ) ....check a
 *      font name
 *    
 *    iCount:
 *      Count of TXNTypeAttributes records in the ioTypeAttributes
 *      array.
 *    
 *    ioTypeAttributes:
 *      Array of TXNTypeAttributes that indicate the type attributes
 *      the caller is interested in. For example: 1. if you wanted to
 *      know if the current selection was continuous in terms of being
 *      all one same font size you could do something like this.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      TXNTypeAttributes       attr[1] = ( kTXNQDFontSizeAttribute,
=======
 *      TXNTypeAttributes       attr[1] = ( TXNFontSizeAttribute,
>>>>>>> graemeg/fixes_2_2
=======
 *      TXNTypeAttributes       attr[1] = ( TXNFontSizeAttribute,
>>>>>>> origin/fixes_2_2
=======
 *      TXNTypeAttributes       attr[1] = ( kTXNQDFontSizeAttribute,
>>>>>>> origin/fixes_2.4
 *      sizeof(Fixed),( 0 ) ) on return from the function if size is
 *      continuous (i.e. if the bit 3 of flags is set) then the third
 *      field (attr[0].data.dataValue) will contain the size of the
 *      font as a Fixed value. 2. if you wanted to know if the current
 *      selection was continuous in ATSUI style you could do something
 *      like this. TXNTypeAttributes       attr[1] = ( kTXNATSUIStyle,
 *      kTXNATSUIStyleSize, ( 0 ) ) on return from the function if
 *      ATSUI style is continuous, then the third field
 *      (attr[0].data.dataPtr) will contain the ATSUI style.
 *  
 *  Result:
 *    A result code indicating success or failure.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNGetContinuousTypeAttributes( iTXNObject: TXNObject; var oContinuousFlags: TXNContinuousFlags; iCount: ItemCount; ioTypeAttributes: {variable-size-array} TXNTypeAttributesPtr { can be NULL } ): OSStatus; external name '_TXNGetContinuousTypeAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  TXNSetTypeAttributes()
 *  
 *  Summary:
 *    Set the current ranges font information.  Values are passed in
 *    the attributes array. Values <= sizeof(UInt32) are passed by
 *    value. > sizeof(UInt32) are passed as a pointer. That is the
 *    TXNTypeAttributes' 3rd field is a union that servers as either a
 *    32-bit integer where values can be written or a 32-bit pointer a
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    value.  Functional in NoUserIO mode. When you call
 *    TXNSetTypeAttributes, any attributes that you do not set retain
 *    their previous values.
=======
 *    value.  Functional in NoUserIO mode.
>>>>>>> graemeg/fixes_2_2
=======
 *    value.  Functional in NoUserIO mode.
>>>>>>> origin/fixes_2_2
=======
 *    value.  Functional in NoUserIO mode. When you call
 *    TXNSetTypeAttributes, any attributes that you do not set retain
 *    their previous values.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    iAttrCount:
 *      Count of type attributes in the TXNTypeAttributes array.
 *    
 *    iAttributes:
 *      Attributes that caller would like to set.
 *    
 *    iStartOffset:
 *      Start of the range where text attributes should be changed.
 *    
 *    iEndOffset:
 *      End of the range.
 *  
 *  Result:
 *    A result code indicating success or failure.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNSetTypeAttributes( iTXNObject: TXNObject; iAttrCount: ItemCount; {const} iAttributes: {variable-size-array} TXNTypeAttributesPtr; iStartOffset: TXNOffset; iEndOffset: TXNOffset ): OSStatus; external name '_TXNSetTypeAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Set/Get Object Controls                                                                           }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

{
 *  TXNSetTXNObjectControls()
 *  
 *  Summary:
 *    Sets formatting and privileges attributes (such as justification,
 *    line direction, tab values, and read-only status) that apply to
 *    the entire text object.
 *  
 *  Discussion:
 *    On systems that use Apple Type Services for Unicode Imaging
 *    (ATSUI), the ATSUI line control attribute tags can be passed to
 *    this function in the iControlTag parameter. This is the case for
 *    all the ATSUI tags except kATSULineRotationTag. ATSUI tags are
 *    applied to the entire text object.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      The text object that identifies the document for which you want
 *      to set formatting and privileges attributes.
 *    
 *    iClearAll:
 *      A Boolean value. If you set this to true, all formatting and
 *      privileges attributes are reset to their default value. That
 *      is, true clears existing tags and resets each to its default
 *      value.  This can be done even when the object is in NoUserIO
 *      mode.
 *    
 *    iControlCount:
 *      The number of items in the iControlTags array.
 *    
 *    iControlTags:
 *      An array of values that specifies kind of data that is passed
 *      in the iControlData parameter. See “Formatting and Privileges
 *      Settings” for a description of possible values. On systems that
 *      use Apple Type Services for Unicode Imaging (ATSUI), you can
 *      also pass ATSUI attribute tag constants. See the ATSUI
 *      documentation for a description of the ATSUI constants. Can be
 *      NULL if iClearAll is true.
 *    
 *    iControlData:
 *      An array of TXNControlData unions that contain the information
 *      your application wants to set. The value you supply to the
 *      iControlTags parameter specifies how the union of type
 *      TXNControlData is treated. You must make sure that the value
 *      you assign to the iControlData parameter is the appropriate
 *      type implied by the value you passed in the iControlTags
 *      parameter. Can be NULL if iClearAll is true.
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNSetTXNObjectControls( iTXNObject: TXNObject; iClearAll: Boolean; iControlCount: ItemCount; {const} iControlTags: {variable-size-array} TXNControlTagPtr { can be NULL }; {const} iControlData: {variable-size-array} TXNControlDataPtr { can be NULL } ): OSStatus; external name '_TXNSetTXNObjectControls';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  TXNGetTXNObjectControls()
 *  
 *  Summary:
 *    Gets the current formatting and privileges attributes (such as
 *    justification, line direction, tab values, and read-only status)
 *    for a text object.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      The text object that identifies the document to be activated.
 *      If NULL then the default value for an MLTE object is returned.
 *    
 *    iControlCount:
 *      The number of items in the iControlTags array.
 *    
 *    iControlTags:
 *      An array of values that specify the kind of formatting
 *      information you want returned in the oControlData array. See
 *      “Formatting and Privileges Settings” for a description of
 *      possible values.
 *    
 *    oControlData:
 *      An array of TXNControlData unions. On return, the array
 *      contains the information that was requested through the
 *      iControlTags array. Your application must allocate the
 *      oControlData array.
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNGetTXNObjectControls( iTXNObject: TXNObject { can be NULL }; iControlCount: ItemCount; {const} iControlTags: {variable-size-array} TXNControlTagPtr; oControlData: {variable-size-array} TXNControlDataPtr ): OSStatus; external name '_TXNGetTXNObjectControls';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Other Settings                                                                                    }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

{
 *  TXNSetBackground()
 *  
 *  Summary:
 *    Set the type of background the TXNObject's text, etc. is drawn
 *    onto.  At this point the background can be a color.  Functional
 *    in NoUserIO mode.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    iBackgroundInfo:
 *      Struct containing information that describes the background.
 *  
 *  Result:
 *    A result code indicating success or failure.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNSetBackground( iTXNObject: TXNObject; const (*var*) iBackgroundInfo: TXNBackground ): OSStatus; external name '_TXNSetBackground';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  TXNEchoMode()
 *  
 *  Summary:
 *    Put the TXNObject into echo mode. What that means is that all
 *    characters in the TXNObject have the character 'echoCharacter'
 *    substituted for the actual glyph when drawing occurs.
 *  
 *  Discussion:
 *    Note that the echoCharacter is typed as a UniChar, but this is
 *    done merely to facilitate passing any 2 byte character.  The
 *    encoding parameter actually determines the encoding used to
 *    locate a font and display a character.  Thus if you wanted to
 *    display the diamond found in the Shift-JIS encoding for MacOS you
 *    would pass in 0x86A6 for the character but an encoding that was
 *    built to represent the MacOS Japanese encoding.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    iEchoCharacter:
 *      Character to use in substitution.
 *    
 *    iEncoding:
 *      Encoding from which character is drawn.
 *    
 *    iOn:
 *      True if turning EchoMode on.  False if turning it off.
 *  
 *  Result:
 *    A result code indicating success or failure.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNEchoMode( iTXNObject: TXNObject; iEchoCharacter: UniChar; iEncoding: TextEncoding; iOn: Boolean ): OSStatus; external name '_TXNEchoMode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Retrieve Run Info                                                                                 }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

{
 *  TXNCountRunsInRange()
 *  
 *  Summary:
 *    Given a range specified by the starting and ending offset return
 *    a count of the runs in that range.  Run in this case means
 *    changes in TextSyles or a graphic or sound.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    iStartOffset:
 *      Start of range.
 *    
 *    iEndOffset:
 *      End of range.
 *    
 *    oRunCount:
 *      Count of runs in the range
 *  
 *  Result:
 *    A result code indicating success or failure.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNCountRunsInRange( iTXNObject: TXNObject; iStartOffset: TXNOffset; iEndOffset: TXNOffset; var oRunCount: ItemCount ): OSStatus; external name '_TXNCountRunsInRange';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  TXNGetIndexedRunInfoFromRange()
 *  
 *  Summary:
 *    Gets information about a run in a range of data.
 *  
 *  Discussion:
 *    You should first call the TXNCountRunsInRange function to get the
 *    count. The TXNTypeAttributes structure must specify the text
 *    attribute in which the application is interested. In other words,
 *    the tag field must be set. If you asked for the kTXNATSUIStyle
 *    info, you are now responsible for disposing the ATSUI style
 *    returned from the attribute array by calling ATSUDisposeStyle.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      The text object for the current text area.
 *    
 *    iIndex:
 *      The value that corresponds to the run for which you want to get
 *      information. You call the TXNCountRunsInRange function to get
 *      the number of runs in a range. The iIndex parameter is
 *      zero-based, so its possible values are from 0 to the number of
 *      runs in a range minus 1.
 *    
 *    iStartOffset:
 *      The offset at which you want to start to obtain run information.
 *    
 *    iEndOffset:
 *      The offset at which you want run information to end.
 *    
 *    oRunStartOffset:
 *      On return, a pointer to a value that identifies the start of
 *      run relative to the beginning of the text, not the beginning of
 *      the range you specified in the iStartOffset parameter.
 *    
 *    oRunEndOffset:
 *      On return, a pointer to a value that identifies the end of the
 *      run relative to the beginning of the text, not the beginning of
 *      the range you specified in the iStartOffset parameter.
 *    
 *    oRunDataType:
 *      On return, a pointer to a value that identifies the type of
 *      data in the run. See “Supported Data Types” for a description
 *      of possible values.
 *    
 *    iTypeAttributeCount:
 *      The number of font attributes.
 *    
 *    ioTypeAttributes:
 *      A pointer to a structure of type TXNTypeAttributes. On input,
 *      you specify the attribute (such as size) in the tag field and
 *      the attribute size in the size field. You can pass NULL for the
 *      data field. On return, the data field contains the attribute
 *      data. The data field is a union that serves either as a 32-bit
 *      integer or a 32-bit pointer, depending on the size field.
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNGetIndexedRunInfoFromRange( iTXNObject: TXNObject; iIndex: ItemCount; iStartOffset: TXNOffset; iEndOffset: TXNOffset; oRunStartOffset: TXNOffsetPtr { can be NULL }; oRunEndOffset: TXNOffsetPtr { can be NULL }; oRunDataType: TXNDataTypePtr { can be NULL }; iTypeAttributeCount: ItemCount; ioTypeAttributes: TXNTypeAttributesPtr { can be NULL } ): OSStatus; external name '_TXNGetIndexedRunInfoFromRange';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Get Data Size                                                                                     }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

{
 *  TXNDataSize()
 *  
 *  Summary:
 *    Return the size in bytes of the characters in a given TXNObject.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *  
 *  Result:
 *    The bytes required to hold the characters.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNDataSize( iTXNObject: TXNObject ): ByteCount; external name '_TXNDataSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Read and Write APIs                                                                           }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}


{
 *  TXNWriteRangeToCFURL()
 *  
 *  Summary:
 *    Write a range of a TXNObject to a CFURLRef.
 *  
 *  Discussion:
 *    Write a range of a text object to a file or a special file bundle
 *    (directory). It supports different document formats and encodings
 *    which can be specified in the data options dictionary. Clients
 *    can specify additional document attributes when data is written
 *    out using a file format that supports such attributes (ie. RTF
 *    and native MLTE file format). See DocumentAttribute key
 *    definitions for additional information on the attributes
 *    supported.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    iStartOffset:
 *      The initial offset in iTXNObject to write out to iFileURL.
 *    
 *    iEndOffset:
 *      The last offset in iTXNObject to write out to iFileURL.
 *    
 *    iDataOptions:
 *      A CFDictionaryRef that specifies options for writing out the
 *      data. See Data Options key documentation for a list of the
 *      options supported. If this parameter is NULL, the data is
 *      written out using MLTE's native format.
 *    
 *    iDocumentAttributes:
 *      Specifies the document attributes to be embedded in the data
 *      stream. This param is only supported when writing out the data
 *      using one of the following formats: RTF and MLTE native format.
 *      Only the key / values defined in Document Attributes are
 *      written out. The content of the dictionary is ignored for any
 *      other format. If the dictionary is NULL, no attributes are
 *      added to the data stream.
 *    
 *    iFileURL:
 *      Should point to an existing file or directory whichever is
 *      correct for file type. On exit, iFileURL will contain a copy of
 *      the data in the given range for the iTXNObject with the format
 *      and encoding specified by iDataOptions.
 *  
 *  Result:
 *    A result code indicating success or failure.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function TXNWriteRangeToCFURL( iTXNObject: TXNObject; iStartOffset: TXNOffset; iEndOffset: TXNOffset; iDataOptions: CFDictionaryRef { can be NULL }; iDocumentAttributes: CFDictionaryRef { can be NULL }; iFileURL: CFURLRef ): OSStatus; external name '_TXNWriteRangeToCFURL';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  TXNReadFromCFURL()
 *  
 *  Summary:
 *    Read data from a CFURLRef into a TXNObject.
 *  
 *  Discussion:
 *    Read in data from a file or or special file bundle (directory)
 *    into a text object. Offset parameters are used to specify whether
 *    the new data is inserted, appended or replaces an existing data
 *    range in the text object. Clients can specify the document format
 *    and encoding of the data using the DataOptions dictionary. This
 *    functions also returns the document attributes present in the
 *    data stream. Document attributes are only supported for the rich
 *    text file formats supported by MLTE: RTF and MLTE native file
 *    format.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject. New data will
 *      be added to this object.
 *    
 *    iStartOffset:
 *      The initial offset in iTXNObject to place the data read in from
 *      iFileURL.
 *    
 *    iEndOffset:
 *      The last offset in iTXNObject to place the data read in from
 *      iFileURL.
 *    
 *    iDataOptions:
 *      A CFDictionaryRef that specifies options for reading in the
 *      data. See Data Options key documentation for a list of the
 *      options supported. If this parameter is NULL, the data is
 *      written out using MLTE's native format.
 *    
 *    iFileURL:
 *      A value of type CFURLRef. The data to be added to the
 *      iTXNObject.
 *    
 *    oDocumentAttributes:
 *      A value of type CFDictionaryRef. It contains the document
 *      attributes for the text object. On exit, this dictionary
 *      contains the document attributes present in the data stream, if
 *      the file format supports them; otherwise it will be NULL. The
 *      native MLTE file format and RTF support embedded document
 *      attributes. See the DocumentAttribute key documentation for a
 *      list of the attributes supported. If this parameter is NULL, no
 *      document attributes are written out. Sample code: If the caller
 *      passes a pointer to a dictionary ref, the API will return a ref
 *      to the dictionary of attributes if there is one (caller must
 *      release dictionary), otherwise the API will set the ref to NULL
 *      in all other cases. CFDictionaryRef oDocumentAttributes = NULL;
 *      status = TXNReadFromCFURL (...., &oDocumentAttributes); if
 *      (oDocumentAttributes != NULL) ::CFRelease(oDocumentAttributes);
 *  
 *  Result:
 *    A result code indicating success or failure.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function TXNReadFromCFURL( iTXNObject: TXNObject; iStartOffset: TXNOffset; iEndOffset: TXNOffset; iDataOptions: CFDictionaryRef { can be NULL }; iFileURL: CFURLRef; oDocumentAttributes: CFDictionaryRefPtr { can be NULL } ): OSStatus; external name '_TXNReadFromCFURL';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  TXNCopyTypeIdentifiersForRange()
 *  
 *  Summary:
 *    Returns an array of univeral type identifiers (UTIs) that can be
 *    used to write out a range of a text object with no information
 *    loss.
 *  
 *  Discussion:
 *    Some file formats support limited embedding of data when writing
 *    out to disk, and use attachments instead, such as RTF. This API
 *    can be used to determine the document format to use with
 *    TXNWriteRangeToCFURL - i.e., whether to use RTF (Rich Text
 *    Format). Note that support for new document formats could be
 *    added in the future.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    iStartOffset:
 *      The initial offset in iTXNObject for the range to check.
 *    
 *    iEndOffset:
 *      The last offset in iTXNObject for the range to check.
 *    
 *    oTypeIdentifiersForRange:
 *      A pointer to a CFArrayRef. On exit, the array will contain the
 *      list of universal type identifiers (UTI) that MLTE supports,
 *      and that can be used to write the object out to disk with no
 *      data loss. Each entry in the array is a CFStringRef.
 *  
 *  Result:
 *    A result code indicating success or failure.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function TXNCopyTypeIdentifiersForRange( iTXNObject: TXNObject; iStartOffset: TXNOffset; iEndOffset: TXNOffset; var oTypeIdentifiersForRange: CFArrayRef ): OSStatus; external name '_TXNCopyTypeIdentifiersForRange';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  TXNGetData()
 *  
 *  Summary:
 *    Copy the data in the range specified by startOffset and endOffset.
 *  
 *  Discussion:
 *    This function should be used in conjunction with TXNNextDataRun. 
 *    The client would call TXNCountRunsInRange to the number of data
 *    runs in a given range.  The client can then walk the runs with
 *    the function TXNGetIndexedRunInfoFromRange. 
 *    TXNGetIndexedRunInfoFromRange lets you examine each runs type and
 *    text attributes. For each data run of interest (i.e. one whose
 *    data the caller wanted to look at) the client would call
 *    TXNGetData. The handle passed to TXNGetData should not be
 *    allocated. TXNGetData takes care of allocating the dataHandle as
 *    necessary.  However, the caller is responsible for disposing the
 *    handle.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    iStartOffset:
 *      Absolute offset from which data copy should begin.
 *    
 *    iEndOffset:
 *      Absolute offset at which data copy should end.
 *    
 *    oDataHandle:
 *      If noErr a new handle containing the requested data. The caller
 *      is responsible for disposing the handle.  Note that the handle
 *      is a copy so it can be safely disposed at any time.
 *  
 *  Result:
 *    Memory errors or TXN_IllegalToCrossDataBoundaries if offsets
 *    specify a range that crosses a data type boundary.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNGetData( iTXNObject: TXNObject; iStartOffset: TXNOffset; iEndOffset: TXNOffset; var oDataHandle: Handle ): OSStatus; external name '_TXNGetData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  TXNGetDataEncoded()
 *  
 *  Summary:
 *    Copy the data in the range specified by startOffset and endOffset.
 *  
 *  Discussion:
 *    The handle passed to TXNGetDataEncoded should not be allocated.
 *    TXNGetData takes care of allocating the dataHandle as necessary. 
 *    However, the caller is responsible for disposing the handle.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    iStartOffset:
 *      Absolute offset from which data copy should begin.
 *    
 *    iEndOffset:
 *      Absolute offset at which data copy should end.
 *    
 *    oDataHandle:
 *      If noErr a new handle containing the requested data.
 *    
 *    iEncoding:
 *      should be kTXNTextData or kTXNUnicodeTextData.
 *  
 *  Result:
 *    Memory errors or  TXN_IllegalToCrossDataBoundaries if offsets
 *    specify a range that crosses a data type boundary.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNGetDataEncoded( iTXNObject: TXNObject; iStartOffset: TXNOffset; iEndOffset: TXNOffset; var oDataHandle: Handle; iEncoding: TXNDataType ): OSStatus; external name '_TXNGetDataEncoded';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  TXNSetData()
 *  
 *  Summary:
 *    Replaces a range of data (text, graphics, and so forth).
 *  
 *  Discussion:
 *    Functional in NoUserIO mode.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      The text object that identifies the document in which you want
 *      to replace data.
 *    
 *    iDataType:
 *      The type of the replacement data. See “Supported Data Types”
 *      for a description of possible values.
 *    
 *    iDataPtr:
 *      A pointer to the data that will replace the data that is in the
 *      range specified by the iStartOffset and iEndOffset parameters. 
 *      Can be NULL if the start and end offsets are different.
 *    
 *    iDataSize:
 *      The size of the data to which iDataPtr points.
 *    
 *    iStartOffset:
 *      The beginning of the range of data to replace. You can use the
 *      TXNGetSelection function to get the absolute offsets of the
 *      current selection.
 *    
 *    iEndOffset:
 *      The end of the range to replace. You can use the
 *      TXNGetSelection function to get the absolute offsets of the
 *      current selection. If you want to insert text, the ending and
 *      starting offsets should be the same value.
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNSetData( iTXNObject: TXNObject; iDataType: TXNDataType; iDataPtr: {const} UnivPtr { can be NULL }; iDataSize: ByteCount; iStartOffset: TXNOffset; iEndOffset: TXNOffset ): OSStatus; external name '_TXNSetData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  TXNFlattenObjectToCFDataRef()
 *  
 *  Summary:
 *    Flattens a text object so it can be saved to disk or embedded
 *    with other data.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.  Retrieve
 *      flattened data from this text object.
 *    
 *    iTXNDataType:
 *      A value of type TXNDataType that specifies the format in which
 *      the data is written out.
 *    
 *    oDataRef:
 *      A pointer to a structure of type CFDataRef. On return the data
 *      will contain a flattened version of the iTXNObject in the
 *      format specified by iTXNDataType. Clients are responsible to
 *      retain the returned CFDataRef.
 *  
 *  Result:
 *    A result code indicating success or failure.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function TXNFlattenObjectToCFDataRef( iTXNObject: TXNObject; iTXNDataType: TXNDataType; var oDataRef: CFDataRef ): OSStatus; external name '_TXNFlattenObjectToCFDataRef';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Revert                                                                                            }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

{
 *  TXNRevert()
 *  
 *  Summary:
 *    Revert  to the last saved version of this document.  If the file
 *    was not previously saved the document is reverted to an empty
 *    document.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *  
 *  Result:
 *    A result code indicating success or failure. (such as File
 *    Manager errors)
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNRevert( iTXNObject: TXNObject ): OSStatus; external name '_TXNRevert';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Printing                                                                                          }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

{
 *  TXNPageSetup()
 *  
 *  Summary:
 *    Display the Page Setup dialog of the current default printer and
 *    react to any changes (i.e. Reformat the text if the page layout
 *    changes.)
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *  
 *  Result:
 *    A result code indicating success or failure. ( such as Print
 *    Manager errors )
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNPageSetup( iTXNObject: TXNObject ): OSStatus; external name '_TXNPageSetup';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  TXNPrint()
 *  
 *  Summary:
 *    Print the document.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *  
 *  Result:
 *    A result code indicating success or failure. ( such as Print
 *    Manager errors )
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNPrint( iTXNObject: TXNObject ): OSStatus; external name '_TXNPrint';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Search                                                                                            }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

{
 *  TXNFind()
 *  
 *  Summary:
 *    Find a piece of text or a graphics object.
 *  
 *  Discussion:
 *    The default matching behavior is pretty simple for Text a basic
 *    binary compare is done.  If the matchOptions say to ignore case
 *    the characters to be searched are duplicated and case
 *    neutralized. This naturally can fail due to lack of memory if
 *    there is a large amount of text.  It also slows things down.  If
 *    MatchOptions say find an entire word that once a match is found
 *    an effort is made to determine if the match is a word.  The
 *    default behavior is to test the character before and after the to
 *    see if it is White space.  If the kTXNUseEncodingWordRulesBit is
 *    set than the Script Manager's FindWord function is called to make
 *    this determination. If the caller is looking for a non-text type
 *    than each non-text type in the document is returned. If more
 *    elaborate ( a regular expression processor or whatever ) is what
 *    you want then that is what the FindProc is for.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    iMatchTextDataPtr:
 *      Ptr to a MatchTextRecord which contains the text to match, the
 *      length of that text and the TextEncoding the text is encoded
 *      in.  This must be there if you are looking for Text, but can be
 *      NULL if you are looking for a graphics object.
 *    
 *    iDataType:
 *      The type of data to find.  This can be any of the types defined
 *      in TXNDataType enum (TEXT, PICT, moov, snd ).  However, if
 *      PICT, moov, or snd is passed then the default behavior is to
 *      match on any non-Text object.  If you really want to find a
 *      specific type you can provide a custom find callback or ignore
 *      matches which aren't the precise type you are interested in.
 *    
 *    iMatchOptions:
 *      Options on what to search for.
 *    
 *    iStartSearchOffset:
 *      The offset at which a search should begin. The constant
 *      kTXNStartOffset specifies the start of the objects data.
 *    
 *    iEndSearchOffset:
 *      The offset at which the search should end. The constant
 *      kTXNEndOffset specifies the end of the objects data.
 *    
 *    iFindProc:
 *      A custom callback.  If will be called to match things rather
 *      than the default matching behavior.
 *    
 *    iRefCon:
 *      This can be use for whatever the caller likes.  It is passed to
 *      the FindProc (if a FindProc is provided.
 *    
 *    oStartMatchOffset:
 *      Absolute offset to start of match.  set to 0xFFFFFFFF if not
 *      match.
 *    
 *    oEndMatchOffset:
 *      Absolute offset to end of match.  Set to 0xFFFFFFFF is no match.
 *  
 *  Result:
 *    A result code indicating success or failure.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNFind( iTXNObject: TXNObject; {const} iMatchTextDataPtr: TXNMatchTextRecordPtr { can be NULL }; iDataType: TXNDataType; iMatchOptions: TXNMatchOptions; iStartSearchOffset: TXNOffset; iEndSearchOffset: TXNOffset; iFindProc: TXNFindUPP; iRefCon: SRefCon; var oStartMatchOffset: TXNOffset; var oEndMatchOffset: TXNOffset ): OSStatus; external name '_TXNFind';
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNFind( iTXNObject: TXNObject; {const} iMatchTextDataPtr: TXNMatchTextRecordPtr { can be NULL }; iDataType: TXNDataType; iMatchOptions: TXNMatchOptions; iStartSearchOffset: TXNOffset; iEndSearchOffset: TXNOffset; iFindProc: TXNFindUPP; iRefCon: SInt32; var oStartMatchOffset: TXNOffset; var oEndMatchOffset: TXNOffset ): OSStatus; external name '_TXNFind';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNFind( iTXNObject: TXNObject; {const} iMatchTextDataPtr: TXNMatchTextRecordPtr { can be NULL }; iDataType: TXNDataType; iMatchOptions: TXNMatchOptions; iStartSearchOffset: TXNOffset; iEndSearchOffset: TXNOffset; iFindProc: TXNFindUPP; iRefCon: SRefCon; var oStartMatchOffset: TXNOffset; var oEndMatchOffset: TXNOffset ): OSStatus; external name '_TXNFind';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————————————————————}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{  • Static Text Box                                                                                   }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

{
 *  TXNDrawUnicodeTextBox()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use HIThemeDrawTextBox, CoreText (CTLineDraw,..) or Cocoa
 *    (NSStringDrawing,..)
 *  
 *  Summary:
 *    Draws a Unicode string in the specified rectangle.
 *  
 *  Discussion:
 *    Client is supposed to do an EraseRect if needed. The drawing will
 *    be clipped to the rect unless the client specifies a rotation.
 *    Use kTXNUseVerticalTextMask to display text vertically (no need
 *    to use the kTXNRotateTextMask flag in this case).
=======
=======
>>>>>>> origin/fixes_2_2
{  • Font Defaults                                                                                     }
=======
{  • Static Text Box                                                                                   }
>>>>>>> origin/fixes_2.4
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

{
 *  TXNDrawUnicodeTextBox()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use HIThemeDrawTextBox, CoreText (CTLineDraw,..) or Cocoa
 *    (NSStringDrawing,..)
 *  
 *  Summary:
<<<<<<< HEAD
 *    For a given TXNObject specify the font defaults for each script.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Draws a Unicode string in the specified rectangle.
 *  
 *  Discussion:
 *    Client is supposed to do an EraseRect if needed. The drawing will
 *    be clipped to the rect unless the client specifies a rotation.
 *    Use kTXNUseVerticalTextMask to display text vertically (no need
 *    to use the kTXNRotateTextMask flag in this case).
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *    iText:
 *      Ptr to a Unicode string (UTF16 chars).
 *    
 *    iLen:
 *      Number of UniChars in iText (this is not the size of iText).
<<<<<<< HEAD
 *    
 *    ioBox:
 *      On input the text box where the text will be displayed. On
 *      return will be updated to reflect the minimum bounding Rect
 *      that will enclose the text (unless kTXNDontUpdateBoxRectMask is
 *      used).
 *    
 *    iStyle:
 *      Style to use to display the text.
 *    
 *    iOptions:
 *      Can be used to specify non-default behavior.
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in Textension 1.1 and later
 }
function TXNDrawUnicodeTextBox( {const} iText: {variable-size-array} UniCharPtr; iLen: UniCharCount; var ioBox: Rect; iStyle: ATSUStyle { can be NULL }; {const} iOptions: TXNTextBoxOptionsDataPtr { can be NULL } ): OSStatus; external name '_TXNDrawUnicodeTextBox';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  TXNDrawCFStringTextBox()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use HIThemeDrawTextBox, CoreText (CTLineDraw,..) or Cocoa
 *    (NSStringDrawing,..)
 *  
 *  Summary:
 *    Draws a CFString in the specified rectangle.
 *  
 *  Discussion:
 *    Client is supposed to do an EraseRect if needed. The drawing will
 *    be clipped to the rect unless the client specifies a rotation.
 *    Use kTXNUseVerticalTextMask to display text vertically (no need
 *    to use the kTXNRotateTextMask flag in this case).
=======
=======
>>>>>>> origin/fixes_2_2
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    iCount:
 *      Count of FontDescriptions.
=======
>>>>>>> origin/fixes_2.4
 *    
 *    ioBox:
 *      On input the text box where the text will be displayed. On
 *      return will be updated to reflect the minimum bounding Rect
 *      that will enclose the text (unless kTXNDontUpdateBoxRectMask is
 *      used).
 *    
 *    iStyle:
 *      Style to use to display the text.
 *    
 *    iOptions:
 *      Can be used to specify non-default behavior.
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in Textension 1.1 and later
 }
function TXNDrawUnicodeTextBox( {const} iText: {variable-size-array} UniCharPtr; iLen: UniCharCount; var ioBox: Rect; iStyle: ATSUStyle { can be NULL }; {const} iOptions: TXNTextBoxOptionsDataPtr { can be NULL } ): OSStatus; external name '_TXNDrawUnicodeTextBox';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  TXNDrawCFStringTextBox()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use HIThemeDrawTextBox, CoreText (CTLineDraw,..) or Cocoa
 *    (NSStringDrawing,..)
 *  
 *  Summary:
 *    Draws a CFString in the specified rectangle.
 *  
 *  Discussion:
<<<<<<< HEAD
 *    To determine how many font descriptions need to be in the array
 *    you should call this function with a NULL for the array.  iCount
 *    will return with the number of font defaults currently stored.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Client is supposed to do an EraseRect if needed. The drawing will
 *    be clipped to the rect unless the client specifies a rotation.
 *    Use kTXNUseVerticalTextMask to display text vertically (no need
 *    to use the kTXNRotateTextMask flag in this case).
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    iText:
 *      A CFStringRef (see CFBase.h and CFString.h).
=======
=======
>>>>>>> origin/fixes_2_2
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    ioCount:
 *      Count of FontDescriptions in the array.
 *    
 *    oFontDefaults:
 *      Array of FontDescriptins to be filled out.
 *  
 *  Result:
 *    A result code indicating success or failure.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNGetFontDefaults( iTXNObject: TXNObject; var ioCount: ItemCount; oFontDefaults: {variable-size-array} TXNMacOSPreferredFontDescriptionPtr { can be NULL } ): OSStatus; external name '_TXNGetFontDefaults';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Font Menu                                                                                         }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

{
 *  TXNNewFontMenuObject()
 *  
 *  Summary:
 *    Get a FontMenuObject.  Caller can extract a fontmenu from this
 *    object and pass this object to the active TXNObject to handle
 *    events in the font menu.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iFontMenuHandle:
 *      An empty menu handle (well the title is there) that the caller
 *      created via NewMenu or GetNewMenu. This menu handle should not
 *      be disposed before the returned TXNFontMenuObject has been
 *      disposed via TXNDisposeFontMenuObject.
 *    
 *    iMenuID:
 *      The MenuID for iFontMenuHandle.
 *    
 *    iStartHierMenuID:
 *      The first MenuID to use if any hierarchical menus need to be
 *      created. TXNNewFontMenuObject uses SetMenuItemHierarchicalID
 *      when creating hierarchial menus.  The iStartHierMenuID must
 *      therefor follow the rules for this function.  On systems less
 *      than system 8.5 the submenuID must be less than 255.  For
 *      systems above system 8.5 the range can be as large can be as
 *      large 32767.  However, it is important to remember that
 *      TXNNewFontMenuObject only uses iStartHierMenuID as a starting
 *      id when adding hierarchical menus.  Therefore provide plenty of
 *      room to increment this value. For example, on a system less
 *      than 8.5 it would be good to start at 175.  On systems greater
 *      than 8.5 it is probably a good idea to not use a value higher
 *      than 32000.
 *    
 *    oTXNFontMenuObject:
 *      A font menu object.
 *  
 *  Result:
 *    A result code indicating success or failure.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNNewFontMenuObject( iFontMenuHandle: MenuRef; iMenuID: SInt16; iStartHierMenuID: SInt16; var oTXNFontMenuObject: TXNFontMenuObject ): OSStatus; external name '_TXNNewFontMenuObject';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  TXNGetFontMenuHandle()
 *  
 *  Summary:
 *    Get the MenuRef from the TXNFontMenuObject.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNFontMenuObject:
 *      A Font Menu Object obtained from TXNNewFontMenuObject.
 *    
 *    oFontMenuHandle:
 *      The returned font menu. Returned value could be NULL.
 *  
 *  Result:
 *    A result code indicating success or failure.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNGetFontMenuHandle( iTXNFontMenuObject: TXNFontMenuObject; var oFontMenuHandle: MenuRef ): OSStatus; external name '_TXNGetFontMenuHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


function TXNGetFontMenuRef( iTXNFontMenuObject: TXNFontMenuObject; var oFontMenuHandle: MenuRef ): OSStatus; external name '_TXNGetFontMenuHandle';


{
 *  TXNDisposeFontMenuObject()
 *  
 *  Summary:
 *    Dispose a TXNFontMenuObject and its font menu handle.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNFontMenuObject:
 *      A Font Menu Object obtained from TXNNewFontMenuObject.
 *  
 *  Result:
 *    A result code indicating success or failure.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNDisposeFontMenuObject( iTXNFontMenuObject: TXNFontMenuObject ): OSStatus; external name '_TXNDisposeFontMenuObject';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  TXNDoFontMenuSelection()
 *  
 *  Summary:
 *    Given the menuID and menu item returned by MenuSelect determine
 *    the selected font and change the current selection to be that
 *    Font.  If the input TXNObject is not active a parameter error is
 *    returned.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    iTXNFontMenuObject:
 *      A Font Menu Object obtained from TXNNewFontMenuObject.
 *    
 *    iMenuID:
 *      SInt16 the ID of the selected menu.
 *    
 *    iMenuItem:
 *      The item that was selected.
 *  
 *  Result:
 *    A result code indicating success or failure.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNDoFontMenuSelection( iTXNObject: TXNObject; iTXNFontMenuObject: TXNFontMenuObject; iMenuID: SInt16; iMenuItem: SInt16 ): OSStatus; external name '_TXNDoFontMenuSelection';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  TXNPrepareFontMenu()
 *  
 *  Summary:
 *    Prepares a Font menu for display.
 *  
 *  Discussion:
 *    You should call the TXNPrepareFontMenu function just before your
 *    application opens the Font menu for your user. If the text
 *    object’s current selection is a single font, MLTE places a
 *    checkmark next to the menu item for that font.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      The text object that identifies the document with the Font menu
 *      you want to prepare. Pass NULL to display an inactive menu
 *      (dimmed).
 *    
 *    iTXNFontMenuObject:
 *      A Font menu object.
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNPrepareFontMenu( iTXNObject: TXNObject { can be NULL }; iTXNFontMenuObject: TXNFontMenuObject ): OSStatus; external name '_TXNPrepareFontMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Static Text Box                                                                                   }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

{
 *  TXNDrawUnicodeTextBox()
 *  
 *  Summary:
 *    Draws a Unicode string in the specified rectangle.
 *  
 *  Discussion:
 *    Client is supposed to do an EraseRect if needed. The drawing will
 *    be clipped to the rect unless the client specifies a rotation.
 *    Use kTXNUseVerticalTextMask to display text vertically (no need
 *    to use the kTXNRotateTextMask flag in this case).
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iText:
 *      Ptr to a Unicode string (UTF16 chars).
 *    
 *    iLen:
 *      Number of UniChars in iText (this is not the size of iText).
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 *    
=======
 *    iText:
 *      A CFStringRef (see CFBase.h and CFString.h).
 *    
>>>>>>> origin/fixes_2.4
 *    ioBox:
 *      On input the text box where the text will be displayed. On
 *      return will be updated to reflect the minimum bounding Rect
 *      that will enclose the text (unless kTXNDontUpdateBoxRectMask is
 *      used).
 *    
 *    iStyle:
 *      Style to use to display the text.
 *    
 *    iOptions:
 *      Can be used to specify non-default behavior.
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.6
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in Textension 1.1 and later
 }
function TXNDrawUnicodeTextBox( {const} iText: {variable-size-array} UniCharPtr; iLen: UniCharCount; var ioBox: Rect; iStyle: ATSUStyle { can be NULL }; {const} iOptions: TXNTextBoxOptionsDataPtr { can be NULL } ): OSStatus; external name '_TXNDrawUnicodeTextBox';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  TXNDrawCFStringTextBox()
 *  
 *  Summary:
 *    Draws a CFString in the specified rectangle.
 *  
 *  Discussion:
 *    Client is supposed to do an EraseRect if needed. The drawing will
 *    be clipped to the rect unless the client specifies a rotation.
 *    Use kTXNUseVerticalTextMask to display text vertically (no need
 *    to use the kTXNRotateTextMask flag in this case).
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iText:
 *      A CFStringRef (see CFBase.h and CFString.h).
 *    
 *    ioBox:
 *      On input the text box where the text will be displayed. On
 *      return will be updated to reflect the minimum bounding Rect
 *      that will enclose the text (unless kTXNDontUpdateBoxRectMask is
 *      used).
 *    
 *    iStyle:
 *      Style to use to display the text.
 *    
 *    iOptions:
 *      Can be used to specify non-default behavior.
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function TXNDrawCFStringTextBox( iText: CFStringRef; var ioBox: Rect; iStyle: ATSUStyle { can be NULL }; {const} iOptions: TXNTextBoxOptionsDataPtr { can be NULL } ): OSStatus; external name '_TXNDrawCFStringTextBox';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4


{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Get Line Info                                                                                     }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

{
 *  TXNGetLineCount()
 *  
 *  Summary:
 *    Get the total number of lines in the TXNObject.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    oLineTotal:
 *      On return the total number of lines in the object.
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in Textension 1.1 and later
 }
function TXNGetLineCount( iTXNObject: TXNObject; var oLineTotal: ItemCount ): OSStatus; external name '_TXNGetLineCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  TXNGetLineMetrics()
 *  
 *  Summary:
 *    Get the metrics for the specified line.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    iLineNumber:
 *      The line we want the metrics for (0 based).
 *    
 *    oLineWidth:
 *      On return the width of the line (in Fixed format).
 *    
 *    oLineHeight:
 *      On return the height (ascent + descent) of the line (in Fixed
 *      format).
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in Textension 1.1 and later
 }
function TXNGetLineMetrics( iTXNObject: TXNObject; iLineNumber: UNSIGNEDLONG; var oLineWidth: Fixed; var oLineHeight: Fixed ): OSStatus; external name '_TXNGetLineMetrics';
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in Textension 1.1 and later
 }
function TXNGetLineMetrics( iTXNObject: TXNObject; iLineNumber: UInt32; var oLineWidth: Fixed; var oLineHeight: Fixed ): OSStatus; external name '_TXNGetLineMetrics';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in Textension 1.1 and later
 }
function TXNGetLineMetrics( iTXNObject: TXNObject; iLineNumber: UNSIGNEDLONG; var oLineWidth: Fixed; var oLineHeight: Fixed ): OSStatus; external name '_TXNGetLineMetrics';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Count Changes                                                                                     }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

{
 *  TXNGetChangeCount()
 *  
 *  Summary:
 *    Retrieve number of times document has been changed.
 *  
 *  Discussion:
 *    That is for every committed command (keydown, cut, copy) the
 *    value returned is count of those. This is useful for deciding if 
 *    the Save item in the File menu should be active.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *  
 *  Result:
 *    ItemCount: count of changes.  This is total changes since
 *    document  was created or last saved.  Not count since this
 *    routine was last called or anything like that.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNGetChangeCount( iTXNObject: TXNObject ): ItemCount; external name '_TXNGetChangeCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  TXNGetCountForActionType()
 *  
 *  Summary:
 *    Retrieves the number of times a given kind of action has
 *    occurred. There are three MLTE defined counts:
 *    kTXNActionCountOfTextChanges, kTXNActionCountOfStyleChanges and
 *    kTXNActionCountOfAllChanges. In addition, this API can retrieve
 *    the number of times a custom action has occurred. See
 *    TXNBeginActionGroup for how to define a custom action group.
 *  
 *  Discussion:
 *    Call TXNClearCountForActionType to reset the counters.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    iActionTypeName:
 *      Specifies the kind of action changes to be included when
 *      retrieving the count. The value can be
 *      kTXNActionCountOfTextChanges, kTXNActionCountOfStyleChanges,
 *      kTXNActionCountforAllChanges, or a string passed to
 *      TXNBeginActionGroup
 *    
 *    oCount:
 *      The number of times the iActionGroupName action has occurred.
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function TXNGetCountForActionType( iTXNObject: TXNObject; iActionTypeName: CFStringRef; var oCount: ItemCount ): OSStatus; external name '_TXNGetCountForActionType';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  TXNClearCountForActionType()
 *  
 *  Summary:
 *    Reset the counter(s) for the specified kind of action(s) to zero.
 *  
 *  Discussion:
 *    Use kTXNActionCountOfAllChanges to reset all counters.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    iActionTypeName:
 *      Specifies the kind of action changes for which the counter
 *      should be set to zero. The value can be
 *      kTXNActionCountOfTextChanges, kTXNActionCountOfStyleChanges,
 *      kTXNActionCountforAllChanges or any string used to form an
 *      action group with TXNBeginActionGroup.
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function TXNClearCountForActionType( iTXNObject: TXNObject; iActionTypeName: CFStringRef ): OSStatus; external name '_TXNClearCountForActionType';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Set/Get Object Bounds                                                                             }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

{
 *  TXNSetHIRectBounds()
 *  
 *  Summary:
 *    Sets the text object's view, the destination rectangles or both.
 *  
 *  Discussion:
 *    Either of the input rectangle can be NULL. HIRect provides an
 *    uniform interface to the HIView coordinate system.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.  The bounds for
 *      this text object will be set.
 *    
 *    iViewRect:
 *      A pointer to a HIRect data structure that contains the new
 *      coordinates for the view rectangle. Pass NULL if you don’t want
 *      to change the view rectangle.
 *    
 *    iDestinationRect:
 *      A pointer to a HIRect data structure that contains the new
 *      coordinates for the destination rectangle. Pass NULL if you
 *      don’t want to change the destination rectangle.
 *    
 *    iUpdate:
 *      Pass true if you want the location of the text and scrollbars
 *      to be recalculated and redrawn, otherwise pass false.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
procedure TXNSetHIRectBounds( iTXNObject: TXNObject; {const} iViewRect: HIRectPtr; {const} iDestinationRect: HIRectPtr; iUpdate: Boolean ); external name '_TXNSetHIRectBounds';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  TXNGetHIRect()
 *  
 *  Summary:
 *    Gets one of the text object's bounds based on the specified
 *    TXNRectKey
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject. The specified
 *      bounds for this text object will be returned.
 *    
 *    iTXNRectKey:
 *      The value for the type of rectangle you want the function to
 *      return.
 *    
 *    oRectangle:
 *      On return, a pointer to the HIRect data structure that contains
 *      the coordinates for the requested rectangle. If a rect is not
 *      defined, a pointer to an empty rect will be returned. Note that
 *      only scrollbar rectangle may be undefined for the text object.
 *  
 *  Result:
 *    A result code indicating success or failure.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function TXNGetHIRect( iTXNObject: TXNObject; iTXNRectKey: TXNRectKey; var oRectangle: HIRect ): OSStatus; external name '_TXNGetHIRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  TXNResizeFrame()
 *  
 *  Summary:
 *    Changes the frame's size to match the new width and height.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    iWidth:
 *      New width in pixels.
 *    
 *    iHeight:
 *      New height in pixels.
 *    
 *    iTXNFrameID:
 *      Deprecated. Pass 0.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
procedure TXNResizeFrame( iTXNObject: TXNObject; iWidth: UInt32; iHeight: UInt32; iTXNFrameID: TXNFrameID ); external name '_TXNResizeFrame';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  TXNSetFrameBounds()
 *  
 *  Summary:
 *    Changes the frame's bounds to match the Rect.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    iTop:
 *      Top of the bounds.
 *    
 *    iLeft:
 *      Left of the bounds.
 *    
 *    iBottom:
 *      Bottom of the bounds.
 *    
 *    iRight:
 *      Right of the bounds.
 *    
 *    iTXNFrameID:
 *      Deprecated. Pass 0.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
procedure TXNSetFrameBounds( iTXNObject: TXNObject; iTop: SInt32; iLeft: SInt32; iBottom: SInt32; iRight: SInt32; iTXNFrameID: TXNFrameID ); external name '_TXNSetFrameBounds';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  TXNGetViewRect()
 *  
 *  Summary:
 *    Get the rectangle describing the current view into the document.
 *    The coordinates of this rectangle will be local to the the window.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    oViewRect:
 *      The requested view rectangle.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
procedure TXNGetViewRect( iTXNObject: TXNObject; var oViewRect: Rect ); external name '_TXNGetViewRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Layout Calculation                                                                                }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

{
 *  TXNRecalcTextLayout()
 *  
 *  Summary:
 *    Recalculates the text layout based on the new View and
 *    Destination rectangles.
 *  
 *  Discussion:
 *    Call this if you called TXNSetRectBounds with the iUpdate
 *    parameter set to false. It will also recalcuate where the
 *    scrollbars, if any, should be placed. Finally an update event
 *    will be generated so that the TXNObject is redrawn.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.1 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.1 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.1 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
procedure TXNRecalcTextLayout( iTXNObject: TXNObject ); external name '_TXNRecalcTextLayout';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Scrolling                                                                                         }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

{
 *  TXNScroll()
 *  
 *  Discussion:
 *    TXNScroll scrolls the text within a view rectangle of the
 *    specified object by the designated number of units.  For example,
 *    you might want to scroll the text in an object in response to
 *    user input in a control other than the standard scrollbars that
 *    MLTE supplies.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    iVerticalScrollUnit:
 *      Specifies what units the values in ioVerticalDelta are in.  If
 *      iVerticalScrollUnit is equal to kTXNScrollUnitsInPixels the
 *      value is treated as pixels.  If the value is
 *      kTXNScrollUnitsInLines the value is treated as a count of
 *      lines. Note that using this value is the slowest because each
 *      line must be measured before it scrolls.  Finally if
 *      kTXNScrollUnitsInViewRects the value is treated as the height
 *      of the current viewRect.
 *    
 *    iHorizontalScrollUnit:
 *      Specifies what units the values in iDh are in.  If
 *      iHorizontalScrollUnit is equal to kTXNScrollUnitsInPixels the
 *      value is treated as pixels.  If the value is
 *      kTXNScrollUnitsInLines the value is treated as a count of
 *      lines. Note that using this value for horizontal scrolling
 *      means that 16 pixels will be used to represent a line.  Finally
 *      if kTXNScrollUnitsInViewRects the value is treated as the width
 *      of the current viewRect.
 *    
 *    ioVerticalDelta:
 *      The vertical amount to scroll.  The values in ioVerticalDelta
 *      can be treated as pixels, lines or viewrects.  See the
 *      discussion of the TXNScrollUnit parameters for more information
 *      for this.  On return this will contain the number of pixels
 *      actually scrolled in the vertical direction. A positive value
 *      moves the text down.
 *    
 *    ioHorizontalDelta:
 *      The horizontal amount to scroll. The values in
 *      ioHorizontalDelta can specify a scroll amount that is pixels,
 *      lines or view rects.  Set TXNScrollUnit discussion for more
 *      information. On return this will contain the number of pixels
 *      actually scrolled in the horizontal direction. A positive value
 *      moves the text to the right.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   in Textension not yet available
 }
function TXNScroll( iTXNObject: TXNObject; iVerticalScrollUnit: TXNScrollUnit; iHorizontalScrollUnit: TXNScrollUnit; var ioVerticalDelta: SIGNEDLONG; var ioHorizontalDelta: SIGNEDLONG ): OSStatus; external name '_TXNScroll';
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   in Textension not yet available
 }
function TXNScroll( iTXNObject: TXNObject; iVerticalScrollUnit: TXNScrollUnit; iHorizontalScrollUnit: TXNScrollUnit; var ioVerticalDelta: SInt32; var ioHorizontalDelta: SInt32 ): OSStatus; external name '_TXNScroll';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   in Textension not yet available
 }
function TXNScroll( iTXNObject: TXNObject; iVerticalScrollUnit: TXNScrollUnit; iHorizontalScrollUnit: TXNScrollUnit; var ioVerticalDelta: SIGNEDLONG; var ioHorizontalDelta: SIGNEDLONG ): OSStatus; external name '_TXNScroll';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  TXNRegisterScrollInfoProc()
 *  
 *  Discussion:
 *    If your application is drawing and handling its own scrolling
 *    widgets use this function to register a TXNScrollInfoUPP.  If you
 *    register a TXNScrollInfoUPP it will be called every time MLTE
 *    would normally update the values and maximum values of an MLTE
 *    scrollbar. For example when the user types the return key to add
 *    a new line at the end of their text MLTE will calculate a new
 *    maximum value.  If you have registered a TXNScrollInfoUPP it will
 *    be called with this nex maximum value. To turn off the callbacks
 *    call TXNRegisterScrollInfoProc with a value of NULL for the
 *    iTXNScrollInfoUPP.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    iTXNScrollInfoUPP:
 *      A universal procedure pointer.
 *    
 *    iRefCon:
 *      A refcon that is passed to the callback.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   in Textension not yet available
 }
procedure TXNRegisterScrollInfoProc( iTXNObject: TXNObject; iTXNScrollInfoUPP: TXNScrollInfoUPP; iRefCon: SRefCon ); external name '_TXNRegisterScrollInfoProc';
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   in Textension not yet available
 }
procedure TXNRegisterScrollInfoProc( iTXNObject: TXNObject; iTXNScrollInfoUPP: TXNScrollInfoUPP; iRefCon: SInt32 ); external name '_TXNRegisterScrollInfoProc';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   in Textension not yet available
 }
procedure TXNRegisterScrollInfoProc( iTXNObject: TXNObject; iTXNScrollInfoUPP: TXNScrollInfoUPP; iRefCon: SRefCon ); external name '_TXNRegisterScrollInfoProc';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  TXNSetScrollbarState()
 *  
 *  Summary:
 *    Sets the state of the scrollbars
 *  
 *  Discussion:
 *    This replaces TXNActivate, which was confusing to many
 *    developers, since it only activates/inactivates the scrollbar. 
 *    This is useful for activating scrollbar(s) even when the object
 *    does not have focus.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    iActiveState:
 *      Boolean: if true, scrollbars will be active even if the object
 *      does not have the keyboard focus.  If false, scrollbars are
 *      synched with active state
 *  
 *  Result:
 *    A result code indicating success or failure.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function TXNSetScrollbarState( iTXNObject: TXNObject; iActiveState: TXNScrollBarState ): OSStatus; external name '_TXNSetScrollbarState';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Offset/Point Conversion                                                                           }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

{
 *  TXNHIPointToOffset()
 *  
 *  Summary:
 *    Gets the coordinates of the point that corresponds to a specified
 *    offset in a text object.
 *  
 *  Discussion:
 *    The coordinates of the point are in the coordinate system of the
 *    window or view owning the text object.  Note that the owner of
 *    the a text object is a view only in the case when the object is
 *    in a HITextView.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject. The text object
 *      for which you want to obtain an offset value.
 *    
 *    iHIPoint:
 *      A pointer to an HIPoint.
 *    
 *    oOffset:
 *      On return, a pointer to the offset that corresponds to the
 *      value of the iHIPoint parameter.
 *  
 *  Result:
 *    A result code indicating success or failure.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function TXNHIPointToOffset( iTXNObject: TXNObject; const (*var*) iHIPoint: HIPoint; var oOffset: TXNOffset ): OSStatus; external name '_TXNHIPointToOffset';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  TXNOffsetToHIPoint()
 *  
 *  Summary:
 *    Gets the coordinates of the point that corresponds to a specified
 *    offset in a text object.
 *  
 *  Discussion:
 *    The coordinates of the point are in the coordinate system of the
 *    window or view owning the text object.  Note that the owner of
 *    the a text object is a view only in the case when the object is
 *    in a HITextView.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject. The text object
 *      for which you want to obtain the coordinates of a point.
 *    
 *    iOffset:
 *      A text offset value.
 *    
 *    oHIPoint:
 *      On return, a pointer to the point that corresponds to the value
 *      of the iOffset parameter.
 *  
 *  Result:
 *    A result code indicating success or failure.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function TXNOffsetToHIPoint( iTXNObject: TXNObject; iOffset: TXNOffset; var oHIPoint: HIPoint ): OSStatus; external name '_TXNOffsetToHIPoint';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Drag and Drop                                                                                     }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

{
 *  TXNDragTracker()
 *  
 *  Summary:
 *    If you ask that Drag handling procs not be installed.  Call this
 *    when your drag tracker is called and you want Textension to take
 *    over.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNNewObject.
 *    
 *    iTXNFrameID:
 *      Deprecated. Pass 0.
 *    
 *    iMessage:
 *      Drag message obtained from Drag Manager.
 *    
 *    iWindow:
 *      WindowRef obtained from Drag Manager.
 *    
 *    iDragReference:
 *      DragReference obtained from Drag Manager.
 *    
 *    iDifferentObjectSameWindow:
 *      Pass true if the drag is still in the same window that it
 *      started in. False if the drag has moved into a different window.
 *  
 *  Result:
 *    A result code indicating success or failure.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNDragTracker( iTXNObject: TXNObject; iTXNFrameID: TXNFrameID; iMessage: DragTrackingMessage; iWindow: WindowRef; iDragReference: DragReference; iDifferentObjectSameWindow: Boolean ): OSErr; external name '_TXNDragTracker';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  TXNDragReceiver()
 *  
 *  Summary:
 *    If you ask that Drag handling procs not be installed.  Call this
 *    when your drag receiver is called and you want Textension to take
 *    over.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNNewObject.
 *    
 *    iTXNFrameID:
 *      Deprecated. Pass 0.
 *    
 *    iWindow:
 *      WindowRef obtained from Drag Manager.
 *    
 *    iDragReference:
 *      DragReference obtained from Drag Manager.
 *    
 *    iDifferentObjectSameWindow:
 *      Pass true if the drag is still in the same window that it
 *      started in. False if the drag has moved into a different window.
 *  
 *  Result:
 *    A result code indicating success or failure.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNDragReceiver( iTXNObject: TXNObject; iTXNFrameID: TXNFrameID; iWindow: WindowRef; iDragReference: DragReference; iDifferentObjectSameWindow: Boolean ): OSErr; external name '_TXNDragReceiver';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Command Events and Spell Checking                                                                 }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not TARGET_CPU_64}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {not TARGET_CPU_64}

>>>>>>> origin/fixes_2.4

{
 *  TXNCommandEventSupportOptions
 *  
 *  Discussion:
 *    Use these constants to turn support for specific HICommands in a
 *    given MLTE object.
 }
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	TXNCommandEventSupportOptions = OptionBits;
=======
	TXNCommandEventSupportOptions = UInt32;
>>>>>>> graemeg/fixes_2_2
=======
	TXNCommandEventSupportOptions = UInt32;
>>>>>>> origin/fixes_2_2
=======
	TXNCommandEventSupportOptions = OptionBits;
>>>>>>> origin/fixes_2.4
const
{
   * Setting this bit enables processing for these HICommands:
   * kHICommandUndo, kHICommandRedo, kHICommandCut, kHICommandCopy,
   * kHICommandPaste, kHICommandClear, kHICommandSelectAll.
   }
	kTXNSupportEditCommandProcessing = 1 shl 0;

  {
   * Setting this bit turns support on for updating the the menu item
   * associated with the above edit commands. What that means is:
   * 
   * 
   * For Undo the undo item will be enabled if there are any undoable
   * actions in MLTE's command stack.  Additionally, if you have
   * installed an action key mapper proc that will be called to get the
   * appropriate string for the undo item. 
   * 
   * For Redo the Redo item is enabled if there are any redoable
   * actions and the action key mapper callback will be called if it is
   * installed. 
   * 
   * For Cut and Clear the item is enabled if a the current selection
   * is not empty.  If the selection is empty this is disabled.
   * 
   * 
   * For Paste the item is enabled if the clipboard is not empty.  It
   * is disabled if the clipboard is empty or contains data that MLTE
   * does not understand. 
   * <BR> For Select All the item is always enabled.
   }
	kTXNSupportEditCommandUpdating = 1 shl 1;

  {
   * Turns on support for spell checking.  The spell checking commands
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
   * supported are kHICommandShowSpellingPanel,
   * kHICommandCheckSpelling, kHICommandChangeSpelling,
   * kHICommandCheckSpellingAsYouType, kHICommandIgnoreSpelling, and
   * kHICommandLearnWord.
<<<<<<< HEAD
=======
   * supported are: Show Spelling Panel ('shsp'), Check Spelling
   * ('cksp'), Change Spelling ('chsp'), enable check spelling as you
   * type ('aspc'), ignore spelling ('igsp'), learn spelling ('lrwd').
>>>>>>> graemeg/fixes_2_2
=======
   * supported are: Show Spelling Panel ('shsp'), Check Spelling
   * ('cksp'), Change Spelling ('chsp'), enable check spelling as you
   * type ('aspc'), ignore spelling ('igsp'), learn spelling ('lrwd').
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
   }
	kTXNSupportSpellCheckCommandProcessing = 1 shl 2;

  {
   * Turns on support for updating the menu item associated with a
   * given spell checking command. 
   * 
   * Show Spelling: This item is always enabled. 
   * 
   * Check Spelling: This item is always enabled. 
   * 
   * Change Spelling:  Typically this is not included in a spelling
   * menu.  If it is included it is enabled if the current selection is
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   * a misspelled word.  It is disabled if the current selection is
=======
   * a missspelled word.  It is disabled if the current selection is
>>>>>>> graemeg/fixes_2_2
=======
   * a missspelled word.  It is disabled if the current selection is
>>>>>>> origin/fixes_2_2
=======
   * a misspelled word.  It is disabled if the current selection is
>>>>>>> origin/fixes_2.4
   * empty or is not a misspelled word. 
   * 
   * Check Spelling as You Type:  This item is always enabled.  It is
   * checked if this feature has been enabled.  By default when you
   * turn on spell checking this is enabled. If this feature has been
   * disabled the item is unchecked. 
   * 
   * Ignore Spelling.  This is a command that usually does not have a
   * corresponding menu item.  If a menu does have this item it is
   * disabled if the current selection is empty or is not a misspelled
   * word.  It is enabled if the current selection is a misspelled
   * word. 
   * 
   * Learn Spelling.  Another command that typically does not have a
   * corresponding menu item.  It behaves exactly like the Ignore
   * spelling command.
   }
	kTXNSupportSpellCheckCommandUpdating = 1 shl 3;

  {
   * Setting this bit to turn on Carbon Font Panel support. Once
   * support is turned on, MLTE handles the following Carbon Events
   * defined in FontPanel.h: • kHICommandShowHideFontPanel and
   * kEventFontPanelClosed to show and hide the Carbon font panel •
   * kEventFontSelection event to update the document after you select
   * a new font, size, style, color, or any features setting from the
   * Typography Panel.
   }
	kTXNSupportFontCommandProcessing = 1 shl 4;

  {
   * Setting this bit to turn on support for updating the selection in
   * Carbon Font Panel when the current selection in MLTE document is
   * changed. When you set this bit, kTXNSupportFontCommandProcessing
   * has to be set also, which means that you only can get this support
   * if Carbon Font Panel support is on.
   }
	kTXNSupportFontCommandUpdating = 1 shl 5;

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
 *  TXNSetCommandEventSupport()
 *  
 *  Summary:
 *    Turn support for a variety of HICommands on or off in MLTE. See
 *    the documentation for the TXNCommandEventSupportOptions constants
 *    for the set of commands that are supported.
 *  
 *  Discussion:
 *    This function enables support for a variety of commands.  It is
 *    important to know that the handlers are installed immediatedly
 *    when an MLTE object is associated with an HIObject that can serve
 *    as an event target ( via a call to TXNAttachObjectToWindow or
 *    TXNSetEventTarget ). However when the handlers are installed they
 *    are not enabled.  Calling TXNSetCommandEventSupport enables the
 *    commands. Using this approach means that an application can
 *    install handlers on top of these and be sure that enabling or
 *    disabling the MLTE handlers will not change the order of the
 *    handler chain.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      The TXNObject.
 *    
 *    iOptions:
 *      Set this to enable/disable command support for selected Edit
 *      commands (kHICommandUndo, kHICommandRedo, kHICommandCut,
 *      kHICommandCopy, kHICommandPaste, kHICommandClear,
 *      kHICommandSelectAll) and/or support for spell checking commands.
 *  
 *  Result:
 *    An OSStatus code.  noErr if the operation was successful.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function TXNSetCommandEventSupport( iTXNObject: TXNObject; iOptions: TXNCommandEventSupportOptions ): OSStatus; external name '_TXNSetCommandEventSupport';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  TXNGetCommandEventSupport()
 *  
 *  Summary:
 *    Returns the command event support that is currently set for an
 *    MLTE object.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      The TXNObject.
 *    
 *    oOptions:
 *      A pointer to a TXNCommandEventSupportOptions. Contains the
 *      option settings or return.
 *  
 *  Result:
 *    An OSStatus code.  noErr if the operation was successful.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function TXNGetCommandEventSupport( iTXNObject: TXNObject; var oOptions: TXNCommandEventSupportOptions ): OSStatus; external name '_TXNGetCommandEventSupport';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  TXNSetSpellCheckAsYouType()
 *  
 *  Summary:
 *    Call to turn on spell check as you type.  If spell checking as
 *    not been enabled via a call to TXNSetCommandEventSupport than an
 *    error is returned.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      The TXNObject.
 *    
 *    iActivate:
 *      Turns "Spell Check as You Type" on if true.  Turns it off if
 *      false.
 *  
 *  Result:
 *    An OSStatus code.  noErr if the operation was successful.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function TXNSetSpellCheckAsYouType( iTXNObject: TXNObject; iActivate: Boolean ): OSStatus; external name '_TXNSetSpellCheckAsYouType';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  TXNGetSpellCheckAsYouType()
 *  
 *  Summary:
 *    Call this function to determine if "Spell Check as You Type" is
 *    turned on or off.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      The TXNObject.
 *  
 *  Result:
 *    A Boolean.  The value is true if Spell Check As You Type is on
 *    and false if not.  Additionally if Spell Checking is not enabled
 *    for the TXNObject then the result of this function will be false.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function TXNGetSpellCheckAsYouType( iTXNObject: TXNObject ): Boolean; external name '_TXNGetSpellCheckAsYouType';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
    Functions for controlling the current event target object.
}
{
 *  TXNSetEventTarget()
 *  
 *  Summary:
 *    Specifies a Carbon Event target for MLTE Carbon Event Handlers.
 *  
 *  Discussion:
 *    Call this function when you want to override or set the
 *    TXNObject's Carbon Event Target.  The default target for
 *    TXNObjects that are not contained in an HITextView is the
 *    TXNObject's window.  Note that if the TXNObject already has a
 *    default target when this function is called the handlers will be
 *    removed from the old target before install the new handlers that
 *    are listed below. 
 *    
 *    When this function returns handlers for the following Carbon
 *    Events are installed and are active: 
 *    
 *    For kEventClassTextInput: 
 *     kEventTextInputUpdateActiveInputArea 
 *     kEventTextInputUnicodeForKeyEvent 
 *     kEventTextInputUnicodeText 
 *    kEventTextInputOffsetToPos 
 *    kEventTextInputPosToOffset 
 *    kEventTextInputGetSelectedText 
 *    
 *    For kEventClassTSMDocumentAccess: 
 *     kEventTSMDocumentAccessGetLength 
 *     kEventTSMDocumentAccessGetSelectedRange 
 *     kEventTSMDocumentAccessGetCharactersPtr 
 *     kEventTSMDocumentAccessGetCharactersPtrForLargestBuffer 
 *     kEventTSMDocumentAccessGetCharacters 
 *     kEventTSMDocumentAccessGetFont 
 *     kEventTSMDocumentAccessGetGlyphInfo 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *     kEventTSMDocumentAccessGetFirstRectForRange 
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *     kEventTSMDocumentAccessGetFirstRectForRange 
>>>>>>> origin/fixes_2.4
 *    
 *    For kEventClassFont: 
 *    kEventFontPanelClosed 
 *    kEventFontSelection 
 *    
 *    Additionally these handlers for kEventClassCommand are installed,
 *    but are inactive by default. 
 *    kEventProcessCommand 
 *    kEventCommandUpdateStatus 
 *    
 *    The kEventClassCommand handlers support the following commands:
 *    
 *    kHICommandUndo 
 *    kHICommandRedo 
 *    kHICommandSelectAll 
 *    kHICommandCut 
 *    kHICommandCopy 
 *    kHICommandPaste 
 *    kHICommandClear 
 *    kHICommandShowSpellingPanel 
 *    kHICommandCheckSpelling 
 *    kHICommandChangeSpelling 
 *    kHICommandCheckSpellingAsYouType 
 *    kHICommandIgnoreSpelling 
 *    kHICommandLearnWord 
 *    
 *    Activate command support by calling TXNSetCommandEventSupport
 *    with the correct options.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      The TXNObject.
 *    
 *    iEventTarget:
 *      An HIObjectRef.  The HIObject will become the event target for
 *      all of the TXNObject's Carbon Event handlers.
 *  
 *  Result:
 *    An OSStatus code.  noErr if the operation was successful.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function TXNSetEventTarget( iTXNObject: TXNObject; iEventTarget: HIObjectRef { can be NULL } ): OSStatus; external name '_TXNSetEventTarget';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  TXNGetEventTarget()
 *  
 *  Summary:
 *    Requests a TXNObject's current event target.
 *  
 *  Discussion:
 *    This function is used to obtain the HIObjectRef which is the
 *    TXNObject's carbon event target. Callers can use this function to
 *    obtain the target and subsequently install their own handlers.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      The TXNObject.
 *    
 *    oEventTarget:
 *      An HIObjectRef pointer.  The current Carbon Event target.
 *  
 *  Result:
 *    An OSStatus code.  noErr if the operation was successful.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function TXNGetEventTarget( iTXNObject: TXNObject; var oEventTarget: HIObjectRef ): OSStatus; external name '_TXNGetEventTarget';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  TXNSetContextualMenuSetup()
 *  
 *  Summary:
 *    Provides a callback function that is called before MLTE displays
 *    its contextual menu.
 *  
 *  Discussion:
 *    The menuSetupProc is called immediately before MLTE displays its
 *    contextual menu.  The menu that is passed to the callback will
 *    only contain MLTE specific items.  The client items and handlers
 *    should be installed each time the callback is called.  When the
 *    callback is called MLTE will have selected the word that the user
 *    option-clicked on.  For convenience sake the TXNObject associated
 *    with the callback is passed to the callback as well as the user
 *    data.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      The TXNObject.
 *    
 *    iMenuSetupProc:
 *      The callback.
 *    
 *    iUserData:
 *      A pointer to some useful data.
 *  
 *  Result:
 *    OStatus.  noErr is the operation is successful.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function TXNSetContextualMenuSetup( iTXNObject: TXNObject; iMenuSetupProc: TXNContextualMenuSetupUPP; iUserData: {const} UnivPtr ): OSStatus; external name '_TXNSetContextualMenuSetup';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Accessibility                                                                                     }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

{
 *  TXNGetAccessibilityHIObject()
 *  
 *  Summary:
 *    Return an HIObjectRef used to represent the MLTE object for
 *    accessibility.
 *  
 *  Discussion:
 *    For each MLTE object a view creates, the view needs to call
 *    TXNGetAccessibilityHIObject to get an HIObjectRef that can be
 *    used to represent the MLTE object as an accessible
 *    object.
 *    
 *    After the view gets this HIObjectRef, it must add the HIObjectRef
 *    as a child of itself.  The accessibility engine will then route
 *    events to MLTE accessible objects automatically.
 *    
 *    The view must install Carbon event handlers for
 *    kEventAccessibleGetAllAttributeNames and
 *    kEventAccessibleGetNamedAttribute, using the HIObjectRef as the
 *    target, to provide information for at least the following
 *    required attributes:
 *    
 *    kAXRoleAttribute
 *    kAXRoleDescriptionAttribute
 *    kAXWindowAttribute
 *    kAXTopUIElementAttribute
 *    kAXPositionAttribute
 *    kAXSizeAttribute
 *    
 *    MLTE also installs handlers for
 *    kEventAccessibleGetAllAttributeNames and
 *    kEventAccessibleGetNamedAttribute.  These handlers return
 *    information for the following attributes.  Note that these
 *    handlers will not get called unless the client-installed handlers
 *    return eventNotHandledErr.
 *    
 *    kAXEnabledAttribute
 *    kAXFocusedAttribute
 *    kAXValueAttribute
 *    kAXSelectedTextAttribute
 *    kAXSelectedTextRangeAttribute
 *     kAXNumberOfCharactersAttribute
 *    
 *    kAXLineForIndexParameterizedAttribute
 *     kAXRangeForLineParameterizedAttribute
 *     kAXStringForRangeParameterizedAttribute
 *     kAXRangeForPositionParameterizedAttribute
 *     kAXRangeForIndexParameterizedAttribute
 *     kAXBoundsForRangeParameterizedAttribute
 *     kAXStyleRangeForIndexParameterizedAttribute
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    oHIObjectRef:
 *      An HIObjectRef which represents iTXNObject as an accessible
 *      object.
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.4 and later in Carbon.framework [32-bit only]
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function TXNGetAccessibilityHIObject( iTXNObject: TXNObject; var oHIObjectRef: HIObjectRef ): OSStatus; external name '_TXNGetAccessibilityHIObject';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{ **************************************************************************************************** }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{                                  •  DEPRECATED •                                                     }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{ **************************************************************************************************** }

{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • DEPRECATED CONSTANTS                                                                              }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{$endc} {not TARGET_CPU_64}

{$ifc not TARGET_CPU_64}
{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{ Deprecated Font Menu support on Mac OS X version 10.5 and later, use Font Panel instead              }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	TXNFontMenuObject = ^OpaqueTXNFontMenuObject; { an opaque type }
	OpaqueTXNFontMenuObject = record end;
=======
	TXNFontMenuObject = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	TXNFontMenuObject = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	TXNFontMenuObject = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	TXNFontMenuObject = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
	TXNFontMenuObjectPtr = ^TXNFontMenuObject;  { when a var xx:TXNFontMenuObject parameter can be nil, it is changed to xx: TXNFontMenuObjectPtr }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kTXNFontMenuRefKey CFSTRP('FontMenuRef')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kTXNFontMenuObjectKey CFSTRP('FontMenuObject')}
{$endc}

{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{ Deprecated Font / keyboard synchronization support constants on Mac OS X version 10.5 and later      }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}
const
	kTXNKeyboardSyncStateTag = FourCharCode('kbsy');

{ kTXNKeyboardSyncStateTag}
const
	kTXNSyncKeyboard = false;
	kTXNNoSyncKeyboard = true;

const
	kTXNNoKeyboardSyncBit = 6;

const
	kTXNNoKeyboardSyncMask = 1 shl kTXNNoKeyboardSyncBit;

{$endc} {not TARGET_CPU_64}


{
=======
=======
>>>>>>> origin/fixes_2_2
{                                  • HITEXTVIEW APIs •                                                 }
=======
{                                  •  DEPRECATED •                                                     }
>>>>>>> origin/fixes_2.4
{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{ **************************************************************************************************** }

{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • DEPRECATED CONSTANTS                                                                              }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{$endc} {not TARGET_CPU_64}

{$ifc not TARGET_CPU_64}
{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{ Deprecated Font Menu support on Mac OS X version 10.5 and later, use Font Panel instead              }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}
type
	TXNFontMenuObject = ^SInt32; { an opaque type }
	TXNFontMenuObjectPtr = ^TXNFontMenuObject;  { when a var xx:TXNFontMenuObject parameter can be nil, it is changed to xx: TXNFontMenuObjectPtr }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kTXNFontMenuRefKey CFSTRP('FontMenuRef')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kTXNFontMenuObjectKey CFSTRP('FontMenuObject')}
{$endc}

{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{ Deprecated Font / keyboard synchronization support constants on Mac OS X version 10.5 and later      }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}
const
	kTXNKeyboardSyncStateTag = FourCharCode('kbsy');

{ kTXNKeyboardSyncStateTag}
const
	kTXNSyncKeyboard = false;
	kTXNNoSyncKeyboard = true;

const
	kTXNNoKeyboardSyncBit = 6;

const
	kTXNNoKeyboardSyncMask = 1 shl kTXNNoKeyboardSyncBit;

{$endc} {not TARGET_CPU_64}


{
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 *  TXTNTag
 *  
 *  Summary:
 *    Deprecated. Use TXNTypeRunAttributes.
 }

type
	TXTNTag = FourCharCode;

{
 *  TXNErrors
 *  
 *  Summary:
 *    Deprecated. Use OSStatus.
 }
type
	TXNErrors = OSStatus;

{
 *  TXNObjectRefcon
 *  
 *  Summary:
 *    Deprecated. Only used by TXNNewObject API, which has been
 *    deprecated.
 }
type
	TXNObjectRefcon = UnivPtr;

{
 *  TXNLongRect
 *  
 *  Summary:
 *    Deprecated. Only used by TXNGetRectBounds/TXNSetRectBounds APIs,
 *    which have been deprecated.
 }
type
	TXNLongRectPtr = ^TXNLongRect;
	TXNLongRect = record
		top: SInt32;
		left: SInt32;
		bottom: SInt32;
		right: SInt32;
	end;

{
 *  TXNFrameType
 *  
 *  Summary:
 *    Deprecated.  Only used by TXNNewObject API, which has been
 *    deprecated. Only kTXNTextEditStyleFrameType is supported.
 }
type
	TXNFrameType = UInt32;
const
	kTXNTextEditStyleFrameType = 1;
	kTXNPageFrameType = 2;    { Not supported}
	kTXNMultipleFrameType = 3;     { Not supported}


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{  Deprecated all QD TXNTypeRunAttributes & TXNTypeRunAttributeSizes constants.}
const
	kTXNQDFontNameAttribute = FourCharCode('fntn');
	kTXNQDFontFamilyIDAttribute = FourCharCode('font');
	kTXNQDFontSizeAttribute = FourCharCode('size'); {the corresponding TXNTypeRunAttributeSizes is kTXNFontSizeAttributeSize}
	kTXNQDFontStyleAttribute = FourCharCode('face');
	kTXNQDFontColorAttribute = FourCharCode('klor');

const
	kTXNQDFontNameAttributeSize = SizeOf(Str255);
	kTXNQDFontFamilyIDAttributeSize = SizeOf(SInt16);
	kTXNFontSizeAttributeSize = SizeOf(Fixed);
	kTXNQDFontSizeAttributeSize = SizeOf(SInt16);
	kTXNQDFontStyleAttributeSize = SizeOf(Style);
	kTXNQDFontColorAttributeSize = SizeOf(RGBColor);
=======
=======
>>>>>>> origin/fixes_2_2
{
    Deprecated TXNTypeRunAttributes and TXNTypeRunAttributeSizes constants.
    kTXNQDFontSizeAttribute is obsolete.  Incorrect font sizes are always returned as a Fixed
    value.  Use kTXNFontSizeAttribute instead.
}

=======
{  Deprecated all QD TXNTypeRunAttributes & TXNTypeRunAttributeSizes constants.}
>>>>>>> origin/fixes_2.4
const
	kTXNQDFontNameAttribute = FourCharCode('fntn');
	kTXNQDFontFamilyIDAttribute = FourCharCode('font');
	kTXNQDFontSizeAttribute = FourCharCode('size'); {the corresponding TXNTypeRunAttributeSizes is kTXNFontSizeAttributeSize}
	kTXNQDFontStyleAttribute = FourCharCode('face');
	kTXNQDFontColorAttribute = FourCharCode('klor');

const
	kTXNQDFontNameAttributeSize = SizeOf(Str255);
	kTXNQDFontFamilyIDAttributeSize = SizeOf(SInt16);
	kTXNFontSizeAttributeSize = SizeOf(Fixed);
	kTXNQDFontSizeAttributeSize = SizeOf(SInt16);
<<<<<<< HEAD

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kTXNQDFontStyleAttributeSize = SizeOf(Style);
	kTXNQDFontColorAttributeSize = SizeOf(RGBColor);
>>>>>>> origin/fixes_2.4

{
    Deprecated QuickDraw imaging constants.
    On Mac OS X version 10.4 and later, MLTE always uses Quartz imaging.    
}

const
	kTXNUseQDforImagingBit = 16;

const
	kTXNUseQDforImagingMask = 1 shl kTXNUseQDforImagingBit;

const
	kTXNImageWithQDBit = 8;

const
	kTXNImageWithQDMask = 1 shl kTXNImageWithQDBit;


{
    Deprecated TXNControlTag constant.
    On Mac OS X version 10.4 and later, MLTE never draws the caret when inactive.    
}

const
	kTXNDrawCaretWhenInactiveTag = FourCharCode('dcrt'); { Caret will never be drawn when inactive.}

const
	kTXNDontDrawCaretWhenInactive = false;
	kTXNDrawCaretWhenInactive = true;

const
	kTXNDontDrawCaretWhenInactiveBit = 12;

const
	kTXNDontDrawCaretWhenInactiveMask = 1 shl kTXNDontDrawCaretWhenInactiveBit;


{
    Deprecated TSM constants.
    On Mac OS X, MLTE uses the Text Services Manager.
}

const
	kTXNNoTSMEverBit = 4;

const
	kTXNNoTSMEverMask = 1 shl kTXNNoTSMEverBit;


{
    Deprecated TXNCarbonEventInfo flag constants.
    MLTE does not support AppleEvents on Mac OS X version 10.1 and later.
}

const
	kTXNNoAppleEventHandlersBit = 0;
	kTXNRestartAppleEventHandlersBit = 1;

const
	kTXNNoAppleEventHandlersMask = 1 shl kTXNNoAppleEventHandlersBit;
	kTXNRestartAppleEventHandlersMask = 1 shl kTXNRestartAppleEventHandlersBit;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
{ Use kTXNFontMenuObjectKey}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kTXNFontMenuRefKey CFSTRP('FontMenuRef')}
{$endc}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{
 *  TXNActionKey
 *  
 }
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	TXNActionKeyPtr = ^TXNActionKey; { when a VAR xx: TXNActionKey parameter can be nil, it is changed to xx: TXNActionKeyPtr }
=======
	TXNActionKeyPtr				= ^TXNActionKey; { when a VAR xx: TXNActionKey parameter can be nil, it is changed to xx: TXNActionKeyPtr }
>>>>>>> graemeg/fixes_2_2
=======
	TXNActionKeyPtr				= ^TXNActionKey; { when a VAR xx: TXNActionKey parameter can be nil, it is changed to xx: TXNActionKeyPtr }
>>>>>>> origin/fixes_2_2
=======
	TXNActionKeyPtr = ^TXNActionKey; { when a VAR xx: TXNActionKey parameter can be nil, it is changed to xx: TXNActionKeyPtr }
>>>>>>> origin/fixes_2.4
	TXNActionKey = UInt32;
const
	kTXNTypingAction = 0;
	kTXNCutAction = 1;
	kTXNPasteAction = 2;
	kTXNClearAction = 3;
	kTXNChangeFontAction = 4;
	kTXNChangeFontColorAction = 5;
	kTXNChangeFontSizeAction = 6;
	kTXNChangeStyleAction = 7;
	kTXNAlignLeftAction = 8;
	kTXNAlignCenterAction = 9;
	kTXNAlignRightAction = 10;
	kTXNDropAction = 11;
	kTXNMoveAction = 12;
	kTXNFontFeatureAction = 13;
	kTXNFontVariationAction = 14;

  {
   * Use if none of the above apply.
   }
	kTXNUndoLastAction = 1024;

{ Deprecated on Mac OS X version 10.4 and later, use kTXNActionNameMapperKey instead.}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kTXNActionKeyMapperKey CFSTRP('ActionKeyMapper')}
{$endc}

{ Deprecated on Mac OS X version 10.4 and later, use TXNActionNameMapperProcPtr instead.}
type
	TXNActionKeyMapperProcPtr = function( actionKey: TXNActionKey; commandID: UInt32 ): CFStringRef;
type
	TXNActionKeyMapperUPP = TXNActionKeyMapperProcPtr;
{
 *  NewTXNActionKeyMapperUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewTXNActionKeyMapperUPP( userRoutine: TXNActionKeyMapperProcPtr ): TXNActionKeyMapperUPP; external name '_NewTXNActionKeyMapperUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposeTXNActionKeyMapperUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeTXNActionKeyMapperUPP( userUPP: TXNActionKeyMapperUPP ); external name '_DisposeTXNActionKeyMapperUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokeTXNActionKeyMapperUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeTXNActionKeyMapperUPP( actionKey: TXNActionKey; commandID: UInt32; userUPP: TXNActionKeyMapperUPP ): CFStringRef; external name '_InvokeTXNActionKeyMapperUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Count Option Bits                                                                                 }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  Options for TXNGet/ClearActionChangeCount to decide what type(s) of action count to use.            }

const
	kTXNTextInputCountBit = 0;
	kTXNRunCountBit = 1;


{
 *  TXNCountOptions
 *  
 }
type
	TXNCountOptions = OptionBits;
const
	kTXNTextInputCountMask = 1 shl kTXNTextInputCountBit;
	kTXNRunCountMask = 1 shl kTXNRunCountBit;
	kTXNAllCountMask = kTXNTextInputCountMask or kTXNRunCountMask;


{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • DEPRECATED APIs,  Mac OS X version 10.2 and later                                                 }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

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
 *  TXNSetViewRect()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use TXNSetFrameBounds or TXNSetRectBounds APIs instead.
 *  
 *  Discussion:
 *    Set the rectangle describing the current view into the document.
 *    This will change how much text is viewable.  Not where a line of
 *    text wraps. That is controlled by TXNSetFrameBoundsSize.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    iViewRect:
 *      Rect of the view.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.2
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.2
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.2
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in Textension 1.3 and later
 }
procedure TXNSetViewRect( iTXNObject: TXNObject; const (*var*) iViewRect: Rect ); external name '_TXNSetViewRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_2 *)


{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • DEPRECATED APIs, Mac OS X version 10.3 and later                                                  }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{
 *  TXNNewObject()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use TXNCreateObject API instead.
 *  
 *  Discussion:
 *    Allocates a new frame (i.e. new is called to allocate a
 *    TXNObject) and returns a pointer to the object in the newDoc
 *    parameter.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iFileSpec:
 *      If NULL you start with an empty document.  If not NULL, the
 *      file is read to obtain the document contents  after the object
 *      is successfully allocated.
 *    
 *    iWindow:
 *      Required.  The window in which the document is going to be 
 *      displayed.  If a fileSpec is provided during creation, the
 *      filename is going to be used as the window title.  If iWindow
 *      is NULL, TXNAttachObjectToWindow needs to be called after
 *      creation.
 *    
 *    iFrame:
 *      If text-area does not fill the entire window.  This specifies
 *      the area to fill.  Can be NULL. In  which case, the window’s
 *      portRect is used as the frame.
 *    
 *    iFrameOptions:
 *      Specify the options to be supported by this frame.  The
 *      available options are support for cutting and pasting  movies
 *      and sound, handle scrollbars and handle grow box in  the frame.
 *    
 *    iFrameType:
 *      Specify the type of frame to be used.  In MLTE version 1.1 and
 *      earlier, only kTXNTextEditStyleFrameType is supported.
 *    
 *    iFileType:
 *      Specify the primary file type.  If you  use 
 *      kTextensionTextFile files will be saved in a private format
 *      (see xxx).  If you  want saved files to be plain text files you
 *      should specify 'TEXT' here. If you specify 'TEXT' here you can
 *      use the frameOptions parameter to specify  whether the TEXT
 *      files should be saved  with 'MPSR' resources or 'styl'
 *      resources. These are resources which contain style information
 *      for a  file, and they  both have there own limitations.  If you
 *      use 'styl' resources to save style info your documents can have
 *      as many styles as you like however tabs will not be saved.  If
 *      you use 'MPSR' resources only the first style in the document 
 *      will be saved (you as client are expected to apply all style
 *      changes to the entire document).  If you  truly want  rich
 *      documents which can potentially contain graphics and sound you
 *      should specify kTextensionTextFileOutput.  If you want a plain
 *      text editor like SimpleText specify that style information by
 *      saved via ‘styl’ resources. If you want files similar to those
 *      output by CW IDE, BBEdit, and MPW specify that style
 *      information be saved in a ‘MPSR’ resource.
 *    
 *    iPermanentEncoding:
 *      The encoding in which the document should be saved (Unicode,
 *      Text or System default).
 *    
 *    oTXNObject:
 *      Pointer to the opaque datastructure allocated by the function. 
 *      Most of the subsequent functions require that such a pointer be
 *      passed in.
 *    
 *    oTXNFrameID:
 *      Unique ID for the frame.  This value is always set to 0.
 *    
 *    iRefCon:
 *      Caller can set this to any value.  It is retained by the
 *      TXNNewObject which can later be asked to return it.
 *  
 *  Result:
 *    If anything goes wrong the error is returned.  Success must be
 *    complete. That is if everything  works, but there is a failure
 *    reading a specified file the  object is freed.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.3
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.3
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.3
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.3
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNNewObject( {const} iFileSpec: FSSpecPtr { can be NULL }; iWindow: WindowRef; {const} iFrame: RectPtr { can be NULL }; iFrameOptions: TXNFrameOptions; iFrameType: TXNFrameType; iFileType: TXNFileType; iPermanentEncoding: TXNPermanentTextEncodingType; var oTXNObject: TXNObject; var oTXNFrameID: TXNFrameID; iRefCon: TXNObjectRefcon ): OSStatus; external name '_TXNNewObject';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


{
 *  TXNTerminateTextension()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    No longer needed.
 *  
 *  Discussion:
 *    Close the Textension library.  It is necessary to call this
 *    function so that Textension can correctly close down any TSM
 *    connections and do other clean up.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.3
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.3
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.3
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.3
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
procedure TXNTerminateTextension; external name '_TXNTerminateTextension';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


{
 *  TXNSetDataFromFile()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use TXNSetDataFromCFURLRef API instead.
 *  
 *  Discussion:
 *    Replace the specified range with the contents of the specified
 *    file.  The data fork of the file must be opened by the caller. 
 *    Functional in NoUserIO mode.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    iFileRefNum:
 *      File RefNum.
 *    
 *    iFileType:
 *      File type.
 *    
 *    iFileLength:
 *      The length of data in the file that should be considered data. 
 *      This parameter is available to enable callers to embed text
 *      inside their own private data structures.  Note that if the
 *      data is in the Textension(txtn) format this parameter is
 *      ignored since length, etc. information is part of the format.
 *      Further note that if you you just want Textension to read a
 *      file and you are not interested in embedding you can just pass
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      kTXNEndOffset, and Textension will use the file manager to
 *      determine the files length.
=======
 *      kTXNEndOffset(0x7FFFFFFF), and Textension will use the file
 *      manager to determine the files length.
>>>>>>> graemeg/fixes_2_2
=======
 *      kTXNEndOffset(0x7FFFFFFF), and Textension will use the file
 *      manager to determine the files length.
>>>>>>> origin/fixes_2_2
=======
 *      kTXNEndOffset, and Textension will use the file manager to
 *      determine the files length.
>>>>>>> origin/fixes_2.4
 *    
 *    iStartOffset:
 *      Start position at which to insert the file into the document.
 *    
 *    iEndOffset:
 *      End position of range being replaced by the file.
 *  
 *  Result:
 *    File manager error or noErr.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.3
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.3
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.3
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.3
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNSetDataFromFile( iTXNObject: TXNObject; iFileRefNum: SInt16; iFileType: OSType; iFileLength: ByteCount; iStartOffset: TXNOffset; iEndOffset: TXNOffset ): OSStatus; external name '_TXNSetDataFromFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


{
 *  TXNConvertToPublicScrap()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    This is a no-op in OS X.
 *  
 *  Discussion:
 *    Convert the Textension private scrap to the public clipboard. 
 *    This should be called on suspend events and before the
 *    application displays a dialog that might support cut and paste. 
 *    Or more generally, whenever someone other than the Textension
 *    Shared Library needs access to the scrap data. This is a no-op in
 *    OS X.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Result:
 *    Memory Manager errors, Scrap Manager errors, or noErr.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.3
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.3
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.3
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.3
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNConvertToPublicScrap: OSStatus; external name '_TXNConvertToPublicScrap';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


{
 *  TXNConvertFromPublicScrap()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    No longer needed.
 *  
 *  Discussion:
 *    Convert the  public clipboard to our private scrap .  This should
 *    be called on resume events and after an application has modified
 *    the scrap. Before doing work we check the validity of the public
 *    scrap (date modification and type). 
 *    
 *    This is no longer needed in Mac OS X version 10.2 and later. 
 *    Calling TXNPaste will automatically handle conversion from public
 *    scrap.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Result:
 *    Memory Manager errors, Scrap Manager errors, or noErr.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.3
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.3
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.3
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.3
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNConvertFromPublicScrap: OSStatus; external name '_TXNConvertFromPublicScrap';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


{
 *  TXNDraw()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Deprecated.
 *  
 *  Discussion:
 *    Redraw the TXNObject including any scrollbars associated with the
 *    text frame.  Call this function in response to an update event
 *    for a window that contains multiple TXNObjects or some other
 *    graphic element.  The caller is responsible for calling
 *    BeginUpdate/EndUpdate in response to the update event.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    iDrawPort:
 *      Can be NULL. If NULL the port is drawn to the port currently
 *      attached to the iTXNObject.  If not NULL drawing goes to the
 *      iDrawPort.  If drawing is done to the iDrawPort selection is
 *      not updated.  This works this way so that it is possible to
 *      Draw a TXNObject to a static port (i.e. print the thing without
 *      reflowing the text to match the paper size which is what
 *      TXNPrint does) and not have a line drawn where the selection
 *      would be.  If you pass an iDrawPort to an active TXNObject
 *      (i.e. editable) the selection will not be updated. In this case
 *      the selection will behave oddly until text is typed which will
 *      serve to realign the selection.  Bottom-line don't pass a port
 *      in unless you want static text (printed or non-editable)
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.3
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.3
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.3
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.3
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
procedure TXNDraw( iTXNObject: TXNObject; iDrawPort: GWorldPtr { can be NULL } ); external name '_TXNDraw';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


{
 *  TXNAttachObjectToWindow()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use TXNAttachObjectToWindowRef API instead.
 *  
 *  Discussion:
 *    If a TXNObject was initialized with a NULL window pointer use
 *    this function to attach a window to that object.  In version 1.0
 *    of Textension attaching a TXNObject to more than one window is
 *    not supported.  Note that if a CGContextRef was passed to the
 *    TXNObject previously through TXNSetTXNObjectControls, that
 *    CGContextRef will be ignored.  The CGContextRef associated with
 *    the iWindow will be used instead.  You may revert back to the
 *    previous CGContextRef by calling the TXNSetTXNObjectControls API
 *    with the desired CGContextRef again after calling
 *    TXNAttachObjectToWindow.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    iWindow:
 *      GWorldPtr that the object should be attached to.
 *    
 *    iIsActualWindow:
 *      Let the library know if the GWorldPtr is actually a WindowRef
 *      or actually a GWorldPtr.  This is important if the client is
 *      taking advantage of the editing packages scrollbar support.
 *  
 *  Result:
 *    paramErrs or noErr.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.3
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.3
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.3
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.3
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNAttachObjectToWindow( iTXNObject: TXNObject; iWindow: GWorldPtr; iIsActualWindow: Boolean ): OSStatus; external name '_TXNAttachObjectToWindow';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


{
 *  TXNIsObjectAttachedToWindow()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Deprecated.
 *  
 *  Discussion:
 *    A utility function that allows a caller to check a TXNObject to
 *    see if it is attached to a window.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *  
 *  Result:
 *    True if object is attached, false if TXNObject is not attached.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.3
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.3
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.3
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.3
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNIsObjectAttachedToWindow( iTXNObject: TXNObject ): Boolean; external name '_TXNIsObjectAttachedToWindow';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


{
 *  TXNIsObjectAttachedToSpecificWindow()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Deprecated.
 *  
 *  Discussion:
 *    Determines whether the given object is attached to the given
 *    window.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNNewObject.
 *    
 *    iWindow:
 *      The window to check attachment against.
 *    
 *    oAttached:
 *      true if the object is attached to the given window, false
 *      otherwise.
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.3
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.3
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.3
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.3
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in Textension 1.2 and later
 }
function TXNIsObjectAttachedToSpecificWindow( iTXNObject: TXNObject; iWindow: WindowRef; var oAttached: Boolean ): OSStatus; external name '_TXNIsObjectAttachedToSpecificWindow';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


{
 *  TXNSetRectBounds()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use TXNSetHIRectBounds API instead.
 *  
 *  Discussion:
 *    The View rectangle controls the text you see.  The Destination
 *    rectangle controls how text is laid out.  The Scrollbar is drawn
 *    inside the View rectangle. You only need to pass in pointers for
 *    the rectangles you want to set.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNNewObject.
 *    
 *    iViewRect:
 *      The new view rectangle.  If you do not want to change the view
 *      rectangle pass NULL.
 *    
 *    iDestinationRect:
 *      The new destination rectangle.  Pass NULL if you don't want to
 *      change the destination retangle.
 *    
 *    iUpdate:
 *      If you would like the the text and where the scrollbars are
 *      placed recalculated and redrawn pass true.  If you prefer to
 *      wait on this pass false.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in Carbon.framework [32-bit only] but deprecated in 10.3
=======
 *    Mac OS X:         in version 10.1 and later in Carbon.framework but deprecated in 10.3
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.1 and later in Carbon.framework but deprecated in 10.3
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.1 and later in Carbon.framework [32-bit only] but deprecated in 10.3
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
procedure TXNSetRectBounds( iTXNObject: TXNObject; {const} iViewRect: RectPtr { can be NULL }; {const} iDestinationRect: TXNLongRectPtr { can be NULL }; iUpdate: Boolean ); external name '_TXNSetRectBounds';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


{
 *  TXNGetRectBounds()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use TXNGetHIRect API instead.
 *  
 *  Discussion:
 *    Get the values for the current View rectangle, Destination
 *    rectangle and Text rectangle.  You only need to pass in pointers
 *    for the rectangles you're interested in.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNNewObject.
 *    
 *    oViewRect:
 *      The current view rectangle
 *    
 *    oDestinationRect:
 *      The current destination rectangle
 *    
 *    oTextRect:
 *      The smallest rectangle needed to contain the current text. 
 *      This rectangle is calculated by walking the lines of text and
 *      measuring each line.  So this can be expensive.  The width of
 *      this rectangle will be the width of the longest line in the
 *      text.
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in Carbon.framework [32-bit only] but deprecated in 10.3
=======
 *    Mac OS X:         in version 10.1 and later in Carbon.framework but deprecated in 10.3
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.1 and later in Carbon.framework but deprecated in 10.3
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.1 and later in Carbon.framework [32-bit only] but deprecated in 10.3
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.5 and later
 *    Non-Carbon CFM:   not available
 }
function TXNGetRectBounds( iTXNObject: TXNObject; oViewRect: RectPtr { can be NULL }; oDestinationRect: TXNLongRectPtr { can be NULL }; oTextRect: TXNLongRectPtr { can be NULL } ): OSStatus; external name '_TXNGetRectBounds';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


{
 *  TXNActivate()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use TXNSetScrollbarState API instead.
 *  
 *  Discussion:
 *    Make the TXNObject object active in the sense that it can be
 *    scrolled if it has scrollbars. If the TXNScrollBarState parameter
 *    is true than the scrollbars will be active even when the
 *    TXNObject is not focused (i.e. insertion point not active).  See
 *    the equivalent TXNSetScrollbarState. 
 *    
 *    This function should be used if you have multiple TXNObjects in a
 *    window, and you want them all to be scrollable even though only
 *    one at a time can have the keyboard focus.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    iTXNFrameID:
 *      TXNFrameID obtained from TXNNewObject.  Deprecated.  Pass 0.
 *    
 *    iActiveState:
 *      Boolean if true Scrollbars active even though TXNObject does
 *      not have the keyboard focus. if false scrollbars are synced
 *      with active state (i.e. a focused object has an active
 *      insertion point or selection and active scrollbars. An
 *      unfocused object has inactive selection (grayed or framed
 *      selection) and inactive scrollbars.  The latter state is the
 *      default and usually the one you use if you have one TXNObject
 *      in a window.
 *  
 *  Result:
 *    A result code indicating success or failure.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.3
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.3
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.3
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.3
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNActivate( iTXNObject: TXNObject; iTXNFrameID: TXNFrameID; iActiveState: TXNScrollBarState ): OSStatus; external name '_TXNActivate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


{
 *  TXNPointToOffset()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use TXNHIPointToOffset API instead.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    iPoint:
 *      A point (in local coordinates).
 *    
 *    oOffset:
 *      Offset corresponding to the point.
 *  
 *  Result:
 *    A result code indicating success or failure. Memory, out of
 *    bounds errors (if the point is out of the ViewRect).
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.3
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.3
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.3
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.3
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in Textension 1.1 and later
 }
function TXNPointToOffset( iTXNObject: TXNObject; iPoint: Point; var oOffset: TXNOffset ): OSStatus; external name '_TXNPointToOffset';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


{
 *  TXNOffsetToPoint()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use TXNOffsetToHIPoint API instead.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    iOffset:
 *      An offset.
 *    
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    oPoint:
 *      Point corresponding to the offset.
 *  
 *  Result:
 *    A result code indicating success or failure. Memory, out of
 *    bounds errors.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in Textension 1.1 and later
 }
function TXNOffsetToPoint( iTXNObject: TXNObject; iOffset: TXNOffset; var oPoint: Point ): OSStatus; external name '_TXNOffsetToPoint';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    oPoint:
 *      Point corresponding to the offset.
 *  
 *  Result:
 *    A result code indicating success or failure. Memory, out of
 *    bounds errors.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in Textension 1.1 and later
 }
function TXNOffsetToPoint( iTXNObject: TXNObject; iOffset: TXNOffset; var oPoint: Point ): OSStatus; external name '_TXNOffsetToPoint';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


{
 *  TXNTSMCheck()
 *  
 *  Summary:
 *    Deprecated.
 *  
 *  Discussion:
 *    Call this when WaitNextEvent returns false or there is no active
 *    TSMObject . The TXNObject parameter can be NULL which allows a
 *    client to call this function at any time.  This is necessary to
 *    insure input methods enough time to be reasonably responsive.
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque struct obtained from TXNNewObject.
 *    
 *    ioEvent:
 *      The event record.  Usually a NULL event.  If the event is not
 *      an NULL event on entry, and an input method consumes the event
 *      the event should return as a NULL event.
=======
 *    oPoint:
 *      Point corresponding to the offset.
>>>>>>> origin/fixes_2.4
 *  
 *  Result:
 *    A result code indicating success or failure. Memory, out of
 *    bounds errors.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in Textension 1.1 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function TXNOffsetToPoint( iTXNObject: TXNObject; iOffset: TXNOffset; var oPoint: Point ): OSStatus; external name '_TXNOffsetToPoint';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)
>>>>>>> origin/fixes_2.4


{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • DEPRECATED APIs, Mac OS X version 10.4 and later                                                  }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

{
 *  TXNCanUndo()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use TXNCanUndoAction API instead.
 *  
 *  Summary:
 *    Use this to determine if the Undo item in Edit menu should be
 *    highlighted or not. Tells you if last command was undoable.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    oTXNActionKey:
 *      The key code that the caller can use to pick a string to
 *      describe the undoable action in the undo item.  Pass in NULL if
 *      the string isn't needed.
 *  
 *  Result:
 *    Boolean: If True the last command is undoable and the undo item
 *    in the menu should be active.  If false last command cannot be
 *    undone and undo should be grayed in the menu.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNCanUndo( iTXNObject: TXNObject; oTXNActionKey: TXNActionKeyPtr { can be NULL } ): Boolean; external name '_TXNCanUndo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TXNCanRedo()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use TXNCanRedoAction API instead.
 *  
 *  Summary:
 *    Use this to determine if the current item on the undo stack is
 *    redoable.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    oTXNActionKey:
 *      The key code that the caller can use to pick a string to
 *      describe the redoable action in the redo item.  Pass in NULL if
 *      the string isn't needed.
 *  
 *  Result:
 *    If it returns true, then the redo item in the edit menu should be
 *    active.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNCanRedo( iTXNObject: TXNObject; oTXNActionKey: TXNActionKeyPtr { can be NULL } ): Boolean; external name '_TXNCanRedo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TXNGetActionChangeCount()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use TXNGetCountForActionType API instead.
 *  
 *  Summary:
 *    Retrieves the number of times the specified action(s) have
 *    occurred.
 *  
 *  Discussion:
 *    Explicit call to TXNClearActionChangeCount is needed when the
 *    counter(s) have to be reset.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    iOptions:
 *      Specify the the type of action changes to be include when
 *      retrieving the count.  Choose from the TXNOptions.
 *    
 *    oCount:
 *      The number of counts returned by the function.
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.1 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.1 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.1 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in Textension 1.3 and later
 }
function TXNGetActionChangeCount( iTXNObject: TXNObject; iOptions: TXNCountOptions; var oCount: ItemCount ): OSStatus; external name '_TXNGetActionChangeCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TXNClearActionChangeCount()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use TXNClearCountForActionType API instead.
 *  
 *  Summary:
 *    Reset the specified action counter(s) to zero.
 *  
 *  Discussion:
 *    Use kAllCountMask to reset everything.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    iOptions:
 *      Specify the the type of action changes to be included when
 *      resetting the count.  Choose from the TXNOptions.
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.1 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.1 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.1 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in Textension 1.3 and later
 }
function TXNClearActionChangeCount( iTXNObject: TXNObject; iOptions: TXNCountOptions ): OSStatus; external name '_TXNClearActionChangeCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TXNSetDataFromCFURLRef()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use TXNReadFromCFURL API instead.
 *  
 *  Summary:
 *    Replaces a range of data with the content of a file.
 *  
 *  Discussion:
 *    Uses URL file name extension to determine the type of the input
 *    file. If the entire content is replaced, calling TXNRevert will
 *    revert to the last saved CFURLRef.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.  Data will be
 *      added to this text object.
 *    
 *    iURL:
 *      The url referring to the file which contains the data you want
 *      to add to the object.
 *    
 *    iStartOffset:
 *      The starting offset at which to insert the file into a
 *      document.. If you want to replace the entire text content then
 *      set the iStartOffset parameter to kTXNStartOffset.
 *    
 *    iEndOffset:
 *      The ending position of the range being replaced by the file. If
 *      you want to replace the entire text content then set the
 *      iEndOffset parameter to kTXNEndOffset.
 *  
 *  Result:
 *    A result code indicating success or failure.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.3 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.3 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function TXNSetDataFromCFURLRef( iTXNObject: TXNObject; iURL: CFURLRef; iStartOffset: TXNOffset; iEndOffset: TXNOffset ): OSStatus; external name '_TXNSetDataFromCFURLRef';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TXNSave()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use TXNWriteRangeToCFURL API instead.
 *  
 *  Summary:
 *    Save the contents of the document as the given type.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    iType:
 *      The type of file to create.
 *    
 *    iResType:
 *      When saving file as plain TEXT the type of resource to save
 *      style information. Use kTXNMultipleStylesPerTextDocumentResType
 *      if your document contains multiple styles and you want a
 *      SimpleText like document.  Use
 *      kTXNSingleStylePerTextDocumentResType if the document has a
 *      single style and you would like a BBEdit, MPW, CW type of
 *      document.
 *    
 *    iPermanentEncoding:
 *      The encoding in which the document should be saved (Unicode,
 *      Text or System default).
 *    
 *    iFileSpecification:
 *      The file specification to which the document should be saved.
 *      The file must have been opened by the caller.  The file
 *      specification is remembered by the TXNObject and is used for
 *      any subsequent calls to TXNRevert.
 *    
 *    iDataReference:
 *      The data fork ref num.  This is used to write data to the data
 *      fork of the file. The data is written beginning at the current
 *      mark.
 *    
 *    iResourceReference:
 *      The resource fork ref num.  If the caller has specified that
 *      style information be saved as a resource (MPW or SimpleText)
 *      than this should be a valid reference to an open resource fork.
 *       If the txtn format is being used than this input value is
 *      ignored.
 *  
 *  Result:
 *    The result of writing the file.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNSave( iTXNObject: TXNObject; iType: TXNFileType; iResType: OSType; iPermanentEncoding: TXNPermanentTextEncodingType; const (*var*) iFileSpecification: FSSpec; iDataReference: SInt16; iResourceReference: SInt16 ): OSStatus; external name '_TXNSave';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • DEPRECATED APIs, Mac OS X version 10.5 and later                                                  }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Font Menu                                                                                         }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{
 *  TXNNewFontMenuObject()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use Font Panel instead.
 *  
 *  Summary:
 *    Get a FontMenuObject.  Caller can extract a fontmenu from this
 *    object and pass this object to the active TXNObject to handle
 *    events in the font menu.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iFontMenuHandle:
 *      An empty menu handle (well the title is there) that the caller
 *      created via NewMenu or GetNewMenu. This menu handle should not
 *      be disposed before the returned TXNFontMenuObject has been
 *      disposed via TXNDisposeFontMenuObject.
 *    
 *    iMenuID:
 *      The MenuID for iFontMenuHandle.
 *    
 *    iStartHierMenuID:
 *      The first MenuID to use if any hierarchical menus need to be
 *      created. TXNNewFontMenuObject uses SetMenuItemHierarchicalID
 *      when creating hierarchial menus.  The iStartHierMenuID must
 *      therefore follow the rules for this function.  On systems less
 *      than system 8.5 the submenuID must be less than 255.  For
 *      systems above system 8.5 the range can be as large as 32767.
 *      However, it is important to remember that TXNNewFontMenuObject
 *      only uses iStartHierMenuID as a starting id when adding
 *      hierarchical menus.  Therefore provide plenty of room to
 *      increment this value. For example, on a system less than 8.5 it
 *      would be good to start at 175.  On systems greater than 8.5 it
 *      is probably a good idea to not use a value higher than 32000.
 *    
 *    oTXNFontMenuObject:
 *      A font menu object.
 *  
 *  Result:
 *    A result code indicating success or failure.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNNewFontMenuObject( iFontMenuHandle: MenuRef; iMenuID: SInt16; iStartHierMenuID: SInt16; var oTXNFontMenuObject: TXNFontMenuObject ): OSStatus; external name '_TXNNewFontMenuObject';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  TXNGetFontMenuHandle()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use Font Panel instead.
 *  
 *  Summary:
 *    Get the MenuRef from the TXNFontMenuObject.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNFontMenuObject:
 *      A Font Menu Object obtained from TXNNewFontMenuObject.
 *    
 *    oFontMenuHandle:
 *      The returned font menu. Returned value could be NULL.
 *  
 *  Result:
 *    A result code indicating success or failure.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNGetFontMenuHandle( iTXNFontMenuObject: TXNFontMenuObject; var oFontMenuHandle: MenuRef ): OSStatus; external name '_TXNGetFontMenuHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


function TXNGetFontMenuRef( iTXNFontMenuObject: TXNFontMenuObject; var oFontMenuHandle: MenuRef ): OSStatus; external name '_TXNGetFontMenuHandle';

{
 *  TXNDisposeFontMenuObject()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use Font Panel instead.
 *  
 *  Summary:
 *    Dispose a TXNFontMenuObject and its font menu handle.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNFontMenuObject:
 *      A Font Menu Object obtained from TXNNewFontMenuObject.
 *  
 *  Result:
 *    A result code indicating success or failure.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNDisposeFontMenuObject( iTXNFontMenuObject: TXNFontMenuObject ): OSStatus; external name '_TXNDisposeFontMenuObject';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  TXNDoFontMenuSelection()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use Font Panel instead.
 *  
 *  Summary:
 *    Given the menuID and menu item returned by MenuSelect determine
 *    the selected font and change the current selection to be that
 *    Font.  If the input TXNObject is not active a parameter error is
 *    returned.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    iTXNFontMenuObject:
 *      A Font Menu Object obtained from TXNNewFontMenuObject.
 *    
 *    iMenuID:
 *      SInt16 the ID of the selected menu.
 *    
 *    iMenuItem:
 *      The item that was selected.
 *  
 *  Result:
 *    A result code indicating success or failure.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNDoFontMenuSelection( iTXNObject: TXNObject; iTXNFontMenuObject: TXNFontMenuObject; iMenuID: SInt16; iMenuItem: SInt16 ): OSStatus; external name '_TXNDoFontMenuSelection';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  TXNPrepareFontMenu()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use Font Panel instead.
 *  
 *  Summary:
 *    Prepares a Font menu for display.
 *  
 *  Discussion:
 *    You should call the TXNPrepareFontMenu function just before your
 *    application opens the Font menu for your user. If the text
 *    object’s current selection is a single font, MLTE places a
 *    checkmark next to the menu item for that font.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      The text object that identifies the document with the Font menu
 *      you want to prepare. Pass NULL to display an inactive menu
 *      (dimmed).
 *    
 *    iTXNFontMenuObject:
 *      A Font menu object.
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNPrepareFontMenu( iTXNObject: TXNObject { can be NULL }; iTXNFontMenuObject: TXNFontMenuObject ): OSStatus; external name '_TXNPrepareFontMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{——————————————————————————————————————————————————————————————————————————————————————————————————————}
{  • Font Defaults                                                                                     }
{——————————————————————————————————————————————————————————————————————————————————————————————————————}

{
 *  TXNSetFontDefaults()   *** DEPRECATED ***
 *  
 *  Summary:
 *    For a given TXNObject specify the font defaults for each script.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    iCount:
 *      Count of FontDescriptions.
 *    
 *    iFontDefaults:
 *      Array of FontDescriptions.
 *  
 *  Result:
 *    A result code indicating success or failure.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNSetFontDefaults( iTXNObject: TXNObject; iCount: ItemCount; {const} iFontDefaults: {variable-size-array} TXNMacOSPreferredFontDescriptionPtr ): OSStatus; external name '_TXNSetFontDefaults';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TXNGetFontDefaults()   *** DEPRECATED ***
 *  
 *  Summary:
 *    For a given TXNObject make a copy of the font defaults.
 *  
 *  Discussion:
 *    To determine how many font descriptions need to be in the array
 *    you should call this function with a NULL for the array.  iCount
 *    will return with the number of font defaults currently stored.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    iTXNObject:
 *      Opaque TXNObject obtained from TXNCreateObject.
 *    
 *    ioCount:
 *      Count of FontDescriptions in the array.
 *    
 *    oFontDefaults:
 *      Array of FontDescriptins to be filled out.
 *  
 *  Result:
 *    A result code indicating success or failure.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in Textension 1.0 and later
 }
function TXNGetFontDefaults( iTXNObject: TXNObject; var ioCount: ItemCount; oFontDefaults: {variable-size-array} TXNMacOSPreferredFontDescriptionPtr { can be NULL } ): OSStatus; external name '_TXNGetFontDefaults';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{$endc} {not TARGET_CPU_64}

{$endc} {TARGET_OS_MAC}

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
