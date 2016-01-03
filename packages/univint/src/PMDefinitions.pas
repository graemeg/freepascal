{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       PrintCore/PMDefinitions.h
 
     Contains:   Carbon Printing Manager Interfaces.
 
     Copyright (c) 1998-2006,2008 by Apple Inc. All Rights Reserved.
=======
=======
>>>>>>> origin/fixes_2_2
     File:       PMDefinitions.p
 
     Contains:   Carbon Printing Manager Interfaces.
 
     Version:    Technology: Mac OS X
                 Release:    Universal Interfaces 3.4.2
 
     Copyright:  © 1998-2002 by Apple Computer, Inc., all rights reserved
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     File:       PrintCore/PMDefinitions.h
 
     Contains:   Carbon Printing Manager Interfaces.
 
     Copyright (c) 1998-2006,2008 by Apple Inc. All Rights Reserved.
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
{    Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{    Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{    Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{    Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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
{    Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit PMDefinitions;
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
=======
{$elsec}
=======
{$elsec}
>>>>>>> origin/cpstrnew
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
<<<<<<< HEAD
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
uses MacTypes,MacErrors,CFString;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not MACOSALLINCLUDE}
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

{$ifc TARGET_OS_MAC}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
{$ifc TARGET_OS_MAC}

>>>>>>> graemeg/cpstrnew
=======
{$ifc TARGET_OS_MAC}

>>>>>>> graemeg/cpstrnew
=======
{$ifc TARGET_OS_MAC}

>>>>>>> graemeg/cpstrnew
=======


{$ifc TARGET_OS_MAC}

>>>>>>> origin/cpstrnew
{$ifc not TARGET_CPU_64}
{$ALIGN MAC68K}
{$elsec}
{$packrecords c}
{$endc} {not TARGET_CPU_64}

{ Printing objects }
type
	PMObject = UnivPtr;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	PMPrintSettings = ^OpaquePMPrintSettings; { an opaque type }
	OpaquePMPrintSettings = record end;
	PMPrintSettingsPtr = ^PMPrintSettings; { when a var xx:PMPrintSettings parameter can be nil, it is changed to xx: PMPrintSettingsPtr }
	PMPageFormat = ^OpaquePMPageFormat; { an opaque type }
	OpaquePMPageFormat = record end;
	PMPageFormatPtr = ^PMPageFormat; { when a var xx:PMPageFormat parameter can be nil, it is changed to xx: PMPageFormatPtr }
	PMPrintSession = ^OpaquePMPrintSession; { an opaque type }
	OpaquePMPrintSession = record end;
	PMPrintSessionPtr = ^PMPrintSession; { when a var xx:PMPrintSession parameter can be nil, it is changed to xx: PMPrintSessionPtr }
	PMPrinter = ^OpaquePMPrinter; { an opaque type }
	OpaquePMPrinter = record end;
	PMPrinterPtr = ^PMPrinter; { when a var xx:PMPrinter parameter can be nil, it is changed to xx: PMPrinterPtr }
	PMServer = ^OpaquePMServer; { an opaque type }
	OpaquePMServer = record end;
	PMServerPtr = ^PMServer; { when a var xx:PMPrinter parameter can be nil, it is changed to xx: PMServerPtr }
	PMPreset = ^OpaquePMPreset; { an opaque type }
	OpaquePMPreset = record end;
	PMPresetPtr = ^PMPreset; { when a var xx:PMPrinter parameter can be nil, it is changed to xx: PMPresetPtr }
	PMPaper = ^OpaquePMPaper; { an opaque type }
	OpaquePMPaper = record end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	PMPrintSettings = ^SInt32; { an opaque type }
	PMPrintSettingsPtr = ^PMPrintSettings; { when a var xx:PMPrintSettings parameter can be nil, it is changed to xx: PMPrintSettingsPtr }
	PMPageFormat = ^SInt32; { an opaque type }
	PMPageFormatPtr = ^PMPageFormat; { when a var xx:PMPageFormat parameter can be nil, it is changed to xx: PMPageFormatPtr }
	PMPrintSession = ^SInt32; { an opaque type }
	PMPrintSessionPtr = ^PMPrintSession; { when a var xx:PMPrintSession parameter can be nil, it is changed to xx: PMPrintSessionPtr }
	PMPrinter = ^SInt32; { an opaque type }
	PMPrinterPtr = ^PMPrinter; { when a var xx:PMPrinter parameter can be nil, it is changed to xx: PMPrinterPtr }
	PMServer = ^SInt32; { an opaque type }
	PMServerPtr = ^PMServer; { when a var xx:PMPrinter parameter can be nil, it is changed to xx: PMServerPtr }
	PMPreset = ^SInt32; { an opaque type }
	PMPresetPtr = ^PMPreset; { when a var xx:PMPrinter parameter can be nil, it is changed to xx: PMPresetPtr }
	PMPaper = ^SInt32; { an opaque type }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	PMPaperPtr = ^PMPaper; { when a var xx:PMPrinter parameter can be nil, it is changed to xx: PMPaperPtr }
const
	kPMCancel = $0080; { user hit cancel button in dialog }

	kPMNoData = nil; { for general use }
	kPMDontWantSize = nil; { for parameters which return size information }
	kPMDontWantData = nil; { for parameters which return data }
	kPMDontWantBoolean = nil; { for parameters which take a boolean reference }
	kPMNoReference = nil; { for parameters which take an address pointer }
{	kPMDuplexDefault = kPMDuplexNone; -- moved below, after declaration of kPMDuplexNone }
<<<<<<< HEAD

{ for parameters which take a PrintSettings reference }
	kPMNoPrintSettings = nil;

=======
	PMPaperPtr = ^PMPaper; { when a var xx:PMPrinter parameter can be nil, it is changed to xx: PMPaperPtr }
const
	kPMCancel = $0080; { user hit cancel button in dialog }
=======
	PMPaperPtr = ^PMPaper; { when a var xx:PMPrinter parameter can be nil, it is changed to xx: PMPaperPtr }
const
	kPMCancel = $0080; { user hit cancel button in dialog }
>>>>>>> graemeg/cpstrnew

	kPMNoData = nil; { for general use }
	kPMDontWantSize = nil; { for parameters which return size information }
	kPMDontWantData = nil; { for parameters which return data }
	kPMDontWantBoolean = nil; { for parameters which take a boolean reference }
	kPMNoReference = nil; { for parameters which take an address pointer }
{	kPMDuplexDefault = kPMDuplexNone; -- moved below, after declaration of kPMDuplexNone }

{ for parameters which take a PrintSettings reference }
	kPMNoPrintSettings = nil;

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
{ for parameters which take a PageFormat reference }
	kPMNoPageFormat = nil;

=======

{ for parameters which take a PrintSettings reference }
	kPMNoPrintSettings = nil;

{ for parameters which take a PageFormat reference }
	kPMNoPageFormat = nil;

>>>>>>> origin/cpstrnew
{ for parameters which take a Server reference }
	kPMServerLocal = nil;
type
	PMDestinationType = UInt16;
const
	kPMDestinationInvalid = 0;
	kPMDestinationPrinter = 1;
	kPMDestinationFile = 2;
	kPMDestinationFax = 3;
	kPMDestinationPreview = 4;
	kPMDestinationProcessPDF = 5;
=======
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4


{$ifc TARGET_OS_MAC}

{$ifc not TARGET_CPU_64}
{$ALIGN MAC68K}
{$elsec}
{$packrecords c}
{$endc} {not TARGET_CPU_64}

{ Printing objects }

type
	PMObject = UnivPtr;
	PMPrintSettings = ^SInt32; { an opaque type }
	PMPrintSettingsPtr = ^PMPrintSettings; { when a var xx:PMPrintSettings parameter can be nil, it is changed to xx: PMPrintSettingsPtr }
	PMPageFormat = ^SInt32; { an opaque type }
	PMPageFormatPtr = ^PMPageFormat; { when a var xx:PMPageFormat parameter can be nil, it is changed to xx: PMPageFormatPtr }
	PMPrintSession = ^SInt32; { an opaque type }
	PMPrintSessionPtr = ^PMPrintSession; { when a var xx:PMPrintSession parameter can be nil, it is changed to xx: PMPrintSessionPtr }
	PMPrinter = ^SInt32; { an opaque type }
	PMPrinterPtr = ^PMPrinter; { when a var xx:PMPrinter parameter can be nil, it is changed to xx: PMPrinterPtr }
	PMServer = ^SInt32; { an opaque type }
	PMServerPtr = ^PMServer; { when a var xx:PMPrinter parameter can be nil, it is changed to xx: PMServerPtr }
	PMPreset = ^SInt32; { an opaque type }
	PMPresetPtr = ^PMPreset; { when a var xx:PMPrinter parameter can be nil, it is changed to xx: PMPresetPtr }
	PMPaper = ^SInt32; { an opaque type }
	PMPaperPtr = ^PMPaper; { when a var xx:PMPrinter parameter can be nil, it is changed to xx: PMPaperPtr }
const
	kPMCancel = $0080; { user hit cancel button in dialog }

	kPMNoData = nil; { for general use }
	kPMDontWantSize = nil; { for parameters which return size information }
	kPMDontWantData = nil; { for parameters which return data }
	kPMDontWantBoolean = nil; { for parameters which take a boolean reference }
	kPMNoReference = nil; { for parameters which take an address pointer }
{	kPMDuplexDefault = kPMDuplexNone; -- moved below, after declaration of kPMDuplexNone }

{ for parameters which take a PrintSettings reference }
	kPMNoPrintSettings = nil;

{ for parameters which take a PageFormat reference }
	kPMNoPageFormat = nil;

{ for parameters which take a Server reference }
	kPMServerLocal = nil;
type
	PMDestinationType = UInt16;
const
<<<<<<< HEAD
	kPMDestinationInvalid		= 0;
	kPMDestinationPrinter		= 1;
	kPMDestinationFile			= 2;
	kPMDestinationFax			= 3;
	kPMDestinationPreview		= 4;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kPMDestinationInvalid = 0;
	kPMDestinationPrinter = 1;
	kPMDestinationFile = 2;
	kPMDestinationFax = 3;
	kPMDestinationPreview = 4;
	kPMDestinationProcessPDF = 5;
>>>>>>> origin/fixes_2.4

	kPMDestinationTypeDefault	= kPMDestinationPrinter;
	
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	PMOrientation = UInt16;
const
	kPMPortrait = 1;
	kPMLandscape = 2;
	kPMReversePortrait = 3;    { will revert to kPortrait for current drivers }
	kPMReverseLandscape = 4;     { will revert to kLandscape for current drivers }
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/cpstrnew

{ Printer states }
type
	PMPrinterState = UInt16;
<<<<<<< HEAD
const
	kPMPrinterIdle = 3;
	kPMPrinterProcessing = 4;
	kPMPrinterStopped = 5;
=======
>>>>>>> graemeg/cpstrnew

type
<<<<<<< HEAD
	PMColorSpaceModel = UInt32;
const
	kPMUnknownColorSpaceModel = 0;
	kPMGrayColorSpaceModel = 1;
	kPMRGBColorSpaceModel = 2;
	kPMCMYKColorSpaceModel = 3;
	kPMDevNColorSpaceModel = 4;
	
	kPMColorSpaceModelCount = 4; { total number of color space models supported }
	
{ Print quality modes "standard options" }
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
type
	PMQualityMode = UInt32;
const
	kPMQualityLowest = $0000; { Absolute lowest print quality }
	kPMQualityInkSaver = $0001; { Saves ink but may be slower }
	kPMQualityDraft = $0004; { Print at highest speed, ink used is secondary consideration }
	kPMQualityNormal = $0008; { Print in printers "general usage" mode for good balance between quality and speed }
	kPMQualityPhoto = $000B; { Optimize quality of photos on the page. Speed is not a concern }
	kPMQualityBest = $000D; { Get best quality output for all objects and photos. }
	kPMQualityHighest = $000F; { Absolute highest quality attained from a printers }

=======
	PMPrinterState = UInt16;
const
	kPMPrinterIdle = 3;
	kPMPrinterProcessing = 4;
	kPMPrinterStopped = 5;
>>>>>>> graemeg/cpstrnew
=======
const
	kPMPrinterIdle = 3;
	kPMPrinterProcessing = 4;
	kPMPrinterStopped = 5;
>>>>>>> origin/cpstrnew

{ Constants for our "standard" paper types }
type
<<<<<<< HEAD
<<<<<<< HEAD
	PMPaperType = UInt32;
const
<<<<<<< HEAD
=======
=======
	PMColorSpaceModel = UInt32;
const
=======
	PMColorSpaceModel = UInt32;
const
>>>>>>> origin/cpstrnew
	kPMUnknownColorSpaceModel = 0;
	kPMGrayColorSpaceModel = 1;
	kPMRGBColorSpaceModel = 2;
	kPMCMYKColorSpaceModel = 3;
	kPMDevNColorSpaceModel = 4;
	
	kPMColorSpaceModelCount = 4; { total number of color space models supported }
	
{ Print quality modes "standard options" }
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
type
	PMQualityMode = UInt32;
const
	kPMQualityLowest = $0000; { Absolute lowest print quality }
	kPMQualityInkSaver = $0001; { Saves ink but may be slower }
	kPMQualityDraft = $0004; { Print at highest speed, ink used is secondary consideration }
	kPMQualityNormal = $0008; { Print in printers "general usage" mode for good balance between quality and speed }
	kPMQualityPhoto = $000B; { Optimize quality of photos on the page. Speed is not a concern }
	kPMQualityBest = $000D; { Get best quality output for all objects and photos. }
	kPMQualityHighest = $000F; { Absolute highest quality attained from a printers }


{ Constants for our "standard" paper types }
type
	PMPaperType = UInt32;
const
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	kPMPaperTypeUnknown = $0000; { Not sure yet what paper type we have. }
	kPMPaperTypePlain = $0001; { Plain paper }
	kPMPaperTypeCoated = $0002; { Has a special coating for sharper images and text }
	kPMPaperTypePremium = $0003; { Special premium coated paper }
	kPMPaperTypeGlossy = $0004; { High gloss special coating }
	kPMPaperTypeTransparency = $0005; { Used for overheads }
	kPMPaperTypeTShirt = $0006; { Used to iron on t-shirts }

{ Scaling alignment: }
<<<<<<< HEAD
<<<<<<< HEAD
type
	PMScalingAlignment = UInt16;
const
	kPMScalingPinTopLeft = 1;
	kPMScalingPinTopRight = 2;
	kPMScalingPinBottomLeft = 3;
	kPMScalingPinBottomRight = 4;
	kPMScalingCenterOnPaper = 5;
	kPMScalingCenterOnImgArea = 6;

{ Duplex Mode: }
type
	PMDuplexMode = UInt32;
const
	kPMDuplexNone = $0001; { Print only on one side of sheet of paper }
	kPMDuplexNoTumble = $0002; { Print on both sides of the paper, with no tumbling. }
	kPMDuplexTumble = $0003; { Print on both sides of the paper, tumbling on. }
	kPMSimplexTumble = $0004; { Print on only one side of the paper, but tumble the images while printing. }
	kPMDuplexDefault = kPMDuplexNone;

{ Layout directions: }
type
	PMLayoutDirection = UInt16;
const
{ Horizontal-major directions: }
	kPMLayoutLeftRightTopBottom = 1;    { English reading direction. }
	kPMLayoutLeftRightBottomTop = 2;
	kPMLayoutRightLeftTopBottom = 3;
	kPMLayoutRightLeftBottomTop = 4;    { Vertical-major directions: }
	kPMLayoutTopBottomLeftRight = 5;
	kPMLayoutTopBottomRightLeft = 6;
	kPMLayoutBottomTopLeftRight = 7;
	kPMLayoutBottomTopRightLeft = 8;

{ Page borders: }
type
	PMBorderType = UInt16;
const
	kPMBorderSingleHairline = 1;
	kPMBorderDoubleHairline = 2;
	kPMBorderSingleThickline = 3;
	kPMBorderDoubleThickline = 4;

{ 
 Options for which items to show inline in the print dialog
 This is only meant to be used in Carbon environment
 }
type
	PMPrintDialogOptionFlags = OptionBits;
const
	kPMHideInlineItems = 0 shl 0; { show nothing in the inline portion of print dialog }
	kPMShowDefaultInlineItems = 1 shl 15; { show the default set of items (copies & pages) in the inline portion of print dialog }
	kPMShowInlineCopies = 1 shl 0; { show Copies edit text, Collate check box and Two Sided check box (if printer supports it) in top portion of print dialog }
	kPMShowInlinePageRange = 1 shl 1; { show Paper Range buttons and From & To Page edit text items in top portion of print dialog }
	kPMShowInlinePageRangeWithSelection = 1 shl 6; { show Paper Range buttons with the addition of a Selection button and the From & To Page edit text items in top portion of print dialog }
	kPMShowInlinePaperSize = 1 shl 2; { show Paper Size popup menu in top portion of print dialog }
	kPMShowInlineOrientation = 1 shl 3; { show Orientation buttons in top portion of print dialog }
	kPMShowInlineScale = 1 shl 7; { show Scaling edit text in top portion of print dialog }
	kPMShowPageAttributesPDE = 1 shl 8; { add a PDE to the print dialog that contains the Page Setup information (paper size, orientation and scale) }

type
	PMPPDDomain = UInt16;
const
	kAllPPDDomains = 1;
	kSystemPPDDomain = 2;
	kLocalPPDDomain = 3;
	kNetworkPPDDomain = 4;
	kUserPPDDomain = 5;
	kCUPSPPDDomain = 6;


=======
	kPMPaperTypeUnknown = $0000; { Not sure yet what paper type we have. }
	kPMPaperTypePlain = $0001; { Plain paper }
	kPMPaperTypeCoated = $0002; { Has a special coating for sharper images and text }
	kPMPaperTypePremium = $0003; { Special premium coated paper }
	kPMPaperTypeGlossy = $0004; { High gloss special coating }
	kPMPaperTypeTransparency = $0005; { Used for overheads }
	kPMPaperTypeTShirt = $0006; { Used to iron on t-shirts }

{ Scaling alignment: }
type
	PMScalingAlignment = UInt16;
const
	kPMScalingPinTopLeft = 1;
	kPMScalingPinTopRight = 2;
	kPMScalingPinBottomLeft = 3;
	kPMScalingPinBottomRight = 4;
	kPMScalingCenterOnPaper = 5;
	kPMScalingCenterOnImgArea = 6;

{ Duplex Mode: }
type
	PMDuplexMode = UInt32;
const
	kPMDuplexNone = $0001; { Print only on one side of sheet of paper }
	kPMDuplexNoTumble = $0002; { Print on both sides of the paper, with no tumbling. }
	kPMDuplexTumble = $0003; { Print on both sides of the paper, tumbling on. }
	kPMSimplexTumble = $0004; { Print on only one side of the paper, but tumble the images while printing. }
	kPMDuplexDefault = kPMDuplexNone;

{ Layout directions: }
type
	PMLayoutDirection = UInt16;
const
{ Horizontal-major directions: }
	kPMLayoutLeftRightTopBottom = 1;    { English reading direction. }
	kPMLayoutLeftRightBottomTop = 2;
	kPMLayoutRightLeftTopBottom = 3;
	kPMLayoutRightLeftBottomTop = 4;    { Vertical-major directions: }
	kPMLayoutTopBottomLeftRight = 5;
	kPMLayoutTopBottomRightLeft = 6;
	kPMLayoutBottomTopLeftRight = 7;
	kPMLayoutBottomTopRightLeft = 8;

{ Page borders: }
type
	PMBorderType = UInt16;
const
=======
type
	PMScalingAlignment = UInt16;
const
	kPMScalingPinTopLeft = 1;
	kPMScalingPinTopRight = 2;
	kPMScalingPinBottomLeft = 3;
	kPMScalingPinBottomRight = 4;
	kPMScalingCenterOnPaper = 5;
	kPMScalingCenterOnImgArea = 6;

{ Duplex Mode: }
type
	PMDuplexMode = UInt32;
const
	kPMDuplexNone = $0001; { Print only on one side of sheet of paper }
	kPMDuplexNoTumble = $0002; { Print on both sides of the paper, with no tumbling. }
	kPMDuplexTumble = $0003; { Print on both sides of the paper, tumbling on. }
	kPMSimplexTumble = $0004; { Print on only one side of the paper, but tumble the images while printing. }
	kPMDuplexDefault = kPMDuplexNone;

{ Layout directions: }
type
	PMLayoutDirection = UInt16;
const
{ Horizontal-major directions: }
	kPMLayoutLeftRightTopBottom = 1;    { English reading direction. }
	kPMLayoutLeftRightBottomTop = 2;
	kPMLayoutRightLeftTopBottom = 3;
	kPMLayoutRightLeftBottomTop = 4;    { Vertical-major directions: }
	kPMLayoutTopBottomLeftRight = 5;
	kPMLayoutTopBottomRightLeft = 6;
	kPMLayoutBottomTopLeftRight = 7;
	kPMLayoutBottomTopRightLeft = 8;

{ Page borders: }
type
	PMBorderType = UInt16;
const
>>>>>>> origin/cpstrnew
	kPMBorderSingleHairline = 1;
	kPMBorderDoubleHairline = 2;
	kPMBorderSingleThickline = 3;
	kPMBorderDoubleThickline = 4;

{ 
 Options for which items to show inline in the print dialog
 This is only meant to be used in Carbon environment
 }
<<<<<<< HEAD
type
	PMPrintDialogOptionFlags = OptionBits;
const
	kPMHideInlineItems = 0 shl 0; { show nothing in the inline portion of print dialog }
	kPMShowDefaultInlineItems = 1 shl 15; { show the default set of items (copies & pages) in the inline portion of print dialog }
	kPMShowInlineCopies = 1 shl 0; { show Copies edit text, Collate check box and Two Sided check box (if printer supports it) in top portion of print dialog }
	kPMShowInlinePageRange = 1 shl 1; { show Paper Range buttons and From & To Page edit text items in top portion of print dialog }
	kPMShowInlinePageRangeWithSelection = 1 shl 6; { show Paper Range buttons with the addition of a Selection button and the From & To Page edit text items in top portion of print dialog }
	kPMShowInlinePaperSize = 1 shl 2; { show Paper Size popup menu in top portion of print dialog }
	kPMShowInlineOrientation = 1 shl 3; { show Orientation buttons in top portion of print dialog }
	kPMShowInlineScale = 1 shl 7; { show Scaling edit text in top portion of print dialog }
	kPMShowPageAttributesPDE = 1 shl 8; { add a PDE to the print dialog that contains the Page Setup information (paper size, orientation and scale) }

type
	PMPPDDomain = UInt16;
const
	kAllPPDDomains = 1;
	kSystemPPDDomain = 2;
	kLocalPPDDomain = 3;
	kNetworkPPDDomain = 4;
	kUserPPDDomain = 5;
	kCUPSPPDDomain = 6;

=======
type
	PMPrintDialogOptionFlags = OptionBits;
const
	kPMHideInlineItems = 0 shl 0; { show nothing in the inline portion of print dialog }
	kPMShowDefaultInlineItems = 1 shl 15; { show the default set of items (copies & pages) in the inline portion of print dialog }
	kPMShowInlineCopies = 1 shl 0; { show Copies edit text, Collate check box and Two Sided check box (if printer supports it) in top portion of print dialog }
	kPMShowInlinePageRange = 1 shl 1; { show Paper Range buttons and From & To Page edit text items in top portion of print dialog }
	kPMShowInlinePageRangeWithSelection = 1 shl 6; { show Paper Range buttons with the addition of a Selection button and the From & To Page edit text items in top portion of print dialog }
	kPMShowInlinePaperSize = 1 shl 2; { show Paper Size popup menu in top portion of print dialog }
	kPMShowInlineOrientation = 1 shl 3; { show Orientation buttons in top portion of print dialog }
	kPMShowInlineScale = 1 shl 7; { show Scaling edit text in top portion of print dialog }
	kPMShowPageAttributesPDE = 1 shl 8; { add a PDE to the print dialog that contains the Page Setup information (paper size, orientation and scale) }
>>>>>>> origin/cpstrnew

>>>>>>> graemeg/cpstrnew
=======
type
<<<<<<< HEAD
	PMScalingAlignment = UInt16;
const
	kPMScalingPinTopLeft = 1;
	kPMScalingPinTopRight = 2;
	kPMScalingPinBottomLeft = 3;
	kPMScalingPinBottomRight = 4;
	kPMScalingCenterOnPaper = 5;
	kPMScalingCenterOnImgArea = 6;

{ Duplex Mode: }
type
	PMDuplexMode = UInt32;
const
	kPMDuplexNone = $0001; { Print only on one side of sheet of paper }
	kPMDuplexNoTumble = $0002; { Print on both sides of the paper, with no tumbling. }
	kPMDuplexTumble = $0003; { Print on both sides of the paper, tumbling on. }
	kPMSimplexTumble = $0004; { Print on only one side of the paper, but tumble the images while printing. }
	kPMDuplexDefault = kPMDuplexNone;

{ Layout directions: }
type
	PMLayoutDirection = UInt16;
const
{ Horizontal-major directions: }
	kPMLayoutLeftRightTopBottom = 1;    { English reading direction. }
	kPMLayoutLeftRightBottomTop = 2;
	kPMLayoutRightLeftTopBottom = 3;
	kPMLayoutRightLeftBottomTop = 4;    { Vertical-major directions: }
	kPMLayoutTopBottomLeftRight = 5;
	kPMLayoutTopBottomRightLeft = 6;
	kPMLayoutBottomTopLeftRight = 7;
	kPMLayoutBottomTopRightLeft = 8;

{ Page borders: }
type
	PMBorderType = UInt16;
const
	kPMBorderSingleHairline = 1;
	kPMBorderDoubleHairline = 2;
	kPMBorderSingleThickline = 3;
	kPMBorderDoubleThickline = 4;

{ 
 Options for which items to show inline in the print dialog
 This is only meant to be used in Carbon environment
 }
type
	PMPrintDialogOptionFlags = OptionBits;
const
	kPMHideInlineItems = 0 shl 0; { show nothing in the inline portion of print dialog }
	kPMShowDefaultInlineItems = 1 shl 15; { show the default set of items (copies & pages) in the inline portion of print dialog }
	kPMShowInlineCopies = 1 shl 0; { show Copies edit text, Collate check box and Two Sided check box (if printer supports it) in top portion of print dialog }
	kPMShowInlinePageRange = 1 shl 1; { show Paper Range buttons and From & To Page edit text items in top portion of print dialog }
	kPMShowInlinePageRangeWithSelection = 1 shl 6; { show Paper Range buttons with the addition of a Selection button and the From & To Page edit text items in top portion of print dialog }
	kPMShowInlinePaperSize = 1 shl 2; { show Paper Size popup menu in top portion of print dialog }
	kPMShowInlineOrientation = 1 shl 3; { show Orientation buttons in top portion of print dialog }
	kPMShowInlineScale = 1 shl 7; { show Scaling edit text in top portion of print dialog }
	kPMShowPageAttributesPDE = 1 shl 8; { add a PDE to the print dialog that contains the Page Setup information (paper size, orientation and scale) }

type
	PMPPDDomain = UInt16;
const
	kAllPPDDomains = 1;
	kSystemPPDDomain = 2;
	kLocalPPDDomain = 3;
	kNetworkPPDDomain = 4;
	kUserPPDDomain = 5;
	kCUPSPPDDomain = 6;


>>>>>>> graemeg/cpstrnew
=======
	PMPPDDomain = UInt16;
const
	kAllPPDDomains = 1;
	kSystemPPDDomain = 2;
	kLocalPPDDomain = 3;
	kNetworkPPDDomain = 4;
	kUserPPDDomain = 5;
	kCUPSPPDDomain = 6;


>>>>>>> origin/cpstrnew
{ Description types }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMPPDDescriptionType CFSTRP('PMPPDDescriptionType')}
{$endc}
{ Document format strings }
=======
=======
>>>>>>> origin/fixes_2_2
	PMTag 						= UInt32;
=======
	PMOrientation = UInt16;
>>>>>>> origin/fixes_2.4
const
	kPMPortrait = 1;
	kPMLandscape = 2;
	kPMReversePortrait = 3;    { will revert to kPortrait for current drivers }
	kPMReverseLandscape = 4;     { will revert to kLandscape for current drivers }

{ Printer states }
type
	PMPrinterState = UInt16;
const
	kPMPrinterIdle = 3;
	kPMPrinterProcessing = 4;
	kPMPrinterStopped = 5;

type
	PMColorSpaceModel = UInt32;
const
	kPMUnknownColorSpaceModel = 0;
	kPMGrayColorSpaceModel = 1;
	kPMRGBColorSpaceModel = 2;
	kPMCMYKColorSpaceModel = 3;
	kPMDevNColorSpaceModel = 4;
	
	kPMColorSpaceModelCount = 4; { total number of color space models supported }
	
{ Print quality modes "standard options" }
type
	PMQualityMode = UInt32;
const
	kPMQualityLowest = $0000; { Absolute lowest print quality }
	kPMQualityInkSaver = $0001; { Saves ink but may be slower }
	kPMQualityDraft = $0004; { Print at highest speed, ink used is secondary consideration }
	kPMQualityNormal = $0008; { Print in printers "general usage" mode for good balance between quality and speed }
	kPMQualityPhoto = $000B; { Optimize quality of photos on the page. Speed is not a concern }
	kPMQualityBest = $000D; { Get best quality output for all objects and photos. }
	kPMQualityHighest = $000F; { Absolute highest quality attained from a printers }


{ Constants for our "standard" paper types }
type
	PMPaperType = UInt32;
const
	kPMPaperTypeUnknown = $0000; { Not sure yet what paper type we have. }
	kPMPaperTypePlain = $0001; { Plain paper }
	kPMPaperTypeCoated = $0002; { Has a special coating for sharper images and text }
	kPMPaperTypePremium = $0003; { Special premium coated paper }
	kPMPaperTypeGlossy = $0004; { High gloss special coating }
	kPMPaperTypeTransparency = $0005; { Used for overheads }
	kPMPaperTypeTShirt = $0006; { Used to iron on t-shirts }

{ Scaling alignment: }
type
	PMScalingAlignment = UInt16;
const
	kPMScalingPinTopLeft = 1;
	kPMScalingPinTopRight = 2;
	kPMScalingPinBottomLeft = 3;
	kPMScalingPinBottomRight = 4;
	kPMScalingCenterOnPaper = 5;
	kPMScalingCenterOnImgArea = 6;

{ Duplex Mode: }
type
	PMDuplexMode = UInt32;
const
	kPMDuplexNone = $0001; { Print only on one side of sheet of paper }
	kPMDuplexNoTumble = $0002; { Print on both sides of the paper, with no tumbling. }
	kPMDuplexTumble = $0003; { Print on both sides of the paper, tumbling on. }
	kPMSimplexTumble = $0004; { Print on only one side of the paper, but tumble the images while printing. }
	kPMDuplexDefault = kPMDuplexNone;

{ Layout directions: }
type
	PMLayoutDirection = UInt16;
const
{ Horizontal-major directions: }
	kPMLayoutLeftRightTopBottom = 1;    { English reading direction. }
	kPMLayoutLeftRightBottomTop = 2;
	kPMLayoutRightLeftTopBottom = 3;
	kPMLayoutRightLeftBottomTop = 4;    { Vertical-major directions: }
	kPMLayoutTopBottomLeftRight = 5;
	kPMLayoutTopBottomRightLeft = 6;
	kPMLayoutBottomTopLeftRight = 7;
	kPMLayoutBottomTopRightLeft = 8;

{ Page borders: }
type
	PMBorderType = UInt16;
const
	kPMBorderSingleHairline = 1;
	kPMBorderDoubleHairline = 2;
	kPMBorderSingleThickline = 3;
	kPMBorderDoubleThickline = 4;

{ 
 Options for which items to show inline in the print dialog
 This is only meant to be used in Carbon environment
 }
type
	PMPrintDialogOptionFlags = OptionBits;
const
	kPMHideInlineItems = 0 shl 0; { show nothing in the inline portion of print dialog }
	kPMShowDefaultInlineItems = 1 shl 15; { show the default set of items (copies & pages) in the inline portion of print dialog }
	kPMShowInlineCopies = 1 shl 0; { show Copies edit text, Collate check box and Two Sided check box (if printer supports it) in top portion of print dialog }
	kPMShowInlinePageRange = 1 shl 1; { show Paper Range buttons and From & To Page edit text items in top portion of print dialog }
	kPMShowInlinePageRangeWithSelection = 1 shl 6; { show Paper Range buttons with the addition of a Selection button and the From & To Page edit text items in top portion of print dialog }
	kPMShowInlinePaperSize = 1 shl 2; { show Paper Size popup menu in top portion of print dialog }
	kPMShowInlineOrientation = 1 shl 3; { show Orientation buttons in top portion of print dialog }
	kPMShowInlineScale = 1 shl 7; { show Scaling edit text in top portion of print dialog }
	kPMShowPageAttributesPDE = 1 shl 8; { add a PDE to the print dialog that contains the Page Setup information (paper size, orientation and scale) }

type
	PMPPDDomain = UInt16;
const
	kAllPPDDomains = 1;
	kSystemPPDDomain = 2;
	kLocalPPDDomain = 3;
	kNetworkPPDDomain = 4;
	kUserPPDDomain = 5;
	kCUPSPPDDomain = 6;


{ Description types }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMPPDDescriptionType CFSTRP('PMPPDDescriptionType')}
{$endc}
<<<<<<< HEAD
	{	 Document format strings 	}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ Document format strings }
>>>>>>> origin/fixes_2.4
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMDocumentFormatDefault CFSTRP('com.apple.documentformat.default')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMDocumentFormatPDF CFSTRP('application/pdf')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$definec kPMDocumentFormatPostScript CFSTRP('application/postscript')}
<<<<<<< HEAD
{$endc}
{ Graphic context strings }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMGraphicsContextDefault CFSTRP('com.apple.graphicscontext.default')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMGraphicsContextCoreGraphics CFSTRP('com.apple.graphicscontext.coregraphics')}
{$endc}
{ PDF Workflow Keys }
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPDFWorkFlowItemURLKey CFSTRP('itemURL')}
{$endc}
=======
{$endc}
{ Graphic context strings }
>>>>>>> origin/cpstrnew
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPDFWorkflowFolderURLKey CFSTRP('folderURL')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
<<<<<<< HEAD
{$definec kPDFWorkflowDisplayNameKey CFSTRP('displayName')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPDFWorkflowItemsKey CFSTRP('items')}
{$endc}
=======
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPDFWorkFlowItemURLKey CFSTRP('itemURL')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPDFWorkflowFolderURLKey CFSTRP('folderURL')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPDFWorkflowDisplayNameKey CFSTRP('displayName')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPDFWorkflowItemsKey CFSTRP('items')}
{$endc}
>>>>>>> graemeg/cpstrnew

{ OSStatus return codes }
const
	kPMNoError = noErr;
	kPMGeneralError = -30870;
	kPMOutOfScope = -30871; { an API call is out of scope }
	kPMInvalidParameter = paramErr; { a required parameter is missing or invalid }
	kPMNoDefaultPrinter = -30872; { no default printer selected }
	kPMNotImplemented = -30873; { this API call is not supported }
	kPMNoSuchEntry = -30874; { no such entry }
	kPMInvalidPrintSettings = -30875; { the printsettings reference is invalid }
	kPMInvalidPageFormat = -30876; { the pageformat reference is invalid }
	kPMValueOutOfRange = -30877; { a value passed in is out of range }
	kPMLockIgnored = -30878; { the lock value was ignored }

const
	kPMInvalidPrintSession = -30879; { the print session is invalid }
	kPMInvalidPrinter = -30880; { the printer reference is invalid }
	kPMObjectInUse = -30881; { the object is in use }
	kPMInvalidPreset = -30899;{ the preset is invalid }
=======
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPDFWorkFlowItemURLKey CFSTRP('itemURL')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPDFWorkflowFolderURLKey CFSTRP('folderURL')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPDFWorkflowDisplayNameKey CFSTRP('displayName')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPDFWorkflowItemsKey CFSTRP('items')}
{$endc}
>>>>>>> graemeg/cpstrnew
=======
{$definec kPMGraphicsContextCoreGraphics CFSTRP('com.apple.graphicscontext.coregraphics')}
{$endc}
{ PDF Workflow Keys }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPDFWorkFlowItemURLKey CFSTRP('itemURL')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPDFWorkflowFolderURLKey CFSTRP('folderURL')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPDFWorkflowDisplayNameKey CFSTRP('displayName')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPDFWorkflowItemsKey CFSTRP('items')}
{$endc}
>>>>>>> origin/cpstrnew

{ OSStatus return codes }
const
	kPMNoError = noErr;
	kPMGeneralError = -30870;
	kPMOutOfScope = -30871; { an API call is out of scope }
	kPMInvalidParameter = paramErr; { a required parameter is missing or invalid }
	kPMNoDefaultPrinter = -30872; { no default printer selected }
	kPMNotImplemented = -30873; { this API call is not supported }
	kPMNoSuchEntry = -30874; { no such entry }
	kPMInvalidPrintSettings = -30875; { the printsettings reference is invalid }
	kPMInvalidPageFormat = -30876; { the pageformat reference is invalid }
	kPMValueOutOfRange = -30877; { a value passed in is out of range }
	kPMLockIgnored = -30878; { the lock value was ignored }

const
	kPMInvalidPrintSession = -30879; { the print session is invalid }
	kPMInvalidPrinter = -30880; { the printer reference is invalid }
	kPMObjectInUse = -30881; { the object is in use }
	kPMInvalidPreset = -30899;{ the preset is invalid }


const
	kPMPrintAllPages = -1;

const
	kPMUnlocked = false;
<<<<<<< HEAD

const
	kPMPrintAllPages = -1;

const
	kPMUnlocked = false;
=======
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
{$definec kPMDocumentFormatPICT CFSTRP('application/vnd.apple.printing-pict')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMDocumentFormatPICTPS CFSTRP('application/vnd.apple.printing-pict-ps')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMDocumentFormatPICTPSwPSNormalizer CFSTRP('application/vnd.apple.printing-pict-ps-viapsnormalizer')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
=======
>>>>>>> origin/fixes_2.4
{$definec kPMDocumentFormatPostScript CFSTRP('application/postscript')}
{$endc}
{ Graphic context strings }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMGraphicsContextDefault CFSTRP('com.apple.graphicscontext.default')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMGraphicsContextCoreGraphics CFSTRP('com.apple.graphicscontext.coregraphics')}
{$endc}
{ PDF Workflow Keys }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPDFWorkFlowItemURLKey CFSTRP('itemURL')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPDFWorkflowFolderURLKey CFSTRP('folderURL')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPDFWorkflowDisplayNameKey CFSTRP('displayName')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPDFWorkflowItemsKey CFSTRP('items')}
{$endc}

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ OSStatus return codes }
const
	kPMNoError = noErr;
	kPMGeneralError = -30870;
	kPMOutOfScope = -30871; { an API call is out of scope }
	kPMInvalidParameter = paramErr; { a required parameter is missing or invalid }
	kPMNoDefaultPrinter = -30872; { no default printer selected }
	kPMNotImplemented = -30873; { this API call is not supported }
	kPMNoSuchEntry = -30874; { no such entry }
	kPMInvalidPrintSettings = -30875; { the printsettings reference is invalid }
	kPMInvalidPageFormat = -30876; { the pageformat reference is invalid }
	kPMValueOutOfRange = -30877; { a value passed in is out of range }
	kPMLockIgnored = -30878; { the lock value was ignored }

const
	kPMInvalidPrintSession = -30879; { the print session is invalid }
	kPMInvalidPrinter = -30880; { the printer reference is invalid }
	kPMObjectInUse = -30881; { the object is in use }
	kPMInvalidPreset = -30899;{ the preset is invalid }


const
	kPMPrintAllPages = -1;

const
	kPMUnlocked = false;
>>>>>>> origin/fixes_2.4

type
	PMRectPtr = ^PMRect;
	PMRect = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		top: Float64;
		left: Float64;
		bottom: Float64;
		right: Float64;
<<<<<<< HEAD
	end;
type
	PMResolutionPtr = ^PMResolution;
	PMResolution = record
		hRes: Float64;
		vRes: Float64;
	end;

	PMLanguageInfoPtr = ^PMLanguageInfo;
{$ifc TARGET_CPU_64}
	PMLanguageInfo = packed record
		level: Str32;
		version: Str32;
		release: Str32;
		pad: SInt8
	end;
{$elsec}
	PMLanguageInfo = record
		level: Str32;
		version: Str32;
		release: Str32;
		pad: SInt8
	end;
{$endc} {TARGET_CPU_64}

type
	PMPaperMargins = PMRect;

{
	PMDataFormat is used with PMPrintSettingsCreateWithDataRepresentation and 
	PMPageFormatCreateDataRepresentation to specify the format of the data representation created.
		
	kPMDataFormatXMLDefault specifies a data format that is compatible with all Mac OS X versions. Data in
	this format can be used with the PMUnflattenXXX routines present in all versions of Mac OS X prior to 10.5.
	However, this data representation is much larger than the more modern data representations described below.
	
	kPMDataFormatXMLMinimal is only compatible and usable with Mac OS X version 10.5 and later. 
	Data in this format can be only be reconsistuted into the equivalent printing manager object with 
	the appropriate PMXXXCreateWithDataRepresentation function. The data representation produced when
	using kPMDataFormatXMLMinimal is approximately 3-5 times smaller than kPMDataFormatXMLDefault. This
	format is a good choice when execution on versions of Mac OS X prior to 10.5 is not necessary and
	an uncompressed XML representation of the data is needed. 
	
	kPMDataFormatXMLCompressed is only compatible and usable with Mac OS X version 10.5 and later. 
	Data in this format can be only be reconsistuted into the equivalent printing manager object with the 
	appropriate PMXXXCreateWithDataRepresentation function. The data representation produced when
	using kPMDataFormatXMLCompressed is approximately 20 times smaller than kPMDataFormatXMLDefault.
	This format is a good choice when execution on versions of Mac OS X prior to 10.5 is not necessary and
	the minimum data size is important.
}
type
	PMDataFormat = SInt32;
const
	kPMDataFormatXMLDefault = 0;
	kPMDataFormatXMLMinimal = 1;
	kPMDataFormatXMLCompressed = 2;

{ PMPreset related }
{
	kPMPresetGraphicsTypeKey is a PMPreset attribute that specifies the graphics type of a given preset
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMPresetGraphicsTypeKey CFSTRP('com.apple.print.preset.graphicsType')}
{$endc}

{
	kPMPresetGraphicsTypePhoto is the graphics type of presets appropriate for printing photos.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMPresetGraphicsTypePhoto CFSTRP('Photo')}
{$endc}

{
	kPMPresetGraphicsTypeAll includes all graphics types.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMPresetGraphicsTypeAll CFSTRP('All')}
{$endc}
{
	kPMPresetGraphicsTypeGeneral is a graphics type that is not specific to any type of document printing.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMPresetGraphicsTypeGeneral CFSTRP('General')}
{$endc}
{
	kPMPresetGraphicsTypeNone excludes all graphics types.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMPresetGraphicsTypeNone CFSTRP('None')}
{$endc}

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
		top:					Double;
		left:					Double;
		bottom:					Double;
		right:					Double;
=======
>>>>>>> origin/fixes_2.4
	end;
type
	PMResolutionPtr = ^PMResolution;
	PMResolution = record
		hRes: Float64;
		vRes: Float64;
	end;

	PMLanguageInfoPtr = ^PMLanguageInfo;
{$ifc TARGET_CPU_64}
	PMLanguageInfo = packed record
		level: Str32;
		version: Str32;
		release: Str32;
		pad: SInt8
	end;
{$elsec}
	PMLanguageInfo = record
		level: Str32;
		version: Str32;
		release: Str32;
		pad: SInt8
	end;
{$endc} {TARGET_CPU_64}

type
	PMPaperMargins = PMRect;

{
	PMDataFormat is used with PMPrintSettingsCreateWithDataRepresentation and 
	PMPageFormatCreateDataRepresentation to specify the format of the data representation created.
		
	kPMDataFormatXMLDefault specifies a data format that is compatible with all Mac OS X versions. Data in
	this format can be used with the PMUnflattenXXX routines present in all versions of Mac OS X prior to 10.5.
	However, this data representation is much larger than the more modern data representations described below.
	
	kPMDataFormatXMLMinimal is only compatible and usable with Mac OS X version 10.5 and later. 
	Data in this format can be only be reconsistuted into the equivalent printing manager object with 
	the appropriate PMXXXCreateWithDataRepresentation function. The data representation produced when
	using kPMDataFormatXMLMinimal is approximately 3-5 times smaller than kPMDataFormatXMLDefault. This
	format is a good choice when execution on versions of Mac OS X prior to 10.5 is not necessary and
	an uncompressed XML representation of the data is needed. 
	
	kPMDataFormatXMLCompressed is only compatible and usable with Mac OS X version 10.5 and later. 
	Data in this format can be only be reconsistuted into the equivalent printing manager object with the 
	appropriate PMXXXCreateWithDataRepresentation function. The data representation produced when
	using kPMDataFormatXMLCompressed is approximately 20 times smaller than kPMDataFormatXMLDefault.
	This format is a good choice when execution on versions of Mac OS X prior to 10.5 is not necessary and
	the minimum data size is important.
}
type
	PMDataFormat = SInt32;
const
	kPMDataFormatXMLDefault = 0;
	kPMDataFormatXMLMinimal = 1;
	kPMDataFormatXMLCompressed = 2;

{ PMPreset related }
{
	kPMPresetGraphicsTypeKey is a PMPreset attribute that specifies the graphics type of a given preset
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMPresetGraphicsTypeKey CFSTRP('com.apple.print.preset.graphicsType')}
{$endc}

{
	kPMPresetGraphicsTypePhoto is the graphics type of presets appropriate for printing photos.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMPresetGraphicsTypePhoto CFSTRP('Photo')}
{$endc}

{
	kPMPresetGraphicsTypeAll includes all graphics types.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMPresetGraphicsTypeAll CFSTRP('All')}
{$endc}
{
	kPMPresetGraphicsTypeGeneral is a graphics type that is not specific to any type of document printing.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMPresetGraphicsTypeGeneral CFSTRP('General')}
{$endc}
{
	kPMPresetGraphicsTypeNone excludes all graphics types.
}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMPresetGraphicsTypeNone CFSTRP('None')}
{$endc}

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
