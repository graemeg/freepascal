{
     File:       PMPrintSettingsKeys.h
 
     Contains:   Mac OS X Printing Manager Print Settings Keys.
 
     Version:    Technology: Mac OS X
                 Release:    1.0
 
     Copyright  (c) 2008 by Apple Inc. All Rights Reserved.
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}
{  Pascal Translation:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
                     http://www.freepascal.org/bugs.html
 
}
{    Pascal Translation:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
<<<<<<< HEAD
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
>>>>>>> origin/fixes_2.4
{
    Modified for use with Free Pascal
    Version 308
    Please report any bugs to <gpc@microbizz.nl>
}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
{$mode macpas}
{$packenum 1}
{$macro on}
{$inline on}
{$calling mwpascal}

unit PMPrintSettingsKeys;
interface
{$setc UNIVERSAL_INTERFACES_VERSION := $0400}
{$setc GAP_INTERFACES_VERSION := $0308}

{$ifc not defined USE_CFSTR_CONSTANT_MACROS}
    {$setc USE_CFSTR_CONSTANT_MACROS := TRUE}
{$endc}

{$ifc defined CPUPOWERPC and defined CPUI386}
	{$error Conflicting initial definitions for CPUPOWERPC and CPUI386}
{$endc}
{$ifc defined FPC_BIG_ENDIAN and defined FPC_LITTLE_ENDIAN}
	{$error Conflicting initial definitions for FPC_BIG_ENDIAN and FPC_LITTLE_ENDIAN}
{$endc}

{$ifc not defined __ppc__ and defined CPUPOWERPC32}
	{$setc __ppc__ := 1}
{$elsec}
	{$setc __ppc__ := 0}
{$endc}
{$ifc not defined __ppc64__ and defined CPUPOWERPC64}
	{$setc __ppc64__ := 1}
{$elsec}
	{$setc __ppc64__ := 0}
{$endc}
{$ifc not defined __i386__ and defined CPUI386}
	{$setc __i386__ := 1}
{$elsec}
	{$setc __i386__ := 0}
{$endc}
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


{$ifc defined __ppc__ and __ppc__ and defined __i386__ and __i386__}
	{$error Conflicting definitions for __ppc__ and __i386__}
{$endc}

{$ifc defined __ppc__ and __ppc__}
	{$setc TARGET_CPU_PPC := TRUE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
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
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/fixes_2.4
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
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
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/fixes_2.4
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
=======
>>>>>>> origin/fixes_2.4
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
=======
>>>>>>> origin/fixes_2.4
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
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/fixes_2.4
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
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
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
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
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
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> origin/cpstrnew
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> origin/fixes_2.4
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
  {$setc TARGET_CPU_64 := FALSE}
{$endc}

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
{$setc TARGET_OS_UNIX := FALSE}
{$setc TARGET_OS_WIN32 := FALSE}
{$setc TARGET_RT_MAC_68881 := FALSE}
{$setc TARGET_RT_MAC_CFM := FALSE}
{$setc TARGET_RT_MAC_MACHO := TRUE}
{$setc TYPED_FUNCTION_POINTERS := TRUE}
{$setc TYPE_BOOL := FALSE}
{$setc TYPE_EXTENDED := FALSE}
{$setc TYPE_LONGLONG := TRUE}
uses MacTypes;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}


const
	kPMCopiesStr = 'com.apple.print.PrintSettings.PMCopies';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMCopiesKey CFSTRP('com.apple.print.PrintSettings.PMCopies')}
{$endc}                       { CFNumber, kCFNumberSInt32Type, number of copies to print. }

const
	kPMCopyCollateStr = 'com.apple.print.PrintSettings.PMCopyCollate';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMCopyCollateKey CFSTRP('com.apple.print.PrintSettings.PMCopyCollate')}
{$endc}                  { CFBoolean, Turns on collating }

const
	kPMOutputOrderStr = 'OutputOrder';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMOutputOrderKey CFSTRP('OutputOrder')}
{$endc}			{ CFString, Reverse or Normal. default is Printer Specific }

const
	kPMPageSetStr = 'page-set';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMPageSetKey CFSTRP('page-set')}
{$endc}	{ CFString, even, odd, or all. default is all }

const
	kPMMirrorStr = 'mirror';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMMirrorKey CFSTRP('mirror')}
{$endc}		{ CFString, true or false. default is false }


const
	kPMPrintSelectionOnlyStr = 'com.apple.print.PrintSettings.PMPrintSelectionOnly';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMPrintSelectionOnlyKey CFSTRP('com.apple.print.PrintSettings.PMPrintSelectionOnly')}
{$endc}            { CFBoolean - True if only current selection should be printed. }

const
	kPMBorderStr = 'com.apple.print.PrintSettings.PMBorder';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMBorderKey CFSTRP('com.apple.print.PrintSettings.PMBorder')}
{$endc}                       { CFBoolean - If true, we do borders. }

const
	kPMBorderTypeStr = 'com.apple.print.PrintSettings.PMBorderType';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMBorderTypeKey CFSTRP('com.apple.print.PrintSettings.PMBorderType')}
{$endc}                   { CFNumber - kCFNumberSInt32Type, Enum (PMBorderType) }

const
	kPMLayoutNUpStr = 'com.apple.print.PrintSettings.PMLayoutNUp';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMLayoutNUpKey CFSTRP('com.apple.print.PrintSettings.PMLayoutNUp')}
{$endc}                    { CFBoolean, Turns on N-Up layout. }

const
	kPMLayoutRowsStr = 'com.apple.print.PrintSettings.PMLayoutRows';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMLayoutRowsKey CFSTRP('com.apple.print.PrintSettings.PMLayoutRows')}
{$endc}                   { CFNumber - kCFNumberSInt32Type, indicates number of layout rows. }

const
	kPMLayoutColumnsStr = 'com.apple.print.PrintSettings.PMLayoutColumns';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMLayoutColumnsKey CFSTRP('com.apple.print.PrintSettings.PMLayoutColumns')}
{$endc}                { CFNumber - kCFNumberSInt32Type, indicates number of layout columns. }

const
	kPMLayoutDirectionStr = 'com.apple.print.PrintSettings.PMLayoutDirection';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMLayoutDirectionKey CFSTRP('com.apple.print.PrintSettings.PMLayoutDirection')}
{$endc}              { CFNumber - kCFNumberSInt32Type, Enum (PMLayoutDirection) }

const
	kPMLayoutTileOrientationStr = 'com.apple.print.PrintSettings.PMLayoutTileOrientation';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMLayoutTileOrientationKey CFSTRP('com.apple.print.PrintSettings.PMLayoutTileOrientation')}
{$endc}        { CFNumber - kCFNumberSInt32Type, PMOrientation, 1 = portrait, 2 = landscape, etc. }

const
	kPMJobStateStr = 'com.apple.print.PrintSettings.PMJobState';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMJobStateKey CFSTRP('com.apple.print.PrintSettings.PMJobState')}
{$endc}                     { CFNumber - kCFNumberSInt32Type, Enum, active = 0, pending, hold until, hold indefinitely, aborted, finished }

const
	kPMJobHoldUntilTimeStr = 'com.apple.print.PrintSettings.PMJobHoldUntilTime';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMJobHoldUntilTimeKey CFSTRP('com.apple.print.PrintSettings.PMJobHoldUntilTime')}
{$endc}             { CFDate - Time we expect to print the job. }

const
	kPMJobPriorityStr = 'com.apple.print.PrintSettings.PMJobPriority';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMJobPriorityKey CFSTRP('com.apple.print.PrintSettings.PMJobPriority')}
{$endc}                  { CFNumber - kCFNumberSInt32Type, Enum, Low = 0, normal, urgent }

const
	kPMDuplexingStr = 'com.apple.print.PrintSettings.PMDuplexing';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMDuplexingKey CFSTRP('com.apple.print.PrintSettings.PMDuplexing')}
{$endc}                    { CFNumber - kCFNumberSInt32Type, Enum, kPMDuplexNone,  kPMDuplexNoTumble, kPMDuplexTumble, kPMSimplexTumble }

const
	kPMColorSyncProfileIDStr = 'com.apple.print.PrintSettings.PMColorSyncProfileID';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMColorSyncProfileIDKey CFSTRP('com.apple.print.PrintSettings.PMColorSyncProfileID')}
{$endc}           { CFNumber - kCFNumberSInt32Type, ID of profile to use. }

const
	kPMPrimaryPaperFeedStr = 'com.apple.print.PrintSettings.PMPrimaryPaperFeed';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMPrimaryPaperFeedKey CFSTRP('com.apple.print.PrintSettings.PMPrimaryPaperFeed')}
{$endc}				{ CFArray - main & option PPD key for input paper feed }

const
	kPMSecondaryPaperFeedStr = 'com.apple.print.PrintSettings.PMSecondaryPaperFeed';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMSecondaryPaperFeedKey CFSTRP('com.apple.print.PrintSettings.PMSecondaryPaperFeed')}
{$endc}			{ CFArray - main & option PPD key for input paper feed }

const
	kPMPSErrorHandlerStr = 'com.apple.print.PrintSettings.PMPSErrorHandler';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMPSErrorHandlerKey CFSTRP('com.apple.print.PrintSettings.PMPSErrorHandler')}
{$endc}				{ CFNumber - kCFNumberSInt32Type  }

const
	kPMPSTraySwitchStr = 'com.apple.print.PrintSettings.PMPSTraySwitch';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMPSTraySwitchKey CFSTRP('com.apple.print.PrintSettings.PMPSTraySwitch')}
{$endc}					{ CFArray - main & option PPD key for tray switching }

const
	kPMTotalBeginPagesStr = 'com.apple.print.PrintSettings.PMTotalBeginPages';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMTotalBeginPagesKey CFSTRP('com.apple.print.PrintSettings.PMTotalBeginPages')}
{$endc}			{ CFNumber the total number of times beginpage was called }

const
	kPMTotalSidesImagedStr = 'com.apple.print.PrintSettings.PMTotalSidesImaged';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMTotalSidesImagedKey CFSTRP('com.apple.print.PrintSettings.PMTotalSidesImaged')}
{$endc}			{ CFNumber the total number of sides that will printed. Does not take into account duplex and collation }


{ Fax Related }
const
	kPMFaxNumberStr = 'phone';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMFaxNumberKey CFSTRP('phone')}
{$endc}			{ CFString - fax number to dial }

const
	kPMFaxToStr = 'faxTo';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMFaxToKey CFSTRP('faxTo')}
{$endc}				{ CFString - entire fax to line }

const
	kPMFaxPrefixStr = 'faxPrefix';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMFaxPrefixKey CFSTRP('faxPrefix')}
{$endc}			{ CFString - fax prefix to dial }

const
	kPMFaxSubjectStr = 'faxSubject';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMFaxSubjectKey CFSTRP('faxSubject')}
{$endc}			{ CFString - fax subject linee}

const
	kPMFaxCoverSheetStr = 'faxCoverSheet';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMFaxCoverSheetKey CFSTRP('faxCoverSheet')}
{$endc}		{ CFString - fax cover sheet }

const
	kPMFaxCoverSheetMessageStr = 'faxCoverSheetMessage';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMFaxCoverSheetMessageKey CFSTRP('faxCoverSheetMessage')}
{$endc}	{ CFString - fax cover sheet message}

const
	kPMFaxToneDialingStr = 'faxToneDialing';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMFaxToneDialingKey CFSTRP('faxToneDialing')}
{$endc}		{ CFString - fax use tone dialing }

const
	kPMFaxUseSoundStr = 'faxUseSound';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMFaxUseSoundKey CFSTRP('faxUseSound')}
{$endc}			{ CFString - fax use sound }

const
	kPMFaxWaitForDialToneStr = 'faxWaitForDialTone';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMFaxWaitForDialToneKey CFSTRP('faxWaitForDialTone')}
{$endc}	{ CFString - fax wait for dial tone }

const
	kPMFaxToLabelStr = 'faxToLabel';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMFaxToLabelKey CFSTRP('faxToLabel')}
{$endc}			{ CFString - To: label }

const
	kPMFaxFromLabelStr = 'faxFromLabel';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMFaxFromLabelKey CFSTRP('faxFromLabel')}
{$endc}			{ CFString - From: label }

const
	kPMFaxDateLabelStr = 'faxDateLabel';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMFaxDateLabelKey CFSTRP('faxDateLabel')}
{$endc}			{ CFString - Date: label }

const
	kPMFaxSubjectLabelStr = 'faxSubjectLabel';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMFaxSubjectLabelKey CFSTRP('faxSubjectLabel')}
{$endc}		{ CFString - Subject: label }

const
	kPMFaxSheetsLabelStr = 'faxSheetsLabel';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMFaxSheetsLabelKey CFSTRP('faxSheetsLabel')}
{$endc}		{ CFString - Sheets to Follow: label }


{ Coverpage Related }
const
	kPMCoverPageStr = 'com.apple.print.PrintSettings.PMCoverPage';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMCoverPageKey CFSTRP('com.apple.print.PrintSettings.PMCoverPage')}
{$endc}                    { CFNumber - kCFNumberSInt32Type, Enum, kPMCoverPageNone,  kPMCoverPageBefore, kPMCoverPageAfter }

{ The values for kPMCoverPageKey }

const
	kPMCoverPageNone = 1;	
	// Print a cover page before printing the document.
	kPMCoverPageBefore = 2;
	// Print a cover page after printing the document.
	kPMCoverPageAfter = 3;
{ If the kPMDuplexingKey is not in a print settings then kPMDuplexDefault should be assumed.
 }
	kPMCoverPageDefault	= kPMCoverPageNone;


const
	kPMCoverPageSourceStr = 'com.apple.print.PrintSettings.PMCoverPageSource';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMCoverPageSourceKey CFSTRP('com.apple.print.PrintSettings.PMCoverPageSource')}
{$endc}				{ CFArray - main & option PPD key for cover page paper source }


const
	kPMDestinationPrinterIDStr = 'DestinationPrinterID';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMDestinationPrinterIDKey CFSTRP('DestinationPrinterID')}
{$endc}	{ CFStringRef - the printer ID corresponding to the destination printer }

const
	kPMInlineWorkflowStr = 'inlineWorkflow';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMInlineWorkflowKey CFSTRP('inlineWorkflow')}
{$endc}	{ CFStringRef - the URL for the inline workflow item that will process this job }

const
	kPMPageToPaperMappingTypeStr = 'com.apple.print.PageToPaperMappingType';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMPageToPaperMappingTypeKey CFSTRP('com.apple.print.PageToPaperMappingType')}
{$endc} { a CFNumber - values from PMPageToPaperMappingType }

const
	kPMPageToPaperMediaNameStr = 'com.apple.print.PageToPaperMappingMediaName';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMPageToPaperMediaNameKey CFSTRP('com.apple.print.PageToPaperMappingMediaName')}
{$endc} { a CFString - the untranslated media name for the destination sheet }

const
	kPMPageToPaperMappingAllowScalingUpStr = 'com.apple.print.PageToPaperMappingAllowScalingUp';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMPageToPaperMappingAllowScalingUpKey CFSTRP('com.apple.print.PageToPaperMappingAllowScalingUp')}
{$endc} { a CFBoolean - if true, allow scaling up to fit
												    destination sheet, otherwise do not scale
												    up if destination sheet is larger than formatting
												    sheet. Default value: false. }

{
    The kPMCustomProfilePathKey key stores a CFString that corresponds to a custom profile setting for a given printer.
}
const
	kPMCustomProfilePathStr = 'PMCustomProfilePath';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMCustomProfilePathKey CFSTRP('PMCustomProfilePath')}
{$endc}

{ Page to Paper Mapping Types }
const
	kPMPageToPaperMappingNone = 1;
	kPMPageToPaperMappingScaleToFit = 2;
type
	PMPageToPaperMappingType = SInt32;


{ Possible values for the kPMColorMatchingModeKey}
const
	kPMVendorColorMatchingStr = 'AP_VendorColorMatching';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMVendorColorMatching CFSTRP('AP_VendorColorMatching')}
{$endc}
const
	kPMApplicationColorMatchingStr = 'AP_ApplicationColorMatching';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMApplicationColorMatching CFSTRP('AP_ApplicationColorMatching')}
{$endc}

const
	kPMColorMatchingModeStr = 'AP_ColorMatchingMode';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMColorMatchingModeKey CFSTRP('AP_ColorMatchingMode')}
{$endc}   { Value is CFStringRef - one of kPMColorSyncMatching (deprecated), 
										kPMVendorColorMatching, kPMApplicationColorMatching }


{ Begin: Use of these keys is discouraged. Use PMSessionSetDestination, PMSessionGetDestinationType, PMSessionCopyDestinationFormat, and PMSessionCopyDestinationLocation instead }
const
	kPMDestinationTypeStr = 'com.apple.print.PrintSettings.PMDestinationType';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMDestinationTypeKey CFSTRP('com.apple.print.PrintSettings.PMDestinationType')}
{$endc}              { CFNumber, kCFNumberSInt32Type kPMDestinationPrinter kPMDestinationFile kPMDestinationFax, etc. }
const
	kPMOutputFilenameStr = 'com.apple.print.PrintSettings.PMOutputFilename';
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kPMOutputFilenameKey CFSTRP('com.apple.print.PrintSettings.PMOutputFilename')}
{$endc}               { CFString - URL for the output filename. }
{ End: Use of these keys is discouraged. Use PMSessionSetDestination, PMSessionGetDestinationType, PMSessionCopyDestinationFormat, and PMSessionCopyDestinationLocation instead }

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
