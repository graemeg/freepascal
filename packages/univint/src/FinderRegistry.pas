{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       OpenScripting/FinderRegistry.h
 
     Contains:   Data types for Finder AppleEvents
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Version:    OSA-148~28
=======
     Version:    OSA-136~14
>>>>>>> graemeg/cpstrnew
=======
     Version:    OSA-136~14
>>>>>>> graemeg/cpstrnew
=======
     Version:    OSA-136~14
>>>>>>> graemeg/cpstrnew
=======
     Version:    OSA-136~14
>>>>>>> origin/cpstrnew
 
     Copyright:  © 1991-2008 by Apple Computer, Inc., all rights reserved.
=======
=======
>>>>>>> origin/fixes_2_2
     File:       FinderRegistry.p
=======
     File:       OpenScripting/FinderRegistry.h
>>>>>>> origin/fixes_2.4
 
     Contains:   Data types for Finder AppleEvents
 
     Version:    OSA-136~14
 
<<<<<<< HEAD
     Copyright:  © 1985-2002 by Apple Computer, Inc., all rights reserved.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  © 1991-2008 by Apple Computer, Inc., all rights reserved.
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
{  Pascal Translation Updated: Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
{ Pascal Translation Updated: Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }

>>>>>>> graemeg/cpstrnew
=======
{ Pascal Translation Updated: Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }

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

{ Pascal Translation Updated: Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }

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

unit FinderRegistry;
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
=======
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
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
{$elsec}
=======
{$elsec}
>>>>>>> graemeg/cpstrnew
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
<<<<<<< HEAD
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
>>>>>>> origin/cpstrnew
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
uses MacTypes,AERegistry,AEDataModel,OSA;
{$endc} {not MACOSALLINCLUDE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}

<<<<<<< HEAD
{$ifc TARGET_OS_MAC}

=======
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> graemeg/cpstrnew


{$ifc TARGET_OS_MAC}

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======


{$ifc TARGET_OS_MAC}

>>>>>>> origin/cpstrnew
{$ALIGN POWER}
=======
=======
>>>>>>> origin/fixes_2_2
uses MacTypes,AERegistry,OSA;
=======
uses MacTypes,AERegistry,AEDataModel,OSA;
{$endc} {not MACOSALLINCLUDE}

>>>>>>> origin/fixes_2.4

{$ifc TARGET_OS_MAC}

<<<<<<< HEAD
{$ALIGN MAC68K}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$ALIGN POWER}
>>>>>>> origin/fixes_2.4


{
  //////////////////////////////////////
   Finder Suite
  //////////////////////////////////////
}

{
   The old Finder Event suite was 'FNDR'
   The new suite is 'fndr'
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
const
	kAEFinderSuite = FourCharCode('fndr');

{
  //////////////////////////////////////
   Finder Events
  //////////////////////////////////////
}
const
	kAECleanUp = FourCharCode('fclu');
	kAEEject = FourCharCode('ejct');
	kAEEmpty = FourCharCode('empt');
	kAEErase = FourCharCode('fera');
	kAEGestalt = FourCharCode('gstl');
	kAEPutAway = FourCharCode('ptwy');
	kAERebuildDesktopDB = FourCharCode('rddb');
	kAESync = FourCharCode('fupd');
	kAEInterceptOpen = FourCharCode('fopn');

{ "Sort" from the database suite:}
const
	kAEDatabaseSuite = FourCharCode('DATA');
	kAESort = FourCharCode('SORT');

{
  ////////////////////////////////////////////////////////////////////////
   Classes
   Note: all classes are defined up front so that the property definitions
   can reference classes.
  ////////////////////////////////////////////////////////////////////////
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

const
	cInternalFinderObject = FourCharCode('obj '); { cReference - used to distinguish objects used inside the Finder only}

{
   Main Finder class definitions
   Indentation implies object model hierarchy
}
const
{ We do not use class cItem from AERegistry.r. Instead our class Item is a cObject}
                                        {         cItem                        = 'citm',   // defined in AERegistry.r}
                                        {          cFile                    = 'file',  // defined in AERegistry.r}
	cAliasFile = FourCharCode('alia');
	cApplicationFile = FourCharCode('appf');
	cControlPanelFile = FourCharCode('ccdv');
	cDeskAccessoryFile = FourCharCode('dafi');
	cDocumentFile = FourCharCode('docf');
	cFontFile = FourCharCode('fntf');
	cSoundFile = FourCharCode('sndf');
	cClippingFile = FourCharCode('clpf');
	cContainer = FourCharCode('ctnr');
	cDesktop = FourCharCode('cdsk');
	cSharableContainer = FourCharCode('sctr');
	cDisk = FourCharCode('cdis');
	cFolder = FourCharCode('cfol');
	cSuitcase = FourCharCode('stcs');
	cAccessorySuitcase = FourCharCode('dsut');
	cFontSuitcase = FourCharCode('fsut');
	cTrash = FourCharCode('ctrs');
	cDesktopPrinter = FourCharCode('dskp');
	cPackage = FourCharCode('pack');
	cContentSpace = FourCharCode('dwnd'); {          cWindow                    = 'cwin',       // defined in AERegistry.r}
	cContainerWindow = FourCharCode('cwnd');
	cInfoWindow = FourCharCode('iwnd');
	cSharingWindow = FourCharCode('swnd');
	cStatusWindow = FourCharCode('qwnd');
	cClippingWindow = FourCharCode('lwnd');
	cPreferencesWindow = FourCharCode('pwnd');
	cDTPWindow = FourCharCode('dtpw');
	cProcess = FourCharCode('prcs');
	cAccessoryProcess = FourCharCode('pcda');
	cApplicationProcess = FourCharCode('pcap');
	cGroup = FourCharCode('sgrp');
	cUser = FourCharCode('cuse'); {         cApplication                  = 'capp',     // defined in AERegistry.r}
	cSharingPrivileges = FourCharCode('priv');
	cPreferences = FourCharCode('cprf');
	cLabel = FourCharCode('clbl');
	cSound = FourCharCode('snd ');
	cAliasList = FourCharCode('alst');
	cSpecialFolders = FourCharCode('spfl'); { For use by viewer search engines:}
	cOnlineDisk = FourCharCode('cods');
	cOnlineLocalDisk = FourCharCode('clds');
	cOnlineRemoteDisk = FourCharCode('crds'); { Miscellaneous class definitions}
	cEntireContents = FourCharCode('ects');
	cIconFamily = FourCharCode('ifam');

=======
>>>>>>> origin/cpstrnew

{
  //////////////////////////////////////
   Properties
  //////////////////////////////////////
}

{ Properties of class cItem (really cObject)}
const
<<<<<<< HEAD
{    pBounds                        = 'pbnd',       // defined in AERegistry.r}
	pComment = FourCharCode('comt');
	pContainer = cContainer;
	pContentSpace = cContentSpace;
	pCreationDateOld = FourCharCode('crtd'); { to support pre-Finder 8 scripts}
	pCreationDate = FourCharCode('ascd'); { from File Commands OSAX}
	pDescription = FourCharCode('dscr');
	pDisk = cDisk;
	pFolderOld = cFolder; { to support pre-Finder 8 scripts}
	pFolder = FourCharCode('asdr'); { from File Commands OSAX}
	pIconBitmap = FourCharCode('iimg'); {    pID                           = 'ID  ',        // defined in AERegistry.r}
	pInfoWindow = cInfoWindow;
	pKind = FourCharCode('kind');
	pLabelIndex = FourCharCode('labi');
	pModificationDateOld = FourCharCode('modd'); { to support pre-Finder 8 scripts}
	pModificationDate = FourCharCode('asmo'); { from File Commands OSAX}
                                        {    pName                      = 'pnam',         // defined in AERegistry.r}
	pPhysicalSize = FourCharCode('phys');
	pPosition = FourCharCode('posn');
	pIsSelected = FourCharCode('issl');
	pSize = pPointSize; { pPointSize defined in AERegistry.r}
	pWindow = cWindow;
	pPreferencesWindow = cPreferencesWindow;


{ Properties of class cFile (subclass of cItem)}
const
	pFileCreator = FourCharCode('fcrt');
	pFileType = FourCharCode('asty'); { from File Commands OSAX}
	pFileTypeOld = FourCharCode('fitp'); { to support pre-Finder 8 scripts}
	pIsLocked = FourCharCode('aslk'); { from File Commands OSAX}
	pIsLockedOld = FourCharCode('islk'); { to support pre-Finder 8 scripts}
                                        {    pIsStationeryPad               = 'pspd',         // defined in AERegistry.r                }
                                        {    pVersion                    = 'vers',       // defined in AERegistry.r}
	pProductVersion = FourCharCode('ver2');


{ Properties of class cAliasFile (subclass of cFile)}
const
	pOriginalItem = FourCharCode('orig');

{ Properties of class cApplicationFile (subclass of cFile)}
const
	pMinAppPartition = FourCharCode('mprt');
	pAppPartition = FourCharCode('appt');
	pSuggestedAppPartition = FourCharCode('sprt');
	pIsScriptable = FourCharCode('isab');

{ Properties of class cURLFile (subclass of cFile)}
const
	pInternetLocation = FourCharCode('iloc');

{ Properties of class cSoundFile (subclass of cFile)}
const
	pSound = FourCharCode('snd ');


{
   Properties of class cControlPanel (Views CP only) (subclass of cFile)
   Note: the other view-like preference settings are not available in the Views
   control panel. These properties are only offered here for backward compatability.
   To set the full range of Finder Preferences, use the Preferences object.
}
const
	pShowFolderSize = FourCharCode('sfsz'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pShowComment = FourCharCode('scom'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pShowDate = FourCharCode('sdat'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pShowCreationDate = FourCharCode('scda'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pShowKind = FourCharCode('sknd'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pShowLabel = FourCharCode('slbl'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pShowSize = FourCharCode('ssiz'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pShowVersion = FourCharCode('svrs'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pSortDirection = FourCharCode('sord');
	pShowDiskInfo = FourCharCode('sdin'); { Always on in Finder 8.0 HIS}
	pListViewIconSize = FourCharCode('lvis'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pGridIcons = FourCharCode('fgrd'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pStaggerIcons = FourCharCode('fstg'); { No longer part of the Finder 8.0 HIS}
	pViewFont = FourCharCode('vfnt');
	pViewFontSize = FourCharCode('vfsz');

{ Properties of class cContainer (subclass of cItem)}
const
	pCompletelyExpanded = FourCharCode('pexc');
	pContainerWindow = cContainerWindow;
	pEntireContents = cEntireContents;
	pExpandable = FourCharCode('pexa');
	pExpanded = FourCharCode('pexp');
	pPreviousView = FourCharCode('svew'); {    pSelection                    = 'sele',       // defined in AERegistry.r}
	pView = FourCharCode('pvew');
	pIconSize = pListViewIconSize; { defined above}
	pKeepArranged = FourCharCode('arrg'); { OBSOLETE in Finder 9 or later}
	pKeepArrangedBy = FourCharCode('arby'); { OBSOLETE in Finder 9 or later}

{ Properties of class cDesktop (subclass of cContainer)}
const
	pStartupDisk = FourCharCode('sdsk');
	pTrash = FourCharCode('trsh');

{ Properties of class cSharableContainer (subclass of cContainer)}
const
	pOwner = FourCharCode('sown');
	pOwnerPrivileges = FourCharCode('ownr');
	pGroup = cGroup;
	pGroupPrivileges = FourCharCode('gppr');
	pGuestPrivileges = FourCharCode('gstp');
	pArePrivilegesInherited = FourCharCode('iprv');
	pExported = FourCharCode('sexp');
	pMounted = FourCharCode('smou');
	pSharingProtection = FourCharCode('spro');
	pSharing = FourCharCode('shar');
	pSharingWindow = cSharingWindow;

{ Properties of class cDisk (subclass of cSharableContainer)}
const
	pCapacity = FourCharCode('capa');
	pEjectable = FourCharCode('isej');
	pFreeSpace = FourCharCode('frsp');
	pLocal = FourCharCode('isrv');
	pIsStartup = FourCharCode('istd');

{ Properties of class cTrash (subclass of cSharableContainer)}
const
	pWarnOnEmpty = FourCharCode('warn');

{ Properties of class cWindow (subclass of cContentSpace)}
const
{    pBounds                        = 'pbnd',   // defined in AERegistry.r}
                                        {    pHasCloseBox                = 'hclb',     // defined in AERegistry.r}
                                        {    pIsFloating                    = 'isfl',     // defined in AERegistry.r}
                                        {    pIndex                     = 'pidx',     // defined in AERegistry.r}
                                        {    pIsModal                    = 'pmod',   // defined in AERegistry.r}
                                        {    pPosition                    = 'posn',     // defined above}
                                        {    pIsResizable                = 'prsz',     // defined in AERegistry.r}
                                        {    pHasTitleBar                = 'ptit',     // defined in AERegistry.r}
                                        {    pVisible                    = 'pvis',   // defined in AERegistry.r}
                                        {    pIsZoomable                    = 'iszm',     // defined in AERegistry.r}
                                        {    pIsZoomed                    = 'pzum',     // defined in AERegistry.r}
	pIsZoomedFull = FourCharCode('zumf');
	pIsPopup = FourCharCode('drwr');
	pIsPulledOpen = FourCharCode('pull'); { only applies to popup windows}
	pIsCollapsed = FourCharCode('wshd'); { only applies to normal windows}

{ Properties of class cContainerWindow (subclass of cWindow)}
const
	pObject = cObject;

{ Properties of class cSharingWindow (subclass of cWindow)}
const
	pSharableContainer = cSharableContainer;

{ Properties of class cInfoWindow (subclass of cWindow)}
const
	pInfoPanel = FourCharCode('panl');


{ Properties of networking support}
const
	pFileShareOn = FourCharCode('fshr');
	pFileShareStartingUp = FourCharCode('fsup');
	pProgramLinkingOn = FourCharCode('iac ');

{ Properties of class cPreferencesWindow (subclass of cWindow)}
const
{    pShowFolderSize                   = 'sfsz',         // defined above for Views CP}
                                        {    pShowComment                = 'scom',      // defined above for Views CP}
	pShowModificationDate = pShowDate; { pShowDate defined above for Views CP}
                                        {    pShowKind                    = 'sknd',        // defined above for Views CP}
                                        {    pShowLabel                    = 'slbl',         // defined above for Views CP}
                                        {    pShowSize                    = 'ssiz',        // defined above for Views CP}
                                        {    pShowVersion                = 'svrs',      // defined above for Views CP}
                                        {    pShowCreationDate             = 'scda',      // Removed from Finder 8.0 HIS}
                                        {    pShowFileType                 = 'sfty',       // Removed from Finder 8.0 HIS}
                                        {    pShowFileCreator               = 'sfcr',         // Removed from Finder 8.0 HIS}
                                        {    pListViewIconSize             = 'lvis',      // defined above for Views CP}
                                        {    pGridIcons                    = 'fgrd',         // defined above for Views CP}
                                        {    pStaggerIcons                 = 'fstg',       // defined above for Views CP}
                                        {    pViewFont                    = 'vfnt',        // defined above for Views CP}
                                        {    pViewFontSize                 = 'vfsz',       // defined above for Views CP}
	pUseRelativeDate = FourCharCode('urdt'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pDelayBeforeSpringing = FourCharCode('dela');
	pSpringOpenFolders = FourCharCode('sprg');
	pUseShortMenus = FourCharCode('usme');
	pUseWideGrid = FourCharCode('uswg');
	pLabel1 = FourCharCode('lbl1');
	pLabel2 = FourCharCode('lbl2');
	pLabel3 = FourCharCode('lbl3');
	pLabel4 = FourCharCode('lbl4');
	pLabel5 = FourCharCode('lbl5');
	pLabel6 = FourCharCode('lbl6');
	pLabel7 = FourCharCode('lbl7');
	pDefaultIconViewIconSize = FourCharCode('iisz');
	pDefaultButtonViewIconSize = FourCharCode('bisz');
	pDefaultListViewIconSize = FourCharCode('lisz'); { old use of this name is now pIconSize}
	pIconViewArrangement = FourCharCode('iarr');
	pButtonViewArrangement = FourCharCode('barr');
=======
>>>>>>> graemeg/cpstrnew

{
   The next bunch are the various arrangements that make up
   enumArrangement
}
const
<<<<<<< HEAD
=======
	cInternalFinderObject = FourCharCode('obj '); { cReference - used to distinguish objects used inside the Finder only}

{
   Main Finder class definitions
   Indentation implies object model hierarchy
}
const
{ We do not use class cItem from AERegistry.r. Instead our class Item is a cObject}
                                        {         cItem                        = 'citm',   // defined in AERegistry.r}
                                        {          cFile                    = 'file',  // defined in AERegistry.r}
	cAliasFile = FourCharCode('alia');
	cApplicationFile = FourCharCode('appf');
	cControlPanelFile = FourCharCode('ccdv');
	cDeskAccessoryFile = FourCharCode('dafi');
	cDocumentFile = FourCharCode('docf');
	cFontFile = FourCharCode('fntf');
	cSoundFile = FourCharCode('sndf');
	cClippingFile = FourCharCode('clpf');
	cContainer = FourCharCode('ctnr');
	cDesktop = FourCharCode('cdsk');
	cSharableContainer = FourCharCode('sctr');
	cDisk = FourCharCode('cdis');
	cFolder = FourCharCode('cfol');
	cSuitcase = FourCharCode('stcs');
	cAccessorySuitcase = FourCharCode('dsut');
	cFontSuitcase = FourCharCode('fsut');
	cTrash = FourCharCode('ctrs');
	cDesktopPrinter = FourCharCode('dskp');
	cPackage = FourCharCode('pack');
	cContentSpace = FourCharCode('dwnd'); {          cWindow                    = 'cwin',       // defined in AERegistry.r}
	cContainerWindow = FourCharCode('cwnd');
	cInfoWindow = FourCharCode('iwnd');
	cSharingWindow = FourCharCode('swnd');
	cStatusWindow = FourCharCode('qwnd');
	cClippingWindow = FourCharCode('lwnd');
	cPreferencesWindow = FourCharCode('pwnd');
	cDTPWindow = FourCharCode('dtpw');
	cProcess = FourCharCode('prcs');
	cAccessoryProcess = FourCharCode('pcda');
	cApplicationProcess = FourCharCode('pcap');
	cGroup = FourCharCode('sgrp');
	cUser = FourCharCode('cuse'); {         cApplication                  = 'capp',     // defined in AERegistry.r}
	cSharingPrivileges = FourCharCode('priv');
	cPreferences = FourCharCode('cprf');
	cLabel = FourCharCode('clbl');
	cSound = FourCharCode('snd ');
	cAliasList = FourCharCode('alst');
	cSpecialFolders = FourCharCode('spfl'); { For use by viewer search engines:}
	cOnlineDisk = FourCharCode('cods');
	cOnlineLocalDisk = FourCharCode('clds');
	cOnlineRemoteDisk = FourCharCode('crds'); { Miscellaneous class definitions}
	cEntireContents = FourCharCode('ects');
	cIconFamily = FourCharCode('ifam');


{
  //////////////////////////////////////
   Properties
  //////////////////////////////////////
}

{ Properties of class cItem (really cObject)}
const
{    pBounds                        = 'pbnd',       // defined in AERegistry.r}
	pComment = FourCharCode('comt');
	pContainer = cContainer;
	pContentSpace = cContentSpace;
	pCreationDateOld = FourCharCode('crtd'); { to support pre-Finder 8 scripts}
	pCreationDate = FourCharCode('ascd'); { from File Commands OSAX}
	pDescription = FourCharCode('dscr');
	pDisk = cDisk;
	pFolderOld = cFolder; { to support pre-Finder 8 scripts}
	pFolder = FourCharCode('asdr'); { from File Commands OSAX}
	pIconBitmap = FourCharCode('iimg'); {    pID                           = 'ID  ',        // defined in AERegistry.r}
	pInfoWindow = cInfoWindow;
	pKind = FourCharCode('kind');
	pLabelIndex = FourCharCode('labi');
	pModificationDateOld = FourCharCode('modd'); { to support pre-Finder 8 scripts}
	pModificationDate = FourCharCode('asmo'); { from File Commands OSAX}
                                        {    pName                      = 'pnam',         // defined in AERegistry.r}
	pPhysicalSize = FourCharCode('phys');
	pPosition = FourCharCode('posn');
	pIsSelected = FourCharCode('issl');
	pSize = pPointSize; { pPointSize defined in AERegistry.r}
	pWindow = cWindow;
	pPreferencesWindow = cPreferencesWindow;


{ Properties of class cFile (subclass of cItem)}
const
	pFileCreator = FourCharCode('fcrt');
	pFileType = FourCharCode('asty'); { from File Commands OSAX}
	pFileTypeOld = FourCharCode('fitp'); { to support pre-Finder 8 scripts}
	pIsLocked = FourCharCode('aslk'); { from File Commands OSAX}
	pIsLockedOld = FourCharCode('islk'); { to support pre-Finder 8 scripts}
                                        {    pIsStationeryPad               = 'pspd',         // defined in AERegistry.r                }
                                        {    pVersion                    = 'vers',       // defined in AERegistry.r}
	pProductVersion = FourCharCode('ver2');


{ Properties of class cAliasFile (subclass of cFile)}
const
	pOriginalItem = FourCharCode('orig');

{ Properties of class cApplicationFile (subclass of cFile)}
const
	pMinAppPartition = FourCharCode('mprt');
	pAppPartition = FourCharCode('appt');
	pSuggestedAppPartition = FourCharCode('sprt');
	pIsScriptable = FourCharCode('isab');

{ Properties of class cURLFile (subclass of cFile)}
const
	pInternetLocation = FourCharCode('iloc');

{ Properties of class cSoundFile (subclass of cFile)}
const
	pSound = FourCharCode('snd ');


{
   Properties of class cControlPanel (Views CP only) (subclass of cFile)
   Note: the other view-like preference settings are not available in the Views
   control panel. These properties are only offered here for backward compatability.
   To set the full range of Finder Preferences, use the Preferences object.
}
const
	pShowFolderSize = FourCharCode('sfsz'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pShowComment = FourCharCode('scom'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pShowDate = FourCharCode('sdat'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pShowCreationDate = FourCharCode('scda'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pShowKind = FourCharCode('sknd'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pShowLabel = FourCharCode('slbl'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pShowSize = FourCharCode('ssiz'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pShowVersion = FourCharCode('svrs'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pSortDirection = FourCharCode('sord');
	pShowDiskInfo = FourCharCode('sdin'); { Always on in Finder 8.0 HIS}
	pListViewIconSize = FourCharCode('lvis'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pGridIcons = FourCharCode('fgrd'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pStaggerIcons = FourCharCode('fstg'); { No longer part of the Finder 8.0 HIS}
	pViewFont = FourCharCode('vfnt');
	pViewFontSize = FourCharCode('vfsz');

{ Properties of class cContainer (subclass of cItem)}
const
	pCompletelyExpanded = FourCharCode('pexc');
	pContainerWindow = cContainerWindow;
	pEntireContents = cEntireContents;
	pExpandable = FourCharCode('pexa');
	pExpanded = FourCharCode('pexp');
	pPreviousView = FourCharCode('svew'); {    pSelection                    = 'sele',       // defined in AERegistry.r}
	pView = FourCharCode('pvew');
	pIconSize = pListViewIconSize; { defined above}
	pKeepArranged = FourCharCode('arrg'); { OBSOLETE in Finder 9 or later}
	pKeepArrangedBy = FourCharCode('arby'); { OBSOLETE in Finder 9 or later}

{ Properties of class cDesktop (subclass of cContainer)}
const
	pStartupDisk = FourCharCode('sdsk');
	pTrash = FourCharCode('trsh');

{ Properties of class cSharableContainer (subclass of cContainer)}
const
	pOwner = FourCharCode('sown');
	pOwnerPrivileges = FourCharCode('ownr');
	pGroup = cGroup;
	pGroupPrivileges = FourCharCode('gppr');
	pGuestPrivileges = FourCharCode('gstp');
	pArePrivilegesInherited = FourCharCode('iprv');
	pExported = FourCharCode('sexp');
	pMounted = FourCharCode('smou');
	pSharingProtection = FourCharCode('spro');
	pSharing = FourCharCode('shar');
	pSharingWindow = cSharingWindow;

{ Properties of class cDisk (subclass of cSharableContainer)}
const
	pCapacity = FourCharCode('capa');
	pEjectable = FourCharCode('isej');
	pFreeSpace = FourCharCode('frsp');
	pLocal = FourCharCode('isrv');
	pIsStartup = FourCharCode('istd');

{ Properties of class cTrash (subclass of cSharableContainer)}
const
	pWarnOnEmpty = FourCharCode('warn');
=======
>>>>>>> graemeg/cpstrnew

{ Properties of class cWindow (subclass of cContentSpace)}
const
<<<<<<< HEAD
{    pBounds                        = 'pbnd',   // defined in AERegistry.r}
                                        {    pHasCloseBox                = 'hclb',     // defined in AERegistry.r}
                                        {    pIsFloating                    = 'isfl',     // defined in AERegistry.r}
                                        {    pIndex                     = 'pidx',     // defined in AERegistry.r}
                                        {    pIsModal                    = 'pmod',   // defined in AERegistry.r}
                                        {    pPosition                    = 'posn',     // defined above}
                                        {    pIsResizable                = 'prsz',     // defined in AERegistry.r}
                                        {    pHasTitleBar                = 'ptit',     // defined in AERegistry.r}
                                        {    pVisible                    = 'pvis',   // defined in AERegistry.r}
                                        {    pIsZoomable                    = 'iszm',     // defined in AERegistry.r}
                                        {    pIsZoomed                    = 'pzum',     // defined in AERegistry.r}
	pIsZoomedFull = FourCharCode('zumf');
	pIsPopup = FourCharCode('drwr');
	pIsPulledOpen = FourCharCode('pull'); { only applies to popup windows}
	pIsCollapsed = FourCharCode('wshd'); { only applies to normal windows}

{ Properties of class cContainerWindow (subclass of cWindow)}
const
	pObject = cObject;

{ Properties of class cSharingWindow (subclass of cWindow)}
const
	pSharableContainer = cSharableContainer;
=======
>>>>>>> graemeg/cpstrnew

{ Properties of class cInfoWindow (subclass of cWindow)}
const
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	cInternalFinderObject = FourCharCode('obj '); { cReference - used to distinguish objects used inside the Finder only}

{
   Main Finder class definitions
   Indentation implies object model hierarchy
}
const
{ We do not use class cItem from AERegistry.r. Instead our class Item is a cObject}
                                        {         cItem                        = 'citm',   // defined in AERegistry.r}
                                        {          cFile                    = 'file',  // defined in AERegistry.r}
	cAliasFile = FourCharCode('alia');
	cApplicationFile = FourCharCode('appf');
	cControlPanelFile = FourCharCode('ccdv');
	cDeskAccessoryFile = FourCharCode('dafi');
	cDocumentFile = FourCharCode('docf');
	cFontFile = FourCharCode('fntf');
	cSoundFile = FourCharCode('sndf');
	cClippingFile = FourCharCode('clpf');
	cContainer = FourCharCode('ctnr');
	cDesktop = FourCharCode('cdsk');
	cSharableContainer = FourCharCode('sctr');
	cDisk = FourCharCode('cdis');
	cFolder = FourCharCode('cfol');
	cSuitcase = FourCharCode('stcs');
	cAccessorySuitcase = FourCharCode('dsut');
	cFontSuitcase = FourCharCode('fsut');
	cTrash = FourCharCode('ctrs');
	cDesktopPrinter = FourCharCode('dskp');
	cPackage = FourCharCode('pack');
	cContentSpace = FourCharCode('dwnd'); {          cWindow                    = 'cwin',       // defined in AERegistry.r}
	cContainerWindow = FourCharCode('cwnd');
	cInfoWindow = FourCharCode('iwnd');
	cSharingWindow = FourCharCode('swnd');
	cStatusWindow = FourCharCode('qwnd');
	cClippingWindow = FourCharCode('lwnd');
	cPreferencesWindow = FourCharCode('pwnd');
	cDTPWindow = FourCharCode('dtpw');
	cProcess = FourCharCode('prcs');
	cAccessoryProcess = FourCharCode('pcda');
	cApplicationProcess = FourCharCode('pcap');
	cGroup = FourCharCode('sgrp');
	cUser = FourCharCode('cuse'); {         cApplication                  = 'capp',     // defined in AERegistry.r}
	cSharingPrivileges = FourCharCode('priv');
	cPreferences = FourCharCode('cprf');
	cLabel = FourCharCode('clbl');
	cSound = FourCharCode('snd ');
	cAliasList = FourCharCode('alst');
	cSpecialFolders = FourCharCode('spfl'); { For use by viewer search engines:}
	cOnlineDisk = FourCharCode('cods');
	cOnlineLocalDisk = FourCharCode('clds');
	cOnlineRemoteDisk = FourCharCode('crds'); { Miscellaneous class definitions}
	cEntireContents = FourCharCode('ects');
	cIconFamily = FourCharCode('ifam');


{
  //////////////////////////////////////
   Properties
  //////////////////////////////////////
}

{ Properties of class cItem (really cObject)}
const
{    pBounds                        = 'pbnd',       // defined in AERegistry.r}
	pComment = FourCharCode('comt');
	pContainer = cContainer;
	pContentSpace = cContentSpace;
	pCreationDateOld = FourCharCode('crtd'); { to support pre-Finder 8 scripts}
	pCreationDate = FourCharCode('ascd'); { from File Commands OSAX}
	pDescription = FourCharCode('dscr');
	pDisk = cDisk;
	pFolderOld = cFolder; { to support pre-Finder 8 scripts}
	pFolder = FourCharCode('asdr'); { from File Commands OSAX}
	pIconBitmap = FourCharCode('iimg'); {    pID                           = 'ID  ',        // defined in AERegistry.r}
	pInfoWindow = cInfoWindow;
	pKind = FourCharCode('kind');
	pLabelIndex = FourCharCode('labi');
	pModificationDateOld = FourCharCode('modd'); { to support pre-Finder 8 scripts}
	pModificationDate = FourCharCode('asmo'); { from File Commands OSAX}
                                        {    pName                      = 'pnam',         // defined in AERegistry.r}
	pPhysicalSize = FourCharCode('phys');
	pPosition = FourCharCode('posn');
	pIsSelected = FourCharCode('issl');
	pSize = pPointSize; { pPointSize defined in AERegistry.r}
	pWindow = cWindow;
	pPreferencesWindow = cPreferencesWindow;


{ Properties of class cFile (subclass of cItem)}
const
	pFileCreator = FourCharCode('fcrt');
	pFileType = FourCharCode('asty'); { from File Commands OSAX}
	pFileTypeOld = FourCharCode('fitp'); { to support pre-Finder 8 scripts}
	pIsLocked = FourCharCode('aslk'); { from File Commands OSAX}
	pIsLockedOld = FourCharCode('islk'); { to support pre-Finder 8 scripts}
                                        {    pIsStationeryPad               = 'pspd',         // defined in AERegistry.r                }
                                        {    pVersion                    = 'vers',       // defined in AERegistry.r}
	pProductVersion = FourCharCode('ver2');


{ Properties of class cAliasFile (subclass of cFile)}
const
	pOriginalItem = FourCharCode('orig');

{ Properties of class cApplicationFile (subclass of cFile)}
const
	pMinAppPartition = FourCharCode('mprt');
	pAppPartition = FourCharCode('appt');
	pSuggestedAppPartition = FourCharCode('sprt');
	pIsScriptable = FourCharCode('isab');

{ Properties of class cURLFile (subclass of cFile)}
const
	pInternetLocation = FourCharCode('iloc');

{ Properties of class cSoundFile (subclass of cFile)}
const
	pSound = FourCharCode('snd ');


{
   Properties of class cControlPanel (Views CP only) (subclass of cFile)
   Note: the other view-like preference settings are not available in the Views
   control panel. These properties are only offered here for backward compatability.
   To set the full range of Finder Preferences, use the Preferences object.
}
const
	pShowFolderSize = FourCharCode('sfsz'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pShowComment = FourCharCode('scom'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pShowDate = FourCharCode('sdat'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pShowCreationDate = FourCharCode('scda'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pShowKind = FourCharCode('sknd'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pShowLabel = FourCharCode('slbl'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pShowSize = FourCharCode('ssiz'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pShowVersion = FourCharCode('svrs'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pSortDirection = FourCharCode('sord');
	pShowDiskInfo = FourCharCode('sdin'); { Always on in Finder 8.0 HIS}
	pListViewIconSize = FourCharCode('lvis'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pGridIcons = FourCharCode('fgrd'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pStaggerIcons = FourCharCode('fstg'); { No longer part of the Finder 8.0 HIS}
	pViewFont = FourCharCode('vfnt');
	pViewFontSize = FourCharCode('vfsz');

{ Properties of class cContainer (subclass of cItem)}
const
	pCompletelyExpanded = FourCharCode('pexc');
	pContainerWindow = cContainerWindow;
	pEntireContents = cEntireContents;
	pExpandable = FourCharCode('pexa');
	pExpanded = FourCharCode('pexp');
	pPreviousView = FourCharCode('svew'); {    pSelection                    = 'sele',       // defined in AERegistry.r}
	pView = FourCharCode('pvew');
	pIconSize = pListViewIconSize; { defined above}
	pKeepArranged = FourCharCode('arrg'); { OBSOLETE in Finder 9 or later}
	pKeepArrangedBy = FourCharCode('arby'); { OBSOLETE in Finder 9 or later}

{ Properties of class cDesktop (subclass of cContainer)}
const
	pStartupDisk = FourCharCode('sdsk');
	pTrash = FourCharCode('trsh');

{ Properties of class cSharableContainer (subclass of cContainer)}
const
	pOwner = FourCharCode('sown');
	pOwnerPrivileges = FourCharCode('ownr');
	pGroup = cGroup;
	pGroupPrivileges = FourCharCode('gppr');
	pGuestPrivileges = FourCharCode('gstp');
	pArePrivilegesInherited = FourCharCode('iprv');
	pExported = FourCharCode('sexp');
	pMounted = FourCharCode('smou');
	pSharingProtection = FourCharCode('spro');
	pSharing = FourCharCode('shar');
	pSharingWindow = cSharingWindow;

{ Properties of class cDisk (subclass of cSharableContainer)}
const
	pCapacity = FourCharCode('capa');
	pEjectable = FourCharCode('isej');
	pFreeSpace = FourCharCode('frsp');
	pLocal = FourCharCode('isrv');
	pIsStartup = FourCharCode('istd');

{ Properties of class cTrash (subclass of cSharableContainer)}
const
	pWarnOnEmpty = FourCharCode('warn');

{ Properties of class cWindow (subclass of cContentSpace)}
const
{    pBounds                        = 'pbnd',   // defined in AERegistry.r}
                                        {    pHasCloseBox                = 'hclb',     // defined in AERegistry.r}
                                        {    pIsFloating                    = 'isfl',     // defined in AERegistry.r}
                                        {    pIndex                     = 'pidx',     // defined in AERegistry.r}
                                        {    pIsModal                    = 'pmod',   // defined in AERegistry.r}
                                        {    pPosition                    = 'posn',     // defined above}
                                        {    pIsResizable                = 'prsz',     // defined in AERegistry.r}
                                        {    pHasTitleBar                = 'ptit',     // defined in AERegistry.r}
                                        {    pVisible                    = 'pvis',   // defined in AERegistry.r}
                                        {    pIsZoomable                    = 'iszm',     // defined in AERegistry.r}
                                        {    pIsZoomed                    = 'pzum',     // defined in AERegistry.r}
	pIsZoomedFull = FourCharCode('zumf');
	pIsPopup = FourCharCode('drwr');
	pIsPulledOpen = FourCharCode('pull'); { only applies to popup windows}
	pIsCollapsed = FourCharCode('wshd'); { only applies to normal windows}

{ Properties of class cContainerWindow (subclass of cWindow)}
const
	pObject = cObject;

{ Properties of class cSharingWindow (subclass of cWindow)}
const
	pSharableContainer = cSharableContainer;

{ Properties of class cInfoWindow (subclass of cWindow)}
const
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	pInfoPanel = FourCharCode('panl');


{ Properties of networking support}
const
	pFileShareOn = FourCharCode('fshr');
	pFileShareStartingUp = FourCharCode('fsup');
	pProgramLinkingOn = FourCharCode('iac ');

{ Properties of class cPreferencesWindow (subclass of cWindow)}
const
{    pShowFolderSize                   = 'sfsz',         // defined above for Views CP}
                                        {    pShowComment                = 'scom',      // defined above for Views CP}
	pShowModificationDate = pShowDate; { pShowDate defined above for Views CP}
                                        {    pShowKind                    = 'sknd',        // defined above for Views CP}
                                        {    pShowLabel                    = 'slbl',         // defined above for Views CP}
                                        {    pShowSize                    = 'ssiz',        // defined above for Views CP}
                                        {    pShowVersion                = 'svrs',      // defined above for Views CP}
                                        {    pShowCreationDate             = 'scda',      // Removed from Finder 8.0 HIS}
                                        {    pShowFileType                 = 'sfty',       // Removed from Finder 8.0 HIS}
                                        {    pShowFileCreator               = 'sfcr',         // Removed from Finder 8.0 HIS}
                                        {    pListViewIconSize             = 'lvis',      // defined above for Views CP}
                                        {    pGridIcons                    = 'fgrd',         // defined above for Views CP}
                                        {    pStaggerIcons                 = 'fstg',       // defined above for Views CP}
                                        {    pViewFont                    = 'vfnt',        // defined above for Views CP}
                                        {    pViewFontSize                 = 'vfsz',       // defined above for Views CP}
	pUseRelativeDate = FourCharCode('urdt'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pDelayBeforeSpringing = FourCharCode('dela');
	pSpringOpenFolders = FourCharCode('sprg');
	pUseShortMenus = FourCharCode('usme');
	pUseWideGrid = FourCharCode('uswg');
	pLabel1 = FourCharCode('lbl1');
	pLabel2 = FourCharCode('lbl2');
	pLabel3 = FourCharCode('lbl3');
	pLabel4 = FourCharCode('lbl4');
	pLabel5 = FourCharCode('lbl5');
	pLabel6 = FourCharCode('lbl6');
	pLabel7 = FourCharCode('lbl7');
	pDefaultIconViewIconSize = FourCharCode('iisz');
	pDefaultButtonViewIconSize = FourCharCode('bisz');
	pDefaultListViewIconSize = FourCharCode('lisz'); { old use of this name is now pIconSize}
	pIconViewArrangement = FourCharCode('iarr');
	pButtonViewArrangement = FourCharCode('barr');

{
   The next bunch are the various arrangements that make up
   enumArrangement
}
const
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
	pNoArrangement = FourCharCode('narr');
	pSnapToGridArrangement = FourCharCode('grda');
	pByNameArrangement = FourCharCode('nama');
	pByModificationDateArrangement = FourCharCode('mdta');
	pByCreationDateArrangement = FourCharCode('cdta');
	pBySizeArrangement = FourCharCode('siza');
	pByKindArrangement = FourCharCode('kina');
	pByLabelArrangement = FourCharCode('laba');

{  #define pObject                                 cObject         // defined above}

{ Properties of class cProcess (subclass of cObject)}
const
{    pName                      = 'pnam',         // defined in AERegistry.r}
	pFile = cFile; {    pCreatorType                = 'fcrt',      // defined above}
                                        {    pFileType                    = 'asty',        // defined above}
                                        {    pIsFrontProcess                   = 'pisf',         // defined in AERegistry.r}
                                        {    pAppPartition                 = 'appt',       // defined above}
	pPartitionSpaceUsed = FourCharCode('pusd'); {    pIsScriptable                 = 'isab',       // defined in AERegistry.r}
                                        {    pVisible                    = 'pvis'      // defined in AERegistry.r}
	pLocalAndRemoteEvents = FourCharCode('revt');
	pHasScriptingTerminology = FourCharCode('hscr');

{ Properties of class cAccessoryProcess (subclass of cProcess)}
const
	pDeskAccessoryFile = cDeskAccessoryFile;

{ Properties of class cApplicationProcess (subclass of cProcess)}
const
	pApplicationFile = cApplicationFile;

{
   Properties of class cGroup (subclass of cObject)
  enum (
    pBounds
    pIconBitmap
    pLabelIndex
    pName
    pPosition
    pWindow                                 = cWindow           // defined above
  );
}

{ Properties of class cUser (subclass of cObject)}
const
{    pBounds}
                                        {    pIconBitmap}
                                        {    pLabelIndex}
                                        {    pName}
                                        {    pPosition}
                                        {    pWindow                        = cWindow,        // defined above}
	pCanConnect = FourCharCode('ccon');
	pCanChangePassword = FourCharCode('ccpw');
	pCanDoProgramLinking = FourCharCode('ciac');
	pIsOwner = FourCharCode('isow');
	pARADialIn = FourCharCode('arad');
	pShouldCallBack = FourCharCode('calb');
	pCallBackNumber = FourCharCode('cbnm');

{
   Properties of class cApplication (subclass of cObject)
   NOTE: properties for the special folders must match their respective kXXXFolderType constants
}
const
	pAboutMacintosh = FourCharCode('abbx');
	pAppleMenuItemsFolder = FourCharCode('amnu'); { kAppleMenuFolderType}
                                        {    pClipboard                    = 'pcli',         // defined in AERegistry.r}
	pControlPanelsFolder = FourCharCode('ctrl'); { kControlPanelFolderType}
	pDesktop = FourCharCode('desk'); { kDesktopFolderType}
	pExtensionsFolder = FourCharCode('extn'); { kExtensionFolderType}
                                        {    pFileShareOn                = 'fshr',      // defined above}
	pFinderPreferences = FourCharCode('pfrp');
	pFontsFolder = FourCharCode('font');
	pFontsFolderPreAllegro = FourCharCode('ffnt'); { DO NOT USE THIS - FOR BACKWARDS COMPAT ONLY}
                                        {    pIsFrontProcess                   = 'pisf',         // defined in AERegistry.r}
                                        {    pInsertionLoc                 = 'pins',       // defined in AERegistry.r}
	pLargestFreeBlock = FourCharCode('mfre');
	pPreferencesFolder = FourCharCode('pref'); { kPreferencesFolderType}
                                        {    pProductVersion                   = 'ver2',         // defined above}
                                        {    pUserSelection                  = 'pusl',        // defined in AERegistry.r}
                                        {    pFileShareStartingUp             = 'fsup',        // defined above}
	pShortCuts = FourCharCode('scut');
	pShutdownFolder = FourCharCode('shdf');
	pStartupItemsFolder = FourCharCode('strt'); { kStartupFolderType}
	pSystemFolder = FourCharCode('macs'); { kSystemFolderType}
	pTemporaryFolder = FourCharCode('temp'); { kTemporaryFolderType}
                                        {    pVersion                    = 'vers',       // defined in AERegistry.r}
	pViewPreferences = FourCharCode('pvwp'); {    pVisible                    = 'pvis',       // defined in AERegistry.r}
	pStartingUp = FourCharCode('awak'); { private property to tell whether the Finder is fully up and running}

{ Properties of class cSharingPrivileges (subclass of cObject)}
const
	pSeeFiles = FourCharCode('prvr');
	pSeeFolders = FourCharCode('prvs');
	pMakeChanges = FourCharCode('prvw');

{
   Properties of class cPreferences (subclass of cObject)
  enum (
    pShowFolderSize                         = 'sfsz',           // defined above for Views CP
    pShowComment                            = 'scom',           // defined above for Views CP
    pShowModificationDate                   = pShowDate,            // pShowDate defined above for Views CP
    pShowKind                               = 'sknd',           // defined above for Views CP
    pShowLabel                              = 'slbl',           // defined above for Views CP
    pShowSize                               = 'ssiz',           // defined above for Views CP
    pShowVersion                            = 'svrs',           // defined above for Views CP
    pShowCreationDate                       = 'scda',           // defined in cPreferencesWindow
    pShowFileType                           = 'sfty',           // defined in cPreferencesWindow
    pShowFileCreator                        = 'sfcr',           // defined in cPreferencesWindow
    pListViewIconSize                       = 'lvis',           // defined above for Views CP
    pGridIcons                              = 'fgrd',           // defined above for Views CP
    pStaggerIcons                           = 'fstg',           // defined above for Views CP
    pViewFont                               = 'vfnt',           // defined above for Views CP
    pViewFontSize                           = 'vfsz',           // defined above for Views CP
    pUseRelativeDate                        = 'urdt',           // defined in cPreferencesWindow
    pDelayBeforeSpringing                   = 'dela',           // defined in cPreferencesWindow
    pShowMacOSFolder                        = 'sosf',           // defined in cPreferencesWindow
    pUseShortMenus                          = 'usme',           // defined in cPreferencesWindow
    pUseCustomNewMenu                       = 'ucnm',           // defined in cPreferencesWindow
    pShowDesktopInBackground                = 'sdtb',           // defined in cPreferencesWindow
    pActivateDesktopOnClick                 = 'adtc',           // defined in cPreferencesWindow
    pLabel1                                 = 'lbl1',           // defined in cPreferencesWindow
    pLabel2                                 = 'lbl2',           // defined in cPreferencesWindow
    pLabel3                                 = 'lbl3',           // defined in cPreferencesWindow
    pLabel4                                 = 'lbl4',           // defined in cPreferencesWindow
    pLabel5                                 = 'lbl5',           // defined in cPreferencesWindow
    pLabel6                                 = 'lbl6',           // defined in cPreferencesWindow
    pLabel7                                 = 'lbl7',           // defined in cPreferencesWindow
    pWindow                                 = cWindow           // defined above
  );
}

{
   Properties of class cLabel (subclass of cObject)
  enum (
    pName                                   = 'pnam',           // defined in AERegistry.r
    pColor                                  = 'colr',           // defined in AERegistry.r
  );
}

{ Misc Properties}
const
	pSmallIcon = FourCharCode('smic');
	pSmallButton = FourCharCode('smbu');
	pLargeButton = FourCharCode('lgbu');
	pGrid = FourCharCode('grid');

{
  //////////////////////////////////////
   Enumerations defined by the Finder
  //////////////////////////////////////
}

const
	enumViewBy = FourCharCode('vwby');
	enumGestalt = FourCharCode('gsen');
	enumConflicts = FourCharCode('cflc');
	enumExistingItems = FourCharCode('exsi');
	enumOlderItems = FourCharCode('oldr');

const
	enumDate = FourCharCode('enda');
	enumAnyDate = FourCharCode('anyd');
	enumToday = FourCharCode('tday');
	enumYesterday = FourCharCode('yday');
	enumThisWeek = FourCharCode('twek');
	enumLastWeek = FourCharCode('lwek');
	enumThisMonth = FourCharCode('tmon');
	enumLastMonth = FourCharCode('lmon');
	enumThisYear = FourCharCode('tyer');
	enumLastYear = FourCharCode('lyer');
	enumBeforeDate = FourCharCode('bfdt');
	enumAfterDate = FourCharCode('afdt');
	enumBetweenDate = FourCharCode('btdt');
	enumOnDate = FourCharCode('ondt');

const
	enumAllDocuments = FourCharCode('alld');
	enumFolders = FourCharCode('fold');
	enumAliases = FourCharCode('alia');
	enumStationery = FourCharCode('stat');

const
	enumWhere = FourCharCode('wher');
	enumAllLocalDisks = FourCharCode('aldk');
	enumAllRemoteDisks = FourCharCode('ardk');
	enumAllDisks = FourCharCode('alld');
	enumAllOpenFolders = FourCharCode('aofo');


const
	enumIconSize = FourCharCode('isiz');
	enumSmallIconSize = pSmallIcon;
	enumMiniIconSize = FourCharCode('miic');
	enumLargeIconSize = FourCharCode('lgic');

const
	enumSortDirection = FourCharCode('sodr');
	enumSortDirectionNormal = FourCharCode('snrm');
	enumSortDirectionReverse = FourCharCode('srvs');

const
	enumArrangement = FourCharCode('earr');

{ Get Info Window panel enumeration}
const
	enumInfoWindowPanel = FourCharCode('ipnl');
	enumGeneralPanel = FourCharCode('gpnl');
	enumSharingPanel = FourCharCode('spnl');
	enumStatusNConfigPanel = FourCharCode('scnl');
	enumFontsPanel = FourCharCode('fpnl');
	enumMemoryPanel = FourCharCode('mpnl');


{ Preferences panel enumeration}
const
	enumPrefsWindowPanel = FourCharCode('pple');
	enumPrefsGeneralPanel = FourCharCode('pgnp');
	enumPrefsLabelPanel = FourCharCode('plbp');
	enumPrefsIconViewPanel = FourCharCode('pivp');
	enumPrefsButtonViewPanel = FourCharCode('pbvp');
	enumPrefsListViewPanel = FourCharCode('plvp');

{
  //////////////////////////////////////
   Types defined by the Finder
  //////////////////////////////////////
}

const
	typeIconFamily = cIconFamily; { An AEList of typeIconAndMask, type8BitIcon, & c.}
	typeIconAndMask = FourCharCode('ICN#');
	type8BitMask = FourCharCode('l8mk');
	type32BitIcon = FourCharCode('il32');
	type8BitIcon = FourCharCode('icl8');
	type4BitIcon = FourCharCode('icl4');
	typeSmallIconAndMask = FourCharCode('ics#');
	typeSmall8BitMask = FourCharCode('s8mk');
	typeSmall32BitIcon = FourCharCode('is32');
	typeSmall8BitIcon = FourCharCode('ics8');
	typeSmall4BitIcon = FourCharCode('ics4');
	typeRelativeTime = FourCharCode('rtim');
	typeConceptualTime = FourCharCode('timc');

{
  //////////////////////////////////////
   Keywords defined by the Finder
  //////////////////////////////////////
}

const
	keyIconAndMask = FourCharCode('ICN#');
	key32BitIcon = FourCharCode('il32');
	key8BitIcon = FourCharCode('icl8');
	key4BitIcon = FourCharCode('icl4');
	key8BitMask = FourCharCode('l8mk');
	keySmallIconAndMask = FourCharCode('ics#');
	keySmall8BitIcon = FourCharCode('ics8');
	keySmall4BitIcon = FourCharCode('ics4');
	keySmall32BitIcon = FourCharCode('is32');
	keySmall8BitMask = FourCharCode('s8mk');
	keyMini1BitMask = FourCharCode('icm#');
	keyMini4BitIcon = FourCharCode('icm4');
	keyMini8BitIcon = FourCharCode('icm8');
	keyAEUsing = FourCharCode('usin');
	keyAEReplacing = FourCharCode('alrp');
	keyAENoAutoRouting = FourCharCode('rout');
	keyLocalPositionList = FourCharCode('mvpl');
	keyGlobalPositionList = FourCharCode('mvpg');
	keyRedirectedDocumentList = FourCharCode('fpdl');

{
  //////////////////////////////////////
   New prepositions used by the Finder
  //////////////////////////////////////
}

const
	keyASPrepositionHas = FourCharCode('has ');
	keyAll = FourCharCode('kyal');
	keyOldFinderItems = FourCharCode('fsel');

{
  //////////////////////////////////////
   New key forms used by the Finder
  //////////////////////////////////////
}

const
	formAlias = typeAlias;
	formCreator = pFileCreator;


{
  //////////////////////////////////////
   Finder error codes
  //////////////////////////////////////
}

const
	errFinderIsBusy = -15260;
	errFinderWindowNotOpen = -15261;
	errFinderCannotPutAway = -15262;
	errFinderWindowMustBeIconView = -15263; { RequireWindowInIconView}
	errFinderWindowMustBeListView = -15264; { RequireWindowInListView}
	errFinderCantMoveToDestination = -15265;
	errFinderCantMoveSource = -15266;
	errFinderCantOverwrite = -15267;
	errFinderIncestuousMove = -15268; { Could just use errFinderCantMoveSource}
	errFinderCantMoveToAncestor = -15269; { Could also use errFinderCantMoveSource}
	errFinderCantUseTrashedItems = -15270;
	errFinderItemAlreadyInDest = -15271; { Move from folder A to folder A}
	errFinderUnknownUser = -15272; { Includes unknown group}
	errFinderSharePointsCantInherit = -15273;
	errFinderWindowWrongType = -15274;
	errFinderPropertyNowWindowBased = -15275;
	errFinderAppFolderProtected = -15276; { used by General controls when folder protection is on}
	errFinderSysFolderProtected = -15277; { used by General controls when folder protection is on}
	errFinderBoundsWrong = -15278;
	errAEValueOutOfRange = -15279;
	errFinderPropertyDoesNotApply = -15280;
	errFinderFileSharingMustBeOn = -15281;
	errFinderMustBeActive = -15282;
	errFinderVolumeNotFound = -15283; { more descriptive than what we get with nsvErr}
	errFinderLockedItemsInTrash = -15284; { there are some locked items in the trash}
	errFinderOnlyLockedItemsInTrash = -15285; { all the items (except folders) in the trash are locked}
	errFinderProgramLinkingMustBeOn = -15286;
	errFinderWindowMustBeButtonView = -15287;
	errFinderBadPackageContents = -15288; { something is wrong within the package   }
	errFinderUnsupportedInsidePackages = -15289; { operation cannot be used on items within a package     }
	errFinderCorruptOpenFolderList = -15290; { was -15276 in Finder 8.6 and earlier, but that conflicted with General Controls}
	errFinderNoInvisibleFiles = -15291; { was -15277 in Finder 8.6 and earlier, but that conflicted with General Controls}
	errFinderCantDeleteImmediately = -15292; { cannot delete immediately via scripting}
	errFinderLastReserved = -15379;

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

const
	cInternalFinderObject = FourCharCode('obj '); { cReference - used to distinguish objects used inside the Finder only}

{
   Main Finder class definitions
   Indentation implies object model hierarchy
}
const
{ We do not use class cItem from AERegistry.r. Instead our class Item is a cObject}
                                        {         cItem                        = 'citm',   // defined in AERegistry.r}
                                        {          cFile                    = 'file',  // defined in AERegistry.r}
	cAliasFile = FourCharCode('alia');
	cApplicationFile = FourCharCode('appf');
	cControlPanelFile = FourCharCode('ccdv');
	cDeskAccessoryFile = FourCharCode('dafi');
	cDocumentFile = FourCharCode('docf');
	cFontFile = FourCharCode('fntf');
	cSoundFile = FourCharCode('sndf');
	cClippingFile = FourCharCode('clpf');
	cContainer = FourCharCode('ctnr');
	cDesktop = FourCharCode('cdsk');
	cSharableContainer = FourCharCode('sctr');
	cDisk = FourCharCode('cdis');
	cFolder = FourCharCode('cfol');
	cSuitcase = FourCharCode('stcs');
	cAccessorySuitcase = FourCharCode('dsut');
	cFontSuitcase = FourCharCode('fsut');
	cTrash = FourCharCode('ctrs');
	cDesktopPrinter = FourCharCode('dskp');
	cPackage = FourCharCode('pack');
	cContentSpace = FourCharCode('dwnd'); {          cWindow                    = 'cwin',       // defined in AERegistry.r}
	cContainerWindow = FourCharCode('cwnd');
	cInfoWindow = FourCharCode('iwnd');
	cSharingWindow = FourCharCode('swnd');
	cStatusWindow = FourCharCode('qwnd');
	cClippingWindow = FourCharCode('lwnd');
	cPreferencesWindow = FourCharCode('pwnd');
	cDTPWindow = FourCharCode('dtpw');
	cProcess = FourCharCode('prcs');
	cAccessoryProcess = FourCharCode('pcda');
	cApplicationProcess = FourCharCode('pcap');
	cGroup = FourCharCode('sgrp');
	cUser = FourCharCode('cuse'); {         cApplication                  = 'capp',     // defined in AERegistry.r}
	cSharingPrivileges = FourCharCode('priv');
	cPreferences = FourCharCode('cprf');
	cLabel = FourCharCode('clbl');
	cSound = FourCharCode('snd ');
	cAliasList = FourCharCode('alst');
	cSpecialFolders = FourCharCode('spfl'); { For use by viewer search engines:}
	cOnlineDisk = FourCharCode('cods');
	cOnlineLocalDisk = FourCharCode('clds');
	cOnlineRemoteDisk = FourCharCode('crds'); { Miscellaneous class definitions}
	cEntireContents = FourCharCode('ects');
	cIconFamily = FourCharCode('ifam');


{
  //////////////////////////////////////
   Properties
  //////////////////////////////////////
}

{ Properties of class cItem (really cObject)}
const
{    pBounds                        = 'pbnd',       // defined in AERegistry.r}
	pComment = FourCharCode('comt');
	pContainer = cContainer;
	pContentSpace = cContentSpace;
	pCreationDateOld = FourCharCode('crtd'); { to support pre-Finder 8 scripts}
	pCreationDate = FourCharCode('ascd'); { from File Commands OSAX}
	pDescription = FourCharCode('dscr');
	pDisk = cDisk;
	pFolderOld = cFolder; { to support pre-Finder 8 scripts}
	pFolder = FourCharCode('asdr'); { from File Commands OSAX}
	pIconBitmap = FourCharCode('iimg'); {    pID                           = 'ID  ',        // defined in AERegistry.r}
	pInfoWindow = cInfoWindow;
	pKind = FourCharCode('kind');
	pLabelIndex = FourCharCode('labi');
	pModificationDateOld = FourCharCode('modd'); { to support pre-Finder 8 scripts}
	pModificationDate = FourCharCode('asmo'); { from File Commands OSAX}
                                        {    pName                      = 'pnam',         // defined in AERegistry.r}
	pPhysicalSize = FourCharCode('phys');
	pPosition = FourCharCode('posn');
	pIsSelected = FourCharCode('issl');
	pSize = pPointSize; { pPointSize defined in AERegistry.r}
	pWindow = cWindow;
	pPreferencesWindow = cPreferencesWindow;


{ Properties of class cFile (subclass of cItem)}
const
	pFileCreator = FourCharCode('fcrt');
	pFileType = FourCharCode('asty'); { from File Commands OSAX}
	pFileTypeOld = FourCharCode('fitp'); { to support pre-Finder 8 scripts}
	pIsLocked = FourCharCode('aslk'); { from File Commands OSAX}
	pIsLockedOld = FourCharCode('islk'); { to support pre-Finder 8 scripts}
                                        {    pIsStationeryPad               = 'pspd',         // defined in AERegistry.r                }
                                        {    pVersion                    = 'vers',       // defined in AERegistry.r}
	pProductVersion = FourCharCode('ver2');


{ Properties of class cAliasFile (subclass of cFile)}
const
	pOriginalItem = FourCharCode('orig');

{ Properties of class cApplicationFile (subclass of cFile)}
const
	pMinAppPartition = FourCharCode('mprt');
	pAppPartition = FourCharCode('appt');
	pSuggestedAppPartition = FourCharCode('sprt');
	pIsScriptable = FourCharCode('isab');

{ Properties of class cURLFile (subclass of cFile)}
const
	pInternetLocation = FourCharCode('iloc');

{ Properties of class cSoundFile (subclass of cFile)}
const
	pSound = FourCharCode('snd ');


{
   Properties of class cControlPanel (Views CP only) (subclass of cFile)
   Note: the other view-like preference settings are not available in the Views
   control panel. These properties are only offered here for backward compatability.
   To set the full range of Finder Preferences, use the Preferences object.
}
const
	pShowFolderSize = FourCharCode('sfsz'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pShowComment = FourCharCode('scom'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pShowDate = FourCharCode('sdat'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pShowCreationDate = FourCharCode('scda'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pShowKind = FourCharCode('sknd'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pShowLabel = FourCharCode('slbl'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pShowSize = FourCharCode('ssiz'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pShowVersion = FourCharCode('svrs'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pSortDirection = FourCharCode('sord');
	pShowDiskInfo = FourCharCode('sdin'); { Always on in Finder 8.0 HIS}
	pListViewIconSize = FourCharCode('lvis'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pGridIcons = FourCharCode('fgrd'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pStaggerIcons = FourCharCode('fstg'); { No longer part of the Finder 8.0 HIS}
	pViewFont = FourCharCode('vfnt');
	pViewFontSize = FourCharCode('vfsz');

{ Properties of class cContainer (subclass of cItem)}
const
	pCompletelyExpanded = FourCharCode('pexc');
	pContainerWindow = cContainerWindow;
	pEntireContents = cEntireContents;
	pExpandable = FourCharCode('pexa');
	pExpanded = FourCharCode('pexp');
	pPreviousView = FourCharCode('svew'); {    pSelection                    = 'sele',       // defined in AERegistry.r}
	pView = FourCharCode('pvew');
	pIconSize = pListViewIconSize; { defined above}
	pKeepArranged = FourCharCode('arrg'); { OBSOLETE in Finder 9 or later}
	pKeepArrangedBy = FourCharCode('arby'); { OBSOLETE in Finder 9 or later}

{ Properties of class cDesktop (subclass of cContainer)}
const
	pStartupDisk = FourCharCode('sdsk');
	pTrash = FourCharCode('trsh');

{ Properties of class cSharableContainer (subclass of cContainer)}
const
	pOwner = FourCharCode('sown');
	pOwnerPrivileges = FourCharCode('ownr');
	pGroup = cGroup;
	pGroupPrivileges = FourCharCode('gppr');
	pGuestPrivileges = FourCharCode('gstp');
	pArePrivilegesInherited = FourCharCode('iprv');
	pExported = FourCharCode('sexp');
	pMounted = FourCharCode('smou');
	pSharingProtection = FourCharCode('spro');
	pSharing = FourCharCode('shar');
	pSharingWindow = cSharingWindow;

{ Properties of class cDisk (subclass of cSharableContainer)}
const
	pCapacity = FourCharCode('capa');
	pEjectable = FourCharCode('isej');
	pFreeSpace = FourCharCode('frsp');
	pLocal = FourCharCode('isrv');
	pIsStartup = FourCharCode('istd');

{ Properties of class cTrash (subclass of cSharableContainer)}
const
	pWarnOnEmpty = FourCharCode('warn');

{ Properties of class cWindow (subclass of cContentSpace)}
const
{    pBounds                        = 'pbnd',   // defined in AERegistry.r}
                                        {    pHasCloseBox                = 'hclb',     // defined in AERegistry.r}
                                        {    pIsFloating                    = 'isfl',     // defined in AERegistry.r}
                                        {    pIndex                     = 'pidx',     // defined in AERegistry.r}
                                        {    pIsModal                    = 'pmod',   // defined in AERegistry.r}
                                        {    pPosition                    = 'posn',     // defined above}
                                        {    pIsResizable                = 'prsz',     // defined in AERegistry.r}
                                        {    pHasTitleBar                = 'ptit',     // defined in AERegistry.r}
                                        {    pVisible                    = 'pvis',   // defined in AERegistry.r}
                                        {    pIsZoomable                    = 'iszm',     // defined in AERegistry.r}
                                        {    pIsZoomed                    = 'pzum',     // defined in AERegistry.r}
	pIsZoomedFull = FourCharCode('zumf');
	pIsPopup = FourCharCode('drwr');
	pIsPulledOpen = FourCharCode('pull'); { only applies to popup windows}
	pIsCollapsed = FourCharCode('wshd'); { only applies to normal windows}

{ Properties of class cContainerWindow (subclass of cWindow)}
const
	pObject = cObject;

{ Properties of class cSharingWindow (subclass of cWindow)}
const
	pSharableContainer = cSharableContainer;

{ Properties of class cInfoWindow (subclass of cWindow)}
const
	pInfoPanel = FourCharCode('panl');


{ Properties of networking support}
const
	pFileShareOn = FourCharCode('fshr');
	pFileShareStartingUp = FourCharCode('fsup');
	pProgramLinkingOn = FourCharCode('iac ');

{ Properties of class cPreferencesWindow (subclass of cWindow)}
const
{    pShowFolderSize                   = 'sfsz',         // defined above for Views CP}
                                        {    pShowComment                = 'scom',      // defined above for Views CP}
	pShowModificationDate = pShowDate; { pShowDate defined above for Views CP}
                                        {    pShowKind                    = 'sknd',        // defined above for Views CP}
                                        {    pShowLabel                    = 'slbl',         // defined above for Views CP}
                                        {    pShowSize                    = 'ssiz',        // defined above for Views CP}
                                        {    pShowVersion                = 'svrs',      // defined above for Views CP}
                                        {    pShowCreationDate             = 'scda',      // Removed from Finder 8.0 HIS}
                                        {    pShowFileType                 = 'sfty',       // Removed from Finder 8.0 HIS}
                                        {    pShowFileCreator               = 'sfcr',         // Removed from Finder 8.0 HIS}
                                        {    pListViewIconSize             = 'lvis',      // defined above for Views CP}
                                        {    pGridIcons                    = 'fgrd',         // defined above for Views CP}
                                        {    pStaggerIcons                 = 'fstg',       // defined above for Views CP}
                                        {    pViewFont                    = 'vfnt',        // defined above for Views CP}
                                        {    pViewFontSize                 = 'vfsz',       // defined above for Views CP}
	pUseRelativeDate = FourCharCode('urdt'); { Moved to a per-folder basis in Finder 8.0 HIS}
	pDelayBeforeSpringing = FourCharCode('dela');
	pSpringOpenFolders = FourCharCode('sprg');
	pUseShortMenus = FourCharCode('usme');
	pUseWideGrid = FourCharCode('uswg');
	pLabel1 = FourCharCode('lbl1');
	pLabel2 = FourCharCode('lbl2');
	pLabel3 = FourCharCode('lbl3');
	pLabel4 = FourCharCode('lbl4');
	pLabel5 = FourCharCode('lbl5');
	pLabel6 = FourCharCode('lbl6');
	pLabel7 = FourCharCode('lbl7');
	pDefaultIconViewIconSize = FourCharCode('iisz');
	pDefaultButtonViewIconSize = FourCharCode('bisz');
	pDefaultListViewIconSize = FourCharCode('lisz'); { old use of this name is now pIconSize}
	pIconViewArrangement = FourCharCode('iarr');
	pButtonViewArrangement = FourCharCode('barr');

{
   The next bunch are the various arrangements that make up
   enumArrangement
}
const
	pNoArrangement = FourCharCode('narr');
	pSnapToGridArrangement = FourCharCode('grda');
	pByNameArrangement = FourCharCode('nama');
	pByModificationDateArrangement = FourCharCode('mdta');
	pByCreationDateArrangement = FourCharCode('cdta');
	pBySizeArrangement = FourCharCode('siza');
	pByKindArrangement = FourCharCode('kina');
	pByLabelArrangement = FourCharCode('laba');

{  #define pObject                                 cObject         // defined above}

{ Properties of class cProcess (subclass of cObject)}
const
{    pName                      = 'pnam',         // defined in AERegistry.r}
	pFile = cFile; {    pCreatorType                = 'fcrt',      // defined above}
                                        {    pFileType                    = 'asty',        // defined above}
                                        {    pIsFrontProcess                   = 'pisf',         // defined in AERegistry.r}
                                        {    pAppPartition                 = 'appt',       // defined above}
	pPartitionSpaceUsed = FourCharCode('pusd'); {    pIsScriptable                 = 'isab',       // defined in AERegistry.r}
                                        {    pVisible                    = 'pvis'      // defined in AERegistry.r}
	pLocalAndRemoteEvents = FourCharCode('revt');
	pHasScriptingTerminology = FourCharCode('hscr');

{ Properties of class cAccessoryProcess (subclass of cProcess)}
const
	pDeskAccessoryFile = cDeskAccessoryFile;

{ Properties of class cApplicationProcess (subclass of cProcess)}
const
	pApplicationFile = cApplicationFile;

{
   Properties of class cGroup (subclass of cObject)
  enum (
    pBounds
    pIconBitmap
    pLabelIndex
    pName
    pPosition
    pWindow                                 = cWindow           // defined above
  );
}

{ Properties of class cUser (subclass of cObject)}
const
{    pBounds}
                                        {    pIconBitmap}
                                        {    pLabelIndex}
                                        {    pName}
                                        {    pPosition}
                                        {    pWindow                        = cWindow,        // defined above}
	pCanConnect = FourCharCode('ccon');
	pCanChangePassword = FourCharCode('ccpw');
	pCanDoProgramLinking = FourCharCode('ciac');
	pIsOwner = FourCharCode('isow');
	pARADialIn = FourCharCode('arad');
	pShouldCallBack = FourCharCode('calb');
	pCallBackNumber = FourCharCode('cbnm');

{
   Properties of class cApplication (subclass of cObject)
   NOTE: properties for the special folders must match their respective kXXXFolderType constants
}
const
	pAboutMacintosh = FourCharCode('abbx');
	pAppleMenuItemsFolder = FourCharCode('amnu'); { kAppleMenuFolderType}
                                        {    pClipboard                    = 'pcli',         // defined in AERegistry.r}
	pControlPanelsFolder = FourCharCode('ctrl'); { kControlPanelFolderType}
	pDesktop = FourCharCode('desk'); { kDesktopFolderType}
	pExtensionsFolder = FourCharCode('extn'); { kExtensionFolderType}
                                        {    pFileShareOn                = 'fshr',      // defined above}
	pFinderPreferences = FourCharCode('pfrp');
	pFontsFolder = FourCharCode('font');
	pFontsFolderPreAllegro = FourCharCode('ffnt'); { DO NOT USE THIS - FOR BACKWARDS COMPAT ONLY}
                                        {    pIsFrontProcess                   = 'pisf',         // defined in AERegistry.r}
                                        {    pInsertionLoc                 = 'pins',       // defined in AERegistry.r}
	pLargestFreeBlock = FourCharCode('mfre');
	pPreferencesFolder = FourCharCode('pref'); { kPreferencesFolderType}
                                        {    pProductVersion                   = 'ver2',         // defined above}
                                        {    pUserSelection                  = 'pusl',        // defined in AERegistry.r}
                                        {    pFileShareStartingUp             = 'fsup',        // defined above}
	pShortCuts = FourCharCode('scut');
	pShutdownFolder = FourCharCode('shdf');
	pStartupItemsFolder = FourCharCode('strt'); { kStartupFolderType}
	pSystemFolder = FourCharCode('macs'); { kSystemFolderType}
	pTemporaryFolder = FourCharCode('temp'); { kTemporaryFolderType}
                                        {    pVersion                    = 'vers',       // defined in AERegistry.r}
	pViewPreferences = FourCharCode('pvwp'); {    pVisible                    = 'pvis',       // defined in AERegistry.r}
	pStartingUp = FourCharCode('awak'); { private property to tell whether the Finder is fully up and running}

{ Properties of class cSharingPrivileges (subclass of cObject)}
const
	pSeeFiles = FourCharCode('prvr');
	pSeeFolders = FourCharCode('prvs');
	pMakeChanges = FourCharCode('prvw');

{
   Properties of class cPreferences (subclass of cObject)
  enum (
    pShowFolderSize                         = 'sfsz',           // defined above for Views CP
    pShowComment                            = 'scom',           // defined above for Views CP
    pShowModificationDate                   = pShowDate,            // pShowDate defined above for Views CP
    pShowKind                               = 'sknd',           // defined above for Views CP
    pShowLabel                              = 'slbl',           // defined above for Views CP
    pShowSize                               = 'ssiz',           // defined above for Views CP
    pShowVersion                            = 'svrs',           // defined above for Views CP
    pShowCreationDate                       = 'scda',           // defined in cPreferencesWindow
    pShowFileType                           = 'sfty',           // defined in cPreferencesWindow
    pShowFileCreator                        = 'sfcr',           // defined in cPreferencesWindow
    pListViewIconSize                       = 'lvis',           // defined above for Views CP
    pGridIcons                              = 'fgrd',           // defined above for Views CP
    pStaggerIcons                           = 'fstg',           // defined above for Views CP
    pViewFont                               = 'vfnt',           // defined above for Views CP
    pViewFontSize                           = 'vfsz',           // defined above for Views CP
    pUseRelativeDate                        = 'urdt',           // defined in cPreferencesWindow
    pDelayBeforeSpringing                   = 'dela',           // defined in cPreferencesWindow
    pShowMacOSFolder                        = 'sosf',           // defined in cPreferencesWindow
    pUseShortMenus                          = 'usme',           // defined in cPreferencesWindow
    pUseCustomNewMenu                       = 'ucnm',           // defined in cPreferencesWindow
    pShowDesktopInBackground                = 'sdtb',           // defined in cPreferencesWindow
    pActivateDesktopOnClick                 = 'adtc',           // defined in cPreferencesWindow
    pLabel1                                 = 'lbl1',           // defined in cPreferencesWindow
    pLabel2                                 = 'lbl2',           // defined in cPreferencesWindow
    pLabel3                                 = 'lbl3',           // defined in cPreferencesWindow
    pLabel4                                 = 'lbl4',           // defined in cPreferencesWindow
    pLabel5                                 = 'lbl5',           // defined in cPreferencesWindow
    pLabel6                                 = 'lbl6',           // defined in cPreferencesWindow
    pLabel7                                 = 'lbl7',           // defined in cPreferencesWindow
    pWindow                                 = cWindow           // defined above
  );
}

{
   Properties of class cLabel (subclass of cObject)
  enum (
    pName                                   = 'pnam',           // defined in AERegistry.r
    pColor                                  = 'colr',           // defined in AERegistry.r
  );
}

{ Misc Properties}
const
	pSmallIcon = FourCharCode('smic');
	pSmallButton = FourCharCode('smbu');
	pLargeButton = FourCharCode('lgbu');
	pGrid = FourCharCode('grid');

{
  //////////////////////////////////////
   Enumerations defined by the Finder
  //////////////////////////////////////
}

const
	enumViewBy = FourCharCode('vwby');
	enumGestalt = FourCharCode('gsen');
	enumConflicts = FourCharCode('cflc');
	enumExistingItems = FourCharCode('exsi');
	enumOlderItems = FourCharCode('oldr');

const
	enumDate = FourCharCode('enda');
	enumAnyDate = FourCharCode('anyd');
	enumToday = FourCharCode('tday');
	enumYesterday = FourCharCode('yday');
	enumThisWeek = FourCharCode('twek');
	enumLastWeek = FourCharCode('lwek');
	enumThisMonth = FourCharCode('tmon');
	enumLastMonth = FourCharCode('lmon');
	enumThisYear = FourCharCode('tyer');
	enumLastYear = FourCharCode('lyer');
	enumBeforeDate = FourCharCode('bfdt');
	enumAfterDate = FourCharCode('afdt');
	enumBetweenDate = FourCharCode('btdt');
	enumOnDate = FourCharCode('ondt');

const
	enumAllDocuments = FourCharCode('alld');
	enumFolders = FourCharCode('fold');
	enumAliases = FourCharCode('alia');
	enumStationery = FourCharCode('stat');

const
	enumWhere = FourCharCode('wher');
	enumAllLocalDisks = FourCharCode('aldk');
	enumAllRemoteDisks = FourCharCode('ardk');
	enumAllDisks = FourCharCode('alld');
	enumAllOpenFolders = FourCharCode('aofo');


const
	enumIconSize = FourCharCode('isiz');
	enumSmallIconSize = pSmallIcon;
	enumMiniIconSize = FourCharCode('miic');
	enumLargeIconSize = FourCharCode('lgic');

const
	enumSortDirection = FourCharCode('sodr');
	enumSortDirectionNormal = FourCharCode('snrm');
	enumSortDirectionReverse = FourCharCode('srvs');

const
	enumArrangement = FourCharCode('earr');

{ Get Info Window panel enumeration}
const
	enumInfoWindowPanel = FourCharCode('ipnl');
	enumGeneralPanel = FourCharCode('gpnl');
	enumSharingPanel = FourCharCode('spnl');
	enumStatusNConfigPanel = FourCharCode('scnl');
	enumFontsPanel = FourCharCode('fpnl');
	enumMemoryPanel = FourCharCode('mpnl');


{ Preferences panel enumeration}
const
	enumPrefsWindowPanel = FourCharCode('pple');
	enumPrefsGeneralPanel = FourCharCode('pgnp');
	enumPrefsLabelPanel = FourCharCode('plbp');
	enumPrefsIconViewPanel = FourCharCode('pivp');
	enumPrefsButtonViewPanel = FourCharCode('pbvp');
	enumPrefsListViewPanel = FourCharCode('plvp');

{
  //////////////////////////////////////
   Types defined by the Finder
  //////////////////////////////////////
}

const
	typeIconFamily = cIconFamily; { An AEList of typeIconAndMask, type8BitIcon, & c.}
	typeIconAndMask = FourCharCode('ICN#');
	type8BitMask = FourCharCode('l8mk');
	type32BitIcon = FourCharCode('il32');
	type8BitIcon = FourCharCode('icl8');
	type4BitIcon = FourCharCode('icl4');
	typeSmallIconAndMask = FourCharCode('ics#');
	typeSmall8BitMask = FourCharCode('s8mk');
	typeSmall32BitIcon = FourCharCode('is32');
	typeSmall8BitIcon = FourCharCode('ics8');
	typeSmall4BitIcon = FourCharCode('ics4');
	typeRelativeTime = FourCharCode('rtim');
	typeConceptualTime = FourCharCode('timc');

{
  //////////////////////////////////////
   Keywords defined by the Finder
  //////////////////////////////////////
}

const
	keyIconAndMask = FourCharCode('ICN#');
	key32BitIcon = FourCharCode('il32');
	key8BitIcon = FourCharCode('icl8');
	key4BitIcon = FourCharCode('icl4');
	key8BitMask = FourCharCode('l8mk');
	keySmallIconAndMask = FourCharCode('ics#');
	keySmall8BitIcon = FourCharCode('ics8');
	keySmall4BitIcon = FourCharCode('ics4');
	keySmall32BitIcon = FourCharCode('is32');
	keySmall8BitMask = FourCharCode('s8mk');
	keyMini1BitMask = FourCharCode('icm#');
	keyMini4BitIcon = FourCharCode('icm4');
	keyMini8BitIcon = FourCharCode('icm8');
	keyAEUsing = FourCharCode('usin');
	keyAEReplacing = FourCharCode('alrp');
	keyAENoAutoRouting = FourCharCode('rout');
	keyLocalPositionList = FourCharCode('mvpl');
	keyGlobalPositionList = FourCharCode('mvpg');
	keyRedirectedDocumentList = FourCharCode('fpdl');

{
  //////////////////////////////////////
   New prepositions used by the Finder
  //////////////////////////////////////
}

const
	keyASPrepositionHas = FourCharCode('has ');
	keyAll = FourCharCode('kyal');
	keyOldFinderItems = FourCharCode('fsel');

{
  //////////////////////////////////////
   New key forms used by the Finder
  //////////////////////////////////////
}

const
	formAlias = typeAlias;
	formCreator = pFileCreator;


{
  //////////////////////////////////////
   Finder error codes
  //////////////////////////////////////
}

const
	errFinderIsBusy = -15260;
	errFinderWindowNotOpen = -15261;
	errFinderCannotPutAway = -15262;
	errFinderWindowMustBeIconView = -15263; { RequireWindowInIconView}
	errFinderWindowMustBeListView = -15264; { RequireWindowInListView}
	errFinderCantMoveToDestination = -15265;
	errFinderCantMoveSource = -15266;
	errFinderCantOverwrite = -15267;
	errFinderIncestuousMove = -15268; { Could just use errFinderCantMoveSource}
	errFinderCantMoveToAncestor = -15269; { Could also use errFinderCantMoveSource}
	errFinderCantUseTrashedItems = -15270;
	errFinderItemAlreadyInDest = -15271; { Move from folder A to folder A}
	errFinderUnknownUser = -15272; { Includes unknown group}
	errFinderSharePointsCantInherit = -15273;
	errFinderWindowWrongType = -15274;
	errFinderPropertyNowWindowBased = -15275;
	errFinderAppFolderProtected = -15276; { used by General controls when folder protection is on}
	errFinderSysFolderProtected = -15277; { used by General controls when folder protection is on}
	errFinderBoundsWrong = -15278;
	errAEValueOutOfRange = -15279;
	errFinderPropertyDoesNotApply = -15280;
	errFinderFileSharingMustBeOn = -15281;
	errFinderMustBeActive = -15282;
	errFinderVolumeNotFound = -15283; { more descriptive than what we get with nsvErr}
	errFinderLockedItemsInTrash = -15284; { there are some locked items in the trash}
	errFinderOnlyLockedItemsInTrash = -15285; { all the items (except folders) in the trash are locked}
	errFinderProgramLinkingMustBeOn = -15286;
	errFinderWindowMustBeButtonView = -15287;
	errFinderBadPackageContents = -15288; { something is wrong within the package   }
	errFinderUnsupportedInsidePackages = -15289; { operation cannot be used on items within a package     }
	errFinderCorruptOpenFolderList = -15290; { was -15276 in Finder 8.6 and earlier, but that conflicted with General Controls}
	errFinderNoInvisibleFiles = -15291; { was -15277 in Finder 8.6 and earlier, but that conflicted with General Controls}
	errFinderCantDeleteImmediately = -15292; { cannot delete immediately via scripting}
	errFinderLastReserved = -15379;

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
