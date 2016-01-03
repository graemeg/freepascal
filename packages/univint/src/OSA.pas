{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       OpenScripting/OSA.h
 
     Contains:   Open Scripting Architecture Client Interfaces.
 
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
 
     Copyright:  © 1992-2008 by Apple Inc., all rights reserved
=======
=======
>>>>>>> origin/fixes_2_2
     File:       OSA.p
=======
     File:       OpenScripting/OSA.h
>>>>>>> origin/fixes_2.4
 
     Contains:   Open Scripting Architecture Client Interfaces.
 
     Version:    OSA-136~14
 
<<<<<<< HEAD
     Copyright:  © 1992-2002 by Apple Computer, Inc., all rights reserved
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  © 1992-2008 by Apple Inc., all rights reserved
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
=======
>>>>>>> origin/cpstrnew
{ Pascal Translation Updated: Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }

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

unit OSA;
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
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
{$elsec}
=======
=======
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
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
uses MacTypes,AEDataModel,MacErrors,AppleEvents,AEObjects,AEInteraction,Components,Files,CFBase,CFURL,CFAttributedString;
{$endc} {not MACOSALLINCLUDE}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{$ifc TARGET_OS_MAC}

=======

{$ifc TARGET_OS_MAC}

>>>>>>> graemeg/cpstrnew
=======

{$ifc TARGET_OS_MAC}

>>>>>>> graemeg/cpstrnew
=======

{$ifc TARGET_OS_MAC}

>>>>>>> origin/cpstrnew
{$ALIGN POWER}

=======
=======
>>>>>>> origin/fixes_2_2
uses MacTypes,AEDataModel,MacErrors,AppleEvents,AEObjects,AEInteraction,Components;
=======
uses MacTypes,AEDataModel,MacErrors,AppleEvents,AEObjects,AEInteraction,Components,Files,CFBase,CFURL,CFAttributedString;
{$endc} {not MACOSALLINCLUDE}

>>>>>>> origin/fixes_2.4

{$ifc TARGET_OS_MAC}

{$ALIGN POWER}

<<<<<<< HEAD
{$ALIGN MAC68K}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{*************************************************************************
    Types and Constants
*************************************************************************}

{    The componenent manager type code for components that
        support the OSA interface defined here. }
{ 0x6f736120 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
const
	kOSAComponentType = FourCharCode('osa ');

{ 0x73637074 }
const
	kOSAGenericScriptingComponentSubtype = FourCharCode('scpt');

{  Type of script document files.  }
{ 0x6f736173 }
const
	kOSAFileType = FourCharCode('osas');

{
        Suite and event code of the RecordedText event. 
        (See OSAStartRecording, below.)
    }
{ 0x61736372 }
const
	kOSASuite = FourCharCode('ascr');

{ 0x72656364 }
const
	kOSARecordedText = FourCharCode('recd');

{ Selector returns boolean }
{ 0x6d6f6469 }
const
	kOSAScriptIsModified = FourCharCode('modi');

{ Selector returns boolean }
{ 0x63736372 }
const
	kOSAScriptIsTypeCompiledScript = FourCharCode('cscr');

{ Selector returns boolean }
{ 0x76616c75 }
const
	kOSAScriptIsTypeScriptValue = FourCharCode('valu');

{ Selector returns boolean }
{ 0x636e7478 }
const
	kOSAScriptIsTypeScriptContext = FourCharCode('cntx');

{ Selector returns a DescType which may be passed to OSACoerceToDesc }
{ 0x62657374 }
const
	kOSAScriptBestType = FourCharCode('best');

{
        This selector is used to determine whether a script has source 
        associated with it that when given to OSAGetSource, the call will not
        fail.  The selector returns a boolean.
    }
{ 0x67737263 }
const
	kOSACanGetSource = FourCharCode('gsrc');


const
	typeOSADialectInfo = FourCharCode('difo'); {  0x6469666f   }
	keyOSADialectName = FourCharCode('dnam'); {  0x646e616d   }
	keyOSADialectCode = FourCharCode('dcod'); {  0x64636f64   }
	keyOSADialectLangCode = FourCharCode('dlcd'); {  0x646c6364   }
	keyOSADialectScriptCode = FourCharCode('dscd'); {  0x64736364   }

type
	OSAError = ComponentResult;
{ Under the Open Scripting Architecture all error results are longs }
type
	OSAID = UInt32;
{
        OSAIDs allow transparent manipulation of scripts associated with
        various scripting systems.
    }
<<<<<<< HEAD
const
	kOSANullScript = 0;

{ No -script constant. }
const
	kOSANullMode = 0;    { sounds better }
	kOSAModeNull = 0;     { tastes consistent }

{
        Some routines take flags that control their execution.  This constant
        declares default mode settings are used.
    }
type
	OSACreateAppleEventProcPtr = function( theAEEventClass: AEEventClass; theAEEventID: AEEventID; const (*var*) target: AEAddressDesc; returnID: SInt16; transactionID: SInt32; var result: AppleEvent; refCon: SRefCon ): OSErr;
	OSASendProcPtr = function( const (*var*) theAppleEvent: AppleEvent; var reply: AppleEvent; sendMode: AESendMode; sendPriority: AESendPriority; timeOutInTicks: SInt32; idleProc: AEIdleUPP; filterProc: AEFilterUPP; refCon: SRefCon ): OSErr;
	OSACreateAppleEventUPP = OSACreateAppleEventProcPtr;
	OSASendUPP = OSASendProcPtr;
<<<<<<< HEAD

{
 *  NewOSACreateAppleEventUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewOSACreateAppleEventUPP( userRoutine: OSACreateAppleEventProcPtr ): OSACreateAppleEventUPP; external name '_NewOSACreateAppleEventUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======

{
 *  NewOSACreateAppleEventUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewOSACreateAppleEventUPP( userRoutine: OSACreateAppleEventProcPtr ): OSACreateAppleEventUPP; external name '_NewOSACreateAppleEventUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
const
	kOSANullScript = 0;

{ No -script constant. }
const
	kOSANullMode = 0;    { sounds better }
	kOSAModeNull = 0;     { tastes consistent }

{
        Some routines take flags that control their execution.  This constant
        declares default mode settings are used.
    }
type
	OSACreateAppleEventProcPtr = function( theAEEventClass: AEEventClass; theAEEventID: AEEventID; const (*var*) target: AEAddressDesc; returnID: SInt16; transactionID: SInt32; var result: AppleEvent; refCon: SRefCon ): OSErr;
	OSASendProcPtr = function( const (*var*) theAppleEvent: AppleEvent; var reply: AppleEvent; sendMode: AESendMode; sendPriority: AESendPriority; timeOutInTicks: SInt32; idleProc: AEIdleUPP; filterProc: AEFilterUPP; refCon: SRefCon ): OSErr;
	OSACreateAppleEventUPP = OSACreateAppleEventProcPtr;
	OSASendUPP = OSASendProcPtr;

{
 *  NewOSACreateAppleEventUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewOSACreateAppleEventUPP( userRoutine: OSACreateAppleEventProcPtr ): OSACreateAppleEventUPP; external name '_NewOSACreateAppleEventUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2

=======
>>>>>>> origin/fixes_2.4
const
	kOSAComponentType = FourCharCode('osa ');

{ 0x73637074 }
const
	kOSAGenericScriptingComponentSubtype = FourCharCode('scpt');

{  Type of script document files.  }
{ 0x6f736173 }
const
	kOSAFileType = FourCharCode('osas');

{
        Suite and event code of the RecordedText event. 
        (See OSAStartRecording, below.)
    }
{ 0x61736372 }
const
	kOSASuite = FourCharCode('ascr');

{ 0x72656364 }
const
	kOSARecordedText = FourCharCode('recd');

{ Selector returns boolean }
{ 0x6d6f6469 }
const
	kOSAScriptIsModified = FourCharCode('modi');

{ Selector returns boolean }
{ 0x63736372 }
const
	kOSAScriptIsTypeCompiledScript = FourCharCode('cscr');

{ Selector returns boolean }
{ 0x76616c75 }
const
	kOSAScriptIsTypeScriptValue = FourCharCode('valu');

{ Selector returns boolean }
{ 0x636e7478 }
const
	kOSAScriptIsTypeScriptContext = FourCharCode('cntx');

{ Selector returns a DescType which may be passed to OSACoerceToDesc }
{ 0x62657374 }
const
	kOSAScriptBestType = FourCharCode('best');

{
        This selector is used to determine whether a script has source 
        associated with it that when given to OSAGetSource, the call will not
        fail.  The selector returns a boolean.
    }
{ 0x67737263 }
const
	kOSACanGetSource = FourCharCode('gsrc');


const
	typeOSADialectInfo = FourCharCode('difo'); {  0x6469666f   }
	keyOSADialectName = FourCharCode('dnam'); {  0x646e616d   }
	keyOSADialectCode = FourCharCode('dcod'); {  0x64636f64   }
	keyOSADialectLangCode = FourCharCode('dlcd'); {  0x646c6364   }
	keyOSADialectScriptCode = FourCharCode('dscd'); {  0x64736364   }

type
	OSAError = ComponentResult;
{ Under the Open Scripting Architecture all error results are longs }
type
	OSAID = UInt32;
{
        OSAIDs allow transparent manipulation of scripts associated with
        various scripting systems.
    }
const
	kOSANullScript = 0;

{ No -script constant. }
const
	kOSANullMode = 0;    { sounds better }
	kOSAModeNull = 0;     { tastes consistent }

{
        Some routines take flags that control their execution.  This constant
        declares default mode settings are used.
    }
type
	OSACreateAppleEventProcPtr = function( theAEEventClass: AEEventClass; theAEEventID: AEEventID; const (*var*) target: AEAddressDesc; returnID: SInt16; transactionID: SInt32; var result: AppleEvent; refCon: SRefCon ): OSErr;
	OSASendProcPtr = function( const (*var*) theAppleEvent: AppleEvent; var reply: AppleEvent; sendMode: AESendMode; sendPriority: AESendPriority; timeOutInTicks: SInt32; idleProc: AEIdleUPP; filterProc: AEFilterUPP; refCon: SRefCon ): OSErr;
	OSACreateAppleEventUPP = OSACreateAppleEventProcPtr;
	OSASendUPP = OSASendProcPtr;

{
 *  NewOSACreateAppleEventUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewOSACreateAppleEventUPP( userRoutine: OSACreateAppleEventProcPtr ): OSACreateAppleEventUPP; external name '_NewOSACreateAppleEventUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

<<<<<<< HEAD
const
	uppOSACreateAppleEventProcInfo = $000FEFE0;
	uppOSASendProcInfo = $003FEFE0;
	{
	 *  NewOSACreateAppleEventUPP()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   available as macro/inline
	 *    CarbonLib:        in CarbonLib 1.0 and later
	 *    Mac OS X:         in version 10.0 and later
	 	}
function NewOSACreateAppleEventUPP(userRoutine: OSACreateAppleEventProcPtr): OSACreateAppleEventUPP; external name '_NewOSACreateAppleEventUPP'; { old name was NewOSACreateAppleEventProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{
 *  NewOSASendUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewOSASendUPP( userRoutine: OSASendProcPtr ): OSASendUPP; external name '_NewOSASendUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function NewOSASendUPP(userRoutine: OSASendProcPtr): OSASendUPP; external name '_NewOSASendUPP'; { old name was NewOSASendProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function NewOSASendUPP( userRoutine: OSASendProcPtr ): OSASendUPP; external name '_NewOSASendUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  DisposeOSACreateAppleEventUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeOSACreateAppleEventUPP( userUPP: OSACreateAppleEventUPP ); external name '_DisposeOSACreateAppleEventUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure DisposeOSACreateAppleEventUPP(userUPP: OSACreateAppleEventUPP); external name '_DisposeOSACreateAppleEventUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeOSACreateAppleEventUPP( userUPP: OSACreateAppleEventUPP ); external name '_DisposeOSACreateAppleEventUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  DisposeOSASendUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeOSASendUPP( userUPP: OSASendUPP ); external name '_DisposeOSASendUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure DisposeOSASendUPP(userUPP: OSASendUPP); external name '_DisposeOSASendUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeOSASendUPP( userUPP: OSASendUPP ); external name '_DisposeOSASendUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  InvokeOSACreateAppleEventUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeOSACreateAppleEventUPP( theAEEventClass: AEEventClass; theAEEventID: AEEventID; const (*var*) target: AEAddressDesc; returnID: SInt16; transactionID: SInt32; var result: AppleEvent; refCon: SRefCon; userUPP: OSACreateAppleEventUPP ): OSErr; external name '_InvokeOSACreateAppleEventUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function InvokeOSACreateAppleEventUPP(theAEEventClass: AEEventClass; theAEEventID: AEEventID; const (*var*) target: AEAddressDesc; returnID: SInt16; transactionID: SInt32; var result: AppleEvent; refCon: SInt32; userRoutine: OSACreateAppleEventUPP): OSErr; external name '_InvokeOSACreateAppleEventUPP'; { old name was CallOSACreateAppleEventProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function InvokeOSACreateAppleEventUPP( theAEEventClass: AEEventClass; theAEEventID: AEEventID; const (*var*) target: AEAddressDesc; returnID: SInt16; transactionID: SInt32; var result: AppleEvent; refCon: SRefCon; userUPP: OSACreateAppleEventUPP ): OSErr; external name '_InvokeOSACreateAppleEventUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  InvokeOSASendUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeOSASendUPP( const (*var*) theAppleEvent: AppleEvent; var reply: AppleEvent; sendMode: AESendMode; sendPriority: AESendPriority; timeOutInTicks: SInt32; idleProc: AEIdleUPP; filterProc: AEFilterUPP; refCon: SRefCon; userUPP: OSASendUPP ): OSErr; external name '_InvokeOSASendUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function InvokeOSASendUPP(const (*var*) theAppleEvent: AppleEvent; var reply: AppleEvent; sendMode: AESendMode; sendPriority: AESendPriority; timeOutInTicks: SInt32; idleProc: AEIdleUPP; filterProc: AEFilterUPP; refCon: SInt32; userRoutine: OSASendUPP): OSErr; external name '_InvokeOSASendUPP'; { old name was CallOSASendProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function InvokeOSASendUPP( const (*var*) theAppleEvent: AppleEvent; var reply: AppleEvent; sendMode: AESendMode; sendPriority: AESendPriority; timeOutInTicks: SInt32; idleProc: AEIdleUPP; filterProc: AEFilterUPP; refCon: SRefCon; userUPP: OSASendUPP ): OSErr; external name '_InvokeOSASendUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{*************************************************************************
    OSA Interface Descriptions
**************************************************************************
    The OSA Interface is broken down into a required interface, and several
    optional interfaces to support additional functionality.  A given scripting
    component may choose to support only some of the optional interfaces in
    addition to the basic interface.  The OSA Component Flags may be used to 
    query the Component Manager to find a scripting component with a particular
    capability, or determine if a particular scripting component supports a 
    particular capability.
*************************************************************************}
{ OSA Component Flags: }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
const
	kOSASupportsCompiling = $0002;
	kOSASupportsGetSource = $0004;
	kOSASupportsAECoercion = $0008;
	kOSASupportsAESending = $0010;
	kOSASupportsRecording = $0020;
	kOSASupportsConvenience = $0040;
	kOSASupportsDialects = $0080;
	kOSASupportsEventHandling = $0100;

{ Component Selectors: }
const
	kOSASelectLoad = $0001;
	kOSASelectStore = $0002;
	kOSASelectExecute = $0003;
	kOSASelectDisplay = $0004;
	kOSASelectScriptError = $0005;
	kOSASelectDispose = $0006;
	kOSASelectSetScriptInfo = $0007;
	kOSASelectGetScriptInfo = $0008;
	kOSASelectSetActiveProc = $0009;
	kOSASelectGetActiveProc = $000A;
	kOSASelectCopyDisplayString = $000B;

{ Compiling: }
const
	kOSASelectScriptingComponentName = $0102;
	kOSASelectCompile = $0103;
	kOSASelectCopyID = $0104;
	kOSASelectCopyScript = $0105;

{ GetSource: }
const
	kOSASelectGetSource = $0201;
	kOSASelectCopySourceString = $0202;

{ AECoercion: }
const
	kOSASelectCoerceFromDesc = $0301;
	kOSASelectCoerceToDesc = $0302;

{ AESending: }
const
	kOSASelectSetSendProc = $0401;
	kOSASelectGetSendProc = $0402;
	kOSASelectSetCreateProc = $0403;
	kOSASelectGetCreateProc = $0404;
	kOSASelectSetDefaultTarget = $0405;

{ Recording: }
const
	kOSASelectStartRecording = $0501;
	kOSASelectStopRecording = $0502;

{ Convenience: }
const
	kOSASelectLoadExecute = $0601;
	kOSASelectCompileExecute = $0602;
	kOSASelectDoScript = $0603;

{ Dialects: }
const
	kOSASelectSetCurrentDialect = $0701;
	kOSASelectGetCurrentDialect = $0702;
	kOSASelectAvailableDialects = $0703;
	kOSASelectGetDialectInfo = $0704;
	kOSASelectAvailableDialectCodeList = $0705;

{ Event Handling: }
const
	kOSASelectSetResumeDispatchProc = $0801;
	kOSASelectGetResumeDispatchProc = $0802;
	kOSASelectExecuteEvent = $0803;
	kOSASelectDoEvent = $0804;
	kOSASelectMakeContext = $0805;


{ scripting component specific selectors are added beginning with this value  }
const
	kOSASelectComponentSpecificStart = $1001;


{        Mode Flags:

    Warning: These should not conflict with the AESend mode flags in
    AppleEvents.h, because we may want to use them as OSA mode flags too.
}

{
        This mode flag may be passed to OSALoad, OSAStore or OSACompile to
        instruct the scripting component to not retain the "source" of an
        expression.  This will cause the OSAGetSource call to return the error
        errOSASourceNotAvailable if used.  However, some scripting components
        may not retain the source anyway.  This is mainly used when either space
        efficiency is desired, or a script is to be "locked" so that its
        implementation may not be viewed.
    }
const
	kOSAModePreventGetSource = $00000001;

{
        These mode flags may be passed to OSACompile, OSAExecute, OSALoadExecute
        OSACompileExecute, OSADoScript, OSAExecuteEvent, or OSADoEvent to
        indicate whether or not the script may interact with the user, switch
        layer or reconnect if necessary.  Any AppleEvents will be sent with the
        corresponding AESend mode supplied.
    }
const
	kOSAModeNeverInteract = kAENeverInteract;
	kOSAModeCanInteract = kAECanInteract;
	kOSAModeAlwaysInteract = kAEAlwaysInteract;
	kOSAModeDontReconnect = kAEDontReconnect;

{
        This mode flag may be passed to OSACompile, OSAExecute, OSALoadExecute
        OSACompileExecute, OSADoScript, OSAExecuteEvent, or OSADoEvent to
        indicate whether or not AppleEvents should be sent with the
        kAECanSwitchLayer mode flag sent or not. NOTE: This flag is exactly the
        opposite sense of the AppleEvent flag kAECanSwitchLayer.  This is to
        provide a more convenient default, i.e. not supplying any mode
        (kOSAModeNull) means to send events with kAECanSwitchLayer.  Supplying
        the kOSAModeCantSwitchLayer mode flag will cause AESend to be called
        without kAECanSwitchLayer.
    }
const
	kOSAModeCantSwitchLayer = $00000040;

{
        This mode flag may be passed to OSACompile, OSAExecute, OSALoadExecute
        OSACompileExecute, OSADoScript, OSAExecuteEvent, or OSADoEvent to
        indicate whether or not AppleEvents should be sent with the kAEDontRecord
        mode flag sent or not. NOTE: This flag is exactly the opposite sense of
        the AppleEvent flag kAEDontRecord.  This is to provide a more convenient
        default, i.e. not supplying any mode (kOSAModeNull) means to send events
        with kAEDontRecord.  Supplying the kOSAModeDoRecord mode flag will 
        cause AESend to be called without kAEDontRecord.
    }
const
	kOSAModeDoRecord = $00001000;

{
        This is a mode flag for OSACompile that indicates that a context should
        be created as the result of compilation. All handler definitions are
        inserted into the new context, and variables are initialized by
        evaluating their initial values in a null context (i.e. they must be
        constant expressions).
    }
const
	kOSAModeCompileIntoContext = $00000002;

{
        This is a mode flag for OSACompile that indicates that the previous
        script ID (input to OSACompile) should be augmented with any new
        definitions in the sourceData rather than replaced with a new script.
        This means that the previous script ID must designate a context.
        The presence of this flag causes the kOSAModeCompileIntoContext flag
        to be implicitly used, causing any new definitions to be initialized
        in a null context.
    }
const
	kOSAModeAugmentContext = $00000004;

{
        This mode flag may be passed to OSADisplay or OSADoScript to indicate
        that output only need be human-readable, not re-compilable by OSACompile.
        If used, output may be arbitrarily "beautified", e.g. quotes may be left
        off of string values, long lists may have elipses, etc.
    }
const
	kOSAModeDisplayForHumans = $00000008;

{
        This mode flag may be passed to OSAStore in the case where the scriptID
        is a context.  This causes the context to be saved, but not the context's
        parent context.  When the stored context is loaded back in, the parent
        will be kOSANullScript.
    }
const
	kOSAModeDontStoreParent = $00010000;

{
        This mode flag may be passed to OSAExecuteEvent to cause the event to
        be dispatched to the direct object of the event. The direct object (or
        subject attribute if the direct object is a non-object specifier) will
        be resolved, and the resulting script object will be the recipient of
        the message. The context argument to OSAExecuteEvent will serve as the
        root of the lookup/resolution process.
    }
const
	kOSAModeDispatchToDirectObject = $00020000;

{
        This mode flag may be passed to OSAExecuteEvent to indicate that
        components do not have to get the data of object specifier arguments.
    }
const
	kOSAModeDontGetDataForArguments = $00040000;

{
        This mode flag may be passed to OSACoerceToDesc to indicate that
        the resulting descriptor should be fully qualified (i.e. should
        include the root application reference).
    }
const
	kOSAModeFullyQualifyDescriptors = $00080000;

{*************************************************************************
    OSA Basic Scripting Interface
**************************************************************************
    Scripting components must at least support the Basic Scripting interface.
*************************************************************************}
{        Loading and Storing Scripts:

    These routines allow scripts to be loaded and stored in their internal
    (possibly compiled, non-text) representation.
}

{ Resource type for scripts }
const
	kOSAScriptResourceType = kOSAGenericScriptingComponentSubtype;

{
        Default type given to OSAStore which creates "generic" loadable script
        data descriptors.
    }
const
	typeOSAGenericStorage = kOSAScriptResourceType;

{
 *  OSALoad()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
function OSALoad( scriptingComponent: ComponentInstance; const (*var*) scriptData: AEDesc; modeFlags: SInt32; var resultingScriptID: OSAID ): OSAError; external name '_OSALoad';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2

=======
>>>>>>> origin/fixes_2.4
const
	kOSASupportsCompiling = $0002;
	kOSASupportsGetSource = $0004;
	kOSASupportsAECoercion = $0008;
	kOSASupportsAESending = $0010;
	kOSASupportsRecording = $0020;
	kOSASupportsConvenience = $0040;
	kOSASupportsDialects = $0080;
	kOSASupportsEventHandling = $0100;

{ Component Selectors: }
const
	kOSASelectLoad = $0001;
	kOSASelectStore = $0002;
	kOSASelectExecute = $0003;
	kOSASelectDisplay = $0004;
	kOSASelectScriptError = $0005;
	kOSASelectDispose = $0006;
	kOSASelectSetScriptInfo = $0007;
	kOSASelectGetScriptInfo = $0008;
	kOSASelectSetActiveProc = $0009;
	kOSASelectGetActiveProc = $000A;
	kOSASelectCopyDisplayString = $000B;

{ Compiling: }
const
	kOSASelectScriptingComponentName = $0102;
	kOSASelectCompile = $0103;
	kOSASelectCopyID = $0104;
	kOSASelectCopyScript = $0105;

{ GetSource: }
const
	kOSASelectGetSource = $0201;
	kOSASelectCopySourceString = $0202;

{ AECoercion: }
const
	kOSASelectCoerceFromDesc = $0301;
	kOSASelectCoerceToDesc = $0302;

{ AESending: }
const
	kOSASelectSetSendProc = $0401;
	kOSASelectGetSendProc = $0402;
	kOSASelectSetCreateProc = $0403;
	kOSASelectGetCreateProc = $0404;
	kOSASelectSetDefaultTarget = $0405;

{ Recording: }
const
	kOSASelectStartRecording = $0501;
	kOSASelectStopRecording = $0502;

{ Convenience: }
const
	kOSASelectLoadExecute = $0601;
	kOSASelectCompileExecute = $0602;
	kOSASelectDoScript = $0603;

{ Dialects: }
const
	kOSASelectSetCurrentDialect = $0701;
	kOSASelectGetCurrentDialect = $0702;
	kOSASelectAvailableDialects = $0703;
	kOSASelectGetDialectInfo = $0704;
	kOSASelectAvailableDialectCodeList = $0705;

{ Event Handling: }
const
	kOSASelectSetResumeDispatchProc = $0801;
	kOSASelectGetResumeDispatchProc = $0802;
	kOSASelectExecuteEvent = $0803;
	kOSASelectDoEvent = $0804;
	kOSASelectMakeContext = $0805;


{ scripting component specific selectors are added beginning with this value  }
const
	kOSASelectComponentSpecificStart = $1001;


{        Mode Flags:

    Warning: These should not conflict with the AESend mode flags in
    AppleEvents.h, because we may want to use them as OSA mode flags too.
}

{
        This mode flag may be passed to OSALoad, OSAStore or OSACompile to
        instruct the scripting component to not retain the "source" of an
        expression.  This will cause the OSAGetSource call to return the error
        errOSASourceNotAvailable if used.  However, some scripting components
        may not retain the source anyway.  This is mainly used when either space
        efficiency is desired, or a script is to be "locked" so that its
        implementation may not be viewed.
    }
const
	kOSAModePreventGetSource = $00000001;

{
        These mode flags may be passed to OSACompile, OSAExecute, OSALoadExecute
        OSACompileExecute, OSADoScript, OSAExecuteEvent, or OSADoEvent to
        indicate whether or not the script may interact with the user, switch
        layer or reconnect if necessary.  Any AppleEvents will be sent with the
        corresponding AESend mode supplied.
    }
const
	kOSAModeNeverInteract = kAENeverInteract;
	kOSAModeCanInteract = kAECanInteract;
	kOSAModeAlwaysInteract = kAEAlwaysInteract;
	kOSAModeDontReconnect = kAEDontReconnect;

{
        This mode flag may be passed to OSACompile, OSAExecute, OSALoadExecute
        OSACompileExecute, OSADoScript, OSAExecuteEvent, or OSADoEvent to
        indicate whether or not AppleEvents should be sent with the
        kAECanSwitchLayer mode flag sent or not. NOTE: This flag is exactly the
        opposite sense of the AppleEvent flag kAECanSwitchLayer.  This is to
        provide a more convenient default, i.e. not supplying any mode
        (kOSAModeNull) means to send events with kAECanSwitchLayer.  Supplying
        the kOSAModeCantSwitchLayer mode flag will cause AESend to be called
        without kAECanSwitchLayer.
    }
const
	kOSAModeCantSwitchLayer = $00000040;

{
        This mode flag may be passed to OSACompile, OSAExecute, OSALoadExecute
        OSACompileExecute, OSADoScript, OSAExecuteEvent, or OSADoEvent to
        indicate whether or not AppleEvents should be sent with the kAEDontRecord
        mode flag sent or not. NOTE: This flag is exactly the opposite sense of
        the AppleEvent flag kAEDontRecord.  This is to provide a more convenient
        default, i.e. not supplying any mode (kOSAModeNull) means to send events
        with kAEDontRecord.  Supplying the kOSAModeDoRecord mode flag will 
        cause AESend to be called without kAEDontRecord.
    }
const
	kOSAModeDoRecord = $00001000;

{
        This is a mode flag for OSACompile that indicates that a context should
        be created as the result of compilation. All handler definitions are
        inserted into the new context, and variables are initialized by
        evaluating their initial values in a null context (i.e. they must be
        constant expressions).
    }
const
	kOSAModeCompileIntoContext = $00000002;

{
        This is a mode flag for OSACompile that indicates that the previous
        script ID (input to OSACompile) should be augmented with any new
        definitions in the sourceData rather than replaced with a new script.
        This means that the previous script ID must designate a context.
        The presence of this flag causes the kOSAModeCompileIntoContext flag
        to be implicitly used, causing any new definitions to be initialized
        in a null context.
    }
const
	kOSAModeAugmentContext = $00000004;

{
        This mode flag may be passed to OSADisplay or OSADoScript to indicate
        that output only need be human-readable, not re-compilable by OSACompile.
        If used, output may be arbitrarily "beautified", e.g. quotes may be left
        off of string values, long lists may have elipses, etc.
    }
const
	kOSAModeDisplayForHumans = $00000008;

{
        This mode flag may be passed to OSAStore in the case where the scriptID
        is a context.  This causes the context to be saved, but not the context's
        parent context.  When the stored context is loaded back in, the parent
        will be kOSANullScript.
    }
const
	kOSAModeDontStoreParent = $00010000;

{
        This mode flag may be passed to OSAExecuteEvent to cause the event to
        be dispatched to the direct object of the event. The direct object (or
        subject attribute if the direct object is a non-object specifier) will
        be resolved, and the resulting script object will be the recipient of
        the message. The context argument to OSAExecuteEvent will serve as the
        root of the lookup/resolution process.
    }
const
	kOSAModeDispatchToDirectObject = $00020000;

{
        This mode flag may be passed to OSAExecuteEvent to indicate that
        components do not have to get the data of object specifier arguments.
    }
const
	kOSAModeDontGetDataForArguments = $00040000;

<<<<<<< HEAD
	{	*************************************************************************
	    OSA Basic Scripting Interface
	**************************************************************************
	    Scripting components must at least support the Basic Scripting interface.
	*************************************************************************	}
	{	        Loading and Storing Scripts:
	
	    These routines allow scripts to be loaded and stored in their internal
	    (possibly compiled, non-text) representation.
		}

	{	 Resource type for scripts 	}
	kOSAScriptResourceType		= FourCharCode('scpt');

	{	
	        Default type given to OSAStore which creates "generic" loadable script
	        data descriptors.
	    	}
	typeOSAGenericStorage		= FourCharCode('scpt');

	{
	 *  OSALoad()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
	 *    CarbonLib:        in CarbonLib 1.0 and later
	 *    Mac OS X:         in version 10.0 and later
	 	}
function OSALoad(scriptingComponent: ComponentInstance; const (*var*) scriptData: AEDesc; modeFlags: SInt32; var resultingScriptID: OSAID): OSAError; external name '_OSALoad';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{
        This mode flag may be passed to OSACoerceToDesc to indicate that
        the resulting descriptor should be fully qualified (i.e. should
        include the root application reference).
    }
const
	kOSAModeFullyQualifyDescriptors = $00080000;

{*************************************************************************
    OSA Basic Scripting Interface
**************************************************************************
    Scripting components must at least support the Basic Scripting interface.
*************************************************************************}
{        Loading and Storing Scripts:

    These routines allow scripts to be loaded and stored in their internal
    (possibly compiled, non-text) representation.
}

{ Resource type for scripts }
const
	kOSAScriptResourceType = kOSAGenericScriptingComponentSubtype;

{
        Default type given to OSAStore which creates "generic" loadable script
        data descriptors.
    }
const
	typeOSAGenericStorage = kOSAScriptResourceType;

{
 *  OSALoad()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
function OSALoad( scriptingComponent: ComponentInstance; const (*var*) scriptData: AEDesc; modeFlags: SInt32; var resultingScriptID: OSAID ): OSAError; external name '_OSALoad';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
        OSAComponentFunctionInline(kOSASelectLoad, 12);
    
        Errors:
            badComponentInstance        invalid scripting component instance
            errOSASystemError
            errOSABadStorageType:       scriptData not for this scripting component
            errOSACorruptData:          data seems to be corrupt
            errOSADataFormatObsolete    script data format is no longer supported
            errOSADataFormatTooNew      script data format is from a newer version
        
        ModeFlags:
            kOSAModePreventGetSource
    }
{
 *  OSAStore()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
function OSAStore( scriptingComponent: ComponentInstance; scriptID: OSAID; desiredType: DescType; modeFlags: SInt32; var resultingScriptData: AEDesc ): OSAError; external name '_OSAStore';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
<<<<<<< HEAD
function OSAStore(scriptingComponent: ComponentInstance; scriptID: OSAID; desiredType: DescType; modeFlags: SInt32; var resultingScriptData: AEDesc): OSAError; external name '_OSAStore';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function OSAStore( scriptingComponent: ComponentInstance; scriptID: OSAID; desiredType: DescType; modeFlags: SInt32; var resultingScriptData: AEDesc ): OSAError; external name '_OSAStore';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
        OSAComponentFunctionInline(kOSASelectStore, 16);
    
        Errors:
            badComponentInstance    invalid scripting component instance
            errOSASystemError
            errOSAInvalidID
            errOSABadStorageType:   desiredType not for this scripting component
        
        ModeFlags:
            kOSAModePreventGetSource
            kOSAModeDontStoreParent
    }
{ Executing Scripts: }
{
 *  OSAExecute()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
function OSAExecute( scriptingComponent: ComponentInstance; compiledScriptID: OSAID; contextID: OSAID; modeFlags: SInt32; var resultingScriptValueID: OSAID ): OSAError; external name '_OSAExecute';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
<<<<<<< HEAD
function OSAExecute(scriptingComponent: ComponentInstance; compiledScriptID: OSAID; contextID: OSAID; modeFlags: SInt32; var resultingScriptValueID: OSAID): OSAError; external name '_OSAExecute';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function OSAExecute( scriptingComponent: ComponentInstance; compiledScriptID: OSAID; contextID: OSAID; modeFlags: SInt32; var resultingScriptValueID: OSAID ): OSAError; external name '_OSAExecute';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
        OSAComponentFunctionInline(kOSASelectExecute, 16);
        This call runs a script.  The contextID represents the environment
        with which global variables in the script are resolved.  The constant
        kOSANullScript may be used for the contextID if the application wishes
        to not deal with context directly (a default one is associated with each
        scripting component instance).  The resultingScriptValueID is the 
        result of evaluation, and contains a value which may be displayed using
        the OSAGetSource call.  The modeFlags convey scripting component
        specific information.
    
        Errors:
            badComponentInstance    invalid scripting component instance
            errOSASystemError
            errOSAInvalidID
            errOSAScriptError:      the executing script got an error
    
        ModeFlags:
            kOSAModeNeverInteract
            kOSAModeCanInteract
            kOSAModeAlwaysInteract
            kOSAModeCantSwitchLayer
            kOSAModeDontReconnect
            kOSAModeDoRecord
    }
{ Displaying results: }
{
 *  OSADisplay()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
function OSADisplay( scriptingComponent: ComponentInstance; scriptValueID: OSAID; desiredType: DescType; modeFlags: SInt32; var resultingText: AEDesc ): OSAError; external name '_OSADisplay';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
<<<<<<< HEAD
function OSADisplay(scriptingComponent: ComponentInstance; scriptValueID: OSAID; desiredType: DescType; modeFlags: SInt32; var resultingText: AEDesc): OSAError; external name '_OSADisplay';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function OSADisplay( scriptingComponent: ComponentInstance; scriptValueID: OSAID; desiredType: DescType; modeFlags: SInt32; var resultingText: AEDesc ): OSAError; external name '_OSADisplay';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
        OSAComponentFunctionInline(kOSASelectDisplay, 16);
        This call is used to convert results (script value IDs) into displayable
        text. The desiredType should be at least typeChar, and modeFlags are
        scripting system specific flags to control the formatting of the
        resulting text. This call differs from OSAGetSource in that (1) it
        always produces at least typeChar, (2) is only works on script values,
        (3) it may display it's output in non-compilable form (e.g. without
        string quotes, elipses inserted in long and/or circular lists, etc.) and
        (4) it is required by the basic scripting interface.
    
        Errors:
            badComponentInstance    invalid scripting component instance
            errOSASystemError
            errOSAInvalidID
            errAECoercionFail:      desiredType not supported by scripting component
    
        ModeFlags:
            kOSAModeDisplayForHumans
    }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{
    @function   OSACopyDisplayString

    @discussion Similar to OSADisplay, but returns the text as a CFAttributedStringRef.
}
{
 *  OSACopyDisplayString()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function OSACopyDisplayString( scriptingComponent: ComponentInstance; scriptID: OSAID; modeFlags: SInt32; var result: CFAttributedStringRef ): OSAError; external name '_OSACopyDisplayString';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{ Getting Error Information: }
{
 *  OSAScriptError()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
function OSAScriptError( scriptingComponent: ComponentInstance; selector: OSType; desiredType: DescType; var resultingErrorDescription: AEDesc ): OSAError; external name '_OSAScriptError';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
<<<<<<< HEAD
function OSAScriptError(scriptingComponent: ComponentInstance; selector: OSType; desiredType: DescType; var resultingErrorDescription: AEDesc): OSAError; external name '_OSAScriptError';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function OSAScriptError( scriptingComponent: ComponentInstance; selector: OSType; desiredType: DescType; var resultingErrorDescription: AEDesc ): OSAError; external name '_OSAScriptError';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
        OSAComponentFunctionInline(kOSASelectScriptError, 12);
        Whenever script execution returns errOSAExecutionError, this routine
        may be used to get information about that error.  The selector describes
        the type of information desired about the error (various selectors are
        listed below).  The desiredType indicates the data type of the result
        desired for that selector.
    
        Errors:
            badComponentInstance    invalid scripting component instance
            errOSASystemError
            errOSABadSelector:      selector not supported by scripting component
            errAECoercionFail:      desiredType not supported by scripting component
    }
{ OSAScriptError selectors: }
{
        This selector is used to determine the error number of a script error.
        These error numbers may be either system error numbers, or error numbers
        that are scripting component specific.
        Required desiredTypes:  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            typeSInt32
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    }
const
	kOSAErrorNumber = keyErrorNumber;

{
        This selector is used to determine the full error message associated
        with the error number.  It should include the name of the application
        which caused the error, as well as the specific error that occurred.
        This selector is sufficient for simple error reporting (but see
        kOSAErrorBriefMessage, below).
        Required desiredTypes:
            typeChar                    error message string
    }
const
	kOSAErrorMessage = keyErrorString;

{
        This selector is used to determine a brief error message associated with
        the error number.  This message and should not mention the name of the
        application which caused the error, any partial results or offending
        object (see kOSAErrorApp, kOSAErrorPartialResult and
        kOSAErrorOffendingObject, below).
        Required desiredTypes:
            typeChar                    brief error message string
    }
{  0x65727262  }
const
=======
    }
const
	kOSAErrorNumber = keyErrorNumber;

{
        This selector is used to determine the full error message associated
        with the error number.  It should include the name of the application
        which caused the error, as well as the specific error that occurred.
        This selector is sufficient for simple error reporting (but see
        kOSAErrorBriefMessage, below).
        Required desiredTypes:
            typeChar                    error message string
    }
const
	kOSAErrorMessage = keyErrorString;

{
        This selector is used to determine a brief error message associated with
        the error number.  This message and should not mention the name of the
        application which caused the error, any partial results or offending
        object (see kOSAErrorApp, kOSAErrorPartialResult and
        kOSAErrorOffendingObject, below).
        Required desiredTypes:
            typeChar                    brief error message string
    }
{  0x65727262  }
const
>>>>>>> graemeg/cpstrnew
	kOSAErrorBriefMessage = FourCharCode('errb');

{
        This selector is used to determine which application actually got the
        error (if it was the result of an AESend), or the current application
        if ....
        Required desiredTypes:
            typeProcessSerialNumber     PSN of the errant application
            typeChar                    name of the errant application
    }
{  0x65726170  }
const
	kOSAErrorApp = FourCharCode('erap');

{
        This selector is used to determine any partial result returned by an 
        operation. If an AESend call failed, but a partial result was returned,
        then the partial result may be returned as an AEDesc.
        Required desiredTypes:
            typeBest                    AEDesc of any partial result
    }
{  0x70746c72   }
const
	kOSAErrorPartialResult = FourCharCode('ptlr');

{
        This selector is used to determine any object which caused the error
        that may have been indicated by an application.  The result is an 
        AEDesc.
        Required desiredTypes:
            typeBest                    AEDesc of any offending object
    }
{  0x65726f62   }
const
	kOSAErrorOffendingObject = FourCharCode('erob');

{
        This selector is used to determine the type expected by a coercion 
        operation if a type error occurred.
    }
{  0x65727274   }
const
	kOSAErrorExpectedType = FourCharCode('errt');

{
        This selector is used to determine the source text range (start and 
        end positions) of where the error occurred.
        Required desiredTypes:
            typeOSAErrorRange
    }
{  0x65726e67  }
const
	kOSAErrorRange = FourCharCode('erng');

{
        An AERecord type containing keyOSASourceStart and keyOSASourceEnd fields
        of type short.
    }
{  0x65726e67   }
const
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    }
const
	kOSAErrorNumber = keyErrorNumber;

{
        This selector is used to determine the full error message associated
        with the error number.  It should include the name of the application
        which caused the error, as well as the specific error that occurred.
        This selector is sufficient for simple error reporting (but see
        kOSAErrorBriefMessage, below).
        Required desiredTypes:
            typeChar                    error message string
    }
const
	kOSAErrorMessage = keyErrorString;

{
        This selector is used to determine a brief error message associated with
        the error number.  This message and should not mention the name of the
        application which caused the error, any partial results or offending
        object (see kOSAErrorApp, kOSAErrorPartialResult and
        kOSAErrorOffendingObject, below).
        Required desiredTypes:
            typeChar                    brief error message string
    }
{  0x65727262  }
const
	kOSAErrorBriefMessage = FourCharCode('errb');

{
        This selector is used to determine which application actually got the
        error (if it was the result of an AESend), or the current application
        if ....
        Required desiredTypes:
            typeProcessSerialNumber     PSN of the errant application
            typeChar                    name of the errant application
    }
{  0x65726170  }
const
	kOSAErrorApp = FourCharCode('erap');

{
        This selector is used to determine any partial result returned by an 
        operation. If an AESend call failed, but a partial result was returned,
        then the partial result may be returned as an AEDesc.
        Required desiredTypes:
            typeBest                    AEDesc of any partial result
    }
{  0x70746c72   }
const
	kOSAErrorPartialResult = FourCharCode('ptlr');

{
        This selector is used to determine any object which caused the error
        that may have been indicated by an application.  The result is an 
        AEDesc.
        Required desiredTypes:
            typeBest                    AEDesc of any offending object
    }
{  0x65726f62   }
const
	kOSAErrorOffendingObject = FourCharCode('erob');

{
        This selector is used to determine the type expected by a coercion 
        operation if a type error occurred.
    }
{  0x65727274   }
const
	kOSAErrorExpectedType = FourCharCode('errt');

{
        This selector is used to determine the source text range (start and 
        end positions) of where the error occurred.
        Required desiredTypes:
            typeOSAErrorRange
    }
{  0x65726e67  }
const
	kOSAErrorRange = FourCharCode('erng');

{
        An AERecord type containing keyOSASourceStart and keyOSASourceEnd fields
        of type short.
    }
{  0x65726e67   }
const
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	typeOSAErrorRange = FourCharCode('erng');

{ Field of a typeOSAErrorRange record of typeSInt16 }
{  0x73726373    }
const
	keyOSASourceStart = FourCharCode('srcs');

{ Field of a typeOSAErrorRange record of typeSInt16 }
{  0x73726365   }
const
	keyOSASourceEnd = FourCharCode('srce');

{ Disposing Script IDs: }
{
 *  OSADispose()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
function OSADispose( scriptingComponent: ComponentInstance; scriptID: OSAID ): OSAError; external name '_OSADispose';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
            typeShortInteger
=======
            typeSInt32
>>>>>>> origin/fixes_2.4
    }
const
	kOSAErrorNumber = keyErrorNumber;

{
        This selector is used to determine the full error message associated
        with the error number.  It should include the name of the application
        which caused the error, as well as the specific error that occurred.
        This selector is sufficient for simple error reporting (but see
        kOSAErrorBriefMessage, below).
        Required desiredTypes:
            typeChar                    error message string
    }
const
<<<<<<< HEAD
	kOSAErrorNumber				= FourCharCode('errn');

	{	
	        This selector is used to determine the full error message associated
	        with the error number.  It should include the name of the application
	        which caused the error, as well as the specific error that occurred.
	        This selector is sufficient for simple error reporting (but see
	        kOSAErrorBriefMessage, below).
	        Required desiredTypes:
	            typeChar                    error message string
	    	}
	kOSAErrorMessage			= FourCharCode('errs');

	{	
	        This selector is used to determine a brief error message associated with
	        the error number.  This message and should not mention the name of the
	        application which caused the error, any partial results or offending
	        object (see kOSAErrorApp, kOSAErrorPartialResult and
	        kOSAErrorOffendingObject, below).
	        Required desiredTypes:
	            typeChar                    brief error message string
	    	}
	{	  0x65727262  	}
	kOSAErrorBriefMessage		= FourCharCode('errb');

	{	
	        This selector is used to determine which application actually got the
	        error (if it was the result of an AESend), or the current application
	        if ....
	        Required desiredTypes:
	            typeProcessSerialNumber     PSN of the errant application
	            typeChar                    name of the errant application
	    	}
	{	  0x65726170  	}
	kOSAErrorApp				= FourCharCode('erap');

	{	
	        This selector is used to determine any partial result returned by an 
	        operation. If an AESend call failed, but a partial result was returned,
	        then the partial result may be returned as an AEDesc.
	        Required desiredTypes:
	            typeBest                    AEDesc of any partial result
	    	}
	{	  0x70746c72   	}
	kOSAErrorPartialResult		= FourCharCode('ptlr');

	{	
	        This selector is used to determine any object which caused the error
	        that may have been indicated by an application.  The result is an 
	        AEDesc.
	        Required desiredTypes:
	            typeBest                    AEDesc of any offending object
	    	}
	{	  0x65726f62   	}
	kOSAErrorOffendingObject	= FourCharCode('erob');

	{	
	        This selector is used to determine the type expected by a coercion 
	        operation if a type error occurred.
	    	}
	{	  0x65727274   	}
	kOSAErrorExpectedType		= FourCharCode('errt');

	{	
	        This selector is used to determine the source text range (start and 
	        end positions) of where the error occurred.
	        Required desiredTypes:
	            typeOSAErrorRange
	    	}
	{	  0x65726e67  	}
	kOSAErrorRange				= FourCharCode('erng');

	{	
	        An AERecord type containing keyOSASourceStart and keyOSASourceEnd fields
	        of type short.
	    	}
	{	  0x65726e67   	}
	typeOSAErrorRange			= FourCharCode('erng');

	{	 Field of a typeOSAErrorRange record of typeShortInteger 	}
	{	  0x73726373    	}
	keyOSASourceStart			= FourCharCode('srcs');

	{	 Field of a typeOSAErrorRange record of typeShortInteger 	}
	{	  0x73726365   	}
	keyOSASourceEnd				= FourCharCode('srce');

	{	 Disposing Script IDs: 	}
	{
	 *  OSADispose()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
	 *    CarbonLib:        in CarbonLib 1.0 and later
	 *    Mac OS X:         in version 10.0 and later
	 	}
function OSADispose(scriptingComponent: ComponentInstance; scriptID: OSAID): OSAError; external name '_OSADispose';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kOSAErrorMessage = keyErrorString;

{
        This selector is used to determine a brief error message associated with
        the error number.  This message and should not mention the name of the
        application which caused the error, any partial results or offending
        object (see kOSAErrorApp, kOSAErrorPartialResult and
        kOSAErrorOffendingObject, below).
        Required desiredTypes:
            typeChar                    brief error message string
    }
{  0x65727262  }
const
	kOSAErrorBriefMessage = FourCharCode('errb');

{
        This selector is used to determine which application actually got the
        error (if it was the result of an AESend), or the current application
        if ....
        Required desiredTypes:
            typeProcessSerialNumber     PSN of the errant application
            typeChar                    name of the errant application
    }
{  0x65726170  }
const
	kOSAErrorApp = FourCharCode('erap');

{
        This selector is used to determine any partial result returned by an 
        operation. If an AESend call failed, but a partial result was returned,
        then the partial result may be returned as an AEDesc.
        Required desiredTypes:
            typeBest                    AEDesc of any partial result
    }
{  0x70746c72   }
const
	kOSAErrorPartialResult = FourCharCode('ptlr');

{
        This selector is used to determine any object which caused the error
        that may have been indicated by an application.  The result is an 
        AEDesc.
        Required desiredTypes:
            typeBest                    AEDesc of any offending object
    }
{  0x65726f62   }
const
	kOSAErrorOffendingObject = FourCharCode('erob');

{
        This selector is used to determine the type expected by a coercion 
        operation if a type error occurred.
    }
{  0x65727274   }
const
	kOSAErrorExpectedType = FourCharCode('errt');

{
        This selector is used to determine the source text range (start and 
        end positions) of where the error occurred.
        Required desiredTypes:
            typeOSAErrorRange
    }
{  0x65726e67  }
const
	kOSAErrorRange = FourCharCode('erng');

{
        An AERecord type containing keyOSASourceStart and keyOSASourceEnd fields
        of type short.
    }
{  0x65726e67   }
const
	typeOSAErrorRange = FourCharCode('erng');

{ Field of a typeOSAErrorRange record of typeSInt16 }
{  0x73726373    }
const
	keyOSASourceStart = FourCharCode('srcs');

{ Field of a typeOSAErrorRange record of typeSInt16 }
{  0x73726365   }
const
	keyOSASourceEnd = FourCharCode('srce');

{ Disposing Script IDs: }
{
 *  OSADispose()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
function OSADispose( scriptingComponent: ComponentInstance; scriptID: OSAID ): OSAError; external name '_OSADispose';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
        OSAComponentFunctionInline(kOSASelectDispose, 4);
        Disposes a script or context.
    
        Errors:
            badComponentInstance    invalid scripting component instance
            errOSASystemError
            errOSAInvalidID
    }
{ Getting and Setting Script Information: }
{
 *  OSASetScriptInfo()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
function OSASetScriptInfo( scriptingComponent: ComponentInstance; scriptID: OSAID; selector: OSType; value: SIGNEDLONG ): OSAError; external name '_OSASetScriptInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
<<<<<<< HEAD
function OSASetScriptInfo(scriptingComponent: ComponentInstance; scriptID: OSAID; selector: OSType; value: SInt32): OSAError; external name '_OSASetScriptInfo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function OSASetScriptInfo( scriptingComponent: ComponentInstance; scriptID: OSAID; selector: OSType; value: SIGNEDLONG ): OSAError; external name '_OSASetScriptInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
        OSAComponentFunctionInline(kOSASelectSetScriptInfo, 12);
    
        Errors:
            badComponentInstance    invalid scripting component instance
            errOSASystemError
            errOSAInvalidID
            errOSABadSelector:      selector not supported by scripting component
                                    or selector not for this scriptID
    }
{
 *  OSAGetScriptInfo()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
function OSAGetScriptInfo( scriptingComponent: ComponentInstance; scriptID: OSAID; selector: OSType; var result: SIGNEDLONG ): OSAError; external name '_OSAGetScriptInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
<<<<<<< HEAD
function OSAGetScriptInfo(scriptingComponent: ComponentInstance; scriptID: OSAID; selector: OSType; var result: SInt32): OSAError; external name '_OSAGetScriptInfo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function OSAGetScriptInfo( scriptingComponent: ComponentInstance; scriptID: OSAID; selector: OSType; var result: SIGNEDLONG ): OSAError; external name '_OSAGetScriptInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
        OSAComponentFunctionInline(kOSASelectGetScriptInfo, 12);
    
        Errors:
            badComponentInstance    invalid scripting component instance
            errOSASystemError
            errOSAInvalidID
            errOSABadSelector:      selector not supported by scripting component
                                    or selector not for this scriptID
    }
{ Manipulating the ActiveProc:

    Scripting systems will supply default values for these procedures if they
    are not set by the client:
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
	OSAActiveProcPtr = function( refCon: SRefCon ): OSErr;
	OSAActiveUPP = OSAActiveProcPtr;

{
 *  NewOSAActiveUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewOSAActiveUPP( userRoutine: OSAActiveProcPtr ): OSAActiveUPP; external name '_NewOSAActiveUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2

=======
>>>>>>> origin/fixes_2.4
type
	OSAActiveProcPtr = function( refCon: SRefCon ): OSErr;
	OSAActiveUPP = OSAActiveProcPtr;

{
 *  NewOSAActiveUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewOSAActiveUPP( userRoutine: OSAActiveProcPtr ): OSAActiveUPP; external name '_NewOSAActiveUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

<<<<<<< HEAD
const
	uppOSAActiveProcInfo = $000000E0;
	{
	 *  NewOSAActiveUPP()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   available as macro/inline
	 *    CarbonLib:        in CarbonLib 1.0 and later
	 *    Mac OS X:         in version 10.0 and later
	 	}
function NewOSAActiveUPP(userRoutine: OSAActiveProcPtr): OSAActiveUPP; external name '_NewOSAActiveUPP'; { old name was NewOSAActiveProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{
 *  DisposeOSAActiveUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeOSAActiveUPP( userUPP: OSAActiveUPP ); external name '_DisposeOSAActiveUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure DisposeOSAActiveUPP(userUPP: OSAActiveUPP); external name '_DisposeOSAActiveUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeOSAActiveUPP( userUPP: OSAActiveUPP ); external name '_DisposeOSAActiveUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  InvokeOSAActiveUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeOSAActiveUPP( refCon: SRefCon; userUPP: OSAActiveUPP ): OSErr; external name '_InvokeOSAActiveUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function InvokeOSAActiveUPP(refCon: SInt32; userRoutine: OSAActiveUPP): OSErr; external name '_InvokeOSAActiveUPP'; { old name was CallOSAActiveProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function InvokeOSAActiveUPP( refCon: SRefCon; userUPP: OSAActiveUPP ): OSErr; external name '_InvokeOSAActiveUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  OSASetActiveProc()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
function OSASetActiveProc( scriptingComponent: ComponentInstance; activeProc: OSAActiveUPP; refCon: SRefCon ): OSAError; external name '_OSASetActiveProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
<<<<<<< HEAD
function OSASetActiveProc(scriptingComponent: ComponentInstance; activeProc: OSAActiveUPP; refCon: SInt32): OSAError; external name '_OSASetActiveProc';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function OSASetActiveProc( scriptingComponent: ComponentInstance; activeProc: OSAActiveUPP; refCon: SRefCon ): OSAError; external name '_OSASetActiveProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
        OSAComponentFunctionInline(kOSASelectSetActiveProc, 8);
        If activeProc is nil, the default activeProc is used.
    
        Errors:
            badComponentInstance    invalid scripting component instance
            errOSASystemError
    }
{
 *  OSAGetActiveProc()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
function OSAGetActiveProc( scriptingComponent: ComponentInstance; var activeProc: OSAActiveUPP; var refCon: SRefCon ): OSAError; external name '_OSAGetActiveProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
<<<<<<< HEAD
function OSAGetActiveProc(scriptingComponent: ComponentInstance; var activeProc: OSAActiveUPP; var refCon: SInt32): OSAError; external name '_OSAGetActiveProc';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function OSAGetActiveProc( scriptingComponent: ComponentInstance; var activeProc: OSAActiveUPP; var refCon: SRefCon ): OSAError; external name '_OSAGetActiveProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
        OSAComponentFunctionInline(kOSASelectGetActiveProc, 8);
    
        Errors:
            badComponentInstance    invalid scripting component instance
            errOSASystemError
    }
{*************************************************************************
    OSA Optional Compiling Interface
**************************************************************************
    Scripting components that support the Compiling interface have the 
    kOSASupportsCompiling bit set in it's ComponentDescription.
*************************************************************************}
{
 *  OSAScriptingComponentName()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
function OSAScriptingComponentName( scriptingComponent: ComponentInstance; var resultingScriptingComponentName: AEDesc ): OSAError; external name '_OSAScriptingComponentName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
<<<<<<< HEAD
function OSAScriptingComponentName(scriptingComponent: ComponentInstance; var resultingScriptingComponentName: AEDesc): OSAError; external name '_OSAScriptingComponentName';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function OSAScriptingComponentName( scriptingComponent: ComponentInstance; var resultingScriptingComponentName: AEDesc ): OSAError; external name '_OSAScriptingComponentName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
        OSAComponentFunctionInline(kOSASelectScriptingComponentName, 4);
        Given a scripting component, this routine returns the name of that
        scripting component in a type that is coercable to text (typeChar).
        The generic scripting component returns the name of the default
        scripting component.  This name should be sufficient to convey to the
        user the kind of script (syntax) he is expected to write.
    
        Errors:
            badComponentInstance    invalid scripting component instance
            errOSASystemError
    }
{
 *  OSACompile()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
function OSACompile( scriptingComponent: ComponentInstance; const (*var*) sourceData: AEDesc; modeFlags: SInt32; var previousAndResultingScriptID: OSAID ): OSAError; external name '_OSACompile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
<<<<<<< HEAD
function OSACompile(scriptingComponent: ComponentInstance; const (*var*) sourceData: AEDesc; modeFlags: SInt32; var previousAndResultingScriptID: OSAID): OSAError; external name '_OSACompile';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function OSACompile( scriptingComponent: ComponentInstance; const (*var*) sourceData: AEDesc; modeFlags: SInt32; var previousAndResultingScriptID: OSAID ): OSAError; external name '_OSACompile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
        OSAComponentFunctionInline(kOSASelectCompile, 12);
        Coerces input desc (possibly text) into a script's internal format.
        Once compiled, the script is ready to run.  The modeFlags convey
        scripting component specific information.  The previous script ID
        (result parameter) is made to refer to the newly compiled script,
        unless it was originally kOSANullScript.  In this case a new script
        ID is created and used.
    
        Errors:
            badComponentInstance    invalid scripting component instance
            errOSASystemError
            errAECoercionFail:      sourceData is not compilable
            errOSAScriptError:      sourceData was a bad script (syntax error)
            errOSAInvalidID:        previousAndResultingCompiledScriptID was not
                                    valid on input
    
        ModeFlags:
            kOSAModePreventGetSource
            kOSAModeCompileIntoContext
            kOSAModeAugmentContext
            kOSAModeNeverInteract
            kOSAModeCanInteract
            kOSAModeAlwaysInteract
            kOSAModeCantSwitchLayer
            kOSAModeDontReconnect
            kOSAModeDoRecord
    }
{
 *  OSACopyID()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
function OSACopyID( scriptingComponent: ComponentInstance; fromID: OSAID; var toID: OSAID ): OSAError; external name '_OSACopyID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
<<<<<<< HEAD
function OSACopyID(scriptingComponent: ComponentInstance; fromID: OSAID; var toID: OSAID): OSAError; external name '_OSACopyID';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function OSACopyID( scriptingComponent: ComponentInstance; fromID: OSAID; var toID: OSAID ): OSAError; external name '_OSACopyID';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
        OSAComponentFunctionInline(kOSASelectCopyID, 8);
        If toID is a reference to kOSANullScript then it is updated to have a
        new scriptID value.  This call can be used to perform undo or revert
        operations on scripts. 
    
        Errors:
            badComponentInstance    invalid scripting component instance
            errOSASystemError
            errOSAInvalidID
    }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
{$ifc CALL_NOT_IN_CARBON}
>>>>>>> graemeg/fixes_2_2
{
 *  OSACopyScript()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.6 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in AppleScriptLib 1.5 and later
 }
function OSACopyScript( scriptingComponent: ComponentInstance; fromID: OSAID; var toID: OSAID ): OSAError; external name '_OSACopyScript';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)


=======
=======
{$ifc CALL_NOT_IN_CARBON}
=======
>>>>>>> origin/fixes_2.4
{
 *  OSACopyScript()
 *  
 *  Availability:
<<<<<<< HEAD
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in AppleScriptLib 1.5 and later
=======
 *    Mac OS X:         in version 10.6 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in AppleScriptLib 1.5 and later
 }
<<<<<<< HEAD
function OSACopyScript(scriptingComponent: ComponentInstance; fromID: OSAID; var toID: OSAID): OSAError; external name '_OSACopyScript';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function OSACopyScript( scriptingComponent: ComponentInstance; fromID: OSAID; var toID: OSAID ): OSAError; external name '_OSACopyScript';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
        OSAComponentFunctionInline(kOSASelectCopyScript, 8);
        Creates a duplicate copy of the script with the given OSAID and returns
        a new OSAID for it.  Can be used by script editors or debuggers. 
    
        Errors:
            badComponentInstance    invalid scripting component instance
            errOSASystemError
            errOSAInvalidID
    }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
{$endc}  {CALL_NOT_IN_CARBON}

>>>>>>> graemeg/fixes_2_2
=======
{$endc}  {CALL_NOT_IN_CARBON}

>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{*************************************************************************
    OSA Optional GetSource Interface
**************************************************************************
    Scripting components that support the GetSource interface have the 
    kOSASupportsGetSource bit set in their ComponentDescription.
*************************************************************************}
{
 *  OSAGetSource()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
function OSAGetSource( scriptingComponent: ComponentInstance; scriptID: OSAID; desiredType: DescType; var resultingSourceData: AEDesc ): OSAError; external name '_OSAGetSource';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
<<<<<<< HEAD
function OSAGetSource(scriptingComponent: ComponentInstance; scriptID: OSAID; desiredType: DescType; var resultingSourceData: AEDesc): OSAError; external name '_OSAGetSource';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function OSAGetSource( scriptingComponent: ComponentInstance; scriptID: OSAID; desiredType: DescType; var resultingSourceData: AEDesc ): OSAError; external name '_OSAGetSource';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
        OSAComponentFunctionInline(kOSASelectGetSource, 12);
        This routine causes a compiled script to be output in a form (possibly
        text) such that it is suitable to be passed back to OSACompile.

        Errors:
            badComponentInstance    invalid scripting component instance
            errOSASystemError
            errOSAInvalidID
            errOSASourceNotAvailable    can't get source for this scriptID
    }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{
    @function   OSACopySourceString

    @discussion Similar to OSAGetSource, but returns the text as a CFAttributedStringRef.
}
{
 *  OSACopySourceString()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function OSACopySourceString( scriptingComponent: ComponentInstance; scriptID: OSAID; modeFlags: SInt32; var result: CFAttributedStringRef ): OSAError; external name '_OSACopySourceString';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{*************************************************************************
    OSA Optional AECoercion Interface
**************************************************************************
    Scripting components that support the AECoercion interface have the 
    kOSASupportsAECoercion bit set in their ComponentDescription.
*************************************************************************}
{
 *  OSACoerceFromDesc()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
function OSACoerceFromDesc( scriptingComponent: ComponentInstance; const (*var*) scriptData: AEDesc; modeFlags: SInt32; var resultingScriptID: OSAID ): OSAError; external name '_OSACoerceFromDesc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
<<<<<<< HEAD
function OSACoerceFromDesc(scriptingComponent: ComponentInstance; const (*var*) scriptData: AEDesc; modeFlags: SInt32; var resultingScriptID: OSAID): OSAError; external name '_OSACoerceFromDesc';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function OSACoerceFromDesc( scriptingComponent: ComponentInstance; const (*var*) scriptData: AEDesc; modeFlags: SInt32; var resultingScriptID: OSAID ): OSAError; external name '_OSACoerceFromDesc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
        OSAComponentFunctionInline(kOSASelectCoerceFromDesc, 12);
        This routine causes script data to be coerced into a script value.
        If the scriptData is an AppleEvent, then the resultingScriptID is a
        compiled script ID (mode flags for OSACompile may be used in this case).
        Other scriptData descriptors create script value IDs.
    
        Errors:
            badComponentInstance    invalid scripting component instance
            errOSASystemError
    
        ModeFlags:
            kOSAModePreventGetSource
            kOSAModeCompileIntoContext
            kOSAModeNeverInteract
            kOSAModeCanInteract
            kOSAModeAlwaysInteract
            kOSAModeCantSwitchLayer
            kOSAModeDontReconnect
            kOSAModeDoRecord
    }
{
 *  OSACoerceToDesc()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
function OSACoerceToDesc( scriptingComponent: ComponentInstance; scriptID: OSAID; desiredType: DescType; modeFlags: SInt32; var result: AEDesc ): OSAError; external name '_OSACoerceToDesc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
<<<<<<< HEAD
function OSACoerceToDesc(scriptingComponent: ComponentInstance; scriptID: OSAID; desiredType: DescType; modeFlags: SInt32; var result: AEDesc): OSAError; external name '_OSACoerceToDesc';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function OSACoerceToDesc( scriptingComponent: ComponentInstance; scriptID: OSAID; desiredType: DescType; modeFlags: SInt32; var result: AEDesc ): OSAError; external name '_OSACoerceToDesc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
        OSAComponentFunctionInline(kOSASelectCoerceToDesc, 16);
        This routine causes a script value to be coerced into any desired form.
        If the scriptID denotes a compiled script, then it may be coerced to 
        typeAppleEvent.
    
        Errors:
            badComponentInstance    invalid scripting component instance
            errOSASystemError
            errOSAInvalidID
    }
{*************************************************************************
    OSA Optional AESending Interface
**************************************************************************
    Scripting components that support the AESending interface have the 
    kOSASupportsAESending bit set in their ComponentDescription.
*************************************************************************}
{
    Scripting systems will supply default values for these procedures if they
    are not set by the client:
}
{
 *  OSASetSendProc()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
function OSASetSendProc( scriptingComponent: ComponentInstance; sendProc: OSASendUPP; refCon: SRefCon ): OSAError; external name '_OSASetSendProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
<<<<<<< HEAD
function OSASetSendProc(scriptingComponent: ComponentInstance; sendProc: OSASendUPP; refCon: SInt32): OSAError; external name '_OSASetSendProc';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function OSASetSendProc( scriptingComponent: ComponentInstance; sendProc: OSASendUPP; refCon: SRefCon ): OSAError; external name '_OSASetSendProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
        OSAComponentFunctionInline(kOSASelectSetSendProc, 8);
        If sendProc is nil, the default sendProc is used.
    
        Errors:
            badComponentInstance    invalid scripting component instance
            errOSASystemError
    }
{
 *  OSAGetSendProc()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
function OSAGetSendProc( scriptingComponent: ComponentInstance; var sendProc: OSASendUPP; var refCon: SRefCon ): OSAError; external name '_OSAGetSendProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
<<<<<<< HEAD
function OSAGetSendProc(scriptingComponent: ComponentInstance; var sendProc: OSASendUPP; var refCon: SInt32): OSAError; external name '_OSAGetSendProc';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function OSAGetSendProc( scriptingComponent: ComponentInstance; var sendProc: OSASendUPP; var refCon: SRefCon ): OSAError; external name '_OSAGetSendProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
        OSAComponentFunctionInline(kOSASelectGetSendProc, 8);
    
        Errors:
            badComponentInstance    invalid scripting component instance
            errOSASystemError
    }
{
 *  OSASetCreateProc()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
function OSASetCreateProc( scriptingComponent: ComponentInstance; createProc: OSACreateAppleEventUPP; refCon: SRefCon ): OSAError; external name '_OSASetCreateProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
<<<<<<< HEAD
function OSASetCreateProc(scriptingComponent: ComponentInstance; createProc: OSACreateAppleEventUPP; refCon: SInt32): OSAError; external name '_OSASetCreateProc';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function OSASetCreateProc( scriptingComponent: ComponentInstance; createProc: OSACreateAppleEventUPP; refCon: SRefCon ): OSAError; external name '_OSASetCreateProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
        OSAComponentFunctionInline(kOSASelectSetCreateProc, 8);
        If createProc is nil, the default createProc is used.
    
        Errors:
            badComponentInstance    invalid scripting component instance
            errOSASystemError
    }
{
 *  OSAGetCreateProc()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
function OSAGetCreateProc( scriptingComponent: ComponentInstance; var createProc: OSACreateAppleEventUPP; var refCon: SRefCon ): OSAError; external name '_OSAGetCreateProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
<<<<<<< HEAD
function OSAGetCreateProc(scriptingComponent: ComponentInstance; var createProc: OSACreateAppleEventUPP; var refCon: SInt32): OSAError; external name '_OSAGetCreateProc';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function OSAGetCreateProc( scriptingComponent: ComponentInstance; var createProc: OSACreateAppleEventUPP; var refCon: SRefCon ): OSAError; external name '_OSAGetCreateProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
        OSAComponentFunctionInline(kOSASelectGetCreateProc, 8);
    
        Errors:
            badComponentInstance    invalid scripting component instance
            errOSASystemError
    }
{
 *  OSASetDefaultTarget()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
function OSASetDefaultTarget( scriptingComponent: ComponentInstance; const (*var*) target: AEAddressDesc ): OSAError; external name '_OSASetDefaultTarget';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
<<<<<<< HEAD
function OSASetDefaultTarget(scriptingComponent: ComponentInstance; const (*var*) target: AEAddressDesc): OSAError; external name '_OSASetDefaultTarget';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function OSASetDefaultTarget( scriptingComponent: ComponentInstance; const (*var*) target: AEAddressDesc ): OSAError; external name '_OSASetDefaultTarget';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
        OSAComponentFunctionInline(kOSASelectSetDefaultTarget, 4);
        This routine sets the default target application for AE sending.
        It also establishes the default target from which terminologies come.
        It is effectively like having an AppleScript "tell" statement around
        the entire program.  If this routine is not called, or if the target 
        is a null AEDesc, then the current application is the default target.
    
        Errors:
            badComponentInstance    invalid scripting component instance
            errOSASystemError
    }
{*************************************************************************
    OSA Optional Recording Interface
**************************************************************************
    Scripting components that support the Recording interface have the 
    kOSASupportsRecording bit set in their ComponentDescription.
*************************************************************************}
{
 *  OSAStartRecording()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
function OSAStartRecording( scriptingComponent: ComponentInstance; var compiledScriptToModifyID: OSAID ): OSAError; external name '_OSAStartRecording';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
<<<<<<< HEAD
function OSAStartRecording(scriptingComponent: ComponentInstance; var compiledScriptToModifyID: OSAID): OSAError; external name '_OSAStartRecording';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function OSAStartRecording( scriptingComponent: ComponentInstance; var compiledScriptToModifyID: OSAID ): OSAError; external name '_OSAStartRecording';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
        OSAComponentFunctionInline(kOSASelectStartRecording, 4);
        Starts recording.  If compiledScriptToModifyID is kOSANullScript, a
        new script ID is created and returned.  If the current application has
        a handler for the kOSARecordedText event, then kOSARecordedText events
        are sent to the application containing the text of each AppleEvent 
        recorded.
    
        Errors:
            badComponentInstance    invalid scripting component instance
            errOSASystemError
            errOSAInvalidID
            errOSARecordingIsAlreadyOn
    }
{
 *  OSAStopRecording()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
function OSAStopRecording( scriptingComponent: ComponentInstance; compiledScriptID: OSAID ): OSAError; external name '_OSAStopRecording';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
<<<<<<< HEAD
function OSAStopRecording(scriptingComponent: ComponentInstance; compiledScriptID: OSAID): OSAError; external name '_OSAStopRecording';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function OSAStopRecording( scriptingComponent: ComponentInstance; compiledScriptID: OSAID ): OSAError; external name '_OSAStopRecording';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
        OSAComponentFunctionInline(kOSASelectStopRecording, 4);
        If compiledScriptID is not being recorded into or recording is not
        currently on, no error is returned.
    
        Errors:
            badComponentInstance    invalid scripting component instance
            errOSASystemError
            errOSAInvalidID
    }
{*************************************************************************
    OSA Optional Convenience Interface
**************************************************************************
    Scripting components that support the Convenience interface have the 
    kOSASupportsConvenience bit set in their ComponentDescription.
*************************************************************************}
{
 *  OSALoadExecute()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
function OSALoadExecute( scriptingComponent: ComponentInstance; const (*var*) scriptData: AEDesc; contextID: OSAID; modeFlags: SInt32; var resultingScriptValueID: OSAID ): OSAError; external name '_OSALoadExecute';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
<<<<<<< HEAD
function OSALoadExecute(scriptingComponent: ComponentInstance; const (*var*) scriptData: AEDesc; contextID: OSAID; modeFlags: SInt32; var resultingScriptValueID: OSAID): OSAError; external name '_OSALoadExecute';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function OSALoadExecute( scriptingComponent: ComponentInstance; const (*var*) scriptData: AEDesc; contextID: OSAID; modeFlags: SInt32; var resultingScriptValueID: OSAID ): OSAError; external name '_OSALoadExecute';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
        OSAComponentFunctionInline(kOSASelectLoadExecute, 16);
        This routine is effectively equivalent to calling OSALoad followed by
        OSAExecute.  After execution, the compiled source is disposed.  Only the
        resulting value ID is retained.
    
        Errors:
            badComponentInstance        invalid scripting component instance
            errOSASystemError
            errOSABadStorageType:       scriptData not for this scripting component
            errOSACorruptData:          data seems to be corrupt
            errOSADataFormatObsolete    script data format is no longer supported
            errOSADataFormatTooNew      script data format is from a newer version
            errOSAInvalidID
            errOSAScriptError:          the executing script got an error
    
        ModeFlags:
            kOSAModeNeverInteract
            kOSAModeCanInteract
            kOSAModeAlwaysInteract
            kOSAModeCantSwitchLayer
            kOSAModeDontReconnect
            kOSAModeDoRecord
    }
{
 *  OSACompileExecute()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
function OSACompileExecute( scriptingComponent: ComponentInstance; const (*var*) sourceData: AEDesc; contextID: OSAID; modeFlags: SInt32; var resultingScriptValueID: OSAID ): OSAError; external name '_OSACompileExecute';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
<<<<<<< HEAD
function OSACompileExecute(scriptingComponent: ComponentInstance; const (*var*) sourceData: AEDesc; contextID: OSAID; modeFlags: SInt32; var resultingScriptValueID: OSAID): OSAError; external name '_OSACompileExecute';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function OSACompileExecute( scriptingComponent: ComponentInstance; const (*var*) sourceData: AEDesc; contextID: OSAID; modeFlags: SInt32; var resultingScriptValueID: OSAID ): OSAError; external name '_OSACompileExecute';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
        OSAComponentFunctionInline(kOSASelectCompileExecute, 16);
        This routine is effectively equivalent to calling OSACompile followed by
        OSAExecute.  After execution, the compiled source is disposed.  Only the
        resulting value ID is retained.
    
        Errors:
            badComponentInstance    invalid scripting component instance
            errOSASystemError
            errAECoercionFail:      sourceData is not compilable
            errOSAScriptError:      sourceData was a bad script (syntax error)
            errOSAInvalidID:        previousAndResultingCompiledScriptID was not
                                    valid on input
            errOSAScriptError:      the executing script got an error
    
        ModeFlags:
            kOSAModeNeverInteract
            kOSAModeCanInteract
            kOSAModeAlwaysInteract
            kOSAModeCantSwitchLayer
            kOSAModeDontReconnect
            kOSAModeDoRecord
    }
{
 *  OSADoScript()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
function OSADoScript( scriptingComponent: ComponentInstance; const (*var*) sourceData: AEDesc; contextID: OSAID; desiredType: DescType; modeFlags: SInt32; var resultingText: AEDesc ): OSAError; external name '_OSADoScript';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
<<<<<<< HEAD
function OSADoScript(scriptingComponent: ComponentInstance; const (*var*) sourceData: AEDesc; contextID: OSAID; desiredType: DescType; modeFlags: SInt32; var resultingText: AEDesc): OSAError; external name '_OSADoScript';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function OSADoScript( scriptingComponent: ComponentInstance; const (*var*) sourceData: AEDesc; contextID: OSAID; desiredType: DescType; modeFlags: SInt32; var resultingText: AEDesc ): OSAError; external name '_OSADoScript';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
        OSAComponentFunctionInline(kOSASelectDoScript, 20);
        This routine is effectively equivalent to calling OSACompile followed by
        OSAExecute and then OSADisplay.  After execution, the compiled source
        and the resulting value are is disposed.  Only the resultingText
        descriptor is retained.  If a script error occur during processing, the 
        resultingText gets the error message of the error, and errOSAScriptError
        is returned.  OSAScriptError may still be used to extract more 
        information about the particular error.
    
        Errors:
            badComponentInstance    invalid scripting component instance
            errOSASystemError
            errAECoercionFail:      sourceData is not compilable or 
                                    desiredType not supported by scripting component
            errOSAScriptError:      sourceData was a bad script (syntax error)
            errOSAInvalidID:        previousAndResultingCompiledScriptID was not
                                    valid on input
            errOSAScriptError:      the executing script got an error
    
        ModeFlags:
            kOSAModeNeverInteract
            kOSAModeCanInteract
            kOSAModeAlwaysInteract
            kOSAModeCantSwitchLayer
            kOSAModeDontReconnect
            kOSAModeDoRecord
            kOSAModeDisplayForHumans
    }
{*************************************************************************
    OSA Optional Dialects Interface
**************************************************************************
    Scripting components that support the Dialects interface have the 
    kOSASupportsDialects bit set in their ComponentDescription.
*************************************************************************}
{
    These calls allows an scripting component that supports different dialects
    to dynamically switch between those dialects.  Although this interface is
    specified, the particular dialect codes are scripting component dependent.
}
{
 *  OSASetCurrentDialect()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
function OSASetCurrentDialect( scriptingComponent: ComponentInstance; dialectCode: SInt16 ): OSAError; external name '_OSASetCurrentDialect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
<<<<<<< HEAD
function OSASetCurrentDialect(scriptingComponent: ComponentInstance; dialectCode: SInt16): OSAError; external name '_OSASetCurrentDialect';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function OSASetCurrentDialect( scriptingComponent: ComponentInstance; dialectCode: SInt16 ): OSAError; external name '_OSASetCurrentDialect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
        OSAComponentFunctionInline(kOSASelectSetCurrentDialect, 2);
    
        Errors:
            badComponentInstance    invalid scripting component instance
            errOSASystemError
            errOSANoSuchDialect:    invalid dialectCode
    }
{
 *  OSAGetCurrentDialect()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
function OSAGetCurrentDialect( scriptingComponent: ComponentInstance; var resultingDialectCode: SInt16 ): OSAError; external name '_OSAGetCurrentDialect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
<<<<<<< HEAD
function OSAGetCurrentDialect(scriptingComponent: ComponentInstance; var resultingDialectCode: SInt16): OSAError; external name '_OSAGetCurrentDialect';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function OSAGetCurrentDialect( scriptingComponent: ComponentInstance; var resultingDialectCode: SInt16 ): OSAError; external name '_OSAGetCurrentDialect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
        OSAComponentFunctionInline(kOSASelectGetCurrentDialect, 4);
    
        Errors:
            badComponentInstance    invalid scripting component instance
            errOSASystemError
    }
{
 *  OSAAvailableDialects()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
function OSAAvailableDialects( scriptingComponent: ComponentInstance; var resultingDialectInfoList: AEDesc ): OSAError; external name '_OSAAvailableDialects';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
<<<<<<< HEAD
function OSAAvailableDialects(scriptingComponent: ComponentInstance; var resultingDialectInfoList: AEDesc): OSAError; external name '_OSAAvailableDialects';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function OSAAvailableDialects( scriptingComponent: ComponentInstance; var resultingDialectInfoList: AEDesc ): OSAError; external name '_OSAAvailableDialects';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
        OSAComponentFunctionInline(kOSASelectAvailableDialects, 4);
        This call return an AEList containing information about each of the
        currently available dialects of a scripting component.  Each item
        is an AERecord of typeOSADialectInfo that contains at least the fields
        keyOSADialectName, keyOSADialectCode, KeyOSADialectLangCode and 
        keyOSADialectScriptCode.
    
        Errors:
            badComponentInstance    invalid scripting component instance
            errOSASystemError
    }
{
 *  OSAGetDialectInfo()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
function OSAGetDialectInfo( scriptingComponent: ComponentInstance; dialectCode: SInt16; selector: OSType; var resultingDialectInfo: AEDesc ): OSAError; external name '_OSAGetDialectInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
<<<<<<< HEAD
function OSAGetDialectInfo(scriptingComponent: ComponentInstance; dialectCode: SInt16; selector: OSType; var resultingDialectInfo: AEDesc): OSAError; external name '_OSAGetDialectInfo';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function OSAGetDialectInfo( scriptingComponent: ComponentInstance; dialectCode: SInt16; selector: OSType; var resultingDialectInfo: AEDesc ): OSAError; external name '_OSAGetDialectInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
        OSAComponentFunctionInline(kOSASelectGetDialectInfo, 10);
        This call gives information about the specified dialect of a scripting
        component. It returns an AEDesc whose type depends on the selector 
        specified. Available selectors are the same as the field keys for a
        dialect info record. The type of AEDesc returned is the same as the 
        type of the field that has same key as the selector.
    
        Errors:
            badComponentInstance    invalid scripting component instance
            errOSASystemError
            errOSABadSelector
            errOSANoSuchDialect:    invalid dialectCode
    }
{
 *  OSAAvailableDialectCodeList()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
function OSAAvailableDialectCodeList( scriptingComponent: ComponentInstance; var resultingDialectCodeList: AEDesc ): OSAError; external name '_OSAAvailableDialectCodeList';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
<<<<<<< HEAD
function OSAAvailableDialectCodeList(scriptingComponent: ComponentInstance; var resultingDialectCodeList: AEDesc): OSAError; external name '_OSAAvailableDialectCodeList';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function OSAAvailableDialectCodeList( scriptingComponent: ComponentInstance; var resultingDialectCodeList: AEDesc ): OSAError; external name '_OSAAvailableDialectCodeList';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
        OSAComponentFunctionInline(kOSASelectAvailableDialectCodeList, 4);
        This is alternative to OSAGetAvailableDialectCodeList. Use this call
        and  OSAGetDialectInfo to get information on dialects.
        This call return an AEList containing dialect code for each of the
        currently available dialects of a scripting component. Each dialect
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        code is a short integer of type typeSInt16.
=======
        code is a short SInt16 of type typeShortInteger.
>>>>>>> graemeg/fixes_2_2
=======
        code is a short SInt16 of type typeShortInteger.
>>>>>>> origin/fixes_2_2
=======
        code is a short integer of type typeSInt16.
>>>>>>> origin/fixes_2.4
    
        Errors:
            badComponentInstance    invalid scripting component instance
            errOSASystemError

        Type of a dialect info record containing at least keyOSADialectName
        and keyOSADialectCode fields.

        keys for dialect info record, also used as selectors to OSAGetDialectInfo.

        Field of a typeOSADialectInfo record of typeChar.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        Field of a typeOSADialectInfo record of typeSInt16.
        Field of a typeOSADialectInfo record of typeSInt16.
        Field of a typeOSADialectInfo record of typeSInt16.
=======
        Field of a typeOSADialectInfo record of typeShortInteger.
        Field of a typeOSADialectInfo record of typeShortInteger.
        Field of a typeOSADialectInfo record of typeShortInteger.
>>>>>>> graemeg/fixes_2_2
=======
        Field of a typeOSADialectInfo record of typeShortInteger.
        Field of a typeOSADialectInfo record of typeShortInteger.
        Field of a typeOSADialectInfo record of typeShortInteger.
>>>>>>> origin/fixes_2_2
=======
        Field of a typeOSADialectInfo record of typeSInt16.
        Field of a typeOSADialectInfo record of typeSInt16.
        Field of a typeOSADialectInfo record of typeSInt16.
>>>>>>> origin/fixes_2.4
    }
{*************************************************************************
    OSA Optional Event Handling Interface
**************************************************************************
    Scripting components that support the Event Handling interface have the 
    kOSASupportsEventHandling bit set in their ComponentDescription.
*************************************************************************}
{
 *  OSASetResumeDispatchProc()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
function OSASetResumeDispatchProc( scriptingComponent: ComponentInstance; resumeDispatchProc: AEEventHandlerUPP; refCon: SRefCon ): OSAError; external name '_OSASetResumeDispatchProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
<<<<<<< HEAD
function OSASetResumeDispatchProc(scriptingComponent: ComponentInstance; resumeDispatchProc: AEEventHandlerUPP; refCon: SInt32): OSAError; external name '_OSASetResumeDispatchProc';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function OSASetResumeDispatchProc( scriptingComponent: ComponentInstance; resumeDispatchProc: AEEventHandlerUPP; refCon: SRefCon ): OSAError; external name '_OSASetResumeDispatchProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
        OSAComponentFunctionInline(kOSASelectSetResumeDispatchProc, 8);
        This function is used to set the ResumeDispatchProc that will be used
        by OSAExecuteEvent and OSADoEvent if either no event handler can be
        found in the context, or the context event hander "continues" control
        onward. The two constants kOSAUseStandardDispatch and kOSANoDispatch
        may also be passed to this routine indicating that the handler registered
        in the application with AEInstallEventHandler should be used, or no
        dispatch should occur, respectively.
    
        Errors:
            badComponentInstance    invalid scripting component instance
            errOSASystemError
    }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
const
	kOSAUseStandardDispatch = kAEUseStandardDispatch;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{
        Special ResumeDispatchProc constant which may be passed to 
        OSASetResumeDispatchProc indicating that the handler registered
        in the application with AEInstallEventHandler should be used.
        
        NOTE:   Had to remove the cast (AEEventHandlerUPP).  The C compiler
                doesn't allow pointer types to be assigned to an enum.  All
                constants must be assigned as enums to translate properly to
                Pascal.
    }
const
	kOSANoDispatch = kAENoDispatch;

{
        Special ResumeDispatchProc constant which may be passed to 
        OSASetResumeDispatchProc indicating that no dispatch should occur.
=======

{
        Special ResumeDispatchProc constant which may be passed to 
        OSASetResumeDispatchProc indicating that the handler registered
        in the application with AEInstallEventHandler should be used.
>>>>>>> graemeg/cpstrnew
        
        NOTE:   Had to remove the cast (AEEventHandlerUPP).  The C compiler
                doesn't allow pointer types to be assigned to an enum.  All
                constants must be assigned as enums to translate properly to
                Pascal.
    }
const
<<<<<<< HEAD
=======
	kOSANoDispatch = kAENoDispatch;

{
        Special ResumeDispatchProc constant which may be passed to 
        OSASetResumeDispatchProc indicating that no dispatch should occur.
=======

{
        Special ResumeDispatchProc constant which may be passed to 
        OSASetResumeDispatchProc indicating that the handler registered
        in the application with AEInstallEventHandler should be used.
>>>>>>> graemeg/cpstrnew
        
        NOTE:   Had to remove the cast (AEEventHandlerUPP).  The C compiler
                doesn't allow pointer types to be assigned to an enum.  All
                constants must be assigned as enums to translate properly to
                Pascal.
    }
const
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
	kOSANoDispatch = kAENoDispatch;

{
        Special ResumeDispatchProc constant which may be passed to 
        OSASetResumeDispatchProc indicating that no dispatch should occur.
=======

{
        Special ResumeDispatchProc constant which may be passed to 
        OSASetResumeDispatchProc indicating that the handler registered
        in the application with AEInstallEventHandler should be used.
>>>>>>> graemeg/cpstrnew
        
        NOTE:   Had to remove the cast (AEEventHandlerUPP).  The C compiler
                doesn't allow pointer types to be assigned to an enum.  All
                constants must be assigned as enums to translate properly to
                Pascal.
    }
const
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
	kOSANoDispatch = kAENoDispatch;

{
        Special ResumeDispatchProc constant which may be passed to 
        OSASetResumeDispatchProc indicating that no dispatch should occur.
=======

{
        Special ResumeDispatchProc constant which may be passed to 
        OSASetResumeDispatchProc indicating that the handler registered
        in the application with AEInstallEventHandler should be used.
>>>>>>> origin/cpstrnew
        
        NOTE:   Had to remove the cast (AEEventHandlerUPP).  The C compiler
                doesn't allow pointer types to be assigned to an enum.  All
                constants must be assigned as enums to translate properly to
                Pascal.
    }
const
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
	kOSANoDispatch = kAENoDispatch;

{
        Special ResumeDispatchProc constant which may be passed to 
        OSASetResumeDispatchProc indicating that no dispatch should occur.
        
        NOTE:   Had to remove the cast (AEEventHandlerUPP).  The C compiler
                doesn't allow pointer types to be assigned to an enum.  All
                constants must be assigned as enums to translate properly to
                Pascal.
    }
const
>>>>>>> origin/cpstrnew
	kOSADontUsePhac = $0001;

{
        Special refCon constant that may be given to OSASetResumeDispatchProc
        only when kOSAUseStandardDispatch is used as the ResumeDispatchProc.
        This causes the standard dispatch to be performed, except the phac
        handler is not called.  This is useful during tinkerability, when
        the phac handler is used to lookup a context associated with an event's 
        direct parameter, and call OSAExecuteEvent or OSADoEvent.  Failure to
        bypass the phac handler would result in an infinite loop.
    }
{
 *  OSAGetResumeDispatchProc()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
function OSAGetResumeDispatchProc( scriptingComponent: ComponentInstance; var resumeDispatchProc: AEEventHandlerUPP; var refCon: SRefCon ): OSAError; external name '_OSAGetResumeDispatchProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
=======
const
	kOSAUseStandardDispatch = kAEUseStandardDispatch;
>>>>>>> origin/fixes_2.4

{
        Special ResumeDispatchProc constant which may be passed to 
        OSASetResumeDispatchProc indicating that the handler registered
        in the application with AEInstallEventHandler should be used.
        
        NOTE:   Had to remove the cast (AEEventHandlerUPP).  The C compiler
                doesn't allow pointer types to be assigned to an enum.  All
                constants must be assigned as enums to translate properly to
                Pascal.
    }
const
<<<<<<< HEAD
	kOSAUseStandardDispatch		= $FFFFFFFF;

	{	
	        Special ResumeDispatchProc constant which may be passed to 
	        OSASetResumeDispatchProc indicating that the handler registered
	        in the application with AEInstallEventHandler should be used.
	        
	        NOTE:   Had to remove the cast (AEEventHandlerUPP).  The C compiler
	                doesn't allow pointer types to be assigned to an enum.  All
	                constants must be assigned as enums to translate properly to
	                Pascal.
	    	}
	kOSANoDispatch				= 0;

	{	
	        Special ResumeDispatchProc constant which may be passed to 
	        OSASetResumeDispatchProc indicating that no dispatch should occur.
	        
	        NOTE:   Had to remove the cast (AEEventHandlerUPP).  The C compiler
	                doesn't allow pointer types to be assigned to an enum.  All
	                constants must be assigned as enums to translate properly to
	                Pascal.
	    	}
	kOSADontUsePhac				= $0001;

	{	
	        Special refCon constant that may be given to OSASetResumeDispatchProc
	        only when kOSAUseStandardDispatch is used as the ResumeDispatchProc.
	        This causes the standard dispatch to be performed, except the phac
	        handler is not called.  This is useful during tinkerability, when
	        the phac handler is used to lookup a context associated with an event's 
	        direct parameter, and call OSAExecuteEvent or OSADoEvent.  Failure to
	        bypass the phac handler would result in an infinite loop.
	    	}
	{
	 *  OSAGetResumeDispatchProc()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
	 *    CarbonLib:        in CarbonLib 1.0 and later
	 *    Mac OS X:         in version 10.0 and later
	 	}
function OSAGetResumeDispatchProc(scriptingComponent: ComponentInstance; var resumeDispatchProc: AEEventHandlerUPP; var refCon: SInt32): OSAError; external name '_OSAGetResumeDispatchProc';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kOSANoDispatch = kAENoDispatch;

{
        Special ResumeDispatchProc constant which may be passed to 
        OSASetResumeDispatchProc indicating that no dispatch should occur.
        
        NOTE:   Had to remove the cast (AEEventHandlerUPP).  The C compiler
                doesn't allow pointer types to be assigned to an enum.  All
                constants must be assigned as enums to translate properly to
                Pascal.
    }
const
	kOSADontUsePhac = $0001;

{
        Special refCon constant that may be given to OSASetResumeDispatchProc
        only when kOSAUseStandardDispatch is used as the ResumeDispatchProc.
        This causes the standard dispatch to be performed, except the phac
        handler is not called.  This is useful during tinkerability, when
        the phac handler is used to lookup a context associated with an event's 
        direct parameter, and call OSAExecuteEvent or OSADoEvent.  Failure to
        bypass the phac handler would result in an infinite loop.
    }
{
 *  OSAGetResumeDispatchProc()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
function OSAGetResumeDispatchProc( scriptingComponent: ComponentInstance; var resumeDispatchProc: AEEventHandlerUPP; var refCon: SRefCon ): OSAError; external name '_OSAGetResumeDispatchProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
        OSAComponentFunctionInline(kOSASelectGetResumeDispatchProc, 8);
        Returns the registered ResumeDispatchProc.  If no ResumeDispatchProc has
        been registered, then kOSAUseStandardDispatch (the default) is returned.
    
        Errors:
            badComponentInstance    invalid scripting component instance
            errOSASystemError
    }
{
 *  OSAExecuteEvent()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
function OSAExecuteEvent( scriptingComponent: ComponentInstance; const (*var*) theAppleEvent: AppleEvent; contextID: OSAID; modeFlags: SInt32; var resultingScriptValueID: OSAID ): OSAError; external name '_OSAExecuteEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
<<<<<<< HEAD
function OSAExecuteEvent(scriptingComponent: ComponentInstance; const (*var*) theAppleEvent: AppleEvent; contextID: OSAID; modeFlags: SInt32; var resultingScriptValueID: OSAID): OSAError; external name '_OSAExecuteEvent';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function OSAExecuteEvent( scriptingComponent: ComponentInstance; const (*var*) theAppleEvent: AppleEvent; contextID: OSAID; modeFlags: SInt32; var resultingScriptValueID: OSAID ): OSAError; external name '_OSAExecuteEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
        OSAComponentFunctionInline(kOSASelectExecuteEvent, 16);
        This call is similar to OSAExecute except the initial command to
        execute comes in the form of an AppleEvent.  If the contextID
        defines any event handlers for that event, they are used to process
        the event.  If no event handler can be found in the context
        errAEEventNotHandled is returned.  If an event handler is found and
        the hander "continues" control onward, the ResumeDispatchProc
        (registered with OSASetResumeDispatchProc, above) is called given the
        AppleEvent.  The result is returned as a scriptValueID.
    
        Errors:
            badComponentInstance    invalid scripting component instance
            errOSASystemError
            errOSAInvalidID
            errOSAScriptError:      the executing script got an error
            errAEEventNotHandled:   no handler for event in contextID
    
        ModeFlags:
            kOSAModeNeverInteract
            kOSAModeCanInteract
            kOSAModeAlwaysInteract
            kOSAModeCantSwitchLayer
            kOSAModeDontReconnect
            kOSAModeDoRecord
    }
{
 *  OSADoEvent()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
function OSADoEvent( scriptingComponent: ComponentInstance; const (*var*) theAppleEvent: AppleEvent; contextID: OSAID; modeFlags: SInt32; var reply: AppleEvent ): OSAError; external name '_OSADoEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
<<<<<<< HEAD
function OSADoEvent(scriptingComponent: ComponentInstance; const (*var*) theAppleEvent: AppleEvent; contextID: OSAID; modeFlags: SInt32; var reply: AppleEvent): OSAError; external name '_OSADoEvent';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function OSADoEvent( scriptingComponent: ComponentInstance; const (*var*) theAppleEvent: AppleEvent; contextID: OSAID; modeFlags: SInt32; var reply: AppleEvent ): OSAError; external name '_OSADoEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
        OSAComponentFunctionInline(kOSASelectDoEvent, 16);
        This call is similar to OSADoScript except the initial command to
        execute comes in the form of an AppleEvent, and the result is an 
        AppleEvent reply record.  If the contextID defines any event handlers
        for that event, they are used to process the event.  If no event handler
        can be found in the context errAEEventNotHandled is returned.  If an
        event handler is found and the hander "continues" control onward, the
        ResumeDispatchProc (registered with OSASetResumeDispatchProc, above) is
        called given the AppleEvent.  The result is returned in the form of an
        AppleEvent reply descriptor. If at any time the script gets an error, or
        if the ResumeDispatchProc returns a reply event indicating an error,
        then the OSADoEvent call itself returns an error reply (i.e. OSADoEvent
        should never return errOSAScriptError).  Any error result returned by
        the ResumeDispatchProc will be returned by OSADoEvent.
    
        Errors:
            badComponentInstance    invalid scripting component instance
            errOSASystemError
            errOSAInvalidID
            errAEEventNotHandled:   no handler for event in contextID
    
        ModeFlags:
            kOSAModeNeverInteract
            kOSAModeCanInteract
            kOSAModeAlwaysInteract
            kOSAModeCantSwitchLayer
            kOSAModeDontReconnect
            kOSAModeDoRecord
    }
{
 *  OSAMakeContext()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
function OSAMakeContext( scriptingComponent: ComponentInstance; const (*var*) contextName: AEDesc; parentContext: OSAID; var resultingContextID: OSAID ): OSAError; external name '_OSAMakeContext';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in AppleScriptLib 1.1 and later
 }
<<<<<<< HEAD
function OSAMakeContext(scriptingComponent: ComponentInstance; const (*var*) contextName: AEDesc; parentContext: OSAID; var resultingContextID: OSAID): OSAError; external name '_OSAMakeContext';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function OSAMakeContext( scriptingComponent: ComponentInstance; const (*var*) contextName: AEDesc; parentContext: OSAID; var resultingContextID: OSAID ): OSAError; external name '_OSAMakeContext';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
        OSAComponentFunctionInline(kOSASelectMakeContext, 12);
        Makes a new empty context which may be passed to OSAExecute or 
        OSAExecuteEvent.  If contextName is typeNull, an unnamed context is
        created. If parentContext is kOSANullScript then the resulting context
        does not inherit bindings from any other context.
    
        Errors:
            badComponentInstance    invalid scripting component instance
            errOSASystemError
            errOSAInvalidID
            errAECoercionFail:      contextName is invalid
    }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD


{*************************************************************************
    OSA Script File Interface
*************************************************************************}
{
 *  OSAGetScriptDataFromURL()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.6 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function OSAGetScriptDataFromURL( scriptURL: CFURLRef; storable: BooleanPtr { can be NULL }; modeFlags: SInt32; var resultingScriptData: AEDesc ): OSAError; external name '_OSAGetScriptDataFromURL';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)


{
        This routine reads script data from a URL into a descriptor. The
        URL may refer to a compiled script, script application, or source
        text file. If "scriptURL" refers to a text file, the resulting data
        is the source text. If "storable" is non-NULL, it will be set to
        indicate whether a script can be stored into the script file using
        OSAStoreFile().
     
        You may use OSALoadScriptData() with the resulting descriptor to
        load the script into a component instance. Doing this in two steps
        affords the opportunity to examine the script data with
        OSAGetStorageType() and select a component instance.
    
        Errors:
            errOSASystemError
            File system errors.
            
        ModeFlags:
            No mode flags are supported at this time.
    }
{
 *  OSALoadScriptData()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.6 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function OSALoadScriptData( scriptingComponent: ComponentInstance; const (*var*) scriptData: AEDesc; fromURL: CFURLRef { can be NULL }; modeFlags: SInt32; var resultingScriptID: OSAID ): OSAError; external name '_OSALoadScriptData';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)


{
        This routine loads script data from a descriptor into the specified
        scripting component. If "scriptData" is source, it will be compiled.
        If "fromURL" is non-NULL, it indicates the file from which the data
        was read.
    
        Errors:
            See OSALoad() and OSACompile().
            
        ModeFlags:
            See OSALoad() and OSACompile().
    }
{
 *  OSALoadFile()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function OSALoadFile( scriptingComponent: ComponentInstance; const (*var*) scriptFile: FSRef; storable: BooleanPtr { can be NULL }; modeFlags: SInt32; var resultingScriptID: OSAID ): OSAError; external name '_OSALoadFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
        This routine loads a script into the specified scripting component.
        If "scriptFile" is a text file, the script will be compiled. If
        "storable" is non-NULL, it will be set to indicate whether a
        script can be stored into the script file using OSAStoreFile().
    
        Errors:
            See OSALoad() and OSACompile().
            
        ModeFlags:
            See OSALoad() and OSACompile().
    }
{
 *  OSAStoreFile()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function OSAStoreFile( scriptingComponent: ComponentInstance; scriptID: OSAID; desiredType: DescType; modeFlags: SInt32; const (*var*) scriptFile: FSRef ): OSAError; external name '_OSAStoreFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
        This routine stores a script into the specified file.
    
        Errors:
            See OSAStore().
        
        ModeFlags:
            See OSAStore().
    }
{
 *  OSALoadExecuteFile()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function OSALoadExecuteFile( scriptingComponent: ComponentInstance; const (*var*) scriptFile: FSRef; contextID: OSAID; modeFlags: SInt32; var resultingScriptValueID: OSAID ): OSAError; external name '_OSALoadExecuteFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
        This routine is effectively equivalent to calling OSALoadFile followed by
        OSAExecute.  After execution, the compiled source is disposed.  Only the
        resulting value ID is retained.
    
        Errors:
            See OSALoadExecute() and OSACompileExecute().
    
        ModeFlags:
            See OSALoadExecute() and OSACompileExecute().
    }
{
 *  OSADoScriptFile()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function OSADoScriptFile( scriptingComponent: ComponentInstance; const (*var*) scriptFile: FSRef; contextID: OSAID; desiredType: DescType; modeFlags: SInt32; var resultingText: AEDesc ): OSAError; external name '_OSADoScriptFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
        This routine is effectively equivalent to calling OSALoadFile, followed by 
        OSAExecute, OSADisplay, and then OSAStoreFile if the script has persistent 
        properties.  After execution, the compiled source and the resulting value are 
        disposed.  Only the resultingText descriptor is retained.  If a script error 
        occurs during processing, the resultingText gets the error message of the error, 
        and errOSAScriptError is returned.  OSAScriptError may still be used to extract 
        more information about the particular error.
    
        Errors:
            See OSALoad(), OSACompile(), OSAExecute(), OSADisplay(), and OSAStore().
    
        ModeFlags:
            See OSALoad(), OSACompile(), OSAExecute(), and OSADisplay().
    }

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
{
 * Debugging API
 }
{
 * Types
 }

type
	OSADebugSessionRef					= OSAID;
	OSADebugCallFrameRef				= OSAID;
	{	
	 * Constants
	 	}
	OSAProgramState 			= UInt32;
const
	eNotStarted					= 0;
	eRunnable					= 1;
	eRunning					= 2;
	eStopped					= 3;
	eTerminated					= 4;


type
	OSADebugStepKind 			= UInt32;
const
	eStepOver					= 0;
	eStepIn						= 1;
	eStepOut					= 2;
	eRun						= 3;


type
	OSALocalOrGlobal 			= UInt32;
const
	eLocal						= 0;
	eGlobal						= 1;
	eProperties					= 2;
=======
>>>>>>> origin/fixes_2.4


{*************************************************************************
    OSA Script File Interface
*************************************************************************}
{
 *  OSAGetScriptDataFromURL()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.6 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function OSAGetScriptDataFromURL( scriptURL: CFURLRef; storable: BooleanPtr { can be NULL }; modeFlags: SInt32; var resultingScriptData: AEDesc ): OSAError; external name '_OSAGetScriptDataFromURL';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)


{
        This routine reads script data from a URL into a descriptor. The
        URL may refer to a compiled script, script application, or source
        text file. If "scriptURL" refers to a text file, the resulting data
        is the source text. If "storable" is non-NULL, it will be set to
        indicate whether a script can be stored into the script file using
        OSAStoreFile().
     
        You may use OSALoadScriptData() with the resulting descriptor to
        load the script into a component instance. Doing this in two steps
        affords the opportunity to examine the script data with
        OSAGetStorageType() and select a component instance.
    
        Errors:
            errOSASystemError
            File system errors.
            
        ModeFlags:
            No mode flags are supported at this time.
    }
{
 *  OSALoadScriptData()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.6 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function OSALoadScriptData( scriptingComponent: ComponentInstance; const (*var*) scriptData: AEDesc; fromURL: CFURLRef { can be NULL }; modeFlags: SInt32; var resultingScriptID: OSAID ): OSAError; external name '_OSALoadScriptData';
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)


{
        This routine loads script data from a descriptor into the specified
        scripting component. If "scriptData" is source, it will be compiled.
        If "fromURL" is non-NULL, it indicates the file from which the data
        was read.
    
        Errors:
            See OSALoad() and OSACompile().
            
        ModeFlags:
            See OSALoad() and OSACompile().
    }
{
 *  OSALoadFile()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function OSALoadFile( scriptingComponent: ComponentInstance; const (*var*) scriptFile: FSRef; storable: BooleanPtr { can be NULL }; modeFlags: SInt32; var resultingScriptID: OSAID ): OSAError; external name '_OSALoadFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
        This routine loads a script into the specified scripting component.
        If "scriptFile" is a text file, the script will be compiled. If
        "storable" is non-NULL, it will be set to indicate whether a
        script can be stored into the script file using OSAStoreFile().
    
        Errors:
            See OSALoad() and OSACompile().
            
        ModeFlags:
            See OSALoad() and OSACompile().
    }
{
 *  OSAStoreFile()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function OSAStoreFile( scriptingComponent: ComponentInstance; scriptID: OSAID; desiredType: DescType; modeFlags: SInt32; const (*var*) scriptFile: FSRef ): OSAError; external name '_OSAStoreFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
        This routine stores a script into the specified file.
    
        Errors:
            See OSAStore().
        
        ModeFlags:
            See OSAStore().
    }
{
 *  OSALoadExecuteFile()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function OSALoadExecuteFile( scriptingComponent: ComponentInstance; const (*var*) scriptFile: FSRef; contextID: OSAID; modeFlags: SInt32; var resultingScriptValueID: OSAID ): OSAError; external name '_OSALoadExecuteFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
        This routine is effectively equivalent to calling OSALoadFile followed by
        OSAExecute.  After execution, the compiled source is disposed.  Only the
        resulting value ID is retained.
    
        Errors:
            See OSALoadExecute() and OSACompileExecute().
    
        ModeFlags:
            See OSALoadExecute() and OSACompileExecute().
    }
{
 *  OSADoScriptFile()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function OSADoScriptFile( scriptingComponent: ComponentInstance; const (*var*) scriptFile: FSRef; contextID: OSAID; desiredType: DescType; modeFlags: SInt32; var resultingText: AEDesc ): OSAError; external name '_OSADoScriptFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
        This routine is effectively equivalent to calling OSALoadFile, followed by 
        OSAExecute, OSADisplay, and then OSAStoreFile if the script has persistent 
        properties.  After execution, the compiled source and the resulting value are 
        disposed.  Only the resultingText descriptor is retained.  If a script error 
        occurs during processing, the resultingText gets the error message of the error, 
        and errOSAScriptError is returned.  OSAScriptError may still be used to extract 
        more information about the particular error.
    
        Errors:
            See OSALoad(), OSACompile(), OSAExecute(), OSADisplay(), and OSAStore().
    
        ModeFlags:
            See OSALoad(), OSACompile(), OSAExecute(), and OSADisplay().
    }

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
