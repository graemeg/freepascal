{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       OpenScripting/ASRegistry.h
 
     Contains:   AppleScript Registry constants.
 
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
 
     Copyright:  © 1991-2008 by Apple Computer, Inc., all rights reserved
=======
=======
>>>>>>> origin/fixes_2_2
     File:       ASRegistry.p
=======
     File:       OpenScripting/ASRegistry.h
>>>>>>> origin/fixes_2.4
 
     Contains:   AppleScript Registry constants.
 
     Version:    OSA-136~14
 
<<<<<<< HEAD
     Copyright:  © 1991-2002 by Apple Computer, Inc., all rights reserved
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  © 1991-2008 by Apple Computer, Inc., all rights reserved
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

unit ASRegistry;
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
<<<<<<< HEAD
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
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
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
=======
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
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
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
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
uses MacTypes,AERegistry,AEDataModel,AEObjects;
{$endc} {not MACOSALLINCLUDE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
uses MacTypes,AERegistry,AEDataModel,AEObjects;
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}


const
	keyAETarget = FourCharCode('targ');
	keySubjectAttr = FourCharCode('subj'); { Magic 'returning' parameter: }
	keyASReturning = FourCharCode('Krtn'); { AppleScript Specific Codes: }
	kASAppleScriptSuite = FourCharCode('ascr');
	kASScriptEditorSuite = FourCharCode('ToyS');
	kASTypeNamesSuite = FourCharCode('tpnm'); { dynamic terminologies }
	typeAETE = FourCharCode('aete');
	typeAEUT = FourCharCode('aeut');
	kGetAETE = FourCharCode('gdte');
	kGetAEUT = FourCharCode('gdut');
	kUpdateAEUT = FourCharCode('udut');
	kUpdateAETE = FourCharCode('udte');
	kCleanUpAEUT = FourCharCode('cdut');
	kASComment = FourCharCode('cmnt');
	kASLaunchEvent = FourCharCode('noop');
	keyScszResource = FourCharCode('scsz');
	typeScszResource = FourCharCode('scsz'); { subroutine calls }
	kASSubroutineEvent = FourCharCode('psbr');
	keyASSubroutineName = FourCharCode('snam');
	kASPrepositionalSubroutine = FourCharCode('psbr');
	keyASPositionalArgs = FourCharCode('parg');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

const
{ Add this parameter to a Get Data result if your app handled the 'as' parameter }
	keyAppHandledCoercion = FourCharCode('idas');

const
{ Miscellaneous AppleScript commands }
	kASStartLogEvent = FourCharCode('log1');
	kASStopLogEvent = FourCharCode('log0');
	kASCommentEvent = FourCharCode('cmnt');


{ Operator Events: }
const
{ Binary: }
	kASAdd = FourCharCode('+   ');
	kASSubtract = FourCharCode('-   ');
	kASMultiply = FourCharCode('*   ');
	kASDivide = FourCharCode('/   ');
	kASQuotient = FourCharCode('div ');
	kASRemainder = FourCharCode('mod ');
	kASPower = FourCharCode('^   ');
	kASEqual = kAEEquals;
	kASNotEqual = $AD202020;
	kASGreaterThan = kAEGreaterThan;
	kASGreaterThanOrEqual = kAEGreaterThanEquals;
	kASLessThan = kAELessThan;
	kASLessThanOrEqual = kAELessThanEquals;
	kASComesBefore = FourCharCode('cbfr');
	kASComesAfter = FourCharCode('cafr');
	kASConcatenate = FourCharCode('ccat');
	kASStartsWith = kAEBeginsWith;
	kASEndsWith = kAEEndsWith;
	kASContains = kAEContains;

const
	kASAnd = kAEAND;
	kASOr = kAEOR; { Unary: }
	kASNot = kAENOT;
	kASNegate = FourCharCode('neg ');
	keyASArg = FourCharCode('arg ');

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc TARGET_OS_MAC}

{$ALIGN POWER}


const
	keyAETarget = FourCharCode('targ');
	keySubjectAttr = FourCharCode('subj'); { Magic 'returning' parameter: }
	keyASReturning = FourCharCode('Krtn'); { AppleScript Specific Codes: }
	kASAppleScriptSuite = FourCharCode('ascr');
	kASScriptEditorSuite = FourCharCode('ToyS');
	kASTypeNamesSuite = FourCharCode('tpnm'); { dynamic terminologies }
	typeAETE = FourCharCode('aete');
	typeAEUT = FourCharCode('aeut');
	kGetAETE = FourCharCode('gdte');
	kGetAEUT = FourCharCode('gdut');
	kUpdateAEUT = FourCharCode('udut');
	kUpdateAETE = FourCharCode('udte');
	kCleanUpAEUT = FourCharCode('cdut');
	kASComment = FourCharCode('cmnt');
	kASLaunchEvent = FourCharCode('noop');
	keyScszResource = FourCharCode('scsz');
	typeScszResource = FourCharCode('scsz'); { subroutine calls }
	kASSubroutineEvent = FourCharCode('psbr');
	keyASSubroutineName = FourCharCode('snam');
	kASPrepositionalSubroutine = FourCharCode('psbr');
	keyASPositionalArgs = FourCharCode('parg');

=======

>>>>>>> graemeg/cpstrnew
const
{ Add this parameter to a Get Data result if your app handled the 'as' parameter }
	keyAppHandledCoercion = FourCharCode('idas');

const
{ Miscellaneous AppleScript commands }
	kASStartLogEvent = FourCharCode('log1');
	kASStopLogEvent = FourCharCode('log0');
	kASCommentEvent = FourCharCode('cmnt');


{ Operator Events: }
const
{ Binary: }
	kASAdd = FourCharCode('+   ');
	kASSubtract = FourCharCode('-   ');
	kASMultiply = FourCharCode('*   ');
	kASDivide = FourCharCode('/   ');
	kASQuotient = FourCharCode('div ');
	kASRemainder = FourCharCode('mod ');
	kASPower = FourCharCode('^   ');
	kASEqual = kAEEquals;
	kASNotEqual = $AD202020;
	kASGreaterThan = kAEGreaterThan;
	kASGreaterThanOrEqual = kAEGreaterThanEquals;
	kASLessThan = kAELessThan;
	kASLessThanOrEqual = kAELessThanEquals;
	kASComesBefore = FourCharCode('cbfr');
	kASComesAfter = FourCharCode('cafr');
	kASConcatenate = FourCharCode('ccat');
	kASStartsWith = kAEBeginsWith;
	kASEndsWith = kAEEndsWith;
	kASContains = kAEContains;

const
	kASAnd = kAEAND;
	kASOr = kAEOR; { Unary: }
	kASNot = kAENOT;
	kASNegate = FourCharCode('neg ');
	keyASArg = FourCharCode('arg ');
<<<<<<< HEAD

const
{ event code for the 'error' statement }
	kASErrorEventCode = FourCharCode('err ');
	kOSAErrorArgs = FourCharCode('erra');
	keyAEErrorObject = FourCharCode('erob'); { Properties: }
	pLength = FourCharCode('leng');
	pReverse = FourCharCode('rvse');
	pRest = FourCharCode('rest');
	pInherits = FourCharCode('c@#^');
	pProperties = FourCharCode('pALL'); { User-Defined Record Fields: }
	keyASUserRecordFields = FourCharCode('usrf');
	typeUserRecordFields = typeAEList;

{ Prepositions: }
const
	keyASPrepositionAt = FourCharCode('at  ');
	keyASPrepositionIn = FourCharCode('in  ');
	keyASPrepositionFrom = FourCharCode('from');
	keyASPrepositionFor = FourCharCode('for ');
	keyASPrepositionTo = FourCharCode('to  ');
	keyASPrepositionThru = FourCharCode('thru');
	keyASPrepositionThrough = FourCharCode('thgh');
	keyASPrepositionBy = FourCharCode('by  ');
	keyASPrepositionOn = FourCharCode('on  ');
	keyASPrepositionInto = FourCharCode('into');
	keyASPrepositionOnto = FourCharCode('onto');
	keyASPrepositionBetween = FourCharCode('btwn');
	keyASPrepositionAgainst = FourCharCode('agst');
	keyASPrepositionOutOf = FourCharCode('outo');
	keyASPrepositionInsteadOf = FourCharCode('isto');
	keyASPrepositionAsideFrom = FourCharCode('asdf');
	keyASPrepositionAround = FourCharCode('arnd');
	keyASPrepositionBeside = FourCharCode('bsid');
	keyASPrepositionBeneath = FourCharCode('bnth');
	keyASPrepositionUnder = FourCharCode('undr');

const
	keyASPrepositionOver = FourCharCode('over');
	keyASPrepositionAbove = FourCharCode('abve');
	keyASPrepositionBelow = FourCharCode('belw');
	keyASPrepositionApartFrom = FourCharCode('aprt');
	keyASPrepositionGiven = FourCharCode('givn');
	keyASPrepositionWith = FourCharCode('with');
	keyASPrepositionWithout = FourCharCode('wout');
	keyASPrepositionAbout = FourCharCode('abou');
	keyASPrepositionSince = FourCharCode('snce');
	keyASPrepositionUntil = FourCharCode('till');

const
=======
const
{ event code for the 'error' statement }
	kASErrorEventCode = FourCharCode('err ');
	kOSAErrorArgs = FourCharCode('erra');
	keyAEErrorObject = FourCharCode('erob'); { Properties: }
	pLength = FourCharCode('leng');
	pReverse = FourCharCode('rvse');
	pRest = FourCharCode('rest');
	pInherits = FourCharCode('c@#^');
	pProperties = FourCharCode('pALL'); { User-Defined Record Fields: }
	keyASUserRecordFields = FourCharCode('usrf');
	typeUserRecordFields = typeAEList;

{ Prepositions: }
const
	keyASPrepositionAt = FourCharCode('at  ');
	keyASPrepositionIn = FourCharCode('in  ');
	keyASPrepositionFrom = FourCharCode('from');
	keyASPrepositionFor = FourCharCode('for ');
	keyASPrepositionTo = FourCharCode('to  ');
	keyASPrepositionThru = FourCharCode('thru');
	keyASPrepositionThrough = FourCharCode('thgh');
	keyASPrepositionBy = FourCharCode('by  ');
	keyASPrepositionOn = FourCharCode('on  ');
	keyASPrepositionInto = FourCharCode('into');
	keyASPrepositionOnto = FourCharCode('onto');
	keyASPrepositionBetween = FourCharCode('btwn');
	keyASPrepositionAgainst = FourCharCode('agst');
	keyASPrepositionOutOf = FourCharCode('outo');
	keyASPrepositionInsteadOf = FourCharCode('isto');
	keyASPrepositionAsideFrom = FourCharCode('asdf');
	keyASPrepositionAround = FourCharCode('arnd');
	keyASPrepositionBeside = FourCharCode('bsid');
	keyASPrepositionBeneath = FourCharCode('bnth');
	keyASPrepositionUnder = FourCharCode('undr');

const
	keyASPrepositionOver = FourCharCode('over');
	keyASPrepositionAbove = FourCharCode('abve');
	keyASPrepositionBelow = FourCharCode('belw');
	keyASPrepositionApartFrom = FourCharCode('aprt');
	keyASPrepositionGiven = FourCharCode('givn');
	keyASPrepositionWith = FourCharCode('with');
	keyASPrepositionWithout = FourCharCode('wout');
	keyASPrepositionAbout = FourCharCode('abou');
	keyASPrepositionSince = FourCharCode('snce');
	keyASPrepositionUntil = FourCharCode('till');

const
>>>>>>> graemeg/cpstrnew
{ Terminology & Dialect things: }
	kDialectBundleResType = FourCharCode('Dbdl'); { AppleScript Classes and Enums: }
	cConstant = typeEnumerated;
	cClassIdentifier = pClass;
	cObjectBeingExamined = typeObjectBeingExamined;
	cList = typeAEList;
	cSmallReal = typeIEEE32BitFloatingPoint;
	cReal = typeIEEE64BitFloatingPoint;
	cRecord = typeAERecord;
	cReference = cObjectSpecifier;
	cUndefined = FourCharCode('undf');
	cMissingValue = FourCharCode('msng');
	cSymbol = FourCharCode('symb');
	cLinkedList = FourCharCode('llst');
	cVector = FourCharCode('vect');
	cEventIdentifier = FourCharCode('evnt');
	cKeyIdentifier = FourCharCode('kyid');
	cUserIdentifier = FourCharCode('uid ');
	cPreposition = FourCharCode('prep');
	cKeyForm = enumKeyForm;
	cScript = FourCharCode('scpt');
	cHandler = FourCharCode('hand');
	cProcedure = FourCharCode('proc');

const
	cHandleBreakpoint = FourCharCode('brak');

const
	cClosure = FourCharCode('clsr');
	cRawData = FourCharCode('rdat');
	cStringClass = typeChar;
	cNumber = FourCharCode('nmbr');
	cListElement = FourCharCode('celm');
	cListOrRecord = FourCharCode('lr  ');
	cListOrString = FourCharCode('ls  ');
	cListRecordOrString = FourCharCode('lrs ');
	cNumberOrString = FourCharCode('ns  ');
	cNumberOrDateTime = FourCharCode('nd  ');
	cNumberDateTimeOrString = FourCharCode('nds ');
	cAliasOrString = FourCharCode('sf  ');
	cSeconds = FourCharCode('scnd');
	typeSound = FourCharCode('snd ');
	enumBooleanValues = FourCharCode('boov'); {  Use this instead of typeBoolean to avoid with/without conversion  }
	kAETrue = typeTrue;
	kAEFalse = typeFalse;
	enumMiscValues = FourCharCode('misc');
	kASCurrentApplication = FourCharCode('cura'); { User-defined property ospecs: }
	formUserPropertyID = FourCharCode('usrp');

const
=======

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
const
{ event code for the 'error' statement }
	kASErrorEventCode = FourCharCode('err ');
	kOSAErrorArgs = FourCharCode('erra');
	keyAEErrorObject = FourCharCode('erob'); { Properties: }
	pLength = FourCharCode('leng');
	pReverse = FourCharCode('rvse');
	pRest = FourCharCode('rest');
	pInherits = FourCharCode('c@#^');
	pProperties = FourCharCode('pALL'); { User-Defined Record Fields: }
	keyASUserRecordFields = FourCharCode('usrf');
	typeUserRecordFields = typeAEList;

{ Prepositions: }
const
	keyASPrepositionAt = FourCharCode('at  ');
	keyASPrepositionIn = FourCharCode('in  ');
	keyASPrepositionFrom = FourCharCode('from');
	keyASPrepositionFor = FourCharCode('for ');
	keyASPrepositionTo = FourCharCode('to  ');
	keyASPrepositionThru = FourCharCode('thru');
	keyASPrepositionThrough = FourCharCode('thgh');
	keyASPrepositionBy = FourCharCode('by  ');
	keyASPrepositionOn = FourCharCode('on  ');
	keyASPrepositionInto = FourCharCode('into');
	keyASPrepositionOnto = FourCharCode('onto');
	keyASPrepositionBetween = FourCharCode('btwn');
	keyASPrepositionAgainst = FourCharCode('agst');
	keyASPrepositionOutOf = FourCharCode('outo');
	keyASPrepositionInsteadOf = FourCharCode('isto');
	keyASPrepositionAsideFrom = FourCharCode('asdf');
	keyASPrepositionAround = FourCharCode('arnd');
	keyASPrepositionBeside = FourCharCode('bsid');
	keyASPrepositionBeneath = FourCharCode('bnth');
	keyASPrepositionUnder = FourCharCode('undr');

const
	keyASPrepositionOver = FourCharCode('over');
	keyASPrepositionAbove = FourCharCode('abve');
	keyASPrepositionBelow = FourCharCode('belw');
	keyASPrepositionApartFrom = FourCharCode('aprt');
	keyASPrepositionGiven = FourCharCode('givn');
	keyASPrepositionWith = FourCharCode('with');
	keyASPrepositionWithout = FourCharCode('wout');
	keyASPrepositionAbout = FourCharCode('abou');
	keyASPrepositionSince = FourCharCode('snce');
	keyASPrepositionUntil = FourCharCode('till');

const
{ Terminology & Dialect things: }
	kDialectBundleResType = FourCharCode('Dbdl'); { AppleScript Classes and Enums: }
	cConstant = typeEnumerated;
	cClassIdentifier = pClass;
	cObjectBeingExamined = typeObjectBeingExamined;
	cList = typeAEList;
	cSmallReal = typeIEEE32BitFloatingPoint;
	cReal = typeIEEE64BitFloatingPoint;
	cRecord = typeAERecord;
	cReference = cObjectSpecifier;
	cUndefined = FourCharCode('undf');
	cMissingValue = FourCharCode('msng');
	cSymbol = FourCharCode('symb');
	cLinkedList = FourCharCode('llst');
	cVector = FourCharCode('vect');
	cEventIdentifier = FourCharCode('evnt');
	cKeyIdentifier = FourCharCode('kyid');
	cUserIdentifier = FourCharCode('uid ');
	cPreposition = FourCharCode('prep');
	cKeyForm = enumKeyForm;
	cScript = FourCharCode('scpt');
	cHandler = FourCharCode('hand');
	cProcedure = FourCharCode('proc');

const
	cHandleBreakpoint = FourCharCode('brak');

const
	cClosure = FourCharCode('clsr');
	cRawData = FourCharCode('rdat');
	cStringClass = typeChar;
	cNumber = FourCharCode('nmbr');
	cListElement = FourCharCode('celm');
	cListOrRecord = FourCharCode('lr  ');
	cListOrString = FourCharCode('ls  ');
	cListRecordOrString = FourCharCode('lrs ');
	cNumberOrString = FourCharCode('ns  ');
	cNumberOrDateTime = FourCharCode('nd  ');
	cNumberDateTimeOrString = FourCharCode('nds ');
	cAliasOrString = FourCharCode('sf  ');
	cSeconds = FourCharCode('scnd');
	typeSound = FourCharCode('snd ');
	enumBooleanValues = FourCharCode('boov'); {  Use this instead of typeBoolean to avoid with/without conversion  }
	kAETrue = typeTrue;
	kAEFalse = typeFalse;
	enumMiscValues = FourCharCode('misc');
	kASCurrentApplication = FourCharCode('cura'); { User-defined property ospecs: }
	formUserPropertyID = FourCharCode('usrp');

const
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	cString = cStringClass; { old name for cStringClass - can't be used in .r files}

const
{ Global properties: }
	pASIt = FourCharCode('it  ');
	pASMe = FourCharCode('me  ');
	pASResult = FourCharCode('rslt');
	pASSpace = FourCharCode('spac');
	pASReturn = FourCharCode('ret ');
	pASTab = FourCharCode('tab ');
	pASPi = FourCharCode('pi  ');
	pASParent = FourCharCode('pare');
	kASInitializeEventCode = FourCharCode('init');
	pASPrintLength = FourCharCode('prln');
	pASPrintDepth = FourCharCode('prdp');
	pASTopLevelScript = FourCharCode('ascr');

const
{ Considerations }
	kAECase = FourCharCode('case');
	kAEDiacritic = FourCharCode('diac');
	kAEWhiteSpace = FourCharCode('whit');
	kAEHyphens = FourCharCode('hyph');
	kAEExpansion = FourCharCode('expa');
	kAEPunctuation = FourCharCode('punc');
	kAEZenkakuHankaku = FourCharCode('zkhk');
	kAESmallKana = FourCharCode('skna');
	kAEKataHiragana = FourCharCode('hika');
	kASConsiderReplies = FourCharCode('rmte');
	kASNumericStrings = FourCharCode('nume');
	enumConsiderations = FourCharCode('cons');

{ Considerations bit masks }
const
	kAECaseConsiderMask = $00000001;
	kAEDiacriticConsiderMask = $00000002;
	kAEWhiteSpaceConsiderMask = $00000004;
	kAEHyphensConsiderMask = $00000008;
	kAEExpansionConsiderMask = $00000010;
	kAEPunctuationConsiderMask = $00000020;
	kASConsiderRepliesConsiderMask = $00000040;
	kASNumericStringsConsiderMask = $00000080;
	kAECaseIgnoreMask = $00010000;
	kAEDiacriticIgnoreMask = $00020000;
	kAEWhiteSpaceIgnoreMask = $00040000;
	kAEHyphensIgnoreMask = $00080000;
	kAEExpansionIgnoreMask = $00100000;
	kAEPunctuationIgnoreMask = $00200000;
	kASConsiderRepliesIgnoreMask = $00400000;
	kASNumericStringsIgnoreMask = $00800000;
	enumConsidsAndIgnores = FourCharCode('csig');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

const
	cCoercion = FourCharCode('coec');
	cCoerceUpperCase = FourCharCode('txup');
	cCoerceLowerCase = FourCharCode('txlo');
	cCoerceRemoveDiacriticals = FourCharCode('txdc');
	cCoerceRemovePunctuation = FourCharCode('txpc');
	cCoerceRemoveHyphens = FourCharCode('txhy');
	cCoerceOneByteToTwoByte = FourCharCode('txex');
	cCoerceRemoveWhiteSpace = FourCharCode('txws');
	cCoerceSmallKana = FourCharCode('txsk');
	cCoerceZenkakuhankaku = FourCharCode('txze');
	cCoerceKataHiragana = FourCharCode('txkh'); { Lorax things: }
	cZone = FourCharCode('zone');
	cMachine = FourCharCode('mach');
	cAddress = FourCharCode('addr');
	cRunningAddress = FourCharCode('radd');
	cStorage = FourCharCode('stor');

const
=======
=======
>>>>>>> origin/cpstrnew

const
	cCoercion = FourCharCode('coec');
	cCoerceUpperCase = FourCharCode('txup');
	cCoerceLowerCase = FourCharCode('txlo');
	cCoerceRemoveDiacriticals = FourCharCode('txdc');
	cCoerceRemovePunctuation = FourCharCode('txpc');
	cCoerceRemoveHyphens = FourCharCode('txhy');
	cCoerceOneByteToTwoByte = FourCharCode('txex');
	cCoerceRemoveWhiteSpace = FourCharCode('txws');
	cCoerceSmallKana = FourCharCode('txsk');
	cCoerceZenkakuhankaku = FourCharCode('txze');
	cCoerceKataHiragana = FourCharCode('txkh'); { Lorax things: }
	cZone = FourCharCode('zone');
	cMachine = FourCharCode('mach');
	cAddress = FourCharCode('addr');
	cRunningAddress = FourCharCode('radd');
	cStorage = FourCharCode('stor');

const
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======

const
	cCoercion = FourCharCode('coec');
	cCoerceUpperCase = FourCharCode('txup');
	cCoerceLowerCase = FourCharCode('txlo');
	cCoerceRemoveDiacriticals = FourCharCode('txdc');
	cCoerceRemovePunctuation = FourCharCode('txpc');
	cCoerceRemoveHyphens = FourCharCode('txhy');
	cCoerceOneByteToTwoByte = FourCharCode('txex');
	cCoerceRemoveWhiteSpace = FourCharCode('txws');
	cCoerceSmallKana = FourCharCode('txsk');
	cCoerceZenkakuhankaku = FourCharCode('txze');
	cCoerceKataHiragana = FourCharCode('txkh'); { Lorax things: }
	cZone = FourCharCode('zone');
	cMachine = FourCharCode('mach');
	cAddress = FourCharCode('addr');
	cRunningAddress = FourCharCode('radd');
	cStorage = FourCharCode('stor');

const
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{ DateTime things: }
	pASWeekday = FourCharCode('wkdy');
	pASMonth = FourCharCode('mnth');
	pASDay = FourCharCode('day ');
	pASYear = FourCharCode('year');
	pASTime = FourCharCode('time');
	pASDateString = FourCharCode('dstr');
	pASTimeString = FourCharCode('tstr'); { Months }
	cMonth = pASMonth;
	cJanuary = FourCharCode('jan ');
	cFebruary = FourCharCode('feb ');
	cMarch = FourCharCode('mar ');
	cApril = FourCharCode('apr ');
	cMay = FourCharCode('may ');
	cJune = FourCharCode('jun ');
	cJuly = FourCharCode('jul ');
	cAugust = FourCharCode('aug ');
	cSeptember = FourCharCode('sep ');
	cOctober = FourCharCode('oct ');
	cNovember = FourCharCode('nov ');
	cDecember = FourCharCode('dec ');

const
{ Weekdays }
	cWeekday = pASWeekday;
	cSunday = FourCharCode('sun ');
	cMonday = FourCharCode('mon ');
	cTuesday = FourCharCode('tue ');
	cWednesday = FourCharCode('wed ');
	cThursday = FourCharCode('thu ');
	cFriday = FourCharCode('fri ');
	cSaturday = FourCharCode('sat '); { AS 1.1 Globals: }
	pASQuote = FourCharCode('quot');
	pASSeconds = FourCharCode('secs');
	pASMinutes = FourCharCode('min ');
	pASHours = FourCharCode('hour');
	pASDays = FourCharCode('days');
	pASWeeks = FourCharCode('week'); { Writing Code things: }
	cWritingCodeInfo = FourCharCode('citl');
	pScriptCode = FourCharCode('pscd');
	pLangCode = FourCharCode('plcd'); { Magic Tell and End Tell events for logging: }
	kASMagicTellEvent = FourCharCode('tell');
	kASMagicEndTellEvent = FourCharCode('tend');

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
uses MacTypes,AERegistry,AEObjects;
=======
>>>>>>> origin/fixes_2.4

const
{ Add this parameter to a Get Data result if your app handled the 'as' parameter }
	keyAppHandledCoercion = FourCharCode('idas');

const
{ Miscellaneous AppleScript commands }
	kASStartLogEvent = FourCharCode('log1');
	kASStopLogEvent = FourCharCode('log0');
	kASCommentEvent = FourCharCode('cmnt');


{ Operator Events: }
const
{ Binary: }
	kASAdd = FourCharCode('+   ');
	kASSubtract = FourCharCode('-   ');
	kASMultiply = FourCharCode('*   ');
	kASDivide = FourCharCode('/   ');
	kASQuotient = FourCharCode('div ');
	kASRemainder = FourCharCode('mod ');
	kASPower = FourCharCode('^   ');
	kASEqual = kAEEquals;
	kASNotEqual = $AD202020;
	kASGreaterThan = kAEGreaterThan;
	kASGreaterThanOrEqual = kAEGreaterThanEquals;
	kASLessThan = kAELessThan;
	kASLessThanOrEqual = kAELessThanEquals;
	kASComesBefore = FourCharCode('cbfr');
	kASComesAfter = FourCharCode('cafr');
	kASConcatenate = FourCharCode('ccat');
	kASStartsWith = kAEBeginsWith;
	kASEndsWith = kAEEndsWith;
	kASContains = kAEContains;

const
	kASAnd = kAEAND;
	kASOr = kAEOR; { Unary: }
	kASNot = kAENOT;
	kASNegate = FourCharCode('neg ');
	keyASArg = FourCharCode('arg ');

const
{ event code for the 'error' statement }
	kASErrorEventCode = FourCharCode('err ');
	kOSAErrorArgs = FourCharCode('erra');
	keyAEErrorObject = FourCharCode('erob'); { Properties: }
	pLength = FourCharCode('leng');
	pReverse = FourCharCode('rvse');
	pRest = FourCharCode('rest');
	pInherits = FourCharCode('c@#^');
	pProperties = FourCharCode('pALL'); { User-Defined Record Fields: }
	keyASUserRecordFields = FourCharCode('usrf');
	typeUserRecordFields = typeAEList;

{ Prepositions: }
const
	keyASPrepositionAt = FourCharCode('at  ');
	keyASPrepositionIn = FourCharCode('in  ');
	keyASPrepositionFrom = FourCharCode('from');
	keyASPrepositionFor = FourCharCode('for ');
	keyASPrepositionTo = FourCharCode('to  ');
	keyASPrepositionThru = FourCharCode('thru');
	keyASPrepositionThrough = FourCharCode('thgh');
	keyASPrepositionBy = FourCharCode('by  ');
	keyASPrepositionOn = FourCharCode('on  ');
	keyASPrepositionInto = FourCharCode('into');
	keyASPrepositionOnto = FourCharCode('onto');
	keyASPrepositionBetween = FourCharCode('btwn');
	keyASPrepositionAgainst = FourCharCode('agst');
	keyASPrepositionOutOf = FourCharCode('outo');
	keyASPrepositionInsteadOf = FourCharCode('isto');
	keyASPrepositionAsideFrom = FourCharCode('asdf');
	keyASPrepositionAround = FourCharCode('arnd');
	keyASPrepositionBeside = FourCharCode('bsid');
	keyASPrepositionBeneath = FourCharCode('bnth');
	keyASPrepositionUnder = FourCharCode('undr');

const
	keyASPrepositionOver = FourCharCode('over');
	keyASPrepositionAbove = FourCharCode('abve');
	keyASPrepositionBelow = FourCharCode('belw');
	keyASPrepositionApartFrom = FourCharCode('aprt');
	keyASPrepositionGiven = FourCharCode('givn');
	keyASPrepositionWith = FourCharCode('with');
	keyASPrepositionWithout = FourCharCode('wout');
	keyASPrepositionAbout = FourCharCode('abou');
	keyASPrepositionSince = FourCharCode('snce');
	keyASPrepositionUntil = FourCharCode('till');

const
{ Terminology & Dialect things: }
	kDialectBundleResType = FourCharCode('Dbdl'); { AppleScript Classes and Enums: }
	cConstant = typeEnumerated;
	cClassIdentifier = pClass;
	cObjectBeingExamined = typeObjectBeingExamined;
	cList = typeAEList;
	cSmallReal = typeIEEE32BitFloatingPoint;
	cReal = typeIEEE64BitFloatingPoint;
	cRecord = typeAERecord;
	cReference = cObjectSpecifier;
	cUndefined = FourCharCode('undf');
	cMissingValue = FourCharCode('msng');
	cSymbol = FourCharCode('symb');
	cLinkedList = FourCharCode('llst');
	cVector = FourCharCode('vect');
	cEventIdentifier = FourCharCode('evnt');
	cKeyIdentifier = FourCharCode('kyid');
	cUserIdentifier = FourCharCode('uid ');
	cPreposition = FourCharCode('prep');
	cKeyForm = enumKeyForm;
	cScript = FourCharCode('scpt');
	cHandler = FourCharCode('hand');
	cProcedure = FourCharCode('proc');

const
	cHandleBreakpoint = FourCharCode('brak');

const
	cClosure = FourCharCode('clsr');
	cRawData = FourCharCode('rdat');
	cStringClass = typeChar;
	cNumber = FourCharCode('nmbr');
	cListElement = FourCharCode('celm');
	cListOrRecord = FourCharCode('lr  ');
	cListOrString = FourCharCode('ls  ');
	cListRecordOrString = FourCharCode('lrs ');
	cNumberOrString = FourCharCode('ns  ');
	cNumberOrDateTime = FourCharCode('nd  ');
	cNumberDateTimeOrString = FourCharCode('nds ');
	cAliasOrString = FourCharCode('sf  ');
	cSeconds = FourCharCode('scnd');
	typeSound = FourCharCode('snd ');
	enumBooleanValues = FourCharCode('boov'); {  Use this instead of typeBoolean to avoid with/without conversion  }
	kAETrue = typeTrue;
	kAEFalse = typeFalse;
	enumMiscValues = FourCharCode('misc');
	kASCurrentApplication = FourCharCode('cura'); { User-defined property ospecs: }
	formUserPropertyID = FourCharCode('usrp');

const
	cString = cStringClass; { old name for cStringClass - can't be used in .r files}

const
{ Global properties: }
	pASIt = FourCharCode('it  ');
	pASMe = FourCharCode('me  ');
	pASResult = FourCharCode('rslt');
	pASSpace = FourCharCode('spac');
	pASReturn = FourCharCode('ret ');
	pASTab = FourCharCode('tab ');
	pASPi = FourCharCode('pi  ');
	pASParent = FourCharCode('pare');
	kASInitializeEventCode = FourCharCode('init');
	pASPrintLength = FourCharCode('prln');
	pASPrintDepth = FourCharCode('prdp');
	pASTopLevelScript = FourCharCode('ascr');

const
{ Considerations }
	kAECase = FourCharCode('case');
	kAEDiacritic = FourCharCode('diac');
	kAEWhiteSpace = FourCharCode('whit');
	kAEHyphens = FourCharCode('hyph');
	kAEExpansion = FourCharCode('expa');
	kAEPunctuation = FourCharCode('punc');
	kAEZenkakuHankaku = FourCharCode('zkhk');
	kAESmallKana = FourCharCode('skna');
	kAEKataHiragana = FourCharCode('hika');
	kASConsiderReplies = FourCharCode('rmte');
	kASNumericStrings = FourCharCode('nume');
	enumConsiderations = FourCharCode('cons');

{ Considerations bit masks }
const
	kAECaseConsiderMask = $00000001;
	kAEDiacriticConsiderMask = $00000002;
	kAEWhiteSpaceConsiderMask = $00000004;
	kAEHyphensConsiderMask = $00000008;
	kAEExpansionConsiderMask = $00000010;
	kAEPunctuationConsiderMask = $00000020;
	kASConsiderRepliesConsiderMask = $00000040;
	kASNumericStringsConsiderMask = $00000080;
	kAECaseIgnoreMask = $00010000;
	kAEDiacriticIgnoreMask = $00020000;
	kAEWhiteSpaceIgnoreMask = $00040000;
	kAEHyphensIgnoreMask = $00080000;
	kAEExpansionIgnoreMask = $00100000;
	kAEPunctuationIgnoreMask = $00200000;
	kASConsiderRepliesIgnoreMask = $00400000;
	kASNumericStringsIgnoreMask = $00800000;
	enumConsidsAndIgnores = FourCharCode('csig');

const
	cCoercion = FourCharCode('coec');
	cCoerceUpperCase = FourCharCode('txup');
	cCoerceLowerCase = FourCharCode('txlo');
	cCoerceRemoveDiacriticals = FourCharCode('txdc');
	cCoerceRemovePunctuation = FourCharCode('txpc');
	cCoerceRemoveHyphens = FourCharCode('txhy');
	cCoerceOneByteToTwoByte = FourCharCode('txex');
	cCoerceRemoveWhiteSpace = FourCharCode('txws');
	cCoerceSmallKana = FourCharCode('txsk');
	cCoerceZenkakuhankaku = FourCharCode('txze');
	cCoerceKataHiragana = FourCharCode('txkh'); { Lorax things: }
	cZone = FourCharCode('zone');
	cMachine = FourCharCode('mach');
	cAddress = FourCharCode('addr');
	cRunningAddress = FourCharCode('radd');
	cStorage = FourCharCode('stor');

const
{ DateTime things: }
	pASWeekday = FourCharCode('wkdy');
	pASMonth = FourCharCode('mnth');
	pASDay = FourCharCode('day ');
	pASYear = FourCharCode('year');
	pASTime = FourCharCode('time');
	pASDateString = FourCharCode('dstr');
	pASTimeString = FourCharCode('tstr'); { Months }
	cMonth = pASMonth;
	cJanuary = FourCharCode('jan ');
	cFebruary = FourCharCode('feb ');
	cMarch = FourCharCode('mar ');
	cApril = FourCharCode('apr ');
	cMay = FourCharCode('may ');
	cJune = FourCharCode('jun ');
	cJuly = FourCharCode('jul ');
	cAugust = FourCharCode('aug ');
	cSeptember = FourCharCode('sep ');
	cOctober = FourCharCode('oct ');
	cNovember = FourCharCode('nov ');
	cDecember = FourCharCode('dec ');

const
{ Weekdays }
	cWeekday = pASWeekday;
	cSunday = FourCharCode('sun ');
	cMonday = FourCharCode('mon ');
	cTuesday = FourCharCode('tue ');
	cWednesday = FourCharCode('wed ');
	cThursday = FourCharCode('thu ');
	cFriday = FourCharCode('fri ');
	cSaturday = FourCharCode('sat '); { AS 1.1 Globals: }
	pASQuote = FourCharCode('quot');
	pASSeconds = FourCharCode('secs');
	pASMinutes = FourCharCode('min ');
	pASHours = FourCharCode('hour');
	pASDays = FourCharCode('days');
	pASWeeks = FourCharCode('week'); { Writing Code things: }
	cWritingCodeInfo = FourCharCode('citl');
	pScriptCode = FourCharCode('pscd');
	pLangCode = FourCharCode('plcd'); { Magic Tell and End Tell events for logging: }
	kASMagicTellEvent = FourCharCode('tell');
	kASMagicEndTellEvent = FourCharCode('tend');

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
