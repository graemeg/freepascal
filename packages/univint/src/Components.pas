{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       CarbonCore/Components.h
 
     Contains:   Component Manager Interfaces.
                 The contents of this header file are deprecated.
                 Use Foundation or CoreFoundation bundles and plugins instead.
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Copyright:  © 1991-2011 by Apple Inc. All rights reserved.
}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
     Version:    CarbonCore-859.2~1
 
     Copyright:  © 1991-2008 by Apple Computer, Inc., all rights reserved.
=======
=======
>>>>>>> origin/fixes_2_2
     File:       Components.p
=======
     File:       CarbonCore/Components.h
>>>>>>> origin/fixes_2.4
 
     Contains:   Component Manager Interfaces.
 
     Version:    CarbonCore-859.2~1
 
<<<<<<< HEAD
     Copyright:  © 1991-2002 by Apple Computer, Inc., all rights reserved.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  © 1991-2008 by Apple Computer, Inc., all rights reserved.
>>>>>>> origin/fixes_2.4
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
                     http://www.freepascal.org/bugs.html
 
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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


=======
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> origin/fixes_2.4
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

unit Components;
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
=======
>>>>>>> graemeg/cpstrnew
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
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/cpstrnew
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
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
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
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
uses MacTypes,MacErrors,Files,MixedMode,Resources;
{$endc} {not MACOSALLINCLUDE}
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/cpstrnew

=======
>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}

{$ifc TARGET_OS_MAC}

{$ifc TARGET_OS_MAC}
=======
uses MacTypes,MacErrors,MixedMode,Files;

>>>>>>> graemeg/fixes_2_2
=======
uses MacTypes,MacErrors,MixedMode,Files;
=======
uses MacTypes,MacErrors,Files,MixedMode,Resources;
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4

>>>>>>> origin/fixes_2_2

{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}


const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
	kAppleManufacturer = FourCharCode('appl'); { Apple supplied components }
	kComponentResourceType = FourCharCode('thng'); { a components resource type }
	kComponentAliasResourceType = FourCharCode('thga'); { component alias resource type }

const
	kAnyComponentType = 0;
	kAnyComponentSubType = 0;
	kAnyComponentManufacturer = 0;
	kAnyComponentFlagsMask = 0;

const
	cmpThreadSafe = 1 shl 28; { component is thread-safe }
	cmpIsMissing = 1 shl 29;
	cmpWantsRegisterMessage = 1 shl 31;

const
	kComponentOpenSelect = -1;   { ComponentInstance for this open }
	kComponentCloseSelect = -2;   { ComponentInstance for this close }
	kComponentCanDoSelect = -3;   { selector # being queried }
	kComponentVersionSelect = -4;   { no params }
	kComponentRegisterSelect = -5;   { no params }
	kComponentTargetSelect = -6;   { ComponentInstance for top of call chain }
	kComponentUnregisterSelect = -7;   { no params }
	kComponentGetMPWorkFunctionSelect = -8; { some params }
	kComponentExecuteWiredActionSelect = -9; { QTAtomContainer actionContainer, QTAtom actionAtom, QTCustomActionTargetPtr target, QTEventRecordPtr event }
	kComponentGetPublicResourceSelect = -10; { OSType resourceType, short resourceId, Handle *resource }

{ Component Resource Extension flags }
const
	componentDoAutoVersion = 1 shl 0;
	componentWantsUnregister = 1 shl 1;
	componentAutoVersionIncludeFlags = 1 shl 2;
	componentHasMultiplePlatforms = 1 shl 3;
	componentLoadResident = 1 shl 4;


{ Set Default Component flags }
const
	defaultComponentIdentical = 0;
	defaultComponentAnyFlags = 1;
<<<<<<< HEAD
	defaultComponentAnyManufacturer = 2;
	defaultComponentAnySubType = 4;
	defaultComponentAnyFlagsAnyManufacturer = defaultComponentAnyFlags + defaultComponentAnyManufacturer;
	defaultComponentAnyFlagsAnyManufacturerAnySubType = defaultComponentAnyFlags + defaultComponentAnyManufacturer + defaultComponentAnySubType;

{ RegisterComponentResource flags }
const
	registerComponentGlobal = 1;
=======
=======
>>>>>>> origin/fixes_2_2
	kAppleManufacturer			= FourCharCode('appl');						{  Apple supplied components  }
	kComponentResourceType		= FourCharCode('thng');						{  a components resource type  }
	kComponentAliasResourceType	= FourCharCode('thga');						{  component alias resource type  }

	kAnyComponentType			= 0;
	kAnyComponentSubType		= 0;
	kAnyComponentManufacturer	= 0;
	kAnyComponentFlagsMask		= 0;

	cmpIsMissing				= $20000000;
	cmpWantsRegisterMessage		= $80000000;

	kComponentOpenSelect		= -1;							{  ComponentInstance for this open  }
	kComponentCloseSelect		= -2;							{  ComponentInstance for this close  }
	kComponentCanDoSelect		= -3;							{  selector # being queried  }
	kComponentVersionSelect		= -4;							{  no params  }
	kComponentRegisterSelect	= -5;							{  no params  }
	kComponentTargetSelect		= -6;							{  ComponentInstance for top of call chain  }
	kComponentUnregisterSelect	= -7;							{  no params  }
	kComponentGetMPWorkFunctionSelect = -8;						{  some params  }
	kComponentExecuteWiredActionSelect = -9;					{  QTAtomContainer actionContainer, QTAtom actionAtom, QTCustomActionTargetPtr target, QTEventRecordPtr event  }
	kComponentGetPublicResourceSelect = -10;					{  OSType resourceType, short resourceId, Handle *resource  }

	{	 Component Resource Extension flags 	}
	componentDoAutoVersion		= $01;
	componentWantsUnregister	= $02;
	componentAutoVersionIncludeFlags = $04;
	componentHasMultiplePlatforms = $08;
	componentLoadResident		= $10;


	{	 Set Default Component flags 	}
	defaultComponentIdentical	= 0;
	defaultComponentAnyFlags	= 1;
=======
>>>>>>> origin/fixes_2.4
	defaultComponentAnyManufacturer = 2;
	defaultComponentAnySubType = 4;
	defaultComponentAnyFlagsAnyManufacturer = defaultComponentAnyFlags + defaultComponentAnyManufacturer;
	defaultComponentAnyFlagsAnyManufacturerAnySubType = defaultComponentAnyFlags + defaultComponentAnyManufacturer + defaultComponentAnySubType;

<<<<<<< HEAD
	{	 RegisterComponentResource flags 	}
	registerComponentGlobal		= 1;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ RegisterComponentResource flags }
const
	registerComponentGlobal = 1;
>>>>>>> origin/fixes_2.4
	registerComponentNoDuplicates = 2;
	registerComponentAfterExisting = 4;
	registerComponentAliasesOnly = 8;


type
	ComponentDescriptionPtr = ^ComponentDescription;
	ComponentDescription = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		componentType: OSType;          { A unique 4-byte code indentifying the command set }
		componentSubType: OSType;       { Particular flavor of this instance }
		componentManufacturer: OSType;  { Vendor indentification }
		componentFlags: UInt32;         { 8 each for Component,Type,SubType,Manuf/revision }
		componentFlagsMask: UInt32;     { Mask for specifying which flags to consider in search, zero during registration }
<<<<<<< HEAD
	end;

type
	ResourceSpecPtr = ^ResourceSpec;
	ResourceSpec = record
		resType: OSType;                { 4-byte code    }
		resID: SInt16;                  {         }
	end;
type
	ComponentResource = record
		cd: ComponentDescription;                   { Registration parameters }
		component: ResourceSpec;              { resource where Component code is found }
		componentName: ResourceSpec;          { name string resource }
		componentInfo: ResourceSpec;          { info string resource }
		componentIcon: ResourceSpec;          { icon resource }
	end;
	ComponentResourcePtr = ^ComponentResource;
type
	ComponentResourceHandle = ^ComponentResourcePtr;
	ComponentPlatformInfo = record
		componentFlags: SInt32;         { flags of Component }
		component: ResourceSpec;              { resource where Component code is found }
		platformType: SInt16;           { gestaltSysArchitecture result }
	end;
type
	ComponentResourceExtensionPtr = ^ComponentResourceExtension;
	ComponentResourceExtension = record
		componentVersion: SInt32;       { version of Component }
		componentRegisterFlags: SInt32; { flags for registration }
		componentIconFamily: SInt16;    { resource id of Icon Family }
	end;
type
	ComponentPlatformInfoArrayPtr = ^ComponentPlatformInfoArray;
	ComponentPlatformInfoArray = record
		count: SInt32;
		platformArray: array [0..0] of ComponentPlatformInfo;
	end;
type
	ExtComponentResource = record
		cd: ComponentDescription;                   { registration parameters }
		component: ResourceSpec;              { resource where Component code is found }
		componentName: ResourceSpec;          { name string resource }
		componentInfo: ResourceSpec;          { info string resource }
		componentIcon: ResourceSpec;          { icon resource }
		componentVersion: SInt32;       { version of Component }
		componentRegisterFlags: SInt32; { flags for registration }
		componentIconFamily: SInt16;    { resource id of Icon Family }
		count: SInt32;                  { elements in platformArray }
    platformArray: array [0..0] of ComponentPlatformInfo;
	end;
	ExtComponentResourcePtr = ^ExtComponentResource;
type
	ExtComponentResourceHandle = ^ExtComponentResourcePtr;
	ComponentAliasResource = record
		cr: ComponentResource;                     { Registration parameters }
		aliasCD: ComponentDescription;              { component alias description }
	end;
{  Structure received by Component:        }
type
	ComponentParametersPtr = ^ComponentParameters;
	ComponentParameters = record
		flags: UInt8;                  { call modifiers: sync/async, deferred, immed, etc }
		paramSize: UInt8;              { size in bytes of actual parameters passed to this call }
		what: SInt16;                   { routine selector, negative for Component management calls }
{$ifc TARGET_CPU_64}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

		padding: UInt32;
{$endc} {TARGET_CPU_64}

<<<<<<< HEAD
<<<<<<< HEAD
=======

		padding: UInt32;
{$endc} {TARGET_CPU_64}

>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  params: array[0..0] of SIGNEDLONG;             { actual parameters for the indicated routine }
	end;
type
	ComponentRecord = record
		data: array [0..0] of SIGNEDLONG;
	end;
type
	Component = ^ComponentRecord;
	ComponentInstanceRecordPtr = ^ComponentInstanceRecord;
	ComponentInstanceRecord = record
		data: array [0..0] of SIGNEDLONG;
	end;
type
	ComponentInstance = ^ComponentInstanceRecord;
	RegisteredComponentRecord = record
		data: array [0..0] of SIGNEDLONG;
	end;
type
	RegisteredComponentRecordPtr = ^RegisteredComponentRecord;
type
	RegisteredComponentInstanceRecord = record
		data: array [0..0] of SIGNEDLONG;
	end;
	RegisteredComponentInstanceRecordPtr = ^RegisteredComponentInstanceRecord;
type
	ComponentResult = SInt32;
const
	platform68k = 1;    { platform type (response from gestaltComponentPlatform) }
	platformPowerPC = 2;    { (when gestaltComponentPlatform is not implemented, use }
	platformInterpreted = 3;    { gestaltSysArchitecture) }
	platformWin32 = 4;
	platformPowerPCNativeEntryPoint = 5;
	platformIA32NativeEntryPoint = 6;
	platformPowerPC64NativeEntryPoint = 7;
	platformX86_64NativeEntryPoint = 8;

const
	platformIRIXmips = 1000;
	platformSunOSsparc = 1100;
	platformSunOSintel = 1101;
	platformLinuxppc = 1200;
	platformLinuxintel = 1201;
	platformAIXppc = 1300;
	platformNeXTIntel = 1400;
	platformNeXTppc = 1401;
	platformNeXTsparc = 1402;
	platformNeXT68k = 1403;
	platformMacOSx86 = 1500;

const
	mpWorkFlagDoWork = 1 shl 0;
	mpWorkFlagDoCompletion = 1 shl 1;
	mpWorkFlagCopyWorkBlock = 1 shl 2;
	mpWorkFlagDontBlock = 1 shl 3;
	mpWorkFlagGetProcessorCount = 1 shl 4;
	mpWorkFlagGetIsRunning = 1 shl 6;

const
	cmpAliasNoFlags = 0;
	cmpAliasOnlyThisFile = 1;

type
	CSComponentsThreadMode = UInt32;
const
	kCSAcceptAllComponentsMode = 0;
	kCSAcceptThreadSafeComponentsOnlyMode = 1;

{
 *  CSSetComponentsThreadMode()
 *  
 *  Summary:
 *    Set whether or not using thread-unsafe components is allowed on
 *    the current thread.
 *  
 *  Discussion:
 *    When set to kCSAcceptThreadSafeComponentsOnlyMode, the current
 *    thread can only make thread-safe calls. Applications and other
 *    high-level code that wants to call QuickTime (and other) APIs
 *    from preemptive threads should call  SetComponentsThreadMode(
 *    kCSAcceptThreadSafeComponentsOnlyMode );  from their thread
 *    beforehand. The safeguard flag should only be left
 *    kCSAcceptAllComponentsMode for the main thread and other threads
 *    that participate in cooperative locking with it (such as the
 *    Carbon Thread Manager-style cooperative threads and application 
 *    threads that perform private locking).
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    mode:
 *      The thread-safety mode in current thread.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
procedure CSSetComponentsThreadMode( mode: CSComponentsThreadMode ); external name '_CSSetComponentsThreadMode';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_3, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/cpstrnew


{
 *  CSGetComponentsThreadMode()
 *  
 *  Summary:
 *    Get the current thread's thread-safety mode.
 *  
 *  Discussion:
 *    Returns kCSAcceptThreadSafeComponentsOnlyMode if only thread-safe
 *    components are allowed in current thread and
 *    kCSAcceptAllComponentsMode if all components are accepted
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function CSGetComponentsThreadMode: CSComponentsThreadMode; external name '_CSGetComponentsThreadMode';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_3, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/cpstrnew


type
	ComponentMPWorkFunctionHeaderRecord = record
		headerSize: UInt32;
		recordSize: UInt32;
		workFlags: UInt32;
		processorCount: UInt16;
		unused: UInt8;
		isRunning: UInt8;
	end;
	ComponentMPWorkFunctionHeaderRecordPtr = ^ComponentMPWorkFunctionHeaderRecord;
type
	ComponentMPWorkFunctionProcPtr = function( globalRefCon: UnivPtr; header: ComponentMPWorkFunctionHeaderRecordPtr ): ComponentResult;
	ComponentRoutineProcPtr = function( var cp: ComponentParameters; componentStorage: Handle ): ComponentResult;
	GetMissingComponentResourceProcPtr = function( c: Component; resType: OSType; resID: SInt16; refCon: UnivPtr; var resource: Handle ): OSErr;
	ComponentMPWorkFunctionUPP = ComponentMPWorkFunctionProcPtr;
	ComponentRoutineUPP = ComponentRoutineProcPtr;
	GetMissingComponentResourceUPP = GetMissingComponentResourceProcPtr;
{
    The parameter list for each ComponentFunction is unique. It is
    therefore up to users to create the appropriate procInfo for their
    own ComponentFunctions where necessary.
}
type
	ComponentFunctionUPP = UniversalProcPtr;
{
 *  NewComponentFunctionUPP()
 *  
 *  Discussion:
 *    For use in writing a Carbon compliant Component.  It is used to
 *    create a ComponentFunctionUPP needed to call
 *    CallComponentFunction in the Components dispatch routine.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewComponentFunctionUPP( userRoutine: ProcPtr; procInfo: ProcInfoType ): ComponentFunctionUPP; external name '_NewComponentFunctionUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


{
 *  DisposeComponentFunctionUPP()
 *  
 *  Discussion:
 *    For use in writing a Carbon compliant Component.  It is used to
 *    dispose of a ComponentFunctionUPP created by
 *    NewComponentFunctionUPP.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeComponentFunctionUPP( userUPP: ComponentFunctionUPP ); external name '_DisposeComponentFunctionUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
		componentType:			OSType;									{  A unique 4-byte code indentifying the command set  }
		componentSubType:		OSType;									{  Particular flavor of this instance  }
		componentManufacturer:	OSType;									{  Vendor indentification  }
		componentFlags:			UInt32;									{  8 each for Component,Type,SubType,Manuf/revision  }
		componentFlagsMask:		UInt32;									{  Mask for specifying which flags to consider in search, zero during registration  }
=======
>>>>>>> origin/fixes_2.4
	end;

type
	ResourceSpecPtr = ^ResourceSpec;
	ResourceSpec = record
		resType: OSType;                { 4-byte code    }
		resID: SInt16;                  {         }
	end;
type
	ComponentResource = record
		cd: ComponentDescription;                   { Registration parameters }
		component: ResourceSpec;              { resource where Component code is found }
		componentName: ResourceSpec;          { name string resource }
		componentInfo: ResourceSpec;          { info string resource }
		componentIcon: ResourceSpec;          { icon resource }
	end;
	ComponentResourcePtr = ^ComponentResource;
type
	ComponentResourceHandle = ^ComponentResourcePtr;
	ComponentPlatformInfo = record
		componentFlags: SInt32;         { flags of Component }
		component: ResourceSpec;              { resource where Component code is found }
		platformType: SInt16;           { gestaltSysArchitecture result }
	end;
type
	ComponentResourceExtensionPtr = ^ComponentResourceExtension;
	ComponentResourceExtension = record
		componentVersion: SInt32;       { version of Component }
		componentRegisterFlags: SInt32; { flags for registration }
		componentIconFamily: SInt16;    { resource id of Icon Family }
	end;
type
	ComponentPlatformInfoArrayPtr = ^ComponentPlatformInfoArray;
	ComponentPlatformInfoArray = record
		count: SInt32;
		platformArray: array [0..0] of ComponentPlatformInfo;
	end;
type
	ExtComponentResource = record
		cd: ComponentDescription;                   { registration parameters }
		component: ResourceSpec;              { resource where Component code is found }
		componentName: ResourceSpec;          { name string resource }
		componentInfo: ResourceSpec;          { info string resource }
		componentIcon: ResourceSpec;          { icon resource }
		componentVersion: SInt32;       { version of Component }
		componentRegisterFlags: SInt32; { flags for registration }
		componentIconFamily: SInt16;    { resource id of Icon Family }
		count: SInt32;                  { elements in platformArray }
    platformArray: array [0..0] of ComponentPlatformInfo;
	end;
	ExtComponentResourcePtr = ^ExtComponentResource;
type
	ExtComponentResourceHandle = ^ExtComponentResourcePtr;
	ComponentAliasResource = record
		cr: ComponentResource;                     { Registration parameters }
		aliasCD: ComponentDescription;              { component alias description }
	end;
{  Structure received by Component:        }
type
	ComponentParametersPtr = ^ComponentParameters;
	ComponentParameters = record
		flags: UInt8;                  { call modifiers: sync/async, deferred, immed, etc }
		paramSize: UInt8;              { size in bytes of actual parameters passed to this call }
		what: SInt16;                   { routine selector, negative for Component management calls }
{$ifc TARGET_CPU_64}

		padding: UInt32;
{$endc} {TARGET_CPU_64}

  params: array[0..0] of SIGNEDLONG;             { actual parameters for the indicated routine }
	end;
type
	ComponentRecord = record
		data: array [0..0] of SIGNEDLONG;
	end;
type
	Component = ^ComponentRecord;
	ComponentInstanceRecordPtr = ^ComponentInstanceRecord;
	ComponentInstanceRecord = record
		data: array [0..0] of SIGNEDLONG;
	end;
type
	ComponentInstance = ^ComponentInstanceRecord;
	RegisteredComponentRecord = record
		data: array [0..0] of SIGNEDLONG;
	end;
type
	RegisteredComponentRecordPtr = ^RegisteredComponentRecord;
type
	RegisteredComponentInstanceRecord = record
		data: array [0..0] of SIGNEDLONG;
	end;
	RegisteredComponentInstanceRecordPtr = ^RegisteredComponentInstanceRecord;
type
	ComponentResult = SInt32;
const
	platform68k = 1;    { platform type (response from gestaltComponentPlatform) }
	platformPowerPC = 2;    { (when gestaltComponentPlatform is not implemented, use }
	platformInterpreted = 3;    { gestaltSysArchitecture) }
	platformWin32 = 4;
	platformPowerPCNativeEntryPoint = 5;
	platformIA32NativeEntryPoint = 6;
	platformPowerPC64NativeEntryPoint = 7;
	platformX86_64NativeEntryPoint = 8;

const
	platformIRIXmips = 1000;
	platformSunOSsparc = 1100;
	platformSunOSintel = 1101;
	platformLinuxppc = 1200;
	platformLinuxintel = 1201;
	platformAIXppc = 1300;
	platformNeXTIntel = 1400;
	platformNeXTppc = 1401;
	platformNeXTsparc = 1402;
	platformNeXT68k = 1403;
	platformMacOSx86 = 1500;

const
	mpWorkFlagDoWork = 1 shl 0;
	mpWorkFlagDoCompletion = 1 shl 1;
	mpWorkFlagCopyWorkBlock = 1 shl 2;
	mpWorkFlagDontBlock = 1 shl 3;
	mpWorkFlagGetProcessorCount = 1 shl 4;
	mpWorkFlagGetIsRunning = 1 shl 6;

const
	cmpAliasNoFlags = 0;
	cmpAliasOnlyThisFile = 1;

type
	CSComponentsThreadMode = UInt32;
const
	kCSAcceptAllComponentsMode = 0;
	kCSAcceptThreadSafeComponentsOnlyMode = 1;

{
 *  CSSetComponentsThreadMode()
 *  
 *  Summary:
 *    Set whether or not using thread-unsafe components is allowed on
 *    the current thread.
 *  
 *  Discussion:
 *    When set to kCSAcceptThreadSafeComponentsOnlyMode, the current
 *    thread can only make thread-safe calls. Applications and other
 *    high-level code that wants to call QuickTime (and other) APIs
 *    from preemptive threads should call  SetComponentsThreadMode(
 *    kCSAcceptThreadSafeComponentsOnlyMode );  from their thread
 *    beforehand. The safeguard flag should only be left
 *    kCSAcceptAllComponentsMode for the main thread and other threads
 *    that participate in cooperative locking with it (such as the
 *    Carbon Thread Manager-style cooperative threads and application 
 *    threads that perform private locking).
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    mode:
 *      The thread-safety mode in current thread.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
procedure CSSetComponentsThreadMode( mode: CSComponentsThreadMode ); external name '_CSSetComponentsThreadMode';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  CSGetComponentsThreadMode()
 *  
 *  Summary:
 *    Get the current thread's thread-safety mode.
 *  
 *  Discussion:
 *    Returns kCSAcceptThreadSafeComponentsOnlyMode if only thread-safe
 *    components are allowed in current thread and
 *    kCSAcceptAllComponentsMode if all components are accepted
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function CSGetComponentsThreadMode: CSComponentsThreadMode; external name '_CSGetComponentsThreadMode';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


type
	ComponentMPWorkFunctionHeaderRecord = record
		headerSize: UInt32;
		recordSize: UInt32;
		workFlags: UInt32;
		processorCount: UInt16;
		unused: UInt8;
		isRunning: UInt8;
	end;
	ComponentMPWorkFunctionHeaderRecordPtr = ^ComponentMPWorkFunctionHeaderRecord;
type
	ComponentMPWorkFunctionProcPtr = function( globalRefCon: UnivPtr; header: ComponentMPWorkFunctionHeaderRecordPtr ): ComponentResult;
	ComponentRoutineProcPtr = function( var cp: ComponentParameters; componentStorage: Handle ): ComponentResult;
	GetMissingComponentResourceProcPtr = function( c: Component; resType: OSType; resID: SInt16; refCon: UnivPtr; var resource: Handle ): OSErr;
	ComponentMPWorkFunctionUPP = ComponentMPWorkFunctionProcPtr;
	ComponentRoutineUPP = ComponentRoutineProcPtr;
	GetMissingComponentResourceUPP = GetMissingComponentResourceProcPtr;
{
    The parameter list for each ComponentFunction is unique. It is
    therefore up to users to create the appropriate procInfo for their
    own ComponentFunctions where necessary.
}
type
	ComponentFunctionUPP = UniversalProcPtr;
{
 *  NewComponentFunctionUPP()
 *  
 *  Discussion:
 *    For use in writing a Carbon compliant Component.  It is used to
 *    create a ComponentFunctionUPP needed to call
 *    CallComponentFunction in the Components dispatch routine.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewComponentFunctionUPP( userRoutine: ProcPtr; procInfo: ProcInfoType ): ComponentFunctionUPP; external name '_NewComponentFunctionUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DisposeComponentFunctionUPP()
 *  
 *  Discussion:
 *    For use in writing a Carbon compliant Component.  It is used to
 *    dispose of a ComponentFunctionUPP created by
 *    NewComponentFunctionUPP.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeComponentFunctionUPP( userUPP: ComponentFunctionUPP ); external name '_DisposeComponentFunctionUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{*******************************************************
*                                                       *
*               APPLICATION LEVEL CALLS                 *
*                                                       *
*******************************************************}
{*******************************************************
* Component Database Add, Delete, and Query Routines
*******************************************************}
{
 *  RegisterComponent()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function RegisterComponent( var cd: ComponentDescription; componentEntryPoint: ComponentRoutineUPP; global: SInt16; componentName: Handle; componentInfo: Handle; componentIcon: Handle ): Component; external name '_RegisterComponent';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  RegisterComponentResource()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function RegisterComponentResource( cr: ComponentResourceHandle; global: SInt16 ): Component; external name '_RegisterComponentResource';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  UnregisterComponent()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function UnregisterComponent( aComponent: Component ): OSErr; external name '_UnregisterComponent';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  FindNextComponent()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function FindNextComponent( aComponent: Component; var looking: ComponentDescription ): Component; external name '_FindNextComponent';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  CountComponents()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function CountComponents( var looking: ComponentDescription ): SIGNEDLONG; external name '_CountComponents';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  GetComponentInfo()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function GetComponentInfo( aComponent: Component; var cd: ComponentDescription; componentName: Handle; componentInfo: Handle; componentIcon: Handle ): OSErr; external name '_GetComponentInfo';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  GetComponentListModSeed()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function GetComponentListModSeed: SInt32; external name '_GetComponentListModSeed';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  GetComponentTypeModSeed()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 2.5 and later
<<<<<<< HEAD
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function GetComponentTypeModSeed( componentType: OSType ): SInt32; external name '_GetComponentTypeModSeed';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


=======
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
=======
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function RegisterComponent(var cd: ComponentDescription; componentEntryPoint: ComponentRoutineUPP; global: SInt16; componentName: Handle; componentInfo: Handle; componentIcon: Handle): Component; external name '_RegisterComponent';
{
 *  RegisterComponentResource()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function RegisterComponentResource(cr: ComponentResourceHandle; global: SInt16): Component; external name '_RegisterComponentResource';
{
 *  UnregisterComponent()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function UnregisterComponent(aComponent: Component): OSErr; external name '_UnregisterComponent';
{
 *  FindNextComponent()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function FindNextComponent(aComponent: Component; var looking: ComponentDescription): Component; external name '_FindNextComponent';
{
 *  CountComponents()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function CountComponents(var looking: ComponentDescription): SInt32; external name '_CountComponents';
{
 *  GetComponentInfo()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function GetComponentInfo(aComponent: Component; var cd: ComponentDescription; componentName: Handle; componentInfo: Handle; componentIcon: Handle): OSErr; external name '_GetComponentInfo';
{
 *  GetComponentListModSeed()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function GetComponentListModSeed: SInt32; external name '_GetComponentListModSeed';
{
 *  GetComponentTypeModSeed()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function GetComponentTypeModSeed(componentType: OSType): SInt32; external name '_GetComponentTypeModSeed';
{*******************************************************
* Component Instance Allocation and dispatch routines
*******************************************************}
{
 *  OpenAComponent()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 2.5 and later
>>>>>>> origin/fixes_2_2
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function RegisterComponent(var cd: ComponentDescription; componentEntryPoint: ComponentRoutineUPP; global: SInt16; componentName: Handle; componentInfo: Handle; componentIcon: Handle): Component; external name '_RegisterComponent';
{
 *  RegisterComponentResource()
=======
function OpenAComponent(aComponent: Component; var ci: ComponentInstance): OSErr; external name '_OpenAComponent';
{
 *  OpenComponent()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function RegisterComponentResource(cr: ComponentResourceHandle; global: SInt16): Component; external name '_RegisterComponentResource';
{
 *  UnregisterComponent()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function UnregisterComponent(aComponent: Component): OSErr; external name '_UnregisterComponent';
{
 *  FindNextComponent()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function FindNextComponent(aComponent: Component; var looking: ComponentDescription): Component; external name '_FindNextComponent';
{
 *  CountComponents()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function CountComponents(var looking: ComponentDescription): SInt32; external name '_CountComponents';
{
 *  GetComponentInfo()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function GetComponentInfo(aComponent: Component; var cd: ComponentDescription; componentName: Handle; componentInfo: Handle; componentIcon: Handle): OSErr; external name '_GetComponentInfo';
{
 *  GetComponentListModSeed()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function GetComponentListModSeed: SInt32; external name '_GetComponentListModSeed';
{
 *  GetComponentTypeModSeed()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function GetComponentTypeModSeed(componentType: OSType): SInt32; external name '_GetComponentTypeModSeed';
>>>>>>> graemeg/fixes_2_2
=======
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function GetComponentTypeModSeed( componentType: OSType ): SInt32; external name '_GetComponentTypeModSeed';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{*******************************************************
* Component Instance Allocation and dispatch routines
*******************************************************}
{
 *  OpenAComponent()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function OpenAComponent( aComponent: Component; var ci: ComponentInstance ): OSErr; external name '_OpenAComponent';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


{
 *  OpenComponent()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function OpenComponent( aComponent: Component ): ComponentInstance; external name '_OpenComponent';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


{
 *  CloseComponent()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function CloseComponent( aComponentInstance: ComponentInstance ): OSErr; external name '_CloseComponent';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


{
 *  GetComponentInstanceError()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function GetComponentInstanceError( aComponentInstance: ComponentInstance ): OSErr; external name '_GetComponentInstanceError';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


=======
=======
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function OpenAComponent( aComponent: Component; var ci: ComponentInstance ): OSErr; external name '_OpenAComponent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  OpenComponent()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function OpenComponent( aComponent: Component ): ComponentInstance; external name '_OpenComponent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CloseComponent()
=======
function OpenComponent(aComponent: Component): ComponentInstance; external name '_OpenComponent';
{
 *  CloseComponent()
>>>>>>> origin/fixes_2_2
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function CloseComponent( aComponentInstance: ComponentInstance ): OSErr; external name '_CloseComponent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetComponentInstanceError()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function GetComponentInstanceError(aComponentInstance: ComponentInstance): OSErr; external name '_GetComponentInstanceError';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function GetComponentInstanceError( aComponentInstance: ComponentInstance ): OSErr; external name '_GetComponentInstanceError';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{*******************************************************
* Component aliases
*******************************************************}
{
 *  ResolveComponentAlias()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function ResolveComponentAlias( aComponent: Component ): Component; external name '_ResolveComponentAlias';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


=======
=======
>>>>>>> origin/fixes_2_2
=======
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function ResolveComponentAlias(aComponent: Component): Component; external name '_ResolveComponentAlias';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ResolveComponentAlias( aComponent: Component ): Component; external name '_ResolveComponentAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{*******************************************************
* Component public resources and public string lists
*******************************************************}
{ Note: GetComponentPublicResource returns a Handle, not a resource.  The caller must dispose it with DisposeHandle. }
{
 *  GetComponentPublicResource()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 4.0 and later
 }
function GetComponentPublicResource( aComponent: Component; resourceType: OSType; resourceID: SInt16; var theResource: Handle ): OSErr; external name '_GetComponentPublicResource';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


{
 *  GetComponentPublicResourceList()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 4.0 and later
 }
function GetComponentPublicResourceList( resourceType: OSType; resourceID: SInt16; flags: SInt32; var cd: ComponentDescription; missingProc: GetMissingComponentResourceUPP; refCon: UnivPtr; atomContainerPtr: UnivPtr ): OSErr; external name '_GetComponentPublicResourceList';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


{
 *  GetComponentPublicIndString()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 4.0 and later
 }
function GetComponentPublicIndString( aComponent: Component; var theString: Str255; strListID: SInt16; index: SInt16 ): OSErr; external name '_GetComponentPublicIndString';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


=======
=======
>>>>>>> origin/fixes_2_2
=======
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 4.0 and later
 }
function GetComponentPublicResource( aComponent: Component; resourceType: OSType; resourceID: SInt16; var theResource: Handle ): OSErr; external name '_GetComponentPublicResource';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetComponentPublicResourceList()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 4.0 and later
 }
function GetComponentPublicResourceList( resourceType: OSType; resourceID: SInt16; flags: SInt32; var cd: ComponentDescription; missingProc: GetMissingComponentResourceUPP; refCon: UnivPtr; atomContainerPtr: UnivPtr ): OSErr; external name '_GetComponentPublicResourceList';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetComponentPublicIndString()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.3 and later
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 4.0 and later
 }
<<<<<<< HEAD
function GetComponentPublicIndString(aComponent: Component; var theString: Str255; strListID: SInt16; index: SInt16): OSErr; external name '_GetComponentPublicIndString';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function GetComponentPublicIndString( aComponent: Component; var theString: Str255; strListID: SInt16; index: SInt16 ): OSErr; external name '_GetComponentPublicIndString';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{*******************************************************
*                                                       *
*                   CALLS MADE BY COMPONENTS            *
*                                                       *
*******************************************************}
{*******************************************************
* Component Management routines
*******************************************************}
{
 *  SetComponentInstanceError()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
<<<<<<< HEAD
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
procedure SetComponentInstanceError( aComponentInstance: ComponentInstance; theError: OSErr ); external name '_SetComponentInstanceError';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


{
 *  GetComponentRefcon()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function GetComponentRefcon( aComponent: Component ): SIGNEDLONG; external name '_GetComponentRefcon';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


{
 *  SetComponentRefcon()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
procedure SetComponentRefcon( aComponent: Component; theRefcon: SIGNEDLONG ); external name '_SetComponentRefcon';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


{
 *  OpenComponentResFile()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function OpenComponentResFile( aComponent: Component ): ResFileRefNum; external name '_OpenComponentResFile';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


{
 *  OpenAComponentResFile()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function OpenAComponentResFile( aComponent: Component; var resRef: ResFileRefNum ): OSErr; external name '_OpenAComponentResFile';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


{
 *  CloseComponentResFile()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function CloseComponentResFile( refnum: ResFileRefNum ): OSErr; external name '_CloseComponentResFile';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


=======
=======
>>>>>>> origin/fixes_2_2
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
procedure SetComponentInstanceError(aComponentInstance: ComponentInstance; theError: OSErr); external name '_SetComponentInstanceError';
{
 *  GetComponentRefcon()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function GetComponentRefcon(aComponent: Component): SInt32; external name '_GetComponentRefcon';
{
 *  SetComponentRefcon()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
procedure SetComponentRefcon(aComponent: Component; theRefcon: SInt32); external name '_SetComponentRefcon';
{
 *  OpenComponentResFile()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function OpenComponentResFile(aComponent: Component): SInt16; external name '_OpenComponentResFile';
{
 *  OpenAComponentResFile()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function OpenAComponentResFile(aComponent: Component; var resRef: SInt16): OSErr; external name '_OpenAComponentResFile';
{
 *  CloseComponentResFile()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function CloseComponentResFile(refnum: SInt16): OSErr; external name '_CloseComponentResFile';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{ Note: GetComponentResource returns a Handle, not a resource.  The caller must dispose it with DisposeHandle. }
{
 *  GetComponentResource()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function GetComponentResource( aComponent: Component; resType: OSType; resID: SInt16; var theResource: Handle ): OSErr; external name '_GetComponentResource';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


{
 *  GetComponentIndString()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function GetComponentIndString( aComponent: Component; var theString: Str255; strListID: SInt16; index: SInt16 ): OSErr; external name '_GetComponentIndString';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


=======
=======
>>>>>>> origin/fixes_2_2
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 3.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function GetComponentResource(aComponent: Component; resType: OSType; resID: SInt16; var theResource: Handle): OSErr; external name '_GetComponentResource';
{
 *  GetComponentIndString()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 3.0 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function GetComponentIndString(aComponent: Component; var theString: Str255; strListID: SInt16; index: SInt16): OSErr; external name '_GetComponentIndString';
>>>>>>> graemeg/fixes_2_2
{*******************************************************
* Component Instance Management routines
*******************************************************}
{
 *  GetComponentInstanceStorage()
 *  
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function GetComponentInstanceStorage( aComponentInstance: ComponentInstance ): Handle; external name '_GetComponentInstanceStorage';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


{
 *  SetComponentInstanceStorage()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
procedure SetComponentInstanceStorage( aComponentInstance: ComponentInstance; theStorage: Handle ); external name '_SetComponentInstanceStorage';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function GetComponentInstanceStorage(aComponentInstance: ComponentInstance): Handle; external name '_GetComponentInstanceStorage';
{
 *  SetComponentInstanceStorage()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
procedure SetComponentInstanceStorage(aComponentInstance: ComponentInstance; theStorage: Handle); external name '_SetComponentInstanceStorage';
{$ifc CALL_NOT_IN_CARBON}
{
 *  GetComponentInstanceA5()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function GetComponentInstanceA5(aComponentInstance: ComponentInstance): SInt32; external name '_GetComponentInstanceA5';
{
 *  SetComponentInstanceA5()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
procedure SetComponentInstanceA5(aComponentInstance: ComponentInstance; theA5: SInt32); external name '_SetComponentInstanceA5';
{$endc}  {CALL_NOT_IN_CARBON}
>>>>>>> graemeg/fixes_2_2

{
 *  CountComponentInstances()
 *  
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function CountComponentInstances( aComponent: Component ): SIGNEDLONG; external name '_CountComponentInstances';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


=======
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function CountComponentInstances(aComponent: Component): SInt32; external name '_CountComponentInstances';
>>>>>>> graemeg/fixes_2_2
{ useful helper routines for convenient method dispatching }
{
 *  CallComponentFunction()
 *  
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function CallComponentFunction( var params: ComponentParameters; func: ComponentFunctionUPP ): ComponentResult; external name '_CallComponentFunction';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


{
 *  CallComponentFunctionWithStorage()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function CallComponentFunctionWithStorage( storage: Handle; var params: ComponentParameters; func: ComponentFunctionUPP ): ComponentResult; external name '_CallComponentFunctionWithStorage';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function CallComponentFunction(var params: ComponentParameters; func: ComponentFunctionUPP): SInt32; external name '_CallComponentFunction';
{
 *  CallComponentFunctionWithStorage()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function CallComponentFunctionWithStorage(storage: Handle; var params: ComponentParameters; func: ComponentFunctionUPP): SInt32; external name '_CallComponentFunctionWithStorage';
>>>>>>> graemeg/fixes_2_2

{
 *  CallComponentFunctionWithStorageProcInfo()
 *  
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 2.5 and later
 }
function CallComponentFunctionWithStorageProcInfo( storage: Handle; var params: ComponentParameters; func: ProcPtr; funcProcInfo: ProcInfoType ): ComponentResult; external name '_CallComponentFunctionWithStorageProcInfo';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function CallComponentFunctionWithStorageProcInfo(storage: Handle; var params: ComponentParameters; func: ProcPtr; funcProcInfo: ProcInfoType): SInt32; external name '_CallComponentFunctionWithStorageProcInfo';
>>>>>>> graemeg/fixes_2_2

{
 *  DelegateComponentCall()
 *  
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function DelegateComponentCall( var originalParams: ComponentParameters; ci: ComponentInstance ): ComponentResult; external name '_DelegateComponentCall';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


{
 *  SetDefaultComponent()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function SetDefaultComponent( aComponent: Component; flags: SInt16 ): OSErr; external name '_SetDefaultComponent';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


{
 *  OpenDefaultComponent()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function OpenDefaultComponent( componentType: OSType; componentSubType: OSType ): ComponentInstance; external name '_OpenDefaultComponent';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


{
 *  OpenADefaultComponent()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function OpenADefaultComponent( componentType: OSType; componentSubType: OSType; var ci: ComponentInstance ): OSErr; external name '_OpenADefaultComponent';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


{
 *  CaptureComponent()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function CaptureComponent( capturedComponent: Component; capturingComponent: Component ): Component; external name '_CaptureComponent';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
procedure SetComponentInstanceError( aComponentInstance: ComponentInstance; theError: OSErr ); external name '_SetComponentInstanceError';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  UncaptureComponent()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function UncaptureComponent( aComponent: Component ): OSErr; external name '_UncaptureComponent';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
function GetComponentRefcon( aComponent: Component ): SIGNEDLONG; external name '_GetComponentRefcon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  RegisterComponentResourceFile()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function RegisterComponentResourceFile( resRefNum: SInt16; global: SInt16 ): SInt32; external name '_RegisterComponentResourceFile';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


{ This call is deprecated. Please use GetIconRefFromComponent() instead.}
{$ifc not TARGET_CPU_64}
=======
procedure SetComponentRefcon( aComponent: Component; theRefcon: SIGNEDLONG ); external name '_SetComponentRefcon';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  GetComponentIconSuite()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function GetComponentIconSuite( aComponent: Component; var iconSuite: Handle ): OSErr; external name '_GetComponentIconSuite';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


{$endc} {not TARGET_CPU_64}
=======
function OpenComponentResFile( aComponent: Component ): ResFileRefNum; external name '_OpenComponentResFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 * These calls allow you to register a file system entity.  The
 * Component Manager will "do the right thing" with the entity,
 * whether it is a standard resource fork based CFM component, CFM
 * bundle, mach-o bundle, or packaged bundle.  
 *
 * The *Entries calls allow you to specify a component description
 * which will be used to register selective components.  (Passing
 * NULL, 0 means to register all components.  
 }
{$ifc not TARGET_CPU_64}
{
 *  RegisterComponentFile()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function RegisterComponentFile( const (*var*) spec: FSSpec; global: SInt16 ): OSErr; external name '_RegisterComponentFile';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function OpenAComponentResFile( aComponent: Component; var resRef: ResFileRefNum ): OSErr; external name '_OpenAComponentResFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  RegisterComponentFileEntries()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function RegisterComponentFileEntries( const (*var*) spec: FSSpec; global: SInt16; {const} toRegister: ComponentDescriptionPtr { can be NULL }; registerCount: UInt32 ): OSErr; external name '_RegisterComponentFileEntries';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/cpstrnew


{$endc} {not TARGET_CPU_64}

=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function CloseComponentResFile( refnum: ResFileRefNum ): OSErr; external name '_CloseComponentResFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Note: GetComponentResource returns a Handle, not a resource.  The caller must dispose it with DisposeHandle. }
>>>>>>> origin/fixes_2.4
{
 *  RegisterComponentFileRef()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
<<<<<<< HEAD
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function RegisterComponentFileRef( const (*var*) ref: FSRef; global: SInt16 ): OSErr; external name '_RegisterComponentFileRef';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function GetComponentResource( aComponent: Component; resType: OSType; resID: SInt16; var theResource: Handle ): OSErr; external name '_GetComponentResource';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  RegisterComponentFileRefEntries()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
<<<<<<< HEAD
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function RegisterComponentFileRefEntries( const (*var*) ref: FSRef; global: SInt16; {const} toRegister: ComponentDescriptionPtr { can be NULL }; registerCount: UInt32 ): OSErr; external name '_RegisterComponentFileRefEntries';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


=======
 *  Availability:
=======
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 3.0 and later
=======
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 3.0 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function GetComponentIndString( aComponent: Component; var theString: Str255; strListID: SInt16; index: SInt16 ): OSErr; external name '_GetComponentIndString';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{*******************************************************
* Component Instance Management routines
*******************************************************}
{
 *  GetComponentInstanceStorage()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function GetComponentInstanceStorage( aComponentInstance: ComponentInstance ): Handle; external name '_GetComponentInstanceStorage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetComponentInstanceStorage()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
procedure SetComponentInstanceStorage( aComponentInstance: ComponentInstance; theStorage: Handle ); external name '_SetComponentInstanceStorage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CountComponentInstances()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function CountComponentInstances( aComponent: Component ): SIGNEDLONG; external name '_CountComponentInstances';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ useful helper routines for convenient method dispatching }
{
 *  CallComponentFunction()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function CallComponentFunction( var params: ComponentParameters; func: ComponentFunctionUPP ): ComponentResult; external name '_CallComponentFunction';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CallComponentFunctionWithStorage()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function CallComponentFunctionWithStorage( storage: Handle; var params: ComponentParameters; func: ComponentFunctionUPP ): ComponentResult; external name '_CallComponentFunctionWithStorage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CallComponentFunctionWithStorageProcInfo()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 2.5 and later
 }
function CallComponentFunctionWithStorageProcInfo( storage: Handle; var params: ComponentParameters; func: ProcPtr; funcProcInfo: ProcInfoType ): ComponentResult; external name '_CallComponentFunctionWithStorageProcInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DelegateComponentCall()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function DelegateComponentCall( var originalParams: ComponentParameters; ci: ComponentInstance ): ComponentResult; external name '_DelegateComponentCall';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetDefaultComponent()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function SetDefaultComponent( aComponent: Component; flags: SInt16 ): OSErr; external name '_SetDefaultComponent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  OpenDefaultComponent()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function OpenDefaultComponent( componentType: OSType; componentSubType: OSType ): ComponentInstance; external name '_OpenDefaultComponent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  OpenADefaultComponent()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function OpenADefaultComponent( componentType: OSType; componentSubType: OSType; var ci: ComponentInstance ): OSErr; external name '_OpenADefaultComponent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CaptureComponent()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function CaptureComponent( capturedComponent: Component; capturingComponent: Component ): Component; external name '_CaptureComponent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  UncaptureComponent()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function UncaptureComponent( aComponent: Component ): OSErr; external name '_UncaptureComponent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  RegisterComponentResourceFile()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function RegisterComponentResourceFile( resRefNum: SInt16; global: SInt16 ): SInt32; external name '_RegisterComponentResourceFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ This call is deprecated. Please use GetIconRefFromComponent() instead.}
{$ifc not TARGET_CPU_64}
{
 *  GetComponentIconSuite()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function GetComponentIconSuite(aComponent: Component; var iconSuite: Handle): OSErr; external name '_GetComponentIconSuite';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function GetComponentIconSuite( aComponent: Component; var iconSuite: Handle ): OSErr; external name '_GetComponentIconSuite';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$endc} {not TARGET_CPU_64}

{
 * These calls allow you to register a file system entity.  The
 * Component Manager will "do the right thing" with the entity,
 * whether it is a standard resource fork based CFM component, CFM
 * bundle, mach-o bundle, or packaged bundle.  
 *
 * The *Entries calls allow you to specify a component description
 * which will be used to register selective components.  (Passing
 * NULL, 0 means to register all components.  
 }
{$ifc not TARGET_CPU_64}
{
 *  RegisterComponentFile()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function RegisterComponentFile( const (*var*) spec: FSSpec; global: SInt16 ): OSErr; external name '_RegisterComponentFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  RegisterComponentFileEntries()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function RegisterComponentFileEntries( const (*var*) spec: FSSpec; global: SInt16; {const} toRegister: ComponentDescriptionPtr { can be NULL }; registerCount: UInt32 ): OSErr; external name '_RegisterComponentFileEntries';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{$endc} {not TARGET_CPU_64}

{
 *  RegisterComponentFileRef()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function RegisterComponentFileRef( const (*var*) ref: FSRef; global: SInt16 ): OSErr; external name '_RegisterComponentFileRef';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  RegisterComponentFileRefEntries()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function RegisterComponentFileRefEntries( const (*var*) ref: FSRef; global: SInt16; {const} toRegister: ComponentDescriptionPtr { can be NULL }; registerCount: UInt32 ): OSErr; external name '_RegisterComponentFileRefEntries';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{*******************************************************
*                                                       *
*           Direct calls to the Components              *
*                                                       *
*******************************************************}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ Old style names}

{$ifc not TARGET_CPU_64}
{
 *  ComponentFunctionImplemented()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function ComponentFunctionImplemented( ci: ComponentInstance; ftnNumber: SInt16 ): ComponentResult; external name '_ComponentFunctionImplemented';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/cpstrnew


{
 *  GetComponentVersion()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function GetComponentVersion( ci: ComponentInstance ): ComponentResult; external name '_GetComponentVersion';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/cpstrnew


{
 *  ComponentSetTarget()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function ComponentSetTarget( ci: ComponentInstance; target: ComponentInstance ): ComponentResult; external name '_ComponentSetTarget';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/cpstrnew


{ New style names}

{$endc} {not TARGET_CPU_64}
=======
=======
>>>>>>> origin/fixes_2_2
{  Old style names }
=======
{ Old style names}
>>>>>>> origin/fixes_2.4

{$ifc not TARGET_CPU_64}
{
 *  ComponentFunctionImplemented()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function ComponentFunctionImplemented( ci: ComponentInstance; ftnNumber: SInt16 ): ComponentResult; external name '_ComponentFunctionImplemented';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetComponentVersion()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function GetComponentVersion( ci: ComponentInstance ): ComponentResult; external name '_GetComponentVersion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  ComponentSetTarget()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
<<<<<<< HEAD
function ComponentSetTarget(ci: ComponentInstance; target: ComponentInstance): SInt32; external name '_ComponentSetTarget';
{  New style names }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function ComponentSetTarget( ci: ComponentInstance; target: ComponentInstance ): ComponentResult; external name '_ComponentSetTarget';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ New style names}

{$endc} {not TARGET_CPU_64}
>>>>>>> origin/fixes_2.4

{
 *  CallComponentOpen()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function CallComponentOpen( ci: ComponentInstance; self: ComponentInstance ): ComponentResult; external name '_CallComponentOpen';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


{
 *  CallComponentClose()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function CallComponentClose( ci: ComponentInstance; self: ComponentInstance ): ComponentResult; external name '_CallComponentClose';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


{
 *  CallComponentCanDo()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function CallComponentCanDo( ci: ComponentInstance; ftnNumber: SInt16 ): ComponentResult; external name '_CallComponentCanDo';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


{
 *  CallComponentVersion()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function CallComponentVersion( ci: ComponentInstance ): ComponentResult; external name '_CallComponentVersion';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


{
 *  CallComponentRegister()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function CallComponentRegister( ci: ComponentInstance ): ComponentResult; external name '_CallComponentRegister';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


{
 *  CallComponentTarget()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function CallComponentTarget( ci: ComponentInstance; target: ComponentInstance ): ComponentResult; external name '_CallComponentTarget';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


{
 *  CallComponentUnregister()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function CallComponentUnregister( ci: ComponentInstance ): ComponentResult; external name '_CallComponentUnregister';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


{
 *  CallComponentGetMPWorkFunction()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function CallComponentGetMPWorkFunction( ci: ComponentInstance; var workFunction: ComponentMPWorkFunctionUPP; var refCon: UnivPtr ): ComponentResult; external name '_CallComponentGetMPWorkFunction';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


{
 *  CallComponentGetPublicResource()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 4.0 and later
 }
function CallComponentGetPublicResource( ci: ComponentInstance; resourceType: OSType; resourceID: SInt16; var resource: Handle ): ComponentResult; external name '_CallComponentGetPublicResource';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
=======
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function CallComponentOpen( ci: ComponentInstance; self: ComponentInstance ): ComponentResult; external name '_CallComponentOpen';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CallComponentClose()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function CallComponentClose( ci: ComponentInstance; self: ComponentInstance ): ComponentResult; external name '_CallComponentClose';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CallComponentCanDo()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function CallComponentCanDo( ci: ComponentInstance; ftnNumber: SInt16 ): ComponentResult; external name '_CallComponentCanDo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CallComponentVersion()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function CallComponentVersion( ci: ComponentInstance ): ComponentResult; external name '_CallComponentVersion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CallComponentRegister()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function CallComponentRegister( ci: ComponentInstance ): ComponentResult; external name '_CallComponentRegister';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CallComponentTarget()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function CallComponentTarget( ci: ComponentInstance; target: ComponentInstance ): ComponentResult; external name '_CallComponentTarget';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CallComponentUnregister()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function CallComponentUnregister( ci: ComponentInstance ): ComponentResult; external name '_CallComponentUnregister';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CallComponentGetMPWorkFunction()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 2.5 and later
 *    Windows:          in qtmlClient.lib 3.0 and later
 }
function CallComponentGetMPWorkFunction( ci: ComponentInstance; var workFunction: ComponentMPWorkFunctionUPP; var refCon: UnivPtr ): ComponentResult; external name '_CallComponentGetMPWorkFunction';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CallComponentGetPublicResource()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib via QuickTime 4.0 and later
 }
function CallComponentGetPublicResource( ci: ComponentInstance; resourceType: OSType; resourceID: SInt16; var resource: Handle ): ComponentResult; external name '_CallComponentGetPublicResource';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

<<<<<<< HEAD
{$endc}  {CALL_NOT_IN_CARBON}
{$endc}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{
    CallComponentDispatch is a CarbonLib routine that replaces CallComponent inline glue
    to call a component function.
 }
{
 *  CallComponentDispatch()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function CallComponentDispatch( var cp: ComponentParameters ): ComponentResult; external name '_CallComponentDispatch';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


{ UPP call backs }
{
 *  NewComponentMPWorkFunctionUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewComponentMPWorkFunctionUPP( userRoutine: ComponentMPWorkFunctionProcPtr ): ComponentMPWorkFunctionUPP; external name '_NewComponentMPWorkFunctionUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
=======
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function CallComponentDispatch( var cp: ComponentParameters ): ComponentResult; external name '_CallComponentDispatch';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ UPP call backs }
{
 *  NewComponentMPWorkFunctionUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewComponentMPWorkFunctionUPP( userRoutine: ComponentMPWorkFunctionProcPtr ): ComponentMPWorkFunctionUPP; external name '_NewComponentMPWorkFunctionUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

<<<<<<< HEAD
const
	uppComponentMPWorkFunctionProcInfo = $000003F0;
	uppComponentRoutineProcInfo = $000003F0;
	uppGetMissingComponentResourceProcInfo = $0000FBE0;
	{
	 *  NewComponentMPWorkFunctionUPP()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   available as macro/inline
	 *    CarbonLib:        in CarbonLib 1.0 and later
	 *    Mac OS X:         in version 10.0 and later
	 	}
function NewComponentMPWorkFunctionUPP(userRoutine: ComponentMPWorkFunctionProcPtr): ComponentMPWorkFunctionUPP; external name '_NewComponentMPWorkFunctionUPP'; { old name was NewComponentMPWorkFunctionProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{
 *  NewComponentRoutineUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewComponentRoutineUPP( userRoutine: ComponentRoutineProcPtr ): ComponentRoutineUPP; external name '_NewComponentRoutineUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function NewComponentRoutineUPP(userRoutine: ComponentRoutineProcPtr): ComponentRoutineUPP; external name '_NewComponentRoutineUPP'; { old name was NewComponentRoutineProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function NewComponentRoutineUPP( userRoutine: ComponentRoutineProcPtr ): ComponentRoutineUPP; external name '_NewComponentRoutineUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  NewGetMissingComponentResourceUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewGetMissingComponentResourceUPP( userRoutine: GetMissingComponentResourceProcPtr ): GetMissingComponentResourceUPP; external name '_NewGetMissingComponentResourceUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function NewGetMissingComponentResourceUPP(userRoutine: GetMissingComponentResourceProcPtr): GetMissingComponentResourceUPP; external name '_NewGetMissingComponentResourceUPP'; { old name was NewGetMissingComponentResourceProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function NewGetMissingComponentResourceUPP( userRoutine: GetMissingComponentResourceProcPtr ): GetMissingComponentResourceUPP; external name '_NewGetMissingComponentResourceUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  DisposeComponentMPWorkFunctionUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeComponentMPWorkFunctionUPP( userUPP: ComponentMPWorkFunctionUPP ); external name '_DisposeComponentMPWorkFunctionUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure DisposeComponentMPWorkFunctionUPP(userUPP: ComponentMPWorkFunctionUPP); external name '_DisposeComponentMPWorkFunctionUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeComponentMPWorkFunctionUPP( userUPP: ComponentMPWorkFunctionUPP ); external name '_DisposeComponentMPWorkFunctionUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  DisposeComponentRoutineUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeComponentRoutineUPP( userUPP: ComponentRoutineUPP ); external name '_DisposeComponentRoutineUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure DisposeComponentRoutineUPP(userUPP: ComponentRoutineUPP); external name '_DisposeComponentRoutineUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeComponentRoutineUPP( userUPP: ComponentRoutineUPP ); external name '_DisposeComponentRoutineUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  DisposeGetMissingComponentResourceUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeGetMissingComponentResourceUPP( userUPP: GetMissingComponentResourceUPP ); external name '_DisposeGetMissingComponentResourceUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure DisposeGetMissingComponentResourceUPP(userUPP: GetMissingComponentResourceUPP); external name '_DisposeGetMissingComponentResourceUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeGetMissingComponentResourceUPP( userUPP: GetMissingComponentResourceUPP ); external name '_DisposeGetMissingComponentResourceUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  InvokeComponentMPWorkFunctionUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeComponentMPWorkFunctionUPP( globalRefCon: UnivPtr; header: ComponentMPWorkFunctionHeaderRecordPtr; userUPP: ComponentMPWorkFunctionUPP ): ComponentResult; external name '_InvokeComponentMPWorkFunctionUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function InvokeComponentMPWorkFunctionUPP(globalRefCon: UnivPtr; header: ComponentMPWorkFunctionHeaderRecordPtr; userRoutine: ComponentMPWorkFunctionUPP): ComponentResult; external name '_InvokeComponentMPWorkFunctionUPP'; { old name was CallComponentMPWorkFunctionProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function InvokeComponentMPWorkFunctionUPP( globalRefCon: UnivPtr; header: ComponentMPWorkFunctionHeaderRecordPtr; userUPP: ComponentMPWorkFunctionUPP ): ComponentResult; external name '_InvokeComponentMPWorkFunctionUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  InvokeComponentRoutineUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeComponentRoutineUPP( var cp: ComponentParameters; componentStorage: Handle; userUPP: ComponentRoutineUPP ): ComponentResult; external name '_InvokeComponentRoutineUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function InvokeComponentRoutineUPP(var cp: ComponentParameters; componentStorage: Handle; userRoutine: ComponentRoutineUPP): ComponentResult; external name '_InvokeComponentRoutineUPP'; { old name was CallComponentRoutineProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function InvokeComponentRoutineUPP( var cp: ComponentParameters; componentStorage: Handle; userUPP: ComponentRoutineUPP ): ComponentResult; external name '_InvokeComponentRoutineUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  InvokeGetMissingComponentResourceUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeGetMissingComponentResourceUPP( c: Component; resType: OSType; resID: SInt16; refCon: UnivPtr; var resource: Handle; userUPP: GetMissingComponentResourceUPP ): OSErr; external name '_InvokeGetMissingComponentResourceUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew

{ ProcInfos }

{ MixedMode ProcInfo constants for component calls }
const
	uppComponentFunctionImplementedProcInfo = $000002F0;
	uppGetComponentVersionProcInfo = $000000F0;
	uppComponentSetTargetProcInfo = $000003F0;
	uppCallComponentOpenProcInfo = $000003F0;
	uppCallComponentCloseProcInfo = $000003F0;
	uppCallComponentCanDoProcInfo = $000002F0;
	uppCallComponentVersionProcInfo = $000000F0;
	uppCallComponentRegisterProcInfo = $000000F0;
	uppCallComponentTargetProcInfo = $000003F0;
	uppCallComponentUnregisterProcInfo = $000000F0;
	uppCallComponentGetMPWorkFunctionProcInfo = $00000FF0;
	uppCallComponentGetPublicResourceProcInfo = $00003BF0;

{$endc} {TARGET_OS_MAC}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeGetMissingComponentResourceUPP( c: Component; resType: OSType; resID: SInt16; refCon: UnivPtr; var resource: Handle; userUPP: GetMissingComponentResourceUPP ): OSErr; external name '_InvokeGetMissingComponentResourceUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{ ProcInfos }

{ MixedMode ProcInfo constants for component calls }
const
	uppComponentFunctionImplementedProcInfo = $000002F0;
	uppGetComponentVersionProcInfo = $000000F0;
	uppComponentSetTargetProcInfo = $000003F0;
	uppCallComponentOpenProcInfo = $000003F0;
	uppCallComponentCloseProcInfo = $000003F0;
	uppCallComponentCanDoProcInfo = $000002F0;
	uppCallComponentVersionProcInfo = $000000F0;
	uppCallComponentRegisterProcInfo = $000000F0;
	uppCallComponentTargetProcInfo = $000003F0;
	uppCallComponentUnregisterProcInfo = $000000F0;
	uppCallComponentGetMPWorkFunctionProcInfo = $00000FF0;
	uppCallComponentGetPublicResourceProcInfo = $00003BF0;

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
