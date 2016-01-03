{	CFDate.h
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	Copyright (c) 1998-2013, Apple Inc. All rights reserved.
}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	Copyright (c) 1998-2009, Apple, Inc. All rights reserved.
}
{   Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, September 2005 }
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
	Copyright (c) 1998-2005, Apple, Inc. All rights reserved.
=======
	Copyright (c) 1998-2009, Apple, Inc. All rights reserved.
>>>>>>> origin/fixes_2.4
}
{   Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, September 2005 }
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

unit CFDate;
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
uses MacTypes,CFBase;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not MACOSALLINCLUDE}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {not MACOSALLINCLUDE}

>>>>>>> origin/fixes_2.4
{$ALIGN POWER}


type
	CFTimeInterval = Float64;
	CFAbsoluteTime = CFTimeInterval;
	CFAbsoluteTimePtr = ^CFAbsoluteTime;
{ absolute time is the time interval since the reference date }
{ the reference date (epoch) is 00:00:00 1 January 2001. }

function CFAbsoluteTimeGetCurrent: CFAbsoluteTime; external name '_CFAbsoluteTimeGetCurrent';

var kCFAbsoluteTimeIntervalSince1970: CFTimeInterval; external name '_kCFAbsoluteTimeIntervalSince1970'; (* attribute const *)
var kCFAbsoluteTimeIntervalSince1904: CFTimeInterval; external name '_kCFAbsoluteTimeIntervalSince1904'; (* attribute const *)

type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CFDateRef = ^__CFDate; { an opaque type }
	__CFDate = record end;
=======
	CFDateRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CFDateRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CFDateRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CFDateRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	CFDateRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> graemeg/fixes_2_2
=======
	CFDateRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> origin/fixes_2_2
=======
	CFDateRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4
	CFDateRefPtr = ^CFDateRef;

function CFDateGetTypeID: CFTypeID; external name '_CFDateGetTypeID';

function CFDateCreate( allocator: CFAllocatorRef; at: CFAbsoluteTime ): CFDateRef; external name '_CFDateCreate';

function CFDateGetAbsoluteTime( theDate: CFDateRef ): CFAbsoluteTime; external name '_CFDateGetAbsoluteTime';

function CFDateGetTimeIntervalSinceDate( theDate: CFDateRef; otherDate: CFDateRef ): CFTimeInterval; external name '_CFDateGetTimeIntervalSinceDate';

function CFDateCompare( theDate: CFDateRef; otherDate: CFDateRef; context: UnivPtr ): CFComparisonResult; external name '_CFDateCompare';

type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CFTimeZoneRef = ^__CFTimeZone; { an opaque type }
	__CFTimeZone = record end;
=======
	CFTimeZoneRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CFTimeZoneRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CFTimeZoneRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CFTimeZoneRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	CFTimeZoneRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> graemeg/fixes_2_2
=======
	CFTimeZoneRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> origin/fixes_2_2
=======
	CFTimeZoneRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4
	CFTimeZoneRefPtr = ^CFTimeZoneRef;

type
	CFGregorianDate = record
		year: SInt32;
		month: SInt8;
		day: SInt8;
		hour: SInt8;
		minute: SInt8;
		second: Float64;
	end;
<<<<<<< HEAD
<<<<<<< HEAD
	(* CF_CALENDAR_DEPRECATED(10_4, 10_9, 2_0, 7_0, "Use CFCalendar or NSCalendar API instead") *)
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
	CFGregorianDatePtr = ^CFGregorianDate;

type
	CFGregorianUnits = record
		years: SInt32;
		months: SInt32;
		days: SInt32;
		hours: SInt32;
		minutes: SInt32;
		seconds: Float64;
	end;
<<<<<<< HEAD
<<<<<<< HEAD
	(* CF_CALENDAR_DEPRECATED(10_4, 10_9, 2_0, 7_0, "Use CFCalendar or NSCalendar API instead") *)
	CFGregorianUnitsPtr = ^CFGregorianUnits;

type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CFGregorianUnitFlags = CFOptionFlags;
=======
	CFGregorianUnitFlags = UNSIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	CFGregorianUnitFlags = UNSIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	CFGregorianUnitFlags = UNSIGNEDLONG;
>>>>>>> graemeg/cpstrnew
=======
	CFGregorianUnitFlags = UNSIGNEDLONG;
>>>>>>> origin/cpstrnew
const
	kCFGregorianUnitsYears = 1 shl 0;
	(* CF_CALENDAR_DEPRECATED(10_4, 10_9, 2_0, 7_0, "Use CFCalendar or NSCalendar API instead") *)
	kCFGregorianUnitsMonths = 1 shl 1;
	(* CF_CALENDAR_DEPRECATED(10_4, 10_9, 2_0, 7_0, "Use CFCalendar or NSCalendar API instead") *)
	kCFGregorianUnitsDays = 1 shl 2;
	(* CF_CALENDAR_DEPRECATED(10_4, 10_9, 2_0, 7_0, "Use CFCalendar or NSCalendar API instead") *)
	kCFGregorianUnitsHours = 1 shl 3;
	(* CF_CALENDAR_DEPRECATED(10_4, 10_9, 2_0, 7_0, "Use CFCalendar or NSCalendar API instead") *)
	kCFGregorianUnitsMinutes = 1 shl 4;
	(* CF_CALENDAR_DEPRECATED(10_4, 10_9, 2_0, 7_0, "Use CFCalendar or NSCalendar API instead") *)
	kCFGregorianUnitsSeconds = 1 shl 5;
	(* CF_CALENDAR_DEPRECATED(10_4, 10_9, 2_0, 7_0, "Use CFCalendar or NSCalendar API instead") *)
	kCFGregorianAllUnits = $00FFFFFF;
	(* CF_CALENDAR_DEPRECATED(10_4, 10_9, 2_0, 7_0, "Use CFCalendar or NSCalendar API instead") *)

function CFGregorianDateIsValid( gdate: CFGregorianDate; unitFlags: CFOptionFlags ): Boolean; external name '_CFGregorianDateIsValid';
(* CF_CALENDAR_DEPRECATED(10_4, 10_9, 2_0, 7_0, "Use CFCalendar or NSCalendar API instead") *)

function CFGregorianDateGetAbsoluteTime( gdate: CFGregorianDate; tz: CFTimeZoneRef ): CFAbsoluteTime; external name '_CFGregorianDateGetAbsoluteTime'; 
(* CF_CALENDAR_DEPRECATED(10_4, 10_9, 2_0, 7_0, "Use CFCalendar or NSCalendar API instead") *)

function CFAbsoluteTimeGetGregorianDate( at: CFAbsoluteTime; tz: CFTimeZoneRef ): CFGregorianDate; external name '_CFAbsoluteTimeGetGregorianDate';
(* CF_CALENDAR_DEPRECATED(10_4, 10_9, 2_0, 7_0, "Use CFCalendar or NSCalendar API instead") *)

function CFAbsoluteTimeAddGregorianUnits( at: CFAbsoluteTime; tz: CFTimeZoneRef; units: CFGregorianUnits ): CFAbsoluteTime; external name '_CFAbsoluteTimeAddGregorianUnits';
(* CF_CALENDAR_DEPRECATED(10_4, 10_9, 2_0, 7_0, "Use CFCalendar or NSCalendar API instead") *)

function CFAbsoluteTimeGetDifferenceAsGregorianUnits( at1: CFAbsoluteTime; at2: CFAbsoluteTime; tz: CFTimeZoneRef; unitFlags: CFOptionFlags ): CFGregorianUnits; external name '_CFAbsoluteTimeGetDifferenceAsGregorianUnits';
(* CF_CALENDAR_DEPRECATED(10_4, 10_9, 2_0, 7_0, "Use CFCalendar or NSCalendar API instead") *)

function CFAbsoluteTimeGetDayOfWeek( at: CFAbsoluteTime; tz: CFTimeZoneRef ): SInt32; external name '_CFAbsoluteTimeGetDayOfWeek';
(* CF_CALENDAR_DEPRECATED(10_4, 10_9, 2_0, 7_0, "Use CFCalendar or NSCalendar API instead") *)

function CFAbsoluteTimeGetDayOfYear( at: CFAbsoluteTime; tz: CFTimeZoneRef ): SInt32; external name '_CFAbsoluteTimeGetDayOfYear';
(* CF_CALENDAR_DEPRECATED(10_4, 10_9, 2_0, 7_0, "Use CFCalendar or NSCalendar API instead") *)

function CFAbsoluteTimeGetWeekOfYear( at: CFAbsoluteTime; tz: CFTimeZoneRef ): SInt32; external name '_CFAbsoluteTimeGetWeekOfYear';
(* CF_CALENDAR_DEPRECATED(10_4, 10_9, 2_0, 7_0, "Use CFCalendar or NSCalendar API instead") *)

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
	CFGregorianUnitsPtr = ^CFGregorianUnits;

type
	CFGregorianUnitFlags = UNSIGNEDLONG;
const
	kCFGregorianUnitsYears = 1 shl 0;
	kCFGregorianUnitsMonths = 1 shl 1;
	kCFGregorianUnitsDays = 1 shl 2;
	kCFGregorianUnitsHours = 1 shl 3;
	kCFGregorianUnitsMinutes = 1 shl 4;
	kCFGregorianUnitsSeconds = 1 shl 5;
	kCFGregorianAllUnits = $00FFFFFF;

function CFGregorianDateIsValid( gdate: CFGregorianDate; unitFlags: CFOptionFlags ): Boolean; external name '_CFGregorianDateIsValid';

function CFGregorianDateGetAbsoluteTime( gdate: CFGregorianDate; tz: CFTimeZoneRef ): CFAbsoluteTime; external name '_CFGregorianDateGetAbsoluteTime';

function CFAbsoluteTimeGetGregorianDate( at: CFAbsoluteTime; tz: CFTimeZoneRef ): CFGregorianDate; external name '_CFAbsoluteTimeGetGregorianDate';

function CFAbsoluteTimeAddGregorianUnits( at: CFAbsoluteTime; tz: CFTimeZoneRef; units: CFGregorianUnits ): CFAbsoluteTime; external name '_CFAbsoluteTimeAddGregorianUnits';

function CFAbsoluteTimeGetDifferenceAsGregorianUnits( at1: CFAbsoluteTime; at2: CFAbsoluteTime; tz: CFTimeZoneRef; unitFlags: CFOptionFlags ): CFGregorianUnits; external name '_CFAbsoluteTimeGetDifferenceAsGregorianUnits';

function CFAbsoluteTimeGetDayOfWeek( at: CFAbsoluteTime; tz: CFTimeZoneRef ): SInt32; external name '_CFAbsoluteTimeGetDayOfWeek';

function CFAbsoluteTimeGetDayOfYear( at: CFAbsoluteTime; tz: CFTimeZoneRef ): SInt32; external name '_CFAbsoluteTimeGetDayOfYear';

function CFAbsoluteTimeGetWeekOfYear( at: CFAbsoluteTime; tz: CFTimeZoneRef ): SInt32; external name '_CFAbsoluteTimeGetWeekOfYear';

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
