{	CFCalendar.h
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	Copyright (c) 2004-2013, Apple Inc. All rights reserved.
}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	Copyright (c) 2004-2009, Apple Inc. All rights reserved.
}
{       Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Gorazd Krosl, <gorazd_1957@yahoo.ca>, October 2009 }
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
	Copyright (c) 2004-2005, Apple, Inc. All rights reserved.
=======
	Copyright (c) 2004-2009, Apple Inc. All rights reserved.
>>>>>>> origin/fixes_2.4
}
{       Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Gorazd Krosl, <gorazd_1957@yahoo.ca>, October 2009 }
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

unit CFCalendar;
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
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
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
>>>>>>> origin/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
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
uses MacTypes,CFBase,CFLocale,CFDate,CFTimeZone;
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


{#if MAC_OS_X_VERSION_MAX_ALLOWED >= MAC_OS_X_VERSION_10_4}


type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CFCalendarRef = ^__CFCalendar; { an opaque type }
	__CFCalendar = record end;
=======
	CFCalendarRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CFCalendarRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CFCalendarRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CFCalendarRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	CFCalendarRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> graemeg/fixes_2_2
=======
	CFCalendarRef = ^SInt32; { an opaque 32-bit type }
>>>>>>> origin/fixes_2_2
=======
	CFCalendarRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4

function CFCalendarGetTypeID: CFTypeID; external name '_CFCalendarGetTypeID';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

function CFCalendarCopyCurrent: CFCalendarRef; external name '_CFCalendarCopyCurrent';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

function CFCalendarCreateWithIdentifier( allocator: CFAllocatorRef; identifier: CFStringRef ): CFCalendarRef; external name '_CFCalendarCreateWithIdentifier';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
	// Create a calendar.  The identifiers are the kCF*Calendar
	// constants in CFLocale.h.

function CFCalendarGetIdentifier( calendar: CFCalendarRef ): CFStringRef; external name '_CFCalendarGetIdentifier';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
	// Returns the calendar's identifier.

function CFCalendarCopyLocale( calendar: CFCalendarRef ): CFLocaleRef; external name '_CFCalendarCopyLocale';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

procedure CFCalendarSetLocale( calendar: CFCalendarRef; locale: CFLocaleRef ); external name '_CFCalendarSetLocale';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

function CFCalendarCopyTimeZone( calendar: CFCalendarRef ): CFTimeZoneRef; external name '_CFCalendarCopyTimeZone';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

procedure CFCalendarSetTimeZone( calendar: CFCalendarRef; tz: CFTimeZoneRef ); external name '_CFCalendarSetTimeZone';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

function CFCalendarGetFirstWeekday( calendar: CFCalendarRef ): CFIndex; external name '_CFCalendarGetFirstWeekday';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

procedure CFCalendarSetFirstWeekday( calendar: CFCalendarRef; wkdy: CFIndex ); external name '_CFCalendarSetFirstWeekday';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

function CFCalendarGetMinimumDaysInFirstWeek( calendar: CFCalendarRef ): CFIndex; external name '_CFCalendarGetMinimumDaysInFirstWeek';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

procedure CFCalendarSetMinimumDaysInFirstWeek( calendar: CFCalendarRef; mwd: CFIndex ); external name '_CFCalendarSetMinimumDaysInFirstWeek';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CFCalendarUnit = UNSIGNEDLONG;
=======
	CFCalendarUnit = SInt32;
>>>>>>> graemeg/fixes_2_2
=======
	CFCalendarUnit = SInt32;
>>>>>>> origin/fixes_2_2
=======
	CFCalendarUnit = UNSIGNEDLONG;
>>>>>>> origin/fixes_2.4
const
	kCFCalendarUnitEra = 1 shl 1;
	kCFCalendarUnitYear = 1 shl 2;
	kCFCalendarUnitMonth = 1 shl 3;
	kCFCalendarUnitDay = 1 shl 4;
	kCFCalendarUnitHour = 1 shl 5;
	kCFCalendarUnitMinute = 1 shl 6;
	kCFCalendarUnitSecond = 1 shl 7;
	kCFCalendarUnitWeek = 1 shl 8;
	kCFCalendarUnitWeekday = 1 shl 9;
	kCFCalendarUnitWeekdayOrdinal = 1 shl 10;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kCFCalendarUnitQuarter = 1 shl 11; { CF_AVAILABLE_STARTING(10_6, 4_0); }
	kCFCalendarUnitWeekOfMonth = 1 shl 12; { CF_AVAILABLE_STARTING(10_7, 5_0) }
	kCFCalendarUnitWeekOfYear = 1 shl 13; { CF_AVAILABLE_STARTING(10_7, 5_0) }
	kCFCalendarUnitYearForWeekOfYear = 1 shl 14; { CF_AVAILABLE_STARTING(10_7, 5_0) }
=======
{#if MAC_OS_X_VERSION_10_6 <= MAC_OS_X_VERSION_MAX_ALLOWED}
	kCFCalendarUnitQuarter = 1 shl 11;
{#endif}
>>>>>>> graemeg/cpstrnew
=======
{#if MAC_OS_X_VERSION_10_6 <= MAC_OS_X_VERSION_MAX_ALLOWED}
	kCFCalendarUnitQuarter = 1 shl 11;
{#endif}
>>>>>>> graemeg/cpstrnew
=======
{#if MAC_OS_X_VERSION_10_6 <= MAC_OS_X_VERSION_MAX_ALLOWED}
	kCFCalendarUnitQuarter = 1 shl 11;
{#endif}
>>>>>>> graemeg/cpstrnew
=======
{#if MAC_OS_X_VERSION_10_6 <= MAC_OS_X_VERSION_MAX_ALLOWED}
	kCFCalendarUnitQuarter = 1 shl 11;
{#endif}
>>>>>>> origin/cpstrnew
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{#if MAC_OS_X_VERSION_10_6 <= MAC_OS_X_VERSION_MAX_ALLOWED}
	kCFCalendarUnitQuarter = 1 shl 11;
{#endif}
>>>>>>> origin/fixes_2.4

function CFCalendarGetMinimumRangeOfUnit( calendar: CFCalendarRef; unt: CFCalendarUnit ): CFRange; external name '_CFCalendarGetMinimumRangeOfUnit';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

function CFCalendarGetMaximumRangeOfUnit( calendar: CFCalendarRef; unt: CFCalendarUnit ): CFRange; external name '_CFCalendarGetMaximumRangeOfUnit';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

function CFCalendarGetRangeOfUnit( calendar: CFCalendarRef; smallerUnit: CFCalendarUnit; biggerUnit: CFCalendarUnit; at: CFAbsoluteTime ): CFRange; external name '_CFCalendarGetRangeOfUnit';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

function CFCalendarGetOrdinalityOfUnit( calendar: CFCalendarRef; smallerUnit: CFCalendarUnit; biggerUnit: CFCalendarUnit; at: CFAbsoluteTime ): CFIndex; external name '_CFCalendarGetOrdinalityOfUnit';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function CFCalendarGetTimeRangeOfUnit( calendar: CFCalendarRef; unt: CFCalendarUnit; at: CFAbsoluteTime; var startp: CFAbsoluteTime; var tip: CFTimeInterval ): Boolean; external name '_CFCalendarGetTimeRangeOfUnit';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CFCalendarGetTimeRangeOfUnit( calendar: CFCalendarRef; unt: CFCalendarUnit; at: CFAbsoluteTime; var startp: CFAbsoluteTime; var tip: CFTimeInterval ): Boolean; external name '_CFCalendarGetTimeRangeOfUnit';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4

function CFCalendarComposeAbsoluteTime( calendar: CFCalendarRef; var at: { out } CFAbsoluteTime; componentDesc: ConstCStringPtr; ... ): Boolean; external name '_CFCalendarComposeAbsoluteTime';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

function CFCalendarDecomposeAbsoluteTime( calendar: CFCalendarRef; at: CFAbsoluteTime; componentDesc: ConstCStringPtr; ... ): Boolean; external name '_CFCalendarDecomposeAbsoluteTime';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


const
	kCFCalendarComponentsWrap = 1 shl 0;  // option for adding

<<<<<<< HEAD
<<<<<<< HEAD
function CFCalendarAddComponents( calendar: CFCalendarRef; var at: { inout } CFAbsoluteTime; options: CFOptionFlags; componentDesc: ConstCStringPtr; ... ): Boolean; external name '_CFCalendarAddComponents';
=======
function CFCalendarAddComponents( calendar: CFCalendarRef; var at: { out } CFAbsoluteTime; options: CFOptionFlags; componentDesc: ConstCStringPtr; ... ): Boolean; external name '_CFCalendarAddComponents';
>>>>>>> graemeg/fixes_2_2
=======
function CFCalendarAddComponents( calendar: CFCalendarRef; var at: { out } CFAbsoluteTime; options: CFOptionFlags; componentDesc: ConstCStringPtr; ... ): Boolean; external name '_CFCalendarAddComponents';
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

function CFCalendarGetComponentDifference( calendar: CFCalendarRef; startingAT: CFAbsoluteTime; resultAT: CFAbsoluteTime; options: CFOptionFlags; componentDesc: ConstCStringPtr; ... ): Boolean; external name '_CFCalendarGetComponentDifference';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)



{#endif}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======

end.
>>>>>>> graemeg/fixes_2_2
=======

end.
>>>>>>> origin/fixes_2_2
=======
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4
