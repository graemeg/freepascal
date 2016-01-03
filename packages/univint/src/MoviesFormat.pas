{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       QuickTime/MoviesFormat.h
 
     Contains:   QuickTime Interfaces.
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Version:    QuickTime 7.7.1
 
     Copyright:  © 1990-2012 by Apple Inc., all rights reserved
=======
     Version:    QuickTime 7.6.3
 
     Copyright:  © 1990-2008 by Apple Inc., all rights reserved
>>>>>>> graemeg/cpstrnew
=======
     Version:    QuickTime 7.6.3
 
     Copyright:  © 1990-2008 by Apple Inc., all rights reserved
>>>>>>> graemeg/cpstrnew
=======
     Version:    QuickTime 7.6.3
 
     Copyright:  © 1990-2008 by Apple Inc., all rights reserved
>>>>>>> graemeg/cpstrnew
=======
     Version:    QuickTime 7.6.3
 
     Copyright:  © 1990-2008 by Apple Inc., all rights reserved
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
     File:       MoviesFormat.p
=======
     File:       QuickTime/MoviesFormat.h
>>>>>>> origin/fixes_2.4
 
     Contains:   QuickTime Interfaces.
 
     Version:    QuickTime 7.6.3
 
<<<<<<< HEAD
     Copyright:  © 1990-2002 by Apple Computer, Inc., all rights reserved
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  © 1990-2008 by Apple Inc., all rights reserved
>>>>>>> origin/fixes_2.4
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
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
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit MoviesFormat;
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
>>>>>>> graemeg/cpstrnew
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
=======
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
{$elsec}
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
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
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
{$setc TARGET_OS_UNIX := FALSE}
{$setc TARGET_OS_WIN32 := FALSE}
{$setc TARGET_RT_MAC_68881 := FALSE}
{$setc TARGET_RT_MAC_CFM := FALSE}
{$setc TARGET_RT_MAC_MACHO := TRUE}
{$setc TYPED_FUNCTION_POINTERS := TRUE}
{$setc TYPE_BOOL := FALSE}
{$setc TYPE_EXTENDED := FALSE}
{$setc TYPE_LONGLONG := TRUE}
uses MacTypes,ImageCompression,Components,Movies;
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
=======
=======
>>>>>>> origin/cpstrnew

{$ifc TARGET_OS_MAC}

{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}
>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ALIGN MAC68K}


=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
const
	kMovieVersion = 0;     { version number of the format here described }

{***************************************
*
*   General Types -
*       These types are used in more than one of the
*       directory types.
*
***************************************}
{ MoviesUserData is the type used for user data in movie and track directories }
type
	MoviesUserDataPtr = ^MoviesUserData;
	MoviesUserData = record
		size: SInt32;                   { size of this user data }
		udType: SInt32;                 { type of user data }
		data: array [0..0] of SInt8;                { the user data }
	end;
type
	UserDataAtomPtr = ^UserDataAtom;
	UserDataAtom = record
		size: SInt32;
		atomType: SInt32;
		userData: array [0..0] of MoviesUserData;
	end;
{ MoviesDataDescription tells us where the data for the movie or track lives.
   The data can follow the directory, be in the datafork of the same file as the directory resource,
   be in the resource fork of the same file as the directory resource, be in another file in the
   data fork or resource fork, or require a specific bottleneck to fetch the data. }
{***************************************
*
*   MediaDirectory information -
*       The MediaDirectory is tightly coupled to the data.
*
***************************************}
{ SampleDescription is in Movies.h }
type
	SampleDescriptionAtomPtr = ^SampleDescriptionAtom;
	SampleDescriptionAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'stsd' }
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }
		numEntries: SInt32;
		sampleDescTable: array [0..0] of SampleDescription;
	end;
{ TimeToSampleNum maps physical sample time to physical sample number. }
type
	TimeToSampleNumPtr = ^TimeToSampleNum;
	TimeToSampleNum = record
		sampleCount: SInt32;
		sampleDuration: TimeValue;
	end;
type
	TimeToSampleNumAtomPtr = ^TimeToSampleNumAtom;
	TimeToSampleNumAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'stts' }
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }
		numEntries: SInt32;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		timeToSampleNumTable: array [0..0] of TimeToSampleNum;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
		timeToSampleNumTable:	array [0..0] of TimeToSampleNum;
>>>>>>> graemeg/cpstrnew
	end;
{ SyncSamples is a list of the physical samples which are self contained. }
type
	SyncSampleAtomPtr = ^SyncSampleAtom;
	SyncSampleAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'stss' }
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }
		numEntries: SInt32;
		syncSampleTable: array [0..0] of SInt32;
	end;
{ SampleToChunk maps physical sample number to chunk number. }
{ same as SampleToChunk, but redundant first sample is removed }
type
	SampleToChunkPtr = ^SampleToChunk;
	SampleToChunk = record
		firstChunk: SInt32;
		samplesPerChunk: SInt32;
		sampleDescriptionID: SInt32;
	end;
type
	SampleToChunkAtomPtr = ^SampleToChunkAtom;
	SampleToChunkAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'stsc' }
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }
		numEntries: SInt32;
		sampleToChunkTable: array [0..0] of SampleToChunk;
	end;
type
	ChunkOffsetAtomPtr = ^ChunkOffsetAtom;
	ChunkOffsetAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'stco' }
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }
		numEntries: SInt32;
		chunkOffsetTable: array [0..0] of SInt32;
	end;
type
	SampleSizeAtomPtr = ^SampleSizeAtom;
	SampleSizeAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'stsz' }
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }
		sampleSize: SInt32;
		numEntries: SInt32;
		sampleSizeTable: array [0..0] of SInt32;
	end;
type
	ShadowSyncPtr = ^ShadowSync;
	ShadowSync = record
		fdSampleNum: SInt32;
		syncSampleNum: SInt32;
	end;
type
	ShadowSyncAtomPtr = ^ShadowSyncAtom;
	ShadowSyncAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'stsz' }
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }
		numEntries: SInt32;
		shadowSyncTable: array [0..0] of ShadowSync;
	end;
{ CompositionOffsetEntry maps sample numbers to composition offsets. }
type
	CompositionOffsetEntryPtr = ^CompositionOffsetEntry;
	CompositionOffsetEntry = record
		sampleCount: SInt32;
		displayOffset: TimeValue;
	end;
type
	CompositionOffsetAtomPtr = ^CompositionOffsetAtom;
	CompositionOffsetAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'ctts' }
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }
		numEntries: SInt32;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		compositionOffsetTable: array [0..0] of CompositionOffsetEntry;
=======
    compositionOffsetTable: array [0..0] of CompositionOffsetEntry;
>>>>>>> graemeg/cpstrnew
	end;
type
	SampleDependencyAtomPtr = ^SampleDependencyAtom;
	SampleDependencyAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'sdtp' }
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }
		sampleDependencyTable: array [0..0] of UInt8;
	end;
=======
=======
>>>>>>> graemeg/cpstrnew
    compositionOffsetTable: array [0..0] of CompositionOffsetEntry;
	end;
type
	SampleDependencyAtomPtr = ^SampleDependencyAtom;
	SampleDependencyAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'sdtp' }
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }
		sampleDependencyTable: array [0..0] of UInt8;
	end;
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
    compositionOffsetTable: array [0..0] of CompositionOffsetEntry;
	end;
type
	SampleDependencyAtomPtr = ^SampleDependencyAtom;
	SampleDependencyAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'sdtp' }
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }
		sampleDependencyTable: array [0..0] of UInt8;
	end;
>>>>>>> origin/cpstrnew
{
NOTE: The values for these flags that shipped with QuickTime 7.0 were incorrect. 
They matched neither the specification nor the implementation -- the "Yes" and "No" bits
were reversed.  The flags have been corrected but renamed to ensure that code using
the incorrect flags is reviewed by developers.
enum (
    kQTSampleDependency_DependsOnOthers = 1<<5,         // INCORRECT VALUE
    kQTSampleDependency_DoesNotDependOnOthers = 1<<4,   // INCORRECT VALUE
    kQTSampleDependency_IsDependedOnByOthers = 1<<3,    // INCORRECT VALUE
    kQTSampleDependency_IsNotDependedOnByOthers = 1<<2, // INCORRECT VALUE
    kQTSampleDependency_HasRedundantCoding = 1<<1,      // INCORRECT VALUE
    kQTSampleDependency_HasNoRedundantCoding = 1<<0     // INCORRECT VALUE
);
}
{ Values for entries in SampleDependencyAtom.sampleDependencyTable[]}
const
{ bit 0x80 is reserved; bit combinations 0x30, 0xC0 and 0x03 are reserved}
	kQTSampleDependency_EarlierDisplayTimesAllowed = 1 shl 6; { corresponds to flag mediaSampleEarlierDisplayTimesAllowed except at different bit offset}
	kQTSampleDependency_DoesNotDependOnOthers_Corrected = 1 shl 5; { ie: an I picture}
	kQTSampleDependency_DependsOnOthers_Corrected = 1 shl 4; { ie: not an I picture}
	kQTSampleDependency_IsNotDependedOnByOthers_Corrected = 1 shl 3; { mediaSampleDroppable}
	kQTSampleDependency_IsDependedOnByOthers_Corrected = 1 shl 2;
	kQTSampleDependency_HasNoRedundantCoding_Corrected = 1 shl 1;
	kQTSampleDependency_HasRedundantCoding_Corrected = 1 shl 0;

type
	CompositionShiftLeastGreatestAtomPtr = ^CompositionShiftLeastGreatestAtom;
	CompositionShiftLeastGreatestAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'cslg' }
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }
		compositionOffsetToDTDDeltaShift: SInt32;
		leastDecodeToDisplayDelta: SInt32;
		greatestDecodeToDisplayDelta: SInt32;
		displayStartTime: SInt32;
		displayEndTime: SInt32;
	end;
type
	PartialSyncSampleAtomPtr = ^PartialSyncSampleAtom;
	PartialSyncSampleAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'stps' }
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }
		numEntries: SInt32;
		partialSyncSampleTable: array [0..0] of UInt32;
	end;
type
	SampleTableAtomPtr = ^SampleTableAtom;
	SampleTableAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'stbl' }

		sampleDescription: SampleDescriptionAtom;
		timeToSampleNum: TimeToSampleNumAtom;
		sampleToChunk: SampleToChunkAtom;
		syncSample: SyncSampleAtom;
		sampleSize: SampleSizeAtom;
		chunkOffset: ChunkOffsetAtom;
		shadowSync: ShadowSyncAtom;
	end;
type
	PublicHandlerInfoPtr = ^PublicHandlerInfo;
	PublicHandlerInfo = record
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }

		componentType: SInt32;
		componentSubType: SInt32;
		componentManufacturer: SInt32;
		componentFlags: SInt32;
		componentFlagsMask: SInt32;
		componentName: array [0..0] of SInt8;
	end;
type
	HandlerAtomPtr = ^HandlerAtom;
	HandlerAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'hdlr' }

		hInfo: PublicHandlerInfo;
	end;
{ a data reference is a private structure }

type
	DataRefAtom = SInt32;
	DataInfoAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'dinf' }

		dataRef: DataRefAtom;
	end;
type
	RgnAtomPtr = ^RgnAtom;
	RgnAtom = record
		size: SInt32;
		atomType: SInt32;

		rgnSize: SInt16;
		rgnBBox: Rect;
		data: array [0..0] of SInt8;
	end;
type
	MatteCompressedAtomPtr = ^MatteCompressedAtom;
	MatteCompressedAtom = record
		size: SInt32;
		atomType: SInt32;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }

=======

		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }

>>>>>>> graemeg/cpstrnew
=======

		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }

>>>>>>> graemeg/cpstrnew
=======

		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }

>>>>>>> origin/cpstrnew
		matteImageDescription: ImageDescription;
		matteData: array [0..0] of SInt8;
	end;
type
	MatteAtomPtr = ^MatteAtom;
	MatteAtom = record
		size: SInt32;
		atomType: SInt32;

		aCompressedMatte: MatteCompressedAtom;
	end;
type
	ClippingAtomPtr = ^ClippingAtom;
	ClippingAtom = record
		size: SInt32;
		atomType: SInt32;

		aRgnClip: RgnAtom;
	end;
{**********************
* Media Info Example Structures
**********************}

type
	VideoMediaInfoHeaderPtr = ^VideoMediaInfoHeader;
	VideoMediaInfoHeader = record
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }

		graphicsMode: SInt16;           { for QD - transfer mode }
		opColorRed: SInt16;             { opcolor for transfer mode }
		opColorGreen: SInt16;
		opColorBlue: SInt16;
	end;
type
	VideoMediaInfoHeaderAtomPtr = ^VideoMediaInfoHeaderAtom;
	VideoMediaInfoHeaderAtom = record
		size: SInt32;                   { size of Media info }
		atomType: SInt32;               { = 'vmhd' }
		vmiHeader: VideoMediaInfoHeader;
	end;
type
	VideoMediaInfoPtr = ^VideoMediaInfo;
	VideoMediaInfo = record
		size: SInt32;                   { size of Media info }
		atomType: SInt32;               { = 'minf' }

		header: VideoMediaInfoHeaderAtom;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

		dataHandler: HandlerAtom;

=======

		dataHandler: HandlerAtom;

>>>>>>> graemeg/cpstrnew
=======

		dataHandler: HandlerAtom;

>>>>>>> graemeg/cpstrnew
=======

		dataHandler: HandlerAtom;

>>>>>>> origin/cpstrnew
		dataInfo: DataInfoAtom;

		sampleTable: SampleTableAtom;
	end;
type
	SoundMediaInfoHeaderPtr = ^SoundMediaInfoHeader;
	SoundMediaInfoHeader = record
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }

		balance: SInt16;
		rsrvd: SInt16;
	end;
type
	SoundMediaInfoHeaderAtomPtr = ^SoundMediaInfoHeaderAtom;
	SoundMediaInfoHeaderAtom = record
		size: SInt32;                   { size of Media info }
		atomType: SInt32;               { = 'vmhd' }

		smiHeader: SoundMediaInfoHeader;
	end;
type
	SoundMediaInfoPtr = ^SoundMediaInfo;
	SoundMediaInfo = record
		size: SInt32;                   { size of Media info }
		atomType: SInt32;               { = 'minf' }

		header: SoundMediaInfoHeaderAtom;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

		dataHandler: HandlerAtom;

=======

		dataHandler: HandlerAtom;

>>>>>>> graemeg/cpstrnew
=======

		dataHandler: HandlerAtom;

>>>>>>> graemeg/cpstrnew
=======

		dataHandler: HandlerAtom;

>>>>>>> origin/cpstrnew
		dataReference: DataRefAtom;

		sampleTable: SampleTableAtom;
	end;
{ whatever data the media handler needs goes after the atomType }
type
	MediaInfoPtr = ^MediaInfo;
	MediaInfo = record
		size: SInt32;
		atomType: SInt32;
	end;
	MediaInfo_fix = MediaInfo;
{**********************
* Media Directory Structures
**********************}
type
	MediaHeaderPtr = ^MediaHeader;
	MediaHeader = record
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

		creationTime: SInt32;           { seconds since Jan 1904 when directory was created }
		modificationTime: SInt32;       { seconds since Jan 1904 when directory was appended }

=======
=======
>>>>>>> graemeg/cpstrnew

		creationTime: SInt32;           { seconds since Jan 1904 when directory was created }
		modificationTime: SInt32;       { seconds since Jan 1904 when directory was appended }

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/cpstrnew

		creationTime: SInt32;           { seconds since Jan 1904 when directory was created }
		modificationTime: SInt32;       { seconds since Jan 1904 when directory was appended }

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
		timeScale: TimeValue;              { start time for Media (Media time) }
		duration: TimeValue;               { length of Media (Media time) }

		language: SInt16;
		quality: SInt16;
	end;
type
	MediaHeaderAtomPtr = ^MediaHeaderAtom;
	MediaHeaderAtom = record
		size: SInt32;
		atomType: SInt32;

		header: MediaHeader;
	end;
type
	MediaDirectoryPtr = ^MediaDirectory;
	MediaDirectory = record
		size: SInt32;
		atomType: SInt32;               { = 'mdia' }

		mediaHeader: MediaHeaderAtom;            { standard Media information }

		mediaHandler: HandlerAtom;

		mediaInfo: MediaInfo_fix;
	end;
{**********************
* Track Structures
**********************}
const
	TrackEnable = 1 shl 0;
	TrackInMovie = 1 shl 1;
	TrackInPreview = 1 shl 2;
	TrackInPoster = 1 shl 3;
=======
=======
{$setc TARGET_OS_MAC := TRUE}
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
uses MacTypes,ImageCompression,Components,Movies;
<<<<<<< HEAD
>>>>>>> origin/fixes_2_2
=======
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4


{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}


const
	kMovieVersion = 0;     { version number of the format here described }

{***************************************
*
*   General Types -
*       These types are used in more than one of the
*       directory types.
*
***************************************}
{ MoviesUserData is the type used for user data in movie and track directories }
type
	MoviesUserDataPtr = ^MoviesUserData;
	MoviesUserData = record
		size: SInt32;                   { size of this user data }
		udType: SInt32;                 { type of user data }
		data: array [0..0] of SInt8;                { the user data }
	end;
type
	UserDataAtomPtr = ^UserDataAtom;
	UserDataAtom = record
		size: SInt32;
		atomType: SInt32;
		userData: array [0..0] of MoviesUserData;
	end;
{ MoviesDataDescription tells us where the data for the movie or track lives.
   The data can follow the directory, be in the datafork of the same file as the directory resource,
   be in the resource fork of the same file as the directory resource, be in another file in the
   data fork or resource fork, or require a specific bottleneck to fetch the data. }
{***************************************
*
*   MediaDirectory information -
*       The MediaDirectory is tightly coupled to the data.
*
***************************************}
{ SampleDescription is in Movies.h }
type
	SampleDescriptionAtomPtr = ^SampleDescriptionAtom;
	SampleDescriptionAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'stsd' }
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }
		numEntries: SInt32;
		sampleDescTable: array [0..0] of SampleDescription;
	end;
{ TimeToSampleNum maps physical sample time to physical sample number. }
type
	TimeToSampleNumPtr = ^TimeToSampleNum;
	TimeToSampleNum = record
		sampleCount: SInt32;
		sampleDuration: TimeValue;
	end;
type
	TimeToSampleNumAtomPtr = ^TimeToSampleNumAtom;
	TimeToSampleNumAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'stts' }
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }
		numEntries: SInt32;
		timeToSampleNumTable:	array [0..0] of TimeToSampleNum;
	end;
{ SyncSamples is a list of the physical samples which are self contained. }
type
	SyncSampleAtomPtr = ^SyncSampleAtom;
	SyncSampleAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'stss' }
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }
		numEntries: SInt32;
		syncSampleTable: array [0..0] of SInt32;
	end;
{ SampleToChunk maps physical sample number to chunk number. }
{ same as SampleToChunk, but redundant first sample is removed }
type
	SampleToChunkPtr = ^SampleToChunk;
	SampleToChunk = record
		firstChunk: SInt32;
		samplesPerChunk: SInt32;
		sampleDescriptionID: SInt32;
	end;
type
	SampleToChunkAtomPtr = ^SampleToChunkAtom;
	SampleToChunkAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'stsc' }
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }
		numEntries: SInt32;
		sampleToChunkTable: array [0..0] of SampleToChunk;
	end;
type
	ChunkOffsetAtomPtr = ^ChunkOffsetAtom;
	ChunkOffsetAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'stco' }
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }
		numEntries: SInt32;
		chunkOffsetTable: array [0..0] of SInt32;
	end;
type
	SampleSizeAtomPtr = ^SampleSizeAtom;
	SampleSizeAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'stsz' }
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }
		sampleSize: SInt32;
		numEntries: SInt32;
		sampleSizeTable: array [0..0] of SInt32;
	end;
type
	ShadowSyncPtr = ^ShadowSync;
	ShadowSync = record
		fdSampleNum: SInt32;
		syncSampleNum: SInt32;
	end;
type
	ShadowSyncAtomPtr = ^ShadowSyncAtom;
	ShadowSyncAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'stsz' }
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }
		numEntries: SInt32;
		shadowSyncTable: array [0..0] of ShadowSync;
	end;
{ CompositionOffsetEntry maps sample numbers to composition offsets. }
type
	CompositionOffsetEntryPtr = ^CompositionOffsetEntry;
	CompositionOffsetEntry = record
		sampleCount: SInt32;
		displayOffset: TimeValue;
	end;
type
	CompositionOffsetAtomPtr = ^CompositionOffsetAtom;
	CompositionOffsetAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'ctts' }
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }
		numEntries: SInt32;
    compositionOffsetTable: array [0..0] of CompositionOffsetEntry;
	end;
type
	SampleDependencyAtomPtr = ^SampleDependencyAtom;
	SampleDependencyAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'sdtp' }
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }
		sampleDependencyTable: array [0..0] of UInt8;
	end;
{
NOTE: The values for these flags that shipped with QuickTime 7.0 were incorrect. 
They matched neither the specification nor the implementation -- the "Yes" and "No" bits
were reversed.  The flags have been corrected but renamed to ensure that code using
the incorrect flags is reviewed by developers.
enum (
    kQTSampleDependency_DependsOnOthers = 1<<5,         // INCORRECT VALUE
    kQTSampleDependency_DoesNotDependOnOthers = 1<<4,   // INCORRECT VALUE
    kQTSampleDependency_IsDependedOnByOthers = 1<<3,    // INCORRECT VALUE
    kQTSampleDependency_IsNotDependedOnByOthers = 1<<2, // INCORRECT VALUE
    kQTSampleDependency_HasRedundantCoding = 1<<1,      // INCORRECT VALUE
    kQTSampleDependency_HasNoRedundantCoding = 1<<0     // INCORRECT VALUE
);
}
{ Values for entries in SampleDependencyAtom.sampleDependencyTable[]}
const
{ bit 0x80 is reserved; bit combinations 0x30, 0xC0 and 0x03 are reserved}
	kQTSampleDependency_EarlierDisplayTimesAllowed = 1 shl 6; { corresponds to flag mediaSampleEarlierDisplayTimesAllowed except at different bit offset}
	kQTSampleDependency_DoesNotDependOnOthers_Corrected = 1 shl 5; { ie: an I picture}
	kQTSampleDependency_DependsOnOthers_Corrected = 1 shl 4; { ie: not an I picture}
	kQTSampleDependency_IsNotDependedOnByOthers_Corrected = 1 shl 3; { mediaSampleDroppable}
	kQTSampleDependency_IsDependedOnByOthers_Corrected = 1 shl 2;
	kQTSampleDependency_HasNoRedundantCoding_Corrected = 1 shl 1;
	kQTSampleDependency_HasRedundantCoding_Corrected = 1 shl 0;

type
	CompositionShiftLeastGreatestAtomPtr = ^CompositionShiftLeastGreatestAtom;
	CompositionShiftLeastGreatestAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'cslg' }
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }
		compositionOffsetToDTDDeltaShift: SInt32;
		leastDecodeToDisplayDelta: SInt32;
		greatestDecodeToDisplayDelta: SInt32;
		displayStartTime: SInt32;
		displayEndTime: SInt32;
	end;
type
	PartialSyncSampleAtomPtr = ^PartialSyncSampleAtom;
	PartialSyncSampleAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'stps' }
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }
		numEntries: SInt32;
		partialSyncSampleTable: array [0..0] of UInt32;
	end;
type
	SampleTableAtomPtr = ^SampleTableAtom;
	SampleTableAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'stbl' }

		sampleDescription: SampleDescriptionAtom;
		timeToSampleNum: TimeToSampleNumAtom;
		sampleToChunk: SampleToChunkAtom;
		syncSample: SyncSampleAtom;
		sampleSize: SampleSizeAtom;
		chunkOffset: ChunkOffsetAtom;
		shadowSync: ShadowSyncAtom;
	end;
type
	PublicHandlerInfoPtr = ^PublicHandlerInfo;
	PublicHandlerInfo = record
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }

		componentType: SInt32;
		componentSubType: SInt32;
		componentManufacturer: SInt32;
		componentFlags: SInt32;
		componentFlagsMask: SInt32;
		componentName: array [0..0] of SInt8;
	end;
type
	HandlerAtomPtr = ^HandlerAtom;
	HandlerAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'hdlr' }

		hInfo: PublicHandlerInfo;
	end;
{ a data reference is a private structure }

type
	DataRefAtom = SInt32;
	DataInfoAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'dinf' }

		dataRef: DataRefAtom;
	end;
type
	RgnAtomPtr = ^RgnAtom;
	RgnAtom = record
		size: SInt32;
		atomType: SInt32;

		rgnSize: SInt16;
		rgnBBox: Rect;
		data: array [0..0] of SInt8;
	end;
type
	MatteCompressedAtomPtr = ^MatteCompressedAtom;
	MatteCompressedAtom = record
		size: SInt32;
		atomType: SInt32;

		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }

		matteImageDescription: ImageDescription;
		matteData: array [0..0] of SInt8;
	end;
type
	MatteAtomPtr = ^MatteAtom;
	MatteAtom = record
		size: SInt32;
		atomType: SInt32;

		aCompressedMatte: MatteCompressedAtom;
	end;
type
	ClippingAtomPtr = ^ClippingAtom;
	ClippingAtom = record
		size: SInt32;
		atomType: SInt32;

		aRgnClip: RgnAtom;
	end;
{**********************
* Media Info Example Structures
**********************}

type
	VideoMediaInfoHeaderPtr = ^VideoMediaInfoHeader;
	VideoMediaInfoHeader = record
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }

		graphicsMode: SInt16;           { for QD - transfer mode }
		opColorRed: SInt16;             { opcolor for transfer mode }
		opColorGreen: SInt16;
		opColorBlue: SInt16;
	end;
type
	VideoMediaInfoHeaderAtomPtr = ^VideoMediaInfoHeaderAtom;
	VideoMediaInfoHeaderAtom = record
		size: SInt32;                   { size of Media info }
		atomType: SInt32;               { = 'vmhd' }
		vmiHeader: VideoMediaInfoHeader;
	end;
type
	VideoMediaInfoPtr = ^VideoMediaInfo;
	VideoMediaInfo = record
		size: SInt32;                   { size of Media info }
		atomType: SInt32;               { = 'minf' }

		header: VideoMediaInfoHeaderAtom;

		dataHandler: HandlerAtom;

		dataInfo: DataInfoAtom;

		sampleTable: SampleTableAtom;
	end;
type
	SoundMediaInfoHeaderPtr = ^SoundMediaInfoHeader;
	SoundMediaInfoHeader = record
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }

		balance: SInt16;
		rsrvd: SInt16;
	end;
type
	SoundMediaInfoHeaderAtomPtr = ^SoundMediaInfoHeaderAtom;
	SoundMediaInfoHeaderAtom = record
		size: SInt32;                   { size of Media info }
		atomType: SInt32;               { = 'vmhd' }

		smiHeader: SoundMediaInfoHeader;
	end;
type
	SoundMediaInfoPtr = ^SoundMediaInfo;
	SoundMediaInfo = record
		size: SInt32;                   { size of Media info }
		atomType: SInt32;               { = 'minf' }

		header: SoundMediaInfoHeaderAtom;

		dataHandler: HandlerAtom;

		dataReference: DataRefAtom;

		sampleTable: SampleTableAtom;
	end;
{ whatever data the media handler needs goes after the atomType }
type
	MediaInfoPtr = ^MediaInfo;
	MediaInfo = record
		size: SInt32;
		atomType: SInt32;
	end;
	MediaInfo_fix = MediaInfo;
{**********************
* Media Directory Structures
**********************}
type
	MediaHeaderPtr = ^MediaHeader;
	MediaHeader = record
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }

		creationTime: SInt32;           { seconds since Jan 1904 when directory was created }
		modificationTime: SInt32;       { seconds since Jan 1904 when directory was appended }

		timeScale: TimeValue;              { start time for Media (Media time) }
		duration: TimeValue;               { length of Media (Media time) }

		language: SInt16;
		quality: SInt16;
	end;
type
	MediaHeaderAtomPtr = ^MediaHeaderAtom;
	MediaHeaderAtom = record
		size: SInt32;
		atomType: SInt32;

		header: MediaHeader;
	end;
type
	MediaDirectoryPtr = ^MediaDirectory;
	MediaDirectory = record
		size: SInt32;
		atomType: SInt32;               { = 'mdia' }

		mediaHeader: MediaHeaderAtom;            { standard Media information }

		mediaHandler: HandlerAtom;

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
		mediaInfo: MediaInfo_fix;
	end;
{**********************
* Track Structures
**********************}
const
	TrackEnable = 1 shl 0;
	TrackInMovie = 1 shl 1;
	TrackInPreview = 1 shl 2;
	TrackInPoster = 1 shl 3;
>>>>>>> origin/fixes_2.4

type
	TrackHeaderPtr = ^TrackHeader;
	TrackHeader = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }
>>>>>>> origin/fixes_2.4

		creationTime: SInt32;           { seconds since Jan 1904 when directory was created }
		modificationTime: SInt32;       { seconds since Jan 1904 when directory was appended }

		trackID: SInt32;

		reserved1: SInt32;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======

		creationTime: SInt32;           { seconds since Jan 1904 when directory was created }
		modificationTime: SInt32;       { seconds since Jan 1904 when directory was appended }

		trackID: SInt32;

		reserved1: SInt32;

>>>>>>> graemeg/cpstrnew
		duration: TimeValue;               { length of track (track time) }

		reserved2: SInt32;
		reserved3: SInt32;

		layer: SInt16;
		alternateGroup: SInt16;

=======

		duration: TimeValue;               { length of track (track time) }

		reserved2: SInt32;
		reserved3: SInt32;

		layer: SInt16;
		alternateGroup: SInt16;

>>>>>>> graemeg/cpstrnew
=======

=======

>>>>>>> origin/cpstrnew
		duration: TimeValue;               { length of track (track time) }

		reserved2: SInt32;
		reserved3: SInt32;

		layer: SInt16;
		alternateGroup: SInt16;

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
		volume: SInt16;
		reserved4: SInt16;

		matrix: MatrixRecord;
		trackWidth: Fixed;
		trackHeight: Fixed;
	end;
type
	TrackHeaderAtomPtr = ^TrackHeaderAtom;
	TrackHeaderAtom = record
		size: SInt32;                   { size of track header }
		atomType: SInt32;               { = 'tkhd' }

		header: TrackHeader;
	end;
type
	EditListTypePtr = ^EditListType;
	EditListType = record
		trackDuration: TimeValue;
		mediaTime: TimeValue;
		mediaRate: Fixed;
	end;
type
	EditListAtomPtr = ^EditListAtom;
	EditListAtom = record
		size: SInt32;
		atomType: SInt32;               { = elst }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }

=======

		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }

>>>>>>> graemeg/cpstrnew
=======

		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }

>>>>>>> graemeg/cpstrnew
=======

		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }

>>>>>>> origin/cpstrnew
		numEntries: SInt32;
editListTable: array [0..0] of EditListType;
	end;
type
	EditsAtomPtr = ^EditsAtom;
	EditsAtom = record
		size: SInt32;
		atomType: SInt32;               { = edts }

		editList: EditListAtom;
	end;
type
	TrackLoadSettingsPtr = ^TrackLoadSettings;
	TrackLoadSettings = record
		preloadStartTime: TimeValue;
		preloadDuration: TimeValue;
		preloadFlags: SInt32;
		defaultHints: SInt32;
	end;
type
	TrackLoadSettingsAtomPtr = ^TrackLoadSettingsAtom;
	TrackLoadSettingsAtom = record
		size: SInt32;
		atomType: SInt32;               { = load }

		settings: TrackLoadSettings;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
	end;
type
	TrackCleanApertureDimensionsPtr = ^TrackCleanApertureDimensions;
	TrackCleanApertureDimensions = record
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }
		cleanApertureDimensions: FixedPoint;
	end;
type
<<<<<<< HEAD
=======
=======
>>>>>>> origin/cpstrnew
	end;
type
	TrackCleanApertureDimensionsPtr = ^TrackCleanApertureDimensions;
	TrackCleanApertureDimensions = record
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }
		cleanApertureDimensions: FixedPoint;
	end;
type
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	TrackCleanApertureDimensionsAtomPtr = ^TrackCleanApertureDimensionsAtom;
	TrackCleanApertureDimensionsAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'tapt' }
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/cpstrnew

		cleanApertureDimensions: TrackCleanApertureDimensions;
	end;
type
	TrackProductionApertureDimensionsPtr = ^TrackProductionApertureDimensions;
	TrackProductionApertureDimensions = record
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }
		productionApertureDimensions: FixedPoint;
	end;
type
	TrackProductionApertureDimensionsAtomPtr = ^TrackProductionApertureDimensionsAtom;
	TrackProductionApertureDimensionsAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'prof' }
<<<<<<< HEAD

=======

		cleanApertureDimensions: TrackCleanApertureDimensions;
	end;
type
	TrackProductionApertureDimensionsPtr = ^TrackProductionApertureDimensions;
	TrackProductionApertureDimensions = record
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }
		productionApertureDimensions: FixedPoint;
	end;
type
=======
	end;
type
	TrackCleanApertureDimensionsPtr = ^TrackCleanApertureDimensions;
	TrackCleanApertureDimensions = record
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }
		cleanApertureDimensions: FixedPoint;
	end;
type
=======
>>>>>>> graemeg/cpstrnew
	TrackCleanApertureDimensionsAtomPtr = ^TrackCleanApertureDimensionsAtom;
	TrackCleanApertureDimensionsAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'tapt' }

		cleanApertureDimensions: TrackCleanApertureDimensions;
	end;
type
	TrackProductionApertureDimensionsPtr = ^TrackProductionApertureDimensions;
	TrackProductionApertureDimensions = record
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }
		productionApertureDimensions: FixedPoint;
	end;
type
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
	TrackProductionApertureDimensionsAtomPtr = ^TrackProductionApertureDimensionsAtom;
	TrackProductionApertureDimensionsAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'prof' }
<<<<<<< HEAD

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
		productionApertureDimensions: TrackProductionApertureDimensions;
	end;
type
	TrackEncodedPixelsDimensionsPtr = ^TrackEncodedPixelsDimensions;
	TrackEncodedPixelsDimensions = record
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }
		encodedPixelsDimensions: FixedPoint;
	end;
type
	TrackEncodedPixelsDimensionsAtomPtr = ^TrackEncodedPixelsDimensionsAtom;
	TrackEncodedPixelsDimensionsAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'enof' }

=======

=======

>>>>>>> origin/cpstrnew
		productionApertureDimensions: TrackProductionApertureDimensions;
	end;
type
	TrackEncodedPixelsDimensionsPtr = ^TrackEncodedPixelsDimensions;
	TrackEncodedPixelsDimensions = record
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }
		encodedPixelsDimensions: FixedPoint;
	end;
type
	TrackEncodedPixelsDimensionsAtomPtr = ^TrackEncodedPixelsDimensionsAtom;
	TrackEncodedPixelsDimensionsAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'enof' }

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
		encodedPixelsDimensions: TrackEncodedPixelsDimensions;
	end;
type
	TrackDirectoryPtr = ^TrackDirectory;
	TrackDirectory = record
		size: SInt32;
		atomType: SInt32;               { = 'trak' }

		trackHeader: TrackHeaderAtom;            { standard track information }

		trackClip: ClippingAtom;

		edits: EditsAtom;

		media: MediaDirectory;

		userData: UserDataAtom;               { space for extending with new data types }
	end;
	TrackDirectory_fix = TrackDirectory;
{***************************************
*
*   MovieDirectory -
*       The MovieDirectory is the top level structure which
*       holds the TrackInstance describing where the
*       TrackDirectories are.
*
***************************************}
type
	MovieHeaderPtr = ^MovieHeader;
	MovieHeader = record
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }
<<<<<<< HEAD
<<<<<<< HEAD

		creationTime: SInt32;           { seconds since Jan 1904 when directory was created }
		modificationTime: SInt32;       { seconds since Jan 1904 when directory was appended }
<<<<<<< HEAD
=======

		creationTime: SInt32;           { seconds since Jan 1904 when directory was created }
		modificationTime: SInt32;       { seconds since Jan 1904 when directory was appended }
>>>>>>> origin/cpstrnew

		timeScale: TimeValue;              { Time specifications }
		duration: TimeValue;
		preferredRate: Fixed;          { rate at which to play this movie }

		preferredVolume: SInt16;        { volume to play movie at }
		reserved1: SInt16;
<<<<<<< HEAD

		preferredLong1: SInt32;
		preferredLong2: SInt32;

		matrix: MatrixRecord;

		previewTime: TimeValue;            { time in track the proxy begins (track time) }
		previewDuration: TimeValue;        { how long the proxy lasts (track time) }
<<<<<<< HEAD

		posterTime: TimeValue;             { time in track the proxy begins (track time) }

=======

		posterTime: TimeValue;             { time in track the proxy begins (track time) }

>>>>>>> graemeg/cpstrnew
=======

		timeScale: TimeValue;              { Time specifications }
		duration: TimeValue;
		preferredRate: Fixed;          { rate at which to play this movie }

		preferredVolume: SInt16;        { volume to play movie at }
		reserved1: SInt16;

		preferredLong1: SInt32;
		preferredLong2: SInt32;

		matrix: MatrixRecord;

		previewTime: TimeValue;            { time in track the proxy begins (track time) }
		previewDuration: TimeValue;        { how long the proxy lasts (track time) }

		posterTime: TimeValue;             { time in track the proxy begins (track time) }

>>>>>>> graemeg/cpstrnew
=======

		creationTime: SInt32;           { seconds since Jan 1904 when directory was created }
		modificationTime: SInt32;       { seconds since Jan 1904 when directory was appended }

		timeScale: TimeValue;              { Time specifications }
		duration: TimeValue;
		preferredRate: Fixed;          { rate at which to play this movie }

		preferredVolume: SInt16;        { volume to play movie at }
		reserved1: SInt16;

		preferredLong1: SInt32;
		preferredLong2: SInt32;

		matrix: MatrixRecord;

		previewTime: TimeValue;            { time in track the proxy begins (track time) }
		previewDuration: TimeValue;        { how long the proxy lasts (track time) }

		posterTime: TimeValue;             { time in track the proxy begins (track time) }

>>>>>>> graemeg/cpstrnew
=======

		preferredLong1: SInt32;
		preferredLong2: SInt32;

		matrix: MatrixRecord;

		previewTime: TimeValue;            { time in track the proxy begins (track time) }
		previewDuration: TimeValue;        { how long the proxy lasts (track time) }

		posterTime: TimeValue;             { time in track the proxy begins (track time) }

>>>>>>> origin/cpstrnew
		selectionTime: TimeValue;          { time in track the proxy begins (track time) }
		selectionDuration: TimeValue;      { time in track the proxy begins (track time) }
		currentTime: TimeValue;            { time in track the proxy begins (track time) }

		nextTrackID: SInt32;            { next value to use for a TrackID }
	end;
type
	MovieHeaderAtomPtr = ^MovieHeaderAtom;
	MovieHeaderAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'mvhd' }

		header: MovieHeader;
	end;
type
	TrackDirectoryEntryPtr = ^TrackDirectoryEntry;
	TrackDirectoryEntry = record
		trackDirectory: TrackDirectory_fix;         { Track directory information }
	end;
type
	MovieDirectoryPtr = ^MovieDirectory;
	MovieDirectory = record
		size: SInt32;
		atomType: SInt32;               { = 'moov' }

		header: MovieHeaderAtom;

		movieClip: ClippingAtom;

                                              { Track Directories }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		track: array [0..0] of TrackDirectoryEntry;              { Track directory information }
=======
    track: array [0..0] of TrackDirectoryEntry;              { Track directory information }
>>>>>>> graemeg/cpstrnew
=======
    track: array [0..0] of TrackDirectoryEntry;              { Track directory information }
>>>>>>> graemeg/cpstrnew
=======
    track: array [0..0] of TrackDirectoryEntry;              { Track directory information }
>>>>>>> graemeg/cpstrnew
=======
    track: array [0..0] of TrackDirectoryEntry;              { Track directory information }
>>>>>>> origin/cpstrnew

                                              { User data for Movie }
		userData: UserDataAtom;               { space for user extensions }
	end;
{***************************************
***************************************}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{ Movie formats and tags }
const
{ some system defined format IDs }
	QT_MOVIE_TYPE = FourCharCode('moov');
	QT_TRACK_TYPE = FourCharCode('trak');
	QT_MEDIA_TYPE = FourCharCode('mdia');
	QT_VIDEO_TYPE = FourCharCode('vide');
	QT_SOUND_TYPE = FourCharCode('soun');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

const
	MOVIE_TYPE = FourCharCode('moov');
	TRACK_TYPE = FourCharCode('trak');
	VIDEO_TYPE = FourCharCode('vide');
	SOUND_TYPE = FourCharCode('soun');
<<<<<<< HEAD
<<<<<<< HEAD

=======

const
	MOVIE_TYPE = FourCharCode('moov');
	TRACK_TYPE = FourCharCode('trak');
	VIDEO_TYPE = FourCharCode('vide');
	SOUND_TYPE = FourCharCode('soun');

>>>>>>> graemeg/cpstrnew
=======

{ Movie formats and tags }
const
{ some system defined format IDs }
	QT_MOVIE_TYPE = FourCharCode('moov');
	QT_TRACK_TYPE = FourCharCode('trak');
	QT_MEDIA_TYPE = FourCharCode('mdia');
	QT_VIDEO_TYPE = FourCharCode('vide');
	QT_SOUND_TYPE = FourCharCode('soun');

const
	MOVIE_TYPE = FourCharCode('moov');
	TRACK_TYPE = FourCharCode('trak');
	VIDEO_TYPE = FourCharCode('vide');
	SOUND_TYPE = FourCharCode('soun');

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
{$ifc not TARGET_OS_WIN32}
{ The name "MEDIA_TYPE" has a name space collision on Win32.}
const
	MEDIA_TYPE = FourCharCode('mdia');

{$endc} {!TARGET_OS_WIN32}

{ atom id's }
const
=======

{$ifc not TARGET_OS_WIN32}
{ The name "MEDIA_TYPE" has a name space collision on Win32.}
const
	MEDIA_TYPE = FourCharCode('mdia');

{$endc} {!TARGET_OS_WIN32}

{ atom id's }
const
>>>>>>> origin/cpstrnew
	MovieAID = FourCharCode('moov');
	MovieHeaderAID = FourCharCode('mvhd');
	ClipAID = FourCharCode('clip');
	RgnClipAID = FourCharCode('crgn');
	MatteAID = FourCharCode('matt');
	MatteCompAID = FourCharCode('kmat');
	TrackAID = FourCharCode('trak');
	UserDataAID = FourCharCode('udta');
	TrackHeaderAID = FourCharCode('tkhd');
	EditsAID = FourCharCode('edts');
	EditListAID = FourCharCode('elst');
	MediaAID = FourCharCode('mdia');
	MediaHeaderAID = FourCharCode('mdhd');
	MediaInfoAID = FourCharCode('minf');
	VideoMediaInfoHeaderAID = FourCharCode('vmhd');
	SoundMediaInfoHeaderAID = FourCharCode('smhd');
	GenericMediaInfoHeaderAID = FourCharCode('gmhd');
	GenericMediaInfoAID = FourCharCode('gmin');
	DataInfoAID = FourCharCode('dinf');
	DataRefAID = FourCharCode('dref');
	SampleTableAID = FourCharCode('stbl');
	STSampleDescAID = FourCharCode('stsd');
	STTimeToSampAID = FourCharCode('stts');
	STSyncSampleAID = FourCharCode('stss');
	STSampleToChunkAID = FourCharCode('stsc');
	STShadowSyncAID = FourCharCode('stsh');
	HandlerAID = FourCharCode('hdlr');
	STSampleSizeAID = FourCharCode('stsz');
	STChunkOffsetAID = FourCharCode('stco');
	STChunkOffset64AID = FourCharCode('co64');
	STSampleIDAID = FourCharCode('stid');
	STCompositionOffsetAID = FourCharCode('ctts');
	STSampleDependencyAID = FourCharCode('sdtp');
	STCompositionShiftLeastGreatestAID = FourCharCode('cslg');
	STPartialSyncSampleAID = FourCharCode('stps');
	DataRefContainerAID = FourCharCode('drfc');
	TrackReferenceAID = FourCharCode('tref');
	ColorTableAID = FourCharCode('ctab');
	LoadSettingsAID = FourCharCode('load');
	PropertyAtomAID = FourCharCode('code');
	InputMapAID = FourCharCode('imap');
	MovieBufferHintsAID = FourCharCode('mbfh');
	MovieDataRefAliasAID = FourCharCode('mdra');
	SoundLocalizationAID = FourCharCode('sloc');
	CompressedMovieAID = FourCharCode('cmov');
	CompressedMovieDataAID = FourCharCode('cmvd');
	DataCompressionAtomAID = FourCharCode('dcom');
	ReferenceMovieRecordAID = FourCharCode('rmra');
	ReferenceMovieDescriptorAID = FourCharCode('rmda');
	ReferenceMovieDataRefAID = FourCharCode('rdrf');
	ReferenceMovieVersionCheckAID = FourCharCode('rmvc');
	ReferenceMovieDataRateAID = FourCharCode('rmdr');
	ReferenceMovieComponentCheckAID = FourCharCode('rmcd');
	ReferenceMovieQualityAID = FourCharCode('rmqu');
	ReferenceMovieLanguageAID = FourCharCode('rmla');
	ReferenceMovieCPURatingAID = FourCharCode('rmcs');
	ReferenceMovieAlternateGroupAID = FourCharCode('rmag');
	ReferenceMovieNetworkStatusAID = FourCharCode('rnet');
	CloneMediaAID = FourCharCode('clon');
	FileTypeAID = FourCharCode('ftyp');
	SecureContentInfoAID = FourCharCode('sinf');
	SecureContentSchemeTypeAID = FourCharCode('schm');
	SecureContentSchemeInfoAID = FourCharCode('schi');
	TrackApertureModeDimensionsAID = FourCharCode('tapt'); { container atom including TrackCleanApertureDimensionsAID, TrackProductionApertureDimensionsAID and TrackEncodedPixelsDimensionsAID }
	TrackCleanApertureDimensionsAID = FourCharCode('clef');
	TrackProductionApertureDimensionsAID = FourCharCode('prof');
	TrackEncodedPixelsDimensionsAID = FourCharCode('enof');

{ Text ATOM definitions}
=======
=======
>>>>>>> origin/fixes_2_2
		flags:					SInt32;								{  1 byte of version / 3 bytes of flags  }
		creationTime:			SInt32;								{  seconds since Jan 1904 when directory was created  }
		modificationTime:		SInt32;								{  seconds since Jan 1904 when directory was appended  }
		trackID:				SInt32;
		reserved1:				SInt32;
		duration:				TimeValue;								{  length of track (track time)  }
		reserved2:				SInt32;
		reserved3:				SInt32;
		layer:					SInt16;
		alternateGroup:			SInt16;
		volume:					SInt16;
		reserved4:				SInt16;
		matrix:					MatrixRecord;
		trackWidth:				Fixed;
		trackHeight:			Fixed;
	end;
=======
>>>>>>> origin/fixes_2.4

		duration: TimeValue;               { length of track (track time) }

		reserved2: SInt32;
		reserved3: SInt32;

		layer: SInt16;
		alternateGroup: SInt16;

		volume: SInt16;
		reserved4: SInt16;

		matrix: MatrixRecord;
		trackWidth: Fixed;
		trackHeight: Fixed;
	end;
type
	TrackHeaderAtomPtr = ^TrackHeaderAtom;
	TrackHeaderAtom = record
		size: SInt32;                   { size of track header }
		atomType: SInt32;               { = 'tkhd' }

		header: TrackHeader;
	end;
type
	EditListTypePtr = ^EditListType;
	EditListType = record
		trackDuration: TimeValue;
		mediaTime: TimeValue;
		mediaRate: Fixed;
	end;
type
	EditListAtomPtr = ^EditListAtom;
	EditListAtom = record
		size: SInt32;
		atomType: SInt32;               { = elst }

		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }

		numEntries: SInt32;
editListTable: array [0..0] of EditListType;
	end;
type
	EditsAtomPtr = ^EditsAtom;
	EditsAtom = record
		size: SInt32;
		atomType: SInt32;               { = edts }

		editList: EditListAtom;
	end;
type
	TrackLoadSettingsPtr = ^TrackLoadSettings;
	TrackLoadSettings = record
		preloadStartTime: TimeValue;
		preloadDuration: TimeValue;
		preloadFlags: SInt32;
		defaultHints: SInt32;
	end;
type
	TrackLoadSettingsAtomPtr = ^TrackLoadSettingsAtom;
	TrackLoadSettingsAtom = record
		size: SInt32;
		atomType: SInt32;               { = load }

		settings: TrackLoadSettings;
	end;
type
	TrackCleanApertureDimensionsPtr = ^TrackCleanApertureDimensions;
	TrackCleanApertureDimensions = record
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }
		cleanApertureDimensions: FixedPoint;
	end;
type
	TrackCleanApertureDimensionsAtomPtr = ^TrackCleanApertureDimensionsAtom;
	TrackCleanApertureDimensionsAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'tapt' }

		cleanApertureDimensions: TrackCleanApertureDimensions;
	end;
type
	TrackProductionApertureDimensionsPtr = ^TrackProductionApertureDimensions;
	TrackProductionApertureDimensions = record
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }
		productionApertureDimensions: FixedPoint;
	end;
type
	TrackProductionApertureDimensionsAtomPtr = ^TrackProductionApertureDimensionsAtom;
	TrackProductionApertureDimensionsAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'prof' }

		productionApertureDimensions: TrackProductionApertureDimensions;
	end;
type
	TrackEncodedPixelsDimensionsPtr = ^TrackEncodedPixelsDimensions;
	TrackEncodedPixelsDimensions = record
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }
		encodedPixelsDimensions: FixedPoint;
	end;
type
	TrackEncodedPixelsDimensionsAtomPtr = ^TrackEncodedPixelsDimensionsAtom;
	TrackEncodedPixelsDimensionsAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'enof' }

		encodedPixelsDimensions: TrackEncodedPixelsDimensions;
	end;
type
	TrackDirectoryPtr = ^TrackDirectory;
	TrackDirectory = record
		size: SInt32;
		atomType: SInt32;               { = 'trak' }

		trackHeader: TrackHeaderAtom;            { standard track information }

		trackClip: ClippingAtom;

		edits: EditsAtom;

		media: MediaDirectory;

		userData: UserDataAtom;               { space for extending with new data types }
	end;
	TrackDirectory_fix = TrackDirectory;
{***************************************
*
*   MovieDirectory -
*       The MovieDirectory is the top level structure which
*       holds the TrackInstance describing where the
*       TrackDirectories are.
*
***************************************}
type
	MovieHeaderPtr = ^MovieHeader;
	MovieHeader = record
		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }

		creationTime: SInt32;           { seconds since Jan 1904 when directory was created }
		modificationTime: SInt32;       { seconds since Jan 1904 when directory was appended }

		timeScale: TimeValue;              { Time specifications }
		duration: TimeValue;
		preferredRate: Fixed;          { rate at which to play this movie }

		preferredVolume: SInt16;        { volume to play movie at }
		reserved1: SInt16;

		preferredLong1: SInt32;
		preferredLong2: SInt32;

		matrix: MatrixRecord;

		previewTime: TimeValue;            { time in track the proxy begins (track time) }
		previewDuration: TimeValue;        { how long the proxy lasts (track time) }

		posterTime: TimeValue;             { time in track the proxy begins (track time) }

		selectionTime: TimeValue;          { time in track the proxy begins (track time) }
		selectionDuration: TimeValue;      { time in track the proxy begins (track time) }
		currentTime: TimeValue;            { time in track the proxy begins (track time) }

		nextTrackID: SInt32;            { next value to use for a TrackID }
	end;
type
	MovieHeaderAtomPtr = ^MovieHeaderAtom;
	MovieHeaderAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'mvhd' }

		header: MovieHeader;
	end;
type
	TrackDirectoryEntryPtr = ^TrackDirectoryEntry;
	TrackDirectoryEntry = record
		trackDirectory: TrackDirectory_fix;         { Track directory information }
	end;
type
	MovieDirectoryPtr = ^MovieDirectory;
	MovieDirectory = record
		size: SInt32;
		atomType: SInt32;               { = 'moov' }

		header: MovieHeaderAtom;

		movieClip: ClippingAtom;

                                              { Track Directories }
    track: array [0..0] of TrackDirectoryEntry;              { Track directory information }

                                              { User data for Movie }
		userData: UserDataAtom;               { space for user extensions }
	end;
{***************************************
***************************************}

{ Movie formats and tags }
const
{ some system defined format IDs }
	QT_MOVIE_TYPE = FourCharCode('moov');
	QT_TRACK_TYPE = FourCharCode('trak');
	QT_MEDIA_TYPE = FourCharCode('mdia');
	QT_VIDEO_TYPE = FourCharCode('vide');
	QT_SOUND_TYPE = FourCharCode('soun');

const
	MOVIE_TYPE = FourCharCode('moov');
	TRACK_TYPE = FourCharCode('trak');
	VIDEO_TYPE = FourCharCode('vide');
	SOUND_TYPE = FourCharCode('soun');

{$ifc not TARGET_OS_WIN32}
{ The name "MEDIA_TYPE" has a name space collision on Win32.}
const
	MEDIA_TYPE = FourCharCode('mdia');

{$endc} {!TARGET_OS_WIN32}

{ atom id's }
const
	MovieAID = FourCharCode('moov');
	MovieHeaderAID = FourCharCode('mvhd');
	ClipAID = FourCharCode('clip');
	RgnClipAID = FourCharCode('crgn');
	MatteAID = FourCharCode('matt');
	MatteCompAID = FourCharCode('kmat');
	TrackAID = FourCharCode('trak');
	UserDataAID = FourCharCode('udta');
	TrackHeaderAID = FourCharCode('tkhd');
	EditsAID = FourCharCode('edts');
	EditListAID = FourCharCode('elst');
	MediaAID = FourCharCode('mdia');
	MediaHeaderAID = FourCharCode('mdhd');
	MediaInfoAID = FourCharCode('minf');
	VideoMediaInfoHeaderAID = FourCharCode('vmhd');
	SoundMediaInfoHeaderAID = FourCharCode('smhd');
	GenericMediaInfoHeaderAID = FourCharCode('gmhd');
	GenericMediaInfoAID = FourCharCode('gmin');
	DataInfoAID = FourCharCode('dinf');
	DataRefAID = FourCharCode('dref');
	SampleTableAID = FourCharCode('stbl');
	STSampleDescAID = FourCharCode('stsd');
	STTimeToSampAID = FourCharCode('stts');
	STSyncSampleAID = FourCharCode('stss');
	STSampleToChunkAID = FourCharCode('stsc');
	STShadowSyncAID = FourCharCode('stsh');
	HandlerAID = FourCharCode('hdlr');
	STSampleSizeAID = FourCharCode('stsz');
	STChunkOffsetAID = FourCharCode('stco');
	STChunkOffset64AID = FourCharCode('co64');
	STSampleIDAID = FourCharCode('stid');
	STCompositionOffsetAID = FourCharCode('ctts');
	STSampleDependencyAID = FourCharCode('sdtp');
	STCompositionShiftLeastGreatestAID = FourCharCode('cslg');
	STPartialSyncSampleAID = FourCharCode('stps');
	DataRefContainerAID = FourCharCode('drfc');
	TrackReferenceAID = FourCharCode('tref');
	ColorTableAID = FourCharCode('ctab');
	LoadSettingsAID = FourCharCode('load');
	PropertyAtomAID = FourCharCode('code');
	InputMapAID = FourCharCode('imap');
	MovieBufferHintsAID = FourCharCode('mbfh');
	MovieDataRefAliasAID = FourCharCode('mdra');
	SoundLocalizationAID = FourCharCode('sloc');
	CompressedMovieAID = FourCharCode('cmov');
	CompressedMovieDataAID = FourCharCode('cmvd');
	DataCompressionAtomAID = FourCharCode('dcom');
	ReferenceMovieRecordAID = FourCharCode('rmra');
	ReferenceMovieDescriptorAID = FourCharCode('rmda');
	ReferenceMovieDataRefAID = FourCharCode('rdrf');
	ReferenceMovieVersionCheckAID = FourCharCode('rmvc');
	ReferenceMovieDataRateAID = FourCharCode('rmdr');
	ReferenceMovieComponentCheckAID = FourCharCode('rmcd');
	ReferenceMovieQualityAID = FourCharCode('rmqu');
	ReferenceMovieLanguageAID = FourCharCode('rmla');
	ReferenceMovieCPURatingAID = FourCharCode('rmcs');
	ReferenceMovieAlternateGroupAID = FourCharCode('rmag');
	ReferenceMovieNetworkStatusAID = FourCharCode('rnet');
<<<<<<< HEAD
	CloneMediaAID				= FourCharCode('clon');
	FileTypeAID					= FourCharCode('ftyp');
	SecureContentInfoAID		= FourCharCode('sinf');
	SecureContentSchemeTypeAID	= FourCharCode('schm');
	SecureContentSchemeInfoAID	= FourCharCode('schi');

	{  Text ATOM definitions }

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	CloneMediaAID = FourCharCode('clon');
	FileTypeAID = FourCharCode('ftyp');
	SecureContentInfoAID = FourCharCode('sinf');
	SecureContentSchemeTypeAID = FourCharCode('schm');
	SecureContentSchemeInfoAID = FourCharCode('schi');
	TrackApertureModeDimensionsAID = FourCharCode('tapt'); { container atom including TrackCleanApertureDimensionsAID, TrackProductionApertureDimensionsAID and TrackEncodedPixelsDimensionsAID }
	TrackCleanApertureDimensionsAID = FourCharCode('clef');
	TrackProductionApertureDimensionsAID = FourCharCode('prof');
	TrackEncodedPixelsDimensionsAID = FourCharCode('enof');

{ Text ATOM definitions}
>>>>>>> origin/fixes_2.4

type
	TextBoxAtomPtr = ^TextBoxAtom;
	TextBoxAtom = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		size: SInt32;
		atomType: SInt32;               { = 'tbox' }
		textBox: Rect;                { New text box (overrides defaultTextBox)}
	end;
type
	HiliteAtomPtr = ^HiliteAtom;
	HiliteAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'hlit' }
		selStart: SInt32;               { hilite selection start character}
		selEnd: SInt32;                 { hilite selection end character}
	end;
type
	KaraokeRecPtr = ^KaraokeRec;
	KaraokeRec = record
		timeVal: TimeValue;
		beginHilite: SInt16;
		endHilite: SInt16;
	end;
type
	KaraokeAtomPtr = ^KaraokeAtom;
	KaraokeAtom = record
		numEntries: SInt32;
		karaokeEntries: array [0..0] of KaraokeRec;
	end;
{ for ReferenceMovieDataRefRecord.flags}
const
	kDataRefIsSelfContained = 1 shl 0;
=======
=======
>>>>>>> origin/fixes_2_2
		size:					SInt32;
		atomType:				SInt32;								{  = 'tbox'  }
		textBox:				Rect;									{  New text box (overrides defaultTextBox) }
=======
		size: SInt32;
		atomType: SInt32;               { = 'tbox' }
		textBox: Rect;                { New text box (overrides defaultTextBox)}
>>>>>>> origin/fixes_2.4
	end;
type
	HiliteAtomPtr = ^HiliteAtom;
	HiliteAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'hlit' }
		selStart: SInt32;               { hilite selection start character}
		selEnd: SInt32;                 { hilite selection end character}
	end;
type
	KaraokeRecPtr = ^KaraokeRec;
	KaraokeRec = record
		timeVal: TimeValue;
		beginHilite: SInt16;
		endHilite: SInt16;
	end;
type
	KaraokeAtomPtr = ^KaraokeAtom;
	KaraokeAtom = record
		numEntries: SInt32;
		karaokeEntries: array [0..0] of KaraokeRec;
	end;
{ for ReferenceMovieDataRefRecord.flags}
const
<<<<<<< HEAD
	kDataRefIsSelfContained		= $01;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kDataRefIsSelfContained = 1 shl 0;
>>>>>>> origin/fixes_2.4

type
	ReferenceMovieDataRefRecordPtr = ^ReferenceMovieDataRefRecord;
	ReferenceMovieDataRefRecord = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		flags: SInt32;
		dataRefType: OSType;
		dataRefSize: SInt32;
		dataRef: array [0..0] of SInt8;
<<<<<<< HEAD
	end;
{ for VersionCheckRecord.checkType}
const
	kVersionCheckMin = 0;    { val1 is the min. version required}
	kVersionCheckMask = 1;     { (gestalt return value & val2) must == val1}
=======
=======
>>>>>>> origin/fixes_2_2
		flags:					SInt32;
		dataRefType:			OSType;
		dataRefSize:			SInt32;
		dataRef:				SInt8;
=======
>>>>>>> origin/fixes_2.4
	end;
{ for VersionCheckRecord.checkType}
const
<<<<<<< HEAD
	kVersionCheckMin			= 0;							{  val1 is the min. version required }
	kVersionCheckMask			= 1;							{  (gestalt return value & val2) must == val1 }

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kVersionCheckMin = 0;    { val1 is the min. version required}
	kVersionCheckMask = 1;     { (gestalt return value & val2) must == val1}
>>>>>>> origin/fixes_2.4

type
	QTAltVersionCheckRecordPtr = ^QTAltVersionCheckRecord;
	QTAltVersionCheckRecord = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
		flags: SInt32;                  { currently always 0}
		gestaltTag: OSType;
		val1: UInt32;
		val2: UInt32;
		checkType: SInt16;
<<<<<<< HEAD
	end;
{ some helpful constants for DataRateRecord.dataRate }
const
	kDataRate144ModemRate = 1400;
	kDataRate288ModemRate = 2800;
	kDataRateISDNRate = 5600;
	kDataRateDualISDNRate = 11200;
	kDataRate256kbpsRate = 25600;
	kDataRate384kbpsRate = 38400;
	kDataRate512kbpsRate = 51200;
	kDataRate768kbpsRate = 76800;
	kDataRate1MbpsRate = 100000;
	kDataRateT1Rate = 150000;
	kDataRateInfiniteRate = $7FFFFFFF;
	kDataRateDefaultIfNotSet = kDataRate512kbpsRate;
=======
=======
>>>>>>> origin/fixes_2_2
		flags:					SInt32;								{  currently always 0 }
		gestaltTag:				OSType;
		val1:					UInt32;
		val2:					UInt32;
		checkType:				SInt16;
=======
>>>>>>> origin/fixes_2.4
	end;
{ some helpful constants for DataRateRecord.dataRate }
const
<<<<<<< HEAD
	kDataRate144ModemRate		= 1400;
	kDataRate288ModemRate		= 2800;
	kDataRateISDNRate			= 5600;
	kDataRateDualISDNRate		= 11200;
	kDataRate256kbpsRate		= 25600;
	kDataRate384kbpsRate		= 38400;
	kDataRate512kbpsRate		= 51200;
	kDataRate768kbpsRate		= 76800;
	kDataRate1MbpsRate			= 100000;
	kDataRateT1Rate				= 150000;
	kDataRateInfiniteRate		= $7FFFFFFF;
	kDataRateDefaultIfNotSet	= 5600;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kDataRate144ModemRate = 1400;
	kDataRate288ModemRate = 2800;
	kDataRateISDNRate = 5600;
	kDataRateDualISDNRate = 11200;
	kDataRate256kbpsRate = 25600;
	kDataRate384kbpsRate = 38400;
	kDataRate512kbpsRate = 51200;
	kDataRate768kbpsRate = 76800;
	kDataRate1MbpsRate = 100000;
	kDataRateT1Rate = 150000;
	kDataRateInfiniteRate = $7FFFFFFF;
	kDataRateDefaultIfNotSet = kDataRate512kbpsRate;
>>>>>>> origin/fixes_2.4

type
	QTAltDataRateRecordPtr = ^QTAltDataRateRecord;
	QTAltDataRateRecord = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		flags: SInt32;                  { currently always 0}
		dataRate: SInt32;
	end;
type
	QTAltComponentCheckRecordPtr = ^QTAltComponentCheckRecord;
	QTAltComponentCheckRecord = record
		flags: SInt32;                  { currently always 0 }
		cd: ComponentDescription;
		minVersion: UInt32;
	end;
type
	QTAltLanguageRecordPtr = ^QTAltLanguageRecord;
	QTAltLanguageRecord = record
		flags: SInt32;                  { currently always 0}
		language: SInt16;
	end;

const
	kQTCPUSpeed1Rating = 100;  { slowest}
	kQTCPUSpeed2Rating = 200;
	kQTCPUSpeed3Rating = 300;
	kQTCPUSpeed4Rating = 400;
	kQTCPUSpeed5Rating = 500;   { fastest}
=======
=======
>>>>>>> origin/fixes_2_2
		flags:					SInt32;								{  currently always 0 }
		dataRate:				SInt32;
=======
		flags: SInt32;                  { currently always 0}
		dataRate: SInt32;
>>>>>>> origin/fixes_2.4
	end;
type
	QTAltComponentCheckRecordPtr = ^QTAltComponentCheckRecord;
	QTAltComponentCheckRecord = record
		flags: SInt32;                  { currently always 0 }
		cd: ComponentDescription;
		minVersion: UInt32;
	end;
type
	QTAltLanguageRecordPtr = ^QTAltLanguageRecord;
	QTAltLanguageRecord = record
		flags: SInt32;                  { currently always 0}
		language: SInt16;
	end;

const
<<<<<<< HEAD
	kQTCPUSpeed1Rating			= 100;							{  slowest }
	kQTCPUSpeed2Rating			= 200;
	kQTCPUSpeed3Rating			= 300;
	kQTCPUSpeed4Rating			= 400;
	kQTCPUSpeed5Rating			= 500;							{  fastest }

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kQTCPUSpeed1Rating = 100;  { slowest}
	kQTCPUSpeed2Rating = 200;
	kQTCPUSpeed3Rating = 300;
	kQTCPUSpeed4Rating = 400;
	kQTCPUSpeed5Rating = 500;   { fastest}
>>>>>>> origin/fixes_2.4

type
	QTAltCPURatingRecordPtr = ^QTAltCPURatingRecord;
	QTAltCPURatingRecord = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		flags: UInt32;                  { currently always 0}
		speed: UInt16;
	end;
type
	ReferenceMovieNetworkStatusRecordPtr = ^ReferenceMovieNetworkStatusRecord;
	ReferenceMovieNetworkStatusRecord = record
		flags: UInt32;                  { currently always 0}
		valueCount: UInt32;             { how many status values are in array}
		netStatusValues: array [0..0] of SInt32;     { a value from kQTNetworkStatus... constants}
	end;
type
	CloneRecordPtr = ^CloneRecord;
	CloneRecord = record
		flags: SInt32;
		masterTrackID: SInt32;          { track ID of the track we're cloning }
	end;
type
	CloneAtomPtr = ^CloneAtom;
	CloneAtom = record
		size: SInt32;
		atomType: SInt32;               { = clon }

		cloneInfo: CloneRecord;
	end;
type
	FileTypeAtomPtr = ^FileTypeAtom;
	FileTypeAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'ftyp' }
		majorBrand: SInt32;             { best use brand }
		minorVersion: SInt32;
		compatibleBrands: array [0..3] of SInt32;    { 1 or greater }
	end;
const
	kQTFileTypeBrandQuickTimeMovie = FourCharCode('qt  '); { QuickTime movie files}
	kQTFileTypeBrandISOFile = FourCharCode('isom'); { ISO Base Media files}
	kQTFileTypeBrandMPEG4v1 = FourCharCode('mp41'); { MPEG-4 (ISO/IEC 14496-1) version 1 files}
	kQTFileTypeBrandMPEG4v2 = FourCharCode('mp42'); { MPEG-4 (ISO/IEC 14496-1) version 2 files}
=======
=======
>>>>>>> origin/fixes_2_2
		flags:					UInt32;									{  currently always 0 }
		speed:					UInt16;
=======
		flags: UInt32;                  { currently always 0}
		speed: UInt16;
>>>>>>> origin/fixes_2.4
	end;
type
	ReferenceMovieNetworkStatusRecordPtr = ^ReferenceMovieNetworkStatusRecord;
	ReferenceMovieNetworkStatusRecord = record
		flags: UInt32;                  { currently always 0}
		valueCount: UInt32;             { how many status values are in array}
		netStatusValues: array [0..0] of SInt32;     { a value from kQTNetworkStatus... constants}
	end;
type
	CloneRecordPtr = ^CloneRecord;
	CloneRecord = record
		flags: SInt32;
		masterTrackID: SInt32;          { track ID of the track we're cloning }
	end;
type
	CloneAtomPtr = ^CloneAtom;
	CloneAtom = record
		size: SInt32;
		atomType: SInt32;               { = clon }

		cloneInfo: CloneRecord;
	end;
type
	FileTypeAtomPtr = ^FileTypeAtom;
	FileTypeAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'ftyp' }
		majorBrand: SInt32;             { best use brand }
		minorVersion: SInt32;
		compatibleBrands: array [0..3] of SInt32;    { 1 or greater }
	end;
const
<<<<<<< HEAD
	kQTFileTypeBrandQuickTimeMovie = FourCharCode('qt  ');					{  QuickTime movie files }
	kQTFileTypeBrandISOFile		= FourCharCode('isom');						{  ISO Base Media files }
	kQTFileTypeBrandMPEG4v1		= FourCharCode('mp41');						{  MPEG-4 (ISO/IEC 14496-1) version 1 files }
	kQTFileTypeBrandMPEG4v2		= FourCharCode('mp42');						{  MPEG-4 (ISO/IEC 14496-1) version 2 files }

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kQTFileTypeBrandQuickTimeMovie = FourCharCode('qt  '); { QuickTime movie files}
	kQTFileTypeBrandISOFile = FourCharCode('isom'); { ISO Base Media files}
	kQTFileTypeBrandMPEG4v1 = FourCharCode('mp41'); { MPEG-4 (ISO/IEC 14496-1) version 1 files}
	kQTFileTypeBrandMPEG4v2 = FourCharCode('mp42'); { MPEG-4 (ISO/IEC 14496-1) version 2 files}
>>>>>>> origin/fixes_2.4

type
	SecureContentInfoAtomPtr = ^SecureContentInfoAtom;
	SecureContentInfoAtom = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		size: SInt32;
		atomType: SInt32;               { = 'sinf' }
	end;
type
	SecureContentSchemeTypeAtomPtr = ^SecureContentSchemeTypeAtom;
	SecureContentSchemeTypeAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'schm' }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }

=======

		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }

>>>>>>> graemeg/cpstrnew
=======

		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }

>>>>>>> graemeg/cpstrnew
=======

		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }

>>>>>>> origin/cpstrnew
		schemeType: SInt32;
		schemeVersion: UInt32;
                                              { if flags & 1, C string holding URL for security component server}
	end;
type
	SecureContentSchemeInfoAtomPtr = ^SecureContentSchemeInfoAtom;
	SecureContentSchemeInfoAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'schi' }
	end;

{$endc} {TARGET_OS_MAC}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
		size:					SInt32;
		atomType:				SInt32;								{  = 'sinf'  }
=======
		size: SInt32;
		atomType: SInt32;               { = 'sinf' }
>>>>>>> origin/fixes_2.4
	end;
type
	SecureContentSchemeTypeAtomPtr = ^SecureContentSchemeTypeAtom;
	SecureContentSchemeTypeAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'schm' }

		flags: SInt32;                  { 1 byte of version / 3 bytes of flags }

		schemeType: SInt32;
		schemeVersion: UInt32;
                                              { if flags & 1, C string holding URL for security component server}
	end;
type
	SecureContentSchemeInfoAtomPtr = ^SecureContentSchemeInfoAtom;
	SecureContentSchemeInfoAtom = record
		size: SInt32;
		atomType: SInt32;               { = 'schi' }
	end;

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
