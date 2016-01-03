{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       QD/FontSync.h
 
     Contains:   Public interface for FontSync
 
     Version:    Quickdraw-262~1
 
     Copyright:  © 1999-2008 by Apple Inc. all rights reserved.
=======
=======
>>>>>>> origin/fixes_2_2
     File:       FontSync.p
=======
     File:       QD/FontSync.h
>>>>>>> origin/fixes_2.4
 
     Contains:   Public interface for FontSync
 
     Version:    Quickdraw-262~1
 
<<<<<<< HEAD
     Copyright:  © 1999-2002 by Apple Computer, Inc., all rights reserved.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  © 1999-2008 by Apple Inc. all rights reserved.
>>>>>>> origin/fixes_2.4
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}
{   Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit FontSync;
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
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
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
{$elsec}
=======
=======
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
{$elsec}
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
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
>>>>>>> graemeg/cpstrnew
=======
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
uses MacTypes,ATSTypes,Files,Fonts,SFNTTypes,MacErrors;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not MACOSALLINCLUDE}
<<<<<<< HEAD
=======

>>>>>>> origin/cpstrnew

{$ifc TARGET_OS_MAC}

{$ifc TARGET_OS_MAC}
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
=======
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}
>>>>>>> origin/fixes_2.4

{$ALIGN POWER}


{ Matching Options }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
	FNSMatchOptions = UInt32;
	FNSMatchOptionsPtr = ^FNSMatchOptions; { when a VAR xx: FNSMatchOptions parameter can be nil, it is changed to xx: FNSMatchOptionsPtr }
const
	kFNSMatchNames = $00000001; { font names must match }
	kFNSMatchTechnology = $00000002; { scaler technology must match }
	kFNSMatchGlyphs = $00000004; { glyph data must match }
	kFNSMatchEncodings = $00000008; { cmaps must match }
	kFNSMatchQDMetrics = $00000010; { QuickDraw Text metrics must match }
	kFNSMatchATSUMetrics = $00000020; { ATSUI metrics (incl. vertical) must match }
	kFNSMatchKerning = $00000040; { kerning data must match }
	kFNSMatchWSLayout = $00000080; { WorldScript layout tables must match }
	kFNSMatchAATLayout = $00000100; { AAT (incl. OpenType) layout tables must match }
	kFNSMatchPrintEncoding = $00000200; { PostScript font and glyph names and re-encoding vector must match }
	kFNSMissingDataNoMatch = $80000000; { treat missing data as mismatch }
	kFNSMatchAll = $FFFFFFFF; { everything must match }
	kFNSMatchDefaults = 0;     { use global default match options }

{$ifc not TARGET_CPU_64}
{
 *  FNSMatchDefaultsGet()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSMatchDefaultsGet: FNSMatchOptions; external name '_FNSMatchDefaultsGet';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ Version control }
{$endc} {not TARGET_CPU_64}

type
	FNSObjectVersion = UInt32;
const
	kFNSVersionDontCare = 0;
	kFNSCurSysInfoVersion = 1;

{ No features defined yet.}
type
	FNSFeatureFlags = UInt32;
{
   The FontSync library version number is binary-coded decimal:
   8 bits of major version, 4 minor version and 4 bits revision.
}
type
	FNSSysInfoPtr = ^FNSSysInfo;
	FNSSysInfo = record
		iSysInfoVersion: FNSObjectVersion;        { fill this in before calling FNSSysInfoGet}
		oFeatures: FNSFeatureFlags;
		oCurRefVersion: FNSObjectVersion;
		oMinRefVersion: FNSObjectVersion;
		oCurProfileVersion: FNSObjectVersion;
		oMinProfileVersion: FNSObjectVersion;
		oFontSyncVersion: UInt16;
	end;
{$ifc not TARGET_CPU_64}
{
 *  FNSSysInfoGet()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
procedure FNSSysInfoGet( var ioInfo: FNSSysInfo ); external name '_FNSSysInfoGet';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ FontSync References }
{$endc} {not TARGET_CPU_64}

type
	FNSFontReference = ^SInt32; { an opaque type }
	FNSFontReferencePtr = ^FNSFontReference;  { when a var xx:FNSFontReference parameter can be nil, it is changed to xx: FNSFontReferencePtr }
{$ifc not TARGET_CPU_64}
{
 *  FNSReferenceGetVersion()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSReferenceGetVersion( iReference: FNSFontReference; var oVersion: FNSObjectVersion ): OSStatus; external name '_FNSReferenceGetVersion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSReferenceDispose()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSReferenceDispose( iReference: FNSFontReference ): OSStatus; external name '_FNSReferenceDispose';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSReferenceMatch()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSReferenceMatch( iReference1: FNSFontReference; iReference2: FNSFontReference; iOptions: FNSMatchOptions; oFailedMatchOptions: FNSMatchOptionsPtr { can be NULL } ): OSStatus; external name '_FNSReferenceMatch';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSReferenceFlattenedSize()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSReferenceFlattenedSize( iReference: FNSFontReference; var oFlattenedSize: ByteCount ): OSStatus; external name '_FNSReferenceFlattenedSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSReferenceFlatten()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSReferenceFlatten( iReference: FNSFontReference; oFlatReference: UnivPtr { can be NULL }; oFlattenedSize: ByteCountPtr { can be NULL } ): OSStatus; external name '_FNSReferenceFlatten';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSReferenceUnflatten()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSReferenceUnflatten( iFlatReference: {const} UnivPtr; iFlattenedSize: ByteCount; var oReference: FNSFontReference ): OSStatus; external name '_FNSReferenceUnflatten';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ FontSync Profiles }
{$endc} {not TARGET_CPU_64}

const
	kFNSCreatorDefault = 0;
	kFNSProfileFileType = FourCharCode('fnsp');

type
	FNSFontProfile = ^SInt32; { an opaque type }
	FNSFontProfilePtr = ^FNSFontProfile;  { when a var xx:FNSFontProfile parameter can be nil, it is changed to xx: FNSFontProfilePtr }
{$ifc not TARGET_CPU_64}
{
 *  FNSProfileCreate()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSProfileCreate( const (*var*) iFile: FSSpec; iCreator: FourCharCode; iEstNumRefs: ItemCount; iDesiredVersion: FNSObjectVersion; var oProfile: FNSFontProfile ): OSStatus; external name '_FNSProfileCreate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSProfileOpen()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSProfileOpen( const (*var*) iFile: FSSpec; iOpenForWrite: Boolean; var oProfile: FNSFontProfile ): OSStatus; external name '_FNSProfileOpen';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSProfileCreateWithFSRef()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   not available
 }
function FNSProfileCreateWithFSRef( const (*var*) iParentDirectory: FSRef; iNameLength: UniCharCount; iName: ConstUniCharPtr; iCreator: FourCharCode; iEstNumRefs: ItemCount; iDesiredVersion: FNSObjectVersion; var oProfile: FNSFontProfile ): OSStatus; external name '_FNSProfileCreateWithFSRef';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSProfileOpenWithFSRef()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   not available
 }
function FNSProfileOpenWithFSRef( const (*var*) iFile: FSRef; iOpenForWrite: Boolean; var oProfile: FNSFontProfile ): OSStatus; external name '_FNSProfileOpenWithFSRef';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSProfileGetVersion()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSProfileGetVersion( iProfile: FNSFontProfile; var oVersion: FNSObjectVersion ): OSStatus; external name '_FNSProfileGetVersion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSProfileCompact()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in  in ApplicationServices.framework [32-bit only] but deprecated in LeopardX
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSProfileCompact( iProfile: FNSFontProfile ): OSStatus; external name '_FNSProfileCompact';


{
 *  FNSProfileClose()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSProfileClose( iProfile: FNSFontProfile ): OSStatus; external name '_FNSProfileClose';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSProfileAddReference()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSProfileAddReference( iProfile: FNSFontProfile; iReference: FNSFontReference ): OSStatus; external name '_FNSProfileAddReference';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSProfileRemoveReference()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSProfileRemoveReference( iProfile: FNSFontProfile; iReference: FNSFontReference ): OSStatus; external name '_FNSProfileRemoveReference';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSProfileRemoveIndReference()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSProfileRemoveIndReference( iProfile: FNSFontProfile; iIndex: UInt32 ): OSStatus; external name '_FNSProfileRemoveIndReference';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSProfileClear()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSProfileClear( iProfile: FNSFontProfile ): OSStatus; external name '_FNSProfileClear';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSProfileCountReferences()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSProfileCountReferences( iProfile: FNSFontProfile; var oCount: ItemCount ): OSStatus; external name '_FNSProfileCountReferences';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSProfileGetIndReference()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSProfileGetIndReference( iProfile: FNSFontProfile; iWhichReference: UInt32; var oReference: FNSFontReference ): OSStatus; external name '_FNSProfileGetIndReference';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSProfileMatchReference()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSProfileMatchReference( iProfile: FNSFontProfile; iReference: FNSFontReference; iMatchOptions: FNSMatchOptions; iOutputSize: ItemCount; oIndices: {variable-size-array} UInt32Ptr { can be NULL }; oNumMatches: ItemCountPtr { can be NULL } ): OSStatus; external name '_FNSProfileMatchReference';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
=======
=======
>>>>>>> origin/fixes_2_2

=======
>>>>>>> origin/fixes_2.4
type
	FNSMatchOptions = UInt32;
	FNSMatchOptionsPtr = ^FNSMatchOptions; { when a VAR xx: FNSMatchOptions parameter can be nil, it is changed to xx: FNSMatchOptionsPtr }
const
	kFNSMatchNames = $00000001; { font names must match }
	kFNSMatchTechnology = $00000002; { scaler technology must match }
	kFNSMatchGlyphs = $00000004; { glyph data must match }
	kFNSMatchEncodings = $00000008; { cmaps must match }
	kFNSMatchQDMetrics = $00000010; { QuickDraw Text metrics must match }
	kFNSMatchATSUMetrics = $00000020; { ATSUI metrics (incl. vertical) must match }
	kFNSMatchKerning = $00000040; { kerning data must match }
	kFNSMatchWSLayout = $00000080; { WorldScript layout tables must match }
	kFNSMatchAATLayout = $00000100; { AAT (incl. OpenType) layout tables must match }
	kFNSMatchPrintEncoding = $00000200; { PostScript font and glyph names and re-encoding vector must match }
	kFNSMissingDataNoMatch = $80000000; { treat missing data as mismatch }
	kFNSMatchAll = $FFFFFFFF; { everything must match }
	kFNSMatchDefaults = 0;     { use global default match options }

{$ifc not TARGET_CPU_64}
{
 *  FNSMatchDefaultsGet()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSMatchDefaultsGet: FNSMatchOptions; external name '_FNSMatchDefaultsGet';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ Version control }
{$endc} {not TARGET_CPU_64}

type
	FNSObjectVersion = UInt32;
const
	kFNSVersionDontCare = 0;
	kFNSCurSysInfoVersion = 1;

{ No features defined yet.}
type
	FNSFeatureFlags = UInt32;
{
   The FontSync library version number is binary-coded decimal:
   8 bits of major version, 4 minor version and 4 bits revision.
}
type
	FNSSysInfoPtr = ^FNSSysInfo;
	FNSSysInfo = record
		iSysInfoVersion: FNSObjectVersion;        { fill this in before calling FNSSysInfoGet}
		oFeatures: FNSFeatureFlags;
		oCurRefVersion: FNSObjectVersion;
		oMinRefVersion: FNSObjectVersion;
		oCurProfileVersion: FNSObjectVersion;
		oMinProfileVersion: FNSObjectVersion;
		oFontSyncVersion: UInt16;
	end;
{$ifc not TARGET_CPU_64}
{
 *  FNSSysInfoGet()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
procedure FNSSysInfoGet( var ioInfo: FNSSysInfo ); external name '_FNSSysInfoGet';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ FontSync References }
{$endc} {not TARGET_CPU_64}

type
	FNSFontReference = ^SInt32; { an opaque type }
	FNSFontReferencePtr = ^FNSFontReference;  { when a var xx:FNSFontReference parameter can be nil, it is changed to xx: FNSFontReferencePtr }
{$ifc not TARGET_CPU_64}
{
 *  FNSReferenceGetVersion()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSReferenceGetVersion( iReference: FNSFontReference; var oVersion: FNSObjectVersion ): OSStatus; external name '_FNSReferenceGetVersion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSReferenceDispose()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSReferenceDispose( iReference: FNSFontReference ): OSStatus; external name '_FNSReferenceDispose';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSReferenceMatch()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSReferenceMatch( iReference1: FNSFontReference; iReference2: FNSFontReference; iOptions: FNSMatchOptions; oFailedMatchOptions: FNSMatchOptionsPtr { can be NULL } ): OSStatus; external name '_FNSReferenceMatch';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSReferenceFlattenedSize()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSReferenceFlattenedSize( iReference: FNSFontReference; var oFlattenedSize: ByteCount ): OSStatus; external name '_FNSReferenceFlattenedSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSReferenceFlatten()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSReferenceFlatten( iReference: FNSFontReference; oFlatReference: UnivPtr { can be NULL }; oFlattenedSize: ByteCountPtr { can be NULL } ): OSStatus; external name '_FNSReferenceFlatten';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSReferenceUnflatten()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSReferenceUnflatten( iFlatReference: {const} UnivPtr; iFlattenedSize: ByteCount; var oReference: FNSFontReference ): OSStatus; external name '_FNSReferenceUnflatten';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ FontSync Profiles }
{$endc} {not TARGET_CPU_64}

const
	kFNSCreatorDefault = 0;
	kFNSProfileFileType = FourCharCode('fnsp');

type
	FNSFontProfile = ^SInt32; { an opaque type }
	FNSFontProfilePtr = ^FNSFontProfile;  { when a var xx:FNSFontProfile parameter can be nil, it is changed to xx: FNSFontProfilePtr }
{$ifc not TARGET_CPU_64}
{
 *  FNSProfileCreate()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSProfileCreate( const (*var*) iFile: FSSpec; iCreator: FourCharCode; iEstNumRefs: ItemCount; iDesiredVersion: FNSObjectVersion; var oProfile: FNSFontProfile ): OSStatus; external name '_FNSProfileCreate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSProfileOpen()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSProfileOpen( const (*var*) iFile: FSSpec; iOpenForWrite: Boolean; var oProfile: FNSFontProfile ): OSStatus; external name '_FNSProfileOpen';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSProfileCreateWithFSRef()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   not available
 }
function FNSProfileCreateWithFSRef( const (*var*) iParentDirectory: FSRef; iNameLength: UniCharCount; iName: ConstUniCharPtr; iCreator: FourCharCode; iEstNumRefs: ItemCount; iDesiredVersion: FNSObjectVersion; var oProfile: FNSFontProfile ): OSStatus; external name '_FNSProfileCreateWithFSRef';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSProfileOpenWithFSRef()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   not available
 }
function FNSProfileOpenWithFSRef( const (*var*) iFile: FSRef; iOpenForWrite: Boolean; var oProfile: FNSFontProfile ): OSStatus; external name '_FNSProfileOpenWithFSRef';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSProfileGetVersion()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSProfileGetVersion( iProfile: FNSFontProfile; var oVersion: FNSObjectVersion ): OSStatus; external name '_FNSProfileGetVersion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSProfileCompact()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in  in ApplicationServices.framework [32-bit only] but deprecated in LeopardX
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSProfileCompact( iProfile: FNSFontProfile ): OSStatus; external name '_FNSProfileCompact';


{
 *  FNSProfileClose()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSProfileClose( iProfile: FNSFontProfile ): OSStatus; external name '_FNSProfileClose';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSProfileAddReference()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSProfileAddReference( iProfile: FNSFontProfile; iReference: FNSFontReference ): OSStatus; external name '_FNSProfileAddReference';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSProfileRemoveReference()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSProfileRemoveReference( iProfile: FNSFontProfile; iReference: FNSFontReference ): OSStatus; external name '_FNSProfileRemoveReference';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSProfileRemoveIndReference()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSProfileRemoveIndReference( iProfile: FNSFontProfile; iIndex: UInt32 ): OSStatus; external name '_FNSProfileRemoveIndReference';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSProfileClear()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSProfileClear( iProfile: FNSFontProfile ): OSStatus; external name '_FNSProfileClear';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSProfileCountReferences()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSProfileCountReferences( iProfile: FNSFontProfile; var oCount: ItemCount ): OSStatus; external name '_FNSProfileCountReferences';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSProfileGetIndReference()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSProfileGetIndReference( iProfile: FNSFontProfile; iWhichReference: UInt32; var oReference: FNSFontReference ): OSStatus; external name '_FNSProfileGetIndReference';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSProfileMatchReference()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
<<<<<<< HEAD
function FNSProfileMatchReference(iProfile: FNSFontProfile; iReference: FNSFontReference; iMatchOptions: FNSMatchOptions; iOutputSize: ItemCount; oIndices: UInt32Ptr; oNumMatches: ItemCountPtr): OSStatus; external name '_FNSProfileMatchReference';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function FNSProfileMatchReference( iProfile: FNSFontProfile; iReference: FNSFontReference; iMatchOptions: FNSMatchOptions; iOutputSize: ItemCount; oIndices: {variable-size-array} UInt32Ptr { can be NULL }; oNumMatches: ItemCountPtr { can be NULL } ): OSStatus; external name '_FNSProfileMatchReference';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4


{ Mapping to and from Font Objects }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  FNSReferenceCreate()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSReferenceCreate( iFont: FMFont; iDesiredVersion: FNSObjectVersion; var oReference: FNSFontReference ): OSStatus; external name '_FNSReferenceCreate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSReferenceMatchFonts()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSReferenceMatchFonts( iReference: FNSFontReference; iMatchOptions: FNSMatchOptions; iOutputSize: ItemCount; oFonts: {variable-size-array} FMFontPtr { can be NULL }; oNumMatches: ItemCountPtr { can be NULL } ): OSStatus; external name '_FNSReferenceMatchFonts';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
=======
=======
>>>>>>> origin/fixes_2_2
 *  FNSReferenceCreate()
=======
 *  FNSReferenceCreate()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSReferenceCreate( iFont: FMFont; iDesiredVersion: FNSObjectVersion; var oReference: FNSFontReference ): OSStatus; external name '_FNSReferenceCreate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSReferenceMatchFonts()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
<<<<<<< HEAD
function FNSReferenceMatchFonts(iReference: FNSFontReference; iMatchOptions: FNSMatchOptions; iOutputSize: ItemCount; oFonts: FMFontPtr; oNumMatches: ItemCountPtr): OSStatus; external name '_FNSReferenceMatchFonts';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function FNSReferenceMatchFonts( iReference: FNSFontReference; iMatchOptions: FNSMatchOptions; iOutputSize: ItemCount; oFonts: {variable-size-array} FMFontPtr { can be NULL }; oNumMatches: ItemCountPtr { can be NULL } ): OSStatus; external name '_FNSReferenceMatchFonts';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4


{ Mapping to and from Font Families }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  FNSReferenceCreateFromFamily()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSReferenceCreateFromFamily( iFamily: FMFontFamily; iStyle: FMFontStyle; iDesiredVersion: FNSObjectVersion; oReference: FNSFontReferencePtr { can be NULL }; oActualStyle: FMFontStylePtr { can be NULL } ): OSStatus; external name '_FNSReferenceCreateFromFamily';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSReferenceMatchFamilies()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSReferenceMatchFamilies( iReference: FNSFontReference; iMatchOptions: FNSMatchOptions; iOutputSize: ItemCount; oFonts: {variable-size-array} FMFontFamilyInstancePtr { can be NULL }; oNumMatches: ItemCountPtr { can be NULL } ): OSStatus; external name '_FNSReferenceMatchFamilies';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
=======
=======
>>>>>>> origin/fixes_2_2
 *  FNSReferenceCreateFromFamily()
=======
 *  FNSReferenceCreateFromFamily()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSReferenceCreateFromFamily( iFamily: FMFontFamily; iStyle: FMFontStyle; iDesiredVersion: FNSObjectVersion; oReference: FNSFontReferencePtr { can be NULL }; oActualStyle: FMFontStylePtr { can be NULL } ): OSStatus; external name '_FNSReferenceCreateFromFamily';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSReferenceMatchFamilies()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
<<<<<<< HEAD
function FNSReferenceMatchFamilies(iReference: FNSFontReference; iMatchOptions: FNSMatchOptions; iOutputSize: ItemCount; oFonts: FMFontFamilyInstancePtr; oNumMatches: ItemCountPtr): OSStatus; external name '_FNSReferenceMatchFamilies';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function FNSReferenceMatchFamilies( iReference: FNSFontReference; iMatchOptions: FNSMatchOptions; iOutputSize: ItemCount; oFonts: {variable-size-array} FMFontFamilyInstancePtr { can be NULL }; oNumMatches: ItemCountPtr { can be NULL } ): OSStatus; external name '_FNSReferenceMatchFamilies';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4


{ UI Support }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  FNSReferenceGetFamilyInfo()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSReferenceGetFamilyInfo( iReference: FNSFontReference; oFamilyName: StringPtr { can be NULL }; oFamilyNameScript: ScriptCodePtr { can be NULL }; oActualStyle: FMFontStylePtr { can be NULL } ): OSStatus; external name '_FNSReferenceGetFamilyInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSReferenceCountNames()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSReferenceCountNames( iReference: FNSFontReference; var oNameCount: ItemCount ): OSStatus; external name '_FNSReferenceCountNames';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSReferenceGetIndName()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSReferenceGetIndName( iReference: FNSFontReference; iFontNameIndex: ItemCount; iMaximumNameLength: ByteCount; oName: Ptr { can be NULL }; oActualNameLength: ByteCountPtr { can be NULL }; oFontNameCode: FontNameCodePtr { can be NULL }; oFontNamePlatform: FontPlatformCodePtr { can be NULL }; oFontNameScript: FontScriptCodePtr { can be NULL }; oFontNameLanguage: FontLanguageCodePtr { can be NULL } ): OSStatus; external name '_FNSReferenceGetIndName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSReferenceFindName()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSReferenceFindName( iReference: FNSFontReference; iFontNameCode: FontNameCode; iFontNamePlatform: FontPlatformCode; iFontNameScript: FontScriptCode; iFontNameLanguage: FontLanguageCode; iMaximumNameLength: ByteCount; oName: Ptr { can be NULL }; oActualNameLength: ByteCountPtr { can be NULL }; oFontNameIndex: ItemCountPtr { can be NULL } ): OSStatus; external name '_FNSReferenceFindName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ Miscellany }
{
 *  FNSEnabled()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSEnabled: Boolean; external name '_FNSEnabled';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{$endc} {not TARGET_CPU_64}


{$endc} {TARGET_OS_MAC}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
 *  FNSReferenceGetFamilyInfo()
=======
 *  FNSReferenceGetFamilyInfo()   *** DEPRECATED ***
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSReferenceGetFamilyInfo( iReference: FNSFontReference; oFamilyName: StringPtr { can be NULL }; oFamilyNameScript: ScriptCodePtr { can be NULL }; oActualStyle: FMFontStylePtr { can be NULL } ): OSStatus; external name '_FNSReferenceGetFamilyInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSReferenceCountNames()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSReferenceCountNames( iReference: FNSFontReference; var oNameCount: ItemCount ): OSStatus; external name '_FNSReferenceCountNames';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSReferenceGetIndName()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSReferenceGetIndName( iReference: FNSFontReference; iFontNameIndex: ItemCount; iMaximumNameLength: ByteCount; oName: Ptr { can be NULL }; oActualNameLength: ByteCountPtr { can be NULL }; oFontNameCode: FontNameCodePtr { can be NULL }; oFontNamePlatform: FontPlatformCodePtr { can be NULL }; oFontNameScript: FontScriptCodePtr { can be NULL }; oFontNameLanguage: FontLanguageCodePtr { can be NULL } ): OSStatus; external name '_FNSReferenceGetIndName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FNSReferenceFindName()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSReferenceFindName( iReference: FNSFontReference; iFontNameCode: FontNameCode; iFontNamePlatform: FontPlatformCode; iFontNameScript: FontScriptCode; iFontNameLanguage: FontLanguageCode; iMaximumNameLength: ByteCount; oName: Ptr { can be NULL }; oActualNameLength: ByteCountPtr { can be NULL }; oFontNameIndex: ItemCountPtr { can be NULL } ): OSStatus; external name '_FNSReferenceFindName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ Miscellany }
{
 *  FNSEnabled()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FontSyncLib 1.0 and later
 }
function FNSEnabled: Boolean; external name '_FNSEnabled';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{$endc} {not TARGET_CPU_64}


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
