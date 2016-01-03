{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       AE/AEHelpers.h
 
     Contains:   AEPrint, AEBuild and AEStream for Carbon
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    
=======
     Version:    AppleEvents-496~1
>>>>>>> graemeg/cpstrnew
=======
     Version:    AppleEvents-496~1
>>>>>>> graemeg/cpstrnew
=======
     Version:    AppleEvents-496~1
>>>>>>> graemeg/cpstrnew
=======
     Version:    AppleEvents-496~1
>>>>>>> origin/cpstrnew
 
     Copyright:  © 1999-2008 by Apple Computer, Inc., all rights reserved.
=======
=======
>>>>>>> origin/fixes_2_2
     File:       AEHelpers.p
=======
     File:       AE/AEHelpers.h
>>>>>>> origin/fixes_2.4
 
     Contains:   AEPrint, AEBuild and AEStream for Carbon
 
     Version:    AppleEvents-496~1
 
<<<<<<< HEAD
     Copyright:  © 1999-2002 by Apple Computer, Inc., all rights reserved.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  © 1999-2008 by Apple Computer, Inc., all rights reserved.
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
=======
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit AEHelpers;
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
	{$setc TARGET_CPU_ARM64 := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
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
>>>>>>> graemeg/cpstrnew
=======
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
=======
>>>>>>> graemeg/cpstrnew
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
>>>>>>> graemeg/cpstrnew
=======
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
uses MacTypes,AppleEvents,AEDataModel;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{
 * Originally from AEGIzmos by Jens Alfke, circa 1992.
 }

<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{$ALIGN MAC68K}

{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 * AEBuild:
 *
 * AEBuild provides a very high level abstraction for building
 * complete AppleEvents and complex ObjectSpeciers.  Using AEBuild it
 * is easy to produce a textual representation of an AEDesc.  The
 * format is similar to the stdio printf call, where meta data is
 * extracted from a format string and used to build the final
 * representation.
 * 
 * For more information on AEBuild and other APIs in AEHelpers, see:
 *     <http://developer.apple.com/technotes/tn/tn2045.html>
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 }
{ Syntax Error Codes: }
type
	AEBuildErrorCode = UInt32;
const
	aeBuildSyntaxNoErr = 0;    { (No error) }
	aeBuildSyntaxBadToken = 1;    { Illegal character }
	aeBuildSyntaxBadEOF = 2;    { Unexpected end of format string }
	aeBuildSyntaxNoEOF = 3;    { Unexpected extra stuff past end }
	aeBuildSyntaxBadNegative = 4;    { "-" not followed by digits }
	aeBuildSyntaxMissingQuote = 5;    { Missing close "'" }
	aeBuildSyntaxBadHex = 6;    { Non-digit in hex string }
	aeBuildSyntaxOddHex = 7;    { Odd # of hex digits }
	aeBuildSyntaxNoCloseHex = 8;    { Missing hex quote close "È" }
	aeBuildSyntaxUncoercedHex = 9;    { Hex string must be coerced to a type }
	aeBuildSyntaxNoCloseString = 10;   { Missing close quote }
	aeBuildSyntaxBadDesc = 11;   { Illegal descriptor }
	aeBuildSyntaxBadData = 12;   { Bad data value inside (Ç È) }
	aeBuildSyntaxNoCloseParen = 13;   { Missing ")" after data value }
	aeBuildSyntaxNoCloseBracket = 14;   { Expected "," or "]" }
	aeBuildSyntaxNoCloseBrace = 15;   { Expected "," or ")" }
	aeBuildSyntaxNoKey = 16;   { Missing keyword in record }
	aeBuildSyntaxNoColon = 17;   { Missing ":" after keyword in record }
	aeBuildSyntaxCoercedList = 18;   { Cannot coerce a list }
	aeBuildSyntaxUncoercedDoubleAt = 19;   { "@@" substitution must be coerced }

{ A structure containing error state.}

type
	AEBuildErrorPtr = ^AEBuildError;
	AEBuildError = record
		fError: AEBuildErrorCode;
		fErrorPos: UInt32;
	end;
{
   Create an AEDesc from the format string.  AEBuildError can be NULL, in which case
   no explicit error information will be returned.
}
{
 *  AEBuildDesc()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AEBuildDesc( var dst: AEDesc; error: AEBuildErrorPtr { can be NULL }; src: ConstCStringPtr; ... ): OSStatus; external name '_AEBuildDesc';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew


{ varargs version of AEBuildDesc}
{
 *  vAEBuildDesc()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
// function vAEBuildDesc( var dst: AEDesc; error: AEBuildErrorPtr { can be NULL }; src: ConstCStringPtr; args: va_list ): OSStatus;
<<<<<<< HEAD
<<<<<<< HEAD
// __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA );
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER;
>>>>>>> graemeg/cpstrnew
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER;
>>>>>>> graemeg/cpstrnew


{ Append parameters to an existing AppleEvent}
{
 *  AEBuildParameters()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AEBuildParameters( var event: AppleEvent; error: AEBuildErrorPtr { can be NULL }; format: ConstCStringPtr; ... ): OSStatus; external name '_AEBuildParameters';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew


{ varargs version of AEBuildParameters}
{
 *  vAEBuildParameters()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
// function vAEBuildParameters( var event: AppleEvent; error: AEBuildErrorPtr { can be NULL }; format: ConstCStringPtr; args: va_list ): OSStatus;
<<<<<<< HEAD
<<<<<<< HEAD
// __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA );
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER;
>>>>>>> graemeg/cpstrnew
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER;
>>>>>>> graemeg/cpstrnew


{ Building an entire Apple event:}
{
 *  AEBuildAppleEvent()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
<<<<<<< HEAD
 }
function AEBuildAppleEvent( theClass: AEEventClass; theID: AEEventID; addressType: DescType; addressData: {const} UnivPtr; addressLength: Size; returnID: SInt16; transactionID: SInt32; var result: AppleEvent; error: AEBuildErrorPtr { can be NULL }; paramsFmt: ConstCStringPtr; ... ): OSStatus; external name '_AEBuildAppleEvent';
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)


{ varargs version of AEBuildAppleEvent}
{
 *  vAEBuildAppleEvent()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
// function vAEBuildAppleEvent( theClass: AEEventClass; theID: AEEventID; addressType: DescType; addressData: {const} UnivPtr; addressLength: Size; returnID: SInt16; transactionID: SInt32; var resultEvt: AppleEvent; error: AEBuildErrorPtr { can be NULL }; paramsFmt: ConstCStringPtr; args: va_list ): OSStatus;
// __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA );
=======
 }
function AEBuildAppleEvent( theClass: AEEventClass; theID: AEEventID; addressType: DescType; addressData: {const} UnivPtr; addressLength: Size; returnID: SInt16; transactionID: SInt32; var result: AppleEvent; error: AEBuildErrorPtr { can be NULL }; paramsFmt: ConstCStringPtr; ... ): OSStatus; external name '_AEBuildAppleEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ varargs version of AEBuildAppleEvent}
{
 *  vAEBuildAppleEvent()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
// function vAEBuildAppleEvent( theClass: AEEventClass; theID: AEEventID; addressType: DescType; addressData: {const} UnivPtr; addressLength: Size; returnID: SInt16; transactionID: SInt32; var resultEvt: AppleEvent; error: AEBuildErrorPtr { can be NULL }; paramsFmt: ConstCStringPtr; args: va_list ): OSStatus;
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER;
>>>>>>> graemeg/cpstrnew
=======
 }
=======
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
 }
{ Syntax Error Codes: }
type
	AEBuildErrorCode = UInt32;
const
	aeBuildSyntaxNoErr = 0;    { (No error) }
	aeBuildSyntaxBadToken = 1;    { Illegal character }
	aeBuildSyntaxBadEOF = 2;    { Unexpected end of format string }
	aeBuildSyntaxNoEOF = 3;    { Unexpected extra stuff past end }
	aeBuildSyntaxBadNegative = 4;    { "-" not followed by digits }
	aeBuildSyntaxMissingQuote = 5;    { Missing close "'" }
	aeBuildSyntaxBadHex = 6;    { Non-digit in hex string }
	aeBuildSyntaxOddHex = 7;    { Odd # of hex digits }
	aeBuildSyntaxNoCloseHex = 8;    { Missing hex quote close "È" }
	aeBuildSyntaxUncoercedHex = 9;    { Hex string must be coerced to a type }
	aeBuildSyntaxNoCloseString = 10;   { Missing close quote }
	aeBuildSyntaxBadDesc = 11;   { Illegal descriptor }
	aeBuildSyntaxBadData = 12;   { Bad data value inside (Ç È) }
	aeBuildSyntaxNoCloseParen = 13;   { Missing ")" after data value }
	aeBuildSyntaxNoCloseBracket = 14;   { Expected "," or "]" }
	aeBuildSyntaxNoCloseBrace = 15;   { Expected "," or ")" }
	aeBuildSyntaxNoKey = 16;   { Missing keyword in record }
	aeBuildSyntaxNoColon = 17;   { Missing ":" after keyword in record }
	aeBuildSyntaxCoercedList = 18;   { Cannot coerce a list }
	aeBuildSyntaxUncoercedDoubleAt = 19;   { "@@" substitution must be coerced }

{ A structure containing error state.}

type
	AEBuildErrorPtr = ^AEBuildError;
	AEBuildError = record
		fError: AEBuildErrorCode;
		fErrorPos: UInt32;
	end;
{
   Create an AEDesc from the format string.  AEBuildError can be NULL, in which case
   no explicit error information will be returned.
}
{
 *  AEBuildDesc()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AEBuildDesc( var dst: AEDesc; error: AEBuildErrorPtr { can be NULL }; src: ConstCStringPtr; ... ): OSStatus; external name '_AEBuildDesc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ varargs version of AEBuildDesc}
{
 *  vAEBuildDesc()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
// function vAEBuildDesc( var dst: AEDesc; error: AEBuildErrorPtr { can be NULL }; src: ConstCStringPtr; args: va_list ): OSStatus;
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER;


{ Append parameters to an existing AppleEvent}
{
 *  AEBuildParameters()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AEBuildParameters( var event: AppleEvent; error: AEBuildErrorPtr { can be NULL }; format: ConstCStringPtr; ... ): OSStatus; external name '_AEBuildParameters';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ varargs version of AEBuildParameters}
{
 *  vAEBuildParameters()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
// function vAEBuildParameters( var event: AppleEvent; error: AEBuildErrorPtr { can be NULL }; format: ConstCStringPtr; args: va_list ): OSStatus;
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER;


{ Building an entire Apple event:}
{
 *  AEBuildAppleEvent()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
 }
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function AEBuildAppleEvent( theClass: AEEventClass; theID: AEEventID; addressType: DescType; addressData: {const} UnivPtr; addressLength: Size; returnID: SInt16; transactionID: SInt32; var result: AppleEvent; error: AEBuildErrorPtr { can be NULL }; paramsFmt: ConstCStringPtr; ... ): OSStatus; external name '_AEBuildAppleEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ varargs version of AEBuildAppleEvent}
{
 *  vAEBuildAppleEvent()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
// function vAEBuildAppleEvent( theClass: AEEventClass; theID: AEEventID; addressType: DescType; addressData: {const} UnivPtr; addressLength: Size; returnID: SInt16; transactionID: SInt32; var resultEvt: AppleEvent; error: AEBuildErrorPtr { can be NULL }; paramsFmt: ConstCStringPtr; args: va_list ): OSStatus;
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER;
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew


=======
 * AEBuild is only available for C programmers.
 }
>>>>>>> graemeg/fixes_2_2
=======
 * AEBuild is only available for C programmers.
 }
>>>>>>> origin/fixes_2_2
=======
 }
function AEBuildAppleEvent( theClass: AEEventClass; theID: AEEventID; addressType: DescType; addressData: {const} UnivPtr; addressLength: Size; returnID: SInt16; transactionID: SInt32; var result: AppleEvent; error: AEBuildErrorPtr { can be NULL }; paramsFmt: ConstCStringPtr; ... ): OSStatus; external name '_AEBuildAppleEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ varargs version of AEBuildAppleEvent}
{
 *  vAEBuildAppleEvent()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
// function vAEBuildAppleEvent( theClass: AEEventClass; theID: AEEventID; addressType: DescType; addressData: {const} UnivPtr; addressLength: Size; returnID: SInt16; transactionID: SInt32; var resultEvt: AppleEvent; error: AEBuildErrorPtr { can be NULL }; paramsFmt: ConstCStringPtr; args: va_list ): OSStatus;
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER;


>>>>>>> origin/fixes_2.4
{
 * AEPrintDescToHandle
 *
 * AEPrintDescToHandle provides a way to turn an AEDesc into a textual
 * representation.  This is most useful for debugging calls to
 * AEBuildDesc and friends.  The Handle returned should be disposed by
 * the caller.  The size of the handle is the actual number of
 * characters in the string.
 }
{
 *  AEPrintDescToHandle()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AEPrintDescToHandle( const (*var*) desc: AEDesc; var result: Handle ): OSStatus; external name '_AEPrintDescToHandle';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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
 *    Thread safe since version 10.2
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function AEPrintDescToHandle(const (*var*) desc: AEDesc; var result: Handle): OSStatus; external name '_AEPrintDescToHandle';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function AEPrintDescToHandle( const (*var*) desc: AEDesc; var result: Handle ): OSStatus; external name '_AEPrintDescToHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 * AEStream:
 *
 * The AEStream interface allows you to build AppleEvents by appending
 * to an opaque structure (an AEStreamRef) and then turning this
 * structure into an AppleEvent.  The basic idea is to open the
 * stream, write data, and then close it - closing it produces an
 * AEDesc, which may be partially complete, or may be a complete
 * AppleEvent.
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	AEStreamRef = ^OpaqueAEStreamRef; { an opaque type }
	OpaqueAEStreamRef = record end;
=======
	AEStreamRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	AEStreamRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	AEStreamRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	AEStreamRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
{
   Create and return an AEStreamRef
   Returns NULL on memory allocation failure
}
{
 *  AEStreamOpen()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AEStreamOpen: AEStreamRef; external name '_AEStreamOpen';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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
>>>>>>> origin/fixes_2.4
type
	AEStreamRef = ^SInt32; { an opaque type }
{
   Create and return an AEStreamRef
   Returns NULL on memory allocation failure
}
{
 *  AEStreamOpen()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AEStreamOpen: AEStreamRef; external name '_AEStreamOpen';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
   Closes and disposes of an AEStreamRef, producing
   results in the desc.  You must dispose of the desc yourself.
   If you just want to dispose of the AEStreamRef, you can pass NULL for desc.
}
{
 *  AEStreamClose()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AEStreamClose( ref: AEStreamRef; var desc: AEDesc ): OSStatus; external name '_AEStreamClose';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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
 *    Thread safe since version 10.2
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function AEStreamClose(ref: AEStreamRef; var desc: AEDesc): OSStatus; external name '_AEStreamClose';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function AEStreamClose( ref: AEStreamRef; var desc: AEDesc ): OSStatus; external name '_AEStreamClose';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
   Prepares an AEStreamRef for appending data to a newly created desc.
   You append data with AEStreamWriteData
}
{
 *  AEStreamOpenDesc()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AEStreamOpenDesc( ref: AEStreamRef; newType: DescType ): OSStatus; external name '_AEStreamOpenDesc';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)


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
{ Append data to the previously opened desc.}
{
 *  AEStreamWriteData()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AEStreamWriteData( ref: AEStreamRef; data: {const} UnivPtr; length: Size ): OSStatus; external name '_AEStreamWriteData';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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
 *    Thread safe since version 10.2
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AEStreamOpenDesc( ref: AEStreamRef; newType: DescType ): OSStatus; external name '_AEStreamOpenDesc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Append data to the previously opened desc.}
{
 *  AEStreamWriteData()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function AEStreamWriteData(ref: AEStreamRef; data: UnivPtr; length: Size): OSStatus; external name '_AEStreamWriteData';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function AEStreamWriteData( ref: AEStreamRef; data: {const} UnivPtr; length: Size ): OSStatus; external name '_AEStreamWriteData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
   Finish a desc.  After this, you can close the stream, or adding new
   descs, if you're assembling a list.
}
{
 *  AEStreamCloseDesc()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AEStreamCloseDesc( ref: AEStreamRef ): OSStatus; external name '_AEStreamCloseDesc';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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
{ Write data as a desc to the stream}
{
 *  AEStreamWriteDesc()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AEStreamWriteDesc( ref: AEStreamRef; newType: DescType; data: {const} UnivPtr; length: Size ): OSStatus; external name '_AEStreamWriteDesc';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)


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
{ Write an entire desc to the stream}
{
 *  AEStreamWriteAEDesc()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AEStreamWriteAEDesc( ref: AEStreamRef; const (*var*) desc: AEDesc ): OSStatus; external name '_AEStreamWriteAEDesc';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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
 *    Thread safe since version 10.2
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AEStreamCloseDesc( ref: AEStreamRef ): OSStatus; external name '_AEStreamCloseDesc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Write data as a desc to the stream}
{
 *  AEStreamWriteDesc()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AEStreamWriteDesc( ref: AEStreamRef; newType: DescType; data: {const} UnivPtr; length: Size ): OSStatus; external name '_AEStreamWriteDesc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Write an entire desc to the stream}
{
 *  AEStreamWriteAEDesc()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function AEStreamWriteAEDesc(ref: AEStreamRef; const (*var*) desc: AEDesc): OSStatus; external name '_AEStreamWriteAEDesc';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function AEStreamWriteAEDesc( ref: AEStreamRef; const (*var*) desc: AEDesc ): OSStatus; external name '_AEStreamWriteAEDesc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
   Begin a list.  You can then append to the list by doing
   AEStreamOpenDesc, or AEStreamWriteDesc.
}
{
 *  AEStreamOpenList()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AEStreamOpenList( ref: AEStreamRef ): OSStatus; external name '_AEStreamOpenList';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)


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
{ Finish a list.}
{
 *  AEStreamCloseList()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AEStreamCloseList( ref: AEStreamRef ): OSStatus; external name '_AEStreamCloseList';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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
 *    Thread safe since version 10.2
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AEStreamOpenList( ref: AEStreamRef ): OSStatus; external name '_AEStreamOpenList';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Finish a list.}
{
 *  AEStreamCloseList()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function AEStreamCloseList(ref: AEStreamRef): OSStatus; external name '_AEStreamCloseList';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function AEStreamCloseList( ref: AEStreamRef ): OSStatus; external name '_AEStreamCloseList';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
   Begin a record.  A record usually has type 'reco', however, this is
   rather generic, and frequently a different type is used.
}
{
 *  AEStreamOpenRecord()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AEStreamOpenRecord( ref: AEStreamRef; newType: DescType ): OSStatus; external name '_AEStreamOpenRecord';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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
{ Change the type of a record.}
{
 *  AEStreamSetRecordType()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AEStreamSetRecordType( ref: AEStreamRef; newType: DescType ): OSStatus; external name '_AEStreamSetRecordType';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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
{ Finish a record}
{
 *  AEStreamCloseRecord()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AEStreamCloseRecord( ref: AEStreamRef ): OSStatus; external name '_AEStreamCloseRecord';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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
 *    Thread safe since version 10.2
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AEStreamOpenRecord( ref: AEStreamRef; newType: DescType ): OSStatus; external name '_AEStreamOpenRecord';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Change the type of a record.}
{
 *  AEStreamSetRecordType()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AEStreamSetRecordType( ref: AEStreamRef; newType: DescType ): OSStatus; external name '_AEStreamSetRecordType';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Finish a record}
{
 *  AEStreamCloseRecord()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function AEStreamCloseRecord(ref: AEStreamRef): OSStatus; external name '_AEStreamCloseRecord';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function AEStreamCloseRecord( ref: AEStreamRef ): OSStatus; external name '_AEStreamCloseRecord';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
   Add a keyed descriptor to a record.  This is analogous to AEPutParamDesc.
   it can only be used when writing to a record.
}
{
 *  AEStreamWriteKeyDesc()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AEStreamWriteKeyDesc( ref: AEStreamRef; key: AEKeyword; newType: DescType; data: {const} UnivPtr; length: Size ): OSStatus; external name '_AEStreamWriteKeyDesc';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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
   OpenDesc for a keyed record entry.  You can use AEStreamWriteData
=======
=======
>>>>>>> origin/fixes_2_2
=======
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AEStreamWriteKeyDesc( ref: AEStreamRef; key: AEKeyword; newType: DescType; data: {const} UnivPtr; length: Size ): OSStatus; external name '_AEStreamWriteKeyDesc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
   OpenDesc for a keyed record entry.  You can youse AEStreamWriteData
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
   OpenDesc for a keyed record entry.  You can use AEStreamWriteData
>>>>>>> origin/fixes_2.4
   after opening a keyed desc.
}
{
 *  AEStreamOpenKeyDesc()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AEStreamOpenKeyDesc( ref: AEStreamRef; key: AEKeyword; newType: DescType ): OSStatus; external name '_AEStreamOpenKeyDesc';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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
{ Write a key to the stream - you can follow this with an AEWriteDesc.}
{
 *  AEStreamWriteKey()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AEStreamWriteKey( ref: AEStreamRef; key: AEKeyword ): OSStatus; external name '_AEStreamWriteKey';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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
 *    Thread safe since version 10.2
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AEStreamOpenKeyDesc( ref: AEStreamRef; key: AEKeyword; newType: DescType ): OSStatus; external name '_AEStreamOpenKeyDesc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Write a key to the stream - you can follow this with an AEWriteDesc.}
{
 *  AEStreamWriteKey()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function AEStreamWriteKey(ref: AEStreamRef; key: AEKeyword): OSStatus; external name '_AEStreamWriteKey';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function AEStreamWriteKey( ref: AEStreamRef; key: AEKeyword ): OSStatus; external name '_AEStreamWriteKey';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
   Create a complete AppleEvent.  This creates and returns a new stream.
   Use this call to populate the meta fields in an AppleEvent record.
   After this, you can add your records, lists and other parameters.
}
{
 *  AEStreamCreateEvent()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AEStreamCreateEvent( clazz: AEEventClass; id: AEEventID; targetType: DescType; targetData: {const} UnivPtr; targetLength: Size; returnID: SInt16; transactionID: SInt32 ): AEStreamRef; external name '_AEStreamCreateEvent';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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
 *    Thread safe since version 10.2
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function AEStreamCreateEvent(clazz: AEEventClass; id: AEEventID; targetType: DescType; targetData: UnivPtr; targetLength: SInt32; returnID: SInt16; transactionID: SInt32): AEStreamRef; external name '_AEStreamCreateEvent';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function AEStreamCreateEvent( clazz: AEEventClass; id: AEEventID; targetType: DescType; targetData: {const} UnivPtr; targetLength: Size; returnID: SInt16; transactionID: SInt32 ): AEStreamRef; external name '_AEStreamCreateEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
   This call lets you augment an existing AppleEvent using the stream
   APIs.  This would be useful, for example, in constructing the reply
   record in an AppleEvent handler.  Note that AEStreamOpenEvent will
   consume the AppleEvent passed in - you can't access it again until the
   stream is closed.  When you're done building the event, AEStreamCloseStream
    will reconstitute it.
}
{
 *  AEStreamOpenEvent()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AEStreamOpenEvent( var event: AppleEvent ): AEStreamRef; external name '_AEStreamOpenEvent';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)


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
{ Mark a keyword as being an optional parameter.}
{
 *  AEStreamOptionalParam()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AEStreamOptionalParam( ref: AEStreamRef; key: AEKeyword ): OSStatus; external name '_AEStreamOptionalParam';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)

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


{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
=======
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AEStreamOpenEvent( var event: AppleEvent ): AEStreamRef; external name '_AEStreamOpenEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Mark a keyword as being an optional parameter.}
{
 *  AEStreamOptionalParam()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function AEStreamOptionalParam( ref: AEStreamRef; key: AEKeyword ): OSStatus; external name '_AEStreamOptionalParam';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


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
