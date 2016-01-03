{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       CarbonCore/PLStringFuncs.h
 
     Contains:   Pascal string manipulation routines that parallel ANSI C string.h
                 The contents of this header file are deprecated.
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Copyright:  © 1999-2011 by Apple Inc. All rights reserved.
}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
     Version:    CarbonCore-859.2~1
 
     Copyright:  © 1999-2008 by Apple Computer, Inc., all rights reserved.
=======
=======
>>>>>>> origin/fixes_2_2
     File:       PLStringFuncs.p
=======
     File:       CarbonCore/PLStringFuncs.h
>>>>>>> origin/fixes_2.4
 
     Contains:   Pascal string manipulation routines that parallel ANSI C string.h
 
     Version:    CarbonCore-859.2~1
 
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

unit PLStringFuncs;
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
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
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
=======
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
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$elifc defined __arm__ and __arm__}
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_CPU_ARM64 := TRUE}
=======
	{$setc TARGET_CPU_ARM := TRUE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_CPU_ARM := TRUE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_CPU_ARM := TRUE}
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
uses MacTypes;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{
 *  PLstrcmp()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use CFString instead.
 *  
 *  Summary:
 *    Compare two pascal strings
 *  
 *  Discussion:
 *    This function compares two pascal strings, and returns a value <
 *    0 if the first string is lexicographically less than the second
 *    string, or 0 if the two strings are identical, or a value > 0 if
 *    the first string is lexicographically greater than the second.
 *    This function should be deprecated since pascal strings are
 *    obsolete on MacOSX and CFString should be used instead.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    Thread safe provided no other thread is modifying str1 or str2.
 *  
 *  Parameters:
 *    
 *    str1:
 *      the first pascal string
 *    
 *    str2:
 *      the second pascal string
 *  
 *  Result:
 *    This function returns an integer greater than, equal to, or less
 *    than 0, according as the string str1 is greater than, equal to,
 *    or less than the string str2.  The comparison is done using
 *    unsigned characters, so that `\200' is greater than `\0'.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PLstrcmp( const (*var*) str1: Str255; const (*var*) str2: Str255 ): SInt16; external name '_PLstrcmp';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{
 *  PLstrncmp()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use CFString instead.
 *  
 *  Summary:
 *    Compare two pascal strings
 *  
 *  Discussion:
 *    This function compares two pascal strings, and returns a value <
 *    0 if the first string is lexicographically less than the second
 *    string, or 0 if the two strings are identical, or a value > 0 if
 *    the first string is lexicographically greater than the second. 
 *    This function compares not more than num characters of either
 *    string, even if their lengths are greater than num.  Two strings
 *    whose first num characters are identical will return 0 when
 *    compared. This function should be deprecated since pascal strings
 *    are obsolete on MacOSX and CFString should be used instead.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    Thread safe provided no other thread is modifying str1 or str2.
 *  
 *  Parameters:
 *    
 *    str1:
 *      the first pascal string
 *    
 *    str2:
 *      the second pascal string
 *    
 *    num:
 *      the maximum number of characters to compare
 *  
 *  Result:
 *    This function returns an integer greater than, equal to, or less
 *    than 0, according as the string str1 is greater than, equal to,
 *    or less than the string str2.  The comparison is done using
 *    unsigned characters, so that `\200' is greater than `\0'.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PLstrncmp( const (*var*) str1: Str255; const (*var*) str2: Str255; num: SInt16 ): SInt16; external name '_PLstrncmp';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{
 *  PLstrcpy()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use CFString instead.
 *  
 *  Summary:
 *    Copy a pascal string
 *  
 *  Discussion:
 *    This function copies the string source to dest (including the
 *    initial length byte ). The caller must ensure that neither source
 *    or dest are NULL, and that dest is large enough to hold the
 *    entire contents of source. This function should be deprecated
 *    since pascal strings are obsolete on MacOSX and CFString should
 *    be used instead.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    dest:
 *      the destination pascal string
 *    
 *    source:
 *      the source pascal string
 *  
 *  Result:
 *    This function returns dest.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PLstrcpy( dest: StringPtr; const (*var*) source: Str255 ): StringPtr; external name '_PLstrcpy';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{
 *  PLstrncpy()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use CFString instead.
 *  
 *  Summary:
 *    Copy a pascal string
 *  
 *  Discussion:
 *    This function copies the string source to dest (including the
 *    initial length byte ), provided the length of source is <= num. 
 *    If the length of source is > num, then the first num characters
 *    of source are copied into dest, and the length of dest is set to
 *    num.  The caller must ensure that neither source or dest are
 *    NULL, and that dest is large enough to hold the entire contents
 *    of source. This function should be deprecated since pascal
 *    strings are obsolete on MacOSX and CFString should be used
 *    instead.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    Thread safe provided no other thread is modifying source.
 *  
 *  Parameters:
 *    
 *    dest:
 *      the destination pascal string
 *    
 *    source:
 *      the source pascal string
 *    
 *    num:
 *      the maximum number of bytes to copy
 *  
 *  Result:
 *    This function returns dest.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PLstrncpy( dest: StringPtr; const (*var*) source: Str255; num: SInt16 ): StringPtr; external name '_PLstrncpy';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{
 *  PLstrcat()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use CFString instead.
 *  
 *  Summary:
 *    Append a pascal string to another pascal string
 *  
 *  Discussion:
 *    This function append a copy of the pascal string append to the
 *    end of the pascal string str.  If the length of str plus the
 *    length of append is greater than 255 ( the maximum size of a
 *    pascal string ) then only enough characters are copied to str to
 *    reach the 255 character limit, and the length of str is set to
 *    255.  The caller must ensure that neither str nor append are
 *    NULL, and that str is large enough to hold the entire contents of
 *    append. This function should be deprecated since pascal strings
 *    are obsolete on MacOSX and CFString should be used instead.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    Thread safe provided no other thread is modifying str or append.
 *  
 *  Parameters:
 *    
 *    str:
 *      the destination pascal string
 *    
 *    append:
 *      the pascal string to append
 *  
 *  Result:
 *    This function returns s.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PLstrcat( str: StringPtr; const (*var*) append: Str255 ): StringPtr; external name '_PLstrcat';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{
 *  PLstrncat()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use CFString instead.
 *  
 *  Summary:
 *    Append up to num bytes of a pascal string to another pascal string
 *  
 *  Discussion:
 *    This function append up to the first num bytes of the pascal
 *    string append to the end of the pascal string s.  If the length
 *    of str plus the length of append is greater than 255 ( the
 *    maximum size of a pascal string ) then only enough characters are
 *    copied to str to reach the 255 character limit, and the length of
 *    str is set to 255.  The caller must ensure that neither str nor
 *    append are NULL, and that str is large enough to hold the entire
 *    contents of append. This function should be deprecated since
 *    pascal strings are obsolete on MacOSX and CFString should be used
 *    instead.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    Thread safe provided no other thread is modifying str1 or append.
 *  
 *  Parameters:
 *    
 *    str1:
 *      the destination pascal string
 *    
 *    append:
 *      the pascal string to append
 *    
 *    num:
 *      the maximum number of bytes of append to append onto s
 *  
 *  Result:
 *    This function returns str.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PLstrncat( str1: StringPtr; const (*var*) append: Str255; num: SInt16 ): StringPtr; external name '_PLstrncat';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{
 *  PLstrchr()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use CFString instead.
 *  
 *  Summary:
 *    Return a pointer to the first occurrence of ch1 in str.
 *  
 *  Discussion:
 *    The PLstrrchr() function locates the first occurrence of ch1
 *    (converted to an unsigned char) in the string s.  If ch1 does not
 *    occur in the string, this returns NULL. This function should be
 *    deprecated since pascal strings are obsolete on MacOSX and
 *    CFString should be used instead.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    Thread safe provided no other thread is modifying str1.
 *  
 *  Parameters:
 *    
 *    str1:
 *      the pascal string
 *    
 *    ch1:
 *      the character to find
 *  
 *  Result:
 *    A pointer to the first occurrence of ch1 in str1, or NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PLstrchr( const (*var*) str1: Str255; ch1: SInt16 ): Ptr; external name '_PLstrchr';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{
 *  PLstrrchr()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use CFString instead.
 *  
 *  Summary:
 *    Return a pointer to the last occurrence of ch1 in str.
 *  
 *  Discussion:
 *    The PLstrrchr() function locates the last occurrence of ch1
 *    (converted to an unsigned char) in the string s.  If ch1 does not
 *    occur in the string, this returns NULL. This function should be
 *    deprecated since pascal strings are obsolete on MacOSX and
 *    CFString should be used instead.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    Thread safe provided no other thread is modifying str1.
 *  
 *  Parameters:
 *    
 *    str1:
 *      the pascal string
 *    
 *    ch1:
 *      the character to find
 *  
 *  Result:
 *    A pointer to the last occurrence of ch1 in str1, or NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PLstrrchr( const (*var*) str1: Str255; ch1: SInt16 ): Ptr; external name '_PLstrrchr';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{
 *  PLstrpbrk()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use CFString instead.
 *  
 *  Summary:
 *    Return a pointer to the first occurrence in str of any character
 *    in charSet.
 *  
 *  Discussion:
 *    The PLstrpbrk() function returns a pointer to the first
 *    occurrence in str of any character in searchStr.  If none of the
 *    characters in searchStr can be found in str, then NULL is
 *    returned. This function should be deprecated since pascal strings
 *    are obsolete on MacOSX and CFString should be used instead.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    Thread safe provided no other thread is modifying str1 or charSet.
 *  
 *  Parameters:
 *    
 *    str1:
 *      the pascal string
 *    
 *    charSet:
 *      the character to find
 *  
 *  Result:
 *    A pointer to the first occurrence of any character in charSet in
 *    str1, or NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PLstrpbrk( const (*var*) str1: Str255; const (*var*) charSet: Str255 ): Ptr; external name '_PLstrpbrk';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{
 *  PLstrspn()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use CFString instead.
 *  
 *  Summary:
 *    Spans the initial part of str1 as long as the characters from
 *    str1 occur in string charset
 *  
 *  Discussion:
 *    The PLstrspn() function spans the initial part of the pascal
 *    string str1 as long as the characters from s occur in string
 *    charset. In effect, this returns a count of the number of
 *    characters at the beginning of the pascal string str1 which are
 *    in charset. This function should be deprecated since pascal
 *    strings are obsolete on MacOSX and CFString should be used
 *    instead.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    Thread safe provided no other thread is modifying str1 or charSet.
 *  
 *  Parameters:
 *    
 *    str1:
 *      the pascal string
 *    
 *    charSet:
 *      the character to find
 *  
 *  Result:
 *    The count of characters at the beginning of str1 which are in
 *    charSet.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PLstrspn( const (*var*) str1: Str255; const (*var*) charSet: Str255 ): SInt16; external name '_PLstrspn';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{
 *  PLstrstr()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use CFString instead.
 *  
 *  Summary:
 *    Returns a pointer to the first occurrence of searchStr in str1
 *  
 *  Discussion:
 *    The PLstrstr() function returns a pointer to the first occurrence
 *    of searchStr in str1, or NULL if searchStr does not exist in
 *    str1. This function should be deprecated since pascal strings are
 *    obsolete on MacOSX and CFString should be used instead.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    Thread safe provided no other thread is modifying str1 or
 *    searchStr.
 *  
 *  Parameters:
 *    
 *    str1:
 *      the pascal string
 *    
 *    searchStr:
 *      the string to find
 *  
 *  Result:
 *    The count of characters at the beginning of str1 which are in
 *    charSet.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PLstrstr( const (*var*) str1: Str255; const (*var*) searchStr: Str255 ): Ptr; external name '_PLstrstr';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{
 *  PLstrlen()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use CFString instead.
 *  
 *  Summary:
 *    Returns the length of the pascal string
 *  
 *  Discussion:
 *    The PLstrlen() function returns the length of the pascal string
 *    str. This function should be deprecated since pascal strings are
 *    obsolete on MacOSX and CFString should be used instead.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    Thread safe provided no other thread is modifying str.
 *  
 *  Parameters:
 *    
 *    str:
 *      the pascal string
 *  
 *  Result:
 *    The length of the pascal string str.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PLstrlen( const (*var*) str: Str255 ): SInt16; external name '_PLstrlen';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{
 *  PLpos()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use CFString instead.
 *  
 *  Summary:
 *    Returns the offset to the first occurrence of searchStr in str1
 *  
 *  Discussion:
 *    The PLpos() function returns the offset of the string searchStr
 *    in str1, or 0 if searchStr does not occur in str1.  For example,
 *    if str1 is "\pHello World" and searchStr is "\pWorld", then this
 *    function will return the value 7. This function should be
 *    deprecated since pascal strings are obsolete on MacOSX and
 *    CFString should be used instead.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    Thread safe provided no other thread is modifying str1 or
 *    searchStr.
 *  
 *  Parameters:
 *    
 *    str1:
 *      the pascal string
 *    
 *    searchStr:
 *      the string to find
 *  
 *  Result:
 *    The count of characters at the beginning of str1 which are in
 *    charSet.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PLpos( const (*var*) str1: Str255; const (*var*) searchStr: Str255 ): SInt16; external name '_PLpos';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

>>>>>>> origin/cpstrnew

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4


{$ifc TARGET_OS_MAC}

{
 *  PLstrcmp()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use CFString instead.
 *  
 *  Summary:
 *    Compare two pascal strings
 *  
 *  Discussion:
 *    This function compares two pascal strings, and returns a value <
 *    0 if the first string is lexicographically less than the second
 *    string, or 0 if the two strings are identical, or a value > 0 if
 *    the first string is lexicographically greater than the second.
 *    This function should be deprecated since pascal strings are
 *    obsolete on MacOSX and CFString should be used instead.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    Thread safe provided no other thread is modifying str1 or str2.
 *  
 *  Parameters:
 *    
 *    str1:
 *      the first pascal string
 *    
 *    str2:
 *      the second pascal string
 *  
 *  Result:
 *    This function returns an integer greater than, equal to, or less
 *    than 0, according as the string str1 is greater than, equal to,
 *    or less than the string str2.  The comparison is done using
 *    unsigned characters, so that `\200' is greater than `\0'.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PLstrcmp( const (*var*) str1: Str255; const (*var*) str2: Str255 ): SInt16; external name '_PLstrcmp';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PLstrncmp()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use CFString instead.
 *  
 *  Summary:
 *    Compare two pascal strings
 *  
 *  Discussion:
 *    This function compares two pascal strings, and returns a value <
 *    0 if the first string is lexicographically less than the second
 *    string, or 0 if the two strings are identical, or a value > 0 if
 *    the first string is lexicographically greater than the second. 
 *    This function compares not more than num characters of either
 *    string, even if their lengths are greater than num.  Two strings
 *    whose first num characters are identical will return 0 when
 *    compared. This function should be deprecated since pascal strings
 *    are obsolete on MacOSX and CFString should be used instead.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    Thread safe provided no other thread is modifying str1 or str2.
 *  
 *  Parameters:
 *    
 *    str1:
 *      the first pascal string
 *    
 *    str2:
 *      the second pascal string
 *    
 *    num:
 *      the maximum number of characters to compare
 *  
 *  Result:
 *    This function returns an integer greater than, equal to, or less
 *    than 0, according as the string str1 is greater than, equal to,
 *    or less than the string str2.  The comparison is done using
 *    unsigned characters, so that `\200' is greater than `\0'.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PLstrncmp( const (*var*) str1: Str255; const (*var*) str2: Str255; num: SInt16 ): SInt16; external name '_PLstrncmp';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PLstrcpy()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use CFString instead.
 *  
 *  Summary:
 *    Copy a pascal string
 *  
 *  Discussion:
 *    This function copies the string source to dest (including the
 *    initial length byte ). The caller must ensure that neither source
 *    or dest are NULL, and that dest is large enough to hold the
 *    entire contents of source. This function should be deprecated
 *    since pascal strings are obsolete on MacOSX and CFString should
 *    be used instead.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    dest:
 *      the destination pascal string
 *    
 *    source:
 *      the source pascal string
 *  
 *  Result:
 *    This function returns dest.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PLstrcpy( dest: StringPtr; const (*var*) source: Str255 ): StringPtr; external name '_PLstrcpy';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PLstrncpy()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use CFString instead.
 *  
 *  Summary:
 *    Copy a pascal string
 *  
 *  Discussion:
 *    This function copies the string source to dest (including the
 *    initial length byte ), provided the length of source is <= num. 
 *    If the length of source is > num, then the first num characters
 *    of source are copied into dest, and the length of dest is set to
 *    num.  The caller must ensure that neither source or dest are
 *    NULL, and that dest is large enough to hold the entire contents
 *    of source. This function should be deprecated since pascal
 *    strings are obsolete on MacOSX and CFString should be used
 *    instead.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    Thread safe provided no other thread is modifying source.
 *  
 *  Parameters:
 *    
 *    dest:
 *      the destination pascal string
 *    
 *    source:
 *      the source pascal string
 *    
 *    num:
 *      the maximum number of bytes to copy
 *  
 *  Result:
 *    This function returns dest.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PLstrncpy( dest: StringPtr; const (*var*) source: Str255; num: SInt16 ): StringPtr; external name '_PLstrncpy';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PLstrcat()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use CFString instead.
 *  
 *  Summary:
 *    Append a pascal string to another pascal string
 *  
 *  Discussion:
 *    This function append a copy of the pascal string append to the
 *    end of the pascal string str.  If the length of str plus the
 *    length of append is greater than 255 ( the maximum size of a
 *    pascal string ) then only enough characters are copied to str to
 *    reach the 255 character limit, and the length of str is set to
 *    255.  The caller must ensure that neither str nor append are
 *    NULL, and that str is large enough to hold the entire contents of
 *    append. This function should be deprecated since pascal strings
 *    are obsolete on MacOSX and CFString should be used instead.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    Thread safe provided no other thread is modifying str or append.
 *  
 *  Parameters:
 *    
 *    str:
 *      the destination pascal string
 *    
 *    append:
 *      the pascal string to append
 *  
 *  Result:
 *    This function returns s.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PLstrcat( str: StringPtr; const (*var*) append: Str255 ): StringPtr; external name '_PLstrcat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PLstrncat()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use CFString instead.
 *  
 *  Summary:
 *    Append up to num bytes of a pascal string to another pascal string
 *  
 *  Discussion:
 *    This function append up to the first num bytes of the pascal
 *    string append to the end of the pascal string s.  If the length
 *    of str plus the length of append is greater than 255 ( the
 *    maximum size of a pascal string ) then only enough characters are
 *    copied to str to reach the 255 character limit, and the length of
 *    str is set to 255.  The caller must ensure that neither str nor
 *    append are NULL, and that str is large enough to hold the entire
 *    contents of append. This function should be deprecated since
 *    pascal strings are obsolete on MacOSX and CFString should be used
 *    instead.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    Thread safe provided no other thread is modifying str1 or append.
 *  
 *  Parameters:
 *    
 *    str1:
 *      the destination pascal string
 *    
 *    append:
 *      the pascal string to append
 *    
 *    num:
 *      the maximum number of bytes of append to append onto s
 *  
 *  Result:
 *    This function returns str.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PLstrncat( str1: StringPtr; const (*var*) append: Str255; num: SInt16 ): StringPtr; external name '_PLstrncat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PLstrchr()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use CFString instead.
 *  
 *  Summary:
 *    Return a pointer to the first occurrence of ch1 in str.
 *  
 *  Discussion:
 *    The PLstrrchr() function locates the first occurrence of ch1
 *    (converted to an unsigned char) in the string s.  If ch1 does not
 *    occur in the string, this returns NULL. This function should be
 *    deprecated since pascal strings are obsolete on MacOSX and
 *    CFString should be used instead.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    Thread safe provided no other thread is modifying str1.
 *  
 *  Parameters:
 *    
 *    str1:
 *      the pascal string
 *    
 *    ch1:
 *      the character to find
 *  
 *  Result:
 *    A pointer to the first occurrence of ch1 in str1, or NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PLstrchr( const (*var*) str1: Str255; ch1: SInt16 ): Ptr; external name '_PLstrchr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PLstrrchr()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use CFString instead.
 *  
 *  Summary:
 *    Return a pointer to the last occurrence of ch1 in str.
 *  
 *  Discussion:
 *    The PLstrrchr() function locates the last occurrence of ch1
 *    (converted to an unsigned char) in the string s.  If ch1 does not
 *    occur in the string, this returns NULL. This function should be
 *    deprecated since pascal strings are obsolete on MacOSX and
 *    CFString should be used instead.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    Thread safe provided no other thread is modifying str1.
 *  
 *  Parameters:
 *    
 *    str1:
 *      the pascal string
 *    
 *    ch1:
 *      the character to find
 *  
 *  Result:
 *    A pointer to the last occurrence of ch1 in str1, or NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PLstrrchr( const (*var*) str1: Str255; ch1: SInt16 ): Ptr; external name '_PLstrrchr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PLstrpbrk()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use CFString instead.
 *  
 *  Summary:
 *    Return a pointer to the first occurrence in str of any character
 *    in charSet.
 *  
 *  Discussion:
 *    The PLstrpbrk() function returns a pointer to the first
 *    occurrence in str of any character in searchStr.  If none of the
 *    characters in searchStr can be found in str, then NULL is
 *    returned. This function should be deprecated since pascal strings
 *    are obsolete on MacOSX and CFString should be used instead.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    Thread safe provided no other thread is modifying str1 or charSet.
 *  
 *  Parameters:
 *    
 *    str1:
 *      the pascal string
 *    
 *    charSet:
 *      the character to find
 *  
 *  Result:
 *    A pointer to the first occurrence of any character in charSet in
 *    str1, or NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PLstrpbrk( const (*var*) str1: Str255; const (*var*) charSet: Str255 ): Ptr; external name '_PLstrpbrk';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PLstrspn()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use CFString instead.
 *  
 *  Summary:
 *    Spans the initial part of str1 as long as the characters from
 *    str1 occur in string charset
 *  
 *  Discussion:
 *    The PLstrspn() function spans the initial part of the pascal
 *    string str1 as long as the characters from s occur in string
 *    charset. In effect, this returns a count of the number of
 *    characters at the beginning of the pascal string str1 which are
 *    in charset. This function should be deprecated since pascal
 *    strings are obsolete on MacOSX and CFString should be used
 *    instead.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    Thread safe provided no other thread is modifying str1 or charSet.
 *  
 *  Parameters:
 *    
 *    str1:
 *      the pascal string
 *    
 *    charSet:
 *      the character to find
 *  
 *  Result:
 *    The count of characters at the beginning of str1 which are in
 *    charSet.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PLstrspn( const (*var*) str1: Str255; const (*var*) charSet: Str255 ): SInt16; external name '_PLstrspn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PLstrstr()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use CFString instead.
 *  
 *  Summary:
 *    Returns a pointer to the first occurrence of searchStr in str1
 *  
 *  Discussion:
 *    The PLstrstr() function returns a pointer to the first occurrence
 *    of searchStr in str1, or NULL if searchStr does not exist in
 *    str1. This function should be deprecated since pascal strings are
 *    obsolete on MacOSX and CFString should be used instead.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    Thread safe provided no other thread is modifying str1 or
 *    searchStr.
 *  
 *  Parameters:
 *    
 *    str1:
 *      the pascal string
 *    
 *    searchStr:
 *      the string to find
 *  
 *  Result:
 *    The count of characters at the beginning of str1 which are in
 *    charSet.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PLstrstr( const (*var*) str1: Str255; const (*var*) searchStr: Str255 ): Ptr; external name '_PLstrstr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PLstrlen()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use CFString instead.
 *  
 *  Summary:
 *    Returns the length of the pascal string
 *  
 *  Discussion:
 *    The PLstrlen() function returns the length of the pascal string
 *    str. This function should be deprecated since pascal strings are
 *    obsolete on MacOSX and CFString should be used instead.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    Thread safe provided no other thread is modifying str.
 *  
 *  Parameters:
 *    
 *    str:
 *      the pascal string
 *  
 *  Result:
 *    The length of the pascal string str.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PLstrlen( const (*var*) str: Str255 ): SInt16; external name '_PLstrlen';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PLpos()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    use CFString instead.
 *  
 *  Summary:
 *    Returns the offset to the first occurrence of searchStr in str1
 *  
 *  Discussion:
 *    The PLpos() function returns the offset of the string searchStr
 *    in str1, or 0 if searchStr does not occur in str1.  For example,
 *    if str1 is "\pHello World" and searchStr is "\pWorld", then this
 *    function will return the value 7. This function should be
 *    deprecated since pascal strings are obsolete on MacOSX and
 *    CFString should be used instead.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    Thread safe provided no other thread is modifying str1 or
 *    searchStr.
 *  
 *  Parameters:
 *    
 *    str1:
 *      the pascal string
 *    
 *    searchStr:
 *      the string to find
 *  
 *  Result:
 *    The count of characters at the beginning of str1 which are in
 *    charSet.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PLpos( const (*var*) str1: Str255; const (*var*) searchStr: Str255 ): SInt16; external name '_PLpos';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


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
