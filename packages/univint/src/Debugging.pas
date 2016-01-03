{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       CarbonCore/Debugging.h
 
     Contains:   Macros to handle exceptions and assertions.
                 The contents of this header file are deprecated.
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Copyright:  © 1989-2011 by Apple Inc. All rights reserved.
}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
     Version:    CarbonCore-859.2~1
 
     Copyright:  © 1989-2008 by Apple Computer, Inc., all rights reserved.
=======
=======
>>>>>>> origin/fixes_2_2
     File:       Debugging.p
=======
     File:       CarbonCore/Debugging.h
>>>>>>> origin/fixes_2.4
 
     Contains:   Macros to handle exceptions and assertions.
 
     Version:    CarbonCore-859.2~1
 
<<<<<<< HEAD
     Copyright:  © 1989-2002 by Apple Computer, Inc., all rights reserved.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  © 1989-2008 by Apple Computer, Inc., all rights reserved.
>>>>>>> origin/fixes_2.4
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
                     http://www.freepascal.org/bugs.html
 
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{    Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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
{    Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit Debugging;
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
>>>>>>> graemeg/cpstrnew
=======
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
=======
>>>>>>> origin/cpstrnew
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
<<<<<<< HEAD
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
=======
{$elsec}
>>>>>>> graemeg/cpstrnew
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
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
uses MacTypes;
{$endc} {not MACOSALLINCLUDE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/cpstrnew

=======
>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}
=======
>>>>>>> graemeg/cpstrnew

{$ALIGN POWER}

{$ifc TARGET_OS_MAC}
=======
>>>>>>> graemeg/cpstrnew

{$ALIGN POWER}

{$ifc TARGET_OS_MAC}

{$ALIGN POWER}

{$ifc TARGET_OS_MAC}

{$ALIGN POWER}

{$ifc TARGET_OS_MAC}

{$ALIGN POWER}

{
    This file supplies standard debugging routines and macros to Carbon and
    Classic Mac OS programs. Other C programs which wish to use the
    exception handling and assertion macros should include AssertMacros.h
    instead of this file.

    To activate debugger breaks, #define DEBUG to 1 (one) before including this
    file. Five further levels of debugging are available, selected by #defining
    one of the following conditionals to 1 after DEBUG is defined to 1.

        DEBUG_INTERNAL      the default; asserts include file and line number
                            information

        DEBUG_EXTERNAL      used for code which must ship to developers outside
                            your organization; no file or line number
                            information is included in asserts

        DEBUG_BREAK_ONLY    where an assertion string would normally be sent to
                            the debugger; call Debugger() instead.

        PRODUCTION          used for shipping code; no debugger breaks are
                            emitted

        PERFORMANCE         same as PRODUCTION

    #defining DEBUG to 0 is equivalent to #defining PRODUCTION 1 when
    DEBUG is 1. (No code for debugger breaks is emitted in either case.)
    
    #defining DEBUG to 1 without specifying a level is equivalent to #defining
    DEBUG_INTERNAL 1.

    In addition, these macros should also be #defined (they are described in detail below):
        kComponentSignatureString
        COMPONENT_SIGNATURE
}
{
 *  Before including this file, #define kComponentSignatureString to a C-string
 *  containing the name of your client and #define COMPONENT_SIGNATURE to your
 *  client's unique signature (i.e., your program's registered creator type).
 *  For example:
 *
 *      #define kComponentSignatureString "SurfWriter"
 *      #define COMPONENT_SIGNATURE 'WAVE'
 *
 *  If you don't define kComponentSignatureString and COMPONENT_SIGNATURE, the
 *  default kComponentSignatureString and COMPONENT_SIGNATURE values will be
 *  used by the DEBUGASSERTMSG macros below.
 }

{
 *  The DEBUGLEVEL's
 }
const
	DEBUG_LEVEL_PRODUCTION = 0;
const
	DEBUG_LEVEL_BREAK_ONLY = 1;
const
	DEBUG_LEVEL_EXTERNAL = 3;
const
	DEBUG_LEVEL_INTERNAL = 4;
	DEBUGFULL = DEBUG_LEVEL_INTERNAL;


{
 * define DEBUGLEVEL
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew

{
 *  The options parameter to DebugAssert and DEBUGASSERTMSG is currently reserved (must be zero).
 }
const
	DEBUG_NO_OPTIONS = 0;
<<<<<<< HEAD

{
=======

{
 *  The options parameter to DebugAssert and DEBUGASSERTMSG is currently reserved (must be zero).
 }
const
	DEBUG_NO_OPTIONS = 0;

{
>>>>>>> graemeg/cpstrnew
 *  DEBUGASSERTMSG()
 *
 *  Summary:
 *    All error reporting is routed through this macro, which calls the system
 *    routine DebugAssert(). If you wish to use your own assertion break
 *    routine, you can override DEBUGASSERTMSG by defining it before including
 *    this file.
 *
 *  Parameters:
 *
 *    componentSignature:
 *      The unique signature of component causing the assertion.
 *    
 *    options:
 *      reserved.
 *    
 *    assertionString:
 *      A pointer to a string constant containing the assertion.
 *      This must be a string constant (and not a string variable or
 *      NULL) because the Proeprocessor concatenates it with other
 *      string constants
 *    
 *    exceptionLabelString:
 *      A pointer to a string containing the exceptionLabel, or NULL.
 *    
 *    errorString:
 *      A pointer to the error string, or NULL. DEBUGASSERTMSG macros
 *      must not attempt to concatenate this string with constant
 *      character strings.
 *    
 *    fileName:
 *      A pointer to the fileName or pathname (generated by the
 *      preprocessor __FILE__ identifier), or NULL.
 *    
 *    lineNumber:
 *      The line number in the file (generated by the preprocessor
 *      __LINE__ identifier), or 0 (zero).
 *    
 *    value:
 *      A value associated with the assertion, or NULL.
 }
<<<<<<< HEAD

{
 *  Define the three inputs to AssertMacros.h
 }

{
 *  Include AssertMacros.h where all of the check, verify, and require macros are defined
 }
{
 *  The following check, verify, and require macros assert that TaskLevel is 0.
 }

{
 *  You can use DPRINTF as a dprintf which goes away in production builds.
 *  DPRINTF is not supported by Carbon because dprintf
 *  is not supported by Carbon.
 *
 *  To use it, double-parenthesize the argument - i.e., use:
 *
 *      DPRINTF(("formatString %d", 5 ));
 *
 *  This is sadly necessary because a macro can not have a variable number
 *  of arguments.
 *
 *  DPRINTF is defined only if it is not already defined to
 *  prevent conflicts with developer code.
 }

{
=======

{
 *  Define the three inputs to AssertMacros.h
 }

{
 *  Include AssertMacros.h where all of the check, verify, and require macros are defined
 }
{
 *  The following check, verify, and require macros assert that TaskLevel is 0.
 }

{
 *  You can use DPRINTF as a dprintf which goes away in production builds.
 *  DPRINTF is not supported by Carbon because dprintf
 *  is not supported by Carbon.
 *
 *  To use it, double-parenthesize the argument - i.e., use:
 *
 *      DPRINTF(("formatString %d", 5 ));
 *
 *  This is sadly necessary because a macro can not have a variable number
 *  of arguments.
 *
 *  DPRINTF is defined only if it is not already defined to
 *  prevent conflicts with developer code.
 }

{
>>>>>>> graemeg/cpstrnew
=======

{
 *  DEBUGASSERTMSG()
 *
 *  Summary:
 *    All error reporting is routed through this macro, which calls the system
 *    routine DebugAssert(). If you wish to use your own assertion break
 *    routine, you can override DEBUGASSERTMSG by defining it before including
 *    this file.
 *
 *  Parameters:
 *
 *    componentSignature:
 *      The unique signature of component causing the assertion.
 *    
 *    options:
 *      reserved.
 *    
 *    assertionString:
 *      A pointer to a string constant containing the assertion.
 *      This must be a string constant (and not a string variable or
 *      NULL) because the Proeprocessor concatenates it with other
 *      string constants
 *    
 *    exceptionLabelString:
 *      A pointer to a string containing the exceptionLabel, or NULL.
 *    
 *    errorString:
 *      A pointer to the error string, or NULL. DEBUGASSERTMSG macros
 *      must not attempt to concatenate this string with constant
 *      character strings.
 *    
 *    fileName:
 *      A pointer to the fileName or pathname (generated by the
 *      preprocessor __FILE__ identifier), or NULL.
 *    
 *    lineNumber:
 *      The line number in the file (generated by the preprocessor
 *      __LINE__ identifier), or 0 (zero).
 *    
 *    value:
 *      A value associated with the assertion, or NULL.
 }

{
 *  Define the three inputs to AssertMacros.h
 }

{
=======

{
 *  The options parameter to DebugAssert and DEBUGASSERTMSG is currently reserved (must be zero).
 }
const
	DEBUG_NO_OPTIONS = 0;

{
 *  DEBUGASSERTMSG()
 *
 *  Summary:
 *    All error reporting is routed through this macro, which calls the system
 *    routine DebugAssert(). If you wish to use your own assertion break
 *    routine, you can override DEBUGASSERTMSG by defining it before including
 *    this file.
 *
 *  Parameters:
 *
 *    componentSignature:
 *      The unique signature of component causing the assertion.
 *    
 *    options:
 *      reserved.
 *    
 *    assertionString:
 *      A pointer to a string constant containing the assertion.
 *      This must be a string constant (and not a string variable or
 *      NULL) because the Proeprocessor concatenates it with other
 *      string constants
 *    
 *    exceptionLabelString:
 *      A pointer to a string containing the exceptionLabel, or NULL.
 *    
 *    errorString:
 *      A pointer to the error string, or NULL. DEBUGASSERTMSG macros
 *      must not attempt to concatenate this string with constant
 *      character strings.
 *    
 *    fileName:
 *      A pointer to the fileName or pathname (generated by the
 *      preprocessor __FILE__ identifier), or NULL.
 *    
 *    lineNumber:
 *      The line number in the file (generated by the preprocessor
 *      __LINE__ identifier), or 0 (zero).
 *    
 *    value:
 *      A value associated with the assertion, or NULL.
 }

{
 *  Define the three inputs to AssertMacros.h
 }

{
>>>>>>> graemeg/cpstrnew
 *  Include AssertMacros.h where all of the check, verify, and require macros are defined
 }
{
 *  The following check, verify, and require macros assert that TaskLevel is 0.
 }
<<<<<<< HEAD

{
 *  You can use DPRINTF as a dprintf which goes away in production builds.
 *  DPRINTF is not supported by Carbon because dprintf
 *  is not supported by Carbon.
 *
 *  To use it, double-parenthesize the argument - i.e., use:
 *
 *      DPRINTF(("formatString %d", 5 ));
 *
 *  This is sadly necessary because a macro can not have a variable number
 *  of arguments.
 *
 *  DPRINTF is defined only if it is not already defined to
 *  prevent conflicts with developer code.
 }

{
>>>>>>> graemeg/cpstrnew
=======

{
 *  You can use DPRINTF as a dprintf which goes away in production builds.
 *  DPRINTF is not supported by Carbon because dprintf
 *  is not supported by Carbon.
 *
 *  To use it, double-parenthesize the argument - i.e., use:
 *
 *      DPRINTF(("formatString %d", 5 ));
 *
 *  This is sadly necessary because a macro can not have a variable number
 *  of arguments.
 *
 *  DPRINTF is defined only if it is not already defined to
 *  prevent conflicts with developer code.
 }

{
>>>>>>> graemeg/cpstrnew
=======

{
 *  The options parameter to DebugAssert and DEBUGASSERTMSG is currently reserved (must be zero).
 }
const
	DEBUG_NO_OPTIONS = 0;

{
 *  DEBUGASSERTMSG()
 *
 *  Summary:
 *    All error reporting is routed through this macro, which calls the system
 *    routine DebugAssert(). If you wish to use your own assertion break
 *    routine, you can override DEBUGASSERTMSG by defining it before including
 *    this file.
 *
 *  Parameters:
 *
 *    componentSignature:
 *      The unique signature of component causing the assertion.
 *    
 *    options:
 *      reserved.
 *    
 *    assertionString:
 *      A pointer to a string constant containing the assertion.
 *      This must be a string constant (and not a string variable or
 *      NULL) because the Proeprocessor concatenates it with other
 *      string constants
 *    
 *    exceptionLabelString:
 *      A pointer to a string containing the exceptionLabel, or NULL.
 *    
 *    errorString:
 *      A pointer to the error string, or NULL. DEBUGASSERTMSG macros
 *      must not attempt to concatenate this string with constant
 *      character strings.
 *    
 *    fileName:
 *      A pointer to the fileName or pathname (generated by the
 *      preprocessor __FILE__ identifier), or NULL.
 *    
 *    lineNumber:
 *      The line number in the file (generated by the preprocessor
 *      __LINE__ identifier), or 0 (zero).
 *    
 *    value:
 *      A value associated with the assertion, or NULL.
 }

{
 *  Define the three inputs to AssertMacros.h
 }

{
 *  Include AssertMacros.h where all of the check, verify, and require macros are defined
 }
{
 *  The following check, verify, and require macros assert that TaskLevel is 0.
 }

{
 *  You can use DPRINTF as a dprintf which goes away in production builds.
 *  DPRINTF is not supported by Carbon because dprintf
 *  is not supported by Carbon.
 *
 *  To use it, double-parenthesize the argument - i.e., use:
 *
 *      DPRINTF(("formatString %d", 5 ));
 *
 *  This is sadly necessary because a macro can not have a variable number
 *  of arguments.
 *
 *  DPRINTF is defined only if it is not already defined to
 *  prevent conflicts with developer code.
 }

{
>>>>>>> origin/cpstrnew
 *  kBlessedBusErrorBait is an address that will never be mapped by
 *  Mac OS 8 or 9. It is close to the middle of the 64K range from 0x68F10000
 *  to 0x68F1FFFF that is unmapped and cannot be accessed without causing an
 *  exception. Thus, it's a good value to use for filling uninitialized
 *  pointers, etc.
 *
 *  Mac OS X programs should NOT use kBlessedBusErrorBait and should use
 *  zero (0) instead, since by default, page 0 is read and write protected
 *  for user code.
 }
const
	kBlessedBusErrorBait = $68F168F1;
=======
=======
>>>>>>> origin/fixes_2_2
uses MacTypes,Files;

{
  ______________________________________________________________________________________
                                                                                        
    This file defines standard exception handling and assertion macros for              
    system-level programming in C.  Originally used in QuickDraw GX, and heavily        
    modified since, these macros are used extensively throughout Mac OS system          
    software.  Now *you* can look and feel like a system software engineer.             
                                                                                        
    To activate debugger breaks, #define DEBUG to 1 (one) before including this file.   
    Five further levels of debugging are available, selected by #defining one           
    of the following conditionals to 1 after DEBUG is defined to 1.                     
                                                                                        
        DEBUG_INTERNAL      the default; includes file and line number information      
                                                                                        
        DEBUG_EXTERNAL      used for code which must ship to developers outside         
                            your organization; no file or line number information is    
                            included in asserts                                         
                                                                                        
        DEBUG_BREAK_ONLY    where an assertion would normally be sent to the debugger,      
                            send an empty string instead.                               
                                                                                        
        PRODUCTION          used for shipping code; no debugger breaks are emitted      
                                                                                        
        PERFORMANCE         same as PRODUCTION                                          
                                                                                        
    #defining DEBUG to 0 is equivalent to #defining PRODUCTION 1 when DEBUG is 1.       
    (No code for debugger breaks is emitted in either case.)                            
                                                                                        
    Of the multitude of macros, the preferred ones are:                                 
                                                                                        
    debug_string(c-string)                                                              
        If debugging is on, c-string is printed in the debugger.                        
        In production builds, debug_string() does nothing.                              
                                                                                        
    check(expression)                                                                   
    check_noerr(error)                                                                  
        If (expression) evaluates to false, break into the debugger.                    
        In production builds, check() does nothing.                                     
        Code inside check() statements is not compiled into production builds.          
                                                                                        
    require(expression, label)                                                          
    require_noerr(expression, label)                                                    
        If (expression) evaluates to false, announce this fact via the                  
        debugger and then goto label.  In production builds, does not call              
        the debugger but still goes to label if expression is false.                    
                                                                                        
    require_action(expression, label, action)                                           
    require_noerr_action(expression, label, action)                                     
        Same as require, but executes (action) before jumping to label.                 
                                                                                        
    check_string(expression, c-string)                                                  
    require_string(expression, label, c-string)                                         
    require_noerr_string(expression, label, c-string)                                   
        If expression evaluates to false, print string and then proceed as in           
        a normal check/require statement                                                
                                                                                        
    verify(expression)                                                                  
    verify_noerr(error)                                                                 
        If debugging is on, verify is the same as check(expression).                    
        If debugging is off, verify still evaluates (expression)                        
        but ignores the result.  Code inside verify() statements                        
        is executed in both production and debug builds.                                
                                                                                        
    Common usage:                                                                       
                                                                                        
        // my pixmap is not purgeable, so locking it should never fail                  
        verify( LockPixels(myPixMap) );                                                 
        verify_noerr( DisposeThread(myThread, &threadResult, true) );                   
  ______________________________________________________________________________________
}
=======
uses MacTypes;
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}

{
    This file supplies standard debugging routines and macros to Carbon and
    Classic Mac OS programs. Other C programs which wish to use the
    exception handling and assertion macros should include AssertMacros.h
    instead of this file.

    To activate debugger breaks, #define DEBUG to 1 (one) before including this
    file. Five further levels of debugging are available, selected by #defining
    one of the following conditionals to 1 after DEBUG is defined to 1.

        DEBUG_INTERNAL      the default; asserts include file and line number
                            information

        DEBUG_EXTERNAL      used for code which must ship to developers outside
                            your organization; no file or line number
                            information is included in asserts

        DEBUG_BREAK_ONLY    where an assertion string would normally be sent to
                            the debugger; call Debugger() instead.

        PRODUCTION          used for shipping code; no debugger breaks are
                            emitted

        PERFORMANCE         same as PRODUCTION

    #defining DEBUG to 0 is equivalent to #defining PRODUCTION 1 when
    DEBUG is 1. (No code for debugger breaks is emitted in either case.)
    
    #defining DEBUG to 1 without specifying a level is equivalent to #defining
    DEBUG_INTERNAL 1.

    In addition, these macros should also be #defined (they are described in detail below):
        kComponentSignatureString
        COMPONENT_SIGNATURE
}
{
 *  Before including this file, #define kComponentSignatureString to a C-string
 *  containing the name of your client and #define COMPONENT_SIGNATURE to your
 *  client's unique signature (i.e., your program's registered creator type).
 *  For example:
 *
 *      #define kComponentSignatureString "SurfWriter"
 *      #define COMPONENT_SIGNATURE 'WAVE'
 *
 *  If you don't define kComponentSignatureString and COMPONENT_SIGNATURE, the
 *  default kComponentSignatureString and COMPONENT_SIGNATURE values will be
 *  used by the DEBUGASSERTMSG macros below.
 }

{
 *  The DEBUGLEVEL's
 }
const
	DEBUG_LEVEL_PRODUCTION = 0;
const
	DEBUG_LEVEL_BREAK_ONLY = 1;
const
	DEBUG_LEVEL_EXTERNAL = 3;
const
	DEBUG_LEVEL_INTERNAL = 4;
	DEBUGFULL = DEBUG_LEVEL_INTERNAL;


{
 * define DEBUGLEVEL
 }

{
 *  The options parameter to DebugAssert and DEBUGASSERTMSG is currently reserved (must be zero).
 }
const
	DEBUG_NO_OPTIONS = 0;

{
 *  DEBUGASSERTMSG()
 *
 *  Summary:
 *    All error reporting is routed through this macro, which calls the system
 *    routine DebugAssert(). If you wish to use your own assertion break
 *    routine, you can override DEBUGASSERTMSG by defining it before including
 *    this file.
 *
 *  Parameters:
 *
 *    componentSignature:
 *      The unique signature of component causing the assertion.
 *    
 *    options:
 *      reserved.
 *    
 *    assertionString:
 *      A pointer to a string constant containing the assertion.
 *      This must be a string constant (and not a string variable or
 *      NULL) because the Proeprocessor concatenates it with other
 *      string constants
 *    
 *    exceptionLabelString:
 *      A pointer to a string containing the exceptionLabel, or NULL.
 *    
 *    errorString:
 *      A pointer to the error string, or NULL. DEBUGASSERTMSG macros
 *      must not attempt to concatenate this string with constant
 *      character strings.
 *    
 *    fileName:
 *      A pointer to the fileName or pathname (generated by the
 *      preprocessor __FILE__ identifier), or NULL.
 *    
 *    lineNumber:
 *      The line number in the file (generated by the preprocessor
 *      __LINE__ identifier), or 0 (zero).
 *    
 *    value:
 *      A value associated with the assertion, or NULL.
 }

{
 *  Define the three inputs to AssertMacros.h
 }

{
 *  Include AssertMacros.h where all of the check, verify, and require macros are defined
 }
{
 *  The following check, verify, and require macros assert that TaskLevel is 0.
 }

{
 *  You can use DPRINTF as a dprintf which goes away in production builds.
 *  DPRINTF is not supported by Carbon because dprintf
 *  is not supported by Carbon.
 *
 *  To use it, double-parenthesize the argument - i.e., use:
 *
 *      DPRINTF(("formatString %d", 5 ));
 *
 *  This is sadly necessary because a macro can not have a variable number
 *  of arguments.
 *
 *  DPRINTF is defined only if it is not already defined to
 *  prevent conflicts with developer code.
 }

<<<<<<< HEAD
{$ALIGN MAC68K}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{
 *  kBlessedBusErrorBait is an address that will never be mapped by
 *  Mac OS 8 or 9. It is close to the middle of the 64K range from 0x68F10000
 *  to 0x68F1FFFF that is unmapped and cannot be accessed without causing an
 *  exception. Thus, it's a good value to use for filling uninitialized
 *  pointers, etc.
 *
 *  Mac OS X programs should NOT use kBlessedBusErrorBait and should use
 *  zero (0) instead, since by default, page 0 is read and write protected
 *  for user code.
 }
const
	kBlessedBusErrorBait = $68F168F1;
>>>>>>> origin/fixes_2.4

{
 *  DebugAssert()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  Summary:
 *    DebugAssert is the system routine that the DEBUGASSERTMSG macro
 *    calls (by default) to display assertion messsages. The output
 *    from DebugAssert can be redirected by installing a
 *    DebugAssertOutputHandler with InstallDebugAssertOutputHandler.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *    This call is thread safe if no debug components are registered,
 *    and may be thread unsafe if there are debug components being
 *    installed or removed.
 *  
 *  Parameters:
 *    
 *    componentSignature:
 *      The unique signature of component causing the assertion.
 *    
 *    options:
 *      reserved.
 *    
 *    assertionString:
 *      A pointer to a string containing the assertion, or NULL.
 *    
 *    exceptionLabelString:
 *      A pointer to a string containing the exceptionLabel, or NULL.
 *    
 *    errorString:
 *      A pointer to the error string, or NULL.
 *    
 *    fileName:
 *      A pointer to the fileName or pathname (generated by the
 *      preprocessor __FILE__ identifier), or NULL.
 *    
 *    lineNumber:
 *      The line number in the file (generated by the preprocessor
 *      __LINE__ identifier), or 0 (zero).
 *    
 *    value:
 *      A value associated with the assertion, or NULL.
 *  
<<<<<<< HEAD
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DebugLib 1.0 and later
 }
procedure DebugAssert( componentSignature: OSType; options: UInt32; assertionString: ConstCStringPtr; exceptionLabelString: ConstCStringPtr; errorString: ConstCStringPtr; fileName: ConstCStringPtr; lineNumber: SIGNEDLONG; value: UnivPtr ); external name '_DebugAssert';
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
 *  TaskLevel masks
 }
const
	k68kInterruptLevelMask = $00000007; { 68K interrupt levels 0 through 7 }
	kInVBLTaskMask = $00000010; { VBLs are executing }
	kInDeferredTaskMask = $00000020; { Deferred tasks are executing }
	kInSecondaryIntHandlerMask = $00000040; { Secondary interrupt handlers are executing }
	kInNestedInterruptMask = $00000080; { The system is handling an interrupt }

{
 *  TaskLevel()
 *  
 *  Summary:
 *    TaskLevel returns 0 if we're (probably) running at non-interrupt
 *    time. There's no way to make this perfect, but this is as close
 *    as we can get. If TaskLevel doesn't return 0, then one of the
 *    TaskLevel masks can be used to learn more. Mac OS X has no
 *    concept of "TaskLevel"; and so it will always return the value 0.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Result:
 *    The current task level.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DebugLib 1.0 and later
 }
function TaskLevel: UInt32; external name '_TaskLevel';
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
 *  Constants used by the DebugComponent functions
 }
const
	kComponentDebugOption = 0;     { optionSelectorNum to turn breaks for component On or Off}

const
	kGetDebugOption = 1;    { get current debug option setting}
	kSetDebugOption = 2;     { set debug option}


{
 *  DebugComponentCallbackProcPtr
 *  
 *  Discussion:
 *    DebugComponentCallback is the callback into a component that
 *    registers with DebugLib. It is called to get the debug option
 *    setting, or to turn a debug option on or off.
 *  
 *  Parameters:
 *    
 *    optionSelectorNum:
 *      The component debug option to set.
 *    
 *    command:
 *      The command the DebugComponentCallbackProc must handle:
 *       kGetDebugOption - get current debug option setting
 *       kSetDebugOption - set debug option
 *    
 *    optionSetting:
 *      A pointer to a Boolean where the DebugComponentCallbackProc
 *      must return the option setting: the current setting if command
 *      is kGetDebugOption; the new debug option if command is 
 *      kSetDebugOption
 }
type
	DebugComponentCallbackProcPtr = procedure( optionSelectorNum: SInt32; command: UInt32; var optionSetting: Boolean );
	DebugComponentCallbackUPP = DebugComponentCallbackProcPtr;
{
 *  NewDebugComponent()
 *  
 *  Summary:
 *    NewDebugComponent registers a component with DebugLib.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    componentSignature:
 *      The unique signature of component.
 *    
 *    componentName:
 *      The displayable string naming the component.
 *    
 *    componentCallback:
 *      The callback into component for working with options.
 *  
 *  Result:
 *    An operating system result code: noErr, memFullErr,
 *    debuggingExecutionContextErr, debuggingDuplicateSignatureErr, or
 *    debuggingInvalidNameErr.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DebugLib 1.0 and later
 }
function NewDebugComponent( componentSignature: OSType; const (*var*) componentName: Str255; componentCallback: DebugComponentCallbackUPP ): OSStatus; external name '_NewDebugComponent';
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
 *  NewDebugOption()
 *  
 *  Summary:
 *    NewDebugOption registers a debug option with DebugLib.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    componentSignature:
 *      The signature of component to register a debug option for.
 *    
 *    optionSelectorNum:
 *      The selector number of this debug option.
 *    
 *    optionName:
 *      The displayable string naming this debug option.
 *  
 *  Result:
 *    An operating system result code: noErr, memFullErr,
 *    debuggingExecutionContextErr, debuggingDuplicateOptionErr,
 *    debuggingInvalidSignatureErr, debuggingInvalidNameErr, or
 *    debuggingNoCallbackErr.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DebugLib 1.0 and later
 }
function NewDebugOption( componentSignature: OSType; optionSelectorNum: SInt32; const (*var*) optionName: Str255 ): OSStatus; external name '_NewDebugOption';
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
 *  DisposeDebugComponent()
 *  
 *  Summary:
 *    DisposeDebugComponent removes a component registration and all
 *    related debug options from DebugLib.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    componentSignature:
 *      The unique signature of a component.
 *  
 *  Result:
 *    An operating system result code: noErr,
 *    debuggingExecutionContextErr, or debuggingInvalidSignatureErr.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DebugLib 1.0 and later
 }
function DisposeDebugComponent( componentSignature: OSType ): OSStatus; external name '_DisposeDebugComponent';
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
 *  GetDebugComponentInfo()
 *  
 *  Summary:
 *    GetDebugComponentInfo returns a component registered with
 *    DebugLib.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    itemIndex:
 *      The index into the list of registered components (1-based).
 *    
 *    componentSignature:
 *      A pointer to an OSType where the unique signature of a
 *      component is returned.
 *    
 *    componentName:
 *      A pointer to an Str255 where the displayable string naming a
 *      component is returned.
 *  
 *  Result:
 *    An operating system result code: noErr or debuggingNoMatchErr.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DebugLib 1.0 and later
 }
function GetDebugComponentInfo( itemIndex: UInt32; var componentSignature: OSType; var componentName: Str255 ): OSStatus; external name '_GetDebugComponentInfo';
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
 *  GetDebugOptionInfo()
 *  
 *  Summary:
 *    GetDebugOptionInfo returns a debug option registered with
 *    DebugLib.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    itemIndex:
 *      The index into the list of registered debug options (0-based);
 *      0 = kComponentDebugOption.
 *    
 *    componentSignature:
 *      The unique signature of a component.
 *    
 *    optionSelectorNum:
 *      A pointer to an SInt32 where the selector number of this debug
 *      option is returned.
 *    
 *    optionName:
 *      A pointer to an Str255 where the displayable string naming this
 *      debug option is returned.
 *    
 *    optionSetting:
 *      A pointer to an Boolean where the current debug option setting
 *      is returned.
 *  
 *  Result:
 *    An operating system result code: noErr,
 *    debuggingInvalidSignatureErr, or debuggingNoMatchErr.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DebugLib 1.0 and later
 }
function GetDebugOptionInfo( itemIndex: UInt32; componentSignature: OSType; var optionSelectorNum: SInt32; var optionName: Str255; var optionSetting: Boolean ): OSStatus; external name '_GetDebugOptionInfo';
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
 *  SetDebugOptionValue()
 *  
 *  Summary:
 *    SetDebugOptionValue sets a debug option registered with DebugLib.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    componentSignature:
 *      The unique signature of a component.
 *    
 *    optionSelectorNum:
 *      The selector number of this debug option.
 *    
 *    newOptionSetting:
 *      The new debug option setting.
 *  
 *  Result:
 *    An operating system result code: noErr,
 *    debuggingInvalidSignatureErr, or debuggingInvalidOptionErr.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DebugLib 1.0 and later
 }
function SetDebugOptionValue( componentSignature: OSType; optionSelectorNum: SInt32; newOptionSetting: Boolean ): OSStatus; external name '_SetDebugOptionValue';
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
 *  DebugAssertOutputHandlerProcPtr
 *  
 *  Discussion:
 *    DebugAssertOutputHandler is the callback that registers with
 *    DebugLib to handle the output from DebugAssert. The
 *    "componentSignature" through "value" parameters are the raw
 *    values passed to DebugAssert when an exception occurs.
 *  
 *  Parameters:
 *    
 *    componentSignature:
 *      The unique signature of component causing the assertion.
 *    
 *    options:
 *      reserved.
 *    
 *    assertionString:
 *      A pointer to a string containing the assertion, or NULL.
 *    
 *    exceptionLabelString:
 *      A pointer to a string containing the exceptionLabel, or NULL.
 *    
 *    errorString:
 *      A pointer to the error string, or NULL.
 *    
 *    fileName:
 *      A pointer to the fileName or pathname (generated by the
 *      preprocessor __FILE__ identifier), or NULL.
 *    
 *    lineNumber:
 *      The line number in the file (generated by the preprocessor
 *      __LINE__ identifier), or 0 (zero).
 *    
 *    value:
 *      A value associated with the assertion, or NULL.
 *    
 *    outputMsg:
 *      The string DebugAssert build which would normally be passed to
 *      DebugStr if a DebugAssertOutputHandler isn't installed.
 }
type
	DebugAssertOutputHandlerProcPtr = procedure( componentSignature: OSType; options: UInt32; assertionString: ConstCStringPtr; exceptionLabelString: ConstCStringPtr; errorString: ConstCStringPtr; fileName: ConstCStringPtr; lineNumber: SIGNEDLONG; value: UnivPtr; const (*var*) outputMsg: Str255 );
	DebugAssertOutputHandlerUPP = DebugAssertOutputHandlerProcPtr;
{
 *  InstallDebugAssertOutputHandler()
 *  
 *  Summary:
 *    InstallDebugAssertOutputHandler installs a
 *    DebugAssertOutputHandler which DebugAssert calls instead of
 *    DebugStr.
 *  
 *  Parameters:
 *    
 *    handler:
 *      The DebugAssertOutputHandler to install or NULL to switch back
 *      to the default handler (DebugStr).
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DebugLib 1.0 and later
 }
procedure InstallDebugAssertOutputHandler( handler: DebugAssertOutputHandlerUPP ); external name '_InstallDebugAssertOutputHandler';
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
 *  dprintf()
 *  
 *  Summary:
 *    printf takes a variable argument list and 'prints' that to the
 *    debugging output handler.  Calling dprintf() from anything but C
 *    or C++ is tricky.
 *  
 *  Parameters:
 *    
 *    format:
 *      The format string.
 *    
 *    ...:
 *      The arguments to the format string.
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DebugLib 1.1 and later
 }


{
 *  vdprintf()
 *  
 *  Summary:
 *    vdprintf takes a va_args list and 'prints' that to the debugging
 *    output handler.
 *  
 *  Parameters:
 *    
 *    format:
 *      The format string.
 *    
 *    va_args_list:
 *      The va_args list.
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DebugLib 1.1 and later
 }


{
 *  GetMacOSStatusErrorString()
 *  
 *  Summary:
 *    Returns a const char* string which corresponds to the textual
 *    constant for the given OSStatus code.
 *  
 *  Discussion:
 *    This function returns a text string which corresponds to the
 *    given OSStatus code, based on the errors in MacErrors.h.  For
 *    example, GetMacOSStatusErrorString( -43 ) returns "fnfErr", which
 *    is the text representation for the error constant -43.  This
 *    function is useful if you want to get or print out ( for
 *    debugging purposes only ) a useful description for a given
 *    OSStatus error.  If no string is available for the given
 *    constant, then the empty string "" is returned. Some error values
 *    have multiple meanings; in those cases the multiple meanings are
 *    all returned seperated by '/'es.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    err:
 *      The OSStatus to return a text string for.
 *  
 *  Result:
 *    A const char* string corresponding to the given OSStatus
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function GetMacOSStatusErrorString( err: OSStatus ): CStringPtr; external name '_GetMacOSStatusErrorString';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_4, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/cpstrnew


{
 *  GetMacOSStatusCommentString()
 *  
 *  Summary:
 *    Returns a const char* string which corresponds to the descriptive
 *    string for the given OSStatus code.
 *  
 *  Discussion:
 *    This function returns a text string which corresponds to a
 *    comment for the given OSStatus code, based on the errors in
 *    MacErrors.h.  For example, GetMacOSStatusConstantString( -43 )
 *    returns "File not found", which is the text representation for
 *    the error constant -43.  This function is useful if you want to
 *    get or print out ( for debugging purposes only ) a useful
 *    description for a given OSStatus error.  If no string is
 *    available for the given constant, then the empty string "" is
 *    returned. If no string is available for the given constant, then
 *    the empty string "" is returned.  Some error values have multiple
 *    meanings; in those cases the multiple meanings are all returned
 *    seperated by '/'es.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    err:
 *      The OSStatus to return a text string for.
 *  
 *  Result:
 *    A const char* string corresponding to the given OSStatus
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function GetMacOSStatusCommentString( err: OSStatus ): CStringPtr; external name '_GetMacOSStatusCommentString';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_4, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)


{
   pascal < threadsafe, exportset=fw_CarbonCore_XMerlot >
   Boolean IsDebuggerAttachedToProcess ( );
}
{
 *  NewDebugComponentCallbackUPP()
 *  
 *  Availability:
=======
 *  
 *  Availability:
=======
 *  
 *  Availability:
>>>>>>> graemeg/cpstrnew
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function GetMacOSStatusCommentString( err: OSStatus ): CStringPtr; external name '_GetMacOSStatusCommentString';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
   pascal < threadsafe, exportset=fw_CarbonCore_XMerlot >
   Boolean IsDebuggerAttachedToProcess ( );
}
{
 *  NewDebugComponentCallbackUPP()
 *  
 *  Availability:
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
 *  
 *  Availability:
=======
 *  
 *  Availability:
>>>>>>> origin/cpstrnew
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function GetMacOSStatusCommentString( err: OSStatus ): CStringPtr; external name '_GetMacOSStatusCommentString';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
   pascal < threadsafe, exportset=fw_CarbonCore_XMerlot >
   Boolean IsDebuggerAttachedToProcess ( );
}
{
 *  NewDebugComponentCallbackUPP()
 *  
 *  Availability:
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDebugComponentCallbackUPP( userRoutine: DebugComponentCallbackProcPtr ): DebugComponentCallbackUPP; external name '_NewDebugComponentCallbackUPP';
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
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DebugLib 1.0 and later
 }
procedure DebugAssert( componentSignature: OSType; options: UInt32; assertionString: ConstCStringPtr; exceptionLabelString: ConstCStringPtr; errorString: ConstCStringPtr; fileName: ConstCStringPtr; lineNumber: SIGNEDLONG; value: UnivPtr ); external name '_DebugAssert';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  TaskLevel masks
 }
const
	k68kInterruptLevelMask = $00000007; { 68K interrupt levels 0 through 7 }
	kInVBLTaskMask = $00000010; { VBLs are executing }
	kInDeferredTaskMask = $00000020; { Deferred tasks are executing }
	kInSecondaryIntHandlerMask = $00000040; { Secondary interrupt handlers are executing }
	kInNestedInterruptMask = $00000080; { The system is handling an interrupt }

{
 *  TaskLevel()
 *  
 *  Summary:
 *    TaskLevel returns 0 if we're (probably) running at non-interrupt
 *    time. There's no way to make this perfect, but this is as close
 *    as we can get. If TaskLevel doesn't return 0, then one of the
 *    TaskLevel masks can be used to learn more. Mac OS X has no
 *    concept of "TaskLevel"; and so it will always return the value 0.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Result:
 *    The current task level.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DebugLib 1.0 and later
 }
function TaskLevel: UInt32; external name '_TaskLevel';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  Constants used by the DebugComponent functions
 }
const
	kComponentDebugOption = 0;     { optionSelectorNum to turn breaks for component On or Off}

const
	kGetDebugOption = 1;    { get current debug option setting}
	kSetDebugOption = 2;     { set debug option}


{
 *  DebugComponentCallbackProcPtr
 *  
 *  Discussion:
 *    DebugComponentCallback is the callback into a component that
 *    registers with DebugLib. It is called to get the debug option
 *    setting, or to turn a debug option on or off.
 *  
 *  Parameters:
 *    
 *    optionSelectorNum:
 *      The component debug option to set.
 *    
 *    command:
 *      The command the DebugComponentCallbackProc must handle:
 *       kGetDebugOption - get current debug option setting
 *       kSetDebugOption - set debug option
 *    
 *    optionSetting:
 *      A pointer to a Boolean where the DebugComponentCallbackProc
 *      must return the option setting: the current setting if command
 *      is kGetDebugOption; the new debug option if command is 
 *      kSetDebugOption
 }
type
	DebugComponentCallbackProcPtr = procedure( optionSelectorNum: SInt32; command: UInt32; var optionSetting: Boolean );
	DebugComponentCallbackUPP = DebugComponentCallbackProcPtr;
{
 *  NewDebugComponent()
 *  
 *  Summary:
 *    NewDebugComponent registers a component with DebugLib.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    componentSignature:
 *      The unique signature of component.
 *    
 *    componentName:
 *      The displayable string naming the component.
 *    
 *    componentCallback:
 *      The callback into component for working with options.
 *  
 *  Result:
 *    An operating system result code: noErr, memFullErr,
 *    debuggingExecutionContextErr, debuggingDuplicateSignatureErr, or
 *    debuggingInvalidNameErr.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DebugLib 1.0 and later
 }
function NewDebugComponent( componentSignature: OSType; const (*var*) componentName: Str255; componentCallback: DebugComponentCallbackUPP ): OSStatus; external name '_NewDebugComponent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  NewDebugOption()
 *  
 *  Summary:
 *    NewDebugOption registers a debug option with DebugLib.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    componentSignature:
 *      The signature of component to register a debug option for.
 *    
 *    optionSelectorNum:
 *      The selector number of this debug option.
 *    
 *    optionName:
 *      The displayable string naming this debug option.
 *  
 *  Result:
 *    An operating system result code: noErr, memFullErr,
 *    debuggingExecutionContextErr, debuggingDuplicateOptionErr,
 *    debuggingInvalidSignatureErr, debuggingInvalidNameErr, or
 *    debuggingNoCallbackErr.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DebugLib 1.0 and later
 }
function NewDebugOption( componentSignature: OSType; optionSelectorNum: SInt32; const (*var*) optionName: Str255 ): OSStatus; external name '_NewDebugOption';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DisposeDebugComponent()
 *  
 *  Summary:
 *    DisposeDebugComponent removes a component registration and all
 *    related debug options from DebugLib.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    componentSignature:
 *      The unique signature of a component.
 *  
 *  Result:
 *    An operating system result code: noErr,
 *    debuggingExecutionContextErr, or debuggingInvalidSignatureErr.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DebugLib 1.0 and later
 }
function DisposeDebugComponent( componentSignature: OSType ): OSStatus; external name '_DisposeDebugComponent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDebugComponentInfo()
 *  
 *  Summary:
 *    GetDebugComponentInfo returns a component registered with
 *    DebugLib.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    itemIndex:
 *      The index into the list of registered components (1-based).
 *    
 *    componentSignature:
 *      A pointer to an OSType where the unique signature of a
 *      component is returned.
 *    
 *    componentName:
 *      A pointer to an Str255 where the displayable string naming a
 *      component is returned.
 *  
 *  Result:
 *    An operating system result code: noErr or debuggingNoMatchErr.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DebugLib 1.0 and later
 }
function GetDebugComponentInfo( itemIndex: UInt32; var componentSignature: OSType; var componentName: Str255 ): OSStatus; external name '_GetDebugComponentInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDebugOptionInfo()
 *  
 *  Summary:
 *    GetDebugOptionInfo returns a debug option registered with
 *    DebugLib.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    itemIndex:
 *      The index into the list of registered debug options (0-based);
 *      0 = kComponentDebugOption.
 *    
 *    componentSignature:
 *      The unique signature of a component.
 *    
 *    optionSelectorNum:
 *      A pointer to an SInt32 where the selector number of this debug
 *      option is returned.
 *    
 *    optionName:
 *      A pointer to an Str255 where the displayable string naming this
 *      debug option is returned.
 *    
 *    optionSetting:
 *      A pointer to an Boolean where the current debug option setting
 *      is returned.
 *  
 *  Result:
 *    An operating system result code: noErr,
 *    debuggingInvalidSignatureErr, or debuggingNoMatchErr.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DebugLib 1.0 and later
 }
function GetDebugOptionInfo( itemIndex: UInt32; componentSignature: OSType; var optionSelectorNum: SInt32; var optionName: Str255; var optionSetting: Boolean ): OSStatus; external name '_GetDebugOptionInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetDebugOptionValue()
 *  
 *  Summary:
 *    SetDebugOptionValue sets a debug option registered with DebugLib.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    componentSignature:
 *      The unique signature of a component.
 *    
 *    optionSelectorNum:
 *      The selector number of this debug option.
 *    
 *    newOptionSetting:
 *      The new debug option setting.
 *  
 *  Result:
 *    An operating system result code: noErr,
 *    debuggingInvalidSignatureErr, or debuggingInvalidOptionErr.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DebugLib 1.0 and later
 }
function SetDebugOptionValue( componentSignature: OSType; optionSelectorNum: SInt32; newOptionSetting: Boolean ): OSStatus; external name '_SetDebugOptionValue';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DebugAssertOutputHandlerProcPtr
 *  
 *  Discussion:
 *    DebugAssertOutputHandler is the callback that registers with
 *    DebugLib to handle the output from DebugAssert. The
 *    "componentSignature" through "value" parameters are the raw
 *    values passed to DebugAssert when an exception occurs.
 *  
 *  Parameters:
 *    
 *    componentSignature:
 *      The unique signature of component causing the assertion.
 *    
 *    options:
 *      reserved.
 *    
 *    assertionString:
 *      A pointer to a string containing the assertion, or NULL.
 *    
 *    exceptionLabelString:
 *      A pointer to a string containing the exceptionLabel, or NULL.
 *    
 *    errorString:
 *      A pointer to the error string, or NULL.
 *    
 *    fileName:
 *      A pointer to the fileName or pathname (generated by the
 *      preprocessor __FILE__ identifier), or NULL.
 *    
 *    lineNumber:
 *      The line number in the file (generated by the preprocessor
 *      __LINE__ identifier), or 0 (zero).
 *    
 *    value:
 *      A value associated with the assertion, or NULL.
 *    
 *    outputMsg:
 *      The string DebugAssert build which would normally be passed to
 *      DebugStr if a DebugAssertOutputHandler isn't installed.
 }
type
	DebugAssertOutputHandlerProcPtr = procedure( componentSignature: OSType; options: UInt32; assertionString: ConstCStringPtr; exceptionLabelString: ConstCStringPtr; errorString: ConstCStringPtr; fileName: ConstCStringPtr; lineNumber: SIGNEDLONG; value: UnivPtr; const (*var*) outputMsg: Str255 );
	DebugAssertOutputHandlerUPP = DebugAssertOutputHandlerProcPtr;
{
 *  InstallDebugAssertOutputHandler()
 *  
 *  Summary:
 *    InstallDebugAssertOutputHandler installs a
 *    DebugAssertOutputHandler which DebugAssert calls instead of
 *    DebugStr.
 *  
 *  Parameters:
 *    
 *    handler:
 *      The DebugAssertOutputHandler to install or NULL to switch back
 *      to the default handler (DebugStr).
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DebugLib 1.0 and later
 }
procedure InstallDebugAssertOutputHandler( handler: DebugAssertOutputHandlerUPP ); external name '_InstallDebugAssertOutputHandler';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  dprintf()
 *  
 *  Summary:
 *    printf takes a variable argument list and 'prints' that to the
 *    debugging output handler.  Calling dprintf() from anything but C
 *    or C++ is tricky.
 *  
 *  Parameters:
 *    
 *    format:
 *      The format string.
 *    
 *    ...:
 *      The arguments to the format string.
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DebugLib 1.1 and later
 }


{
 *  vdprintf()
 *  
 *  Summary:
 *    vdprintf takes a va_args list and 'prints' that to the debugging
 *    output handler.
 *  
 *  Parameters:
 *    
 *    format:
 *      The format string.
 *    
 *    va_args_list:
 *      The va_args list.
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in DebugLib 1.1 and later
 }


{
 *  GetMacOSStatusErrorString()
 *  
 *  Summary:
 *    Returns a const char* string which corresponds to the textual
 *    constant for the given OSStatus code.
 *  
 *  Discussion:
 *    This function returns a text string which corresponds to the
 *    given OSStatus code, based on the errors in MacErrors.h.  For
 *    example, GetMacOSStatusErrorString( -43 ) returns "fnfErr", which
 *    is the text representation for the error constant -43.  This
 *    function is useful if you want to get or print out ( for
 *    debugging purposes only ) a useful description for a given
 *    OSStatus error.  If no string is available for the given
 *    constant, then the empty string "" is returned. Some error values
 *    have multiple meanings; in those cases the multiple meanings are
 *    all returned seperated by '/'es.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    err:
 *      The OSStatus to return a text string for.
 *  
 *  Result:
 *    A const char* string corresponding to the given OSStatus
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function GetMacOSStatusErrorString( err: OSStatus ): CStringPtr; external name '_GetMacOSStatusErrorString';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


<<<<<<< HEAD
const
	uppDebugComponentCallbackProcInfo = $00000FC0;
	uppDebugAssertOutputHandlerProcInfo = $00FFFFC0;
	{
	 *  NewDebugComponentCallbackUPP()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   available as macro/inline
	 *    CarbonLib:        in CarbonLib 1.0 and later
	 *    Mac OS X:         in version 10.0 and later
	 	}
function NewDebugComponentCallbackUPP(userRoutine: DebugComponentCallbackProcPtr): DebugComponentCallbackUPP; external name '_NewDebugComponentCallbackUPP'; { old name was NewDebugComponentCallbackProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{
 *  GetMacOSStatusCommentString()
 *  
 *  Summary:
 *    Returns a const char* string which corresponds to the descriptive
 *    string for the given OSStatus code.
 *  
 *  Discussion:
 *    This function returns a text string which corresponds to a
 *    comment for the given OSStatus code, based on the errors in
 *    MacErrors.h.  For example, GetMacOSStatusConstantString( -43 )
 *    returns "File not found", which is the text representation for
 *    the error constant -43.  This function is useful if you want to
 *    get or print out ( for debugging purposes only ) a useful
 *    description for a given OSStatus error.  If no string is
 *    available for the given constant, then the empty string "" is
 *    returned. If no string is available for the given constant, then
 *    the empty string "" is returned.  Some error values have multiple
 *    meanings; in those cases the multiple meanings are all returned
 *    seperated by '/'es.
 *  
 *  Mac OS X threading:
 *    Thread safe
 *  
 *  Parameters:
 *    
 *    err:
 *      The OSStatus to return a text string for.
 *  
 *  Result:
 *    A const char* string corresponding to the given OSStatus
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDebugAssertOutputHandlerUPP( userRoutine: DebugAssertOutputHandlerProcPtr ): DebugAssertOutputHandlerUPP; external name '_NewDebugAssertOutputHandlerUPP';
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
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function GetMacOSStatusCommentString( err: OSStatus ): CStringPtr; external name '_GetMacOSStatusCommentString';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
   pascal < threadsafe, exportset=fw_CarbonCore_XMerlot >
   Boolean IsDebuggerAttachedToProcess ( );
}
{
 *  NewDebugComponentCallbackUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
>>>>>>> origin/fixes_2.4
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDebugComponentCallbackUPP( userRoutine: DebugComponentCallbackProcPtr ): DebugComponentCallbackUPP; external name '_NewDebugComponentCallbackUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  NewDebugAssertOutputHandlerUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function NewDebugAssertOutputHandlerUPP(userRoutine: DebugAssertOutputHandlerProcPtr): DebugAssertOutputHandlerUPP; external name '_NewDebugAssertOutputHandlerUPP'; { old name was NewDebugAssertOutputHandlerProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function NewDebugAssertOutputHandlerUPP( userRoutine: DebugAssertOutputHandlerProcPtr ): DebugAssertOutputHandlerUPP; external name '_NewDebugAssertOutputHandlerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  DisposeDebugComponentCallbackUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDebugComponentCallbackUPP( userUPP: DebugComponentCallbackUPP ); external name '_DisposeDebugComponentCallbackUPP';
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
procedure DisposeDebugComponentCallbackUPP(userUPP: DebugComponentCallbackUPP); external name '_DisposeDebugComponentCallbackUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeDebugComponentCallbackUPP( userUPP: DebugComponentCallbackUPP ); external name '_DisposeDebugComponentCallbackUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  DisposeDebugAssertOutputHandlerUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDebugAssertOutputHandlerUPP( userUPP: DebugAssertOutputHandlerUPP ); external name '_DisposeDebugAssertOutputHandlerUPP';
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
procedure DisposeDebugAssertOutputHandlerUPP(userUPP: DebugAssertOutputHandlerUPP); external name '_DisposeDebugAssertOutputHandlerUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeDebugAssertOutputHandlerUPP( userUPP: DebugAssertOutputHandlerUPP ); external name '_DisposeDebugAssertOutputHandlerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  InvokeDebugComponentCallbackUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeDebugComponentCallbackUPP( optionSelectorNum: SInt32; command: UInt32; var optionSetting: Boolean; userUPP: DebugComponentCallbackUPP ); external name '_InvokeDebugComponentCallbackUPP';
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
procedure InvokeDebugComponentCallbackUPP(optionSelectorNum: SInt32; command: UInt32; var optionSetting: boolean; userRoutine: DebugComponentCallbackUPP); external name '_InvokeDebugComponentCallbackUPP'; { old name was CallDebugComponentCallbackProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure InvokeDebugComponentCallbackUPP( optionSelectorNum: SInt32; command: UInt32; var optionSetting: Boolean; userUPP: DebugComponentCallbackUPP ); external name '_InvokeDebugComponentCallbackUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  InvokeDebugAssertOutputHandlerUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeDebugAssertOutputHandlerUPP( componentSignature: OSType; options: UInt32; assertionString: ConstCStringPtr; exceptionLabelString: ConstCStringPtr; errorString: ConstCStringPtr; fileName: ConstCStringPtr; lineNumber: SIGNEDLONG; value: UnivPtr; const (*var*) outputMsg: Str255; userUPP: DebugAssertOutputHandlerUPP ); external name '_InvokeDebugAssertOutputHandlerUPP';
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
procedure InvokeDebugAssertOutputHandlerUPP( componentSignature: OSType; options: UInt32; assertionString: ConstCStringPtr; exceptionLabelString: ConstCStringPtr; errorString: ConstCStringPtr; fileName: ConstCStringPtr; lineNumber: SIGNEDLONG; value: UnivPtr; const (*var*) outputMsg: Str255; userUPP: DebugAssertOutputHandlerUPP ); external name '_InvokeDebugAssertOutputHandlerUPP';
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
