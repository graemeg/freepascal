{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       HIServices/InternetConfig.h
 
     Contains:   Internet Config interfaces
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Version:    HIServices-416~44
=======
     Version:    HIServices-308~1
>>>>>>> graemeg/cpstrnew
=======
     Version:    HIServices-308~1
>>>>>>> graemeg/cpstrnew
=======
     Version:    HIServices-308~1
>>>>>>> graemeg/cpstrnew
=======
     Version:    HIServices-308~1
>>>>>>> origin/cpstrnew
 
     Copyright:  � 1999-2008 by Apple Computer, Inc., all rights reserved.
=======
=======
>>>>>>> origin/fixes_2_2
     File:       InternetConfig.p
=======
     File:       HIServices/InternetConfig.h
>>>>>>> origin/fixes_2.4
 
     Contains:   Internet Config interfaces
 
     Version:    HIServices-308~1
 
<<<<<<< HEAD
     Copyright:  � 1999-2002 by Apple Computer, Inc., all rights reserved.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  � 1999-2008 by Apple Computer, Inc., all rights reserved.
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
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
=======
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit InternetConfig;
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
=======
>>>>>>> origin/fixes_2.4
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
>>>>>>> graemeg/cpstrnew
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
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
>>>>>>> origin/cpstrnew
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
>>>>>>> origin/cpstrnew
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
<<<<<<< HEAD
<<<<<<< HEAD
{$elsec}
=======
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
>>>>>>> origin/cpstrnew
  {$setc TARGET_CPU_64 := FALSE}
{$endc}
=======
=======
>>>>>>> origin/fixes_2_2
=======
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
uses MacTypes,Files,Aliases,Components,AEDataModel;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not MACOSALLINCLUDE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4


{$ifc TARGET_OS_MAC}


{
    IMPORTANT NOTES ABOUT THE C HEADERS
    -----------------------------------

    o   When you see the parameter 'y *x', you should be aware that
        you *cannot pass in nil*.  In future this restriction may be eased,
        especially for the attr parameter to ICGetPref.  Parameters where nil
        is legal are declared using the explicit pointer type, ie 'yPtr x'.

    o   Strings are *Pascal* strings.  This means that they must be word aligned.
        MPW and Think C do this automatically.  The last time I checked, Metrowerks
        C does not.  If it still doesn't, then IMHO it's a bug in their compiler
        and you should report it to them.  [IC 1.4 and later no longer require
        word aligned strings.  You can ignore this warning if you require IC 1.4
        or greater.]
}
{*********************************************************************************************}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew


{$ifc TARGET_OS_MAC}


<<<<<<< HEAD
{
    IMPORTANT NOTES ABOUT THE C HEADERS
    -----------------------------------

    o   When you see the parameter 'y *x', you should be aware that
        you *cannot pass in nil*.  In future this restriction may be eased,
        especially for the attr parameter to ICGetPref.  Parameters where nil
        is legal are declared using the explicit pointer type, ie 'yPtr x'.

    o   Strings are *Pascal* strings.  This means that they must be word aligned.
        MPW and Think C do this automatically.  The last time I checked, Metrowerks
        C does not.  If it still doesn't, then IMHO it's a bug in their compiler
        and you should report it to them.  [IC 1.4 and later no longer require
        word aligned strings.  You can ignore this warning if you require IC 1.4
        or greater.]
}
{*********************************************************************************************}
>>>>>>> graemeg/cpstrnew


{$ifc TARGET_OS_MAC}

<<<<<<< HEAD
{

    ***DEPRECATION NOTICE***

    The Internet Config APIs are officially deprecated in 10.7.

    You can replace your use of Internet Config APIs with LaunchServices APIs.
    For example, to find which application is currently preferred for a
    particular MIME type, use LSCopyApplicationForMIMEType().

}

{
    IMPORTANT NOTES ABOUT THE C HEADERS
    -----------------------------------

    o   When you see the parameter 'y *x', you should be aware that
        you *cannot pass in nil*.  In future this restriction may be eased,
        especially for the attr parameter to ICGetPref.  Parameters where nil
        is legal are declared using the explicit pointer type, ie 'yPtr x'.

    o   Strings are *Pascal* strings.  This means that they must be word aligned.
        MPW and Think C do this automatically.  The last time I checked, Metrowerks
        C does not.  If it still doesn't, then IMHO it's a bug in their compiler
        and you should report it to them.  [IC 1.4 and later no longer require
        word aligned strings.  You can ignore this warning if you require IC 1.4
        or greater.]
}
{*********************************************************************************************}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4


{$ALIGN MAC68K}

{***********************************************************************************************
  IC error codes
 ***********************************************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
const
	icPrefNotFoundErr = -666; { preference not found (duh!)  }
	icPermErr = -667; { cannot set preference  }
	icPrefDataErr = -668; { problem with preference data  }
	icInternalErr = -669; { hmm, this is not good  }
	icTruncatedErr = -670; { more data was present than was returned  }
	icNoMoreWritersErr = -671; { you cannot begin a write session because someone else is already doing it  }
	icNothingToOverrideErr = -672; { no component for the override component to capture  }
	icNoURLErr = -673; { no URL found  }
	icConfigNotFoundErr = -674; { no configuration was found  }
	icConfigInappropriateErr = -675; { incorrect manufacturer code  }
	icProfileNotFoundErr = -676; { profile not found  }
	icTooManyProfilesErr = -677;  { too many profiles in database  }

{***********************************************************************************************
  IC versions (not necessarily, but historically, from a component)
 ***********************************************************************************************}

const
	kICComponentInterfaceVersion0 = $00000000; { IC >= 1.0  }
	kICComponentInterfaceVersion1 = $00010000; { IC >= 1.1  }
	kICComponentInterfaceVersion2 = $00020000; { IC >= 1.2  }
	kICComponentInterfaceVersion3 = $00030000; { IC >= 2.0  }
	kICComponentInterfaceVersion4 = $00040000; { IC >= 2.5  }
	kICComponentInterfaceVersion = kICComponentInterfaceVersion4; { current version number is 4  }

{***********************************************************************************************
  opaque type for preference reference
 ***********************************************************************************************}
=======
const
	icPrefNotFoundErr = -666; { preference not found (duh!)  }
	icPermErr = -667; { cannot set preference  }
	icPrefDataErr = -668; { problem with preference data  }
	icInternalErr = -669; { hmm, this is not good  }
	icTruncatedErr = -670; { more data was present than was returned  }
	icNoMoreWritersErr = -671; { you cannot begin a write session because someone else is already doing it  }
	icNothingToOverrideErr = -672; { no component for the override component to capture  }
	icNoURLErr = -673; { no URL found  }
	icConfigNotFoundErr = -674; { no configuration was found  }
	icConfigInappropriateErr = -675; { incorrect manufacturer code  }
	icProfileNotFoundErr = -676; { profile not found  }
	icTooManyProfilesErr = -677;  { too many profiles in database  }

{***********************************************************************************************
=======
const
	icPrefNotFoundErr = -666; { preference not found (duh!)  }
	icPermErr = -667; { cannot set preference  }
	icPrefDataErr = -668; { problem with preference data  }
	icInternalErr = -669; { hmm, this is not good  }
	icTruncatedErr = -670; { more data was present than was returned  }
	icNoMoreWritersErr = -671; { you cannot begin a write session because someone else is already doing it  }
	icNothingToOverrideErr = -672; { no component for the override component to capture  }
	icNoURLErr = -673; { no URL found  }
	icConfigNotFoundErr = -674; { no configuration was found  }
	icConfigInappropriateErr = -675; { incorrect manufacturer code  }
	icProfileNotFoundErr = -676; { profile not found  }
	icTooManyProfilesErr = -677;  { too many profiles in database  }

{***********************************************************************************************
>>>>>>> origin/fixes_2.4
  IC versions (not necessarily, but historically, from a component)
 ***********************************************************************************************}

const
	kICComponentInterfaceVersion0 = $00000000; { IC >= 1.0  }
	kICComponentInterfaceVersion1 = $00010000; { IC >= 1.1  }
	kICComponentInterfaceVersion2 = $00020000; { IC >= 1.2  }
	kICComponentInterfaceVersion3 = $00030000; { IC >= 2.0  }
	kICComponentInterfaceVersion4 = $00040000; { IC >= 2.5  }
	kICComponentInterfaceVersion = kICComponentInterfaceVersion4; { current version number is 4  }
<<<<<<< HEAD

{***********************************************************************************************
  opaque type for preference reference
 ***********************************************************************************************}
=======
=======
>>>>>>> origin/cpstrnew
const
	icPrefNotFoundErr = -666; { preference not found (duh!)  }
	icPermErr = -667; { cannot set preference  }
	icPrefDataErr = -668; { problem with preference data  }
	icInternalErr = -669; { hmm, this is not good  }
	icTruncatedErr = -670; { more data was present than was returned  }
	icNoMoreWritersErr = -671; { you cannot begin a write session because someone else is already doing it  }
	icNothingToOverrideErr = -672; { no component for the override component to capture  }
	icNoURLErr = -673; { no URL found  }
	icConfigNotFoundErr = -674; { no configuration was found  }
	icConfigInappropriateErr = -675; { incorrect manufacturer code  }
	icProfileNotFoundErr = -676; { profile not found  }
	icTooManyProfilesErr = -677;  { too many profiles in database  }

{***********************************************************************************************
<<<<<<< HEAD
=======
const
	icPrefNotFoundErr = -666; { preference not found (duh!)  }
	icPermErr = -667; { cannot set preference  }
	icPrefDataErr = -668; { problem with preference data  }
	icInternalErr = -669; { hmm, this is not good  }
	icTruncatedErr = -670; { more data was present than was returned  }
	icNoMoreWritersErr = -671; { you cannot begin a write session because someone else is already doing it  }
	icNothingToOverrideErr = -672; { no component for the override component to capture  }
	icNoURLErr = -673; { no URL found  }
	icConfigNotFoundErr = -674; { no configuration was found  }
	icConfigInappropriateErr = -675; { incorrect manufacturer code  }
	icProfileNotFoundErr = -676; { profile not found  }
	icTooManyProfilesErr = -677;  { too many profiles in database  }

{***********************************************************************************************
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  IC versions (not necessarily, but historically, from a component)
 ***********************************************************************************************}

const
	kICComponentInterfaceVersion0 = $00000000; { IC >= 1.0  }
	kICComponentInterfaceVersion1 = $00010000; { IC >= 1.1  }
	kICComponentInterfaceVersion2 = $00020000; { IC >= 1.2  }
	kICComponentInterfaceVersion3 = $00030000; { IC >= 2.0  }
	kICComponentInterfaceVersion4 = $00040000; { IC >= 2.5  }
	kICComponentInterfaceVersion = kICComponentInterfaceVersion4; { current version number is 4  }
=======
>>>>>>> origin/fixes_2.4

{***********************************************************************************************
  opaque type for preference reference
 ***********************************************************************************************}

type
	ICInstance = ^SInt32; { an opaque type }
	ICInstancePtr = ^ICInstance;  { when a var xx:ICInstance parameter can be nil, it is changed to xx: ICInstancePtr }
{$ifc not TARGET_CPU_64}
{***********************************************************************************************
  a record that specifies a folder, an array of such records, and a pointer to such an array
 ***********************************************************************************************}
type
	ICDirSpec = record
		vRefNum: SInt16;
		dirID: SIGNEDLONG;
	end;
	ICDirSpecPtr = ^ICDirSpec;

	ICDirSpecArray = array [0..3] of ICDirSpec;
	ICDirSpecArrayPtr = ^ICDirSpecArray;
{$endc} {not TARGET_CPU_64}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{***********************************************************************************************
  preference attributes type, bit number constants, and mask constants
 ***********************************************************************************************}
type
	ICAttr = UInt32;
const
	kICAttrLockedBit = 0;
	kICAttrVolatileBit = 1;

const
	kICAttrNoChange = $FFFFFFFF; { pass this to ICSetPref to tell it not to change the attributes  }
	kICAttrLockedMask = $00000001;
	kICAttrVolatileMask = $00000002;
>>>>>>> graemeg/cpstrnew

{***********************************************************************************************
  permissions for use with ICBegin
 ***********************************************************************************************}
type
<<<<<<< HEAD
	ICInstance = ^SInt32; { an opaque type }
	ICInstancePtr = ^ICInstance;  { when a var xx:ICInstance parameter can be nil, it is changed to xx: ICInstancePtr }
{$ifc not TARGET_CPU_64}
{***********************************************************************************************
  a record that specifies a folder, an array of such records, and a pointer to such an array
 ***********************************************************************************************}
type
	ICDirSpec = record
		vRefNum: SInt16;
		dirID: SIGNEDLONG;
	end;
	ICDirSpecPtr = ^ICDirSpec;

	ICDirSpecArray = array [0..3] of ICDirSpec;
	ICDirSpecArrayPtr = ^ICDirSpecArray;
{$endc} {not TARGET_CPU_64}
>>>>>>> graemeg/cpstrnew
=======
	ICPerm = UInt8;
const
	icNoPerm = 0;
	icReadOnlyPerm = 1;
	icReadWritePerm = 2;

>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4

{***********************************************************************************************
  preference attributes type, bit number constants, and mask constants
 ***********************************************************************************************}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
<<<<<<< HEAD
	ICInstance = ^OpaqueICInstance; { an opaque type }
	OpaqueICInstance = record end;
	ICInstancePtr = ^ICInstance;  { when a var xx:ICInstance parameter can be nil, it is changed to xx: ICInstancePtr }
{$ifc not TARGET_CPU_64}
{***********************************************************************************************
  a record that specifies a folder, an array of such records, and a pointer to such an array
 ***********************************************************************************************}
type
<<<<<<< HEAD
	ICDirSpec = record
		vRefNum: SInt16;
		dirID: SIGNEDLONG;
	end;
	ICDirSpecPtr = ^ICDirSpec;

	ICDirSpecArray = array [0..3] of ICDirSpec;
	ICDirSpecArrayPtr = ^ICDirSpecArray;
{$endc} {not TARGET_CPU_64}

{***********************************************************************************************
  preference attributes type, bit number constants, and mask constants
 ***********************************************************************************************}
type
=======
>>>>>>> graemeg/cpstrnew
=======
type
>>>>>>> graemeg/cpstrnew
=======
type
>>>>>>> origin/cpstrnew
=======
type
>>>>>>> origin/fixes_2.4
	ICAttr = UInt32;
const
	kICAttrLockedBit = 0;
	kICAttrVolatileBit = 1;

const
	kICAttrNoChange = $FFFFFFFF; { pass this to ICSetPref to tell it not to change the attributes  }
	kICAttrLockedMask = $00000001;
	kICAttrVolatileMask = $00000002;

{***********************************************************************************************
  permissions for use with ICBegin
 ***********************************************************************************************}
type
	ICPerm = UInt8;
const
	icNoPerm = 0;
	icReadOnlyPerm = 1;
	icReadWritePerm = 2;


{***********************************************************************************************
  profile IDs
 ***********************************************************************************************}
type
	ICProfileID = SInt32;
	ICProfileIDPtr = ^ICProfileID;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
const
	kICNilProfileID = 0;

{***********************************************************************************************
  other constants
 ***********************************************************************************************}
const
	kICNoUserInteractionBit = 0;

const
	kICNoUserInteractionMask = $00000001;

const
	kICFileType = FourCharCode('ICAp');
	kICCreator = FourCharCode('ICAp');

{***********************************************************************************************
  Apple event constants
 ***********************************************************************************************}
const
	kInternetEventClass = FourCharCode('GURL');
	kAEGetURL = FourCharCode('GURL');
	kAEFetchURL = FourCharCode('FURL');
	keyAEAttaching = FourCharCode('Atch');

{ AERegistry.i defines a compatible keyAEDestination }
const
	kICEditPreferenceEventClass = FourCharCode('ICAp');
	kICEditPreferenceEvent = FourCharCode('ICAp');
	keyICEditPreferenceDestination = FourCharCode('dest');

{***********************************************************************************************
=======
	ICProfileID = SInt32;
	ICProfileIDPtr = ^ICProfileID;
const
	kICNilProfileID = 0;

=======
const
	kICNilProfileID = 0;

>>>>>>> origin/cpstrnew
{***********************************************************************************************
  other constants
 ***********************************************************************************************}
const
	kICNoUserInteractionBit = 0;

const
	kICNoUserInteractionMask = $00000001;

const
	kICFileType = FourCharCode('ICAp');
	kICCreator = FourCharCode('ICAp');

{***********************************************************************************************
  Apple event constants
 ***********************************************************************************************}
const
	kInternetEventClass = FourCharCode('GURL');
	kAEGetURL = FourCharCode('GURL');
	kAEFetchURL = FourCharCode('FURL');
	keyAEAttaching = FourCharCode('Atch');

{ AERegistry.i defines a compatible keyAEDestination }
const
	kICEditPreferenceEventClass = FourCharCode('ICAp');
	kICEditPreferenceEvent = FourCharCode('ICAp');
	keyICEditPreferenceDestination = FourCharCode('dest');

{***********************************************************************************************
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
const
	kICNilProfileID = 0;

{***********************************************************************************************
  other constants
 ***********************************************************************************************}
const
	kICNoUserInteractionBit = 0;

const
	kICNoUserInteractionMask = $00000001;

const
	kICFileType = FourCharCode('ICAp');
	kICCreator = FourCharCode('ICAp');

{***********************************************************************************************
  Apple event constants
 ***********************************************************************************************}
const
	kInternetEventClass = FourCharCode('GURL');
	kAEGetURL = FourCharCode('GURL');
	kAEFetchURL = FourCharCode('FURL');
	keyAEAttaching = FourCharCode('Atch');

{ AERegistry.i defines a compatible keyAEDestination }
const
	kICEditPreferenceEventClass = FourCharCode('ICAp');
	kICEditPreferenceEvent = FourCharCode('ICAp');
	keyICEditPreferenceDestination = FourCharCode('dest');

{***********************************************************************************************
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
const
	kICNilProfileID = 0;

{***********************************************************************************************
  other constants
 ***********************************************************************************************}
const
	kICNoUserInteractionBit = 0;

const
	kICNoUserInteractionMask = $00000001;

const
	kICFileType = FourCharCode('ICAp');
	kICCreator = FourCharCode('ICAp');

{***********************************************************************************************
  Apple event constants
 ***********************************************************************************************}
const
	kInternetEventClass = FourCharCode('GURL');
	kAEGetURL = FourCharCode('GURL');
	kAEFetchURL = FourCharCode('FURL');
	keyAEAttaching = FourCharCode('Atch');

{ AERegistry.i defines a compatible keyAEDestination }
const
	kICEditPreferenceEventClass = FourCharCode('ICAp');
	kICEditPreferenceEvent = FourCharCode('ICAp');
	keyICEditPreferenceDestination = FourCharCode('dest');

{***********************************************************************************************
>>>>>>> origin/fixes_2.4
  constants for use with ICGetVersion
 ***********************************************************************************************}
const
	kICComponentVersion = 0;    { Return a component version, comparable to kICComponentInterfaceVersion  }
	kICNumVersion = 1;     { Return a NumVersion structure  }

{***********************************************************************************************
  types and constants for use with kICDocumentFont, et. al.
 ***********************************************************************************************}
type
	ICFontRecord = record
		size: SInt16;
		face: SInt8;
		pad: SInt8;
		font: Str255;
	end;
	ICFontRecordPtr = ^ICFontRecord;
type
	ICFontRecordHandle = ^ICFontRecordPtr;

{***********************************************************************************************
  types and constants for use with kICCharacterSet, et. al.
 ***********************************************************************************************}
type
	ICCharTable = record
		netToMac: packed array [0..255] of UInt8;
		macToNet: packed array [0..255] of UInt8;
	end;
	ICCharTablePtr = ^ICCharTable;
type
	ICCharTableHandle = ^ICCharTablePtr;

{***********************************************************************************************
  types and constants for use with kICHelper, et. al.
 ***********************************************************************************************}
type
	ICAppSpec = record
		fCreator: OSType;
		name: Str63;
	end;
	ICAppSpecPtr = ^ICAppSpec;
type
	ICAppSpecHandle = ^ICAppSpecPtr;
	ICAppSpecList = record
		numberOfItems: SInt16;
		appSpecs: array [0..0] of ICAppSpec;
	end;
	ICAppSpecListPtr = ^ICAppSpecList;
type
	ICAppSpecListHandle = ^ICAppSpecListPtr;

{***********************************************************************************************
  types and constants for use with kICDownloadFolder, et. al.
 ***********************************************************************************************}
type
	ICFileSpec = record
		volName: Str31;                { this field should be ignored, use the alias }
		volCreationDate: SInt32;        { this field should be ignored, use the alias }
		fss: FSSpec;                    { this field should be ignored, use the alias }
		alias: AliasRecord;
                                              { plus extra data, aliasSize 0 means no alias manager present when}
                                              { ICFileSpecification was created}
	end;
	ICFileSpecPtr = ^ICFileSpec;
type
	ICFileSpecHandle = ^ICFileSpecPtr;
const
	kICFileSpecHeaderSize = SizeOf(ICFileSpec) - sizeof(AliasRecord);

{***********************************************************************************************
  types and constants for use with ICMapFilename, et. al.
 ***********************************************************************************************}
type
	ICMapEntryFlags = SInt32;
	ICFixedLength = SInt16;
	ICMapEntry = record
		totalLength: SInt16;
		fixedLength: ICFixedLength;
		version: SInt16;
		fileType: OSType;
		fileCreator: OSType;
		postCreator: OSType;
		flags: ICMapEntryFlags;
                                              { variable part starts here}
		extension: Str255;
		creatorAppName: Str255;
		postAppName: Str255;
		MIMEType: Str255;
		entryName: Str255;
	end;
	ICMapEntryPtr = ^ICMapEntry;
type
	ICMapEntryHandle = ^ICMapEntryPtr;
<<<<<<< HEAD
const
	kICMapFixedLength = 22;

const
	kICMapBinaryBit = 0;    { file should be transfered in binary as opposed to text mode}
	kICMapResourceForkBit = 1;    { the resource fork of the file is significant}
	kICMapDataForkBit = 2;    { the data fork of the file is significant}
	kICMapPostBit = 3;    { post process using post fields}
	kICMapNotIncomingBit = 4;    { ignore this mapping for incoming files}
	kICMapNotOutgoingBit = 5;     { ignore this mapping for outgoing files}

const
	kICMapBinaryMask = $00000001; { file should be transfered in binary as opposed to text mode}
	kICMapResourceForkMask = $00000002; { the resource fork of the file is significant}
	kICMapDataForkMask = $00000004; { the data fork of the file is significant}
	kICMapPostMask = $00000008; { post process using post fields}
	kICMapNotIncomingMask = $00000010; { ignore this mapping for incoming files}
	kICMapNotOutgoingMask = $00000020; { ignore this mapping for outgoing files}

{***********************************************************************************************
  types and constants for use with kICServices, et. al.
 ***********************************************************************************************}
type
	ICServiceEntryFlags = SInt16;
	ICServiceEntry = record
		name: Str255;
		port: SInt16;
		flags: ICServiceEntryFlags;
	end;
	ICServiceEntryPtr = ^ICServiceEntry;
type
	ICServiceEntryHandle = ^ICServiceEntryPtr;

const
	kICServicesTCPBit = 0;
	kICServicesUDPBit = 1;     { both bits can be set, which means the service is both TCP and UDP, eg daytime}

const
	kICServicesTCPMask = $00000001;
	kICServicesUDPMask = $00000002; { both bits can be set, which means the service is both TCP and UDP, eg daytime}

type
	ICServices = record
		count: SInt16;
		services: array [0..0] of ICServiceEntry;
	end;
	ICServicesPtr = ^ICServices;
type
	ICServicesHandle = ^ICServicesPtr;
=======

const
	icPrefNotFoundErr			= -666;							{  preference not found (duh!)   }
	icPermErr					= -667;							{  cannot set preference   }
	icPrefDataErr				= -668;							{  problem with preference data   }
	icInternalErr				= -669;							{  hmm, this is not good   }
	icTruncatedErr				= -670;							{  more data was present than was returned   }
	icNoMoreWritersErr			= -671;							{  you cannot begin a write session because someone else is already doing it   }
	icNothingToOverrideErr		= -672;							{  no component for the override component to capture   }
	icNoURLErr					= -673;							{  no URL found   }
	icConfigNotFoundErr			= -674;							{  no configuration was found   }
	icConfigInappropriateErr	= -675;							{  incorrect manufacturer code   }
	icProfileNotFoundErr		= -676;							{  profile not found   }
	icTooManyProfilesErr		= -677;							{  too many profiles in database   }

	{	***********************************************************************************************
	  IC versions (not necessarily, but historically, from a component)
	 ***********************************************************************************************	}

	kICComponentInterfaceVersion0 = $00000000;					{  IC >= 1.0   }
	kICComponentInterfaceVersion1 = $00010000;					{  IC >= 1.1   }
	kICComponentInterfaceVersion2 = $00020000;					{  IC >= 1.2   }
	kICComponentInterfaceVersion3 = $00030000;					{  IC >= 2.0   }
	kICComponentInterfaceVersion4 = $00040000;					{  IC >= 2.5   }
	kICComponentInterfaceVersion = $00040000;					{  current version number is 4   }

	{	***********************************************************************************************
	  opaque type for preference reference
	 ***********************************************************************************************	}


type
	ICInstance    = ^SInt32; { an opaque 32-bit type }
	ICInstancePtr = ^ICInstance;  { when a var xx:ICInstance parameter can be nil, it is changed to xx: ICInstancePtr }

	{	***********************************************************************************************
	  a record that specifies a folder, an array of such records, and a pointer to such an array
	 ***********************************************************************************************	}
	ICDirSpecPtr = ^ICDirSpec;
	ICDirSpec = record
		vRefNum:				SInt16;
		dirID:					SInt32;
	end;

	ICDirSpecArray						= array [0..3] of ICDirSpec;
	ICDirSpecArrayPtr					= ^ICDirSpecArray;

	{	***********************************************************************************************
	  preference attributes type, bit number constants, and mask constants
	 ***********************************************************************************************	}
	ICAttr								= UInt32;


const
	kICAttrLockedBit			= 0;
	kICAttrVolatileBit			= 1;

	kICAttrNoChange				= $FFFFFFFF;					{  pass this to ICSetPref to tell it not to change the attributes   }
	kICAttrLockedMask			= $00000001;
	kICAttrVolatileMask			= $00000002;

	{	***********************************************************************************************
	  permissions for use with ICBegin
	 ***********************************************************************************************	}
=======
const
	kICMapFixedLength = 22;

const
	kICMapBinaryBit = 0;    { file should be transfered in binary as opposed to text mode}
	kICMapResourceForkBit = 1;    { the resource fork of the file is significant}
	kICMapDataForkBit = 2;    { the data fork of the file is significant}
	kICMapPostBit = 3;    { post process using post fields}
	kICMapNotIncomingBit = 4;    { ignore this mapping for incoming files}
	kICMapNotOutgoingBit = 5;     { ignore this mapping for outgoing files}

const
	kICMapBinaryMask = $00000001; { file should be transfered in binary as opposed to text mode}
	kICMapResourceForkMask = $00000002; { the resource fork of the file is significant}
	kICMapDataForkMask = $00000004; { the data fork of the file is significant}
	kICMapPostMask = $00000008; { post process using post fields}
	kICMapNotIncomingMask = $00000010; { ignore this mapping for incoming files}
	kICMapNotOutgoingMask = $00000020; { ignore this mapping for outgoing files}

{***********************************************************************************************
  types and constants for use with kICServices, et. al.
 ***********************************************************************************************}
type
	ICServiceEntryFlags = SInt16;
	ICServiceEntry = record
		name: Str255;
		port: SInt16;
		flags: ICServiceEntryFlags;
	end;
	ICServiceEntryPtr = ^ICServiceEntry;
type
	ICServiceEntryHandle = ^ICServiceEntryPtr;

const
	kICServicesTCPBit = 0;
	kICServicesUDPBit = 1;     { both bits can be set, which means the service is both TCP and UDP, eg daytime}
>>>>>>> origin/fixes_2.4

const
	kICServicesTCPMask = $00000001;
	kICServicesUDPMask = $00000002; { both bits can be set, which means the service is both TCP and UDP, eg daytime}

type
<<<<<<< HEAD
	ICPerm								= UInt8;


const
	icNoPerm					= 0;
	icReadOnlyPerm				= 1;
	icReadWritePerm				= 2;

	{	***********************************************************************************************
	  a reference to an instance's current configuration
	 ***********************************************************************************************	}

{$ifc CALL_NOT_IN_CARBON}

type
	ICConfigRefPtr = ^ICConfigRef;
	ICConfigRef = record
		manufacturer:			OSType;
																		{  other private data follows   }
	end;

	ICConfigRefHandle					= ^ICConfigRefPtr;

{$endc}  {CALL_NOT_IN_CARBON}

{***********************************************************************************************
  profile IDs
 ***********************************************************************************************}

type
	ICProfileID							= SInt32;
	ICProfileIDPtr						= ^ICProfileID;


const
	kICNilProfileID				= 0;

	{	***********************************************************************************************
	  other constants
	 ***********************************************************************************************	}

	kICNoUserInteractionBit		= 0;

	kICNoUserInteractionMask	= $00000001;

	kICFileType					= FourCharCode('ICAp');
	kICCreator					= FourCharCode('ICAp');

	{	***********************************************************************************************
	  Apple event constants
	 ***********************************************************************************************	}

	kInternetEventClass			= FourCharCode('GURL');
	kAEGetURL					= FourCharCode('GURL');
	kAEFetchURL					= FourCharCode('FURL');
	keyAEAttaching				= FourCharCode('Atch');

	{	 AERegistry.i defines a compatible keyAEDestination 	}

	kICEditPreferenceEventClass	= FourCharCode('ICAp');
	kICEditPreferenceEvent		= FourCharCode('ICAp');
	keyICEditPreferenceDestination = FourCharCode('dest');

	{	***********************************************************************************************
	  constants for use with ICGetVersion
	 ***********************************************************************************************	}
=======

const
	icPrefNotFoundErr			= -666;							{  preference not found (duh!)   }
	icPermErr					= -667;							{  cannot set preference   }
	icPrefDataErr				= -668;							{  problem with preference data   }
	icInternalErr				= -669;							{  hmm, this is not good   }
	icTruncatedErr				= -670;							{  more data was present than was returned   }
	icNoMoreWritersErr			= -671;							{  you cannot begin a write session because someone else is already doing it   }
	icNothingToOverrideErr		= -672;							{  no component for the override component to capture   }
	icNoURLErr					= -673;							{  no URL found   }
	icConfigNotFoundErr			= -674;							{  no configuration was found   }
	icConfigInappropriateErr	= -675;							{  incorrect manufacturer code   }
	icProfileNotFoundErr		= -676;							{  profile not found   }
	icTooManyProfilesErr		= -677;							{  too many profiles in database   }

	{	***********************************************************************************************
	  IC versions (not necessarily, but historically, from a component)
	 ***********************************************************************************************	}

	kICComponentInterfaceVersion0 = $00000000;					{  IC >= 1.0   }
	kICComponentInterfaceVersion1 = $00010000;					{  IC >= 1.1   }
	kICComponentInterfaceVersion2 = $00020000;					{  IC >= 1.2   }
	kICComponentInterfaceVersion3 = $00030000;					{  IC >= 2.0   }
	kICComponentInterfaceVersion4 = $00040000;					{  IC >= 2.5   }
	kICComponentInterfaceVersion = $00040000;					{  current version number is 4   }

	{	***********************************************************************************************
	  opaque type for preference reference
	 ***********************************************************************************************	}


type
	ICInstance    = ^SInt32; { an opaque 32-bit type }
	ICInstancePtr = ^ICInstance;  { when a var xx:ICInstance parameter can be nil, it is changed to xx: ICInstancePtr }

	{	***********************************************************************************************
	  a record that specifies a folder, an array of such records, and a pointer to such an array
	 ***********************************************************************************************	}
	ICDirSpecPtr = ^ICDirSpec;
	ICDirSpec = record
		vRefNum:				SInt16;
		dirID:					SInt32;
	end;

	ICDirSpecArray						= array [0..3] of ICDirSpec;
	ICDirSpecArrayPtr					= ^ICDirSpecArray;

	{	***********************************************************************************************
	  preference attributes type, bit number constants, and mask constants
	 ***********************************************************************************************	}
	ICAttr								= UInt32;


const
	kICAttrLockedBit			= 0;
	kICAttrVolatileBit			= 1;

	kICAttrNoChange				= $FFFFFFFF;					{  pass this to ICSetPref to tell it not to change the attributes   }
	kICAttrLockedMask			= $00000001;
	kICAttrVolatileMask			= $00000002;

	{	***********************************************************************************************
	  permissions for use with ICBegin
	 ***********************************************************************************************	}


type
	ICPerm								= UInt8;


const
	icNoPerm					= 0;
	icReadOnlyPerm				= 1;
	icReadWritePerm				= 2;

	{	***********************************************************************************************
	  a reference to an instance's current configuration
	 ***********************************************************************************************	}

{$ifc CALL_NOT_IN_CARBON}

type
	ICConfigRefPtr = ^ICConfigRef;
	ICConfigRef = record
		manufacturer:			OSType;
																		{  other private data follows   }
	end;

	ICConfigRefHandle					= ^ICConfigRefPtr;

{$endc}  {CALL_NOT_IN_CARBON}

{***********************************************************************************************
  profile IDs
 ***********************************************************************************************}

type
	ICProfileID							= SInt32;
	ICProfileIDPtr						= ^ICProfileID;


const
	kICNilProfileID				= 0;

	{	***********************************************************************************************
	  other constants
	 ***********************************************************************************************	}

	kICNoUserInteractionBit		= 0;

	kICNoUserInteractionMask	= $00000001;

	kICFileType					= FourCharCode('ICAp');
	kICCreator					= FourCharCode('ICAp');

	{	***********************************************************************************************
	  Apple event constants
	 ***********************************************************************************************	}

	kInternetEventClass			= FourCharCode('GURL');
	kAEGetURL					= FourCharCode('GURL');
	kAEFetchURL					= FourCharCode('FURL');
	keyAEAttaching				= FourCharCode('Atch');

	{	 AERegistry.i defines a compatible keyAEDestination 	}

	kICEditPreferenceEventClass	= FourCharCode('ICAp');
	kICEditPreferenceEvent		= FourCharCode('ICAp');
	keyICEditPreferenceDestination = FourCharCode('dest');

	{	***********************************************************************************************
	  constants for use with ICGetVersion
	 ***********************************************************************************************	}
>>>>>>> origin/fixes_2_2

	kICComponentVersion			= 0;							{  Return a component version, comparable to kICComponentInterfaceVersion   }
	kICNumVersion				= 1;							{  Return a NumVersion structure   }

	{	***********************************************************************************************
	  types and constants for use with kICDocumentFont, et. al.
	 ***********************************************************************************************	}

type
	ICFontRecordPtr = ^ICFontRecord;
	ICFontRecord = record
		size:					SInt16;
		face:					SInt8;
		pad:					SInt8;
		font:					Str255;
	end;

	ICFontRecordHandle					= ^ICFontRecordPtr;

	{	***********************************************************************************************
	  types and constants for use with kICCharacterSet, et. al.
	 ***********************************************************************************************	}
	ICCharTablePtr = ^ICCharTable;
	ICCharTable = record
		netToMac:				packed array [0..255] of UInt8;
		macToNet:				packed array [0..255] of UInt8;
	end;

	ICCharTableHandle					= ^ICCharTablePtr;

	{	***********************************************************************************************
	  types and constants for use with kICHelper, et. al.
	 ***********************************************************************************************	}
	ICAppSpecPtr = ^ICAppSpec;
	ICAppSpec = record
		fCreator:				OSType;
		name:					Str63;
	end;

	ICAppSpecHandle						= ^ICAppSpecPtr;
	ICAppSpecListPtr = ^ICAppSpecList;
	ICAppSpecList = record
		numberOfItems:			SInt16;
		appSpecs:				array [0..0] of ICAppSpec;
	end;

	ICAppSpecListHandle					= ^ICAppSpecListPtr;

	{	***********************************************************************************************
	  types and constants for use with kICDownloadFolder, et. al.
	 ***********************************************************************************************	}

{$ifc NOT OLDROUTINENAMES}
	ICFileSpecPtr = ^ICFileSpec;
	ICFileSpec = record
		volName:				Str31;
		volCreationDate:		SInt32;
		fss:					FSSpec;
		alias:					AliasRecord;
																		{  plus extra data, aliasSize 0 means no alias manager present when }
																		{  ICFileSpecification was created }
	end;

	ICFileSpecHandle					= ^ICFileSpecPtr;
{$elsec}
	ICFileSpecPtr = ^ICFileSpec;
	ICFileSpec = record
		vol_name:				Str31;
		vol_creation_date:		SInt32;
		fss:					FSSpec;
		alias:					AliasRecord;
	end;

	ICFileSpecHandle					= ^ICFileSpecPtr;
{$endc}


const
	kICFileSpecHeaderSize		= 106;

	{	***********************************************************************************************
	  types and constants for use with ICMapFilename, et. al.
	 ***********************************************************************************************	}

type
	ICMapEntryFlags						= SInt32;
	ICFixedLength						= SInt16;

{$ifc NOT OLDROUTINENAMES}
	ICMapEntryPtr = ^ICMapEntry;
	ICMapEntry = record
		totalLength:			SInt16;
		fixedLength:			ICFixedLength;
		version:				SInt16;
		fileType:				OSType;
		fileCreator:			OSType;
		postCreator:			OSType;
		flags:					ICMapEntryFlags;
																		{  variable part starts here }
		extension:				Str255;
		creatorAppName:			Str255;
		postAppName:			Str255;
		MIMEType:				Str255;
		entryName:				Str255;
	end;

	ICMapEntryHandle					= ^ICMapEntryPtr;

{$elsec}
	ICMapEntryPtr = ^ICMapEntry;
	ICMapEntry = record
		total_length:			SInt16;
		fixed_length:			ICFixedLength;
		version:				SInt16;
		file_type:				OSType;
		file_creator:			OSType;
		post_creator:			OSType;
		flags:					ICMapEntryFlags;
		extension:				Str255;
		creator_app_name:		Str255;
		post_app_name:			Str255;
		MIME_type:				Str255;
		entry_name:				Str255;
	end;

	ICMapEntryHandle					= ^ICMapEntryPtr;
{$endc}


const
	kICMapFixedLength			= 22;							{  number in fixedLength field }

	kICMapBinaryBit				= 0;							{  file should be transfered in binary as opposed to text mode }
	kICMapResourceForkBit		= 1;							{  the resource fork of the file is significant }
	kICMapDataForkBit			= 2;							{  the data fork of the file is significant }
	kICMapPostBit				= 3;							{  post process using post fields }
	kICMapNotIncomingBit		= 4;							{  ignore this mapping for incoming files }
	kICMapNotOutgoingBit		= 5;							{  ignore this mapping for outgoing files }

	kICMapBinaryMask			= $00000001;					{  file should be transfered in binary as opposed to text mode }
	kICMapResourceForkMask		= $00000002;					{  the resource fork of the file is significant }
	kICMapDataForkMask			= $00000004;					{  the data fork of the file is significant }
	kICMapPostMask				= $00000008;					{  post process using post fields }
	kICMapNotIncomingMask		= $00000010;					{  ignore this mapping for incoming files }
	kICMapNotOutgoingMask		= $00000020;					{  ignore this mapping for outgoing files }

	{	***********************************************************************************************
	  types and constants for use with kICServices, et. al.
	 ***********************************************************************************************	}

type
	ICServiceEntryFlags					= SInt16;
	ICServiceEntryPtr = ^ICServiceEntry;
	ICServiceEntry = record
		name:					Str255;
		port:					SInt16;
		flags:					ICServiceEntryFlags;
	end;

	ICServiceEntryHandle				= ^ICServiceEntryPtr;


const
	kICServicesTCPBit			= 0;
	kICServicesUDPBit			= 1;							{  both bits can be set, which means the service is both TCP and UDP, eg daytime }

	kICServicesTCPMask			= $00000001;
	kICServicesUDPMask			= $00000002;					{  both bits can be set, which means the service is both TCP and UDP, eg daytime }


type
	ICServicesPtr = ^ICServices;
	ICServices = record
		count:					SInt16;
		services:				array [0..0] of ICServiceEntry;
	end;

	ICServicesHandle					= ^ICServicesPtr;

	{	***********************************************************************************************
	  default file name, for internal use, overridden by a component resource
	 ***********************************************************************************************	}

{$ifc CALL_NOT_IN_CARBON}

const
	kICDefaultFileName			= 'Internet Preferences';
{$endc}  {CALL_NOT_IN_CARBON}

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{***********************************************************************************************
  keys
 ***********************************************************************************************}
{ 
    key reserved for use by Internet Config 
}
<<<<<<< HEAD
<<<<<<< HEAD
const
	kICReservedKey = 'kICReservedKey';
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
{
    STR# -- formatted, list of Archie servers  
}
const
	kICArchieAll = 'ArchieAll';
<<<<<<< HEAD
{
    PString -- formatted, preferred Archie server   
}
const
	kICArchiePreferred = 'ArchiePreferred';
{
    ICCharTable -- Mac-to-Net and Net-to-Mac character translation   
}
const
	kICCharacterSet = 'CharacterSet';
{
    ICFontRecord -- font used for proportional text   
}
const
	kICDocumentFont = 'DocumentFont';
{
    ICFileSpec -- where to put newly downloaded files   
}
const
	kICDownloadFolder = 'DownloadFolder';
{
    PString -- user@host.domain, email address of user, ie return address   
}
const
	kICEmail = 'Email';
{
    PString -- host.domain, default FTP server   
}
const
	kICFTPHost = 'FTPHost';
{
    PString -- second level FTP proxy authorisation   
}
const
	kICFTPProxyAccount = 'FTPProxyAccount';
{
    PString -- host.domain   
}
const
	kICFTPProxyHost = 'FTPProxyHost';
{
    PString -- scrambled, password for FTPProxyUser   
}
const
	kICFTPProxyPassword = 'FTPProxyPassword';
{
    PString -- first level FTP proxy authorisation   
}
const
	kICFTPProxyUser = 'FTPProxyUser';
{
    PString -- host.domain, default finger server   
}
const
	kICFingerHost = 'FingerHost';
{
    PString -- host.domain, default Gopher server   
}
const
	kICGopherHost = 'GopherHost';
{
    PString -- host.domain, see note in Prog Docs   
}
const
	kICGopherProxy = 'GopherProxy';
{
    PString -- host.domain   
}
const
	kICHTTPProxyHost = 'HTTPProxyHost';
{
    ICAppSpec -- helpers for URL schemes   
}
const
	kICHelper = 'Helper�';
{
    PString -- description for URL scheme   
}
const
	kICHelperDesc = 'HelperDesc�';
<<<<<<< HEAD
{
    ICAppSpecList -- list of common helpers for URL schemes   
}
const
	kICHelperList = 'HelperList�';
{
    PString -- host.domain, Internet Relay Chat server   
}
const
	kICIRCHost = 'IRCHost';
{
    STR# -- formatted, list of Info-Mac servers   
}
const
	kICInfoMacAll = 'InfoMacAll';
{
    PString -- formatted, preferred Info-Mac server   
}
const
	kICInfoMacPreferred = 'InfoMacPreferred';
{
    PString -- string LDAP thing   
}
const
	kICLDAPSearchbase = 'LDAPSearchbase';
{
    PString -- host.domain   
}
const
	kICLDAPServer = 'LDAPServer';
{
    ICFontRecord -- font used for lists of items (eg news article lists)   
}
const
	kICListFont = 'ListFont';
{
    PString -- host for MacSearch queries   
}
const
	kICMacSearchHost = 'MacSearchHost';
{
    PString -- user@host.domain, account from which to fetch mail   
}
const
	kICMailAccount = 'MailAccount';
{
    TEXT -- extra headers for mail messages   
}
const
	kICMailHeaders = 'MailHeaders';
{
    PString -- scrambled, password for MailAccount   
}
const
	kICMailPassword = 'MailPassword';
{
    ICMapEntries -- file type mapping, see documentation   
}
const
	kICMapping = 'Mapping';
{
    PString -- host.domain, NNTP server   
}
const
	kICNNTPHost = 'NNTPHost';
<<<<<<< HEAD
{
    PString -- host.domain, Network Time Protocol (NTP)   
}
const
	kICNTPHost = 'NTPHost';
{
    Boolean   
}
const
	kICNewMailDialog = 'NewMailDialog';
{
=======
{
    STR# -- formatted, list of Archie servers  
}
const
	kICArchieAll = 'ArchieAll';
{
    PString -- formatted, preferred Archie server   
}
const
	kICArchiePreferred = 'ArchiePreferred';
{
    ICCharTable -- Mac-to-Net and Net-to-Mac character translation   
}
const
	kICCharacterSet = 'CharacterSet';
{
    ICFontRecord -- font used for proportional text   
}
const
	kICDocumentFont = 'DocumentFont';
{
    ICFileSpec -- where to put newly downloaded files   
}
const
	kICDownloadFolder = 'DownloadFolder';
{
    PString -- user@host.domain, email address of user, ie return address   
}
const
	kICEmail = 'Email';
{
    PString -- host.domain, default FTP server   
}
const
	kICFTPHost = 'FTPHost';
{
    PString -- second level FTP proxy authorisation   
}
const
	kICFTPProxyAccount = 'FTPProxyAccount';
{
    PString -- host.domain   
}
const
	kICFTPProxyHost = 'FTPProxyHost';
{
    PString -- scrambled, password for FTPProxyUser   
}
const
	kICFTPProxyPassword = 'FTPProxyPassword';
{
    PString -- first level FTP proxy authorisation   
}
const
	kICFTPProxyUser = 'FTPProxyUser';
{
    PString -- host.domain, default finger server   
}
const
	kICFingerHost = 'FingerHost';
{
    PString -- host.domain, default Gopher server   
}
const
	kICGopherHost = 'GopherHost';
{
    PString -- host.domain, see note in Prog Docs   
}
const
	kICGopherProxy = 'GopherProxy';
{
    PString -- host.domain   
}
const
	kICHTTPProxyHost = 'HTTPProxyHost';
{
    ICAppSpec -- helpers for URL schemes   
}
const
	kICHelper = 'Helper�';
{
    PString -- description for URL scheme   
}
const
	kICHelperDesc = 'HelperDesc�';
{
    ICAppSpecList -- list of common helpers for URL schemes   
}
const
	kICHelperList = 'HelperList�';
{
    PString -- host.domain, Internet Relay Chat server   
}
const
	kICIRCHost = 'IRCHost';
{
    STR# -- formatted, list of Info-Mac servers   
}
const
	kICInfoMacAll = 'InfoMacAll';
{
    PString -- formatted, preferred Info-Mac server   
}
const
	kICInfoMacPreferred = 'InfoMacPreferred';
{
    PString -- string LDAP thing   
}
const
	kICLDAPSearchbase = 'LDAPSearchbase';
{
    PString -- host.domain   
}
const
	kICLDAPServer = 'LDAPServer';
{
    ICFontRecord -- font used for lists of items (eg news article lists)   
}
const
	kICListFont = 'ListFont';
{
    PString -- host for MacSearch queries   
}
const
	kICMacSearchHost = 'MacSearchHost';
{
    PString -- user@host.domain, account from which to fetch mail   
}
const
	kICMailAccount = 'MailAccount';
{
    TEXT -- extra headers for mail messages   
}
const
	kICMailHeaders = 'MailHeaders';
{
    PString -- scrambled, password for MailAccount   
}
const
	kICMailPassword = 'MailPassword';
{
    ICMapEntries -- file type mapping, see documentation   
}
const
	kICMapping = 'Mapping';
{
    PString -- host.domain, NNTP server   
}
const
	kICNNTPHost = 'NNTPHost';
{
    PString -- host.domain, Network Time Protocol (NTP)   
}
const
	kICNTPHost = 'NTPHost';
{
    Boolean   
}
const
	kICNewMailDialog = 'NewMailDialog';
{
>>>>>>> graemeg/cpstrnew
    Boolean -- how to announce new mail   
}
const
	kICNewMailFlashIcon = 'NewMailFlashIcon';
<<<<<<< HEAD
{
    Boolean   
}
const
	kICNewMailPlaySound = 'NewMailPlaySound';
{
    PString   
}
const
	kICNewMailSoundName = 'NewMailSoundName';
{
    PString -- scrambled, password for NewsAuthUsername   
}
const
	kICNewsAuthPassword = 'NewsAuthPassword';
{
    PString -- user name for authorised news servers   
}
const
	kICNewsAuthUsername = 'NewsAuthUsername';
{
    TEXT -- extra headers for news messages   
}
const
	kICNewsHeaders = 'NewsHeaders';
{
    STR# -- list of domains not to be proxied   
}
const
	kICNoProxyDomains = 'NoProxyDomains';
{
    PString -- for X-Organization string   
}
const
	kICOrganization = 'Organization';
{
    PString -- host.domain, default Ph server   
}
const
	kICPhHost = 'PhHost';
{
    TEXT -- default response for finger servers   
}
const
	kICPlan = 'Plan';
{
    ICFontRecord -- font used to print ScreenFont   
}
const
	kICPrinterFont = 'PrinterFont';
{
    PString -- used to quote responses in news and mail   
}
const
	kICQuotingString = 'QuotingString';
{
    PString -- real name of user   
}
const
	kICRealName = 'RealName';
{
    PString -- RTSP Proxy Host
}
const
	kICRTSPProxyHost = 'RTSPProxyHost';
{
    PString -- host.domain, SMTP server   
}
const
	kICSMTPHost = 'SMTPHost';
{
    ICFontRecord -- font used for monospaced text (eg news articles)   
}
const
	kICScreenFont = 'ScreenFont';
{
    ICServices -- TCP and IP port-to-name mapping   
}
const
	kICServices = 'Services';
{
    TEXT -- append to news and mail messages   
}
const
	kICSignature = 'Signature';
{
    TEXT -- preferred mailing address   
}
const
	kICSnailMailAddress = 'SnailMailAddress';
{
    PString -- host.domain, remember that host.domain format allows ":port" and " port"  
}
const
	kICSocksHost = 'SocksHost';
{
    PString -- host.domain, default Telnet address   
}
const
	kICTelnetHost = 'TelnetHost';
{
    STR# -- formatted, list of UMich servers   
}
const
	kICUMichAll = 'UMichAll';
{
    PString -- formatted, preferred UMich server   
}
const
	kICUMichPreferred = 'UMichPreferred';
{
    Boolean   
}
const
	kICUseFTPProxy = 'UseFTPProxy';
{
    Boolean   
}
const
	kICUseGopherProxy = 'UseGopherProxy';
{
    Boolean   
}
const
	kICUseHTTPProxy = 'UseHTTPProxy';
{
    Boolean -- use PASV command for FTP transfers   
}
const
	kICUsePassiveFTP = 'UsePassiveFTP';
{
    Boolean
}
const
	kICUseRTSPProxy = 'UseRTSPProxy';
{
    Boolean   
}
const
	kICUseSocks = 'UseSocks';
{
    PString -- no idea   
}
const
	kICWAISGateway = 'WAISGateway';
{
    PString -- URL, users default WWW page   
}
const
	kICWWWHomePage = 'WWWHomePage';
{
    RGBColor -- background colour for web pages   
}
const
	kICWebBackgroundColour = 'WebBackgroundColour';
{
    RGBColor -- colour for read links   
}
const
	kICWebReadColor = '646F6777쩦ebReadColor';
{
    PString -- URL, users default search page   
}
const
	kICWebSearchPagePrefs = 'WebSearchPagePrefs';
{
    RGBColor -- colour for normal text   
}
const
	kICWebTextColor = 'WebTextColor';
{
    Boolean -- whether to underline links   
}
const
	kICWebUnderlineLinks = '646F6777쩦ebUnderlineLinks';
{
    RGBColor -- colour for unread links   
}
const
	kICWebUnreadColor = '646F6777쩦ebUnreadColor';
{
    PString -- host.domain, default whois server   
}
const
	kICWhoisHost = 'WhoisHost';

{***********************************************************************************************

      FUNCTIONS

      What do the annotations after each API mean?
      --------------------------------------------

      [r1] Requires IC 1.1 or higher.
      [r2] Requires IC 1.2 or higher.
      [r3] Requires IC 2.0 or higher.
      [r4] Requires IC 2.5 or higher.
      
      IMPORTANT:

      In IC 2.5, instances automatically use the default configuration.
      You no longer need to configure an instance explicitly, except
      if your code might run with an older version of IC.  So the following
      notes only apply to IC 2.0 and earlier.

      [c1]  You must have specified a configuration before calling this routine.
      
      [c2]  You must have specified the default configuration before calling this
            routine.
      
      [c3]  You do not need to specify a configuration before calling this routine.
      
      [b1]  You must be inside a Begin/End pair when calling this routine.
      
      [b2]  You must be inside a Begin/End read/write pair when calling this routine.
      
      [b3]  You do not need to be inside a Begin/End pair when calling this routine.
      
      [b4]  If you are getting or setting multiple preferences, you should make this
            call inside a Begin/End pair. If you do not make this call inside a Begin/End
            pair, the call will automatically do it for you.
      
      [b5]  It is illegal to call this routine inside a Begin/End pair.

 ***********************************************************************************************}

{ ***** Starting Up and Shutting Down *****  }
{
 *  ICStart()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.7
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICStart( var inst: ICInstance; signature: OSType ): OSStatus; external name '_ICStart';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)


{ Call this at application initialisation. Set signature to a value
   * which has been regsitered with DTS to allow for future expansion
   * of the IC system. Returns inst as a connection to the IC system.
   }
{
 *  ICStop()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.7
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICStop( inst: ICInstance ): OSStatus; external name '_ICStop';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)


{ [b5] 
   * Call this at application initialisation, after which inst
   * is no longer valid connection to IC.
   }
{
 *  ICGetVersion()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.7
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetVersion( inst: ICInstance; whichVersion: SIGNEDLONG; var version: UInt32 ): OSStatus; external name '_ICGetVersion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)


{ [r4] [c3] [b3] 
   * Returns the version of Internet Config.  Pass kICComponentVersion
   * to get the version as previously returned by GetComponenVerson.
   * Pass kICNumVersion to get a NumVersion structure.
   }
{
 *  ICGetConfigName()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.7
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetConfigName( inst: ICInstance; longname: Boolean; var name: Str255 ): OSStatus; external name '_ICGetConfigName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)


{ [r2] [c1] [b3] 
   * Returns a string that describes the current configuration at a user
   * level. Set longname to true if you want a long name, up to 255
   * characters, or false if you want a short name, typically about 32
   * characters.
   * The returned string is for user display only. If you rely on the
   * exact format of it, you will conflict with any future IC
   * implementation that doesn't use explicit preference files.
   }
{ ***** Getting Information *****  }
{
 *  ICGetSeed()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.7
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetSeed( inst: ICInstance; var seed: SIGNEDLONG ): OSStatus; external name '_ICGetSeed';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)


{ [c3] [b3] 
   * Returns the current seed for the IC prefs database.
   * This seed changes each time a non-volatile preference is changed.
   * You can poll this to determine if any cached preferences change.
   }
{
 *  ICGetPerm()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.7
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetPerm( inst: ICInstance; var perm: ICPerm ): OSStatus; external name '_ICGetPerm';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)


{ [c3] [b3] 
   * Returns the access permissions currently associated with this instance.
   * While applications normally know what permissions they have,
   * this routine is designed for use by override components.
   }
{ ***** Reading and Writing Preferences *****  }
{
 *  ICBegin()   *** DEPRECATED ***
=======
{
    Boolean   
}
const
	kICNewMailPlaySound = 'NewMailPlaySound';
{
=======
{
    ICAppSpecList -- list of common helpers for URL schemes   
}
const
	kICHelperList = 'HelperList�';
{
    PString -- host.domain, Internet Relay Chat server   
}
const
	kICIRCHost = 'IRCHost';
{
    STR# -- formatted, list of Info-Mac servers   
}
const
	kICInfoMacAll = 'InfoMacAll';
{
    PString -- formatted, preferred Info-Mac server   
}
const
	kICInfoMacPreferred = 'InfoMacPreferred';
{
    PString -- string LDAP thing   
}
const
	kICLDAPSearchbase = 'LDAPSearchbase';
{
    PString -- host.domain   
}
const
	kICLDAPServer = 'LDAPServer';
{
    ICFontRecord -- font used for lists of items (eg news article lists)   
}
const
	kICListFont = 'ListFont';
{
    PString -- host for MacSearch queries   
}
const
	kICMacSearchHost = 'MacSearchHost';
{
    PString -- user@host.domain, account from which to fetch mail   
}
const
	kICMailAccount = 'MailAccount';
{
    TEXT -- extra headers for mail messages   
}
const
	kICMailHeaders = 'MailHeaders';
{
    PString -- scrambled, password for MailAccount   
}
const
	kICMailPassword = 'MailPassword';
{
    ICMapEntries -- file type mapping, see documentation   
}
const
	kICMapping = 'Mapping';
{
    PString -- host.domain, NNTP server   
}
const
	kICNNTPHost = 'NNTPHost';
{
    PString -- host.domain, Network Time Protocol (NTP)   
}
const
	kICNTPHost = 'NTPHost';
{
    Boolean   
}
const
	kICNewMailDialog = 'NewMailDialog';
{
    Boolean -- how to announce new mail   
}
const
	kICNewMailFlashIcon = 'NewMailFlashIcon';
{
    Boolean   
}
const
	kICNewMailPlaySound = 'NewMailPlaySound';
{
>>>>>>> graemeg/cpstrnew
    PString   
}
const
	kICNewMailSoundName = 'NewMailSoundName';
<<<<<<< HEAD
{
    PString -- scrambled, password for NewsAuthUsername   
}
const
	kICNewsAuthPassword = 'NewsAuthPassword';
{
    PString -- user name for authorised news servers   
}
const
	kICNewsAuthUsername = 'NewsAuthUsername';
{
    TEXT -- extra headers for news messages   
}
const
	kICNewsHeaders = 'NewsHeaders';
{
=======
{
    PString -- host.domain, Network Time Protocol (NTP)   
}
const
	kICNTPHost = 'NTPHost';
{
    Boolean   
}
const
	kICNewMailDialog = 'NewMailDialog';
{
    Boolean -- how to announce new mail   
}
const
	kICNewMailFlashIcon = 'NewMailFlashIcon';
{
    Boolean   
}
const
	kICNewMailPlaySound = 'NewMailPlaySound';
{
    PString   
}
const
	kICNewMailSoundName = 'NewMailSoundName';
{
    PString -- scrambled, password for NewsAuthUsername   
}
const
	kICNewsAuthPassword = 'NewsAuthPassword';
{
    PString -- user name for authorised news servers   
}
const
	kICNewsAuthUsername = 'NewsAuthUsername';
{
    TEXT -- extra headers for news messages   
}
const
	kICNewsHeaders = 'NewsHeaders';
{
>>>>>>> graemeg/cpstrnew
    STR# -- list of domains not to be proxied   
}
const
	kICNoProxyDomains = 'NoProxyDomains';
{
    PString -- for X-Organization string   
}
const
	kICOrganization = 'Organization';
{
    PString -- host.domain, default Ph server   
}
const
	kICPhHost = 'PhHost';
{
    TEXT -- default response for finger servers   
}
const
	kICPlan = 'Plan';
{
    ICFontRecord -- font used to print ScreenFont   
}
const
	kICPrinterFont = 'PrinterFont';
{
    PString -- used to quote responses in news and mail   
}
const
	kICQuotingString = 'QuotingString';
{
    PString -- real name of user   
}
const
	kICRealName = 'RealName';
<<<<<<< HEAD
{
    PString -- RTSP Proxy Host
}
const
	kICRTSPProxyHost = 'RTSPProxyHost';
{
    PString -- host.domain, SMTP server   
}
const
	kICSMTPHost = 'SMTPHost';
{
    ICFontRecord -- font used for monospaced text (eg news articles)   
}
const
	kICScreenFont = 'ScreenFont';
{
    ICServices -- TCP and IP port-to-name mapping   
}
const
	kICServices = 'Services';
{
    TEXT -- append to news and mail messages   
}
const
	kICSignature = 'Signature';
{
    TEXT -- preferred mailing address   
}
const
	kICSnailMailAddress = 'SnailMailAddress';
{
    PString -- host.domain, remember that host.domain format allows ":port" and " port"  
}
const
	kICSocksHost = 'SocksHost';
{
    PString -- host.domain, default Telnet address   
}
const
	kICTelnetHost = 'TelnetHost';
{
    STR# -- formatted, list of UMich servers   
}
const
	kICUMichAll = 'UMichAll';
{
    PString -- formatted, preferred UMich server   
}
const
	kICUMichPreferred = 'UMichPreferred';
{
    Boolean   
}
const
	kICUseFTPProxy = 'UseFTPProxy';
{
    Boolean   
}
const
	kICUseGopherProxy = 'UseGopherProxy';
{
    Boolean   
}
const
	kICUseHTTPProxy = 'UseHTTPProxy';
{
    Boolean -- use PASV command for FTP transfers   
}
const
	kICUsePassiveFTP = 'UsePassiveFTP';
{
    Boolean
}
const
	kICUseRTSPProxy = 'UseRTSPProxy';
{
    Boolean   
}
const
	kICUseSocks = 'UseSocks';
{
    PString -- no idea   
}
const
	kICWAISGateway = 'WAISGateway';
{
    PString -- URL, users default WWW page   
}
const
	kICWWWHomePage = 'WWWHomePage';
{
    RGBColor -- background colour for web pages   
}
const
	kICWebBackgroundColour = 'WebBackgroundColour';
{
    RGBColor -- colour for read links   
}
const
	kICWebReadColor = '646F6777쩦ebReadColor';
{
    PString -- URL, users default search page   
}
const
	kICWebSearchPagePrefs = 'WebSearchPagePrefs';
{
    RGBColor -- colour for normal text   
}
const
	kICWebTextColor = 'WebTextColor';
{
    Boolean -- whether to underline links   
}
const
	kICWebUnderlineLinks = '646F6777쩦ebUnderlineLinks';
{
    RGBColor -- colour for unread links   
}
const
	kICWebUnreadColor = '646F6777쩦ebUnreadColor';
{
    PString -- host.domain, default whois server   
}
const
	kICWhoisHost = 'WhoisHost';

{***********************************************************************************************

      FUNCTIONS

      What do the annotations after each API mean?
      --------------------------------------------

      [r1] Requires IC 1.1 or higher.
      [r2] Requires IC 1.2 or higher.
      [r3] Requires IC 2.0 or higher.
      [r4] Requires IC 2.5 or higher.
      
      IMPORTANT:

      In IC 2.5, instances automatically use the default configuration.
      You no longer need to configure an instance explicitly, except
      if your code might run with an older version of IC.  So the following
      notes only apply to IC 2.0 and earlier.

      [c1]  You must have specified a configuration before calling this routine.
      
      [c2]  You must have specified the default configuration before calling this
            routine.
      
      [c3]  You do not need to specify a configuration before calling this routine.
      
      [b1]  You must be inside a Begin/End pair when calling this routine.
      
      [b2]  You must be inside a Begin/End read/write pair when calling this routine.
      
      [b3]  You do not need to be inside a Begin/End pair when calling this routine.
      
      [b4]  If you are getting or setting multiple preferences, you should make this
            call inside a Begin/End pair. If you do not make this call inside a Begin/End
            pair, the call will automatically do it for you.
      
      [b5]  It is illegal to call this routine inside a Begin/End pair.

 ***********************************************************************************************}

{ ***** Starting Up and Shutting Down *****  }
{
 *  ICStart()
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.7
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICBegin( inst: ICInstance; perm: ICPerm ): OSStatus; external name '_ICBegin';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)


{ [c1] [b5] 
   * Starting reading or writing multiple preferences.
   * A call to this must be balanced by a call to ICEnd.
   * Do not call WaitNextEvent between these calls.
   * The perm specifies whether you intend to read or read/write.
   * Only one writer is allowed per instance.
   * Note that this may open resource files that are not closed
   * until you call ICEnd.
   }
{
 *  ICGetPref()   *** DEPRECATED ***
 *  
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICStart( var inst: ICInstance; signature: OSType ): OSStatus; external name '_ICStart';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Call this at application initialisation. Set signature to a value
   * which has been regsitered with DTS to allow for future expansion
   * of the IC system. Returns inst as a connection to the IC system.
   }
{
 *  ICStop()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICStop( inst: ICInstance ): OSStatus; external name '_ICStop';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [b5] 
   * Call this at application initialisation, after which inst
   * is no longer valid connection to IC.
   }
{
 *  ICGetVersion()
 *  
>>>>>>> graemeg/cpstrnew
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.7
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetPref( inst: ICInstance; const (*var*) key: Str255; var attr: ICAttr; buf: UnivPtr; var size: SIGNEDLONG ): OSStatus; external name '_ICGetPref';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)


{ [c1] [b4] 
   * Reads the preference specified by key from the IC database to the
   * buffer pointed to by buf and size.
   * key must not be the empty string.
   * If buf is nil then no data is returned.
   * size must be non-negative.
   * attr and size are always set on return. On errors (except icTruncatedErr)
   * attr is set to ICattr_no_change and size is set to 0.
   * size is the actual size of the data.
   * attr is set to the attributes associated with the preference.
   * If this routine returns icTruncatedErr then the other returned
   * values are valid except that only the first size bytes have been
   * return. size is adjusted to reflect the true size of the preference.
   * Returns icPrefNotFound if there is no preference for the key.
   }
{
 *  ICSetPref()   *** DEPRECATED ***
 *  
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetVersion( inst: ICInstance; whichVersion: SIGNEDLONG; var version: UInt32 ): OSStatus; external name '_ICGetVersion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r4] [c3] [b3] 
   * Returns the version of Internet Config.  Pass kICComponentVersion
   * to get the version as previously returned by GetComponenVerson.
   * Pass kICNumVersion to get a NumVersion structure.
   }
{
 *  ICGetConfigName()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetConfigName( inst: ICInstance; longname: Boolean; var name: Str255 ): OSStatus; external name '_ICGetConfigName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r2] [c1] [b3] 
   * Returns a string that describes the current configuration at a user
   * level. Set longname to true if you want a long name, up to 255
   * characters, or false if you want a short name, typically about 32
   * characters.
   * The returned string is for user display only. If you rely on the
   * exact format of it, you will conflict with any future IC
   * implementation that doesn't use explicit preference files.
   }
{ ***** Getting Information *****  }
{
 *  ICGetSeed()
 *  
>>>>>>> graemeg/cpstrnew
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.7
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICSetPref( inst: ICInstance; const (*var*) key: Str255; attr: ICAttr; buf: {const} UnivPtr; size: SIGNEDLONG ): OSStatus; external name '_ICSetPref';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)


{ [c1] [b4] 
   * Sets the preference specified by key from the IC database to the
   * value pointed to by buf and size.
   * key must not be the empty string.
   * size must be non-negative. 
   * If buf is nil then the preference value is not set and size is ignored.
   * If buf is not nil then the preference value is set to the size
   * bytes pointed to by buf.
   * If attr is ICattr_no_change then the preference attributes are not set.
   * Otherwise the preference attributes are set to attr.
   * Returns icPermErr if the previous ICBegin was passed icReadOnlyPerm.
   * Returns icPermErr if current attr is locked, new attr is locked and buf <> nil.
   }
{
 *  ICFindPrefHandle()   *** DEPRECATED ***
 *  
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetSeed( inst: ICInstance; var seed: SIGNEDLONG ): OSStatus; external name '_ICGetSeed';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [c3] [b3] 
   * Returns the current seed for the IC prefs database.
   * This seed changes each time a non-volatile preference is changed.
   * You can poll this to determine if any cached preferences change.
   }
{
 *  ICGetPerm()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetPerm( inst: ICInstance; var perm: ICPerm ): OSStatus; external name '_ICGetPerm';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [c3] [b3] 
   * Returns the access permissions currently associated with this instance.
   * While applications normally know what permissions they have,
   * this routine is designed for use by override components.
   }
{ ***** Reading and Writing Preferences *****  }
{
 *  ICBegin()
 *  
>>>>>>> graemeg/cpstrnew
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.7
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICFindPrefHandle( inst: ICInstance; const (*var*) key: Str255; var attr: ICAttr; prefh: Handle ): OSStatus; external name '_ICFindPrefHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)


{ [r2] [c1] [b4] 
   * This routine effectively replaces ICGetPrefHandle.
   * Reads the preference specified by key from the IC database into
   * a handle, prefh.
   * key must not be the empty string.
   * attr is set to the attributes associated with the preference.
   * You must set prefh to a non-nil handle before calling this routine.
   * If the preference does not exist, icPrefNotFoundErr is returned.
   }
{
 *  ICGetPrefHandle()   *** DEPRECATED ***
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICBegin( inst: ICInstance; perm: ICPerm ): OSStatus; external name '_ICBegin';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [c1] [b5] 
   * Starting reading or writing multiple preferences.
   * A call to this must be balanced by a call to ICEnd.
   * Do not call WaitNextEvent between these calls.
   * The perm specifies whether you intend to read or read/write.
   * Only one writer is allowed per instance.
   * Note that this may open resource files that are not closed
   * until you call ICEnd.
   }
{
 *  ICGetPref()
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.7
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetPrefHandle( inst: ICInstance; const (*var*) key: Str255; var attr: ICAttr; var prefh: Handle ): OSStatus; external name '_ICGetPrefHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)


{ [r1] [c1] [b4] 
   * This routine is now obsolete. Use ICFindPrefHandle instead.
   * Reads the preference specified by key from the IC database into
   * a newly created handle, prefh.
   * key must not be the empty string.
   * attr is set to the attributes associated with the preference.
   * The incoming value of prefh is ignored.
   * A new handle is created in the current heap and returned in prefh.
   * If the routine returns an error, prefh is set to nil.
   * If the preference does not exist, no error is returned and prefh is set
   * to an empty handle.
   }
{
 *  ICSetPrefHandle()   *** DEPRECATED ***
 *  
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetPref( inst: ICInstance; const (*var*) key: Str255; var attr: ICAttr; buf: UnivPtr; var size: SIGNEDLONG ): OSStatus; external name '_ICGetPref';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [c1] [b4] 
   * Reads the preference specified by key from the IC database to the
   * buffer pointed to by buf and size.
   * key must not be the empty string.
   * If buf is nil then no data is returned.
   * size must be non-negative.
   * attr and size are always set on return. On errors (except icTruncatedErr)
   * attr is set to ICattr_no_change and size is set to 0.
   * size is the actual size of the data.
   * attr is set to the attributes associated with the preference.
   * If this routine returns icTruncatedErr then the other returned
   * values are valid except that only the first size bytes have been
   * return. size is adjusted to reflect the true size of the preference.
   * Returns icPrefNotFound if there is no preference for the key.
   }
{
 *  ICSetPref()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICSetPref( inst: ICInstance; const (*var*) key: Str255; attr: ICAttr; buf: {const} UnivPtr; size: SIGNEDLONG ): OSStatus; external name '_ICSetPref';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [c1] [b4] 
   * Sets the preference specified by key from the IC database to the
   * value pointed to by buf and size.
   * key must not be the empty string.
   * size must be non-negative. 
   * If buf is nil then the preference value is not set and size is ignored.
   * If buf is not nil then the preference value is set to the size
   * bytes pointed to by buf.
   * If attr is ICattr_no_change then the preference attributes are not set.
   * Otherwise the preference attributes are set to attr.
   * Returns icPermErr if the previous ICBegin was passed icReadOnlyPerm.
   * Returns icPermErr if current attr is locked, new attr is locked and buf <> nil.
   }
{
 *  ICFindPrefHandle()
 *  
>>>>>>> graemeg/cpstrnew
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.7
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICSetPrefHandle( inst: ICInstance; const (*var*) key: Str255; attr: ICAttr; prefh: Handle ): OSStatus; external name '_ICSetPrefHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)


{ [r1] [c1] [b4] 
   * Sets the preference specified by key from the IC database to the
   * value contained in prefh.
   * key must not be the empty string.
   * If prefh is nil then the preference value is not set.
   * If prefh is not nil then the preference value is set to the data
   * contained in it.
   * If attr is ICattr_no_change then the preference attributes are not set.
   * Otherwise the preference attributes are set to attr.
   * Returns icPermErr if the previous ICBegin was passed icReadOnlyPerm.
   * Returns icPermErr if current attr is locked, new attr is locked and prefh <> nil.
   }
{
 *  ICCountPref()   *** DEPRECATED ***
 *  
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICFindPrefHandle( inst: ICInstance; const (*var*) key: Str255; var attr: ICAttr; prefh: Handle ): OSStatus; external name '_ICFindPrefHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r2] [c1] [b4] 
   * This routine effectively replaces ICGetPrefHandle.
   * Reads the preference specified by key from the IC database into
   * a handle, prefh.
   * key must not be the empty string.
   * attr is set to the attributes associated with the preference.
   * You must set prefh to a non-nil handle before calling this routine.
   * If the preference does not exist, icPrefNotFoundErr is returned.
   }
{
 *  ICGetPrefHandle()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetPrefHandle( inst: ICInstance; const (*var*) key: Str255; var attr: ICAttr; var prefh: Handle ): OSStatus; external name '_ICGetPrefHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r1] [c1] [b4] 
   * This routine is now obsolete. Use ICFindPrefHandle instead.
   * Reads the preference specified by key from the IC database into
   * a newly created handle, prefh.
   * key must not be the empty string.
   * attr is set to the attributes associated with the preference.
   * The incoming value of prefh is ignored.
   * A new handle is created in the current heap and returned in prefh.
   * If the routine returns an error, prefh is set to nil.
   * If the preference does not exist, no error is returned and prefh is set
   * to an empty handle.
   }
{
 *  ICSetPrefHandle()
 *  
>>>>>>> graemeg/cpstrnew
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.7
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICCountPref( inst: ICInstance; var count: SIGNEDLONG ): OSStatus; external name '_ICCountPref';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)


{ [c1] [b1] 
   * Counts the total number of preferences.
   * If the routine returns an error, count is set to 0.
   }
{
 *  ICGetIndPref()   *** DEPRECATED ***
 *  
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICSetPrefHandle( inst: ICInstance; const (*var*) key: Str255; attr: ICAttr; prefh: Handle ): OSStatus; external name '_ICSetPrefHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r1] [c1] [b4] 
   * Sets the preference specified by key from the IC database to the
   * value contained in prefh.
   * key must not be the empty string.
   * If prefh is nil then the preference value is not set.
   * If prefh is not nil then the preference value is set to the data
   * contained in it.
   * If attr is ICattr_no_change then the preference attributes are not set.
   * Otherwise the preference attributes are set to attr.
   * Returns icPermErr if the previous ICBegin was passed icReadOnlyPerm.
   * Returns icPermErr if current attr is locked, new attr is locked and prefh <> nil.
   }
{
 *  ICCountPref()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICCountPref( inst: ICInstance; var count: SIGNEDLONG ): OSStatus; external name '_ICCountPref';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [c1] [b1] 
   * Counts the total number of preferences.
   * If the routine returns an error, count is set to 0.
   }
{
 *  ICGetIndPref()
 *  
>>>>>>> graemeg/cpstrnew
=======
{
    PString -- RTSP Proxy Host
}
const
	kICRTSPProxyHost = 'RTSPProxyHost';
{
    PString -- host.domain, SMTP server   
}
const
	kICSMTPHost = 'SMTPHost';
{
    ICFontRecord -- font used for monospaced text (eg news articles)   
}
const
	kICScreenFont = 'ScreenFont';
{
    ICServices -- TCP and IP port-to-name mapping   
}
const
	kICServices = 'Services';
{
    TEXT -- append to news and mail messages   
}
const
	kICSignature = 'Signature';
{
    TEXT -- preferred mailing address   
}
const
	kICSnailMailAddress = 'SnailMailAddress';
{
    PString -- host.domain, remember that host.domain format allows ":port" and " port"  
}
const
	kICSocksHost = 'SocksHost';
{
    PString -- host.domain, default Telnet address   
}
const
	kICTelnetHost = 'TelnetHost';
{
    STR# -- formatted, list of UMich servers   
}
const
	kICUMichAll = 'UMichAll';
{
    PString -- formatted, preferred UMich server   
}
const
	kICUMichPreferred = 'UMichPreferred';
{
    Boolean   
}
const
	kICUseFTPProxy = 'UseFTPProxy';
{
    Boolean   
}
const
	kICUseGopherProxy = 'UseGopherProxy';
{
    Boolean   
}
const
	kICUseHTTPProxy = 'UseHTTPProxy';
{
    Boolean -- use PASV command for FTP transfers   
}
const
	kICUsePassiveFTP = 'UsePassiveFTP';
{
    Boolean
}
const
	kICUseRTSPProxy = 'UseRTSPProxy';
{
=======
{
    PString -- scrambled, password for NewsAuthUsername   
}
const
	kICNewsAuthPassword = 'NewsAuthPassword';
{
    PString -- user name for authorised news servers   
}
const
	kICNewsAuthUsername = 'NewsAuthUsername';
{
    TEXT -- extra headers for news messages   
}
const
	kICNewsHeaders = 'NewsHeaders';
{
    STR# -- list of domains not to be proxied   
}
const
	kICNoProxyDomains = 'NoProxyDomains';
{
    PString -- for X-Organization string   
}
const
	kICOrganization = 'Organization';
{
    PString -- host.domain, default Ph server   
}
const
	kICPhHost = 'PhHost';
{
    TEXT -- default response for finger servers   
}
const
	kICPlan = 'Plan';
{
    ICFontRecord -- font used to print ScreenFont   
}
const
	kICPrinterFont = 'PrinterFont';
{
    PString -- used to quote responses in news and mail   
}
const
	kICQuotingString = 'QuotingString';
{
    PString -- real name of user   
}
const
	kICRealName = 'RealName';
{
    PString -- RTSP Proxy Host
}
const
	kICRTSPProxyHost = 'RTSPProxyHost';
{
    PString -- host.domain, SMTP server   
}
const
	kICSMTPHost = 'SMTPHost';
{
    ICFontRecord -- font used for monospaced text (eg news articles)   
}
const
	kICScreenFont = 'ScreenFont';
{
    ICServices -- TCP and IP port-to-name mapping   
}
const
	kICServices = 'Services';
{
    TEXT -- append to news and mail messages   
}
const
	kICSignature = 'Signature';
{
    TEXT -- preferred mailing address   
}
const
	kICSnailMailAddress = 'SnailMailAddress';
{
    PString -- host.domain, remember that host.domain format allows ":port" and " port"  
}
const
	kICSocksHost = 'SocksHost';
{
    PString -- host.domain, default Telnet address   
}
const
	kICTelnetHost = 'TelnetHost';
{
    STR# -- formatted, list of UMich servers   
}
const
	kICUMichAll = 'UMichAll';
{
    PString -- formatted, preferred UMich server   
}
const
	kICUMichPreferred = 'UMichPreferred';
{
    Boolean   
}
const
	kICUseFTPProxy = 'UseFTPProxy';
{
    Boolean   
}
const
	kICUseGopherProxy = 'UseGopherProxy';
{
    Boolean   
}
const
	kICUseHTTPProxy = 'UseHTTPProxy';
{
    Boolean -- use PASV command for FTP transfers   
}
const
	kICUsePassiveFTP = 'UsePassiveFTP';
{
    Boolean
}
const
	kICUseRTSPProxy = 'UseRTSPProxy';
{
>>>>>>> graemeg/cpstrnew
    Boolean   
}
const
	kICUseSocks = 'UseSocks';
<<<<<<< HEAD
{
    PString -- no idea   
}
const
	kICWAISGateway = 'WAISGateway';
{
    PString -- URL, users default WWW page   
}
const
	kICWWWHomePage = 'WWWHomePage';
{
    RGBColor -- background colour for web pages   
}
const
	kICWebBackgroundColour = 'WebBackgroundColour';
{
    RGBColor -- colour for read links   
}
const
	kICWebReadColor = '646F6777쩦ebReadColor';
{
    PString -- URL, users default search page   
}
const
	kICWebSearchPagePrefs = 'WebSearchPagePrefs';
{
    RGBColor -- colour for normal text   
}
const
	kICWebTextColor = 'WebTextColor';
{
    Boolean -- whether to underline links   
}
const
	kICWebUnderlineLinks = '646F6777쩦ebUnderlineLinks';
{
    RGBColor -- colour for unread links   
}
const
	kICWebUnreadColor = '646F6777쩦ebUnreadColor';
{
    PString -- host.domain, default whois server   
}
const
	kICWhoisHost = 'WhoisHost';

{***********************************************************************************************

      FUNCTIONS

      What do the annotations after each API mean?
      --------------------------------------------

      [r1] Requires IC 1.1 or higher.
      [r2] Requires IC 1.2 or higher.
      [r3] Requires IC 2.0 or higher.
      [r4] Requires IC 2.5 or higher.
      
      IMPORTANT:

      In IC 2.5, instances automatically use the default configuration.
      You no longer need to configure an instance explicitly, except
      if your code might run with an older version of IC.  So the following
      notes only apply to IC 2.0 and earlier.

      [c1]  You must have specified a configuration before calling this routine.
      
      [c2]  You must have specified the default configuration before calling this
            routine.
      
      [c3]  You do not need to specify a configuration before calling this routine.
      
      [b1]  You must be inside a Begin/End pair when calling this routine.
      
      [b2]  You must be inside a Begin/End read/write pair when calling this routine.
      
      [b3]  You do not need to be inside a Begin/End pair when calling this routine.
      
      [b4]  If you are getting or setting multiple preferences, you should make this
            call inside a Begin/End pair. If you do not make this call inside a Begin/End
            pair, the call will automatically do it for you.
      
      [b5]  It is illegal to call this routine inside a Begin/End pair.

 ***********************************************************************************************}

{ ***** Starting Up and Shutting Down *****  }
{
 *  ICStart()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICStart( var inst: ICInstance; signature: OSType ): OSStatus; external name '_ICStart';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Call this at application initialisation. Set signature to a value
   * which has been regsitered with DTS to allow for future expansion
   * of the IC system. Returns inst as a connection to the IC system.
   }
{
 *  ICStop()
 *  
>>>>>>> graemeg/cpstrnew
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.7
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetIndPref( inst: ICInstance; index: SIGNEDLONG; var key: Str255 ): OSStatus; external name '_ICGetIndPref';
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew


{ [c1] [b1] 
   * Returns the key of the index'th preference.
   * index must be positive.
   * Returns icPrefNotFoundErr if index is greater than the total number of preferences.
   * If the routine returns an error, key is undefined.
   }
{
<<<<<<< HEAD
 *  ICDeletePref()   *** DEPRECATED ***
=======
 *  ICDeletePref()
>>>>>>> graemeg/cpstrnew
 *  
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICStop( inst: ICInstance ): OSStatus; external name '_ICStop';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [b5] 
   * Call this at application initialisation, after which inst
   * is no longer valid connection to IC.
   }
{
 *  ICGetVersion()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetVersion( inst: ICInstance; whichVersion: SIGNEDLONG; var version: UInt32 ): OSStatus; external name '_ICGetVersion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r4] [c3] [b3] 
   * Returns the version of Internet Config.  Pass kICComponentVersion
   * to get the version as previously returned by GetComponenVerson.
   * Pass kICNumVersion to get a NumVersion structure.
   }
{
 *  ICGetConfigName()
 *  
>>>>>>> graemeg/cpstrnew
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.7
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICDeletePref( inst: ICInstance; const (*var*) key: Str255 ): OSStatus; external name '_ICDeletePref';
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew


{ [c1] [b2] 
   * Deletes the preference specified by key.
   * key must not be the empty string.
   * Returns icPrefNotFound if the preference specified by key is not present.
   }
{
<<<<<<< HEAD
 *  ICEnd()   *** DEPRECATED ***
=======
 *  ICEnd()
>>>>>>> graemeg/cpstrnew
 *  
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetConfigName( inst: ICInstance; longname: Boolean; var name: Str255 ): OSStatus; external name '_ICGetConfigName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r2] [c1] [b3] 
   * Returns a string that describes the current configuration at a user
   * level. Set longname to true if you want a long name, up to 255
   * characters, or false if you want a short name, typically about 32
   * characters.
   * The returned string is for user display only. If you rely on the
   * exact format of it, you will conflict with any future IC
   * implementation that doesn't use explicit preference files.
   }
{ ***** Getting Information *****  }
{
 *  ICGetSeed()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetSeed( inst: ICInstance; var seed: SIGNEDLONG ): OSStatus; external name '_ICGetSeed';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [c3] [b3] 
   * Returns the current seed for the IC prefs database.
   * This seed changes each time a non-volatile preference is changed.
   * You can poll this to determine if any cached preferences change.
   }
{
 *  ICGetPerm()
 *  
>>>>>>> graemeg/cpstrnew
=======
{
    PString -- no idea   
}
const
	kICWAISGateway = 'WAISGateway';
{
    PString -- URL, users default WWW page   
}
const
	kICWWWHomePage = 'WWWHomePage';
{
    RGBColor -- background colour for web pages   
}
const
	kICWebBackgroundColour = 'WebBackgroundColour';
{
    RGBColor -- colour for read links   
}
const
	kICWebReadColor = '646F6777쩦ebReadColor';
{
    PString -- URL, users default search page   
}
const
	kICWebSearchPagePrefs = 'WebSearchPagePrefs';
{
    RGBColor -- colour for normal text   
}
const
	kICWebTextColor = 'WebTextColor';
{
    Boolean -- whether to underline links   
}
const
	kICWebUnderlineLinks = '646F6777쩦ebUnderlineLinks';
{
    RGBColor -- colour for unread links   
}
const
	kICWebUnreadColor = '646F6777쩦ebUnreadColor';
{
    PString -- host.domain, default whois server   
}
const
	kICWhoisHost = 'WhoisHost';

{***********************************************************************************************

      FUNCTIONS

      What do the annotations after each API mean?
      --------------------------------------------

      [r1] Requires IC 1.1 or higher.
      [r2] Requires IC 1.2 or higher.
      [r3] Requires IC 2.0 or higher.
      [r4] Requires IC 2.5 or higher.
      
      IMPORTANT:

      In IC 2.5, instances automatically use the default configuration.
      You no longer need to configure an instance explicitly, except
      if your code might run with an older version of IC.  So the following
      notes only apply to IC 2.0 and earlier.

      [c1]  You must have specified a configuration before calling this routine.
      
      [c2]  You must have specified the default configuration before calling this
            routine.
      
      [c3]  You do not need to specify a configuration before calling this routine.
      
      [b1]  You must be inside a Begin/End pair when calling this routine.
      
      [b2]  You must be inside a Begin/End read/write pair when calling this routine.
      
      [b3]  You do not need to be inside a Begin/End pair when calling this routine.
      
      [b4]  If you are getting or setting multiple preferences, you should make this
            call inside a Begin/End pair. If you do not make this call inside a Begin/End
            pair, the call will automatically do it for you.
      
      [b5]  It is illegal to call this routine inside a Begin/End pair.

 ***********************************************************************************************}

{ ***** Starting Up and Shutting Down *****  }
{
 *  ICStart()
 *  
>>>>>>> graemeg/cpstrnew
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.7
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICEnd( inst: ICInstance ): OSStatus; external name '_ICEnd';
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew


{ [c1] [b1] 
   * Terminates a preference session, as started by ICBegin.
   * You must have called ICBegin before calling this routine.
   }
{
<<<<<<< HEAD
 *  ICGetDefaultPref()   *** DEPRECATED ***
=======
 *  ICGetDefaultPref()
>>>>>>> graemeg/cpstrnew
 *  
=======
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICStart( var inst: ICInstance; signature: OSType ): OSStatus; external name '_ICStart';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Call this at application initialisation. Set signature to a value
   * which has been regsitered with DTS to allow for future expansion
   * of the IC system. Returns inst as a connection to the IC system.
   }
{
 *  ICStop()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
>>>>>>> graemeg/cpstrnew
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
<<<<<<< HEAD
function ICGetPerm( inst: ICInstance; var perm: ICPerm ): OSStatus; external name '_ICGetPerm';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [c3] [b3] 
   * Returns the access permissions currently associated with this instance.
   * While applications normally know what permissions they have,
   * this routine is designed for use by override components.
   }
{ ***** Reading and Writing Preferences *****  }
{
 *  ICBegin()
=======
function ICStop( inst: ICInstance ): OSStatus; external name '_ICStop';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [b5] 
   * Call this at application initialisation, after which inst
   * is no longer valid connection to IC.
   }
{
 *  ICGetVersion()
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
<<<<<<< HEAD
function ICBegin( inst: ICInstance; perm: ICPerm ): OSStatus; external name '_ICBegin';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [c1] [b5] 
   * Starting reading or writing multiple preferences.
   * A call to this must be balanced by a call to ICEnd.
   * Do not call WaitNextEvent between these calls.
   * The perm specifies whether you intend to read or read/write.
   * Only one writer is allowed per instance.
   * Note that this may open resource files that are not closed
   * until you call ICEnd.
   }
{
 *  ICGetPref()
 *  
>>>>>>> graemeg/cpstrnew
=======
function ICGetVersion( inst: ICInstance; whichVersion: SIGNEDLONG; var version: UInt32 ): OSStatus; external name '_ICGetVersion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r4] [c3] [b3] 
   * Returns the version of Internet Config.  Pass kICComponentVersion
   * to get the version as previously returned by GetComponenVerson.
   * Pass kICNumVersion to get a NumVersion structure.
   }
{
 *  ICGetConfigName()
 *  
>>>>>>> graemeg/cpstrnew
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.7
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetDefaultPref( inst: ICInstance; const (*var*) key: Str255; prefH: Handle ): OSStatus; external name '_ICGetDefaultPref';
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew


{ [r4] [c3] [b5] 
   * Returns a default preference value for the specified key.  You
   * must pass in a valid prefH, which is resized to fit the data.
   }
{ ***** User Interface Stuff *****  }
{
<<<<<<< HEAD
 *  ICEditPreferences()   *** DEPRECATED ***
=======
 *  ICEditPreferences()
>>>>>>> graemeg/cpstrnew
 *  
=======
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetConfigName( inst: ICInstance; longname: Boolean; var name: Str255 ): OSStatus; external name '_ICGetConfigName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r2] [c1] [b3] 
   * Returns a string that describes the current configuration at a user
   * level. Set longname to true if you want a long name, up to 255
   * characters, or false if you want a short name, typically about 32
   * characters.
   * The returned string is for user display only. If you rely on the
   * exact format of it, you will conflict with any future IC
   * implementation that doesn't use explicit preference files.
   }
{ ***** Getting Information *****  }
{
 *  ICGetSeed()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
>>>>>>> graemeg/cpstrnew
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
<<<<<<< HEAD
function ICGetPref( inst: ICInstance; const (*var*) key: Str255; var attr: ICAttr; buf: UnivPtr; var size: SIGNEDLONG ): OSStatus; external name '_ICGetPref';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [c1] [b4] 
   * Reads the preference specified by key from the IC database to the
   * buffer pointed to by buf and size.
   * key must not be the empty string.
   * If buf is nil then no data is returned.
   * size must be non-negative.
   * attr and size are always set on return. On errors (except icTruncatedErr)
   * attr is set to ICattr_no_change and size is set to 0.
   * size is the actual size of the data.
   * attr is set to the attributes associated with the preference.
   * If this routine returns icTruncatedErr then the other returned
   * values are valid except that only the first size bytes have been
   * return. size is adjusted to reflect the true size of the preference.
   * Returns icPrefNotFound if there is no preference for the key.
   }
{
 *  ICSetPref()
 *  
=======
function ICGetSeed( inst: ICInstance; var seed: SIGNEDLONG ): OSStatus; external name '_ICGetSeed';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [c3] [b3] 
   * Returns the current seed for the IC prefs database.
   * This seed changes each time a non-volatile preference is changed.
   * You can poll this to determine if any cached preferences change.
   }
{
 *  ICGetPerm()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetPerm( inst: ICInstance; var perm: ICPerm ): OSStatus; external name '_ICGetPerm';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [c3] [b3] 
   * Returns the access permissions currently associated with this instance.
   * While applications normally know what permissions they have,
   * this routine is designed for use by override components.
   }
{ ***** Reading and Writing Preferences *****  }
{
 *  ICBegin()
 *  
>>>>>>> graemeg/cpstrnew
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
<<<<<<< HEAD
function ICSetPref( inst: ICInstance; const (*var*) key: Str255; attr: ICAttr; buf: {const} UnivPtr; size: SIGNEDLONG ): OSStatus; external name '_ICSetPref';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [c1] [b4] 
   * Sets the preference specified by key from the IC database to the
   * value pointed to by buf and size.
   * key must not be the empty string.
   * size must be non-negative. 
   * If buf is nil then the preference value is not set and size is ignored.
   * If buf is not nil then the preference value is set to the size
   * bytes pointed to by buf.
   * If attr is ICattr_no_change then the preference attributes are not set.
   * Otherwise the preference attributes are set to attr.
   * Returns icPermErr if the previous ICBegin was passed icReadOnlyPerm.
   * Returns icPermErr if current attr is locked, new attr is locked and buf <> nil.
   }
{
 *  ICFindPrefHandle()
=======
function ICBegin( inst: ICInstance; perm: ICPerm ): OSStatus; external name '_ICBegin';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [c1] [b5] 
   * Starting reading or writing multiple preferences.
   * A call to this must be balanced by a call to ICEnd.
   * Do not call WaitNextEvent between these calls.
   * The perm specifies whether you intend to read or read/write.
   * Only one writer is allowed per instance.
   * Note that this may open resource files that are not closed
   * until you call ICEnd.
   }
{
 *  ICGetPref()
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
<<<<<<< HEAD
function ICFindPrefHandle( inst: ICInstance; const (*var*) key: Str255; var attr: ICAttr; prefh: Handle ): OSStatus; external name '_ICFindPrefHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r2] [c1] [b4] 
   * This routine effectively replaces ICGetPrefHandle.
   * Reads the preference specified by key from the IC database into
   * a handle, prefh.
   * key must not be the empty string.
   * attr is set to the attributes associated with the preference.
   * You must set prefh to a non-nil handle before calling this routine.
   * If the preference does not exist, icPrefNotFoundErr is returned.
   }
{
 *  ICGetPrefHandle()
 *  
>>>>>>> graemeg/cpstrnew
=======
function ICGetPref( inst: ICInstance; const (*var*) key: Str255; var attr: ICAttr; buf: UnivPtr; var size: SIGNEDLONG ): OSStatus; external name '_ICGetPref';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [c1] [b4] 
   * Reads the preference specified by key from the IC database to the
   * buffer pointed to by buf and size.
   * key must not be the empty string.
   * If buf is nil then no data is returned.
   * size must be non-negative.
   * attr and size are always set on return. On errors (except icTruncatedErr)
   * attr is set to ICattr_no_change and size is set to 0.
   * size is the actual size of the data.
   * attr is set to the attributes associated with the preference.
   * If this routine returns icTruncatedErr then the other returned
   * values are valid except that only the first size bytes have been
   * return. size is adjusted to reflect the true size of the preference.
   * Returns icPrefNotFound if there is no preference for the key.
   }
{
 *  ICSetPref()
 *  
>>>>>>> graemeg/cpstrnew
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.7
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICEditPreferences( inst: ICInstance; const (*var*) key: Str255 ): OSStatus; external name '_ICEditPreferences';
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew


{ [r1] [c1] [b3] 
   * Instructs IC to display the user interface associated with editing
   * preferences and focusing on the preference specified by key.
   * If key is the empty string then no preference should be focused upon.
   * You must have specified a configuration before calling this routine.
   * You do not need to call ICBegin before calling this routine.
   * In the current implementation this launches the IC application
   * (or brings it to the front) and displays the window containing
   * the preference specified by key.
   * It may have a radically different implementation in future
   * IC systems.
   }
{ ***** URL Handling *****  }
{
<<<<<<< HEAD
 *  ICLaunchURL()   *** DEPRECATED ***
=======
 *  ICLaunchURL()
>>>>>>> graemeg/cpstrnew
 *  
=======
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICSetPref( inst: ICInstance; const (*var*) key: Str255; attr: ICAttr; buf: {const} UnivPtr; size: SIGNEDLONG ): OSStatus; external name '_ICSetPref';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [c1] [b4] 
   * Sets the preference specified by key from the IC database to the
   * value pointed to by buf and size.
   * key must not be the empty string.
   * size must be non-negative. 
   * If buf is nil then the preference value is not set and size is ignored.
   * If buf is not nil then the preference value is set to the size
   * bytes pointed to by buf.
   * If attr is ICattr_no_change then the preference attributes are not set.
   * Otherwise the preference attributes are set to attr.
   * Returns icPermErr if the previous ICBegin was passed icReadOnlyPerm.
   * Returns icPermErr if current attr is locked, new attr is locked and buf <> nil.
   }
{
 *  ICFindPrefHandle()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
>>>>>>> graemeg/cpstrnew
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
<<<<<<< HEAD
function ICGetPrefHandle( inst: ICInstance; const (*var*) key: Str255; var attr: ICAttr; var prefh: Handle ): OSStatus; external name '_ICGetPrefHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r1] [c1] [b4] 
   * This routine is now obsolete. Use ICFindPrefHandle instead.
   * Reads the preference specified by key from the IC database into
   * a newly created handle, prefh.
   * key must not be the empty string.
   * attr is set to the attributes associated with the preference.
   * The incoming value of prefh is ignored.
   * A new handle is created in the current heap and returned in prefh.
   * If the routine returns an error, prefh is set to nil.
   * If the preference does not exist, no error is returned and prefh is set
   * to an empty handle.
   }
{
 *  ICSetPrefHandle()
=======
function ICFindPrefHandle( inst: ICInstance; const (*var*) key: Str255; var attr: ICAttr; prefh: Handle ): OSStatus; external name '_ICFindPrefHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r2] [c1] [b4] 
   * This routine effectively replaces ICGetPrefHandle.
   * Reads the preference specified by key from the IC database into
   * a handle, prefh.
   * key must not be the empty string.
   * attr is set to the attributes associated with the preference.
   * You must set prefh to a non-nil handle before calling this routine.
   * If the preference does not exist, icPrefNotFoundErr is returned.
   }
{
 *  ICGetPrefHandle()
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
<<<<<<< HEAD
function ICSetPrefHandle( inst: ICInstance; const (*var*) key: Str255; attr: ICAttr; prefh: Handle ): OSStatus; external name '_ICSetPrefHandle';
=======
function ICGetPrefHandle( inst: ICInstance; const (*var*) key: Str255; var attr: ICAttr; var prefh: Handle ): OSStatus; external name '_ICGetPrefHandle';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r1] [c1] [b4] 
<<<<<<< HEAD
   * Sets the preference specified by key from the IC database to the
   * value contained in prefh.
   * key must not be the empty string.
   * If prefh is nil then the preference value is not set.
   * If prefh is not nil then the preference value is set to the data
   * contained in it.
   * If attr is ICattr_no_change then the preference attributes are not set.
   * Otherwise the preference attributes are set to attr.
   * Returns icPermErr if the previous ICBegin was passed icReadOnlyPerm.
   * Returns icPermErr if current attr is locked, new attr is locked and prefh <> nil.
   }
{
 *  ICCountPref()
 *  
>>>>>>> graemeg/cpstrnew
=======
=======

const
	kICReservedKey				= 'kICReservedKey';
	{	
	    STR# -- formatted, list of Archie servers  
		}
	kICArchieAll				= 'ArchieAll';
	{	
	    PString -- formatted, preferred Archie server   
		}
	kICArchiePreferred			= 'ArchiePreferred';
	{	
	    ICCharTable -- Mac-to-Net and Net-to-Mac character translation   
		}
	kICCharacterSet				= 'CharacterSet';
	{	
	    ICFontRecord -- font used for proportional text   
		}
	kICDocumentFont				= 'DocumentFont';
	{	
	    ICFileSpec -- where to put newly downloaded files   
		}
	kICDownloadFolder			= 'DownloadFolder';
	{	
	    PString -- user@host.domain, email address of user, ie return address   
		}
	kICEmail					= 'Email';
	{	
	    PString -- host.domain, default FTP server   
		}
	kICFTPHost					= 'FTPHost';
	{	
	    PString -- second level FTP proxy authorisation   
		}
	kICFTPProxyAccount			= 'FTPProxyAccount';
	{	
	    PString -- host.domain   
		}
	kICFTPProxyHost				= 'FTPProxyHost';
	{	
	    PString -- scrambled, password for FTPProxyUser   
		}
	kICFTPProxyPassword			= 'FTPProxyPassword';
	{	
	    PString -- first level FTP proxy authorisation   
		}
	kICFTPProxyUser				= 'FTPProxyUser';
	{	
	    PString -- host.domain, default finger server   
		}
	kICFingerHost				= 'FingerHost';
	{	
	    PString -- host.domain, default Gopher server   
		}
	kICGopherHost				= 'GopherHost';
	{	
	    PString -- host.domain, see note in Prog Docs   
		}
	kICGopherProxy				= 'GopherProxy';
	{	
	    PString -- host.domain   
		}
	kICHTTPProxyHost			= 'HTTPProxyHost';
	{	
	    ICAppSpec -- helpers for URL schemes   
		}
	kICHelper					= 'Helper�';
	{	
	    PString -- description for URL scheme   
		}
	kICHelperDesc				= 'HelperDesc�';
	{	
	    ICAppSpecList -- list of common helpers for URL schemes   
		}
	kICHelperList				= 'HelperList�';
	{	
	    PString -- host.domain, Internet Relay Chat server   
		}
	kICIRCHost					= 'IRCHost';
	{	
	    STR# -- formatted, list of Info-Mac servers   
		}
	kICInfoMacAll				= 'InfoMacAll';
	{	
	    PString -- formatted, preferred Info-Mac server   
		}
	kICInfoMacPreferred			= 'InfoMacPreferred';
	{	
	    PString -- string LDAP thing   
		}
	kICLDAPSearchbase			= 'LDAPSearchbase';
	{	
	    PString -- host.domain   
		}
	kICLDAPServer				= 'LDAPServer';
	{	
	    ICFontRecord -- font used for lists of items (eg news article lists)   
		}
	kICListFont					= 'ListFont';
	{	
	    PString -- host for MacSearch queries   
		}
	kICMacSearchHost			= 'MacSearchHost';
	{	
	    PString -- user@host.domain, account from which to fetch mail   
		}
	kICMailAccount				= 'MailAccount';
	{	
	    TEXT -- extra headers for mail messages   
		}
	kICMailHeaders				= 'MailHeaders';
	{	
	    PString -- scrambled, password for MailAccount   
		}
	kICMailPassword				= 'MailPassword';
	{	
	    ICMapEntries -- file type mapping, see documentation   
		}
	kICMapping					= 'Mapping';
	{	
	    PString -- host.domain, NNTP server   
		}
	kICNNTPHost					= 'NNTPHost';
	{	
	    PString -- host.domain, Network Time Protocol (NTP)   
		}
	kICNTPHost					= 'NTPHost';
	{	
	    Boolean   
		}
	kICNewMailDialog			= 'NewMailDialog';
	{	
	    Boolean -- how to announce new mail   
		}
	kICNewMailFlashIcon			= 'NewMailFlashIcon';
	{	
	    Boolean   
		}
	kICNewMailPlaySound			= 'NewMailPlaySound';
	{	
	    PString   
		}
	kICNewMailSoundName			= 'NewMailSoundName';
	{	
	    PString -- scrambled, password for NewsAuthUsername   
		}
	kICNewsAuthPassword			= 'NewsAuthPassword';
	{	
	    PString -- user name for authorised news servers   
		}
	kICNewsAuthUsername			= 'NewsAuthUsername';
	{	
	    TEXT -- extra headers for news messages   
		}
	kICNewsHeaders				= 'NewsHeaders';
	{	
	    STR# -- list of domains not to be proxied   
		}
	kICNoProxyDomains			= 'NoProxyDomains';
	{	
	    PString -- for X-Organization string   
		}
	kICOrganization				= 'Organization';
	{	
	    PString -- host.domain, default Ph server   
		}
	kICPhHost					= 'PhHost';
	{	
	    TEXT -- default response for finger servers   
		}
	kICPlan						= FourCharCode('Plan');
	{	
	    ICFontRecord -- font used to print ScreenFont   
		}
	kICPrinterFont				= 'PrinterFont';
	{	
	    PString -- used to quote responses in news and mail   
		}
	kICQuotingString			= 'QuotingString';
	{	
	    PString -- real name of user   
		}
	kICRealName					= 'RealName';
	{	
	    PString -- RTSP Proxy Host
		}
	kICRTSPProxyHost			= 'RTSPProxyHost';
	{	
	    PString -- host.domain, SMTP server   
		}
	kICSMTPHost					= 'SMTPHost';
	{	
	    ICFontRecord -- font used for monospaced text (eg news articles)   
		}
	kICScreenFont				= 'ScreenFont';
	{	
	    ICServices -- TCP and IP port-to-name mapping   
		}
	kICServices					= 'Services';
	{	
	    TEXT -- append to news and mail messages   
		}
	kICSignature				= 'Signature';
	{	
	    TEXT -- preferred mailing address   
		}
	kICSnailMailAddress			= 'SnailMailAddress';
	{	
	    PString -- host.domain, remember that host.domain format allows ":port" and " port"  
		}
	kICSocksHost				= 'SocksHost';
	{	
	    PString -- host.domain, default Telnet address   
		}
	kICTelnetHost				= 'TelnetHost';
	{	
	    STR# -- formatted, list of UMich servers   
		}
	kICUMichAll					= 'UMichAll';
	{	
	    PString -- formatted, preferred UMich server   
		}
	kICUMichPreferred			= 'UMichPreferred';
	{	
	    Boolean   
		}
	kICUseFTPProxy				= 'UseFTPProxy';
	{	
	    Boolean   
		}
	kICUseGopherProxy			= 'UseGopherProxy';
	{	
	    Boolean   
		}
	kICUseHTTPProxy				= 'UseHTTPProxy';
	{	
	    Boolean -- use PASV command for FTP transfers   
		}
	kICUsePassiveFTP			= 'UsePassiveFTP';
	{	
	    Boolean
		}
	kICUseRTSPProxy				= 'UseRTSPProxy';
	{	
	    Boolean   
		}
	kICUseSocks					= 'UseSocks';
	{	
	    PString -- no idea   
		}
	kICWAISGateway				= 'WAISGateway';
	{	
	    PString -- URL, users default WWW page   
		}
	kICWWWHomePage				= 'WWWHomePage';
	{	
	    RGBColor -- background colour for web pages   
		}
	kICWebBackgroundColour		= 'WebBackgroundColour';
	{	
	    RGBColor -- colour for read links   
		}
	kICWebReadColor				= '646F6777쩦ebReadColor';
	{	
	    PString -- URL, users default search page   
		}
	kICWebSearchPagePrefs		= 'WebSearchPagePrefs';
	{	
	    RGBColor -- colour for normal text   
		}
	kICWebTextColor				= 'WebTextColor';
	{	
	    Boolean -- whether to underline links   
		}
	kICWebUnderlineLinks		= '646F6777쩦ebUnderlineLinks';
	{	
	    RGBColor -- colour for unread links   
		}
	kICWebUnreadColor			= '646F6777쩦ebUnreadColor';
	{	
	    PString -- host.domain, default whois server   
		}
	kICWhoisHost				= 'WhoisHost';

	{	***********************************************************************************************
	
	      FUNCTIONS
	
	      What do the annotations after each API mean?
	      --------------------------------------------
	
	      [r1] Requires IC 1.1 or higher.
	      [r2] Requires IC 1.2 or higher.
	      [r3] Requires IC 2.0 or higher.
	      [r4] Requires IC 2.5 or higher.
	      
	      IMPORTANT:
	
	      In IC 2.5, instances automatically use the default configuration.
	      You no longer need to configure an instance explicitly, except
	      if your code might run with an older version of IC.  So the following
	      notes only apply to IC 2.0 and earlier.
	
	      [c1]  You must have specified a configuration before calling this routine.
	      
	      [c2]  You must have specified the default configuration before calling this
	            routine.
	      
	      [c3]  You do not need to specify a configuration before calling this routine.
	      
	      [b1]  You must be inside a Begin/End pair when calling this routine.
	      
	      [b2]  You must be inside a Begin/End read/write pair when calling this routine.
	      
	      [b3]  You do not need to be inside a Begin/End pair when calling this routine.
	      
	      [b4]  If you are getting or setting multiple preferences, you should make this
	            call inside a Begin/End pair. If you do not make this call inside a Begin/End
	            pair, the call will automatically do it for you.
	      
	      [b5]  It is illegal to call this routine inside a Begin/End pair.
	
	 ***********************************************************************************************	}

	{	 ***** Starting Up and Shutting Down *****  	}
	{
	 *  ICStart()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
	 *    CarbonLib:        in CarbonLib 1.0.2 and later
	 *    Mac OS X:         in version 10.0 and later
	 	}
function ICStart(var inst: ICInstance; signature: OSType): OSStatus; external name '_ICStart';

{ Call this at application initialisation. Set signature to a value
   * which has been regsitered with DTS to allow for future expansion
   * of the IC system. Returns inst as a connection to the IC system.
   }
{
 *  ICStop()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Mac OS X:         in version 10.0 and later
 }
function ICStop(inst: ICInstance): OSStatus; external name '_ICStop';

{ [b5] 
   * Call this at application initialisation, after which inst
   * is no longer valid connection to IC.
   }
{
 *  ICGetVersion()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Mac OS X:         in version 10.0 and later
 }
function ICGetVersion(inst: ICInstance; whichVersion: SInt32; var version: UInt32): OSStatus; external name '_ICGetVersion';
{ [r4] [c3] [b3] 
   * Returns the version of Internet Config.  Pass kICComponentVersion
   * to get the version as previously returned by GetComponenVerson.
   * Pass kICNumVersion to get a NumVersion structure.
   }
{ ***** Specifying a Configuration *****  }
{$ifc CALL_NOT_IN_CARBON}
{
 *  ICFindConfigFile()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function ICFindConfigFile(inst: ICInstance; count: SInt16; folders: ICDirSpecArrayPtr): OSStatus; external name '_ICFindConfigFile';
{ [b5] 
   * Call to configure this connection to IC.
   * Set count as the number of valid elements in folders.
   * Set folders to a pointer to the folders to search.
   * Setting count to 0 and folders to nil is OK.
   * Searches the specified folders and then the Preferences folder
   * in a unspecified manner.
   }
{
 *  ICFindUserConfigFile()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function ICFindUserConfigFile(inst: ICInstance; var where: ICDirSpec): OSStatus; external name '_ICFindUserConfigFile';
{ [r1] [b5] 
   * Similar to ICFindConfigFile except that it only searches the folder
   * specified in where.  If the input parameters are valid the routine
   * will always successful configure the instance, creating an
   * empty configuration if necessary
   * For use with double-clickable preference files.
   }
{
 *  ICGeneralFindConfigFile()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function ICGeneralFindConfigFile(inst: ICInstance; searchPrefs: boolean; canCreate: boolean; count: SInt16; folders: ICDirSpecArrayPtr): OSStatus; external name '_ICGeneralFindConfigFile';
{ [r2] [b5] 
   * Call to configure this connection to IC.
   * This routine acts as a more general replacement for
   * ICFindConfigFile and ICFindUserConfigFile.
   * Set search_prefs to true if you want it to search the preferences folder.
   * Set can_create to true if you want it to be able to create a new config.
   * Set count as the number of valid elements in folders.
   * Set folders to a pointer to the folders to search.
   * Setting count to 0 and folders to nil is OK.
   * Searches the specified folders and then optionally the Preferences folder
   * in a unspecified manner.
   }
{
 *  ICChooseConfig()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function ICChooseConfig(inst: ICInstance): OSStatus; external name '_ICChooseConfig';
{ [r2] [b5] 
   * Requests the user to choose a configuration, typically using some
   * sort of modal dialog. If the user cancels the dialog the configuration
   * state will be unaffected.
   }
{
 *  ICChooseNewConfig()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function ICChooseNewConfig(inst: ICInstance): OSStatus; external name '_ICChooseNewConfig';
{ [r2] [b5] 
   * Requests the user to create a new configuration, typically using some
   * sort of modal dialog. If the user cancels the dialog the configuration
   * state will be unaffected.
   }
{$endc}  {CALL_NOT_IN_CARBON}

{
 *  ICGetConfigName()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Mac OS X:         in version 10.0 and later
 }
function ICGetConfigName(inst: ICInstance; longname: boolean; var name: Str255): OSStatus; external name '_ICGetConfigName';
{ [r2] [c1] [b3] 
   * Returns a string that describes the current configuration at a user
   * level. Set longname to true if you want a long name, up to 255
   * characters, or false if you want a short name, typically about 32
   * characters.
   * The returned string is for user display only. If you rely on the
   * exact format of it, you will conflict with any future IC
   * implementation that doesn't use explicit preference files.
   }
{$ifc CALL_NOT_IN_CARBON}
{
 *  ICGetConfigReference()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function ICGetConfigReference(inst: ICInstance; ref: ICConfigRefHandle): OSStatus; external name '_ICGetConfigReference';
{ [r2] [c1] [b3] 
   * Returns a self-contained reference to the instance's current
   * configuration.
   * ref must be a valid non-nil handle and it will be resized to fit the
   * resulting data.
   }
{
 *  ICSetConfigReference()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function ICSetConfigReference(inst: ICInstance; ref: ICConfigRefHandle; flags: SInt32): OSStatus; external name '_ICSetConfigReference';
{ [r2] [b5] 
   * Reconfigures the instance using a configuration reference that was
   * got using ICGetConfigReference reference. Set the
   * icNoUserInteraction_bit in flags if you require that this routine
   * not present a modal dialog. Other flag bits are reserved and should
   * be set to zero.
   * ref must not be nil.
   }
{ ***** Private Routines *****
 * 
 * If you are calling these routines, you are most probably doing something
 * wrong.  Please read the documentation for more details.
  }
{
 *  ICSpecifyConfigFile()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function ICSpecifyConfigFile(inst: ICInstance; var config: FSSpec): OSStatus; external name '_ICSpecifyConfigFile';
{ [b5] 
   * For use only by the IC application.
   * If you call this routine yourself, you will conflict with any
   * future IC implementation that doesn't use explicit preference files.
   }
{
 *  ICRefreshCaches()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function ICRefreshCaches(inst: ICInstance): OSStatus; external name '_ICRefreshCaches';
{ [r3] [c1] [b3] 
   * For use only by the IC application.
   * If you call this routine yourself, you will conflict with any
   * future IC implementation that doesn't use explicit preference files.
   }
{ ***** Getting Information *****  }
{$endc}  {CALL_NOT_IN_CARBON}

{
 *  ICGetSeed()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Mac OS X:         in version 10.0 and later
 }
function ICGetSeed(inst: ICInstance; var seed: SInt32): OSStatus; external name '_ICGetSeed';
{ [c3] [b3] 
   * Returns the current seed for the IC prefs database.
   * This seed changes each time a non-volatile preference is changed.
   * You can poll this to determine if any cached preferences change.
   }
{
 *  ICGetPerm()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Mac OS X:         in version 10.0 and later
 }
function ICGetPerm(inst: ICInstance; var perm: ICPerm): OSStatus; external name '_ICGetPerm';
{ [c3] [b3] 
   * Returns the access permissions currently associated with this instance.
   * While applications normally know what permissions they have,
   * this routine is designed for use by override components.
   }
{$ifc CALL_NOT_IN_CARBON}
{
 *  ICDefaultFileName()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function ICDefaultFileName(inst: ICInstance; var name: Str63): OSStatus; external name '_ICDefaultFileName';
{ [c3] [b3] 
   * Returns the default file name for IC preference files.
   * Applications should never need to call this routine.
   * If you rely on information returned by this routine yourself,
   * you may conflict with any future IC implementation that doesn't use
   * explicit preference files.
   * The component calls this routine to set up the default IC file name.
   * This allows this operation to be intercepted by a component that has
   * captured us. It currently gets it from the component resource file.
   * The glue version is hardwired to "Internet Preferences".
   }
{
 *  ICGetComponentInstance()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function ICGetComponentInstance(inst: ICInstance; var componentInst: ComponentInstance): OSStatus; external name '_ICGetComponentInstance';

{ [c3] [b3] 
   * Returns noErr and the connection to the IC component,
   * if we're using the component.
   * Returns badComponenInstance and nil if we're operating with glue.
   }
{ ***** Reading and Writing Preferences *****  }
{$endc}  {CALL_NOT_IN_CARBON}

{
 *  ICBegin()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Mac OS X:         in version 10.0 and later
 }
function ICBegin(inst: ICInstance; perm: ByteParameter): OSStatus; external name '_ICBegin';
{ [c1] [b5] 
   * Starting reading or writing multiple preferences.
   * A call to this must be balanced by a call to ICEnd.
   * Do not call WaitNextEvent between these calls.
   * The perm specifies whether you intend to read or read/write.
   * Only one writer is allowed per instance.
   * Note that this may open resource files that are not closed
   * until you call ICEnd.
   }
{
 *  ICGetPref()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Mac OS X:         in version 10.0 and later
 }
function ICGetPref(inst: ICInstance; const (*var*) key: Str255; var attr: ICAttr; buf: UnivPtr; var size: SInt32): OSStatus; external name '_ICGetPref';
{ [c1] [b4] 
   * Reads the preference specified by key from the IC database to the
   * buffer pointed to by buf and size.
   * key must not be the empty string.
   * If buf is nil then no data is returned.
   * size must be non-negative.
   * attr and size are always set on return. On errors (except icTruncatedErr)
   * attr is set to ICattr_no_change and size is set to 0.
   * size is the actual size of the data.
   * attr is set to the attributes associated with the preference.
   * If this routine returns icTruncatedErr then the other returned
   * values are valid except that only the first size bytes have been
   * return. size is adjusted to reflect the true size of the preference.
   * Returns icPrefNotFound if there is no preference for the key.
   }
{
 *  ICSetPref()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Mac OS X:         in version 10.0 and later
 }
function ICSetPref(inst: ICInstance; const (*var*) key: Str255; attr: ICAttr; buf: UnivPtr; size: SInt32): OSStatus; external name '_ICSetPref';
{ [c1] [b4] 
   * Sets the preference specified by key from the IC database to the
   * value pointed to by buf and size.
   * key must not be the empty string.
   * size must be non-negative. 
   * If buf is nil then the preference value is not set and size is ignored.
   * If buf is not nil then the preference value is set to the size
   * bytes pointed to by buf.
   * If attr is ICattr_no_change then the preference attributes are not set.
   * Otherwise the preference attributes are set to attr.
   * Returns icPermErr if the previous ICBegin was passed icReadOnlyPerm.
   * Returns icPermErr if current attr is locked, new attr is locked and buf <> nil.
   }
{
 *  ICFindPrefHandle()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Mac OS X:         in version 10.0 and later
 }
function ICFindPrefHandle(inst: ICInstance; const (*var*) key: Str255; var attr: ICAttr; prefh: Handle): OSStatus; external name '_ICFindPrefHandle';
{ [r2] [c1] [b4] 
   * This routine effectively replaces ICGetPrefHandle.
   * Reads the preference specified by key from the IC database into
   * a handle, prefh.
   * key must not be the empty string.
   * attr is set to the attributes associated with the preference.
   * You must set prefh to a non-nil handle before calling this routine.
   * If the preference does not exist, icPrefNotFoundErr is returned.
   }
{
 *  ICGetPrefHandle()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Mac OS X:         in version 10.0 and later
 }
function ICGetPrefHandle(inst: ICInstance; const (*var*) key: Str255; var attr: ICAttr; var prefh: Handle): OSStatus; external name '_ICGetPrefHandle';
{ [r1] [c1] [b4] 
>>>>>>> graemeg/fixes_2_2
   * This routine is now obsolete. Use ICFindPrefHandle instead.
   * Reads the preference specified by key from the IC database into
   * a newly created handle, prefh.
   * key must not be the empty string.
   * attr is set to the attributes associated with the preference.
   * The incoming value of prefh is ignored.
   * A new handle is created in the current heap and returned in prefh.
   * If the routine returns an error, prefh is set to nil.
   * If the preference does not exist, no error is returned and prefh is set
   * to an empty handle.
   }
{
 *  ICSetPrefHandle()
 *  
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.7
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICLaunchURL( inst: ICInstance; const (*var*) hint: Str255; data: {const} UnivPtr; len: SIGNEDLONG; var selStart: SIGNEDLONG; var selEnd: SIGNEDLONG ): OSStatus; external name '_ICLaunchURL';
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew


{ [r1] [c1] [b3] 
   * Parses a URL out of the specified text and feeds it off to the
   * appropriate helper.
   * hint indicates the default scheme for URLs of the form "name@address".
   * If hint is the empty string then URLs of that form are not allowed.
   * data points to the start of the text. It must not be nil.
   * len indicates the length of the text. It must be non-negative.
   * selStart and selEnd should be passed in as the current selection of
   * the text. This selection is given in the same manner as TextEdit,
   * ie if selStart = selEnd then there is no selection only an insertion
   * point. Also selStart � selEnd and 0 � selStart � len and 0 � selEnd � len.
   * selStart and selEnd are returned as the bounds of the URL. If the
   * routine returns an error then these new boundaries may be
   * invalid but they will be close.
   * The URL is parsed out of the text and passed off to the appropriate
   * helper using the GURL AppleEvent.
   }
{
<<<<<<< HEAD
 *  ICParseURL()   *** DEPRECATED ***
 *  
=======
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICSetPrefHandle( inst: ICInstance; const (*var*) key: Str255; attr: ICAttr; prefh: Handle ): OSStatus; external name '_ICSetPrefHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
 *  Availability:
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Mac OS X:         in version 10.0 and later
 }
function ICSetPrefHandle(inst: ICInstance; const (*var*) key: Str255; attr: ICAttr; prefh: Handle): OSStatus; external name '_ICSetPrefHandle';
>>>>>>> graemeg/fixes_2_2
{ [r1] [c1] [b4] 
   * Sets the preference specified by key from the IC database to the
   * value contained in prefh.
   * key must not be the empty string.
   * If prefh is nil then the preference value is not set.
<<<<<<< HEAD
   * If prefh is not nil then the preference value is set to the data
=======
   * If buf is not nil then the preference value is set to the data
>>>>>>> graemeg/fixes_2_2
   * contained in it.
   * If attr is ICattr_no_change then the preference attributes are not set.
   * Otherwise the preference attributes are set to attr.
   * Returns icPermErr if the previous ICBegin was passed icReadOnlyPerm.
   * Returns icPermErr if current attr is locked, new attr is locked and prefh <> nil.
   }
{
 *  ICCountPref()
<<<<<<< HEAD
=======
{
    PString -- formatted, preferred Archie server   
}
const
	kICArchiePreferred = 'ArchiePreferred';
{
    ICCharTable -- Mac-to-Net and Net-to-Mac character translation   
}
const
	kICCharacterSet = 'CharacterSet';
{
    ICFontRecord -- font used for proportional text   
}
const
	kICDocumentFont = 'DocumentFont';
{
    ICFileSpec -- where to put newly downloaded files   
}
const
	kICDownloadFolder = 'DownloadFolder';
{
    PString -- user@host.domain, email address of user, ie return address   
}
const
	kICEmail = 'Email';
{
    PString -- host.domain, default FTP server   
}
const
	kICFTPHost = 'FTPHost';
{
    PString -- second level FTP proxy authorisation   
}
const
	kICFTPProxyAccount = 'FTPProxyAccount';
{
    PString -- host.domain   
}
const
	kICFTPProxyHost = 'FTPProxyHost';
{
    PString -- scrambled, password for FTPProxyUser   
}
const
	kICFTPProxyPassword = 'FTPProxyPassword';
{
    PString -- first level FTP proxy authorisation   
}
const
	kICFTPProxyUser = 'FTPProxyUser';
{
    PString -- host.domain, default finger server   
}
const
	kICFingerHost = 'FingerHost';
{
    PString -- host.domain, default Gopher server   
}
const
	kICGopherHost = 'GopherHost';
{
    PString -- host.domain, see note in Prog Docs   
}
const
	kICGopherProxy = 'GopherProxy';
{
    PString -- host.domain   
}
const
	kICHTTPProxyHost = 'HTTPProxyHost';
{
    ICAppSpec -- helpers for URL schemes   
}
const
	kICHelper = 'Helper�';
{
    PString -- description for URL scheme   
}
const
	kICHelperDesc = 'HelperDesc�';
{
    ICAppSpecList -- list of common helpers for URL schemes   
}
const
	kICHelperList = 'HelperList�';
{
    PString -- host.domain, Internet Relay Chat server   
}
const
	kICIRCHost = 'IRCHost';
{
    STR# -- formatted, list of Info-Mac servers   
}
const
	kICInfoMacAll = 'InfoMacAll';
{
    PString -- formatted, preferred Info-Mac server   
}
const
	kICInfoMacPreferred = 'InfoMacPreferred';
{
    PString -- string LDAP thing   
}
const
	kICLDAPSearchbase = 'LDAPSearchbase';
{
    PString -- host.domain   
}
const
	kICLDAPServer = 'LDAPServer';
{
    ICFontRecord -- font used for lists of items (eg news article lists)   
}
const
	kICListFont = 'ListFont';
{
    PString -- host for MacSearch queries   
}
const
	kICMacSearchHost = 'MacSearchHost';
{
    PString -- user@host.domain, account from which to fetch mail   
}
const
	kICMailAccount = 'MailAccount';
{
    TEXT -- extra headers for mail messages   
}
const
	kICMailHeaders = 'MailHeaders';
{
    PString -- scrambled, password for MailAccount   
}
const
	kICMailPassword = 'MailPassword';
{
    ICMapEntries -- file type mapping, see documentation   
}
const
	kICMapping = 'Mapping';
{
    PString -- host.domain, NNTP server   
}
const
	kICNNTPHost = 'NNTPHost';
{
    PString -- host.domain, Network Time Protocol (NTP)   
}
const
	kICNTPHost = 'NTPHost';
{
    Boolean   
}
const
	kICNewMailDialog = 'NewMailDialog';
{
    Boolean -- how to announce new mail   
}
const
	kICNewMailFlashIcon = 'NewMailFlashIcon';
{
    Boolean   
}
const
	kICNewMailPlaySound = 'NewMailPlaySound';
{
    PString   
}
const
	kICNewMailSoundName = 'NewMailSoundName';
{
    PString -- scrambled, password for NewsAuthUsername   
}
const
	kICNewsAuthPassword = 'NewsAuthPassword';
{
    PString -- user name for authorised news servers   
}
const
	kICNewsAuthUsername = 'NewsAuthUsername';
{
    TEXT -- extra headers for news messages   
}
const
	kICNewsHeaders = 'NewsHeaders';
{
    STR# -- list of domains not to be proxied   
}
const
	kICNoProxyDomains = 'NoProxyDomains';
{
    PString -- for X-Organization string   
}
const
	kICOrganization = 'Organization';
{
    PString -- host.domain, default Ph server   
}
const
	kICPhHost = 'PhHost';
{
    TEXT -- default response for finger servers   
}
const
	kICPlan = 'Plan';
{
    ICFontRecord -- font used to print ScreenFont   
}
const
	kICPrinterFont = 'PrinterFont';
{
    PString -- used to quote responses in news and mail   
}
const
	kICQuotingString = 'QuotingString';
{
    PString -- real name of user   
}
const
	kICRealName = 'RealName';
{
    PString -- RTSP Proxy Host
}
const
	kICRTSPProxyHost = 'RTSPProxyHost';
{
    PString -- host.domain, SMTP server   
}
const
	kICSMTPHost = 'SMTPHost';
{
    ICFontRecord -- font used for monospaced text (eg news articles)   
}
const
	kICScreenFont = 'ScreenFont';
{
    ICServices -- TCP and IP port-to-name mapping   
}
const
	kICServices = 'Services';
{
    TEXT -- append to news and mail messages   
}
const
	kICSignature = 'Signature';
{
    TEXT -- preferred mailing address   
}
const
	kICSnailMailAddress = 'SnailMailAddress';
{
    PString -- host.domain, remember that host.domain format allows ":port" and " port"  
}
const
	kICSocksHost = 'SocksHost';
{
    PString -- host.domain, default Telnet address   
}
const
	kICTelnetHost = 'TelnetHost';
{
    STR# -- formatted, list of UMich servers   
}
const
	kICUMichAll = 'UMichAll';
{
    PString -- formatted, preferred UMich server   
}
const
	kICUMichPreferred = 'UMichPreferred';
{
    Boolean   
}
const
	kICUseFTPProxy = 'UseFTPProxy';
{
    Boolean   
}
const
	kICUseGopherProxy = 'UseGopherProxy';
{
    Boolean   
}
const
	kICUseHTTPProxy = 'UseHTTPProxy';
{
    Boolean -- use PASV command for FTP transfers   
}
const
	kICUsePassiveFTP = 'UsePassiveFTP';
{
    Boolean
}
const
	kICUseRTSPProxy = 'UseRTSPProxy';
{
    Boolean   
}
const
	kICUseSocks = 'UseSocks';
{
    PString -- no idea   
}
const
	kICWAISGateway = 'WAISGateway';
{
    PString -- URL, users default WWW page   
}
const
	kICWWWHomePage = 'WWWHomePage';
{
    RGBColor -- background colour for web pages   
}
const
	kICWebBackgroundColour = 'WebBackgroundColour';
{
    RGBColor -- colour for read links   
}
const
	kICWebReadColor = '646F6777쩦ebReadColor';
{
    PString -- URL, users default search page   
}
const
	kICWebSearchPagePrefs = 'WebSearchPagePrefs';
{
    RGBColor -- colour for normal text   
}
const
	kICWebTextColor = 'WebTextColor';
{
    Boolean -- whether to underline links   
}
const
	kICWebUnderlineLinks = '646F6777쩦ebUnderlineLinks';
{
    RGBColor -- colour for unread links   
}
const
	kICWebUnreadColor = '646F6777쩦ebUnreadColor';
{
    PString -- host.domain, default whois server   
}
const
	kICWhoisHost = 'WhoisHost';

{***********************************************************************************************

      FUNCTIONS

      What do the annotations after each API mean?
      --------------------------------------------

      [r1] Requires IC 1.1 or higher.
      [r2] Requires IC 1.2 or higher.
      [r3] Requires IC 2.0 or higher.
      [r4] Requires IC 2.5 or higher.
      
      IMPORTANT:

      In IC 2.5, instances automatically use the default configuration.
      You no longer need to configure an instance explicitly, except
      if your code might run with an older version of IC.  So the following
      notes only apply to IC 2.0 and earlier.

      [c1]  You must have specified a configuration before calling this routine.
      
      [c2]  You must have specified the default configuration before calling this
            routine.
      
      [c3]  You do not need to specify a configuration before calling this routine.
      
      [b1]  You must be inside a Begin/End pair when calling this routine.
      
      [b2]  You must be inside a Begin/End read/write pair when calling this routine.
      
      [b3]  You do not need to be inside a Begin/End pair when calling this routine.
      
      [b4]  If you are getting or setting multiple preferences, you should make this
            call inside a Begin/End pair. If you do not make this call inside a Begin/End
            pair, the call will automatically do it for you.
      
      [b5]  It is illegal to call this routine inside a Begin/End pair.

 ***********************************************************************************************}

{ ***** Starting Up and Shutting Down *****  }
{
 *  ICStart()
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
<<<<<<< HEAD
function ICCountPref( inst: ICInstance; var count: SIGNEDLONG ): OSStatus; external name '_ICCountPref';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [c1] [b1] 
   * Counts the total number of preferences.
   * If the routine returns an error, count is set to 0.
   }
{
 *  ICGetIndPref()
=======
function ICStart( var inst: ICInstance; signature: OSType ): OSStatus; external name '_ICStart';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Call this at application initialisation. Set signature to a value
   * which has been regsitered with DTS to allow for future expansion
   * of the IC system. Returns inst as a connection to the IC system.
   }
{
 *  ICStop()
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
<<<<<<< HEAD
function ICGetIndPref( inst: ICInstance; index: SIGNEDLONG; var key: Str255 ): OSStatus; external name '_ICGetIndPref';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [c1] [b1] 
   * Returns the key of the index'th preference.
   * index must be positive.
   * Returns icPrefNotFoundErr if index is greater than the total number of preferences.
   * If the routine returns an error, key is undefined.
   }
{
 *  ICDeletePref()
 *  
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
function ICStop( inst: ICInstance ): OSStatus; external name '_ICStop';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [b5] 
   * Call this at application initialisation, after which inst
   * is no longer valid connection to IC.
   }
{
 *  ICGetVersion()
 *  
>>>>>>> origin/cpstrnew
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.7
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICParseURL( inst: ICInstance; const (*var*) hint: Str255; data: {const} UnivPtr; len: SIGNEDLONG; var selStart: SIGNEDLONG; var selEnd: SIGNEDLONG; url: Handle ): OSStatus; external name '_ICParseURL';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)


=======
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Mac OS X:         in version 10.0 and later
 }
function ICCountPref(inst: ICInstance; var count: SInt32): OSStatus; external name '_ICCountPref';
{ [c1] [b1] 
   * Counts the total number of preferences.
   * If the routine returns an error, count is set to 0.
   }
{
 *  ICGetIndPref()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Mac OS X:         in version 10.0 and later
 }
function ICGetIndPref(inst: ICInstance; index: SInt32; var key: Str255): OSStatus; external name '_ICGetIndPref';
{ [c1] [b1] 
   * Returns the key of the index'th preference.
   * index must be positive.
   * Returns icPrefNotFoundErr if index is greater than the total number of preferences.
   * If the routine returns an error, key is undefined.
   }
{
 *  ICDeletePref()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Mac OS X:         in version 10.0 and later
 }
function ICDeletePref(inst: ICInstance; const (*var*) key: Str255): OSStatus; external name '_ICDeletePref';
{ [c1] [b2] 
   * Deletes the preference specified by key.
   * key must not be the empty string.
   * Returns icPrefNotFound if the preference specified by key is not present.
   }
{
 *  ICEnd()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Mac OS X:         in version 10.0 and later
 }
function ICEnd(inst: ICInstance): OSStatus; external name '_ICEnd';
{ [c1] [b1] 
   * Terminates a preference session, as started by ICBegin.
   * You must have called ICBegin before calling this routine.
   }
{
 *  ICGetDefaultPref()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Mac OS X:         in version 10.0 and later
 }
function ICGetDefaultPref(inst: ICInstance; const (*var*) key: Str255; prefH: Handle): OSStatus; external name '_ICGetDefaultPref';
{ [r4] [c3] [b5] 
   * Returns a default preference value for the specified key.  You
   * must pass in a valid prefH, which is resized to fit the data.
   }
{ ***** User Interface Stuff *****  }
{
 *  ICEditPreferences()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Mac OS X:         in version 10.0 and later
 }
function ICEditPreferences(inst: ICInstance; const (*var*) key: Str255): OSStatus; external name '_ICEditPreferences';
{ [r1] [c1] [b3] 
   * Instructs IC to display the user interface associated with editing
   * preferences and focusing on the preference specified by key.
   * If key is the empty string then no preference should be focused upon.
   * You must have specified a configuration before calling this routine.
   * You do not need to call ICBegin before calling this routine.
   * In the current implementation this launches the IC application
   * (or brings it to the front) and displays the window containing
   * the preference specified by key.
   * It may have a radically different implementation in future
   * IC systems.
   }
{ ***** URL Handling *****  }
{
 *  ICLaunchURL()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Mac OS X:         in version 10.0 and later
 }
function ICLaunchURL(inst: ICInstance; const (*var*) hint: Str255; data: UnivPtr; len: SInt32; var selStart: SInt32; var selEnd: SInt32): OSStatus; external name '_ICLaunchURL';
{ [r1] [c1] [b3] 
   * Parses a URL out of the specified text and feeds it off to the
   * appropriate helper.
   * hint indicates the default scheme for URLs of the form "name@address".
   * If hint is the empty string then URLs of that form are not allowed.
   * data points to the start of the text. It must not be nil.
   * len indicates the length of the text. It must be non-negative.
   * selStart and selEnd should be passed in as the current selection of
   * the text. This selection is given in the same manner as TextEdit,
   * ie if selStart = selEnd then there is no selection only an insertion
   * point. Also selStart � selEnd and 0 � selStart � len and 0 � selEnd � len.
   * selStart and selEnd are returned as the bounds of the URL. If the
   * routine returns an error then these new boundaries may be
   * invalid but they will be close.
   * The URL is parsed out of the text and passed off to the appropriate
   * helper using the GURL AppleEvent.
   }
{
 *  ICParseURL()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Mac OS X:         in version 10.0 and later
 }
function ICParseURL(inst: ICInstance; const (*var*) hint: Str255; data: UnivPtr; len: SInt32; var selStart: SInt32; var selEnd: SInt32; url: Handle): OSStatus; external name '_ICParseURL';
>>>>>>> graemeg/fixes_2_2
{ [r1] [c1] [b3] 
   * Parses a URL out of the specified text and returns it in a canonical form
   * in a handle.
   * hint indicates the default scheme for URLs of the form "name@address".
   * If hint is the empty string then URLs of that form are not allowed.
   * data points to the start of the text. It must not be nil.
   * len indicates the length of the text. It must be non-negative.
   * selStart and selEnd should be passed in as the current selection of
   * the text. This selection is given in the same manner as TextEdit,
   * ie if selStart = selEnd then there is no selection only an insertion
   * point. Also selStart � selEnd and 0 � selStart � len and 0 � selEnd � len.
   * selStart and selEnd are returned as the bounds of the URL. If the
   * routine returns an error then these new boundaries may be
   * invalid but they will be close.
   * The incoming url handle must not be nil.  The resulting URL is normalised
   * and copied into the url handle, which is resized to fit.
   }
{
<<<<<<< HEAD
 *  ICCreateGURLEvent()   *** DEPRECATED ***
 *  
=======
=======
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetVersion( inst: ICInstance; whichVersion: SIGNEDLONG; var version: UInt32 ): OSStatus; external name '_ICGetVersion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r4] [c3] [b3] 
   * Returns the version of Internet Config.  Pass kICComponentVersion
   * to get the version as previously returned by GetComponenVerson.
   * Pass kICNumVersion to get a NumVersion structure.
   }
{
 *  ICGetConfigName()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
>>>>>>> origin/cpstrnew
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
<<<<<<< HEAD
function ICDeletePref( inst: ICInstance; const (*var*) key: Str255 ): OSStatus; external name '_ICDeletePref';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [c1] [b2] 
   * Deletes the preference specified by key.
   * key must not be the empty string.
   * Returns icPrefNotFound if the preference specified by key is not present.
   }
{
 *  ICEnd()
 *  
=======
function ICGetConfigName( inst: ICInstance; longname: Boolean; var name: Str255 ): OSStatus; external name '_ICGetConfigName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r2] [c1] [b3] 
   * Returns a string that describes the current configuration at a user
   * level. Set longname to true if you want a long name, up to 255
   * characters, or false if you want a short name, typically about 32
   * characters.
   * The returned string is for user display only. If you rely on the
   * exact format of it, you will conflict with any future IC
   * implementation that doesn't use explicit preference files.
   }
{ ***** Getting Information *****  }
{
 *  ICGetSeed()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetSeed( inst: ICInstance; var seed: SIGNEDLONG ): OSStatus; external name '_ICGetSeed';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [c3] [b3] 
   * Returns the current seed for the IC prefs database.
   * This seed changes each time a non-volatile preference is changed.
   * You can poll this to determine if any cached preferences change.
   }
{
 *  ICGetPerm()
 *  
>>>>>>> origin/cpstrnew
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
function ICDeletePref( inst: ICInstance; const (*var*) key: Str255 ): OSStatus; external name '_ICDeletePref';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [c1] [b2] 
   * Deletes the preference specified by key.
   * key must not be the empty string.
   * Returns icPrefNotFound if the preference specified by key is not present.
   }
{
 *  ICEnd()
=======
function ICEnd( inst: ICInstance ): OSStatus; external name '_ICEnd';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [c1] [b1] 
   * Terminates a preference session, as started by ICBegin.
   * You must have called ICBegin before calling this routine.
   }
{
 *  ICGetDefaultPref()
>>>>>>> graemeg/cpstrnew
=======
function ICGetPerm( inst: ICInstance; var perm: ICPerm ): OSStatus; external name '_ICGetPerm';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [c3] [b3] 
   * Returns the access permissions currently associated with this instance.
   * While applications normally know what permissions they have,
   * this routine is designed for use by override components.
   }
{ ***** Reading and Writing Preferences *****  }
{
 *  ICBegin()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICBegin( inst: ICInstance; perm: ICPerm ): OSStatus; external name '_ICBegin';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [c1] [b5] 
   * Starting reading or writing multiple preferences.
   * A call to this must be balanced by a call to ICEnd.
   * Do not call WaitNextEvent between these calls.
   * The perm specifies whether you intend to read or read/write.
   * Only one writer is allowed per instance.
   * Note that this may open resource files that are not closed
   * until you call ICEnd.
   }
{
 *  ICGetPref()
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
function ICEnd( inst: ICInstance ): OSStatus; external name '_ICEnd';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [c1] [b1] 
   * Terminates a preference session, as started by ICBegin.
   * You must have called ICBegin before calling this routine.
   }
{
 *  ICGetDefaultPref()
 *  
>>>>>>> graemeg/cpstrnew
=======
function ICGetDefaultPref( inst: ICInstance; const (*var*) key: Str255; prefH: Handle ): OSStatus; external name '_ICGetDefaultPref';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r4] [c3] [b5] 
   * Returns a default preference value for the specified key.  You
   * must pass in a valid prefH, which is resized to fit the data.
   }
{ ***** User Interface Stuff *****  }
{
 *  ICEditPreferences()
 *  
>>>>>>> graemeg/cpstrnew
=======
function ICGetPref( inst: ICInstance; const (*var*) key: Str255; var attr: ICAttr; buf: UnivPtr; var size: SIGNEDLONG ): OSStatus; external name '_ICGetPref';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [c1] [b4] 
   * Reads the preference specified by key from the IC database to the
   * buffer pointed to by buf and size.
   * key must not be the empty string.
   * If buf is nil then no data is returned.
   * size must be non-negative.
   * attr and size are always set on return. On errors (except icTruncatedErr)
   * attr is set to ICattr_no_change and size is set to 0.
   * size is the actual size of the data.
   * attr is set to the attributes associated with the preference.
   * If this routine returns icTruncatedErr then the other returned
   * values are valid except that only the first size bytes have been
   * return. size is adjusted to reflect the true size of the preference.
   * Returns icPrefNotFound if there is no preference for the key.
   }
{
 *  ICSetPref()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICSetPref( inst: ICInstance; const (*var*) key: Str255; attr: ICAttr; buf: {const} UnivPtr; size: SIGNEDLONG ): OSStatus; external name '_ICSetPref';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [c1] [b4] 
   * Sets the preference specified by key from the IC database to the
   * value pointed to by buf and size.
   * key must not be the empty string.
   * size must be non-negative. 
   * If buf is nil then the preference value is not set and size is ignored.
   * If buf is not nil then the preference value is set to the size
   * bytes pointed to by buf.
   * If attr is ICattr_no_change then the preference attributes are not set.
   * Otherwise the preference attributes are set to attr.
   * Returns icPermErr if the previous ICBegin was passed icReadOnlyPerm.
   * Returns icPermErr if current attr is locked, new attr is locked and buf <> nil.
   }
{
 *  ICFindPrefHandle()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICFindPrefHandle( inst: ICInstance; const (*var*) key: Str255; var attr: ICAttr; prefh: Handle ): OSStatus; external name '_ICFindPrefHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r2] [c1] [b4] 
   * This routine effectively replaces ICGetPrefHandle.
   * Reads the preference specified by key from the IC database into
   * a handle, prefh.
   * key must not be the empty string.
   * attr is set to the attributes associated with the preference.
   * You must set prefh to a non-nil handle before calling this routine.
   * If the preference does not exist, icPrefNotFoundErr is returned.
   }
{
 *  ICGetPrefHandle()
 *  
>>>>>>> origin/cpstrnew
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.7
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICCreateGURLEvent( inst: ICInstance; helperCreator: OSType; urlH: Handle; var theEvent: AppleEvent ): OSStatus; external name '_ICCreateGURLEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)


{ [r4] [c1] [b3] 
   * Creates a GURL Apple event, targetted at the application whose creator
   * code is helperCreator, with a direct object containing the URL text from urlH.
   }
{
 *  ICSendGURLEvent()   *** DEPRECATED ***
 *  
=======
 *  ICParseURL()
 *  
=======
=======
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetPrefHandle( inst: ICInstance; const (*var*) key: Str255; var attr: ICAttr; var prefh: Handle ): OSStatus; external name '_ICGetPrefHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r1] [c1] [b4] 
   * This routine is now obsolete. Use ICFindPrefHandle instead.
   * Reads the preference specified by key from the IC database into
   * a newly created handle, prefh.
   * key must not be the empty string.
   * attr is set to the attributes associated with the preference.
   * The incoming value of prefh is ignored.
   * A new handle is created in the current heap and returned in prefh.
   * If the routine returns an error, prefh is set to nil.
   * If the preference does not exist, no error is returned and prefh is set
   * to an empty handle.
   }
{
 *  ICSetPrefHandle()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICSetPrefHandle( inst: ICInstance; const (*var*) key: Str255; attr: ICAttr; prefh: Handle ): OSStatus; external name '_ICSetPrefHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r1] [c1] [b4] 
   * Sets the preference specified by key from the IC database to the
   * value contained in prefh.
   * key must not be the empty string.
   * If prefh is nil then the preference value is not set.
   * If prefh is not nil then the preference value is set to the data
   * contained in it.
   * If attr is ICattr_no_change then the preference attributes are not set.
   * Otherwise the preference attributes are set to attr.
   * Returns icPermErr if the previous ICBegin was passed icReadOnlyPerm.
   * Returns icPermErr if current attr is locked, new attr is locked and prefh <> nil.
   }
{
 *  ICCountPref()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
>>>>>>> origin/cpstrnew
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
<<<<<<< HEAD
function ICEditPreferences( inst: ICInstance; const (*var*) key: Str255 ): OSStatus; external name '_ICEditPreferences';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r1] [c1] [b3] 
   * Instructs IC to display the user interface associated with editing
   * preferences and focusing on the preference specified by key.
   * If key is the empty string then no preference should be focused upon.
   * You must have specified a configuration before calling this routine.
   * You do not need to call ICBegin before calling this routine.
   * In the current implementation this launches the IC application
   * (or brings it to the front) and displays the window containing
   * the preference specified by key.
   * It may have a radically different implementation in future
   * IC systems.
   }
{ ***** URL Handling *****  }
{
 *  ICLaunchURL()
 *  
=======
function ICCountPref( inst: ICInstance; var count: SIGNEDLONG ): OSStatus; external name '_ICCountPref';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [c1] [b1] 
   * Counts the total number of preferences.
   * If the routine returns an error, count is set to 0.
   }
{
 *  ICGetIndPref()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetIndPref( inst: ICInstance; index: SIGNEDLONG; var key: Str255 ): OSStatus; external name '_ICGetIndPref';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [c1] [b1] 
   * Returns the key of the index'th preference.
   * index must be positive.
   * Returns icPrefNotFoundErr if index is greater than the total number of preferences.
   * If the routine returns an error, key is undefined.
   }
{
 *  ICDeletePref()
 *  
>>>>>>> origin/cpstrnew
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
function ICGetDefaultPref( inst: ICInstance; const (*var*) key: Str255; prefH: Handle ): OSStatus; external name '_ICGetDefaultPref';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r4] [c3] [b5] 
   * Returns a default preference value for the specified key.  You
   * must pass in a valid prefH, which is resized to fit the data.
   }
{ ***** User Interface Stuff *****  }
{
 *  ICEditPreferences()
=======
function ICLaunchURL( inst: ICInstance; const (*var*) hint: Str255; data: {const} UnivPtr; len: SIGNEDLONG; var selStart: SIGNEDLONG; var selEnd: SIGNEDLONG ): OSStatus; external name '_ICLaunchURL';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r1] [c1] [b3] 
   * Parses a URL out of the specified text and feeds it off to the
   * appropriate helper.
   * hint indicates the default scheme for URLs of the form "name@address".
   * If hint is the empty string then URLs of that form are not allowed.
   * data points to the start of the text. It must not be nil.
   * len indicates the length of the text. It must be non-negative.
   * selStart and selEnd should be passed in as the current selection of
   * the text. This selection is given in the same manner as TextEdit,
   * ie if selStart = selEnd then there is no selection only an insertion
   * point. Also selStart � selEnd and 0 � selStart � len and 0 � selEnd � len.
   * selStart and selEnd are returned as the bounds of the URL. If the
   * routine returns an error then these new boundaries may be
   * invalid but they will be close.
   * The URL is parsed out of the text and passed off to the appropriate
   * helper using the GURL AppleEvent.
   }
{
 *  ICParseURL()
>>>>>>> graemeg/cpstrnew
 *  
=======
function ICDeletePref( inst: ICInstance; const (*var*) key: Str255 ): OSStatus; external name '_ICDeletePref';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [c1] [b2] 
   * Deletes the preference specified by key.
   * key must not be the empty string.
   * Returns icPrefNotFound if the preference specified by key is not present.
   }
{
 *  ICEnd()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICEnd( inst: ICInstance ): OSStatus; external name '_ICEnd';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [c1] [b1] 
   * Terminates a preference session, as started by ICBegin.
   * You must have called ICBegin before calling this routine.
   }
{
 *  ICGetDefaultPref()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetDefaultPref( inst: ICInstance; const (*var*) key: Str255; prefH: Handle ): OSStatus; external name '_ICGetDefaultPref';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r4] [c3] [b5] 
   * Returns a default preference value for the specified key.  You
   * must pass in a valid prefH, which is resized to fit the data.
   }
{ ***** User Interface Stuff *****  }
{
 *  ICEditPreferences()
 *  
>>>>>>> origin/cpstrnew
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
function ICEditPreferences( inst: ICInstance; const (*var*) key: Str255 ): OSStatus; external name '_ICEditPreferences';
=======
function ICParseURL( inst: ICInstance; const (*var*) hint: Str255; data: {const} UnivPtr; len: SIGNEDLONG; var selStart: SIGNEDLONG; var selEnd: SIGNEDLONG; url: Handle ): OSStatus; external name '_ICParseURL';
>>>>>>> graemeg/cpstrnew
=======
function ICEditPreferences( inst: ICInstance; const (*var*) key: Str255 ): OSStatus; external name '_ICEditPreferences';
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r1] [c1] [b3] 
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/cpstrnew
   * Instructs IC to display the user interface associated with editing
   * preferences and focusing on the preference specified by key.
   * If key is the empty string then no preference should be focused upon.
   * You must have specified a configuration before calling this routine.
   * You do not need to call ICBegin before calling this routine.
   * In the current implementation this launches the IC application
   * (or brings it to the front) and displays the window containing
   * the preference specified by key.
   * It may have a radically different implementation in future
   * IC systems.
   }
{ ***** URL Handling *****  }
{
 *  ICLaunchURL()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICLaunchURL( inst: ICInstance; const (*var*) hint: Str255; data: {const} UnivPtr; len: SIGNEDLONG; var selStart: SIGNEDLONG; var selEnd: SIGNEDLONG ): OSStatus; external name '_ICLaunchURL';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r1] [c1] [b3] 
   * Parses a URL out of the specified text and feeds it off to the
   * appropriate helper.
   * hint indicates the default scheme for URLs of the form "name@address".
   * If hint is the empty string then URLs of that form are not allowed.
   * data points to the start of the text. It must not be nil.
   * len indicates the length of the text. It must be non-negative.
   * selStart and selEnd should be passed in as the current selection of
   * the text. This selection is given in the same manner as TextEdit,
   * ie if selStart = selEnd then there is no selection only an insertion
   * point. Also selStart � selEnd and 0 � selStart � len and 0 � selEnd � len.
   * selStart and selEnd are returned as the bounds of the URL. If the
   * routine returns an error then these new boundaries may be
   * invalid but they will be close.
   * The URL is parsed out of the text and passed off to the appropriate
   * helper using the GURL AppleEvent.
   }
{
 *  ICParseURL()
<<<<<<< HEAD
 *  
>>>>>>> graemeg/cpstrnew
=======
=======
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICParseURL( inst: ICInstance; const (*var*) hint: Str255; data: {const} UnivPtr; len: SIGNEDLONG; var selStart: SIGNEDLONG; var selEnd: SIGNEDLONG; url: Handle ): OSStatus; external name '_ICParseURL';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r1] [c1] [b3] 
>>>>>>> origin/cpstrnew
   * Parses a URL out of the specified text and returns it in a canonical form
   * in a handle.
   * hint indicates the default scheme for URLs of the form "name@address".
   * If hint is the empty string then URLs of that form are not allowed.
   * data points to the start of the text. It must not be nil.
   * len indicates the length of the text. It must be non-negative.
   * selStart and selEnd should be passed in as the current selection of
   * the text. This selection is given in the same manner as TextEdit,
   * ie if selStart = selEnd then there is no selection only an insertion
   * point. Also selStart � selEnd and 0 � selStart � len and 0 � selEnd � len.
   * selStart and selEnd are returned as the bounds of the URL. If the
   * routine returns an error then these new boundaries may be
   * invalid but they will be close.
   * The incoming url handle must not be nil.  The resulting URL is normalised
   * and copied into the url handle, which is resized to fit.
   }
{
 *  ICCreateGURLEvent()
<<<<<<< HEAD
 *  
>>>>>>> graemeg/cpstrnew
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
<<<<<<< HEAD
function ICParseURL( inst: ICInstance; const (*var*) hint: Str255; data: {const} UnivPtr; len: SIGNEDLONG; var selStart: SIGNEDLONG; var selEnd: SIGNEDLONG; url: Handle ): OSStatus; external name '_ICParseURL';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r1] [c1] [b3] 
   * Parses a URL out of the specified text and returns it in a canonical form
   * in a handle.
   * hint indicates the default scheme for URLs of the form "name@address".
   * If hint is the empty string then URLs of that form are not allowed.
   * data points to the start of the text. It must not be nil.
   * len indicates the length of the text. It must be non-negative.
   * selStart and selEnd should be passed in as the current selection of
   * the text. This selection is given in the same manner as TextEdit,
   * ie if selStart = selEnd then there is no selection only an insertion
   * point. Also selStart � selEnd and 0 � selStart � len and 0 � selEnd � len.
   * selStart and selEnd are returned as the bounds of the URL. If the
   * routine returns an error then these new boundaries may be
   * invalid but they will be close.
   * The incoming url handle must not be nil.  The resulting URL is normalised
   * and copied into the url handle, which is resized to fit.
   }
{
 *  ICCreateGURLEvent()
 *  
=======
 *  
>>>>>>> origin/cpstrnew
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICCreateGURLEvent( inst: ICInstance; helperCreator: OSType; urlH: Handle; var theEvent: AppleEvent ): OSStatus; external name '_ICCreateGURLEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r4] [c1] [b3] 
   * Creates a GURL Apple event, targetted at the application whose creator
   * code is helperCreator, with a direct object containing the URL text from urlH.
   }
{
 *  ICSendGURLEvent()
<<<<<<< HEAD
 *  
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.7
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
function ICCreateGURLEvent( inst: ICInstance; helperCreator: OSType; urlH: Handle; var theEvent: AppleEvent ): OSStatus; external name '_ICCreateGURLEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r4] [c1] [b3] 
   * Creates a GURL Apple event, targetted at the application whose creator
   * code is helperCreator, with a direct object containing the URL text from urlH.
   }
{
 *  ICSendGURLEvent()
 *  
=======
 *  
>>>>>>> origin/cpstrnew
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICSendGURLEvent( inst: ICInstance; var theEvent: AppleEvent ): OSStatus; external name '_ICSendGURLEvent';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
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


{ [r4] [c1] [b3] 
   * Sends theEvent to the target application.
   }
{ ***** Mappings Routines *****
 * 
 * Routines for interrogating mappings database.
 * 
 * ----- High Level Routines -----
  }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  ICMapFilename()   *** DEPRECATED ***
=======
 *  ICMapFilename()
>>>>>>> graemeg/cpstrnew
=======
 *  ICMapFilename()
>>>>>>> graemeg/cpstrnew
=======
 *  ICMapFilename()
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.7
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICMapFilename( inst: ICInstance; const (*var*) filename: Str255; var entry: ICMapEntry ): OSStatus; external name '_ICMapFilename';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew


{ [r1] [c1] [b4] 
   * Takes the name of an incoming file and returns the most appropriate
   * mappings database entry, based on its extension.
   * filename must not be the empty string.
   * Returns icPrefNotFoundErr if no suitable entry is found.
   }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  ICMapTypeCreator()   *** DEPRECATED ***
=======
 *  ICMapTypeCreator()
>>>>>>> graemeg/cpstrnew
=======
 *  ICMapTypeCreator()
>>>>>>> graemeg/cpstrnew
=======
 *  ICMapTypeCreator()
>>>>>>> graemeg/cpstrnew
 *  
=======
 *  ICMapFilename()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICMapFilename( inst: ICInstance; const (*var*) filename: Str255; var entry: ICMapEntry ): OSStatus; external name '_ICMapFilename';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r1] [c1] [b4] 
   * Takes the name of an incoming file and returns the most appropriate
   * mappings database entry, based on its extension.
   * filename must not be the empty string.
   * Returns icPrefNotFoundErr if no suitable entry is found.
   }
{
 *  ICMapTypeCreator()
 *  
>>>>>>> origin/cpstrnew
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.7
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICMapTypeCreator( inst: ICInstance; fType: OSType; fCreator: OSType; const (*var*) filename: Str255; var entry: ICMapEntry ): OSStatus; external name '_ICMapTypeCreator';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
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


{ [r1] [c1] [b4] 
   * Takes the type and creator (and optionally the name) of an outgoing
   * file and returns the most appropriate mappings database entry.
   * The filename may be either the name of the outgoing file or
   * the empty string.
   * Returns icPrefNotFoundErr if no suitable entry found.
   }
{ ----- Mid Level Routines -----  }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  ICMapEntriesFilename()   *** DEPRECATED ***
=======
 *  ICMapEntriesFilename()
>>>>>>> graemeg/cpstrnew
=======
 *  ICMapEntriesFilename()
>>>>>>> graemeg/cpstrnew
=======
 *  ICMapEntriesFilename()
>>>>>>> graemeg/cpstrnew
=======
 *  ICMapEntriesFilename()
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.7
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICMapEntriesFilename( inst: ICInstance; entries: Handle; const (*var*) filename: Str255; var entry: ICMapEntry ): OSStatus; external name '_ICMapEntriesFilename';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
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


{ [r1] [c1] [b3] 
   * Takes the name of an incoming file and returns the most appropriate
   * mappings database entry, based on its extension.
   * entries must be a handle to a valid IC mappings database preference.
   * filename must not be the empty string.
   * Returns icPrefNotFoundErr if no suitable entry is found.
   }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  ICMapEntriesTypeCreator()   *** DEPRECATED ***
=======
 *  ICMapEntriesTypeCreator()
>>>>>>> graemeg/cpstrnew
=======
 *  ICMapEntriesTypeCreator()
>>>>>>> graemeg/cpstrnew
=======
 *  ICMapEntriesTypeCreator()
>>>>>>> graemeg/cpstrnew
=======
 *  ICMapEntriesTypeCreator()
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.7
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICMapEntriesTypeCreator( inst: ICInstance; entries: Handle; fType: OSType; fCreator: OSType; const (*var*) filename: Str255; var entry: ICMapEntry ): OSStatus; external name '_ICMapEntriesTypeCreator';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
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


{ [r1] [c1] [b3] 
   * Takes the type and creator (and optionally the name) of an outgoing
   * file and returns the most appropriate mappings database entry.
   * entries must be a handle to a valid IC mappings database preference.
   * The filename may be either the name of the outgoing file or
   * the empty string.
   * Returns icPrefNotFoundErr if no suitable entry found.
   }
{ ----- Low Level Routines -----  }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  ICCountMapEntries()   *** DEPRECATED ***
=======
 *  ICCountMapEntries()
>>>>>>> graemeg/cpstrnew
=======
 *  ICCountMapEntries()
>>>>>>> graemeg/cpstrnew
=======
 *  ICCountMapEntries()
>>>>>>> graemeg/cpstrnew
=======
 *  ICCountMapEntries()
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.7
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICCountMapEntries( inst: ICInstance; entries: Handle; var count: SIGNEDLONG ): OSStatus; external name '_ICCountMapEntries';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
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


{ [r1] [c1] [b3] 
   * Counts the number of entries in the mappings database.
   * entries must be a handle to a valid IC mappings database preference.
   * count is set to the number of entries.
   }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  ICGetIndMapEntry()   *** DEPRECATED ***
=======
 *  ICGetIndMapEntry()
>>>>>>> graemeg/cpstrnew
=======
 *  ICGetIndMapEntry()
>>>>>>> graemeg/cpstrnew
=======
 *  ICGetIndMapEntry()
>>>>>>> graemeg/cpstrnew
=======
 *  ICGetIndMapEntry()
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.7
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetIndMapEntry( inst: ICInstance; entries: Handle; index: SIGNEDLONG; var pos: SIGNEDLONG; var entry: ICMapEntry ): OSStatus; external name '_ICGetIndMapEntry';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
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


{ [r1] [c1] [b3] 
   * Gets the index'th entry in the mappings database.
   * entries must be a handle to a valid IC mappings database preference.
   * index must be in the range from 1 to the number of entries in the database.
   * The value of pos is ignored on input. pos is set to the position of
   * the index'th entry in the database and is suitable for passing back
   * into ICSetMapEntry.
   * Does not return any user data associated with the entry.
   }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  ICGetMapEntry()   *** DEPRECATED ***
=======
 *  ICGetMapEntry()
>>>>>>> graemeg/cpstrnew
=======
 *  ICGetMapEntry()
>>>>>>> graemeg/cpstrnew
=======
 *  ICGetMapEntry()
>>>>>>> graemeg/cpstrnew
=======
 *  ICGetMapEntry()
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.7
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetMapEntry( inst: ICInstance; entries: Handle; pos: SIGNEDLONG; var entry: ICMapEntry ): OSStatus; external name '_ICGetMapEntry';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
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


{ [r1] [c1] [b3] 
   * Returns the entry located at position pos in the mappings database.
   * entries must be a handle to a valid IC mappings database preference.
   * pos should be 0 to get the first entry. To get the subsequent entries, add
   * entry.total_size to pos and iterate.
   * Does not return any user data associated with the entry.
   }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  ICSetMapEntry()   *** DEPRECATED ***
=======
 *  ICSetMapEntry()
>>>>>>> graemeg/cpstrnew
=======
 *  ICSetMapEntry()
>>>>>>> graemeg/cpstrnew
=======
 *  ICSetMapEntry()
>>>>>>> graemeg/cpstrnew
=======
 *  ICSetMapEntry()
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.7
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICSetMapEntry( inst: ICInstance; entries: Handle; pos: SIGNEDLONG; const (*var*) entry: ICMapEntry ): OSStatus; external name '_ICSetMapEntry';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
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


{ [r1] [c1] [b3] 
   * Sets the entry located at position pos in the mappings database.
   * entries must be a handle to a valid IC mappings database preference.
   * pos should be either a value returned from ICGetIndMapEntry or a value
   * calculated using ICGetMapEntry.
   * entry is a var parameter purely for stack space reasons. It is not
   * modified in any way.
   * Any user data associated with the entry is unmodified.
   }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  ICDeleteMapEntry()   *** DEPRECATED ***
=======
 *  ICDeleteMapEntry()
>>>>>>> graemeg/cpstrnew
=======
 *  ICDeleteMapEntry()
>>>>>>> graemeg/cpstrnew
=======
 *  ICDeleteMapEntry()
>>>>>>> graemeg/cpstrnew
=======
 *  ICDeleteMapEntry()
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.7
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICDeleteMapEntry( inst: ICInstance; entries: Handle; pos: SIGNEDLONG ): OSStatus; external name '_ICDeleteMapEntry';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
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


{ [r1] [c1] [b3] 
   * Deletes the mappings database entry at pos.
   * entries must be a handle to a valid IC mappings database preference.
   * pos should be either a value returned from ICGetIndMapEntry or a value
   * calculated using ICGetMapEntry.
   * Also deletes any user data associated with the entry.
   }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  ICAddMapEntry()   *** DEPRECATED ***
=======
 *  ICAddMapEntry()
>>>>>>> graemeg/cpstrnew
=======
 *  ICAddMapEntry()
>>>>>>> graemeg/cpstrnew
=======
 *  ICAddMapEntry()
>>>>>>> graemeg/cpstrnew
=======
 *  ICAddMapEntry()
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.7
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICAddMapEntry( inst: ICInstance; entries: Handle; const (*var*) entry: ICMapEntry ): OSStatus; external name '_ICAddMapEntry';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
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


{ [r1] [c1] [b3] 
   * Adds an entry to the mappings database.
   * entries must be a handle to a valid IC mappings database preference.
   * The entry is added to the end of the entries database.
   * No user data is added.
   }
{ ***** Profile Management Routines *****  }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  ICGetCurrentProfile()   *** DEPRECATED ***
=======
 *  ICGetCurrentProfile()
>>>>>>> graemeg/cpstrnew
=======
 *  ICGetCurrentProfile()
>>>>>>> graemeg/cpstrnew
=======
 *  ICGetCurrentProfile()
>>>>>>> graemeg/cpstrnew
=======
 *  ICGetCurrentProfile()
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.7
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetCurrentProfile( inst: ICInstance; var currentID: ICProfileID ): OSStatus; external name '_ICGetCurrentProfile';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
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


{ [r3] [c1] [b3] 
   * Returns the profile ID of the current profile.
   }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  ICSetCurrentProfile()   *** DEPRECATED ***
=======
 *  ICSetCurrentProfile()
>>>>>>> graemeg/cpstrnew
=======
 *  ICSetCurrentProfile()
>>>>>>> graemeg/cpstrnew
=======
 *  ICSetCurrentProfile()
>>>>>>> graemeg/cpstrnew
=======
 *  ICSetCurrentProfile()
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.7
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICSetCurrentProfile( inst: ICInstance; newID: ICProfileID ): OSStatus; external name '_ICSetCurrentProfile';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
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


{ [r3] [c1] [b3] 
   * Sets the current profile to the profile specified in newProfile.
   }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  ICCountProfiles()   *** DEPRECATED ***
=======
 *  ICCountProfiles()
>>>>>>> graemeg/cpstrnew
=======
 *  ICCountProfiles()
>>>>>>> graemeg/cpstrnew
=======
 *  ICCountProfiles()
>>>>>>> graemeg/cpstrnew
=======
 *  ICCountProfiles()
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.7
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICCountProfiles( inst: ICInstance; var count: SIGNEDLONG ): OSStatus; external name '_ICCountProfiles';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
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


{ [r3] [c1] [b1] 
   * Returns the total number of profiles.
   }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  ICGetIndProfile()   *** DEPRECATED ***
=======
 *  ICGetIndProfile()
>>>>>>> graemeg/cpstrnew
=======
 *  ICGetIndProfile()
>>>>>>> graemeg/cpstrnew
=======
 *  ICGetIndProfile()
>>>>>>> graemeg/cpstrnew
=======
 *  ICGetIndProfile()
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.7
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetIndProfile( inst: ICInstance; index: SIGNEDLONG; var thisID: ICProfileID ): OSStatus; external name '_ICGetIndProfile';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
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


{ [r3] [c1] [b1] 
   * Returns the profile ID of the index'th profile.  index must be positive.
   * Returns icProfileNotFoundErr if index is greater than the total number
   * of profiles.
   }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  ICGetProfileName()   *** DEPRECATED ***
=======
 *  ICGetProfileName()
>>>>>>> graemeg/cpstrnew
=======
 *  ICGetProfileName()
>>>>>>> graemeg/cpstrnew
=======
 *  ICGetProfileName()
>>>>>>> graemeg/cpstrnew
=======
 *  ICGetProfileName()
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.7
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetProfileName( inst: ICInstance; thisID: ICProfileID; var name: Str255 ): OSStatus; external name '_ICGetProfileName';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
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


{ [r3] [c1] [b3] 
   * Returns the name of a profile given its ID.  The name may not uniquely
   * identify the profile.  [That's what the profile ID is for!]  The name
   * is assumed to be in the system script.
   }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  ICSetProfileName()   *** DEPRECATED ***
=======
 *  ICSetProfileName()
>>>>>>> graemeg/cpstrnew
=======
 *  ICSetProfileName()
>>>>>>> graemeg/cpstrnew
=======
 *  ICSetProfileName()
>>>>>>> graemeg/cpstrnew
=======
 *  ICSetProfileName()
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.7
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICSetProfileName( inst: ICInstance; thisID: ICProfileID; const (*var*) name: Str255 ): OSStatus; external name '_ICSetProfileName';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
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


{ [r3] [c1] [b3] 
   * This routine sets the name of the specified profile.  Profile names
   * need not be unique.  The name should be in the system script.
   }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  ICAddProfile()   *** DEPRECATED ***
=======
 *  ICAddProfile()
>>>>>>> graemeg/cpstrnew
=======
 *  ICAddProfile()
>>>>>>> graemeg/cpstrnew
=======
 *  ICAddProfile()
>>>>>>> graemeg/cpstrnew
=======
 *  ICAddProfile()
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.7
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICAddProfile( inst: ICInstance; prototypeID: ICProfileID; var newID: ICProfileID ): OSStatus; external name '_ICAddProfile';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
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


{ [r3] [c1] [b2] 
   * If prototypeID = kICNilProfileID, this routine
   * creates a default profile, otherwise it creates a
   * profile by cloning the prototype profile.  The ID
   * of the new profile is returned in newID.
   * The new profile will be give a new, unique, name.
   * This does not switch to the new profile.
   }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  ICDeleteProfile()   *** DEPRECATED ***
=======
 *  ICDeleteProfile()
>>>>>>> graemeg/cpstrnew
=======
 *  ICDeleteProfile()
>>>>>>> graemeg/cpstrnew
=======
 *  ICDeleteProfile()
>>>>>>> graemeg/cpstrnew
=======
 *  ICDeleteProfile()
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework but deprecated in 10.7
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICDeleteProfile( inst: ICInstance; thisID: ICProfileID ): OSStatus; external name '_ICDeleteProfile';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
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


{ [r3] [c1] [b2] 
   * This routine deletes the profile specified by
   * thisID.  Attempting to delete the current profile
   * or the last profile will return error.
   }

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======

=======
	ICServices = record
		count: SInt16;
		services: array [0..0] of ICServiceEntry;
	end;
	ICServicesPtr = ^ICServices;
type
	ICServicesHandle = ^ICServicesPtr;
{***********************************************************************************************
  keys
 ***********************************************************************************************}
{ 
    key reserved for use by Internet Config 
}
>>>>>>> origin/fixes_2.4
const
	kICReservedKey = 'kICReservedKey';
{
    STR# -- formatted, list of Archie servers  
}
const
	kICArchieAll = 'ArchieAll';
{
    PString -- formatted, preferred Archie server   
}
const
	kICArchiePreferred = 'ArchiePreferred';
{
    ICCharTable -- Mac-to-Net and Net-to-Mac character translation   
}
const
	kICCharacterSet = 'CharacterSet';
{
    ICFontRecord -- font used for proportional text   
}
const
	kICDocumentFont = 'DocumentFont';
{
    ICFileSpec -- where to put newly downloaded files   
}
const
	kICDownloadFolder = 'DownloadFolder';
{
    PString -- user@host.domain, email address of user, ie return address   
}
const
	kICEmail = 'Email';
{
    PString -- host.domain, default FTP server   
}
const
	kICFTPHost = 'FTPHost';
{
    PString -- second level FTP proxy authorisation   
}
const
	kICFTPProxyAccount = 'FTPProxyAccount';
{
    PString -- host.domain   
}
const
	kICFTPProxyHost = 'FTPProxyHost';
{
    PString -- scrambled, password for FTPProxyUser   
}
const
	kICFTPProxyPassword = 'FTPProxyPassword';
{
    PString -- first level FTP proxy authorisation   
}
const
	kICFTPProxyUser = 'FTPProxyUser';
{
    PString -- host.domain, default finger server   
}
const
	kICFingerHost = 'FingerHost';
{
    PString -- host.domain, default Gopher server   
}
const
	kICGopherHost = 'GopherHost';
{
    PString -- host.domain, see note in Prog Docs   
}
const
	kICGopherProxy = 'GopherProxy';
{
    PString -- host.domain   
}
const
	kICHTTPProxyHost = 'HTTPProxyHost';
{
    ICAppSpec -- helpers for URL schemes   
}
const
	kICHelper = 'Helper�';
{
    PString -- description for URL scheme   
}
const
	kICHelperDesc = 'HelperDesc�';
{
    ICAppSpecList -- list of common helpers for URL schemes   
}
const
	kICHelperList = 'HelperList�';
{
    PString -- host.domain, Internet Relay Chat server   
}
const
	kICIRCHost = 'IRCHost';
{
    STR# -- formatted, list of Info-Mac servers   
}
const
	kICInfoMacAll = 'InfoMacAll';
{
    PString -- formatted, preferred Info-Mac server   
}
const
	kICInfoMacPreferred = 'InfoMacPreferred';
{
    PString -- string LDAP thing   
}
const
	kICLDAPSearchbase = 'LDAPSearchbase';
{
    PString -- host.domain   
}
const
	kICLDAPServer = 'LDAPServer';
{
    ICFontRecord -- font used for lists of items (eg news article lists)   
}
const
	kICListFont = 'ListFont';
{
    PString -- host for MacSearch queries   
}
const
	kICMacSearchHost = 'MacSearchHost';
{
    PString -- user@host.domain, account from which to fetch mail   
}
const
	kICMailAccount = 'MailAccount';
{
    TEXT -- extra headers for mail messages   
}
const
	kICMailHeaders = 'MailHeaders';
{
    PString -- scrambled, password for MailAccount   
}
const
	kICMailPassword = 'MailPassword';
{
    ICMapEntries -- file type mapping, see documentation   
}
const
	kICMapping = 'Mapping';
{
    PString -- host.domain, NNTP server   
}
const
	kICNNTPHost = 'NNTPHost';
{
<<<<<<< HEAD
>>>>>>> origin/fixes_2_2
 *  ICCreateGURLEvent()
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Mac OS X:         in version 10.0 and later
 }
function ICCreateGURLEvent(inst: ICInstance; helperCreator: OSType; urlH: Handle; var theEvent: AppleEvent): OSStatus; external name '_ICCreateGURLEvent';
{ [r4] [c1] [b3] 
   * Creates a GURL Apple event, targetted at the application whose creator
   * code is helperCreator, with a direct object containing the URL text from urlH.
   }
=======
    PString -- host.domain, Network Time Protocol (NTP)   
}
const
	kICNTPHost = 'NTPHost';
>>>>>>> origin/fixes_2.4
{
    Boolean   
}
const
	kICNewMailDialog = 'NewMailDialog';
{
    Boolean -- how to announce new mail   
}
const
	kICNewMailFlashIcon = 'NewMailFlashIcon';
{
    Boolean   
}
const
	kICNewMailPlaySound = 'NewMailPlaySound';
{
    PString   
}
const
	kICNewMailSoundName = 'NewMailSoundName';
{
    PString -- scrambled, password for NewsAuthUsername   
}
const
	kICNewsAuthPassword = 'NewsAuthPassword';
{
    PString -- user name for authorised news servers   
}
const
	kICNewsAuthUsername = 'NewsAuthUsername';
{
    TEXT -- extra headers for news messages   
}
const
	kICNewsHeaders = 'NewsHeaders';
{
    STR# -- list of domains not to be proxied   
}
const
	kICNoProxyDomains = 'NoProxyDomains';
{
    PString -- for X-Organization string   
}
const
	kICOrganization = 'Organization';
{
    PString -- host.domain, default Ph server   
}
const
	kICPhHost = 'PhHost';
{
    TEXT -- default response for finger servers   
}
const
	kICPlan = 'Plan';
{
    ICFontRecord -- font used to print ScreenFont   
}
const
	kICPrinterFont = 'PrinterFont';
{
    PString -- used to quote responses in news and mail   
}
const
	kICQuotingString = 'QuotingString';
{
    PString -- real name of user   
}
const
	kICRealName = 'RealName';
{
    PString -- RTSP Proxy Host
}
const
	kICRTSPProxyHost = 'RTSPProxyHost';
{
    PString -- host.domain, SMTP server   
}
const
	kICSMTPHost = 'SMTPHost';
{
    ICFontRecord -- font used for monospaced text (eg news articles)   
}
const
	kICScreenFont = 'ScreenFont';
{
    ICServices -- TCP and IP port-to-name mapping   
}
const
	kICServices = 'Services';
{
    TEXT -- append to news and mail messages   
}
const
	kICSignature = 'Signature';
{
    TEXT -- preferred mailing address   
}
const
	kICSnailMailAddress = 'SnailMailAddress';
{
    PString -- host.domain, remember that host.domain format allows ":port" and " port"  
}
const
	kICSocksHost = 'SocksHost';
{
    PString -- host.domain, default Telnet address   
}
const
	kICTelnetHost = 'TelnetHost';
{
    STR# -- formatted, list of UMich servers   
}
const
	kICUMichAll = 'UMichAll';
{
    PString -- formatted, preferred UMich server   
}
const
	kICUMichPreferred = 'UMichPreferred';
{
    Boolean   
}
const
	kICUseFTPProxy = 'UseFTPProxy';
{
    Boolean   
}
const
	kICUseGopherProxy = 'UseGopherProxy';
{
    Boolean   
}
const
	kICUseHTTPProxy = 'UseHTTPProxy';
{
    Boolean -- use PASV command for FTP transfers   
}
const
	kICUsePassiveFTP = 'UsePassiveFTP';
{
    Boolean
}
const
	kICUseRTSPProxy = 'UseRTSPProxy';
{
    Boolean   
}
const
	kICUseSocks = 'UseSocks';
{
    PString -- no idea   
}
const
	kICWAISGateway = 'WAISGateway';
{
    PString -- URL, users default WWW page   
}
const
	kICWWWHomePage = 'WWWHomePage';
{
    RGBColor -- background colour for web pages   
}
const
	kICWebBackgroundColour = 'WebBackgroundColour';
{
    RGBColor -- colour for read links   
}
const
	kICWebReadColor = '646F6777쩦ebReadColor';
{
    PString -- URL, users default search page   
}
const
	kICWebSearchPagePrefs = 'WebSearchPagePrefs';
{
    RGBColor -- colour for normal text   
}
const
	kICWebTextColor = 'WebTextColor';
{
    Boolean -- whether to underline links   
}
const
	kICWebUnderlineLinks = '646F6777쩦ebUnderlineLinks';
{
    RGBColor -- colour for unread links   
}
const
	kICWebUnreadColor = '646F6777쩦ebUnreadColor';
{
    PString -- host.domain, default whois server   
}
const
	kICWhoisHost = 'WhoisHost';

{***********************************************************************************************

      FUNCTIONS

      What do the annotations after each API mean?
      --------------------------------------------

      [r1] Requires IC 1.1 or higher.
      [r2] Requires IC 1.2 or higher.
      [r3] Requires IC 2.0 or higher.
      [r4] Requires IC 2.5 or higher.
      
      IMPORTANT:

      In IC 2.5, instances automatically use the default configuration.
      You no longer need to configure an instance explicitly, except
      if your code might run with an older version of IC.  So the following
      notes only apply to IC 2.0 and earlier.

      [c1]  You must have specified a configuration before calling this routine.
      
      [c2]  You must have specified the default configuration before calling this
            routine.
      
      [c3]  You do not need to specify a configuration before calling this routine.
      
      [b1]  You must be inside a Begin/End pair when calling this routine.
      
      [b2]  You must be inside a Begin/End read/write pair when calling this routine.
      
      [b3]  You do not need to be inside a Begin/End pair when calling this routine.
      
      [b4]  If you are getting or setting multiple preferences, you should make this
            call inside a Begin/End pair. If you do not make this call inside a Begin/End
            pair, the call will automatically do it for you.
      
      [b5]  It is illegal to call this routine inside a Begin/End pair.

 ***********************************************************************************************}

{ ***** Starting Up and Shutting Down *****  }
{
 *  ICStart()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICStart( var inst: ICInstance; signature: OSType ): OSStatus; external name '_ICStart';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Call this at application initialisation. Set signature to a value
   * which has been regsitered with DTS to allow for future expansion
   * of the IC system. Returns inst as a connection to the IC system.
   }
{
 *  ICStop()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICStop( inst: ICInstance ): OSStatus; external name '_ICStop';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [b5] 
   * Call this at application initialisation, after which inst
   * is no longer valid connection to IC.
   }
{
 *  ICGetVersion()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetVersion( inst: ICInstance; whichVersion: SIGNEDLONG; var version: UInt32 ): OSStatus; external name '_ICGetVersion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r4] [c3] [b3] 
   * Returns the version of Internet Config.  Pass kICComponentVersion
   * to get the version as previously returned by GetComponenVerson.
   * Pass kICNumVersion to get a NumVersion structure.
   }
{
 *  ICGetConfigName()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetConfigName( inst: ICInstance; longname: Boolean; var name: Str255 ): OSStatus; external name '_ICGetConfigName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r2] [c1] [b3] 
   * Returns a string that describes the current configuration at a user
   * level. Set longname to true if you want a long name, up to 255
   * characters, or false if you want a short name, typically about 32
   * characters.
   * The returned string is for user display only. If you rely on the
   * exact format of it, you will conflict with any future IC
   * implementation that doesn't use explicit preference files.
   }
{ ***** Getting Information *****  }
{
 *  ICGetSeed()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetSeed( inst: ICInstance; var seed: SIGNEDLONG ): OSStatus; external name '_ICGetSeed';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [c3] [b3] 
   * Returns the current seed for the IC prefs database.
   * This seed changes each time a non-volatile preference is changed.
   * You can poll this to determine if any cached preferences change.
   }
{
 *  ICGetPerm()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetPerm( inst: ICInstance; var perm: ICPerm ): OSStatus; external name '_ICGetPerm';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [c3] [b3] 
   * Returns the access permissions currently associated with this instance.
   * While applications normally know what permissions they have,
   * this routine is designed for use by override components.
   }
{ ***** Reading and Writing Preferences *****  }
{
 *  ICBegin()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICBegin( inst: ICInstance; perm: ICPerm ): OSStatus; external name '_ICBegin';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [c1] [b5] 
   * Starting reading or writing multiple preferences.
   * A call to this must be balanced by a call to ICEnd.
   * Do not call WaitNextEvent between these calls.
   * The perm specifies whether you intend to read or read/write.
   * Only one writer is allowed per instance.
   * Note that this may open resource files that are not closed
   * until you call ICEnd.
   }
{
 *  ICGetPref()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetPref( inst: ICInstance; const (*var*) key: Str255; var attr: ICAttr; buf: UnivPtr; var size: SIGNEDLONG ): OSStatus; external name '_ICGetPref';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [c1] [b4] 
   * Reads the preference specified by key from the IC database to the
   * buffer pointed to by buf and size.
   * key must not be the empty string.
   * If buf is nil then no data is returned.
   * size must be non-negative.
   * attr and size are always set on return. On errors (except icTruncatedErr)
   * attr is set to ICattr_no_change and size is set to 0.
   * size is the actual size of the data.
   * attr is set to the attributes associated with the preference.
   * If this routine returns icTruncatedErr then the other returned
   * values are valid except that only the first size bytes have been
   * return. size is adjusted to reflect the true size of the preference.
   * Returns icPrefNotFound if there is no preference for the key.
   }
{
 *  ICSetPref()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICSetPref( inst: ICInstance; const (*var*) key: Str255; attr: ICAttr; buf: {const} UnivPtr; size: SIGNEDLONG ): OSStatus; external name '_ICSetPref';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [c1] [b4] 
   * Sets the preference specified by key from the IC database to the
   * value pointed to by buf and size.
   * key must not be the empty string.
   * size must be non-negative. 
   * If buf is nil then the preference value is not set and size is ignored.
   * If buf is not nil then the preference value is set to the size
   * bytes pointed to by buf.
   * If attr is ICattr_no_change then the preference attributes are not set.
   * Otherwise the preference attributes are set to attr.
   * Returns icPermErr if the previous ICBegin was passed icReadOnlyPerm.
   * Returns icPermErr if current attr is locked, new attr is locked and buf <> nil.
   }
{
 *  ICFindPrefHandle()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICFindPrefHandle( inst: ICInstance; const (*var*) key: Str255; var attr: ICAttr; prefh: Handle ): OSStatus; external name '_ICFindPrefHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r2] [c1] [b4] 
   * This routine effectively replaces ICGetPrefHandle.
   * Reads the preference specified by key from the IC database into
   * a handle, prefh.
   * key must not be the empty string.
   * attr is set to the attributes associated with the preference.
   * You must set prefh to a non-nil handle before calling this routine.
   * If the preference does not exist, icPrefNotFoundErr is returned.
   }
{
 *  ICGetPrefHandle()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetPrefHandle( inst: ICInstance; const (*var*) key: Str255; var attr: ICAttr; var prefh: Handle ): OSStatus; external name '_ICGetPrefHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r1] [c1] [b4] 
   * This routine is now obsolete. Use ICFindPrefHandle instead.
   * Reads the preference specified by key from the IC database into
   * a newly created handle, prefh.
   * key must not be the empty string.
   * attr is set to the attributes associated with the preference.
   * The incoming value of prefh is ignored.
   * A new handle is created in the current heap and returned in prefh.
   * If the routine returns an error, prefh is set to nil.
   * If the preference does not exist, no error is returned and prefh is set
   * to an empty handle.
   }
{
 *  ICSetPrefHandle()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICSetPrefHandle( inst: ICInstance; const (*var*) key: Str255; attr: ICAttr; prefh: Handle ): OSStatus; external name '_ICSetPrefHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r1] [c1] [b4] 
   * Sets the preference specified by key from the IC database to the
   * value contained in prefh.
   * key must not be the empty string.
   * If prefh is nil then the preference value is not set.
   * If prefh is not nil then the preference value is set to the data
   * contained in it.
   * If attr is ICattr_no_change then the preference attributes are not set.
   * Otherwise the preference attributes are set to attr.
   * Returns icPermErr if the previous ICBegin was passed icReadOnlyPerm.
   * Returns icPermErr if current attr is locked, new attr is locked and prefh <> nil.
   }
{
 *  ICCountPref()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICCountPref( inst: ICInstance; var count: SIGNEDLONG ): OSStatus; external name '_ICCountPref';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [c1] [b1] 
   * Counts the total number of preferences.
   * If the routine returns an error, count is set to 0.
   }
{
 *  ICGetIndPref()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetIndPref( inst: ICInstance; index: SIGNEDLONG; var key: Str255 ): OSStatus; external name '_ICGetIndPref';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [c1] [b1] 
   * Returns the key of the index'th preference.
   * index must be positive.
   * Returns icPrefNotFoundErr if index is greater than the total number of preferences.
   * If the routine returns an error, key is undefined.
   }
{
 *  ICDeletePref()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICDeletePref( inst: ICInstance; const (*var*) key: Str255 ): OSStatus; external name '_ICDeletePref';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [c1] [b2] 
   * Deletes the preference specified by key.
   * key must not be the empty string.
   * Returns icPrefNotFound if the preference specified by key is not present.
   }
{
 *  ICEnd()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICEnd( inst: ICInstance ): OSStatus; external name '_ICEnd';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [c1] [b1] 
   * Terminates a preference session, as started by ICBegin.
   * You must have called ICBegin before calling this routine.
   }
{
 *  ICGetDefaultPref()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetDefaultPref( inst: ICInstance; const (*var*) key: Str255; prefH: Handle ): OSStatus; external name '_ICGetDefaultPref';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r4] [c3] [b5] 
   * Returns a default preference value for the specified key.  You
   * must pass in a valid prefH, which is resized to fit the data.
   }
{ ***** User Interface Stuff *****  }
{
 *  ICEditPreferences()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICEditPreferences( inst: ICInstance; const (*var*) key: Str255 ): OSStatus; external name '_ICEditPreferences';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r1] [c1] [b3] 
   * Instructs IC to display the user interface associated with editing
   * preferences and focusing on the preference specified by key.
   * If key is the empty string then no preference should be focused upon.
   * You must have specified a configuration before calling this routine.
   * You do not need to call ICBegin before calling this routine.
   * In the current implementation this launches the IC application
   * (or brings it to the front) and displays the window containing
   * the preference specified by key.
   * It may have a radically different implementation in future
   * IC systems.
   }
{ ***** URL Handling *****  }
{
 *  ICLaunchURL()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICLaunchURL( inst: ICInstance; const (*var*) hint: Str255; data: {const} UnivPtr; len: SIGNEDLONG; var selStart: SIGNEDLONG; var selEnd: SIGNEDLONG ): OSStatus; external name '_ICLaunchURL';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r1] [c1] [b3] 
   * Parses a URL out of the specified text and feeds it off to the
   * appropriate helper.
   * hint indicates the default scheme for URLs of the form "name@address".
   * If hint is the empty string then URLs of that form are not allowed.
   * data points to the start of the text. It must not be nil.
   * len indicates the length of the text. It must be non-negative.
   * selStart and selEnd should be passed in as the current selection of
   * the text. This selection is given in the same manner as TextEdit,
   * ie if selStart = selEnd then there is no selection only an insertion
   * point. Also selStart � selEnd and 0 � selStart � len and 0 � selEnd � len.
   * selStart and selEnd are returned as the bounds of the URL. If the
   * routine returns an error then these new boundaries may be
   * invalid but they will be close.
   * The URL is parsed out of the text and passed off to the appropriate
   * helper using the GURL AppleEvent.
   }
{
 *  ICParseURL()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICParseURL( inst: ICInstance; const (*var*) hint: Str255; data: {const} UnivPtr; len: SIGNEDLONG; var selStart: SIGNEDLONG; var selEnd: SIGNEDLONG; url: Handle ): OSStatus; external name '_ICParseURL';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r1] [c1] [b3] 
   * Parses a URL out of the specified text and returns it in a canonical form
   * in a handle.
   * hint indicates the default scheme for URLs of the form "name@address".
   * If hint is the empty string then URLs of that form are not allowed.
   * data points to the start of the text. It must not be nil.
   * len indicates the length of the text. It must be non-negative.
   * selStart and selEnd should be passed in as the current selection of
   * the text. This selection is given in the same manner as TextEdit,
   * ie if selStart = selEnd then there is no selection only an insertion
   * point. Also selStart � selEnd and 0 � selStart � len and 0 � selEnd � len.
   * selStart and selEnd are returned as the bounds of the URL. If the
   * routine returns an error then these new boundaries may be
   * invalid but they will be close.
   * The incoming url handle must not be nil.  The resulting URL is normalised
   * and copied into the url handle, which is resized to fit.
   }
{
 *  ICCreateGURLEvent()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICCreateGURLEvent( inst: ICInstance; helperCreator: OSType; urlH: Handle; var theEvent: AppleEvent ): OSStatus; external name '_ICCreateGURLEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r4] [c1] [b3] 
   * Creates a GURL Apple event, targetted at the application whose creator
   * code is helperCreator, with a direct object containing the URL text from urlH.
   }
{
 *  ICSendGURLEvent()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICSendGURLEvent( inst: ICInstance; var theEvent: AppleEvent ): OSStatus; external name '_ICSendGURLEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r4] [c1] [b3] 
   * Sends theEvent to the target application.
   }
{ ***** Mappings Routines *****
 * 
 * Routines for interrogating mappings database.
 * 
 * ----- High Level Routines -----
  }
{
 *  ICMapFilename()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICMapFilename( inst: ICInstance; const (*var*) filename: Str255; var entry: ICMapEntry ): OSStatus; external name '_ICMapFilename';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r1] [c1] [b4] 
   * Takes the name of an incoming file and returns the most appropriate
   * mappings database entry, based on its extension.
   * filename must not be the empty string.
   * Returns icPrefNotFoundErr if no suitable entry is found.
   }
{
 *  ICMapTypeCreator()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICMapTypeCreator( inst: ICInstance; fType: OSType; fCreator: OSType; const (*var*) filename: Str255; var entry: ICMapEntry ): OSStatus; external name '_ICMapTypeCreator';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r1] [c1] [b4] 
   * Takes the type and creator (and optionally the name) of an outgoing
   * file and returns the most appropriate mappings database entry.
   * The filename may be either the name of the outgoing file or
   * the empty string.
   * Returns icPrefNotFoundErr if no suitable entry found.
   }
{ ----- Mid Level Routines -----  }
{
 *  ICMapEntriesFilename()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICMapEntriesFilename( inst: ICInstance; entries: Handle; const (*var*) filename: Str255; var entry: ICMapEntry ): OSStatus; external name '_ICMapEntriesFilename';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r1] [c1] [b3] 
   * Takes the name of an incoming file and returns the most appropriate
   * mappings database entry, based on its extension.
   * entries must be a handle to a valid IC mappings database preference.
   * filename must not be the empty string.
   * Returns icPrefNotFoundErr if no suitable entry is found.
   }
{
 *  ICMapEntriesTypeCreator()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICMapEntriesTypeCreator( inst: ICInstance; entries: Handle; fType: OSType; fCreator: OSType; const (*var*) filename: Str255; var entry: ICMapEntry ): OSStatus; external name '_ICMapEntriesTypeCreator';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r1] [c1] [b3] 
   * Takes the type and creator (and optionally the name) of an outgoing
   * file and returns the most appropriate mappings database entry.
   * entries must be a handle to a valid IC mappings database preference.
   * The filename may be either the name of the outgoing file or
   * the empty string.
   * Returns icPrefNotFoundErr if no suitable entry found.
   }
{ ----- Low Level Routines -----  }
{
 *  ICCountMapEntries()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICCountMapEntries( inst: ICInstance; entries: Handle; var count: SIGNEDLONG ): OSStatus; external name '_ICCountMapEntries';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r1] [c1] [b3] 
   * Counts the number of entries in the mappings database.
   * entries must be a handle to a valid IC mappings database preference.
   * count is set to the number of entries.
   }
{
 *  ICGetIndMapEntry()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetIndMapEntry( inst: ICInstance; entries: Handle; index: SIGNEDLONG; var pos: SIGNEDLONG; var entry: ICMapEntry ): OSStatus; external name '_ICGetIndMapEntry';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r1] [c1] [b3] 
   * Gets the index'th entry in the mappings database.
   * entries must be a handle to a valid IC mappings database preference.
   * index must be in the range from 1 to the number of entries in the database.
   * The value of pos is ignored on input. pos is set to the position of
   * the index'th entry in the database and is suitable for passing back
   * into ICSetMapEntry.
   * Does not return any user data associated with the entry.
   }
{
 *  ICGetMapEntry()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetMapEntry( inst: ICInstance; entries: Handle; pos: SIGNEDLONG; var entry: ICMapEntry ): OSStatus; external name '_ICGetMapEntry';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r1] [c1] [b3] 
   * Returns the entry located at position pos in the mappings database.
   * entries must be a handle to a valid IC mappings database preference.
   * pos should be 0 to get the first entry. To get the subsequent entries, add
   * entry.total_size to pos and iterate.
   * Does not return any user data associated with the entry.
   }
{
 *  ICSetMapEntry()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICSetMapEntry( inst: ICInstance; entries: Handle; pos: SIGNEDLONG; const (*var*) entry: ICMapEntry ): OSStatus; external name '_ICSetMapEntry';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r1] [c1] [b3] 
   * Sets the entry located at position pos in the mappings database.
   * entries must be a handle to a valid IC mappings database preference.
   * pos should be either a value returned from ICGetIndMapEntry or a value
   * calculated using ICGetMapEntry.
   * entry is a var parameter purely for stack space reasons. It is not
   * modified in any way.
   * Any user data associated with the entry is unmodified.
   }
{
 *  ICDeleteMapEntry()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICDeleteMapEntry( inst: ICInstance; entries: Handle; pos: SIGNEDLONG ): OSStatus; external name '_ICDeleteMapEntry';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r1] [c1] [b3] 
   * Deletes the mappings database entry at pos.
   * entries must be a handle to a valid IC mappings database preference.
   * pos should be either a value returned from ICGetIndMapEntry or a value
   * calculated using ICGetMapEntry.
   * Also deletes any user data associated with the entry.
   }
{
 *  ICAddMapEntry()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICAddMapEntry( inst: ICInstance; entries: Handle; const (*var*) entry: ICMapEntry ): OSStatus; external name '_ICAddMapEntry';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r1] [c1] [b3] 
   * Adds an entry to the mappings database.
   * entries must be a handle to a valid IC mappings database preference.
   * The entry is added to the end of the entries database.
   * No user data is added.
   }
{ ***** Profile Management Routines *****  }
{
 *  ICGetCurrentProfile()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetCurrentProfile( inst: ICInstance; var currentID: ICProfileID ): OSStatus; external name '_ICGetCurrentProfile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r3] [c1] [b3] 
   * Returns the profile ID of the current profile.
   }
{
 *  ICSetCurrentProfile()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICSetCurrentProfile( inst: ICInstance; newID: ICProfileID ): OSStatus; external name '_ICSetCurrentProfile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r3] [c1] [b3] 
   * Sets the current profile to the profile specified in newProfile.
   }
{
 *  ICCountProfiles()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICCountProfiles( inst: ICInstance; var count: SIGNEDLONG ): OSStatus; external name '_ICCountProfiles';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r3] [c1] [b1] 
   * Returns the total number of profiles.
   }
{
 *  ICGetIndProfile()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetIndProfile( inst: ICInstance; index: SIGNEDLONG; var thisID: ICProfileID ): OSStatus; external name '_ICGetIndProfile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r3] [c1] [b1] 
   * Returns the profile ID of the index'th profile.  index must be positive.
   * Returns icProfileNotFoundErr if index is greater than the total number
   * of profiles.
   }
{
 *  ICGetProfileName()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICGetProfileName( inst: ICInstance; thisID: ICProfileID; var name: Str255 ): OSStatus; external name '_ICGetProfileName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r3] [c1] [b3] 
   * Returns the name of a profile given its ID.  The name may not uniquely
   * identify the profile.  [That's what the profile ID is for!]  The name
   * is assumed to be in the system script.
   }
{
 *  ICSetProfileName()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICSetProfileName( inst: ICInstance; thisID: ICProfileID; const (*var*) name: Str255 ): OSStatus; external name '_ICSetProfileName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r3] [c1] [b3] 
   * This routine sets the name of the specified profile.  Profile names
   * need not be unique.  The name should be in the system script.
   }
{
 *  ICAddProfile()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICAddProfile( inst: ICInstance; prototypeID: ICProfileID; var newID: ICProfileID ): OSStatus; external name '_ICAddProfile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r3] [c1] [b2] 
   * If prototypeID = kICNilProfileID, this routine
   * creates a default profile, otherwise it creates a
   * profile by cloning the prototype profile.  The ID
   * of the new profile is returned in newID.
   * The new profile will be give a new, unique, name.
   * This does not switch to the new profile.
   }
{
 *  ICDeleteProfile()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   in InternetConfig 2.5 and later
 }
function ICDeleteProfile( inst: ICInstance; thisID: ICProfileID ): OSStatus; external name '_ICDeleteProfile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ [r3] [c1] [b2] 
   * This routine deletes the profile specified by
   * thisID.  Attempting to delete the current profile
   * or the last profile will return error.
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
