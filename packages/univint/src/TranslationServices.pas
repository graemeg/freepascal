{
     File:       HIServices/TranslationServices.h
 
     Contains:   Translation Services Interfaces.
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
 
     Copyright:  © 2003-2008 by Apple Computer, Inc., all rights reserved.
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}
{  Pascal Translation:  Gale R Paeper, <gpaeper@empirenet.com>, 2006 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                     http://www.freepascal.org/bugs.html
 
}
{	 Pascal Translation:  Gale R Paeper, <gpaeper@empirenet.com>, 2006 }
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
     Version:    HIServices-169~377
=======
     Version:    HIServices-308~1
>>>>>>> origin/fixes_2.4
 
     Copyright:  © 2003-2008 by Apple Computer, Inc., all rights reserved.
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
                     http://www.freepascal.org/bugs.html
 
}
{	 Pascal Translation:  Gale R Paeper, <gpaeper@empirenet.com>, 2006 }
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

unit TranslationServices;
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
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := FALSE}
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
uses MacTypes, CFArray, CFBase, CFData, CFDictionary, CFURL, Files;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{$ALIGN POWER}


{
 *  TranslationServices
 *  
 *  Discussion:
 *    Translation Services provides tools for conversion of data and
 *    file contents from one format to another. All information for
 *    performing a translation is contained within a TranslationRef.
 *    These include source and destination data types, flags indicating
 *    what type of translation can be performed and references to the
 *    system services which execute the translation. TranslationRefs
 *    are CFTypes which must be released after they are created. Source
 *    and destination formats as well as translation flags can be
 *    queried from the TranslationRef. TranslationRefs are generated
 *    either by requesting a specific translation via TranslationCreate
 *    or during discovery of all possible translations via
 *    TranslationCreateWithSourceArray. It is possible to request
 *    TranslationRefs which perform either data or file conversions or
 *    both by using TranslationFlags. When requesting a translation be
 *    executed via TranslationPerformForData, TranslationPerformForFile
 *    or TranslationPerformForURL it is important for the source and
 *    destination data formats to match those found in the
 *    TranslationRef. 
 *    
 *    It is possible to extend the set of system services which provide
 *    translations by creating what is called a filter service. Filter
 *    services are applications similar to those which provide the
 *    services available in the application menu but with a few
 *    modifications in the application plist. Filter services provide
 *    an "NSFilter" entry instead of "NSMessage". Filter Services must
 *    also provide an array of both "NSSendTypes" and "NSReturnTypes"
 *    containing Uniform Type Identifiers indicating from which formats
 *    a filter services translates to what format. Each filter service
 *    may translate multiple send types into a single return type.
 *    Finally, a filter service must indicate what type of translations
 *    it supports via the "NSSupportsDataTranslation" and
 *    "NSSupportsFileTranslation" entries. In the end, a filter
 *    service's plist may look like the following, 
 *    
 *    
 *    
 *            <key>NSServices</key> 
 *                <array> 
 *                    <dict> 
 *                        <key>NSFilter</key> 
 *                        <string>ExampleTranslation</string> 
 *                        <key>NSReturnTypes</key> 
 *                        <array> 
 *                            <string>com.example.returntype</string> 
 *                        </array> 
 *                        <key>NSSendTypes</key> 
 *                        <array> 
 *                            <string>com.example.sourcetype1</string> 
 *                            <string>com.example.sourcetype2</string> 
 *                        </array> 
 *                        <key>NSSupportsDataTranslation</key> 
 *                        <string></string> 
 *                        <key>NSSupportsFileTranslation</key> 
 *                        <string></string> 
 *                    </dict> 
            <array> 

 *    
 *    
 *    
 *    All filter services must handle the kEventServicePerform Carbon
 *    Event.  The filter service will be automatically launched when
 *    necessary and it will receive the kEventServicePerform event with
 *    the message indicated by the NSFilter tag in the plist as well as
 *    a Pasteboard Manager pasteboard containing flavors indicating
 *    what type of translation must be performed. If a filter service
 *    only supports data translations a flavor on the pasteboard will
 *    correspond to one of the type identifiers listed in your plist's
 *    send types.  Upon translation of the data, the filter service
 *    must clear the pasteboard, add the return identifier and
 *    translated data to the pasteboard, and return from the event. For
 *    a filter service which provides file translations,
 *    "public.file-url" and "com.apple.file-contents-type" will be
 *    available on the pasteboard indicating the file location and
 *    contents format from which to translate. Upon translation, the
 *    filter service should place a "public.file-url" flavor on the
 *    pasteboard indicating where the translated file has been placed,
 *    typically next to the orignal named "<filename> (converted)"
 *    (ala. Finder's "<filename> copy" behavior for duplicated files).
 }
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	TranslationRef = ^OpaqueTranslationRef; { an opaque type }
	OpaqueTranslationRef = record end;
=======
	TranslationRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	TranslationRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	TranslationRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	TranslationRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
{ Translation Services error codes}
const
{invalidTranslationPathErr   = -3025}
                                        {couldNotParseSourceFileErr  = -3026}
                                        {noTranslationPathErr         = -3030          // no translation for source and destination provided}
                                        {badTranslationSpecErr      = -3031}
                                        {noPrefAppErr              = -3032}
=======
=======
>>>>>>> origin/fixes_2_2
	TranslationRef = ^SInt32; { an opaque 32-bit type }
{ Translation Services error codes}
const
                                  {invalidTranslationPathErr   = -3025}
                                  {couldNotParseSourceFileErr  = -3026}
                                  {noTranslationPathErr         = -3030          // no translation for source and destination provided}
                                  {badTranslationSpecErr      = -3031}
                                  {noPrefAppErr              = -3032}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	TranslationRef = ^SInt32; { an opaque type }
{ Translation Services error codes}
const
{invalidTranslationPathErr   = -3025}
                                        {couldNotParseSourceFileErr  = -3026}
                                        {noTranslationPathErr         = -3030          // no translation for source and destination provided}
                                        {badTranslationSpecErr      = -3031}
                                        {noPrefAppErr              = -3032}
>>>>>>> origin/fixes_2.4
	badTranslationRefErr = -3031; { TranslationRef does not perform translation requested}


{
 *  TranslationFlags
 *  
 *  Summary:
 *    The following constants are used by the translation creation
 *    routines to indicate which types of translations are requested.
 *    The flags are cumulative (ie. when passing both
 *    kTranslationDataTranslation and kTranslationFileTranslation the
 *    client is requesting only those translations which support both
 *    data AND file translations).
 }
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	TranslationFlags = OptionBits;
=======
	TranslationFlags = UInt32;
>>>>>>> graemeg/fixes_2_2
=======
	TranslationFlags = UInt32;
>>>>>>> origin/fixes_2_2
=======
	TranslationFlags = OptionBits;
>>>>>>> origin/fixes_2.4
const
{
   * Indicates that the client is interested in translations which
   * provide data translations.
   }
	kTranslationDataTranslation = 1 shl 0;

  {
   * Indicates that the client is interested in translations which
   * provide file translations.
   }
	kTranslationFileTranslation = 1 shl 1;

{
 *  TranslationGetTypeID()
 *  
 *  Summary:
 *    Returns the CFType identifier for a translation object.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Result:
 *    A CFTypeID unique to translation instances.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function TranslationGetTypeID: CFTypeID; external name '_TranslationGetTypeID';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  TranslationCreate()
 *  
 *  Summary:
 *    Creates a translation reference describing a system service
 *    providing translations of data from the source type to the
 *    destination type.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inSourceType:
 *      A Uniform Type Identifier specifying the format of source data
 *      to be translated.
 *    
 *    inDestinationType:
 *      A Uniform Type Identifier specifying the destination format to
 *      which the source data should be translated.
 *    
 *    inTranslationFlags:
 *      A set of TranslationFlags indicating what type of translation
 *      is requested.
 *    
 *    outTranslation:
 *      A TranslationRef reference which receives the requested
 *      translation if a system service providing the translation
 *      exists.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function TranslationCreate( inSourceType: CFStringRef; inDestinationType: CFStringRef; inTranslationFlags: TranslationFlags; var outTranslation: TranslationRef ): OSStatus; external name '_TranslationCreate';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  TranslationCreateWithSourceArray()
 *  
 *  Summary:
 *    Creates a list of destination flavors translation references
 *    describing various system services providing translations of data
 *    from the source types to the destination types.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inSourceTypes:
 *      An array of Uniform Type Identifiers specifying the formats of
 *      source data to be translated.
 *    
 *    inTranslationFlags:
 *      A set of TranslationFlags indicating what type of translations
 *      are requested.
 *    
 *    outDestinationTypes:
 *      A CFArrayRef reference which receives an array of Uniform Type
 *      Identifiers specifying what destination formats are available
 *      as translations of the provided source formats. Any destination
 *      formats already represented as a format in the source array are
 *      excluded from the returned list. The search for destination
 *      formats is performed in the order of source formats. This array
 *      must be released by the client.
 *    
 *    outTranslations:
 *      A CFDictionaryRef reference which receives a dictionary of
 *      TranslationRefs representing all translations provided by
 *      system services. The dictionary is keyed by destination flavor.
 *      Any translations with destination formats already represented
 *      as a format in the source array are excluded from the returned
 *      dictionary. This dictionary must be released by the client.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function TranslationCreateWithSourceArray( inSourceTypes: CFArrayRef; inTranslationFlags: TranslationFlags; var outDestinationTypes: CFArrayRef; var outTranslations: CFDictionaryRef ): OSStatus; external name '_TranslationCreateWithSourceArray';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  TranslationPerformForData()
 *  
 *  Summary:
 *    Executes the translation of source data into destination data. 
 *    The formats of the source and destination data are contained
 *    within the TranslationRef.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inTranslation:
 *      A TranslationRef containing information on the source and
 *      destination data formats and how to execute a translation from
 *      one to the other. The formats of the source and destination
 *      data must correspond to those indicated by the TranslationRef.
 *    
 *    inSourceData:
 *      A CFDataRef containing data to be translated.
 *    
 *    outDestinationData:
 *      A CFDataRef reference which receives the translated data.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function TranslationPerformForData( inTranslation: TranslationRef; inSourceData: CFDataRef; var outDestinationData: CFDataRef ): OSStatus; external name '_TranslationPerformForData';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  TranslationPerformForFile()
 *  
 *  Summary:
 *    Executes the translation of source file contents to a destination
 *    file content format. The formats of the source and destination
 *    file contents are held within the TranslationRef.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inTranslation:
 *      A TranslationRef containing information on the source and
 *      destination file content formats and how to execute a
 *      translation from one to the other. The formats of the source
 *      and destination file contents must correspond to those
 *      indicated by the TranslationRef.
 *    
 *    inSourceFile:
 *      A FSRef reference pointing to a file whose contents are to be
 *      translated.
 *    
 *    inDestinationDirectory:
 *      An optional FSRef reference pointing to the desired directory
 *      for the translation. By default the destination directory is
 *      the same as the source file.
 *    
 *    inDestinationName:
 *      An optional CFStringRef indicating the desired name for the
 *      translated file. By default the translated file's name will be
 *      "<filename> (converted)" (ala. Finder's "<filename> copy"
 *      behavior for duplicated files).
 *    
 *    outTranslatedFile:
 *      A FSRef reference which receives a new file with the translated
 *      contents. It is possible for the translated file to not have
 *      been created in the directory or with the name requested by the
 *      client due to disk space or translator limitations. It is
 *      important to rely only on the file reference returned in this
 *      parameter.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function TranslationPerformForFile( inTranslation: TranslationRef; const (*var*) inSourceFile: FSRef; {const} inDestinationDirectory: FSRefPtr { can be NULL }; inDestinationName: CFStringRef { can be NULL }; var outTranslatedFile: FSRef ): OSStatus; external name '_TranslationPerformForFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  TranslationPerformForURL()
 *  
 *  Summary:
 *    Executes the translation of source data pointed to a URL to a
 *    destination format. The formats of the source and destination URL
 *    contents are held within the TranslationRef. Currently, only file
 *    URLs are accepted for URL translations.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inTranslation:
 *      A TranslationRef containing information on the source and
 *      destination URL content formats and how to execute a
 *      translation from one to the other. The formats of the source
 *      and destination URL contents must correspond to those indicated
 *      by the TranslationRef.
 *    
 *    inSourceURL:
 *      A CFURLRef pointing to source data whose contents are to be
 *      translated.  Currently, only file URLs are accepted for URL
 *      translations.
 *    
 *    inDestinationURL:
 *      An optional CFURLRef indicating the desired location for the
 *      translated data. File URLs may either indicate the desired
 *      destination directory or directory and name for the translated
 *      file. By default for file URLs, the translated file's name will
 *      be "<filename> (converted)" (ala. Finder's "<filename> copy"
 *      behavior for duplicated files).
 *    
 *    outTranslatedURL:
 *      A FSRef reference which receives a new file with the translated
 *      contents. For file URLs, it is possible for the translated file
 *      to not have been created in the directory or with the name
 *      requested by the client due to disk space or translator
 *      limitations. It is important to rely only on the URL returned
 *      in this parameter.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function TranslationPerformForURL( inTranslation: TranslationRef; inSourceURL: CFURLRef; inDestinationURL: CFURLRef { can be NULL }; var outTranslatedURL: CFURLRef ): OSStatus; external name '_TranslationPerformForURL';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  TranslationCopySourceType()
 *  
 *  Summary:
 *    Accesses a translation's source type.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inTranslation:
 *      A TranslationRef containing the requested source type.
 *    
 *    outSourceType:
 *      A CFStringRef which receives the TranslationRef's source type.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function TranslationCopySourceType( inTranslation: TranslationRef; var outSourceType: CFStringRef ): OSStatus; external name '_TranslationCopySourceType';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  TranslationCopyDestinationType()
 *  
 *  Summary:
 *    Accesses a translation's destination type.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inTranslation:
 *      A TranslationRef containing the requested destination type.
 *    
 *    outDestinationType:
 *      A CFStringRef which receives the TranslationRef's destination
 *      type.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function TranslationCopyDestinationType( inTranslation: TranslationRef; var outDestinationType: CFStringRef ): OSStatus; external name '_TranslationCopyDestinationType';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  TranslationGetTranslationFlags()
 *  
 *  Summary:
 *    Accesses a translation's flags.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inTranslation:
 *      A TranslationRef containing the requested flags.
 *    
 *    outTranslationFlags:
 *      A TranslationFlags which receives the TranslationRef's flags.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function TranslationGetTranslationFlags( inTranslation: TranslationRef; var outTranslationFlags: TranslationFlags ): OSStatus; external name '_TranslationGetTranslationFlags';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {TARGET_OS_MAC}
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
{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4
