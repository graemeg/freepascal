{
     File:       FileTypesAndCreators.p
 
     Contains:   Symbolic constants for FileTypes and signatures of popular documents.
 
     Version:    Technology: Macintosh Easy Open 1.1
                 Release:    Universal Interfaces 3.4.2
 
     Copyright:  © 1992-2002 by Apple Computer, Inc., all rights reserved.
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
=======
                     http://www.freepascal.org/bugs.html
>>>>>>> graemeg/fixes_2_2
=======
                     http://www.freepascal.org/bugs.html
>>>>>>> origin/fixes_2_2
 
}


{
    Modified for use with Free Pascal
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Version 308
    Please report any bugs to <gpc@microbizz.nl>
}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
    Version 210
    Please report any bugs to <gpc@microbizz.nl>
}

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
    Version 308
    Please report any bugs to <gpc@microbizz.nl>
}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4
{$mode macpas}
{$packenum 1}
{$macro on}
{$inline on}
{$calling mwpascal}

unit FileTypesAndCreators;
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
{$ifc defined(iphonesim)}
 	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := TRUE}
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
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
=======
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
>>>>>>> graemeg/cpstrnew
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_CPU_ARM64 := TRUE}
=======
	{$setc TARGET_CPU_ARM := TRUE}
>>>>>>> graemeg/cpstrnew
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
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
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
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
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
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
uses MacTypes,ConditionalMacros;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not MACOSALLINCLUDE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4

=======
>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{$ifc TARGET_OS_MAC}

{$ifc TARGET_OS_MAC}

{$ifc TARGET_OS_MAC}
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2

{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}

{

Motivation for FileTypesAndCreators.h

    The TranslationMgr (Macintosh Easy Open) uses “FileTypes” to express and manage
    translation capabilities.  This works well for most Macintosh applications.  It
    simply uses the file type stored in the catalog info by HFS.  But, the TranslationMgr
    needs to work with non-Macintosh files.  These do not have file types (or creators).
    Another problem is some Macintosh applications keep the same file type for a file
    after changing the file format from one version to the next.  Thus, the HFS file type
    cannot be used to distinguish between formats.

    This file is an effort to define FileTypes and creator codes for non-Macintosh file
    formats and define FileTypes that distinguish between formats.


There are two main categories of constants defined in FileTypesAndCreators.h.  One is
creator signatures of applications the other is file types.  To make this easier to use,
some style guidelines are provided.

Creators:

    The symbolic constant for all creator codes begin with “sig”. Following “sig” is the
    application name.  Example:

        sigSurfWriter = 'SURF'

    If the application name is localized for different markets, the name of the U.S.
    version is always used. It is optional to make symbolic constants for localized names.
    If a U.S. is not available, another Roman name is used (this is for ease of editing
    in MPW.)  Example:

        sigSurfWriter = 'SURF',
        sigSkiWriter =  'SURF'  // optional definition of Canadian name of app

    If multiple versions of an application exist, and the creator codes are different,
    then the version number is added to the end of the symbolic name, for the first version
    in which the signature changed.  Example:

        sigDirtWriter =     'DIRT', // version 1 and 2 used same signature
        sigDirtWriter3 =    'DRT3'

    If a developer has the same application on another platform as is on the Mac, then the
    creator for a file from the other platform should be the same as the creator of the
    Macintosh application.   Example:

        sigMicrosoftWord =  'MSWD'  // used for Mac and Windows documents

    If there is no similar Macintosh product, then the signature of the platform is used.
    Example:

        sigPCDOS =          'mdos', // for DOS and Windows
        sigAppleProDos =    'pdos'


FileTypes:

    The symbolic constant for all file types begin with “ft”. Following “ft” is the ideal
    kind for that document.  Example:

        ftSurfWriterDocument =  'SRFD'

    If the application name is localized for different markets, the kind string of the U.S.
    version is always used. It is optional to make symbolic constants for localized kind
    strings.  If a U.S. is not available, another Roman kind string is used (this is for
    ease of editing in MPW.)  Example:

        ftSurfWriterDocument =  'SRFD,
        ftSkiWriterDocument =   'SRFD'  // optional Canadian name

    If multiple versions of an application exist, and the document for files created by each
    are different (meaning one version of the app can not read the documents of a newer
    version), then the version number is added to the end of the symbolic name, for the
    first version in which the format changed.  Example:

       // version 1 and 2 docs have same format, version 3 has a different format
        ftDirtWriterDocument =          'DDRT',
        ftDirtWriter3Document =         '3DRT'

    If multiple versions of an application exist, and the document for files created by each
    are different, but the catInfo file type has always been the same, then constants are
    made for each unique format and one is made for the catInfo type actually used.

        ftWordDocument =        'WDBN',
        ftWord3Document =       'WDB3',
        ftWord4Document =       'WDB4',
        ftWord5Document =       'WDB5'

    If a developer has the same application on another platform as is on the Mac and the file
    formats are the same (binary interchangeable), then the file type for a file from the other
    platform should be the same as the file type of the Macintosh document.   Example:

        ftPageMakerPublication = 'ALB4' // used for Mac and PC documents

    If there is no similar Macintosh document format, then a file type is made up.  If the
    program is used on two different non-Mac platforms and their file formats are not the
    same, the platform name is added to the name.  Typically after the program name and before
    document.  Example:

        ftXyWriteIIIDocument =  'XyWr'  // XyWriteIII only has a PC format
        ftSurfWindowsDocument = 'SRFW'  // Surf Windows file format
        ftSurfPCDocument =      'SRFP'  // Surf PC file format
}

{************* Macintosh applications and FileTypes *************}


const
																{  Mac word processors  }
	sigWord						= FourCharCode('MSWD');
	ftWord3Document				= FourCharCode('MSW3');
	ftWord4Document				= FourCharCode('MSW4');
	ftWord5Document				= FourCharCode('MSW5');
	ftWordDocument				= FourCharCode('WDBN');
	ftWordDocumentPC			= FourCharCode('MWPC');						{  not registered  }
	ftWord1DocumentWindows		= FourCharCode('WW1 ');						{  not registered  }
	ftWord2DocumentWindows		= FourCharCode('WW2 ');						{  not registered  }
	ftRTFDocument				= FourCharCode('RTF ');						{  not registered  }
	sigWordPerfect				= FourCharCode('SSIW');
	ftWordPerfectDocument		= FourCharCode('WPD0');
	sigWordPerfect2				= FourCharCode('WPC2');
	ftWordPerfect2Document		= FourCharCode('WPD1');
	ftWordPerfect21Document		= FourCharCode('WPD2');
	ftWordPerfect42DocumentPC	= FourCharCode('.WP4');						{  not registered  }
	ftWordPerfect50DocumentPC	= FourCharCode('.WP5');						{  not registered  }
	ftWordPerfect51DocumentPC	= FourCharCode('WP51');						{  not registered  }
	ftWordPerfectGraphicsPC		= FourCharCode('WPGf');						{  not registered  }
	sigMacWriteII				= FourCharCode('MWII');
	ftMacWriteIIDocument		= FourCharCode('MW2D');
	sigWriteNow					= FourCharCode('nX^n');
	ftWriteNow2Document			= FourCharCode('nX^d');
	ftWriteNow3Document			= FourCharCode('nX^2');
	sigMacWrite					= FourCharCode('MACA');
	ftMacWrite5Document			= FourCharCode('WORD');
	sigFrameMaker				= FourCharCode('Fram');
	ftFrameMakerDocument		= FourCharCode('FASL');
	ftFrameMakerMIFDocument		= FourCharCode('MIF ');
	ftFrameMakerMIF2Document	= FourCharCode('MIF2');
	ftFrameMakerMIF3Document	= FourCharCode('MIF3');
	sigMSWrite					= FourCharCode('MSWT');
	sigActa						= FourCharCode('ACTA');
	sigTHINKPascal				= FourCharCode('PJMM');
	sigTHINKC					= FourCharCode('KAHL');
	sigFullWrite				= FourCharCode('FWRT');
	sigTeachText				= FourCharCode('ttxt');
	ftTeachTextDocument			= FourCharCode('ttro');
	sigSimpleText				= FourCharCode('ttxt');
	ftSimpleTextDocument		= FourCharCode('ttro');
	sigMPWShell					= FourCharCode('MPS ');
	sigQuarkXPress				= FourCharCode('XPR3');
	sigNisus					= FourCharCode('NISI');
	sigOmniPage					= FourCharCode('PRTC');
	sigPersonalPress			= FourCharCode('SCPG');
	sigPublishItEZ				= FourCharCode('2CTY');
	sigReadySetGo				= FourCharCode('MEMR');
	sigRagTime					= FourCharCode('R#+A');
	sigLetraStudio				= FourCharCode('LSTP');
	sigLetterPerfect			= FourCharCode('WPCI');
	sigTheWritingCenter			= $0A1A5750;					{  this 'unprintable unprintable WP' One of the unprintables is a line feed.   }
	sigInstantUpdate			= FourCharCode('IUA0');

																{  databases  }
	sig4thDimension				= FourCharCode('4D03');
	ft4thDimensionDB			= FourCharCode('BAS3');
	sigFileMakerPro				= FourCharCode('FMPR');
	ftFileMakerProDatabase		= FourCharCode('FMPR');
	sigHyperCard				= FourCharCode('WILD');
	ftHyperCard					= FourCharCode('STAK');
	sigSmartFormAsst			= FourCharCode('KCFM');
	ftSmartFormAsst				= FourCharCode('STCK');
	sigSmartFormDesign			= FourCharCode('KCFD');
	ftSmartFormDesign			= FourCharCode('CFRM');
	sigFileForce				= FourCharCode('4D93');
	ftFileForceDatabase			= FourCharCode('FIL3');
	sigFileMaker2				= FourCharCode('FMK4');
	ftFileMaker2Database		= FourCharCode('FMK$');
	sigSuperCard				= FourCharCode('RUNT');
	sigDoubleHelix				= FourCharCode('HELX');
	sigGeoQuery					= FourCharCode('RGgq');
	sigFoxBASE					= FourCharCode('FOX+');
	sigINSPIRATION				= FourCharCode('CER3');
	sigPanorama					= FourCharCode('KAS1');
	sigSilverrunLDM				= FourCharCode('CDML');
	sigSilverrunDFD				= FourCharCode('CDDF');						{  finance  }
	sigQuicken					= FourCharCode('INTU');
	sigMacInTax91				= FourCharCode('MIT1');
	ftMacInTax91				= FourCharCode('MITF');
	sigAccountantInc			= FourCharCode('APRO');
	sigAtOnce					= FourCharCode('KISS');
	sigCAT3						= FourCharCode('tCat');
	sigDollarsNSense			= FourCharCode('EAGP');
	sigInsightExpert			= FourCharCode('LSGL');
	sigMYOB						= FourCharCode('MYOB');
	sigMacMoney					= FourCharCode('SSLA');
	sigManagingYourMoney		= FourCharCode('MYMC');
	sigPlainsAndSimple			= FourCharCode('PEGG');						{  scheduling  }
	sigMacProject2				= FourCharCode('MPRX');
	ftMacProject				= FourCharCode('MPRD');
	sigMSProject				= FourCharCode('MSPJ');
	sigMacProjectPro			= FourCharCode('MPRP');						{  utilities  }
	sigStuffIt					= FourCharCode('SIT!');
	ftStuffItArchive			= FourCharCode('SIT!');
	sigCompactPro				= FourCharCode('CPCT');
	ftCompactProArchive			= FourCharCode('PACT');
	sigFontographer				= FourCharCode('aCa2');
	sigMetamorphosis			= FourCharCode('MEtP');
	sigCorrectGrammar			= FourCharCode('LsCG');
	sigDynodex					= FourCharCode('DYNO');
	sigMariah					= FourCharCode('MarH');
	sigAddressBook				= FourCharCode('AdBk');
	sigThePrintShop				= FourCharCode('PSHP');
	sigQuicKeys2				= FourCharCode('Qky2');
	sigReadStar2Plus			= FourCharCode('INOV');
	sigSoftPC					= FourCharCode('PCXT');
	sigMacMenlo					= FourCharCode('MNLO');
	sigDisinfectant				= FourCharCode('D2CT');						{  communications  }
	sigSmartcom2				= FourCharCode('SCOM');
	sigVersaTermPRO				= FourCharCode('VPRO');
	sigVersaTerm				= FourCharCode('VATM');
	sigWhiteKnight				= FourCharCode('WK11');
	sigNCSATelnet				= FourCharCode('NCSA');
	sigDynaComm					= FourCharCode('PAR2');
	sigQMForms					= FourCharCode('MLTM');						{  math and statistics  }
	sigMathematica				= FourCharCode('OMEG');
	sigMathCAD					= FourCharCode('MCAD');
	sigStatView2				= FourCharCode('STAT');
	sigDataDesk					= FourCharCode('DDSK');
	sigPowerMath2				= FourCharCode('MATH');
	sigSuperANOVA				= FourCharCode('SupA');
	sigSystat					= FourCharCode('SYT1');
	sigTheorist					= FourCharCode('Theo');

																{  spreadsheets  }
	sigExcel					= FourCharCode('XCEL');
	ftExcel2Spreadsheet			= FourCharCode('XLS ');
	ftExcel2Macro				= FourCharCode('XLM ');
	ftExcel2Chart				= FourCharCode('XLC ');
	ftExcel3Spreadsheet			= FourCharCode('XLS3');
	ftExcel3Macro				= FourCharCode('XLM3');
	ftExcel3Chart				= FourCharCode('XLC3');
	ftExcel4Spreadsheet			= FourCharCode('XLS4');
	ftExcel4Macro				= FourCharCode('XLM4');
	ftSYLKSpreadsheet			= FourCharCode('SYLK');
	sigLotus123					= FourCharCode('L123');
	ft123Spreadsheet			= FourCharCode('LWKS');
	sigWingz					= FourCharCode('WNGZ');
	ftWingzSpreadsheet			= FourCharCode('WZSS');
	ftWingzScript				= FourCharCode('WZSC');
	sigResolve					= FourCharCode('Rslv');
	ftResolve					= FourCharCode('RsWs');
	ftResolveScript				= FourCharCode('RsWc');
	sigFullImpact2				= FourCharCode('Flv2');

																{  graphics  }
	sigIllustrator				= FourCharCode('ART5');
	ftPostScriptMac				= FourCharCode('EPSF');
	sigMacPaint					= FourCharCode('MPNT');
	ftMacPaintGraphic			= FourCharCode('PNTG');
	sigSuperPaint				= FourCharCode('SPNT');
	ftSuperPaintGraphic			= FourCharCode('SPTG');
	sigCanvas					= FourCharCode('DAD2');
	ftCanvasGraphic				= FourCharCode('drw2');
	sigUltraPaint				= FourCharCode('ULTR');
	ftUltraPaint				= FourCharCode('UPNT');
	sigPhotoshop				= FourCharCode('8BIM');
	ftPhotoshopGraphic			= FourCharCode('8BIM');
	sigMacDrawPro				= FourCharCode('dPro');
	ftMacDrawProDrawing			= FourCharCode('dDoc');
	sigPageMaker				= FourCharCode('ALD4');
	ftPageMakerPublication		= FourCharCode('ALB4');
	sigFreeHand					= FourCharCode('FHA3');
	ftFreeHandDrawing			= FourCharCode('FHD3');
	sigClarisCAD				= FourCharCode('CCAD');
	ftClarisCAD					= FourCharCode('CAD2');
	sigMacDrawII				= FourCharCode('MDPL');
	ftMacDrawIIDrawing			= FourCharCode('DRWG');
	sigMacroMindDirector		= FourCharCode('MMDR');
	ftMMDirectorMovie			= FourCharCode('VWMD');
	ftMMDirectorSound			= FourCharCode('MMSD');
	sigOptix					= FourCharCode('PIXL');						{  was previously PixelPerfect  }
	sigPixelPaint				= FourCharCode('PIXR');
	ftPixelPaint				= FourCharCode('PX01');
	sigAldusSuper3D				= FourCharCode('SP3D');
	ftSuper3DDrawing			= FourCharCode('3DBX');
	sigSwivel3D					= FourCharCode('SWVL');
	ftSwivel3DDrawing			= FourCharCode('SMDL');
	sigCricketDraw				= FourCharCode('CRDW');
	ftCricketDrawing			= FourCharCode('CKDT');
	sigCricketGraph				= FourCharCode('CGRF');
	ftCricketChart				= FourCharCode('CGPC');
	sigDesignCAD				= FourCharCode('ASBC');
	ftDesignCADDrawing			= FourCharCode('DCAD');
	sigImageStudio				= FourCharCode('FSPE');
	ftImageStudioGraphic		= FourCharCode('RIFF');
	sigVersaCad					= FourCharCode('VCAD');
	ftVersaCADDrawing			= FourCharCode('2D  ');
	sigAdobePremiere			= FourCharCode('PrMr');						{  was previously misspelled as sigAdobePremier }
	ftAdobePremiereMovie		= FourCharCode('MooV');						{  was previously misspelled as ftAdobePremierMovie }
	sigAfterDark				= FourCharCode('ADrk');
	ftAfterDarkModule			= FourCharCode('ADgm');
	sigClip3D					= FourCharCode('EZ3E');
	ftClip3Dgraphic				= FourCharCode('EZ3D');
	sigKaleidaGraph				= FourCharCode('QKPT');
	ftKaleidaGraphGraphic		= FourCharCode('QPCT');
	sigMacFlow					= FourCharCode('MCFL');
	ftMacFlowChart				= FourCharCode('FLCH');
	sigMoviePlayer				= FourCharCode('TVOD');
	ftMoviePlayerMovie			= FourCharCode('MooV');
	sigMacSpin					= FourCharCode('D2SP');
	ftMacSpinDataSet			= FourCharCode('D2BN');
	sigAutoCAD					= FourCharCode('ACAD');
	sigLabVIEW					= FourCharCode('LBVW');
	sigColorMacCheese			= FourCharCode('CMC∆');
	sigMiniCad					= FourCharCode('CDP3');
	sigDreams					= FourCharCode('PHNX');
	sigOmnis5					= FourCharCode('Q2$$');
	sigPhotoMac					= FourCharCode('PMAC');
	sigGraphMaster				= FourCharCode('GRAM');
	sigInfiniD					= FourCharCode('SI∞D');
	sigOfoto					= FourCharCode('APLS');
	sigMacDraw					= FourCharCode('MDRW');
	sigDeltagraphPro			= FourCharCode('DGRH');
	sigDesign2					= FourCharCode('DESG');
	sigDesignStudio				= FourCharCode('MRJN');
	sigDynaperspective			= FourCharCode('PERS');
	sigGenericCADD				= FourCharCode('CAD3');
	sigMacDraft					= FourCharCode('MD20');
	sigModelShop				= FourCharCode('MDSP');
	sigOasis					= FourCharCode('TAOA');
	sigOBJECTMASTER				= FourCharCode('BROW');
	sigMovieRecorder			= FourCharCode('mrcr');
	sigPictureCompressor		= FourCharCode('ppxi');
	sigPICTViewer				= FourCharCode('MDTS');
	sigSmoothie					= FourCharCode('Smoo');
	sigScreenPlay				= FourCharCode('SPLY');
	sigStudio1					= FourCharCode('ST/1');
	sigStudio32					= FourCharCode('ST32');
	sigStudio8					= FourCharCode('ST/8');
	sigKidPix					= FourCharCode('Kid2');
	sigDigDarkroom				= FourCharCode('DIDR');

																{  presentations  }
	sigMore						= FourCharCode('MOR2');
	ftMore3Document				= FourCharCode('MOR3');
	ftMore2Document				= FourCharCode('MOR2');
	sigPersuasion				= FourCharCode('PLP2');
	ftPersuasion1Presentation	= FourCharCode('PRS1');
	ftPersuasion2Presentation	= FourCharCode('PRS2');
	sigPowerPoint				= FourCharCode('PPNT');
	ftPowerPointPresentation	= FourCharCode('SLDS');
	sigCricketPresents			= FourCharCode('CRPR');
	ftCricketPresentation		= FourCharCode('PRDF');						{  works  }
	sigMSWorks					= FourCharCode('PSI2');
	sigMSWorks3					= FourCharCode('MSWK');
	ftMSWorksWordProcessor		= FourCharCode('AWWP');
	ftMSWorksSpreadsheet		= FourCharCode('AWSS');
	ftMSWorksDataBase			= FourCharCode('AWDB');
	ftMSWorksComm				= FourCharCode('AWDC');
	ftMSWorksMacros				= FourCharCode('AWMC');
	ftMSWorks1WordProcessor		= FourCharCode('AWW1');						{  not registered  }
	ftMSWorks1Spreadsheet		= FourCharCode('AWS1');						{  not registered  }
	ftMSWorks1DataBase			= FourCharCode('AWD1');						{  not registered  }
	ftMSWorks2WordProcessor		= FourCharCode('AWW2');						{  not registered  }
	ftMSWorks2Spreadsheet		= FourCharCode('AWS2');						{  not registered  }
	ftMSWorks2DataBase			= FourCharCode('AWD2');						{  not registered  }
	ftMSWorks3WordProcessor		= FourCharCode('AWW3');						{  not registered  }
	ftMSWorks3Spreadsheet		= FourCharCode('AWS3');						{  not registered  }
	ftMSWorks3DataBase			= FourCharCode('AWD3');						{  not registered  }
	ftMSWorks3Comm				= FourCharCode('AWC3');						{  not registered  }
	ftMSWorks3Macro				= FourCharCode('AWM3');						{  not registered  }
	ftMSWorks3Draw				= FourCharCode('AWR3');						{  not registered  }
	ftMSWorks2WordProcessorPC	= FourCharCode('PWW2');						{  not registered  }
	ftMSWorks2DatabasePC		= FourCharCode('PWDB');						{  not registered  }
	sigGreatWorks				= FourCharCode('ZEBR');
	ftGreatWorksWordProcessor	= FourCharCode('ZWRT');
	ftGreatWorksSpreadsheet		= FourCharCode('ZCAL');
	ftGreatWorksPaint			= FourCharCode('ZPNT');
	sigClarisWorks				= FourCharCode('BOBO');
	ftClarisWorksWordProcessor	= FourCharCode('CWWP');
	ftClarisWorksSpreadsheet	= FourCharCode('CWSS');
	ftClarisWorksGraphics		= FourCharCode('CWGR');
	sigBeagleWorks				= FourCharCode('BWks');
	ftBeagleWorksWordProcessor	= FourCharCode('BWwp');
	ftBeagleWorksDatabase		= FourCharCode('BWdb');
	ftBeagleWorksSpreadsheet	= FourCharCode('BWss');
	ftBeagleWorksComm			= FourCharCode('BWcm');
	ftBeagleWorksDrawing		= FourCharCode('BWdr');
	ftBeagleWorksGraphic		= FourCharCode('BWpt');
	ftPICTFile					= FourCharCode('PICT');

																{  entertainment  }
	sigPGATourGolf				= FourCharCode('gOLF');
	sigSimCity					= FourCharCode('MCRP');
	sigHellCats					= FourCharCode('HELL');						{  education  }
	sigReaderRabbit3			= FourCharCode('RDR3');						{  Translation applications  }
	sigDataVizDesktop			= FourCharCode('DVDT');
	sigSotwareBridge			= FourCharCode('mdos');
	sigWordForWord				= FourCharCode('MSTR');
	sigAppleFileExchange		= FourCharCode('PSPT');						{  Apple software  }
	sigAppleLink				= FourCharCode('GEOL');
	ftAppleLinkAddressBook		= FourCharCode('ADRS');
	ftAppleLinkImageFile		= FourCharCode('SIMA');
	ftAppleLinkPackage			= FourCharCode('HBSF');
	ftAppleLinkConnFile			= FourCharCode('PETE');
	ftAppleLinkHelp				= FourCharCode('HLPF');
	sigInstaller				= FourCharCode('bjbc');
	ftInstallerScript			= FourCharCode('bjbc');
	sigDiskCopy					= FourCharCode('dCpy');
	ftDiskCopyImage				= FourCharCode('dImg');
	sigResEdit					= FourCharCode('RSED');
	ftResEditResourceFile		= FourCharCode('rsrc');
	sigAardvark					= FourCharCode('AARD');
	sigCompatibilityChkr		= FourCharCode('wkrp');
	sigMacTerminal				= FourCharCode('Term');
	sigSADE						= FourCharCode('sade');
	sigCurare					= FourCharCode('Cura');
	sigPCXChange				= FourCharCode('dosa');
	sigAtEase					= FourCharCode('mfdr');
	sigStockItToMe				= FourCharCode('SITM');
	sigAppleSearch				= FourCharCode('asis');
	sigAppleSearchToo			= FourCharCode('hobs');						{  the following are files types for system files  }
	ftScriptSystemResourceCollection = FourCharCode('ifil');
	ftSoundFile					= FourCharCode('sfil');
	ftFontFile					= FourCharCode('ffil');
	ftTrueTypeFontFile			= FourCharCode('tfil');
	ftKeyboardLayout			= FourCharCode('kfil');
	ftFontSuitcase				= FourCharCode('FFIL');
	ftDASuitcase				= FourCharCode('DFIL');
	ftSystemExtension			= FourCharCode('INIT');
	ftDAMQueryDocument			= FourCharCode('qery');


	{	************* Special FileTypes and creators *************	}
	ftApplicationName			= FourCharCode('apnm');						{  this is the type used to define the application name in a kind resource  }
	sigIndustryStandard			= FourCharCode('istd');						{  this is the creator used to define a kind string in a kind resource for a FileType that has many creators   }
	ftXTND13TextImport			= FourCharCode('xt13');						{  this is a pseduo-format used by "XTND for Apps". The taDstIsAppTranslation bit is set  }


	{	************* Apple][ applications and FileTypes *************	}
	sigAppleProDOS				= FourCharCode('pdos');						{  not registered  }
	ftAppleWorksWordProcessor	= FourCharCode('1A  ');						{  not registered  }
	ftAppleWorks1WordProcessor	= FourCharCode('1A1 ');						{  not registered  }
	ftAppleWorks2WordProcessor	= FourCharCode('1A2 ');						{  not registered  }
	ftAppleWorks3WordProcessor	= FourCharCode('1A3 ');						{  not registered  }
	ftAppleWorksDataBase		= FourCharCode('19  ');						{  not registered  }
	ftAppleWorks1DataBase		= FourCharCode('191 ');						{  not registered  }
	ftAppleWorks2DataBase		= FourCharCode('192 ');						{  not registered  }
	ftAppleWorks3DataBase		= FourCharCode('193 ');						{  not registered  }
	ftAppleWorksSpreadsheet		= FourCharCode('1B  ');						{  not registered  }
	ftAppleWorks1Spreadsheet	= FourCharCode('1B1 ');						{  not registered  }
	ftAppleWorks2Spreadsheet	= FourCharCode('1B2 ');						{  not registered  }
	ftAppleWorks3Spreadsheet	= FourCharCode('1B3 ');						{  not registered  }
	ftAppleWorksWordProcessorGS	= FourCharCode('50  ');						{  not registered  }
	ftApple2GS_SuperHiRes		= FourCharCode('A2SU');						{  not registered  }
	ftApple2GS_SuperHiResPacked	= FourCharCode('A2SP');						{  not registered  }
	ftApple2GS_PaintWorks		= FourCharCode('A2PW');						{  not registered  }
	ftApple2_DoubleHiRes		= FourCharCode('A2DU');						{  not registered  }
	ftApple2_DoubleHiResPacked	= FourCharCode('A2DP');						{  not registered  }
	ftApple2_DoubleHiRes16colors = FourCharCode('A2DC');						{  not registered  }
	ftApple2_SingleHiRes		= FourCharCode('A2HU');						{  not registered  }
	ftApple2_SingleHiResPacked	= FourCharCode('A2HP');						{  not registered  }
	ftApple2_SingleHiRes8colors	= FourCharCode('A2HC');						{  not registered  }


	{	************* PC-DOS applications and FileTypes *************	}
	sigPCDOS					= FourCharCode('mdos');						{  not registered  }
	ftGenericDocumentPC			= FourCharCode('TEXT');						{     word processor formats  }
	ftWordStarDocumentPC		= FourCharCode('WStr');						{  not registered  }
	ftWordStar4DocumentPC		= FourCharCode('WSt4');						{  not registered  }
	ftWordStar5DocumentPC		= FourCharCode('WSt5');						{  not registered  }
	ftWordStar55DocumentPC		= FourCharCode('WS55');						{  not registered  }
	ftWordStar6DocumentPC		= FourCharCode('WSt6');						{  not registered  }
	ftWordStar2000DocumentPC	= FourCharCode('WS20');						{  not registered  }
	ftXyWriteIIIDocumentPC		= FourCharCode('XyWr');						{  registered???  }
	ftDecDXDocumentPC			= FourCharCode('DX  ');						{  registered???  }
	ftDecWPSPlusDocumentPC		= FourCharCode('WPS+');						{  registered???  }
	ftDisplayWrite3DocumentPC	= FourCharCode('DW3 ');						{  registered???  }
	ftDisplayWrite4DocumentPC	= FourCharCode('DW4 ');						{  registered???  }
	ftDisplayWrite5DocumentPC	= FourCharCode('DW5 ');						{  registered???  }
	ftIBMWritingAsstDocumentPC	= FourCharCode('ASST');						{  registered???  }
	ftManuscript1DocumentPC		= FourCharCode('MAN1');						{  registered???  }
	ftManuscript2DocumentPC		= FourCharCode('MAN2');						{  registered???  }
	ftMass11PCDocumentPC		= FourCharCode('M11P');						{  registered???  }
	ftMass11VaxDocumentPC		= FourCharCode('M11V');						{  registered???  }
	ftMultiMateDocumentPC		= FourCharCode('MMAT');						{  registered???  }
	ftMultiMate36DocumentPC		= FourCharCode('MM36');						{  registered???  }
	ftMultiMate40DocumentPC		= FourCharCode('MM40');						{  registered???  }
	ftMultiMateAdvDocumentPC	= FourCharCode('MMAD');						{  registered???  }
	ftMultiMateNoteDocumentPC	= FourCharCode('MMNT');						{  registered???  }
	ftOfficeWriterDocumentPC	= FourCharCode('OFFW');						{  registered???  }
	ftPCFileLetterDocumentPC	= FourCharCode('PCFL');						{  registered???  }
	ftPFSWriteADocumentPC		= FourCharCode('PFSA');						{  registered???  }
	ftPFSWriteBDocumentPC		= FourCharCode('PFSB');						{  registered???  }
	ftPFSPlanDocumentPC			= FourCharCode('PFSP');						{  registered???  }
	ftProWrite1DocumentPC		= FourCharCode('PW1 ');						{  registered???  }
	ftProWrite2DocumentPC		= FourCharCode('PW2 ');						{  registered???  }
	ftProWritePlusDocumentPC	= FourCharCode('PW+ ');						{  registered???  }
	ftFirstChoiceDocumentPC		= FourCharCode('FCH ');						{  registered???  }
	ftFirstChoice3DocumentPC	= FourCharCode('FCH3');						{  registered???  }
	ftDCARFTDocumentPC			= FourCharCode('RFT ');						{  registered???  }
	ftSamnaDocumentPC			= FourCharCode('SAMN');						{  registered???  }
	ftSmartDocumentPC			= FourCharCode('SMRT');						{  registered???  }
	ftSprintDocumentPC			= FourCharCode('SPRT');						{  registered???  }
	ftTotalWordDocumentPC		= FourCharCode('TOTL');						{  registered???  }
	ftVolksWriterDocumentPC		= FourCharCode('VOLK');						{  registered???  }
	ftWangWPSDocumentPC			= FourCharCode('WPS ');						{  registered???  }
	ftWordMarcDocumentPC		= FourCharCode('MARC');						{  registered???  }
	ftAmiDocumentPC				= FourCharCode('AMI ');						{  registered???  }
	ftAmiProDocumentPC			= FourCharCode('APRO');						{  registered???  }
	ftAmiPro2DocumentPC			= FourCharCode('APR2');						{  registered???  }
	ftEnableDocumentPC			= FourCharCode('ENWP');						{  registered???  }
																{     data base formats  }
	ftdBaseDatabasePC			= FourCharCode('DBF ');						{  registered???  }
	ftdBase3DatabasePC			= FourCharCode('DB3 ');						{  registered???  }
	ftdBase4DatabasePC			= FourCharCode('DB4 ');						{  registered???  }
	ftDataEaseDatabasePC		= FourCharCode('DTEZ');						{  registered???  }
	ftFrameWorkIIIDatabasePC	= FourCharCode('FWK3');						{  registered???  }
	ftRBaseVDatabasePC			= FourCharCode('RBsV');						{  registered???  }
	ftRBase5000DatabasePC		= FourCharCode('RB50');						{  registered???  }
	ftRBaseFile1DatabasePC		= FourCharCode('RBs1');						{  registered???  }
	ftRBaseFile3DatabasePC		= FourCharCode('RBs3');						{  registered???  }
	ftReflexDatabasePC			= FourCharCode('RFLX');						{  registered???  }
	ftQAWriteDatabasePC			= FourCharCode('QAWT');						{  registered???  }
	ftQADBaseDatabasePC			= FourCharCode('QADB');						{  registered???  }
	ftSmartDataBasePC			= FourCharCode('SMTD');						{  registered???  }
	ftFirstChoiceDataBasePC		= FourCharCode('FCDB');						{  registered???  }

																{     spread sheet formats  }
	ftDIFSpreadsheetPC			= FourCharCode('DIF ');						{  registered???  }
	ftEnableSpreadsheetPC		= FourCharCode('ENAB');						{  registered???  }
	ft123R1SpreadsheetPC		= FourCharCode('WKS1');						{  registered???  }
	ft123R2SpreadsheetPC		= FourCharCode('WKS2');						{  registered???  }
	ft123R3SpreadsheetPC		= FourCharCode('WKS3');						{  registered???  }
	ftParadox3SpreadsheetPC		= FourCharCode('PDX3');						{  registered???  }
	ftParadox35SpreadsheetPC	= FourCharCode('PD35');						{  registered???  }
	ftQuattroSpreadsheetPC		= FourCharCode('QTRO');						{  registered???  }
	ftQuattroProSpreadsheetPC	= FourCharCode('QTR5');						{  registered???  }
	ftSuperCalc5SpreadsheetPC	= FourCharCode('SPC5');						{  registered???  }
	ftSymphony1SpreadsheetPC	= FourCharCode('SYM1');						{  registered???  }
	ftTwinSpreadsheetPC			= FourCharCode('TWIN');						{  registered???  }
	ftVPPlannerSpreadsheetPC	= FourCharCode('VPPL');						{  registered???  }
	ftSmartSpeadsheetPC			= FourCharCode('SMSH');						{  registered???  }
	ftFirstChoiceSpeadsheetPC	= FourCharCode('FCSS');						{  registered???  }
																{     graphics formats  }
	ftPCPaintBrushGraphicPC		= FourCharCode('PCX ');						{  not registered  }
	ftLotusPICGraphicPC			= FourCharCode('.PIC');						{  not registered  }
	ftCGMGraphicPC				= FourCharCode('.CGM');						{  not registered  }
	ftGEMGraphicPC				= FourCharCode('.GEM');						{  not registered  }
	ftIMGGraphicPC				= FourCharCode('.IMG');						{  not registered  }
	ftDXFGraphicPC				= FourCharCode('.DXF');						{  not registered  }
	ftBitmapWindows				= FourCharCode('.BMP');						{  not registered  }
	ftMetaFileWindows			= FourCharCode('.WMF');						{  not registered  }
	ftTIFFGraphic				= FourCharCode('TIFF');						{  not registered  }
	ftPostScriptPC				= FourCharCode('EPSP');
	ftPostScriptWindows			= FourCharCode('EPSW');						{  not registered  }
	ftDigitalFX_TitleMan		= FourCharCode('TDIM');						{  registered???  }
	ftDigitalFX_VideoFX			= FourCharCode('GRAF');						{  registered???  }
	ftAutodeskFLIandFLC			= FourCharCode('FLIC');						{  registered???  }
	ftGIF						= FourCharCode('GIFf');						{  registered???  }
	ftIFF						= FourCharCode('ILBM');						{  registered???  }
	ftMicrosoftPaint			= FourCharCode('.MSP');						{  registered???  }
	ftPixar						= FourCharCode('PXAR');						{  registered???  }
	ftQDV						= FourCharCode('.QDV');						{  registered???  }
	ftRLE_Compuserve			= FourCharCode('RLEC');						{  registered???  }
																{     Generic vector formats  }
	ftIGESGraphicPC				= FourCharCode('IGES');						{  not registered  }
	ftDDES2GraphicPC			= FourCharCode('DDES');						{  not registered  }
	ft3DGFGraphicPC				= FourCharCode('3DGF');						{  not registered  }
																{     Plotter formats  }
	ftHPGLGraphicPC				= FourCharCode('HPGL');						{  not registered  }
	ftDMPLGraphicPC				= FourCharCode('DMPL');						{  not registered  }
	ftCalComp906GraphicPC		= FourCharCode('C906');						{  not registered  }
	ftCalComp907GraphicPC		= FourCharCode('C907');						{  not registered  }
																{     Vendor-specific formats  }
	ftStereoLithographyGraphicPC = FourCharCode('STL ');						{     3D Systems     - not registered  }
	ftZoomGraphicPC				= FourCharCode('ZOOM');						{     Abvent          - not registered  }
	ftFocusGraphicPC			= FourCharCode('FOCS');						{     Abvent          - not registered  }
	ftWaveFrontGraphicPC		= FourCharCode('WOBJ');						{     WaveFront      - not registered  }
	ftSculpt4DGraphicPC			= FourCharCode('Scn2');						{     Byte By Byte   - not registered  }
	ftMiniPascal3GraphicPC		= FourCharCode('MPT3');						{     Graphsoft      - not registered  }
	ftMiniPascal4GraphicPC		= FourCharCode('MPT4');						{     Graphsoft      - not registered  }
	ftWalkThroughGraphicPC		= FourCharCode('VWLK');						{     Virtus          - not registered  }
	ftSiliconGraphics			= FourCharCode('.SGI');						{  registered???  }
	ftSunRaster					= FourCharCode('.SUN');						{  registered???  }
	ftTarga						= FourCharCode('TPIC');						{  registered???  }
																{  misc DOS   }
	ftDOSComPC					= FourCharCode('.COM');						{  registered???  }
	ftDOSExecutablePC			= FourCharCode('.EXE');						{  registered???  }
	ftDOSArcPC					= FourCharCode('.ARC');						{  registered???  }
	ftAbekas					= FourCharCode('ABEK');						{  registered???  }
	ftDrHaloCUT					= FourCharCode('.CUT');						{  registered???  }
																{  misc Atari  }
	ftDegas						= FourCharCode('DEGA');						{  not registered  }
	ftNEO						= FourCharCode('.NEO');						{  not registered  }


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
{$ALIGN MAC68K}


end.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4
