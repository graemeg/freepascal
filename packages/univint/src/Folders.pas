{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       CarbonCore/Folders.h
 
     Contains:   Folder Manager Interfaces.
                 The contents of this header file are deprecated.
                 Use NSFileManager instead.
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Copyright:  © 1995-2011 by Apple Inc. All rights reserved.
}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
     Version:    CarbonCore-859.2~1
 
     Copyright:  © 1995-2008 by Apple Computer, Inc., all rights reserved.
=======
=======
>>>>>>> origin/fixes_2_2
     File:       Folders.p
=======
     File:       CarbonCore/Folders.h
>>>>>>> origin/fixes_2.4
 
     Contains:   Folder Manager Interfaces.
 
     Version:    CarbonCore-859.2~1
 
<<<<<<< HEAD
     Copyright:  © 1995-2002 by Apple Computer, Inc., all rights reserved.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  © 1995-2008 by Apple Computer, Inc., all rights reserved.
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

unit Folders;
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
=======
>>>>>>> origin/cpstrnew
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
>>>>>>> origin/cpstrnew
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
=======
>>>>>>> origin/cpstrnew
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
{$elsec}
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
uses MacTypes,Files;
{$endc} {not MACOSALLINCLUDE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}
=======

>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}
=======


{$ifc TARGET_OS_MAC}
>>>>>>> graemeg/cpstrnew
=======


{$ifc TARGET_OS_MAC}
>>>>>>> origin/cpstrnew
=======
uses MacTypes,MixedMode,Files;

>>>>>>> graemeg/fixes_2_2
=======
uses MacTypes,MixedMode,Files;

>>>>>>> origin/fixes_2_2
=======
uses MacTypes,Files;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}
>>>>>>> origin/fixes_2.4

{$ALIGN MAC68K}


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{
    Common folder locations:
    ========================
    kSystemDomain is generally /System and things inside it, so
     - kSystemDomain, kDomainLibraryFolderType is /System/Library/
     - kSystemDomain, kTrashFolderType is a trash folder on the same volume as /System ( the boot disk )
     - kSystemDomain, kDomainTopLevelFolderType is the root of the system domain, so it's the same as /System
    kLocalDomain is generally the admin-writeable, system-wide location for things, so
     - kLocalDomain, kDomainLibraryFolderType is /Library/
    kUserDomain maps to the current user's home folder, so that
     - kUserDomain, kCurrentUserFolderType is the user's home folder itself ( "$HOME", ~ )
     - kUserDomain, kDomainLibraryFolderType is the Library folder in the user home ( "$HOME/Library", "~/Library/" )
     - kUserDomain, kPreferencesFolderType is the Preferences folder in the user home ( "$HOME/Library/Preferences/" )
     - kUserDomain, kTrashFolderType is a trash folder on the same volume as the user home
    kNetworkDomain, if configured, is a network file system which a network administrator may have installed items into
     - kNetworkDomain, kApplicationsFolderType is /Network/Applications/
    kClassicDomain, if configured, is where the Mac OS X Classic environment information is stored
     - kClassicDomain, kSystemFolderType is the currently active Macintosh Classic system folder ( or fnfErr if a Classic isn't installed )
}

<<<<<<< HEAD
const
	kOnSystemDisk = -32768; { previously was 0x8000 but that is an unsigned value whereas vRefNum is signed}
	kOnAppropriateDisk = -32767; { Generally, the same as kOnSystemDisk, but it's clearer that this isn't always the 'boot' disk.}
                                        { Folder Domains - Carbon only.  The constants above can continue to be used, but the folder/volume returned will}
                                        { be from one of the domains below.}
	kSystemDomain = -32766; { Read-only system hierarchy.}
	kLocalDomain = -32765; { All users of a single machine have access to these resources.}
	kNetworkDomain = -32764; { All users configured to use a common network server has access to these resources.}
	kUserDomain = -32763; { Read/write. Resources that are private to the user.}
	kClassicDomain = -32762; { Domain referring to the currently configured Classic System Folder.  Not supported in Mac OS X Leopard and later.}
	kFolderManagerLastDomain = -32760;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/cpstrnew
{
   The ID of the last domain in the above list, used by the Folder Manager to determine if a given 
   parameter should be treated as a domain or a volume...
}
const
	kLastDomainConstant = -32760;

const
	kCreateFolder = true;
	kDontCreateFolder = false;

{
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
{
   The ID of the last domain in the above list, used by the Folder Manager to determine if a given 
   parameter should be treated as a domain or a volume...
}
const
	kLastDomainConstant = -32760;

const
	kCreateFolder = true;
	kDontCreateFolder = false;

{
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 *  FindFolder()
 *  
 *  Summary:
 *    Obtains location information for system-related directories.
 *  
 *  Discussion:
 *    For the folder type on the particular volume (specified,
 *    respectively, in the folderType and vRefNum parameters), the
 *    FindFolder function returns the directory's volume reference
 *    number in the foundVRefNum parameter and its directory ID in the
 *    foundDirID parameter.
 *    
 *    The specified folder used for a given volume might be located on
 *    a different volume in future versions of system software;
 *    therefore, do not assume the volume that you specify in vRefNum
 *    and the volume returned in foundVRefNum will be the same.
 *     
 *    Specify a volume reference number (or the constant kOnSystemDisk
 *    for the startup disk) or one of the domain constants ( on Mac OS
 *    X ) in the vRefNum parameter.
 *    
 *    Specify a four-character folder type--or the constant that
 *    represents it--in the folderType parameter.
 *    
 *    Use the constant kCreateFolder in the createFolder parameter to
 *    tell FindFolder to create a directory if it does not already
 *    exist; otherwise, use the constant kDontCreateFolder. Directories
 *    inside the System Folder are created only if the System Folder
 *    directory exists. The FindFolder function will not create a
 *    System Folder directory even if you specify the kCreateFolder
 *    constant in the createFolder parameter.
 *    
 *    The FindFolder function returns a nonzero result code if the
 *    folder isn't found, and it can also return other file system
 *    errors reported by the File Manager or Memory Manager.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    vRefNum:
 *      Pass the volume reference number of the volume on which you
 *      want to locate a directory, or a constant specifying a disk or
 *      domain.   The constants which you can use in this parameter are
 *      described in "Disk and Domain Constants".
 *      Note that, on Mac OS X, passing a volume reference number in
 *      this parameter does not make sense for most of the folder type
 *      selectors which you can specify in the folderType parameter. On
 *      Mac OS X, folders are "domain-oriented"; because there may be
 *      more than one domain on any given physical volume, asking for
 *      these folders on a per-volume basis yields undefined results.
 *      For example, if you were to request the Fonts folder
 *      (represented by the selector kFontsFolderType ) on volume -100,
 *      are you requesting the folder /System/Library/Fonts,
 *      /Library/Fonts, or ~/Fonts? On Mac OS X you should pass a disk
 *      or domain constant in this parameter.
 *    
 *    folderType:
 *      A four-character folder type, or a constant that represents the
 *      type, for the directory you want to find.
 *    
 *    createFolder:
 *      Pass the constant kCreateFolder in this parameter to create a
 *      directory if it does not already exist; otherwise, pass the
 *      constant kDontCreateFolder.
 *    
 *    foundVRefNum:
 *      The volume reference number, returned by FindFolder , for the
 *      volume containing the directory you specify in the folderType
 *      parameter.
 *    
 *    foundDirID:
 *      The directory ID number, returned by FindFolder , for the
 *      directory you specify in the folderType parameter.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function FindFolder( vRefNum: FSVolumeRefNum; folderType: OSType; createFolder: Boolean; var foundVRefNum: FSVolumeRefNum; var foundDirID: SInt32 ): OSErr; external name '_FindFolder';
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
 *  ReleaseFolder()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    This call is not needed on Mac OS X and later.
 *  
 *  Summary:
 *    On Mac OS 9.x and earlier, release any hold the system may have
 *    on a given folder on a volume so that the volume may be unmounted.
 *  
 *  Discussion:
 *    On Mac OS 9.x, the system sometimes has files open on volumes
 *    which need to be closed in order for the volume to be
 *    successfully unmounted.  This call releases any hold the Folder
 *    Manager may have for the given volume.
 *    <br> This call is unnecessary on Mac OS X and later.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    vRefNum:
 *      The vRefNum to release.
 *    
 *    folderType:
 *      The folder type to release.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FoldersLib 1.0 and later
 }
function ReleaseFolder( vRefNum: FSVolumeRefNum; folderType: OSType ): OSErr; external name '_ReleaseFolder';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_3, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)
>>>>>>> origin/cpstrnew

=======
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
const
	kOnSystemDisk = -32768; { previously was 0x8000 but that is an unsigned value whereas vRefNum is signed}
	kOnAppropriateDisk = -32767; { Generally, the same as kOnSystemDisk, but it's clearer that this isn't always the 'boot' disk.}
                                        { Folder Domains - Carbon only.  The constants above can continue to be used, but the folder/volume returned will}
                                        { be from one of the domains below.}
	kSystemDomain = -32766; { Read-only system hierarchy.}
	kLocalDomain = -32765; { All users of a single machine have access to these resources.}
	kNetworkDomain = -32764; { All users configured to use a common network server has access to these resources.}
	kUserDomain = -32763; { Read/write. Resources that are private to the user.}
	kClassicDomain = -32762; { Domain referring to the currently configured Classic System Folder.  Not supported in Mac OS X Leopard and later.}
	kFolderManagerLastDomain = -32760;

{
   The ID of the last domain in the above list, used by the Folder Manager to determine if a given 
   parameter should be treated as a domain or a volume...
}
const
	kLastDomainConstant = -32760;

const
	kCreateFolder = true;
	kDontCreateFolder = false;

{
 *  FindFolder()
 *  
 *  Summary:
 *    Obtains location information for system-related directories.
 *  
 *  Discussion:
 *    For the folder type on the particular volume (specified,
 *    respectively, in the folderType and vRefNum parameters), the
 *    FindFolder function returns the directory's volume reference
 *    number in the foundVRefNum parameter and its directory ID in the
 *    foundDirID parameter.
 *    
 *    The specified folder used for a given volume might be located on
 *    a different volume in future versions of system software;
 *    therefore, do not assume the volume that you specify in vRefNum
 *    and the volume returned in foundVRefNum will be the same.
 *     
 *    Specify a volume reference number (or the constant kOnSystemDisk
 *    for the startup disk) or one of the domain constants ( on Mac OS
 *    X ) in the vRefNum parameter.
 *    
 *    Specify a four-character folder type--or the constant that
 *    represents it--in the folderType parameter.
 *    
 *    Use the constant kCreateFolder in the createFolder parameter to
 *    tell FindFolder to create a directory if it does not already
 *    exist; otherwise, use the constant kDontCreateFolder. Directories
 *    inside the System Folder are created only if the System Folder
 *    directory exists. The FindFolder function will not create a
 *    System Folder directory even if you specify the kCreateFolder
 *    constant in the createFolder parameter.
 *    
 *    The FindFolder function returns a nonzero result code if the
 *    folder isn't found, and it can also return other file system
 *    errors reported by the File Manager or Memory Manager.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    vRefNum:
 *      Pass the volume reference number of the volume on which you
 *      want to locate a directory, or a constant specifying a disk or
 *      domain.   The constants which you can use in this parameter are
 *      described in "Disk and Domain Constants".
 *      Note that, on Mac OS X, passing a volume reference number in
 *      this parameter does not make sense for most of the folder type
 *      selectors which you can specify in the folderType parameter. On
 *      Mac OS X, folders are "domain-oriented"; because there may be
 *      more than one domain on any given physical volume, asking for
 *      these folders on a per-volume basis yields undefined results.
 *      For example, if you were to request the Fonts folder
 *      (represented by the selector kFontsFolderType ) on volume -100,
 *      are you requesting the folder /System/Library/Fonts,
 *      /Library/Fonts, or ~/Fonts? On Mac OS X you should pass a disk
 *      or domain constant in this parameter.
 *    
 *    folderType:
 *      A four-character folder type, or a constant that represents the
 *      type, for the directory you want to find.
 *    
 *    createFolder:
 *      Pass the constant kCreateFolder in this parameter to create a
 *      directory if it does not already exist; otherwise, pass the
 *      constant kDontCreateFolder.
 *    
 *    foundVRefNum:
 *      The volume reference number, returned by FindFolder , for the
 *      volume containing the directory you specify in the folderType
 *      parameter.
 *    
 *    foundDirID:
 *      The directory ID number, returned by FindFolder , for the
 *      directory you specify in the folderType parameter.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function FindFolder( vRefNum: FSVolumeRefNum; folderType: OSType; createFolder: Boolean; var foundVRefNum: FSVolumeRefNum; var foundDirID: SInt32 ): OSErr; external name '_FindFolder';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  ReleaseFolder()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    This call is not needed on Mac OS X and later.
 *  
 *  Summary:
 *    On Mac OS 9.x and earlier, release any hold the system may have
 *    on a given folder on a volume so that the volume may be unmounted.
 *  
 *  Discussion:
 *    On Mac OS 9.x, the system sometimes has files open on volumes
 *    which need to be closed in order for the volume to be
 *    successfully unmounted.  This call releases any hold the Folder
 *    Manager may have for the given volume.
 *    <br> This call is unnecessary on Mac OS X and later.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    vRefNum:
 *      The vRefNum to release.
 *    
 *    folderType:
 *      The folder type to release.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FoldersLib 1.0 and later
 }
function ReleaseFolder( vRefNum: FSVolumeRefNum; folderType: OSType ): OSErr; external name '_ReleaseFolder';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)

<<<<<<< HEAD
{$endc}  {CALL_NOT_IN_CARBON}
{$endc}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{
 *  FSFindFolder()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  Summary:
 *    FSFindFolder returns an FSRef for certain system-related
 *    directories.
 *  
 *  Discussion:
 *    For the folder type on the particular volume (specified,
 *    respectively, in the folderType and vRefNum parameters), the
 *    FindFolder function returns the FSRef of that directory. 
 *     
 *    The specified folder used for a given volume might be located on
 *    a different volume in future versions of system software;
 *    therefore, do not assume the volume that you specify in vRefNum
 *    and the volume returned in the FSRef will be the same.
 *    
 *    Specify a volume reference number (or the constant kOnSystemDisk
 *    for the startup disk) or one of the domain constants ( on Mac OS
 *    X ) in the vRefNum parameter.
 *    
 *    Specify a four-character folder type--or the constant that
 *    represents it--in the folderType parameter.
 *    
 *    Use the constant kCreateFolder in the createFolder parameter to
 *    tell FindFolder to create a directory if it does not already
 *    exist; otherwise, use the constant kDontCreateFolder. Directories
 *    inside the System Folder are created only if the System Folder
 *    directory exists. The FindFolder function will not create a
 *    System Folder directory even if you specify the kCreateFolder
 *    constant in the createFolder parameter.
 *    
 *    The FindFolder function returns a nonzero result code if the
 *    folder isn't found, and it can also return other file system
 *    errors reported by the File Manager or Memory Manager.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    vRefNum:
 *      The volume reference number (or the constant kOnSystemDisk for
 *      the startup disk) or one of the domain constants ( like
 *      kUserDomain ) of the volume or domain in which you want to
 *      locate a directory.
 *    
 *    folderType:
 *      A four-character folder type, or a constant that represents the
 *      type, for the directory you want to find.
 *    
 *    createFolder:
 *      Pass the constant kCreateFolder in this parameter to create a
 *      directory if it does not already exist; otherwise, pass the
 *      constant kDontCreateFolder.
 *    
 *    foundRef:
 *      The FSRef for the directory you specify on the volume or domain
 *      and folderType given.
 *  
<<<<<<< HEAD
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSFindFolder( vRefNum: FSVolumeRefNum; folderType: OSType; createFolder: Boolean; var foundRef: FSRef ): OSErr; external name '_FSFindFolder';
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


{//}

{
 *  Folder types
 *  
 }
const
	kDesktopFolderType = FourCharCode('desk'); { the desktop folder; objects in this folder show on the desktop. }
	kTrashFolderType = FourCharCode('trsh'); { the trash folder; objects in this folder show up in the trash }
	kWhereToEmptyTrashFolderType = FourCharCode('empt'); { the "empty trash" folder; Finder starts empty from here down }
	kFontsFolderType = FourCharCode('font'); { Fonts go here }
	kPreferencesFolderType = FourCharCode('pref'); { preferences for applications go here }
	kSystemPreferencesFolderType = FourCharCode('sprf'); { the PreferencePanes folder, where Mac OS X Preference Panes go }
	kTemporaryFolderType = FourCharCode('temp'); {    On Mac OS X, each user has their own temporary items folder, and the Folder Manager attempts to set permissions of these}
                                        {    folders such that other users can not access the data inside.  On Mac OS X 10.4 and later the data inside the temporary}
                                        {    items folder is deleted at logout and at boot, but not otherwise.  Earlier version of Mac OS X would delete items inside}
                                        {    the temporary items folder after a period of inaccess.  You can ask for a temporary item in a specific domain or on a }
                                        {    particular volume by FSVolumeRefNum.  If you want a location for temporary items for a short time, then use either}
                                        {    ( kUserDomain, kkTemporaryFolderType ) or ( kSystemDomain, kTemporaryFolderType ).  The kUserDomain varient will always be}
                                        {    on the same volume as the user's home folder, while the kSystemDomain version will be on the same volume as /var/tmp/ ( and}
                                        {    will probably be on the local hard drive in case the user's home is a network volume ).  If you want a location for a temporary}
                                        {    file or folder to use for saving a document, especially if you want to use FSpExchangeFile() to implement a safe-save, then}
                                        {    ask for the temporary items folder on the same volume as the file you are safe saving.}
                                        {    However, be prepared for a failure to find a temporary folder in any domain or on any volume.  Some volumes may not have}
                                        {    a location for a temporary folder, or the permissions of the volume may be such that the Folder Manager can not return}
                                        {    a temporary folder for the volume.}
                                        {    If your application creates an item in a temporary items older you should delete that item as soon as it is not needed,}
                                        {    and certainly before your application exits, since otherwise the item is consuming disk space until the user logs out or}
                                        {    restarts.  Any items left inside a temporary items folder should be moved into a folder inside the Trash folder on the disk}
                                        {    when the user logs in, inside a folder named "Recovered items", in case there is anything useful to the end user.}
	kChewableItemsFolderType = FourCharCode('flnt'); { similar to kTemporaryItemsFolderType, except items in this folder are deleted at boot or when the disk is unmounted }
	kTemporaryItemsInCacheDataFolderType = FourCharCode('vtmp'); { A folder inside the kCachedDataFolderType for the given domain which can be used for transient data}
	kApplicationsFolderType = FourCharCode('apps'); {    Applications on Mac OS X are typically put in this folder ( or a subfolder ).}
	kVolumeRootFolderType = FourCharCode('root'); { root folder of a volume or domain }
	kDomainTopLevelFolderType = FourCharCode('dtop'); { The top-level of a Folder domain, e.g. "/System"}
	kDomainLibraryFolderType = FourCharCode('dlib'); { the Library subfolder of a particular domain}
	kUsersFolderType = FourCharCode('usrs'); { "Users" folder, usually contains one folder for each user. }
	kCurrentUserFolderType = FourCharCode('cusr'); { The folder for the currently logged on user; domain passed in is ignored. }
	kSharedUserDataFolderType = FourCharCode('sdat'); { A Shared folder, readable & writeable by all users }
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/cpstrnew

{
    The following selectors refer specifically to subfolders inside the user's home folder, and should
    be used only with  kUserDomain as the domain in the various FindFolder calls.
}
const
	kDocumentsFolderType = FourCharCode('docs'); {    User documents are typically put in this folder ( or a subfolder ).}
	kPictureDocumentsFolderType = FourCharCode('pdoc'); { Refers to the "Pictures" folder in a users home directory}
	kMovieDocumentsFolderType = FourCharCode('mdoc'); { Refers to the "Movies" folder in a users home directory}
	kMusicDocumentsFolderType = FourCharCode('µdoc'); { Refers to the "Music" folder in a users home directory}
	kInternetSitesFolderType = FourCharCode('site'); { Refers to the "Sites" folder in a users home directory}
	kPublicFolderType = FourCharCode('pubb'); { Refers to the "Public" folder in a users home directory}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{  The following selectors are available on Mac OS X 10.7 and later.}
const
	kDropBoxFolderType = FourCharCode('drop'); { Refers to the "Drop Box" folder inside the user's home directory}

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
const
	kSharedLibrariesFolderType = FourCharCode('Älib'); { for general shared libs. }
	kVoicesFolderType = FourCharCode('fvoc'); { macintalk can live here }
	kUtilitiesFolderType = FourCharCode('utiÄ'); { for Utilities folder }
	kThemesFolderType = FourCharCode('thme'); { for Theme data files }
	kFavoritesFolderType = FourCharCode('favs'); { Favorties folder for Navigation Services }
	kInternetSearchSitesFolderType = FourCharCode('issf'); { Internet Search Sites folder }
	kInstallerLogsFolderType = FourCharCode('ilgf'); { Installer Logs folder }
	kScriptsFolderType = FourCharCode('scrÄ'); { Scripts folder }
	kFolderActionsFolderType = FourCharCode('fasf'); { Folder Actions Scripts folder }
	kSpeakableItemsFolderType = FourCharCode('spki'); { Speakable Items folder }
	kKeychainFolderType = FourCharCode('kchn'); { Keychain folder }

{ New Folder Types to accommodate the Mac OS X Folder Manager }
{ These folder types are not applicable on Mac OS 9.          }
const
	kColorSyncFolderType = FourCharCode('sync'); { Contains ColorSync-related folders}
	kColorSyncCMMFolderType = FourCharCode('ccmm'); { ColorSync CMMs}
	kColorSyncScriptingFolderType = FourCharCode('cscr'); { ColorSync Scripting support}
	kPrintersFolderType = FourCharCode('impr'); { Contains Printing-related folders}
	kSpeechFolderType = FourCharCode('spch'); { Contains Speech-related folders}
	kCarbonLibraryFolderType = FourCharCode('carb'); { Contains Carbon-specific file}
	kDocumentationFolderType = FourCharCode('info'); { Contains Documentation files (not user documents)}
	kISSDownloadsFolderType = FourCharCode('issd'); { Contains Internet Search Sites downloaded from the Internet}
	kUserSpecificTmpFolderType = FourCharCode('utmp'); { Contains temporary items created on behalf of the current user}
	kCachedDataFolderType = FourCharCode('cach'); { Contains various cache files for different clients}
	kFrameworksFolderType = FourCharCode('fram'); { Contains MacOS X Framework folders}
	kPrivateFrameworksFolderType = FourCharCode('pfrm'); { Contains MacOS X Private Framework folders     }
	kClassicDesktopFolderType = FourCharCode('sdsk'); { MacOS 9 compatible desktop folder - same as kSystemDesktopFolderType but with a more appropriate name for Mac OS X code.}
	kSystemSoundsFolderType = FourCharCode('ssnd'); { Contains Mac OS X System Sound Files ( valid in kSystemDomain, kLocalDomain, and kUserDomain )}
	kComponentsFolderType = FourCharCode('cmpd'); { Contains Mac OS X components   ( valid in kSystemDomain, kLocalDomain, and kUserDomain )}
	kQuickTimeComponentsFolderType = FourCharCode('wcmp'); { Contains QuickTime components for Mac OS X ( valid in kSystemDomain, kLocalDomain, and kUserDomain )}
	kCoreServicesFolderType = FourCharCode('csrv'); { Refers to the "/System/Library/CoreServices" folder on Mac OS X}
	kAudioSupportFolderType = FourCharCode('adio'); { Refers to the Audio support folder for Mac OS X}
	kAudioPresetsFolderType = FourCharCode('apst'); { "Presets" folder of "Audio" folder, Mac OS X 10.4 and later}
	kAudioSoundsFolderType = FourCharCode('asnd'); { Refers to the Sounds subfolder of Audio Support}
	kAudioSoundBanksFolderType = FourCharCode('bank'); { Refers to the Banks subfolder of the Sounds Folder}
	kAudioAlertSoundsFolderType = FourCharCode('alrt'); { Refers to the Alert Sounds subfolder of the Sound Folder}
	kAudioPlugInsFolderType = FourCharCode('aplg'); { Refers to the Plug-ins subfolder of the Audio Folder   }
	kAudioComponentsFolderType = FourCharCode('acmp'); { Refers to the Components subfolder of the Audio Plug-ins Folder    }
	kKernelExtensionsFolderType = FourCharCode('kext'); { Refers to the Kernel Extensions Folder on Mac OS X}
	kDirectoryServicesFolderType = FourCharCode('dsrv'); { Refers to the Directory Services folder on Mac OS X}
	kDirectoryServicesPlugInsFolderType = FourCharCode('dplg'); { Refers to the Directory Services Plug-Ins folder on Mac OS X }
	kInstallerReceiptsFolderType = FourCharCode('rcpt'); { Refers to the "Receipts" folder in Mac OS X}
	kFileSystemSupportFolderType = FourCharCode('fsys'); { Refers to the [domain]/Library/Filesystems folder in Mac OS X}
	kAppleShareSupportFolderType = FourCharCode('shar'); { Refers to the [domain]/Library/Filesystems/AppleShare folder in Mac OS X}
	kAppleShareAuthenticationFolderType = FourCharCode('auth'); { Refers to the [domain]/Library/Filesystems/AppleShare/Authentication folder in Mac OS X}
	kMIDIDriversFolderType = FourCharCode('midi'); { Refers to the MIDI Drivers folder on Mac OS X}
	kKeyboardLayoutsFolderType = FourCharCode('klay'); { Refers to the [domain]/Library/KeyboardLayouts folder in Mac OS X}
	kIndexFilesFolderType = FourCharCode('indx'); { Refers to the [domain]/Library/Indexes folder in Mac OS X}
	kFindByContentIndexesFolderType = FourCharCode('fbcx'); { Refers to the [domain]/Library/Indexes/FindByContent folder in Mac OS X}
	kManagedItemsFolderType = FourCharCode('mang'); { Refers to the Managed Items folder for Mac OS X }
	kBootTimeStartupItemsFolderType = FourCharCode('empz'); { Refers to the "StartupItems" folder of Mac OS X }
	kAutomatorWorkflowsFolderType = FourCharCode('flow'); { Automator Workflows folder }
	kAutosaveInformationFolderType = FourCharCode('asav'); { ~/Library/Autosaved Information/ folder, can be used to store autosave information for user's applications.  Available in Mac OS X 10.4 and later.  }
	kSpotlightSavedSearchesFolderType = FourCharCode('spot'); { Usually ~/Library/Saved Searches/; used by Finder and Nav/Cocoa panels to find saved Spotlight searches }
                                        { The following folder types are available in Mac OS X 10.5 and later }
	kSpotlightImportersFolderType = FourCharCode('simp'); { Folder for Spotlight importers, usually /Library/Spotlight/ or ~/Library/Spotlight, etc. }
	kSpotlightMetadataCacheFolderType = FourCharCode('scch'); { Folder for Spotlight metadata caches, for example: ~/Library/Caches/Metadata/ }
	kInputManagersFolderType = FourCharCode('inpt'); { InputManagers }
	kInputMethodsFolderType = FourCharCode('inpf'); { ../Library/Input Methods/ }
	kLibraryAssistantsFolderType = FourCharCode('astl'); { Refers to the [domain]/Library/Assistants folder}
	kAudioDigidesignFolderType = FourCharCode('adig'); { Refers to the Digidesign subfolder of the Audio Plug-ins folder}
	kAudioVSTFolderType = FourCharCode('avst'); { Refers to the VST subfolder of the Audio Plug-ins folder}
	kColorPickersFolderType = FourCharCode('cpkr'); { Refers to the ColorPickers folder}
	kCompositionsFolderType = FourCharCode('cmps'); { Refers to the Compositions folder}
	kFontCollectionsFolderType = FourCharCode('fncl'); { Refers to the FontCollections folder}
	kiMovieFolderType = FourCharCode('imov'); { Refers to the iMovie folder}
	kiMoviePlugInsFolderType = FourCharCode('impi'); { Refers to the Plug-ins subfolder of the iMovie Folder}
	kiMovieSoundEffectsFolderType = FourCharCode('imse'); { Refers to the Sound Effects subfolder of the iMovie Folder}
	kDownloadsFolderType = FourCharCode('down'); { Refers to the ~/Downloads folder}

const
	kColorSyncProfilesFolderType = FourCharCode('prof'); { for ColorSyncª Profiles }
	kApplicationSupportFolderType = FourCharCode('asup'); { third-party items and folders }
	kTextEncodingsFolderType = FourCharCode('Ätex'); { encoding tables }
	kPrinterDescriptionFolderType = FourCharCode('ppdf'); { new folder at root of System folder for printer descs. }
	kPrinterDriverFolderType = FourCharCode('Äprd'); { new folder at root of System folder for printer drivers }
	kScriptingAdditionsFolderType = FourCharCode('Äscr'); { at root of system folder }
<<<<<<< HEAD

const
	kClassicPreferencesFolderType = FourCharCode('cprf'); { "Classic" folder in ~/Library/ for redirected preference files. }

const
	kQuickLookFolderType = FourCharCode('qlck'); { The QuickLook folder, supported in Mac OS X SnowLeopard and later. }

const
=======

{
    The following selectors refer specifically to subfolders inside the user's home folder, and should
    be used only with  kUserDomain as the domain in the various FindFolder calls.
}
const
	kDocumentsFolderType = FourCharCode('docs'); {    User documents are typically put in this folder ( or a subfolder ).}
	kPictureDocumentsFolderType = FourCharCode('pdoc'); { Refers to the "Pictures" folder in a users home directory}
	kMovieDocumentsFolderType = FourCharCode('mdoc'); { Refers to the "Movies" folder in a users home directory}
	kMusicDocumentsFolderType = FourCharCode('µdoc'); { Refers to the "Music" folder in a users home directory}
	kInternetSitesFolderType = FourCharCode('site'); { Refers to the "Sites" folder in a users home directory}
	kPublicFolderType = FourCharCode('pubb'); { Refers to the "Public" folder in a users home directory}

const
	kSharedLibrariesFolderType = FourCharCode('Älib'); { for general shared libs. }
	kVoicesFolderType = FourCharCode('fvoc'); { macintalk can live here }
	kUtilitiesFolderType = FourCharCode('utiÄ'); { for Utilities folder }
	kThemesFolderType = FourCharCode('thme'); { for Theme data files }
	kFavoritesFolderType = FourCharCode('favs'); { Favorties folder for Navigation Services }
	kInternetSearchSitesFolderType = FourCharCode('issf'); { Internet Search Sites folder }
	kInstallerLogsFolderType = FourCharCode('ilgf'); { Installer Logs folder }
	kScriptsFolderType = FourCharCode('scrÄ'); { Scripts folder }
	kFolderActionsFolderType = FourCharCode('fasf'); { Folder Actions Scripts folder }
	kSpeakableItemsFolderType = FourCharCode('spki'); { Speakable Items folder }
	kKeychainFolderType = FourCharCode('kchn'); { Keychain folder }

{ New Folder Types to accommodate the Mac OS X Folder Manager }
{ These folder types are not applicable on Mac OS 9.          }
const
	kColorSyncFolderType = FourCharCode('sync'); { Contains ColorSync-related folders}
	kColorSyncCMMFolderType = FourCharCode('ccmm'); { ColorSync CMMs}
	kColorSyncScriptingFolderType = FourCharCode('cscr'); { ColorSync Scripting support}
	kPrintersFolderType = FourCharCode('impr'); { Contains Printing-related folders}
	kSpeechFolderType = FourCharCode('spch'); { Contains Speech-related folders}
	kCarbonLibraryFolderType = FourCharCode('carb'); { Contains Carbon-specific file}
	kDocumentationFolderType = FourCharCode('info'); { Contains Documentation files (not user documents)}
	kISSDownloadsFolderType = FourCharCode('issd'); { Contains Internet Search Sites downloaded from the Internet}
	kUserSpecificTmpFolderType = FourCharCode('utmp'); { Contains temporary items created on behalf of the current user}
	kCachedDataFolderType = FourCharCode('cach'); { Contains various cache files for different clients}
	kFrameworksFolderType = FourCharCode('fram'); { Contains MacOS X Framework folders}
	kPrivateFrameworksFolderType = FourCharCode('pfrm'); { Contains MacOS X Private Framework folders     }
	kClassicDesktopFolderType = FourCharCode('sdsk'); { MacOS 9 compatible desktop folder - same as kSystemDesktopFolderType but with a more appropriate name for Mac OS X code.}
	kSystemSoundsFolderType = FourCharCode('ssnd'); { Contains Mac OS X System Sound Files ( valid in kSystemDomain, kLocalDomain, and kUserDomain )}
	kComponentsFolderType = FourCharCode('cmpd'); { Contains Mac OS X components   ( valid in kSystemDomain, kLocalDomain, and kUserDomain )}
	kQuickTimeComponentsFolderType = FourCharCode('wcmp'); { Contains QuickTime components for Mac OS X ( valid in kSystemDomain, kLocalDomain, and kUserDomain )}
	kCoreServicesFolderType = FourCharCode('csrv'); { Refers to the "/System/Library/CoreServices" folder on Mac OS X}
	kAudioSupportFolderType = FourCharCode('adio'); { Refers to the Audio support folder for Mac OS X}
	kAudioPresetsFolderType = FourCharCode('apst'); { "Presets" folder of "Audio" folder, Mac OS X 10.4 and later}
	kAudioSoundsFolderType = FourCharCode('asnd'); { Refers to the Sounds subfolder of Audio Support}
	kAudioSoundBanksFolderType = FourCharCode('bank'); { Refers to the Banks subfolder of the Sounds Folder}
	kAudioAlertSoundsFolderType = FourCharCode('alrt'); { Refers to the Alert Sounds subfolder of the Sound Folder}
	kAudioPlugInsFolderType = FourCharCode('aplg'); { Refers to the Plug-ins subfolder of the Audio Folder   }
	kAudioComponentsFolderType = FourCharCode('acmp'); { Refers to the Components subfolder of the Audio Plug-ins Folder    }
	kKernelExtensionsFolderType = FourCharCode('kext'); { Refers to the Kernel Extensions Folder on Mac OS X}
	kDirectoryServicesFolderType = FourCharCode('dsrv'); { Refers to the Directory Services folder on Mac OS X}
	kDirectoryServicesPlugInsFolderType = FourCharCode('dplg'); { Refers to the Directory Services Plug-Ins folder on Mac OS X }
	kInstallerReceiptsFolderType = FourCharCode('rcpt'); { Refers to the "Receipts" folder in Mac OS X}
	kFileSystemSupportFolderType = FourCharCode('fsys'); { Refers to the [domain]/Library/Filesystems folder in Mac OS X}
	kAppleShareSupportFolderType = FourCharCode('shar'); { Refers to the [domain]/Library/Filesystems/AppleShare folder in Mac OS X}
	kAppleShareAuthenticationFolderType = FourCharCode('auth'); { Refers to the [domain]/Library/Filesystems/AppleShare/Authentication folder in Mac OS X}
	kMIDIDriversFolderType = FourCharCode('midi'); { Refers to the MIDI Drivers folder on Mac OS X}
	kKeyboardLayoutsFolderType = FourCharCode('klay'); { Refers to the [domain]/Library/KeyboardLayouts folder in Mac OS X}
	kIndexFilesFolderType = FourCharCode('indx'); { Refers to the [domain]/Library/Indexes folder in Mac OS X}
	kFindByContentIndexesFolderType = FourCharCode('fbcx'); { Refers to the [domain]/Library/Indexes/FindByContent folder in Mac OS X}
	kManagedItemsFolderType = FourCharCode('mang'); { Refers to the Managed Items folder for Mac OS X }
	kBootTimeStartupItemsFolderType = FourCharCode('empz'); { Refers to the "StartupItems" folder of Mac OS X }
	kAutomatorWorkflowsFolderType = FourCharCode('flow'); { Automator Workflows folder }
	kAutosaveInformationFolderType = FourCharCode('asav'); { ~/Library/Autosaved Information/ folder, can be used to store autosave information for user's applications.  Available in Mac OS X 10.4 and later.  }
	kSpotlightSavedSearchesFolderType = FourCharCode('spot'); { Usually ~/Library/Saved Searches/; used by Finder and Nav/Cocoa panels to find saved Spotlight searches }
                                        { The following folder types are available in Mac OS X 10.5 and later }
	kSpotlightImportersFolderType = FourCharCode('simp'); { Folder for Spotlight importers, usually /Library/Spotlight/ or ~/Library/Spotlight, etc. }
	kSpotlightMetadataCacheFolderType = FourCharCode('scch'); { Folder for Spotlight metadata caches, for example: ~/Library/Caches/Metadata/ }
	kInputManagersFolderType = FourCharCode('inpt'); { InputManagers }
	kInputMethodsFolderType = FourCharCode('inpf'); { ../Library/Input Methods/ }
	kLibraryAssistantsFolderType = FourCharCode('astl'); { Refers to the [domain]/Library/Assistants folder}
	kAudioDigidesignFolderType = FourCharCode('adig'); { Refers to the Digidesign subfolder of the Audio Plug-ins folder}
	kAudioVSTFolderType = FourCharCode('avst'); { Refers to the VST subfolder of the Audio Plug-ins folder}
	kColorPickersFolderType = FourCharCode('cpkr'); { Refers to the ColorPickers folder}
	kCompositionsFolderType = FourCharCode('cmps'); { Refers to the Compositions folder}
	kFontCollectionsFolderType = FourCharCode('fncl'); { Refers to the FontCollections folder}
	kiMovieFolderType = FourCharCode('imov'); { Refers to the iMovie folder}
	kiMoviePlugInsFolderType = FourCharCode('impi'); { Refers to the Plug-ins subfolder of the iMovie Folder}
	kiMovieSoundEffectsFolderType = FourCharCode('imse'); { Refers to the Sound Effects subfolder of the iMovie Folder}
	kDownloadsFolderType = FourCharCode('down'); { Refers to the ~/Downloads folder}

const
	kColorSyncProfilesFolderType = FourCharCode('prof'); { for ColorSyncª Profiles }
	kApplicationSupportFolderType = FourCharCode('asup'); { third-party items and folders }
	kTextEncodingsFolderType = FourCharCode('Ätex'); { encoding tables }
	kPrinterDescriptionFolderType = FourCharCode('ppdf'); { new folder at root of System folder for printer descs. }
	kPrinterDriverFolderType = FourCharCode('Äprd'); { new folder at root of System folder for printer drivers }
	kScriptingAdditionsFolderType = FourCharCode('Äscr'); { at root of system folder }

const
	kClassicPreferencesFolderType = FourCharCode('cprf'); { "Classic" folder in ~/Library/ for redirected preference files. }

const
	kQuickLookFolderType = FourCharCode('qlck'); { The QuickLook folder, supported in Mac OS X SnowLeopard and later. }

const
>>>>>>> graemeg/cpstrnew
=======

const
	kClassicPreferencesFolderType = FourCharCode('cprf'); { "Classic" folder in ~/Library/ for redirected preference files. }

const
	kQuickLookFolderType = FourCharCode('qlck'); { The QuickLook folder, supported in Mac OS X SnowLeopard and later. }

const
>>>>>>> origin/cpstrnew
{    The following selectors really only make sense when used within the Classic environment on Mac OS X.}
	kSystemFolderType = FourCharCode('macs'); { the system folder }
	kSystemDesktopFolderType = FourCharCode('sdsk'); { the desktop folder at the root of the hard drive, never the redirected user desktop folder }
	kSystemTrashFolderType = FourCharCode('strs'); { the trash folder at the root of the drive, never the redirected user trash folder }
	kPrintMonitorDocsFolderType = FourCharCode('prnt'); { Print Monitor documents }
	kALMModulesFolderType = FourCharCode('walk'); { for Location Manager Module files except type 'thng' (within kExtensionFolderType) }
	kALMPreferencesFolderType = FourCharCode('trip'); { for Location Manager Preferences (within kPreferencesFolderType; contains kALMLocationsFolderType) }
	kALMLocationsFolderType = FourCharCode('fall'); { for Location Manager Locations (within kALMPreferencesFolderType) }
	kAppleExtrasFolderType = FourCharCode('aexÄ'); { for Apple Extras folder }
	kContextualMenuItemsFolderType = FourCharCode('cmnu'); { for Contextual Menu items }
	kMacOSReadMesFolderType = FourCharCode('morÄ'); { for MacOS ReadMes folder }
	kStartupFolderType = FourCharCode('strt'); { Finder objects (applications, documents, DAs, aliases, to...) to open at startup go here }
	kShutdownFolderType = FourCharCode('shdf'); { Finder objects (applications, documents, DAs, aliases, to...) to open at shutdown go here }
	kAppleMenuFolderType = FourCharCode('amnu'); { Finder objects to put into the Apple menu go here }
	kControlPanelFolderType = FourCharCode('ctrl'); { Control Panels go here (may contain INITs) }
	kSystemControlPanelFolderType = FourCharCode('sctl'); { System control panels folder - never the redirected one, always "Control Panels" inside the System Folder }
	kExtensionFolderType = FourCharCode('extn'); { System extensions go here }
=======
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSFindFolder( vRefNum: FSVolumeRefNum; folderType: OSType; createFolder: Boolean; var foundRef: FSRef ): OSErr; external name '_FSFindFolder';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{//}

{
 *  Folder types
 *  
 }
const
	kDesktopFolderType = FourCharCode('desk'); { the desktop folder; objects in this folder show on the desktop. }
	kTrashFolderType = FourCharCode('trsh'); { the trash folder; objects in this folder show up in the trash }
	kWhereToEmptyTrashFolderType = FourCharCode('empt'); { the "empty trash" folder; Finder starts empty from here down }
	kFontsFolderType = FourCharCode('font'); { Fonts go here }
	kPreferencesFolderType = FourCharCode('pref'); { preferences for applications go here }
	kSystemPreferencesFolderType = FourCharCode('sprf'); { the PreferencePanes folder, where Mac OS X Preference Panes go }
	kTemporaryFolderType = FourCharCode('temp'); {    On Mac OS X, each user has their own temporary items folder, and the Folder Manager attempts to set permissions of these}
                                        {    folders such that other users can not access the data inside.  On Mac OS X 10.4 and later the data inside the temporary}
                                        {    items folder is deleted at logout and at boot, but not otherwise.  Earlier version of Mac OS X would delete items inside}
                                        {    the temporary items folder after a period of inaccess.  You can ask for a temporary item in a specific domain or on a }
                                        {    particular volume by FSVolumeRefNum.  If you want a location for temporary items for a short time, then use either}
                                        {    ( kUserDomain, kkTemporaryFolderType ) or ( kSystemDomain, kTemporaryFolderType ).  The kUserDomain varient will always be}
                                        {    on the same volume as the user's home folder, while the kSystemDomain version will be on the same volume as /var/tmp/ ( and}
                                        {    will probably be on the local hard drive in case the user's home is a network volume ).  If you want a location for a temporary}
                                        {    file or folder to use for saving a document, especially if you want to use FSpExchangeFile() to implement a safe-save, then}
                                        {    ask for the temporary items folder on the same volume as the file you are safe saving.}
                                        {    However, be prepared for a failure to find a temporary folder in any domain or on any volume.  Some volumes may not have}
                                        {    a location for a temporary folder, or the permissions of the volume may be such that the Folder Manager can not return}
                                        {    a temporary folder for the volume.}
                                        {    If your application creates an item in a temporary items older you should delete that item as soon as it is not needed,}
                                        {    and certainly before your application exits, since otherwise the item is consuming disk space until the user logs out or}
                                        {    restarts.  Any items left inside a temporary items folder should be moved into a folder inside the Trash folder on the disk}
                                        {    when the user logs in, inside a folder named "Recovered items", in case there is anything useful to the end user.}
	kChewableItemsFolderType = FourCharCode('flnt'); { similar to kTemporaryItemsFolderType, except items in this folder are deleted at boot or when the disk is unmounted }
	kTemporaryItemsInCacheDataFolderType = FourCharCode('vtmp'); { A folder inside the kCachedDataFolderType for the given domain which can be used for transient data}
	kApplicationsFolderType = FourCharCode('apps'); {    Applications on Mac OS X are typically put in this folder ( or a subfolder ).}
	kVolumeRootFolderType = FourCharCode('root'); { root folder of a volume or domain }
	kDomainTopLevelFolderType = FourCharCode('dtop'); { The top-level of a Folder domain, e.g. "/System"}
	kDomainLibraryFolderType = FourCharCode('dlib'); { the Library subfolder of a particular domain}
	kUsersFolderType = FourCharCode('usrs'); { "Users" folder, usually contains one folder for each user. }
	kCurrentUserFolderType = FourCharCode('cusr'); { The folder for the currently logged on user; domain passed in is ignored. }
	kSharedUserDataFolderType = FourCharCode('sdat'); { A Shared folder, readable & writeable by all users }

{
    The following selectors refer specifically to subfolders inside the user's home folder, and should
    be used only with  kUserDomain as the domain in the various FindFolder calls.
}
const
	kDocumentsFolderType = FourCharCode('docs'); {    User documents are typically put in this folder ( or a subfolder ).}
	kPictureDocumentsFolderType = FourCharCode('pdoc'); { Refers to the "Pictures" folder in a users home directory}
	kMovieDocumentsFolderType = FourCharCode('mdoc'); { Refers to the "Movies" folder in a users home directory}
	kMusicDocumentsFolderType = FourCharCode('µdoc'); { Refers to the "Music" folder in a users home directory}
	kInternetSitesFolderType = FourCharCode('site'); { Refers to the "Sites" folder in a users home directory}
	kPublicFolderType = FourCharCode('pubb'); { Refers to the "Public" folder in a users home directory}

const
	kSharedLibrariesFolderType = FourCharCode('Älib'); { for general shared libs. }
	kVoicesFolderType = FourCharCode('fvoc'); { macintalk can live here }
	kUtilitiesFolderType = FourCharCode('utiÄ'); { for Utilities folder }
	kThemesFolderType = FourCharCode('thme'); { for Theme data files }
	kFavoritesFolderType = FourCharCode('favs'); { Favorties folder for Navigation Services }
	kInternetSearchSitesFolderType = FourCharCode('issf'); { Internet Search Sites folder }
	kInstallerLogsFolderType = FourCharCode('ilgf'); { Installer Logs folder }
	kScriptsFolderType = FourCharCode('scrÄ'); { Scripts folder }
	kFolderActionsFolderType = FourCharCode('fasf'); { Folder Actions Scripts folder }
	kSpeakableItemsFolderType = FourCharCode('spki'); { Speakable Items folder }
	kKeychainFolderType = FourCharCode('kchn'); { Keychain folder }

{ New Folder Types to accommodate the Mac OS X Folder Manager }
{ These folder types are not applicable on Mac OS 9.          }
const
	kColorSyncFolderType = FourCharCode('sync'); { Contains ColorSync-related folders}
	kColorSyncCMMFolderType = FourCharCode('ccmm'); { ColorSync CMMs}
	kColorSyncScriptingFolderType = FourCharCode('cscr'); { ColorSync Scripting support}
	kPrintersFolderType = FourCharCode('impr'); { Contains Printing-related folders}
	kSpeechFolderType = FourCharCode('spch'); { Contains Speech-related folders}
	kCarbonLibraryFolderType = FourCharCode('carb'); { Contains Carbon-specific file}
	kDocumentationFolderType = FourCharCode('info'); { Contains Documentation files (not user documents)}
	kISSDownloadsFolderType = FourCharCode('issd'); { Contains Internet Search Sites downloaded from the Internet}
	kUserSpecificTmpFolderType = FourCharCode('utmp'); { Contains temporary items created on behalf of the current user}
	kCachedDataFolderType = FourCharCode('cach'); { Contains various cache files for different clients}
	kFrameworksFolderType = FourCharCode('fram'); { Contains MacOS X Framework folders}
	kPrivateFrameworksFolderType = FourCharCode('pfrm'); { Contains MacOS X Private Framework folders     }
	kClassicDesktopFolderType = FourCharCode('sdsk'); { MacOS 9 compatible desktop folder - same as kSystemDesktopFolderType but with a more appropriate name for Mac OS X code.}
	kSystemSoundsFolderType = FourCharCode('ssnd'); { Contains Mac OS X System Sound Files ( valid in kSystemDomain, kLocalDomain, and kUserDomain )}
	kComponentsFolderType = FourCharCode('cmpd'); { Contains Mac OS X components   ( valid in kSystemDomain, kLocalDomain, and kUserDomain )}
	kQuickTimeComponentsFolderType = FourCharCode('wcmp'); { Contains QuickTime components for Mac OS X ( valid in kSystemDomain, kLocalDomain, and kUserDomain )}
	kCoreServicesFolderType = FourCharCode('csrv'); { Refers to the "/System/Library/CoreServices" folder on Mac OS X}
	kAudioSupportFolderType = FourCharCode('adio'); { Refers to the Audio support folder for Mac OS X}
	kAudioPresetsFolderType = FourCharCode('apst'); { "Presets" folder of "Audio" folder, Mac OS X 10.4 and later}
	kAudioSoundsFolderType = FourCharCode('asnd'); { Refers to the Sounds subfolder of Audio Support}
	kAudioSoundBanksFolderType = FourCharCode('bank'); { Refers to the Banks subfolder of the Sounds Folder}
	kAudioAlertSoundsFolderType = FourCharCode('alrt'); { Refers to the Alert Sounds subfolder of the Sound Folder}
	kAudioPlugInsFolderType = FourCharCode('aplg'); { Refers to the Plug-ins subfolder of the Audio Folder   }
	kAudioComponentsFolderType = FourCharCode('acmp'); { Refers to the Components subfolder of the Audio Plug-ins Folder    }
	kKernelExtensionsFolderType = FourCharCode('kext'); { Refers to the Kernel Extensions Folder on Mac OS X}
	kDirectoryServicesFolderType = FourCharCode('dsrv'); { Refers to the Directory Services folder on Mac OS X}
	kDirectoryServicesPlugInsFolderType = FourCharCode('dplg'); { Refers to the Directory Services Plug-Ins folder on Mac OS X }
	kInstallerReceiptsFolderType = FourCharCode('rcpt'); { Refers to the "Receipts" folder in Mac OS X}
	kFileSystemSupportFolderType = FourCharCode('fsys'); { Refers to the [domain]/Library/Filesystems folder in Mac OS X}
	kAppleShareSupportFolderType = FourCharCode('shar'); { Refers to the [domain]/Library/Filesystems/AppleShare folder in Mac OS X}
	kAppleShareAuthenticationFolderType = FourCharCode('auth'); { Refers to the [domain]/Library/Filesystems/AppleShare/Authentication folder in Mac OS X}
	kMIDIDriversFolderType = FourCharCode('midi'); { Refers to the MIDI Drivers folder on Mac OS X}
	kKeyboardLayoutsFolderType = FourCharCode('klay'); { Refers to the [domain]/Library/KeyboardLayouts folder in Mac OS X}
	kIndexFilesFolderType = FourCharCode('indx'); { Refers to the [domain]/Library/Indexes folder in Mac OS X}
	kFindByContentIndexesFolderType = FourCharCode('fbcx'); { Refers to the [domain]/Library/Indexes/FindByContent folder in Mac OS X}
	kManagedItemsFolderType = FourCharCode('mang'); { Refers to the Managed Items folder for Mac OS X }
	kBootTimeStartupItemsFolderType = FourCharCode('empz'); { Refers to the "StartupItems" folder of Mac OS X }
	kAutomatorWorkflowsFolderType = FourCharCode('flow'); { Automator Workflows folder }
	kAutosaveInformationFolderType = FourCharCode('asav'); { ~/Library/Autosaved Information/ folder, can be used to store autosave information for user's applications.  Available in Mac OS X 10.4 and later.  }
	kSpotlightSavedSearchesFolderType = FourCharCode('spot'); { Usually ~/Library/Saved Searches/; used by Finder and Nav/Cocoa panels to find saved Spotlight searches }
                                        { The following folder types are available in Mac OS X 10.5 and later }
	kSpotlightImportersFolderType = FourCharCode('simp'); { Folder for Spotlight importers, usually /Library/Spotlight/ or ~/Library/Spotlight, etc. }
	kSpotlightMetadataCacheFolderType = FourCharCode('scch'); { Folder for Spotlight metadata caches, for example: ~/Library/Caches/Metadata/ }
	kInputManagersFolderType = FourCharCode('inpt'); { InputManagers }
	kInputMethodsFolderType = FourCharCode('inpf'); { ../Library/Input Methods/ }
	kLibraryAssistantsFolderType = FourCharCode('astl'); { Refers to the [domain]/Library/Assistants folder}
	kAudioDigidesignFolderType = FourCharCode('adig'); { Refers to the Digidesign subfolder of the Audio Plug-ins folder}
	kAudioVSTFolderType = FourCharCode('avst'); { Refers to the VST subfolder of the Audio Plug-ins folder}
	kColorPickersFolderType = FourCharCode('cpkr'); { Refers to the ColorPickers folder}
	kCompositionsFolderType = FourCharCode('cmps'); { Refers to the Compositions folder}
	kFontCollectionsFolderType = FourCharCode('fncl'); { Refers to the FontCollections folder}
	kiMovieFolderType = FourCharCode('imov'); { Refers to the iMovie folder}
	kiMoviePlugInsFolderType = FourCharCode('impi'); { Refers to the Plug-ins subfolder of the iMovie Folder}
	kiMovieSoundEffectsFolderType = FourCharCode('imse'); { Refers to the Sound Effects subfolder of the iMovie Folder}
	kDownloadsFolderType = FourCharCode('down'); { Refers to the ~/Downloads folder}

const
	kColorSyncProfilesFolderType = FourCharCode('prof'); { for ColorSyncª Profiles }
	kApplicationSupportFolderType = FourCharCode('asup'); { third-party items and folders }
	kTextEncodingsFolderType = FourCharCode('Ätex'); { encoding tables }
	kPrinterDescriptionFolderType = FourCharCode('ppdf'); { new folder at root of System folder for printer descs. }
	kPrinterDriverFolderType = FourCharCode('Äprd'); { new folder at root of System folder for printer drivers }
	kScriptingAdditionsFolderType = FourCharCode('Äscr'); { at root of system folder }

const
	kClassicPreferencesFolderType = FourCharCode('cprf'); { "Classic" folder in ~/Library/ for redirected preference files. }

const
	kQuickLookFolderType = FourCharCode('qlck'); { The QuickLook folder, supported in Mac OS X SnowLeopard and later. }

const
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{    The following selectors really only make sense when used within the Classic environment on Mac OS X.}
	kSystemFolderType = FourCharCode('macs'); { the system folder }
	kSystemDesktopFolderType = FourCharCode('sdsk'); { the desktop folder at the root of the hard drive, never the redirected user desktop folder }
	kSystemTrashFolderType = FourCharCode('strs'); { the trash folder at the root of the drive, never the redirected user trash folder }
	kPrintMonitorDocsFolderType = FourCharCode('prnt'); { Print Monitor documents }
	kALMModulesFolderType = FourCharCode('walk'); { for Location Manager Module files except type 'thng' (within kExtensionFolderType) }
	kALMPreferencesFolderType = FourCharCode('trip'); { for Location Manager Preferences (within kPreferencesFolderType; contains kALMLocationsFolderType) }
	kALMLocationsFolderType = FourCharCode('fall'); { for Location Manager Locations (within kALMPreferencesFolderType) }
	kAppleExtrasFolderType = FourCharCode('aexÄ'); { for Apple Extras folder }
	kContextualMenuItemsFolderType = FourCharCode('cmnu'); { for Contextual Menu items }
	kMacOSReadMesFolderType = FourCharCode('morÄ'); { for MacOS ReadMes folder }
	kStartupFolderType = FourCharCode('strt'); { Finder objects (applications, documents, DAs, aliases, to...) to open at startup go here }
	kShutdownFolderType = FourCharCode('shdf'); { Finder objects (applications, documents, DAs, aliases, to...) to open at shutdown go here }
	kAppleMenuFolderType = FourCharCode('amnu'); { Finder objects to put into the Apple menu go here }
	kControlPanelFolderType = FourCharCode('ctrl'); { Control Panels go here (may contain INITs) }
	kSystemControlPanelFolderType = FourCharCode('sctl'); { System control panels folder - never the redirected one, always "Control Panels" inside the System Folder }
	kExtensionFolderType = FourCharCode('extn'); { System extensions go here }
>>>>>>> origin/fixes_2.4
	kExtensionDisabledFolderType = FourCharCode('extD');
	kControlPanelDisabledFolderType = FourCharCode('ctrD');
	kSystemExtensionDisabledFolderType = FourCharCode('macD');
	kStartupItemsDisabledFolderType = FourCharCode('strD');
	kShutdownItemsDisabledFolderType = FourCharCode('shdD');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
	kAssistantsFolderType = FourCharCode('astÄ'); { for Assistants (MacOS Setup Assistant, etc) }
	kStationeryFolderType = FourCharCode('odst'); { stationery }
	kOpenDocFolderType = FourCharCode('odod'); { OpenDoc root }
	kOpenDocShellPlugInsFolderType = FourCharCode('odsp'); { OpenDoc Shell Plug-Ins in OpenDoc folder }
	kEditorsFolderType = FourCharCode('oded'); { OpenDoc editors in MacOS Folder }
	kOpenDocEditorsFolderType = FourCharCode('Äodf'); { OpenDoc subfolder of Editors folder }
	kOpenDocLibrariesFolderType = FourCharCode('odlb'); { OpenDoc libraries folder }
	kGenEditorsFolderType = FourCharCode('Äedi'); { CKH general editors folder at root level of Sys folder }
	kHelpFolderType = FourCharCode('Ählp'); { CKH help folder currently at root of system folder }
	kInternetPlugInFolderType = FourCharCode('Änet'); { CKH internet plug ins for browsers and stuff }
	kModemScriptsFolderType = FourCharCode('Ämod'); { CKH modem scripts, get 'em OUT of the Extensions folder }
	kControlStripModulesFolderType = FourCharCode('sdev'); { CKH for control strip modules }
	kInternetFolderType = FourCharCode('intÄ'); { Internet folder (root level of startup volume) }
	kAppearanceFolderType = FourCharCode('appr'); { Appearance folder (root of system folder) }
	kSoundSetsFolderType = FourCharCode('snds'); { Sound Sets folder (in Appearance folder) }
	kDesktopPicturesFolderType = FourCharCode('dtpÄ'); { Desktop Pictures folder (in Appearance folder) }
	kFindSupportFolderType = FourCharCode('fnds'); { Find support folder }
	kRecentApplicationsFolderType = FourCharCode('rapp'); { Recent Applications folder }
	kRecentDocumentsFolderType = FourCharCode('rdoc'); { Recent Documents folder }
	kRecentServersFolderType = FourCharCode('rsvr'); { Recent Servers folder }
	kLauncherItemsFolderType = FourCharCode('laun'); { Launcher Items folder }
	kQuickTimeExtensionsFolderType = FourCharCode('qtex'); { QuickTime Extensions Folder (in Extensions folder) }
	kDisplayExtensionsFolderType = FourCharCode('dspl'); { Display Extensions Folder (in Extensions folder) }
	kMultiprocessingFolderType = FourCharCode('mpxf'); { Multiprocessing Folder (in Extensions folder) }
	kPrintingPlugInsFolderType = FourCharCode('pplg'); { Printing Plug-Ins Folder (in Extensions folder) }
	kAppleshareAutomountServerAliasesFolderType = FourCharCode('srvÄ'); { Appleshare puts volumes to automount inside this folder. }
	kVolumeSettingsFolderType = FourCharCode('vsfd'); { Volume specific user information goes here }
	kPreMacOS91ApplicationsFolderType = FourCharCode('Œpps'); { The "Applications" folder, pre Mac OS 9.1 }
	kPreMacOS91InstallerLogsFolderType = FourCharCode('”lgf'); { The "Installer Logs" folder, pre Mac OS 9.1 }
	kPreMacOS91AssistantsFolderType = FourCharCode('ŒstÄ'); { The "Assistants" folder, pre Mac OS 9.1 }
	kPreMacOS91UtilitiesFolderType = FourCharCode('ŸtiÄ'); { The "Utilities" folder, pre Mac OS 9.1 }
	kPreMacOS91AppleExtrasFolderType = FourCharCode('ŒexÄ'); { The "Apple Extras" folder, pre Mac OS 9.1 }
	kPreMacOS91MacOSReadMesFolderType = FourCharCode('µorÄ'); { The "Mac OS ReadMes" folder, pre Mac OS 9.1 }
	kPreMacOS91InternetFolderType = FourCharCode('”ntÄ'); { The "Internet" folder, pre Mac OS 9.1 }
	kPreMacOS91AutomountedServersFolderType = FourCharCode('§rvÄ'); { The "Servers" folder, pre Mac OS 9.1 }
	kPreMacOS91StationeryFolderType = FourCharCode('¿dst'); { The "Stationery" folder, pre Mac OS 9.1 }
	kLocalesFolderType = FourCharCode('Äloc'); { PKE for Locales folder }
	kFindByContentPluginsFolderType = FourCharCode('fbcp'); { Find By Content Plug-ins }
	kFindByContentFolderType = FourCharCode('fbcf'); { Find by content folder }

{  These folder types are not supported on Mac OS X at all and should be removed from your source code.}
const
	kMagicTemporaryItemsFolderType = FourCharCode('mtmp');
	kTemporaryItemsInUserDomainFolderType = FourCharCode('temq');
	kCurrentUserRemoteFolderLocation = FourCharCode('rusf'); { The remote folder for the currently logged on user }
	kCurrentUserRemoteFolderType = FourCharCode('rusr'); { The remote folder location for the currently logged on user }

{
   These folder types are deprecated in 10.5. The location of developer tools is no longer hard coded to "/Developer/" and 
   so these folder types work only when developer tools are installed at the default location.
}
const
	kDeveloperDocsFolderType = FourCharCode('ddoc'); { Deprecated in 10.5. Contains Developer Documentation files and folders}
	kDeveloperHelpFolderType = FourCharCode('devh'); { Deprecated in 10.5. Contains Developer Help related files}
	kDeveloperFolderType = FourCharCode('devf'); { Deprecated in 10.5. Contains MacOS X Developer Resources}
	kDeveloperApplicationsFolderType = FourCharCode('dapp'); { Deprecated in 10.5. Contains Developer Applications}

{ FolderDescFlags values }
const
	kCreateFolderAtBoot = $00000002;
	kCreateFolderAtBootBit = 1;
	kFolderCreatedInvisible = $00000004;
	kFolderCreatedInvisibleBit = 2;
	kFolderCreatedNameLocked = $00000008;
	kFolderCreatedNameLockedBit = 3;
	kFolderCreatedAdminPrivs = $00000010;
	kFolderCreatedAdminPrivsBit = 4;

const
	kFolderInUserFolder = $00000020;
	kFolderInUserFolderBit = 5;
	kFolderTrackedByAlias = $00000040;
	kFolderTrackedByAliasBit = 6;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
	kApplicationsFolderType		= FourCharCode('apps');
	kDocumentsFolderType		= FourCharCode('docs');

																{  new constants  }
	kVolumeRootFolderType		= FourCharCode('root');						{  root folder of a volume  }
	kChewableItemsFolderType	= FourCharCode('flnt');						{  items deleted at boot  }
	kApplicationSupportFolderType = FourCharCode('asup');						{  third-party items and folders  }
	kTextEncodingsFolderType	= FourCharCode('Ätex');						{  encoding tables  }
	kStationeryFolderType		= FourCharCode('odst');						{  stationery  }
	kOpenDocFolderType			= FourCharCode('odod');						{  OpenDoc root  }
	kOpenDocShellPlugInsFolderType = FourCharCode('odsp');					{  OpenDoc Shell Plug-Ins in OpenDoc folder  }
	kEditorsFolderType			= FourCharCode('oded');						{  OpenDoc editors in MacOS Folder  }
	kOpenDocEditorsFolderType	= FourCharCode('Äodf');						{  OpenDoc subfolder of Editors folder  }
	kOpenDocLibrariesFolderType	= FourCharCode('odlb');						{  OpenDoc libraries folder  }
	kGenEditorsFolderType		= FourCharCode('Äedi');						{  CKH general editors folder at root level of Sys folder  }
	kHelpFolderType				= FourCharCode('Ählp');						{  CKH help folder currently at root of system folder  }
	kInternetPlugInFolderType	= FourCharCode('Änet');						{  CKH internet plug ins for browsers and stuff  }
	kModemScriptsFolderType		= FourCharCode('Ämod');						{  CKH modem scripts, get 'em OUT of the Extensions folder  }
	kPrinterDescriptionFolderType = FourCharCode('ppdf');						{  CKH new folder at root of System folder for printer descs.  }
	kPrinterDriverFolderType	= FourCharCode('Äprd');						{  CKH new folder at root of System folder for printer drivers  }
	kScriptingAdditionsFolderType = FourCharCode('Äscr');						{  CKH at root of system folder  }
	kSharedLibrariesFolderType	= FourCharCode('Älib');						{  CKH for general shared libs.  }
	kVoicesFolderType			= FourCharCode('fvoc');						{  CKH macintalk can live here  }
	kControlStripModulesFolderType = FourCharCode('sdev');					{  CKH for control strip modules  }
	kAssistantsFolderType		= FourCharCode('astÄ');						{  SJF for Assistants (MacOS Setup Assistant, etc)  }
	kUtilitiesFolderType		= FourCharCode('utiÄ');						{  SJF for Utilities folder  }
	kAppleExtrasFolderType		= FourCharCode('aexÄ');						{  SJF for Apple Extras folder  }
	kContextualMenuItemsFolderType = FourCharCode('cmnu');					{  SJF for Contextual Menu items  }
	kMacOSReadMesFolderType		= FourCharCode('morÄ');						{  SJF for MacOS ReadMes folder  }
	kALMModulesFolderType		= FourCharCode('walk');						{  EAS for Location Manager Module files except type 'thng' (within kExtensionFolderType)  }
	kALMPreferencesFolderType	= FourCharCode('trip');						{  EAS for Location Manager Preferences (within kPreferencesFolderType; contains kALMLocationsFolderType)  }
	kALMLocationsFolderType		= FourCharCode('fall');						{  EAS for Location Manager Locations (within kALMPreferencesFolderType)  }
	kColorSyncProfilesFolderType = FourCharCode('prof');						{  for ColorSyncª Profiles  }
	kThemesFolderType			= FourCharCode('thme');						{  for Theme data files  }
	kFavoritesFolderType		= FourCharCode('favs');						{  Favorties folder for Navigation Services  }
	kInternetFolderType			= FourCharCode('intÄ');						{  Internet folder (root level of startup volume)  }
	kAppearanceFolderType		= FourCharCode('appr');						{  Appearance folder (root of system folder)  }
	kSoundSetsFolderType		= FourCharCode('snds');						{  Sound Sets folder (in Appearance folder)  }
	kDesktopPicturesFolderType	= FourCharCode('dtpÄ');						{  Desktop Pictures folder (in Appearance folder)  }
	kInternetSearchSitesFolderType = FourCharCode('issf');					{  Internet Search Sites folder  }
	kFindSupportFolderType		= FourCharCode('fnds');						{  Find support folder  }
	kFindByContentFolderType	= FourCharCode('fbcf');						{  Find by content folder  }
	kInstallerLogsFolderType	= FourCharCode('ilgf');						{  Installer Logs folder  }
	kScriptsFolderType			= FourCharCode('scrÄ');						{  Scripts folder  }
	kFolderActionsFolderType	= FourCharCode('fasf');						{  Folder Actions Scripts folder  }
	kLauncherItemsFolderType	= FourCharCode('laun');						{  Launcher Items folder  }
	kRecentApplicationsFolderType = FourCharCode('rapp');						{  Recent Applications folder  }
	kRecentDocumentsFolderType	= FourCharCode('rdoc');						{  Recent Documents folder  }
	kRecentServersFolderType	= FourCharCode('rsvr');						{  Recent Servers folder  }
	kSpeakableItemsFolderType	= FourCharCode('spki');						{  Speakable Items folder  }
	kKeychainFolderType			= FourCharCode('kchn');						{  Keychain folder  }
	kQuickTimeExtensionsFolderType = FourCharCode('qtex');					{  QuickTime Extensions Folder (in Extensions folder)  }
	kDisplayExtensionsFolderType = FourCharCode('dspl');						{  Display Extensions Folder (in Extensions folder)  }
	kMultiprocessingFolderType	= FourCharCode('mpxf');						{  Multiprocessing Folder (in Extensions folder)  }
	kPrintingPlugInsFolderType	= FourCharCode('pplg');						{  Printing Plug-Ins Folder (in Extensions folder)  }


	{	 New Folder Types to accommodate the Mac OS X Folder Manager 	}
	{	 These folder types are not applicable on Mac OS 9.          	}
	kDomainTopLevelFolderType	= FourCharCode('dtop');						{  The top-level of a Folder domain, e.g. "/System" }
	kDomainLibraryFolderType	= FourCharCode('dlib');						{  the Library subfolder of a particular domain }
	kColorSyncFolderType		= FourCharCode('sync');						{  Contains ColorSync-related folders }
	kColorSyncCMMFolderType		= FourCharCode('ccmm');						{  ColorSync CMMs }
	kColorSyncScriptingFolderType = FourCharCode('cscr');						{  ColorSync Scripting support }
	kPrintersFolderType			= FourCharCode('impr');						{  Contains Printing-related folders }
	kSpeechFolderType			= FourCharCode('spch');						{  Contains Speech-related folders }
	kCarbonLibraryFolderType	= FourCharCode('carb');						{  Contains Carbon-specific file }
	kDocumentationFolderType	= FourCharCode('info');						{  Contains Documentation files (not user documents) }
	kDeveloperDocsFolderType	= FourCharCode('ddoc');						{  Contains Developer Documentation files and folders }
	kDeveloperHelpFolderType	= FourCharCode('devh');						{  Contains Developer Help related files }
	kISSDownloadsFolderType		= FourCharCode('issd');						{  Contains Internet Search Sites downloaded from the Internet }
	kUserSpecificTmpFolderType	= FourCharCode('utmp');						{  Contains temporary items created on behalf of the current user }
	kCachedDataFolderType		= FourCharCode('cach');						{  Contains various cache files for different clients }
	kFrameworksFolderType		= FourCharCode('fram');						{  Contains MacOS X Framework folders      }
	kPrivateFrameworksFolderType = FourCharCode('pfrm');						{  Contains MacOS X Private Framework folders      }
	kClassicDesktopFolderType	= FourCharCode('sdsk');						{  MacOS 9 compatible desktop folder - same as  }
																{  kSystemDesktopFolderType but with a more appropriate }
																{  name for Mac OS X code. }
	kDeveloperFolderType		= FourCharCode('devf');						{  Contains MacOS X Developer Resources }
	kSystemSoundsFolderType		= FourCharCode('ssnd');						{  Contains Mac OS X System Sound Files }
	kComponentsFolderType		= FourCharCode('cmpd');						{  Contains Mac OS X components }
	kQuickTimeComponentsFolderType = FourCharCode('wcmp');					{  Contains QuickTime components for Mac OS X }
	kCoreServicesFolderType		= FourCharCode('csrv');						{  Refers to the "CoreServices" folder on Mac OS X }
	kPictureDocumentsFolderType	= FourCharCode('pdoc');						{  Refers to the "Pictures" folder in a users home directory }
	kMovieDocumentsFolderType	= FourCharCode('mdoc');						{  Refers to the "Movies" folder in a users home directory }
	kMusicDocumentsFolderType	= FourCharCode('µdoc');						{  Refers to the "Music" folder in a users home directory }
	kInternetSitesFolderType	= FourCharCode('site');						{  Refers to the "Sites" folder in a users home directory }
	kPublicFolderType			= FourCharCode('pubb');						{  Refers to the "Public" folder in a users home directory }
	kAudioSupportFolderType		= FourCharCode('adio');						{  Refers to the Audio support folder for Mac OS X }
	kAudioSoundsFolderType		= FourCharCode('asnd');						{  Refers to the Sounds subfolder of Audio Support }
	kAudioSoundBanksFolderType	= FourCharCode('bank');						{  Refers to the Banks subfolder of the Sounds Folder }
	kAudioAlertSoundsFolderType	= FourCharCode('alrt');						{  Refers to the Alert Sounds subfolder of the Sound Folder }
	kAudioPlugInsFolderType		= FourCharCode('aplg');						{  Refers to the Plug-ins subfolder of the Audio Folder    }
	kAudioComponentsFolderType	= FourCharCode('acmp');						{  Refers to the Components subfolder of the Audio Plug-ins Folder     }
	kKernelExtensionsFolderType	= FourCharCode('kext');						{  Refers to the Kernel Extensions Folder on Mac OS X }
	kDirectoryServicesFolderType = FourCharCode('dsrv');						{  Refers to the Directory Services folder on Mac OS X }
	kDirectoryServicesPlugInsFolderType = FourCharCode('dplg');				{  Refers to the Directory Services Plug-Ins folder on Mac OS X  }
	kInstallerReceiptsFolderType = FourCharCode('rcpt');						{  Refers to the "Receipts" folder in Mac OS X }
	kFileSystemSupportFolderType = FourCharCode('fsys');						{  Refers to the [domain]/Library/Filesystems folder in Mac OS X }
	kAppleShareSupportFolderType = FourCharCode('shar');						{  Refers to the [domain]/Library/Filesystems/AppleShare folder in Mac OS X }
	kAppleShareAuthenticationFolderType = FourCharCode('auth');				{  Refers to the [domain]/Library/Filesystems/AppleShare/Authentication folder in Mac OS X }
	kMIDIDriversFolderType		= FourCharCode('midi');						{  Refers to the MIDI Drivers folder on Mac OS X }

	kLocalesFolderType			= FourCharCode('Äloc');						{  PKE for Locales folder  }
	kFindByContentPluginsFolderType = FourCharCode('fbcp');					{  Find By Content Plug-ins  }

	kUsersFolderType			= FourCharCode('usrs');						{  "Users" folder, contains one folder for each user.  }
	kCurrentUserFolderType		= FourCharCode('cusr');						{  The folder for the currently logged on user.  }
	kCurrentUserRemoteFolderLocation = FourCharCode('rusf');					{  The remote folder for the currently logged on user  }
	kCurrentUserRemoteFolderType = FourCharCode('rusr');						{  The remote folder location for the currently logged on user  }
	kSharedUserDataFolderType	= FourCharCode('sdat');						{  A Shared "Documents" folder, readable & writeable by all users  }
	kVolumeSettingsFolderType	= FourCharCode('vsfd');						{  Volume specific user information goes here  }

	kAppleshareAutomountServerAliasesFolderType = FourCharCode('srvÄ');		{  Appleshare puts volumes to automount inside this folder.  }
	kPreMacOS91ApplicationsFolderType = FourCharCode('Œpps');					{  The "Applications" folder, pre Mac OS 9.1  }
	kPreMacOS91InstallerLogsFolderType = FourCharCode('”lgf');				{  The "Installer Logs" folder, pre Mac OS 9.1  }
	kPreMacOS91AssistantsFolderType = FourCharCode('ŒstÄ');					{  The "Assistants" folder, pre Mac OS 9.1  }
	kPreMacOS91UtilitiesFolderType = FourCharCode('ŸtiÄ');					{  The "Utilities" folder, pre Mac OS 9.1  }
	kPreMacOS91AppleExtrasFolderType = FourCharCode('ŒexÄ');					{  The "Apple Extras" folder, pre Mac OS 9.1  }
	kPreMacOS91MacOSReadMesFolderType = FourCharCode('µorÄ');					{  The "Mac OS ReadMes" folder, pre Mac OS 9.1  }
	kPreMacOS91InternetFolderType = FourCharCode('”ntÄ');						{  The "Internet" folder, pre Mac OS 9.1  }
	kPreMacOS91AutomountedServersFolderType = FourCharCode('§rvÄ');			{  The "Servers" folder, pre Mac OS 9.1  }
	kPreMacOS91StationeryFolderType = FourCharCode('¿dst');					{  The "Stationery" folder, pre Mac OS 9.1  }

	{	 FolderDescFlags values 	}
	kCreateFolderAtBoot			= $00000002;
	kCreateFolderAtBootBit		= 1;
	kFolderCreatedInvisible		= $00000004;
	kFolderCreatedInvisibleBit	= 2;
	kFolderCreatedNameLocked	= $00000008;
	kFolderCreatedNameLockedBit	= 3;
	kFolderCreatedAdminPrivs	= $00000010;
	kFolderCreatedAdminPrivsBit	= 4;

	kFolderInUserFolder			= $00000020;
	kFolderInUserFolderBit		= 5;
	kFolderTrackedByAlias		= $00000040;
	kFolderTrackedByAliasBit	= 6;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
	kFolderInRemoteUserFolderIfAvailable = $00000080;
	kFolderInRemoteUserFolderIfAvailableBit = 7;
	kFolderNeverMatchedInIdentifyFolder = $00000100;
	kFolderNeverMatchedInIdentifyFolderBit = 8;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kFolderMustStayOnSameVolume = $00000200;
	kFolderMustStayOnSameVolumeBit = 9;
	kFolderManagerFolderInMacOS9FolderIfMacOSXIsInstalledMask = $00000400;
	kFolderManagerFolderInMacOS9FolderIfMacOSXIsInstalledBit = 10;
	kFolderInLocalOrRemoteUserFolder = kFolderInUserFolder or kFolderInRemoteUserFolderIfAvailable;
	kFolderManagerNotCreatedOnRemoteVolumesBit = 11;
	kFolderManagerNotCreatedOnRemoteVolumesMask = 1 shl kFolderManagerNotCreatedOnRemoteVolumesBit;
	kFolderManagerNewlyCreatedFolderIsLocalizedBit = 12;
	kFolderManagerNewlyCreatedFolderShouldHaveDotLocalizedCreatedWithinMask = 1 shl kFolderManagerNewlyCreatedFolderIsLocalizedBit;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

type
	FolderDescFlags = UInt32;
{ FolderClass values }
const
	kRelativeFolder = FourCharCode('relf');
	kRedirectedRelativeFolder = FourCharCode('rrel');
	kSpecialFolder = FourCharCode('spcf');
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======

type
	FolderDescFlags = UInt32;
{ FolderClass values }
const
	kRelativeFolder = FourCharCode('relf');
	kRedirectedRelativeFolder = FourCharCode('rrel');
	kSpecialFolder = FourCharCode('spcf');
=======
=======

type
	FolderClass = OSType;
{ special folder locations }
const
	kBlessedFolder = FourCharCode('blsf');
	kRootFolder = FourCharCode('rotf');
>>>>>>> origin/cpstrnew

type
	FolderClass = OSType;
{ special folder locations }
const
<<<<<<< HEAD
	kBlessedFolder = FourCharCode('blsf');
	kRootFolder = FourCharCode('rotf');
>>>>>>> graemeg/cpstrnew
=======
	kCurrentUserFolderLocation = FourCharCode('cusf'); {    the magic 'Current User' folder location}
>>>>>>> origin/cpstrnew

type
	FolderClass = OSType;
{ special folder locations }
const
<<<<<<< HEAD
	kBlessedFolder = FourCharCode('blsf');
	kRootFolder = FourCharCode('rotf');
>>>>>>> graemeg/cpstrnew
=======
	kCurrentUserFolderLocation = FourCharCode('cusf'); {    the magic 'Current User' folder location}
>>>>>>> graemeg/cpstrnew

<<<<<<< HEAD
type
	FolderDescFlags = UInt32;
{ FolderClass values }
const
<<<<<<< HEAD
	kRelativeFolder = FourCharCode('relf');
	kRedirectedRelativeFolder = FourCharCode('rrel');
	kSpecialFolder = FourCharCode('spcf');
=======
	kCurrentUserFolderLocation = FourCharCode('cusf'); {    the magic 'Current User' folder location}
>>>>>>> graemeg/cpstrnew

<<<<<<< HEAD
type
	FolderClass = OSType;
{ special folder locations }
const
	kBlessedFolder = FourCharCode('blsf');
	kRootFolder = FourCharCode('rotf');
>>>>>>> graemeg/cpstrnew

<<<<<<< HEAD
type
	FolderClass = OSType;
{ special folder locations }
const
<<<<<<< HEAD
	kBlessedFolder = FourCharCode('blsf');
	kRootFolder = FourCharCode('rotf');
=======
	kCurrentUserFolderLocation = FourCharCode('cusf'); {    the magic 'Current User' folder location}
>>>>>>> graemeg/cpstrnew

const
	kCurrentUserFolderLocation = FourCharCode('cusf'); {    the magic 'Current User' folder location}

<<<<<<< HEAD

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
const
	kDictionariesFolderType = FourCharCode('dict'); { Dictionaries folder }
	kLogsFolderType = FourCharCode('logs'); { Logs folder }
	kPreferencePanesFolderType = FourCharCode('ppan'); { PreferencePanes folder, in .../Library/ }


=======
const
	kDictionariesFolderType = FourCharCode('dict'); { Dictionaries folder }
	kLogsFolderType = FourCharCode('logs'); { Logs folder }
	kPreferencePanesFolderType = FourCharCode('ppan'); { PreferencePanes folder, in .../Library/ }


>>>>>>> origin/cpstrnew
const
	kWidgetsFolderType = FourCharCode('wdgt'); { Dashboard Widgets folder, in system, local, and user domains  }
	kScreenSaversFolderType = FourCharCode('scrn'); { Screen Savers folder, in system, local, and user domains }

type
	FolderType = OSType;
	FolderLocation = OSType;

type
	FolderDesc = record
		descSize: Size;
		foldType: FolderType;
		flags: FolderDescFlags;
		foldClass: FolderClass;
		foldLocation: FolderType;
		badgeSignature: OSType;
		badgeType: OSType;
		reserved: UInt32;
		name: StrFileName;                   { Str63 on MacOS}
	end;
	FolderDescPtr = ^FolderDesc;

type
	RoutingFlags = UInt32;
	FolderRouting = record
		descSize: Size;
		fileType: OSType;
		routeFromFolder: FolderType;
		routeToFolder: FolderType;
		flags: RoutingFlags;
	end;
	FolderRoutingPtr = ^FolderRouting;


{
 *  AddFolderDescriptor()
 *  
 *  Summary:
 *    Copies the supplied information into a new folder descriptor
 *    entry in the system folder list. @discussion The
 *    AddFolderDescriptor function copies the supplied information into
 *    a new descriptor entry in the system folder list. You need to
 *    provide folder descriptors for each folder you wish the Folder
 *    Manager to be able to find via the function FindFolder. For
 *    example, a child folder located in a parent folder needs to have
 *    a descriptor created both for it and its parent folder, so that
 *    the child can be found. This function is supported under Mac OS 8
 *    and later. 
 *    On Mac OS X, folder descriptors added in one process are not
 *    visible in other processes.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    foldType:
 *      Pass a constant identifying the type of the folder you wish the
 *      Folder Manager to be able to find. See ÒFolder Type ConstantsÓ.
 *    
 *    flags:
 *      Set these flags to indicate whether a folder is created during
 *      startup, if the folder name is locked, and if the folder is
 *      created invisible; see ÒFolder Descriptor FlagsÓ.
 *    
 *    foldClass:
 *      Pass the class of the folder which you wish the Folder Manager
 *      to be able to find. The folder class determines how the
 *      foldLocation parameter is interpreted. See "Folder Descriptor
 *      Classes" for a discussion of relative and special folder
 *      classes.
 *    
 *    foldLocation:
 *      For a relative folder, specify the folder type of the parent
 *      folder of the target. For a special folder, specify the
 *      location of the folder; see ÒFolder Descriptor LocationsÓ.
 *    
 *    badgeSignature:
 *      Reserved. Pass 0.
 *    
 *    badgeType:
 *      Reserved. Pass 0.
 *    
 *    name:
 *      A string specifying the name of the desired folder. For
 *      relative folders, this is the exact name of the desired folder.
 *      For special folders, the actual target folder may have a
 *      different name than the name specified in the folder
 *      descriptor. For example, the System Folder is often given a
 *      different name, but it can still be located with FindFolder.
 *    
 *    replaceFlag:
 *      Pass a Boolean value indicating whether you wish to replace a
 *      folder descriptor that already exists for the specified folder
 *      type. If true , it replaces the folder descriptor for the
 *      specified folder type. If false , it does not replace the
 *      folder descriptor for the specified folder type.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FoldersLib 1.0 and later
 }
function AddFolderDescriptor( foldType: FolderType; flags: FolderDescFlags; foldClass: FolderClass; foldLocation: FolderLocation; badgeSignature: OSType; badgeType: OSType; const (*var*) name: StrFileName; replaceFlag: Boolean ): OSErr; external name '_AddFolderDescriptor';
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
 *  GetFolderTypes()
 *  
 *  Summary:
 *    Obtains the folder types contained in the global descriptor list.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    requestedTypeCount:
 *      Pass the number of FolderType values that can fit in the buffer
 *      pointed to by the theTypes parameter; see ÒFolder Type
 *      ConstantsÓ.
 *    
 *    totalTypeCount:
 *      Pass a pointer to an unsigned 32-bit integer value. On return,
 *      the value is set to the total number of FolderType values in
 *      the list. The totalTypeCount parameter may produce a value that
 *      is larger or smaller than that of the requestedTypeCount
 *      parameter. If totalTypeCount is equal to or smaller than the
 *      value passed in for requestedTypeCount and the value produced
 *      by the theTypes parameter is non-null, then all folder types
 *      were returned to the caller.
 *    
 *    theTypes:
 *      Pass a pointer to an array of FolderType values; see "Folder
 *      Type Constants". On return, the array contains the folder types
 *      for the installed descriptors. You can step through the array
 *      and call GetFolderDescriptor for each folder type. Pass null if
 *      you only want to know the number of descriptors installed in
 *      the systemÕs global list, rather than the actual folder types
 *      of those descriptors.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FoldersLib 1.0 and later
 }
function GetFolderTypes( requestedTypeCount: UInt32; var totalTypeCount: UInt32; var theTypes: FolderType ): OSErr; external name '_GetFolderTypes';
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
 *  RemoveFolderDescriptor()
 *  
 *  Summary:
 *    Deletes the specified folder descriptor entry from the system
 *    folder list.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    foldType:
 *      Pass a constant identifying the type of the folder for which
 *      you wish to remove a descriptor.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FoldersLib 1.0 and later
 }
function RemoveFolderDescriptor( foldType: FolderType ): OSErr; external name '_RemoveFolderDescriptor';
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
>>>>>>> origin/cpstrnew


{
 *  GetFolderNameUnicode()
 *  
 *  Summary:
 *    Obtains the name of the specified folder.
 *  
 *  Discussion:
 *    The GetFolderName function obtains the name of the folder in the
 *    folder descriptor, not the name of the folder on the disk. The
 *    names may differ for a few special folders such as the System
 *    Folder. For relative folders, however, the actual name is always
 *    returned. You typically do not need to call this function.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    vRefNum:
 *      Pass the volume reference number (or the constant kOnSystemDisk
 *      for the startup disk) of the volume containing the folder for
 *      which you wish the name to be identified.
 *    
 *    foldType:
 *      Pass a constant identifying the type of the folder for which
 *      you wish the name to be identified. See "Folder Type Constants".
 *    
 *    foundVRefNum:
 *      On return, a pointer to the volume reference number for the
 *      volume containing the folder specified in the foldType
 *      parameter.
 *    
 *    name:
 *      A pointer to an HFSUniStr255 which will contain the unicode
 *      name on return.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function GetFolderNameUnicode( vRefNum: FSVolumeRefNum; foldType: OSType; var foundVRefNum: FSVolumeRefNum; var name: HFSUniStr255 ): OSStatus; external name '_GetFolderNameUnicode';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_5, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew

{
 *  GetFolderNameUnicode()
 *  
 *  Summary:
 *    Obtains the name of the specified folder.
 *  
 *  Discussion:
 *    The GetFolderName function obtains the name of the folder in the
 *    folder descriptor, not the name of the folder on the disk. The
 *    names may differ for a few special folders such as the System
 *    Folder. For relative folders, however, the actual name is always
 *    returned. You typically do not need to call this function.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    vRefNum:
 *      Pass the volume reference number (or the constant kOnSystemDisk
 *      for the startup disk) of the volume containing the folder for
 *      which you wish the name to be identified.
 *    
 *    foldType:
 *      Pass a constant identifying the type of the folder for which
 *      you wish the name to be identified. See "Folder Type Constants".
 *    
 *    foundVRefNum:
 *      On return, a pointer to the volume reference number for the
 *      volume containing the folder specified in the foldType
 *      parameter.
 *    
 *    name:
 *      A pointer to an HFSUniStr255 which will contain the unicode
 *      name on return.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function GetFolderNameUnicode( vRefNum: FSVolumeRefNum; foldType: OSType; var foundVRefNum: FSVolumeRefNum; var name: HFSUniStr255 ): OSStatus; external name '_GetFolderNameUnicode';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

<<<<<<< HEAD
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


>>>>>>> origin/cpstrnew
{
 *  InvalidateFolderDescriptorCache()
 *  
 *  Summary:
 *    Invalidates any prior FindFolder results for the specified folder.
 *  
 *  Discussion:
 *    The InvalidateFolderDescriptorCache function searches to see if
 *    there is currently a cache of results from FindFolder calls on
 *    the specified folder. If so, it invalidates the cache from the
 *    previous calls to the FindFolder function in order to force the
 *    Folder Manager to reexamine the disk when FindFolder is called
 *    again on the specified directory ID or volume reference number.
 *    
 *    
 *    You should not normally need to call
 *    InvalidateFolderDescriptorCache.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    vRefNum:
 *      Pass the volume reference number (or the constant kOnSystemDisk
 *      for the startup disk) of the volume containing the folder for
 *      which you wish the descriptor cache to be invalidated. Pass 0
 *      to completely invalidate all folder cache information.
 *    
 *    dirID:
 *      Pass the directory ID number for the folder for which you wish
 *      the descriptor cache to be invalidated. Pass 0 to invalidate
 *      the cache for all folders on the specified disk.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FoldersLib 1.0 and later
 }
function InvalidateFolderDescriptorCache( vRefNum: FSVolumeRefNum; dirID: SInt32 ): OSErr; external name '_InvalidateFolderDescriptorCache';
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

{
 *  InvalidateFolderDescriptorCache()
 *  
 *  Summary:
 *    Invalidates any prior FindFolder results for the specified folder.
 *  
 *  Discussion:
 *    The InvalidateFolderDescriptorCache function searches to see if
 *    there is currently a cache of results from FindFolder calls on
 *    the specified folder. If so, it invalidates the cache from the
 *    previous calls to the FindFolder function in order to force the
 *    Folder Manager to reexamine the disk when FindFolder is called
 *    again on the specified directory ID or volume reference number.
 *    
 *    
 *    You should not normally need to call
 *    InvalidateFolderDescriptorCache.
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  IdentifyFolder()
 *  
 *  Summary:
 *    Obtains the folder type for the specified folder.
 *  
 *  Discussion:
 *    The folder type is identified for the folder specified by the
 *    vRefNum and dirID parameters, if such a folder exists. Note that
 *    IdentifyFolder may take several seconds to complete. Note also
 *    that if there are multiple folder descriptors that map to an
 *    individual folder, IdentifyFolder returns the folder type of only
 *    the first matching descriptor that it finds.
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    vRefNum:
 *      Pass the volume reference number (or the constant kOnSystemDisk
<<<<<<< HEAD
 *      for the startup disk) of the volume containing the folder for
 *      which you wish the descriptor cache to be invalidated. Pass 0
 *      to completely invalidate all folder cache information.
 *    
 *    dirID:
 *      Pass the directory ID number for the folder for which you wish
 *      the descriptor cache to be invalidated. Pass 0 to invalidate
 *      the cache for all folders on the specified disk.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FoldersLib 1.0 and later
 }
function InvalidateFolderDescriptorCache( vRefNum: FSVolumeRefNum; dirID: SInt32 ): OSErr; external name '_InvalidateFolderDescriptorCache';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew

{
 *  IdentifyFolder()
 *  
 *  Summary:
 *    Obtains the folder type for the specified folder.
 *  
 *  Discussion:
 *    The folder type is identified for the folder specified by the
 *    vRefNum and dirID parameters, if such a folder exists. Note that
 *    IdentifyFolder may take several seconds to complete. Note also
 *    that if there are multiple folder descriptors that map to an
 *    individual folder, IdentifyFolder returns the folder type of only
 *    the first matching descriptor that it finds.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    vRefNum:
 *      Pass the volume reference number (or the constant kOnSystemDisk
 *      for the startup disk) of the volume containing the folder whose
 *      type you wish to identify.
 *    
 *    dirID:
 *      Pass the directory ID number for the folder whose type you wish
 *      to identify.
 *    
 *    foldType:
 *      Pass a pointer to a value of type FolderType. On return, the
 *      value is set to the folder type of the folder with the
 *      specified vRefNum and dirID parameters; see "Folder Type
 *      Constants" for descriptions of possible values.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
function IdentifyFolder( vRefNum: FSVolumeRefNum; dirID: SInt32; var foldType: FolderType ): OSErr; external name '_IdentifyFolder';
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


{
 *  FSDetermineIfRefIsEnclosedByFolder()
 *  
 *  Summary:
 *    Determine whether the given FSRef is enclosed inside the given
 *    special folder type for the given domain.
 *  
 *  Discussion:
 *    This is a fairly fast call which can determine whether a given
 *    FSRef on disk is 'inside' the given special folder type for the
 *    given domain.  This call will be more efficient than the
 *    equivalent client code which walked up the file list, checking
 *    each parent with IdentifyFolder() to see if it matches. One use
 *    for this call is to determine if a given file or folder is inside
 *    the trash on a volume, with something like
 *    
 *    err = FSDetermineIfRefIsEnclosedByFolder( kOnAppropriateDisk,
 *    kTrashFolderType, & ref, & result );
 *    if ( err == noErr && result ) (
 *    //  FSRef is inside trash on the volume.<br> )
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    domainOrVRefNum:
 *      The domain or vRefNum to check.  You can also pass
 *      kOnAppropriateDisk to check whatever vRefNum is appropriate for
 *      the given FSRef, or the value 0 to check all vRefNums and
 *      domains.
 *    
 *    folderType:
 *      The folder type to check
 *    
 *    inRef:
 *      The FSRef to look for.
 *    
 *    outResult:
 *      If non-NULL, this will be filled in with true if the given
 *      FSRef is enclosed inside the special folder type for the given
 *      domain, or false otherwise.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function FSDetermineIfRefIsEnclosedByFolder( domainOrVRefNum: FSVolumeRefNum; folderType: OSType; const (*var*) inRef: FSRef; var outResult: Boolean ): OSErr; external name '_FSDetermineIfRefIsEnclosedByFolder';
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
 *      for the startup disk) of the volume containing the folder whose
 *      type you wish to identify.
 *    
 *    dirID:
 *      Pass the directory ID number for the folder whose type you wish
 *      to identify.
 *    
 *    foldType:
 *      Pass a pointer to a value of type FolderType. On return, the
 *      value is set to the folder type of the folder with the
 *      specified vRefNum and dirID parameters; see "Folder Type
 *      Constants" for descriptions of possible values.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
function IdentifyFolder( vRefNum: FSVolumeRefNum; dirID: SInt32; var foldType: FolderType ): OSErr; external name '_IdentifyFolder';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  FSDetermineIfRefIsEnclosedByFolder()
 *  
 *  Summary:
 *    Determine whether the given FSRef is enclosed inside the given
 *    special folder type for the given domain.
 *  
 *  Discussion:
 *    This is a fairly fast call which can determine whether a given
 *    FSRef on disk is 'inside' the given special folder type for the
 *    given domain.  This call will be more efficient than the
 *    equivalent client code which walked up the file list, checking
 *    each parent with IdentifyFolder() to see if it matches. One use
 *    for this call is to determine if a given file or folder is inside
 *    the trash on a volume, with something like
 *    
 *    err = FSDetermineIfRefIsEnclosedByFolder( kOnAppropriateDisk,
 *    kTrashFolderType, & ref, & result );
 *    if ( err == noErr && result ) (
 *    //  FSRef is inside trash on the volume.<br> )
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    domainOrVRefNum:
 *      The domain or vRefNum to check.  You can also pass
 *      kOnAppropriateDisk to check whatever vRefNum is appropriate for
 *      the given FSRef, or the value 0 to check all vRefNums and
 *      domains.
 *    
 *    folderType:
 *      The folder type to check
 *    
 *    inRef:
 *      The FSRef to look for.
 *    
 *    outResult:
 *      If non-NULL, this will be filled in with true if the given
 *      FSRef is enclosed inside the special folder type for the given
 *      domain, or false otherwise.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function FSDetermineIfRefIsEnclosedByFolder( domainOrVRefNum: FSVolumeRefNum; folderType: OSType; const (*var*) inRef: FSRef; var outResult: Boolean ): OSErr; external name '_FSDetermineIfRefIsEnclosedByFolder';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/cpstrnew


{
 *  DetermineIfPathIsEnclosedByFolder()
 *  
 *  Summary:
 *    Determine whether a file path is enclosed inside the given
 *    special folder type for the given domain.
 *  
 *  Discussion:
 *    This is a fairly fast call which can determine whether a given
 *    path on disk is 'inside' the given special folder type for the
 *    given domain.  This call will be more efficient than the
 *    equivalent client code which walked up the file list, checking
 *    each parent with IdentifyFolder() to see if it matches. One use
 *    for this call is to determine if a given file or folder is inside
 *    the trash on a volume, with something like
 *    
 *    err = DetermineIfPathIsEnclosedByFolder( kOnAppropriateDisk,
 *    kTrashFolderType, path, false, & result );
 *    if ( err == noErr && result ) (
 *    //  path is inside trash on the volume.<br> )
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    domainOrVRefNum:
 *      The domain or vRefNum to check.  You can also pass
 *      kOnAppropriateDisk to check whatever vRefNum is appropriate for
 *      the given path, or the value 0 to check all vRefNums and
 *      domains.
 *    
 *    folderType:
 *      The folder type to check
 *    
 *    utf8Path:
 *      A UTF-8 encoded path name for the file.
 *    
 *    pathIsRealPath:
 *      Pass true if utf8Path is guaranteed to be a real pathname, with
 *      no symlinks or relative pathname items. Pass false if the
 *      utf8Path may contain relative pathnames, or symlinks, or
 *      aliases, etc.
 *    
 *    outResult:
 *      If non-NULL, this will be filled in with true if the given path
 *      is enclosed inside the special folder type for the given
 *      domain, or false otherwise.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FoldersLib 1.0 and later
 }
function DetermineIfPathIsEnclosedByFolder( domainOrVRefNum: FSVolumeRefNum; folderType: OSType; utf8Path: ConstCStringPtr; pathIsRealPath: Boolean; var outResult: Boolean ): OSErr; external name '_DetermineIfPathIsEnclosedByFolder';
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


{$ifc not TARGET_CPU_64}
{
 *  FindFolderExtended()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FindFolder instead wherever possible.
 *  
 *  Summary:
 *    Obtains location information for system-related directories.
 *  
 *  Discussion:
 *    For the folder type on the particular volume (specified,
 *    respectively, in the folderType and vRefNum parameters), the
 *    FindFolder function returns the directory's volume reference
 *    number in the foundVRefNum parameter and its directory ID in the
 *    foundDirID parameter.
 *    
 *    The specified folder used for a given volume might be located on
 *    a different volume in future versions of system software;
 *    therefore, do not assume the volume that you specify in vRefNum
 *    and the volume returned in foundVRefNum will be the same.
 *     
 *    Specify a volume reference number (or the constant kOnSystemDisk
 *    for the startup disk) or one of the domain constants ( on Mac OS
 *    X ) in the vRefNum parameter.
 *    
 *    Specify a four-character folder type--or the constant that
 *    represents it--in the folderType parameter.
 *    
 *    Use the constant kCreateFolder in the createFolder parameter to
 *    tell FindFolder to create a directory if it does not already
 *    exist; otherwise, use the constant kDontCreateFolder. Directories
 *    inside the System Folder are created only if the System Folder
 *    directory exists. The FindFolder function will not create a
 *    System Folder directory even if you specify the kCreateFolder
 *    constant in the createFolder parameter.
 *    
 *    The FindFolder function returns a nonzero result code if the
 *    folder isn't found, and it can also return other file system
 *    errors reported by the File Manager or Memory Manager.
 *     FindFolderExtended() is equivalent to FindFolder() on Mac OS X.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    vRefNum:
 *      The volume reference number (or the constant kOnSystemDisk for
 *      the startup disk) or one of the domain constants ( like
 *      kUserDomain ) of the volume or domain in which you want to
 *      locate a directory.
 *    
 *    folderType:
 *      A four-character folder type, or a constant that represents the
 *      type, for the directory you want to find.
 *    
 *    createFolder:
 *      Pass the constant kCreateFolder in this parameter to create a
 *      directory if it does not already exist; otherwise, pass the
 *      constant kDontCreateFolder.
 *    
 *    foundVRefNum:
 *      The volume reference number, returned by FindFolder , for the
 *      volume containing the directory you specify in the folderType
 *      parameter.
 *    
 *    flags:
 *      The flags passed in which control extended behaviour
 *    
 *    data:
 *      Unique data which is interpreted differently depending on the
 *      passed in flags.
 *    
 *    foundDirID:
 *      The directory ID number, returned by FindFolder , for the
 *      directory you specify in the folderType parameter.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.0 and later
 }
function FindFolderExtended( vRefNum: FSVolumeRefNum; folderType: OSType; createFolder: Boolean; flags: UInt32; data: UnivPtr; var foundVRefNum: FSVolumeRefNum; var foundDirID: SInt32 ): OSErr; external name '_FindFolderExtended';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_3, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)
>>>>>>> origin/cpstrnew


{
 *  FSFindFolderExtended()   *** DEPRECATED ***
 *  
 *  Summary:
 *    FSFindFolderExtended returns an FSRef for certain system-related
 *    directories.
 *  
 *  Discussion:
 *    For the folder type on the particular volume (specified,
 *    respectively, in the folderType and vRefNum parameters), the
 *    FindFolder function returns the FSRef of that directory. 
 *     
 *    The specified folder used for a given volume might be located on
 *    a different volume in future versions of system software;
 *    therefore, do not assume the volume that you specify in vRefNum
 *    and the volume returned in the FSRef will be the same.
 *    
 *    Specify a volume reference number (or the constant kOnSystemDisk
 *    for the startup disk) or one of the domain constants ( on Mac OS
 *    X ) in the vRefNum parameter.
 *    
 *    Specify a four-character folder type--or the constant that
 *    represents it--in the folderType parameter.
 *    
 *    Use the constant kCreateFolder in the createFolder parameter to
 *    tell FindFolder to create a directory if it does not already
 *    exist; otherwise, use the constant kDontCreateFolder. Directories
 *    inside the System Folder are created only if the System Folder
 *    directory exists. The FindFolder function will not create a
 *    System Folder directory even if you specify the kCreateFolder
 *    constant in the createFolder parameter.
 *    
 *    The FindFolder function returns a nonzero result code if the
 *    folder isn't found, and it can also return other file system
 *    errors reported by the File Manager or Memory Manager.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    vRefNum:
 *      The volume reference number (or the constant kOnSystemDisk for
 *      the startup disk) or one of the domain constants ( like
 *      kUserDomain ) of the volume or domain in which you want to
 *      locate a directory.
 *    
 *    folderType:
 *      A four-character folder type, or a constant that represents the
 *      type, for the directory you want to find.
 *    
 *    createFolder:
 *      Pass the constant kCreateFolder in this parameter to create a
 *      directory if it does not already exist; otherwise, pass the
 *      constant kDontCreateFolder.
 *    
 *    flags:
 *      The flags passed in which control extended behaviour
 *    
 *    data:
 *      Unique data which is interpreted differently depending on the
 *      passed in flags.
 *    
 *    foundRef:
 *      The FSRef for the directory you specify on the volume or domain
 *      and folderType given.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSFindFolderExtended( vRefNum: FSVolumeRefNum; folderType: OSType; createFolder: Boolean; flags: UInt32; data: UnivPtr; var foundRef: FSRef ): OSErr; external name '_FSFindFolderExtended';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_3, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)
>>>>>>> origin/cpstrnew


{
 *  GetFolderDescriptor()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    GetFolderDescriptor is deprecated on Mac OS X.
 *  
 *  Summary:
 *    Obtains the folder descriptor information for the specified
 *    folder type from the global descriptor list.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    foldType:
 *      Pass a constant identifying the type of the folder for which
 *      you wish to get descriptor information. See "Folder Type
 *      Constants".
 *    
 *    descSize:
 *      Pass the size (in bytes) of the folder descriptor structure for
 *      which a pointer is passed in the foldDesc parameter. This value
 *      is needed in order to determine the version of the structure
 *      being used.
 *    
 *    foldDesc:
 *      Pass a pointer to a folder descriptor structure. On return, the
 *      folder descriptor structure contains information from the
 *      global descriptor list for the specified folder type.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FoldersLib 1.0 and later
 }
function GetFolderDescriptor( foldType: FolderType; descSize: Size; var foldDesc: FolderDesc ): OSErr; external name '_GetFolderDescriptor';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_3, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)
>>>>>>> origin/cpstrnew


{
 *  GetFolderName()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use GetFolderNameUnicode.
 *  
 *  Summary:
 *    Obtains the name of the specified folder.
 *  
 *  Discussion:
 *    The GetFolderName function obtains the name of the folder in the
 *    folder descriptor, not the name of the folder on the disk. The
 *    names may differ for a few special folders such as the System
 *    Folder. For relative folders, however, the actual name is always
 *    returned. You typically do not need to call this function.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    vRefNum:
 *      Pass the volume reference number (or the constant kOnSystemDisk
 *      for the startup disk) of the volume containing the folder for
 *      which you wish the name to be identified.
 *    
 *    foldType:
 *      Pass a constant identifying the type of the folder for which
 *      you wish the name to be identified. See "Folder Type Constants".
 *    
 *    foundVRefNum:
 *      On return, a pointer to the volume reference number for the
 *      volume containing the folder specified in the foldType
 *      parameter.
 *    
 *    name:
 *      On return, a string containing the title of the folder
 *      specified in the foldType and vRefNum parameters.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FoldersLib 1.0 and later
 }
function GetFolderName( vRefNum: FSVolumeRefNum; foldType: OSType; var foundVRefNum: FSVolumeRefNum; var name: StrFileName ): OSErr; external name '_GetFolderName';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/cpstrnew


{
 *  AddFolderRouting()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Folder Manager routing is deprecated on Mac OS X.  Do not rely on
 *    this feature in your application, because support for it will be
 *    removed in a future version of the OS.
 *  
 *  Summary:
 *    Adds a folder routing structure to the global routing list.
 *  
 *  Discussion:
 *    Your application can use the AddFolderRouting function to specify
 *    how the Finder routes a given file type. 
 *    Folder Routing is deprecated on Mac OS X at this time.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    fileType:
 *      Pass the OSType of the file to be routed.
 *    
 *    routeFromFolder:
 *      Pass the folder type of the "from" folder see "Folder Type
 *      Constants" for descriptions of possible values. An item dropped
 *      on the folder specified in this parameter will be routed to the
 *      folder specified in the routeToFolder parameter.
 *    
 *    routeToFolder:
 *      The folder type of the "to" folder see "Folder Type Constants"
 *      for descriptions of possible values.
 *    
 *    flags:
 *      Reserved for future use; pass 0.
 *    
 *    replaceFlag:
 *      Pass a Boolean value indicating whether you wish to replace a
 *      folder routing that already exists. If true , it replaces the
 *      folder to which the item is being routed. If false , it leaves
 *      the folder to which the item is being routed.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FoldersLib 1.0 and later
 }
function AddFolderRouting( fileType: OSType; routeFromFolder: FolderType; routeToFolder: FolderType; flags: RoutingFlags; replaceFlag: Boolean ): OSErr; external name '_AddFolderRouting';
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
 *  RemoveFolderRouting()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Folder Manager routing is deprecated on Mac OS X.  Do not rely on
 *    this feature in your application, because support for it will be
 *    removed in a future version of the OS.
 *  
 *  Summary:
 *    Deletes a folder routing structure from the global routing list.
 *  
 *  Discussion:
 *    Both the file type and the folder type specified must match those
 *    of an existing folder routing structure in the global routing
 *    list for the RemoveFolderRouting function to succeed.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    fileType:
 *      Pass the file type value contained in the folder routing
 *      structure to be removed.
 *    
 *    routeFromFolder:
 *      Pass the folder type of the "from" folder see "Folder Type
 *      Constants" for descriptions of possible values.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FoldersLib 1.0 and later
 }
function RemoveFolderRouting( fileType: OSType; routeFromFolder: FolderType ): OSErr; external name '_RemoveFolderRouting';
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
 *  FindFolderRouting()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Folder Manager routing is deprecated on Mac OS X.  Do not rely on
 *    this feature in your application, because support for it will be
 *    removed in a future version of the OS.
 *  
 *  Summary:
 *    Finds the destination folder from a matching folder routing
 *    structure for the specified file.
 *  
 *  Discussion:
 *    Both the file type and the folder type specified must match those
 *    of a folder routing structure in the global routing list for the
 *    FindFolderRouting function to succeed.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    fileType:
 *      Pass the file type specified in the appropriate folder routing
 *      structure for the file for which you wish to find a destination
 *      folder.
 *    
 *    routeFromFolder:
 *      Pass the folder type of the "from" folder for which you wish to
 *      find a "to" folder see "Folder Type Constants" for descriptions
 *      of possible values. An item dropped on the folder specified in
 *      this parameter will be routed to the folder specified in the
 *      routeToFolder parameter.
 *    
 *    routeToFolder:
 *      A pointer to a value of type FolderType. On return, the value
 *      is set to the folder type of the destination folder.
 *    
 *    flags:
 *      Reserved; pass 0.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FoldersLib 1.0 and later
 }
function FindFolderRouting( fileType: OSType; routeFromFolder: FolderType; var routeToFolder: FolderType; var flags: RoutingFlags ): OSErr; external name '_FindFolderRouting';
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
 *  GetFolderRoutings()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Folder Manager routing is deprecated on Mac OS X.  Do not rely on
 *    this feature in your application, because support for it will be
 *    removed in a future version of the OS.
 *  
 *  Summary:
 *    Obtains folder routing information from the global routing list.
 *  
 *  Discussion:
 *    The folder routing information in the global routing list
 *    determines how the Finder routes files.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    requestedRoutingCount:
 *      An unsigned 32-bit value. Pass the number of folder routing
 *      structures that can fit in the buffer pointed to by the
 *      theRoutings parameter.
 *    
 *    totalRoutingCount:
 *      A pointer to an unsigned 32-bit value. On return, the value is
 *      set to the number of folder routing structures in the global
 *      list. If this value is less than or equal to
 *      requestedRoutingCount , all folder routing structures were
 *      returned to the caller.
 *    
 *    routingSize:
 *      Pass the size (in bytes) of the FolderRouting structure.
 *    
 *    theRoutings:
 *      Pass a pointer to an array of FolderRouting structures. On
 *      return the structure(s) contain the requested routing
 *      information. You may pass null if you do not wish this
 *      information.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FoldersLib 1.0 and later
 }
function GetFolderRoutings( requestedRoutingCount: UInt32; var totalRoutingCount: UInt32; routingSize: Size; var theRoutings: FolderRouting ): OSErr; external name '_GetFolderRoutings';
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
 *  FSpDetermineIfSpecIsEnclosedByFolder()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSDetemineIfRefIsEnclosedByFolder
 *  
 *  Summary:
 *    Determine whether the given FSSpec is enclosed inside the given
 *    special folder type for the given domain.
 *  
 *  Discussion:
 *    This is a fairly fast call which can determine whether a given
 *    FSSpec on disk is 'inside' the given special folder type for the
 *    given domain.  This call will be more efficient than the
 *    equivalent client code which walked up the file list, checking
 *    each parent with IdentifyFolder() to see if it matches. One use
 *    for this call is to determine if a given file or folder is inside
 *    the trash on a volume, with something like
 *    
 *    err = FSpDetermineIfRefIsEnclosedByFolder( kOnAppropriateDisk,
 *    kTrashFolderType, & spec, & result );
 *    if ( err == noErr && result ) (
 *    //  FSSpec is inside trash on the volume.<br> )
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    domainOrVRefNum:
 *      The domain or vRefNum to check.  You can also pass
 *      kOnAppropriateDisk to check whatever vRefNum is appropriate for
 *      the given FSSpec, or the value 0 to check all vRefNums and
 *      domains.
 *    
 *    folderType:
 *      The folder type to check
 *    
 *    inSpec:
 *      The FSSpec to look for.
 *    
 *    outResult:
 *      If non-NULL, this will be filled in with true if the given
 *      FSSpec is enclosed inside the special folder type for the given
 *      domain, or false otherwise.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
<<<<<<< HEAD
 }
function FSpDetermineIfSpecIsEnclosedByFolder( domainOrVRefNum: FSVolumeRefNum; folderType: OSType; const (*var*) inSpec: FSSpec; var outResult: Boolean ): OSErr; external name '_FSpDetermineIfSpecIsEnclosedByFolder';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_4, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew


{$endc} {not TARGET_CPU_64}

type
	FolderManagerNotificationProcPtr = function( message: OSType; arg: UnivPtr; userRefCon: UnivPtr ): OSStatus;
	FolderManagerNotificationUPP = FolderManagerNotificationProcPtr;
{
 *  NewFolderManagerNotificationUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   available as macro/inline
<<<<<<< HEAD
 }
function NewFolderManagerNotificationUPP( userRoutine: FolderManagerNotificationProcPtr ): FolderManagerNotificationUPP; external name '_NewFolderManagerNotificationUPP';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew

{
 *  DisposeFolderManagerNotificationUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   available as macro/inline
<<<<<<< HEAD
 }
procedure DisposeFolderManagerNotificationUPP( userUPP: FolderManagerNotificationUPP ); external name '_DisposeFolderManagerNotificationUPP';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew

{
 *  InvokeFolderManagerNotificationUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeFolderManagerNotificationUPP( message: OSType; arg: UnivPtr; userRefCon: UnivPtr; userUPP: FolderManagerNotificationUPP ): OSStatus; external name '_InvokeFolderManagerNotificationUPP';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew

{$ifc not TARGET_CPU_64}
{
 *  FolderManagerRegisterNotificationProc()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    This function is deprecated on Mac OS X.
 *  
 *  Summary:
 *    Register a function to be called at certain times
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    notificationProc:
 *    
 *    refCon:
 *    
 *    options:
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.0 and later
 }
function FolderManagerRegisterNotificationProc( notificationProc: FolderManagerNotificationUPP; refCon: UnivPtr; options: UInt32 ): OSErr; external name '_FolderManagerRegisterNotificationProc';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_3, __IPHONE_NA, __IPHONE_NA) *)


{
 *  FolderManagerUnregisterNotificationProc()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    This function is deprecated on Mac OS X.
 *  
 *  Summary:
 *    Unregister a function to be called at certain times
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    notificationProc:
 *    
 *    refCon:
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.0 and later
 }
function FolderManagerUnregisterNotificationProc( notificationProc: FolderManagerNotificationUPP; refCon: UnivPtr ): OSErr; external name '_FolderManagerUnregisterNotificationProc';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_3, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)

>>>>>>> graemeg/cpstrnew

{
 *  FolderManagerUnregisterNotificationProc()   *** DEPRECATED ***
=======
 }
procedure DisposeFolderManagerNotificationUPP( userUPP: FolderManagerNotificationUPP ); external name '_DisposeFolderManagerNotificationUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeFolderManagerNotificationUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeFolderManagerNotificationUPP( message: OSType; arg: UnivPtr; userRefCon: UnivPtr; userUPP: FolderManagerNotificationUPP ): OSStatus; external name '_InvokeFolderManagerNotificationUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{$ifc not TARGET_CPU_64}
{
 *  FolderManagerRegisterNotificationProc()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
 *  
 *  Deprecated:
 *    This function is deprecated on Mac OS X.
 *  
 *  Summary:
<<<<<<< HEAD
 *    Unregister a function to be called at certain times
=======
 *    Register a function to be called at certain times
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    notificationProc:
 *    
 *    refCon:
<<<<<<< HEAD
=======
 *    
 *    options:
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.0 and later
 }
<<<<<<< HEAD
function FolderManagerUnregisterNotificationProc( notificationProc: FolderManagerNotificationUPP; refCon: UnivPtr ): OSErr; external name '_FolderManagerUnregisterNotificationProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)

<<<<<<< HEAD
{
 *  FolderManagerRegisterCallNotificationProcs()   *** DEPRECATED ***
=======
 }
function NewFolderManagerNotificationUPP( userRoutine: FolderManagerNotificationProcPtr ): FolderManagerNotificationUPP; external name '_NewFolderManagerNotificationUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeFolderManagerNotificationUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeFolderManagerNotificationUPP( userUPP: FolderManagerNotificationUPP ); external name '_DisposeFolderManagerNotificationUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeFolderManagerNotificationUPP()
 *  
 *  Availability:
=======
 }
function FSpDetermineIfSpecIsEnclosedByFolder( domainOrVRefNum: FSVolumeRefNum; folderType: OSType; const (*var*) inSpec: FSSpec; var outResult: Boolean ): OSErr; external name '_FSpDetermineIfSpecIsEnclosedByFolder';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{$endc} {not TARGET_CPU_64}

type
	FolderManagerNotificationProcPtr = function( message: OSType; arg: UnivPtr; userRefCon: UnivPtr ): OSStatus;
	FolderManagerNotificationUPP = FolderManagerNotificationProcPtr;
{
 *  NewFolderManagerNotificationUPP()
 *  
 *  Availability:
>>>>>>> origin/cpstrnew
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function InvokeFolderManagerNotificationUPP( message: OSType; arg: UnivPtr; userRefCon: UnivPtr; userUPP: FolderManagerNotificationUPP ): OSStatus; external name '_InvokeFolderManagerNotificationUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{$ifc not TARGET_CPU_64}
{
 *  FolderManagerRegisterNotificationProc()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
 *  
 *  Deprecated:
 *    This function is deprecated on Mac OS X.
 *  
 *  Summary:
<<<<<<< HEAD
 *    Call the registered Folder Manager notification procs.
=======
 *    Register a function to be called at certain times
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
<<<<<<< HEAD
 *    message:
 *    
 *    arg:
=======
 *    notificationProc:
 *    
 *    refCon:
>>>>>>> graemeg/cpstrnew
 *    
 *    options:
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.0 and later
 }
<<<<<<< HEAD
function FolderManagerRegisterCallNotificationProcs( message: OSType; arg: UnivPtr; options: UInt32 ): OSStatus; external name '_FolderManagerRegisterCallNotificationProcs';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_3, __IPHONE_NA, __IPHONE_NA) *)

=======

{
 *  FolderManagerRegisterCallNotificationProcs()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    This function is deprecated on Mac OS X.
 *  
 *  Summary:
 *    Call the registered Folder Manager notification procs.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    message:
 *    
 *    arg:
 *    
 *    options:
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.0 and later
 }
function FolderManagerRegisterCallNotificationProcs( message: OSType; arg: UnivPtr; options: UInt32 ): OSStatus; external name '_FolderManagerRegisterCallNotificationProcs';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)

>>>>>>> graemeg/cpstrnew
=======
function FolderManagerRegisterNotificationProc( notificationProc: FolderManagerNotificationUPP; refCon: UnivPtr; options: UInt32 ): OSErr; external name '_FolderManagerRegisterNotificationProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)
=======
function FolderManagerRegisterNotificationProc( notificationProc: FolderManagerNotificationUPP; refCon: UnivPtr; options: UInt32 ): OSErr; external name '_FolderManagerRegisterNotificationProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)

>>>>>>> graemeg/cpstrnew

{
 *  FolderManagerUnregisterNotificationProc()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    This function is deprecated on Mac OS X.
 *  
 *  Summary:
 *    Unregister a function to be called at certain times
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    notificationProc:
 *    
 *    refCon:
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.0 and later
 }
function FolderManagerUnregisterNotificationProc( notificationProc: FolderManagerNotificationUPP; refCon: UnivPtr ): OSErr; external name '_FolderManagerUnregisterNotificationProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)

<<<<<<< HEAD
{
 *  FolderManagerUnregisterNotificationProc()   *** DEPRECATED ***
=======
function NewFolderManagerNotificationUPP( userRoutine: FolderManagerNotificationProcPtr ): FolderManagerNotificationUPP; external name '_NewFolderManagerNotificationUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeFolderManagerNotificationUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeFolderManagerNotificationUPP( userUPP: FolderManagerNotificationUPP ); external name '_DisposeFolderManagerNotificationUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeFolderManagerNotificationUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeFolderManagerNotificationUPP( message: OSType; arg: UnivPtr; userRefCon: UnivPtr; userUPP: FolderManagerNotificationUPP ): OSStatus; external name '_InvokeFolderManagerNotificationUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{$ifc not TARGET_CPU_64}
{
 *  FolderManagerRegisterNotificationProc()   *** DEPRECATED ***
>>>>>>> origin/cpstrnew
 *  
 *  Deprecated:
 *    This function is deprecated on Mac OS X.
 *  
 *  Summary:
<<<<<<< HEAD
 *    Unregister a function to be called at certain times
=======
 *    Register a function to be called at certain times
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    notificationProc:
 *    
 *    refCon:
<<<<<<< HEAD
=======
 *    
 *    options:
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.0 and later
 }
<<<<<<< HEAD
function FolderManagerUnregisterNotificationProc( notificationProc: FolderManagerNotificationUPP; refCon: UnivPtr ): OSErr; external name '_FolderManagerUnregisterNotificationProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


{
 *  FolderManagerRegisterCallNotificationProcs()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    This function is deprecated on Mac OS X.
 *  
 *  Summary:
 *    Call the registered Folder Manager notification procs.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    message:
 *    
 *    arg:
 *    
 *    options:
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.0 and later
 }
function FolderManagerRegisterCallNotificationProcs( message: OSType; arg: UnivPtr; options: UInt32 ): OSStatus; external name '_FolderManagerRegisterCallNotificationProcs';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)

>>>>>>> graemeg/cpstrnew

{$endc} {not TARGET_CPU_64}

=======
=======
function FolderManagerRegisterNotificationProc( notificationProc: FolderManagerNotificationUPP; refCon: UnivPtr; options: UInt32 ): OSErr; external name '_FolderManagerRegisterNotificationProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


{
 *  FolderManagerUnregisterNotificationProc()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    This function is deprecated on Mac OS X.
 *  
 *  Summary:
 *    Unregister a function to be called at certain times
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    notificationProc:
 *    
 *    refCon:
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.0 and later
 }
function FolderManagerUnregisterNotificationProc( notificationProc: FolderManagerNotificationUPP; refCon: UnivPtr ): OSErr; external name '_FolderManagerUnregisterNotificationProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)

>>>>>>> origin/cpstrnew

{
 *  FolderManagerRegisterCallNotificationProcs()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    This function is deprecated on Mac OS X.
 *  
 *  Summary:
 *    Call the registered Folder Manager notification procs.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    message:
 *    
 *    arg:
 *    
 *    options:
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.0 and later
 }
function FolderManagerRegisterCallNotificationProcs( message: OSType; arg: UnivPtr; options: UInt32 ): OSStatus; external name '_FolderManagerRegisterCallNotificationProcs';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)
<<<<<<< HEAD
=======

>>>>>>> origin/cpstrnew

{$endc} {not TARGET_CPU_64}

<<<<<<< HEAD
{$endc} {not TARGET_CPU_64}

>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$endc} {TARGET_OS_MAC}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
	kFolderMustStayOnSameVolume	= $00000200;
=======
	kFolderMustStayOnSameVolume = $00000200;
>>>>>>> origin/fixes_2.4
	kFolderMustStayOnSameVolumeBit = 9;
	kFolderManagerFolderInMacOS9FolderIfMacOSXIsInstalledMask = $00000400;
	kFolderManagerFolderInMacOS9FolderIfMacOSXIsInstalledBit = 10;
	kFolderInLocalOrRemoteUserFolder = kFolderInUserFolder or kFolderInRemoteUserFolderIfAvailable;
	kFolderManagerNotCreatedOnRemoteVolumesBit = 11;
	kFolderManagerNotCreatedOnRemoteVolumesMask = 1 shl kFolderManagerNotCreatedOnRemoteVolumesBit;
	kFolderManagerNewlyCreatedFolderIsLocalizedBit = 12;
	kFolderManagerNewlyCreatedFolderShouldHaveDotLocalizedCreatedWithinMask = 1 shl kFolderManagerNewlyCreatedFolderIsLocalizedBit;

type
	FolderDescFlags = UInt32;
{ FolderClass values }
const
	kRelativeFolder = FourCharCode('relf');
	kRedirectedRelativeFolder = FourCharCode('rrel');
	kSpecialFolder = FourCharCode('spcf');

type
	FolderClass = OSType;
{ special folder locations }
const
	kBlessedFolder = FourCharCode('blsf');
	kRootFolder = FourCharCode('rotf');

const
	kCurrentUserFolderLocation = FourCharCode('cusf'); {    the magic 'Current User' folder location}


const
	kDictionariesFolderType = FourCharCode('dict'); { Dictionaries folder }
	kLogsFolderType = FourCharCode('logs'); { Logs folder }
	kPreferencePanesFolderType = FourCharCode('ppan'); { PreferencePanes folder, in .../Library/ }


const
	kWidgetsFolderType = FourCharCode('wdgt'); { Dashboard Widgets folder, in system, local, and user domains  }
	kScreenSaversFolderType = FourCharCode('scrn'); { Screen Savers folder, in system, local, and user domains }

type
	FolderType = OSType;
	FolderLocation = OSType;

type
	FolderDesc = record
		descSize: Size;
		foldType: FolderType;
		flags: FolderDescFlags;
		foldClass: FolderClass;
		foldLocation: FolderType;
		badgeSignature: OSType;
		badgeType: OSType;
		reserved: UInt32;
		name: StrFileName;                   { Str63 on MacOS}
	end;
	FolderDescPtr = ^FolderDesc;

type
	RoutingFlags = UInt32;
	FolderRouting = record
		descSize: Size;
		fileType: OSType;
		routeFromFolder: FolderType;
		routeToFolder: FolderType;
		flags: RoutingFlags;
	end;
	FolderRoutingPtr = ^FolderRouting;


{
 *  AddFolderDescriptor()
 *  
 *  Summary:
 *    Copies the supplied information into a new folder descriptor
 *    entry in the system folder list. @discussion The
 *    AddFolderDescriptor function copies the supplied information into
 *    a new descriptor entry in the system folder list. You need to
 *    provide folder descriptors for each folder you wish the Folder
 *    Manager to be able to find via the function FindFolder. For
 *    example, a child folder located in a parent folder needs to have
 *    a descriptor created both for it and its parent folder, so that
 *    the child can be found. This function is supported under Mac OS 8
 *    and later. 
 *    On Mac OS X, folder descriptors added in one process are not
 *    visible in other processes.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    foldType:
 *      Pass a constant identifying the type of the folder you wish the
 *      Folder Manager to be able to find. See ÒFolder Type ConstantsÓ.
 *    
 *    flags:
 *      Set these flags to indicate whether a folder is created during
 *      startup, if the folder name is locked, and if the folder is
 *      created invisible; see ÒFolder Descriptor FlagsÓ.
 *    
 *    foldClass:
 *      Pass the class of the folder which you wish the Folder Manager
 *      to be able to find. The folder class determines how the
 *      foldLocation parameter is interpreted. See "Folder Descriptor
 *      Classes" for a discussion of relative and special folder
 *      classes.
 *    
 *    foldLocation:
 *      For a relative folder, specify the folder type of the parent
 *      folder of the target. For a special folder, specify the
 *      location of the folder; see ÒFolder Descriptor LocationsÓ.
 *    
 *    badgeSignature:
 *      Reserved. Pass 0.
 *    
 *    badgeType:
 *      Reserved. Pass 0.
 *    
 *    name:
 *      A string specifying the name of the desired folder. For
 *      relative folders, this is the exact name of the desired folder.
 *      For special folders, the actual target folder may have a
 *      different name than the name specified in the folder
 *      descriptor. For example, the System Folder is often given a
 *      different name, but it can still be located with FindFolder.
 *    
 *    replaceFlag:
 *      Pass a Boolean value indicating whether you wish to replace a
 *      folder descriptor that already exists for the specified folder
 *      type. If true , it replaces the folder descriptor for the
 *      specified folder type. If false , it does not replace the
 *      folder descriptor for the specified folder type.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FoldersLib 1.0 and later
 }
function AddFolderDescriptor( foldType: FolderType; flags: FolderDescFlags; foldClass: FolderClass; foldLocation: FolderLocation; badgeSignature: OSType; badgeType: OSType; const (*var*) name: StrFileName; replaceFlag: Boolean ): OSErr; external name '_AddFolderDescriptor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetFolderTypes()
 *  
 *  Summary:
 *    Obtains the folder types contained in the global descriptor list.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    requestedTypeCount:
 *      Pass the number of FolderType values that can fit in the buffer
 *      pointed to by the theTypes parameter; see ÒFolder Type
 *      ConstantsÓ.
 *    
 *    totalTypeCount:
 *      Pass a pointer to an unsigned 32-bit integer value. On return,
 *      the value is set to the total number of FolderType values in
 *      the list. The totalTypeCount parameter may produce a value that
 *      is larger or smaller than that of the requestedTypeCount
 *      parameter. If totalTypeCount is equal to or smaller than the
 *      value passed in for requestedTypeCount and the value produced
 *      by the theTypes parameter is non-null, then all folder types
 *      were returned to the caller.
 *    
 *    theTypes:
 *      Pass a pointer to an array of FolderType values; see "Folder
 *      Type Constants". On return, the array contains the folder types
 *      for the installed descriptors. You can step through the array
 *      and call GetFolderDescriptor for each folder type. Pass null if
 *      you only want to know the number of descriptors installed in
 *      the systemÕs global list, rather than the actual folder types
 *      of those descriptors.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FoldersLib 1.0 and later
 }
function GetFolderTypes( requestedTypeCount: UInt32; var totalTypeCount: UInt32; var theTypes: FolderType ): OSErr; external name '_GetFolderTypes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  RemoveFolderDescriptor()
 *  
 *  Summary:
 *    Deletes the specified folder descriptor entry from the system
 *    folder list.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    foldType:
 *      Pass a constant identifying the type of the folder for which
 *      you wish to remove a descriptor.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FoldersLib 1.0 and later
 }
function RemoveFolderDescriptor( foldType: FolderType ): OSErr; external name '_RemoveFolderDescriptor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetFolderNameUnicode()
 *  
 *  Summary:
 *    Obtains the name of the specified folder.
 *  
 *  Discussion:
 *    The GetFolderName function obtains the name of the folder in the
 *    folder descriptor, not the name of the folder on the disk. The
 *    names may differ for a few special folders such as the System
 *    Folder. For relative folders, however, the actual name is always
 *    returned. You typically do not need to call this function.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    vRefNum:
 *      Pass the volume reference number (or the constant kOnSystemDisk
 *      for the startup disk) of the volume containing the folder for
 *      which you wish the name to be identified.
 *    
 *    foldType:
 *      Pass a constant identifying the type of the folder for which
 *      you wish the name to be identified. See "Folder Type Constants".
 *    
 *    foundVRefNum:
 *      On return, a pointer to the volume reference number for the
 *      volume containing the folder specified in the foldType
 *      parameter.
 *    
 *    name:
 *      A pointer to an HFSUniStr255 which will contain the unicode
 *      name on return.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function GetFolderNameUnicode( vRefNum: FSVolumeRefNum; foldType: OSType; var foundVRefNum: FSVolumeRefNum; var name: HFSUniStr255 ): OSStatus; external name '_GetFolderNameUnicode';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  InvalidateFolderDescriptorCache()
 *  
 *  Summary:
 *    Invalidates any prior FindFolder results for the specified folder.
 *  
 *  Discussion:
 *    The InvalidateFolderDescriptorCache function searches to see if
 *    there is currently a cache of results from FindFolder calls on
 *    the specified folder. If so, it invalidates the cache from the
 *    previous calls to the FindFolder function in order to force the
 *    Folder Manager to reexamine the disk when FindFolder is called
 *    again on the specified directory ID or volume reference number.
 *    
 *    
 *    You should not normally need to call
 *    InvalidateFolderDescriptorCache.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    vRefNum:
 *      Pass the volume reference number (or the constant kOnSystemDisk
 *      for the startup disk) of the volume containing the folder for
 *      which you wish the descriptor cache to be invalidated. Pass 0
 *      to completely invalidate all folder cache information.
 *    
 *    dirID:
 *      Pass the directory ID number for the folder for which you wish
 *      the descriptor cache to be invalidated. Pass 0 to invalidate
 *      the cache for all folders on the specified disk.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FoldersLib 1.0 and later
 }
function InvalidateFolderDescriptorCache( vRefNum: FSVolumeRefNum; dirID: SInt32 ): OSErr; external name '_InvalidateFolderDescriptorCache';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  IdentifyFolder()
 *  
 *  Summary:
 *    Obtains the folder type for the specified folder.
 *  
 *  Discussion:
 *    The folder type is identified for the folder specified by the
 *    vRefNum and dirID parameters, if such a folder exists. Note that
 *    IdentifyFolder may take several seconds to complete. Note also
 *    that if there are multiple folder descriptors that map to an
 *    individual folder, IdentifyFolder returns the folder type of only
 *    the first matching descriptor that it finds.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    vRefNum:
 *      Pass the volume reference number (or the constant kOnSystemDisk
 *      for the startup disk) of the volume containing the folder whose
 *      type you wish to identify.
 *    
 *    dirID:
 *      Pass the directory ID number for the folder whose type you wish
 *      to identify.
 *    
 *    foldType:
 *      Pass a pointer to a value of type FolderType. On return, the
 *      value is set to the folder type of the folder with the
 *      specified vRefNum and dirID parameters; see "Folder Type
 *      Constants" for descriptions of possible values.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
function IdentifyFolder( vRefNum: FSVolumeRefNum; dirID: SInt32; var foldType: FolderType ): OSErr; external name '_IdentifyFolder';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  FSDetermineIfRefIsEnclosedByFolder()
 *  
 *  Summary:
 *    Determine whether the given FSRef is enclosed inside the given
 *    special folder type for the given domain.
 *  
 *  Discussion:
 *    This is a fairly fast call which can determine whether a given
 *    FSRef on disk is 'inside' the given special folder type for the
 *    given domain.  This call will be more efficient than the
 *    equivalent client code which walked up the file list, checking
 *    each parent with IdentifyFolder() to see if it matches. One use
 *    for this call is to determine if a given file or folder is inside
 *    the trash on a volume, with something like
 *    
 *    err = FSDetermineIfRefIsEnclosedByFolder( kOnAppropriateDisk,
 *    kTrashFolderType, & ref, & result );
 *    if ( err == noErr && result ) (
 *    //  FSRef is inside trash on the volume.<br> )
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    domainOrVRefNum:
 *      The domain or vRefNum to check.  You can also pass
 *      kOnAppropriateDisk to check whatever vRefNum is appropriate for
 *      the given FSRef, or the value 0 to check all vRefNums and
 *      domains.
 *    
 *    folderType:
 *      The folder type to check
 *    
 *    inRef:
 *      The FSRef to look for.
 *    
 *    outResult:
 *      If non-NULL, this will be filled in with true if the given
 *      FSRef is enclosed inside the special folder type for the given
 *      domain, or false otherwise.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function FSDetermineIfRefIsEnclosedByFolder( domainOrVRefNum: FSVolumeRefNum; folderType: OSType; const (*var*) inRef: FSRef; var outResult: Boolean ): OSErr; external name '_FSDetermineIfRefIsEnclosedByFolder';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  DetermineIfPathIsEnclosedByFolder()
 *  
 *  Summary:
 *    Determine whether a file path is enclosed inside the given
 *    special folder type for the given domain.
 *  
 *  Discussion:
 *    This is a fairly fast call which can determine whether a given
 *    path on disk is 'inside' the given special folder type for the
 *    given domain.  This call will be more efficient than the
 *    equivalent client code which walked up the file list, checking
 *    each parent with IdentifyFolder() to see if it matches. One use
 *    for this call is to determine if a given file or folder is inside
 *    the trash on a volume, with something like
 *    
 *    err = DetermineIfPathIsEnclosedByFolder( kOnAppropriateDisk,
 *    kTrashFolderType, path, false, & result );
 *    if ( err == noErr && result ) (
 *    //  path is inside trash on the volume.<br> )
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    domainOrVRefNum:
 *      The domain or vRefNum to check.  You can also pass
 *      kOnAppropriateDisk to check whatever vRefNum is appropriate for
 *      the given path, or the value 0 to check all vRefNums and
 *      domains.
 *    
 *    folderType:
 *      The folder type to check
 *    
 *    utf8Path:
 *      A UTF-8 encoded path name for the file.
 *    
 *    pathIsRealPath:
 *      Pass true if utf8Path is guaranteed to be a real pathname, with
 *      no symlinks or relative pathname items. Pass false if the
 *      utf8Path may contain relative pathnames, or symlinks, or
 *      aliases, etc.
 *    
 *    outResult:
 *      If non-NULL, this will be filled in with true if the given path
 *      is enclosed inside the special folder type for the given
 *      domain, or false otherwise.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FoldersLib 1.0 and later
 }
function DetermineIfPathIsEnclosedByFolder( domainOrVRefNum: FSVolumeRefNum; folderType: OSType; utf8Path: ConstCStringPtr; pathIsRealPath: Boolean; var outResult: Boolean ): OSErr; external name '_DetermineIfPathIsEnclosedByFolder';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{$ifc not TARGET_CPU_64}
{
 *  FindFolderExtended()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FindFolder instead wherever possible.
 *  
 *  Summary:
 *    Obtains location information for system-related directories.
 *  
 *  Discussion:
 *    For the folder type on the particular volume (specified,
 *    respectively, in the folderType and vRefNum parameters), the
 *    FindFolder function returns the directory's volume reference
 *    number in the foundVRefNum parameter and its directory ID in the
 *    foundDirID parameter.
 *    
 *    The specified folder used for a given volume might be located on
 *    a different volume in future versions of system software;
 *    therefore, do not assume the volume that you specify in vRefNum
 *    and the volume returned in foundVRefNum will be the same.
 *     
 *    Specify a volume reference number (or the constant kOnSystemDisk
 *    for the startup disk) or one of the domain constants ( on Mac OS
 *    X ) in the vRefNum parameter.
 *    
 *    Specify a four-character folder type--or the constant that
 *    represents it--in the folderType parameter.
 *    
 *    Use the constant kCreateFolder in the createFolder parameter to
 *    tell FindFolder to create a directory if it does not already
 *    exist; otherwise, use the constant kDontCreateFolder. Directories
 *    inside the System Folder are created only if the System Folder
 *    directory exists. The FindFolder function will not create a
 *    System Folder directory even if you specify the kCreateFolder
 *    constant in the createFolder parameter.
 *    
 *    The FindFolder function returns a nonzero result code if the
 *    folder isn't found, and it can also return other file system
 *    errors reported by the File Manager or Memory Manager.
 *     FindFolderExtended() is equivalent to FindFolder() on Mac OS X.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    vRefNum:
 *      The volume reference number (or the constant kOnSystemDisk for
 *      the startup disk) or one of the domain constants ( like
 *      kUserDomain ) of the volume or domain in which you want to
 *      locate a directory.
 *    
 *    folderType:
 *      A four-character folder type, or a constant that represents the
 *      type, for the directory you want to find.
 *    
 *    createFolder:
 *      Pass the constant kCreateFolder in this parameter to create a
 *      directory if it does not already exist; otherwise, pass the
 *      constant kDontCreateFolder.
 *    
 *    foundVRefNum:
 *      The volume reference number, returned by FindFolder , for the
 *      volume containing the directory you specify in the folderType
 *      parameter.
 *    
 *    flags:
 *      The flags passed in which control extended behaviour
 *    
 *    data:
 *      Unique data which is interpreted differently depending on the
 *      passed in flags.
 *    
 *    foundDirID:
 *      The directory ID number, returned by FindFolder , for the
 *      directory you specify in the folderType parameter.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.0 and later
 }
function FindFolderExtended( vRefNum: FSVolumeRefNum; folderType: OSType; createFolder: Boolean; flags: UInt32; data: UnivPtr; var foundVRefNum: FSVolumeRefNum; var foundDirID: SInt32 ): OSErr; external name '_FindFolderExtended';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


{
 *  FSFindFolderExtended()   *** DEPRECATED ***
 *  
 *  Summary:
 *    FSFindFolderExtended returns an FSRef for certain system-related
 *    directories.
 *  
 *  Discussion:
 *    For the folder type on the particular volume (specified,
 *    respectively, in the folderType and vRefNum parameters), the
 *    FindFolder function returns the FSRef of that directory. 
 *     
 *    The specified folder used for a given volume might be located on
 *    a different volume in future versions of system software;
 *    therefore, do not assume the volume that you specify in vRefNum
 *    and the volume returned in the FSRef will be the same.
 *    
 *    Specify a volume reference number (or the constant kOnSystemDisk
 *    for the startup disk) or one of the domain constants ( on Mac OS
 *    X ) in the vRefNum parameter.
 *    
 *    Specify a four-character folder type--or the constant that
 *    represents it--in the folderType parameter.
 *    
 *    Use the constant kCreateFolder in the createFolder parameter to
 *    tell FindFolder to create a directory if it does not already
 *    exist; otherwise, use the constant kDontCreateFolder. Directories
 *    inside the System Folder are created only if the System Folder
 *    directory exists. The FindFolder function will not create a
 *    System Folder directory even if you specify the kCreateFolder
 *    constant in the createFolder parameter.
 *    
 *    The FindFolder function returns a nonzero result code if the
 *    folder isn't found, and it can also return other file system
 *    errors reported by the File Manager or Memory Manager.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    vRefNum:
 *      The volume reference number (or the constant kOnSystemDisk for
 *      the startup disk) or one of the domain constants ( like
 *      kUserDomain ) of the volume or domain in which you want to
 *      locate a directory.
 *    
 *    folderType:
 *      A four-character folder type, or a constant that represents the
 *      type, for the directory you want to find.
 *    
 *    createFolder:
 *      Pass the constant kCreateFolder in this parameter to create a
 *      directory if it does not already exist; otherwise, pass the
 *      constant kDontCreateFolder.
 *    
 *    flags:
 *      The flags passed in which control extended behaviour
 *    
 *    data:
 *      Unique data which is interpreted differently depending on the
 *      passed in flags.
 *    
 *    foundRef:
 *      The FSRef for the directory you specify on the volume or domain
 *      and folderType given.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSFindFolderExtended( vRefNum: FSVolumeRefNum; folderType: OSType; createFolder: Boolean; flags: UInt32; data: UnivPtr; var foundRef: FSRef ): OSErr; external name '_FSFindFolderExtended';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


{
 *  GetFolderDescriptor()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    GetFolderDescriptor is deprecated on Mac OS X.
 *  
 *  Summary:
 *    Obtains the folder descriptor information for the specified
 *    folder type from the global descriptor list.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    foldType:
 *      Pass a constant identifying the type of the folder for which
 *      you wish to get descriptor information. See "Folder Type
 *      Constants".
 *    
 *    descSize:
 *      Pass the size (in bytes) of the folder descriptor structure for
 *      which a pointer is passed in the foldDesc parameter. This value
 *      is needed in order to determine the version of the structure
 *      being used.
 *    
 *    foldDesc:
 *      Pass a pointer to a folder descriptor structure. On return, the
 *      folder descriptor structure contains information from the
 *      global descriptor list for the specified folder type.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FoldersLib 1.0 and later
 }
function GetFolderDescriptor( foldType: FolderType; descSize: Size; var foldDesc: FolderDesc ): OSErr; external name '_GetFolderDescriptor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


{
 *  GetFolderName()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use GetFolderNameUnicode.
 *  
 *  Summary:
 *    Obtains the name of the specified folder.
 *  
 *  Discussion:
 *    The GetFolderName function obtains the name of the folder in the
 *    folder descriptor, not the name of the folder on the disk. The
 *    names may differ for a few special folders such as the System
 *    Folder. For relative folders, however, the actual name is always
 *    returned. You typically do not need to call this function.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    vRefNum:
 *      Pass the volume reference number (or the constant kOnSystemDisk
 *      for the startup disk) of the volume containing the folder for
 *      which you wish the name to be identified.
 *    
 *    foldType:
 *      Pass a constant identifying the type of the folder for which
 *      you wish the name to be identified. See "Folder Type Constants".
 *    
 *    foundVRefNum:
 *      On return, a pointer to the volume reference number for the
 *      volume containing the folder specified in the foldType
 *      parameter.
 *    
 *    name:
 *      On return, a string containing the title of the folder
 *      specified in the foldType and vRefNum parameters.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FoldersLib 1.0 and later
 }
function GetFolderName( vRefNum: FSVolumeRefNum; foldType: OSType; var foundVRefNum: FSVolumeRefNum; var name: StrFileName ): OSErr; external name '_GetFolderName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  AddFolderRouting()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Folder Manager routing is deprecated on Mac OS X.  Do not rely on
 *    this feature in your application, because support for it will be
 *    removed in a future version of the OS.
 *  
 *  Summary:
 *    Adds a folder routing structure to the global routing list.
 *  
 *  Discussion:
 *    Your application can use the AddFolderRouting function to specify
 *    how the Finder routes a given file type. 
 *    Folder Routing is deprecated on Mac OS X at this time.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    fileType:
 *      Pass the OSType of the file to be routed.
 *    
 *    routeFromFolder:
 *      Pass the folder type of the "from" folder see "Folder Type
 *      Constants" for descriptions of possible values. An item dropped
 *      on the folder specified in this parameter will be routed to the
 *      folder specified in the routeToFolder parameter.
 *    
 *    routeToFolder:
 *      The folder type of the "to" folder see "Folder Type Constants"
 *      for descriptions of possible values.
 *    
 *    flags:
 *      Reserved for future use; pass 0.
 *    
 *    replaceFlag:
 *      Pass a Boolean value indicating whether you wish to replace a
 *      folder routing that already exists. If true , it replaces the
 *      folder to which the item is being routed. If false , it leaves
 *      the folder to which the item is being routed.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FoldersLib 1.0 and later
 }
function AddFolderRouting( fileType: OSType; routeFromFolder: FolderType; routeToFolder: FolderType; flags: RoutingFlags; replaceFlag: Boolean ): OSErr; external name '_AddFolderRouting';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  RemoveFolderRouting()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Folder Manager routing is deprecated on Mac OS X.  Do not rely on
 *    this feature in your application, because support for it will be
 *    removed in a future version of the OS.
 *  
 *  Summary:
 *    Deletes a folder routing structure from the global routing list.
 *  
 *  Discussion:
 *    Both the file type and the folder type specified must match those
 *    of an existing folder routing structure in the global routing
 *    list for the RemoveFolderRouting function to succeed.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    fileType:
 *      Pass the file type value contained in the folder routing
 *      structure to be removed.
 *    
 *    routeFromFolder:
 *      Pass the folder type of the "from" folder see "Folder Type
 *      Constants" for descriptions of possible values.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FoldersLib 1.0 and later
 }
function RemoveFolderRouting( fileType: OSType; routeFromFolder: FolderType ): OSErr; external name '_RemoveFolderRouting';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  FindFolderRouting()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Folder Manager routing is deprecated on Mac OS X.  Do not rely on
 *    this feature in your application, because support for it will be
 *    removed in a future version of the OS.
 *  
 *  Summary:
 *    Finds the destination folder from a matching folder routing
 *    structure for the specified file.
 *  
 *  Discussion:
 *    Both the file type and the folder type specified must match those
 *    of a folder routing structure in the global routing list for the
 *    FindFolderRouting function to succeed.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    fileType:
 *      Pass the file type specified in the appropriate folder routing
 *      structure for the file for which you wish to find a destination
 *      folder.
 *    
 *    routeFromFolder:
 *      Pass the folder type of the "from" folder for which you wish to
 *      find a "to" folder see "Folder Type Constants" for descriptions
 *      of possible values. An item dropped on the folder specified in
 *      this parameter will be routed to the folder specified in the
 *      routeToFolder parameter.
 *    
 *    routeToFolder:
 *      A pointer to a value of type FolderType. On return, the value
 *      is set to the folder type of the destination folder.
 *    
 *    flags:
 *      Reserved; pass 0.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FoldersLib 1.0 and later
 }
function FindFolderRouting( fileType: OSType; routeFromFolder: FolderType; var routeToFolder: FolderType; var flags: RoutingFlags ): OSErr; external name '_FindFolderRouting';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  GetFolderRoutings()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Folder Manager routing is deprecated on Mac OS X.  Do not rely on
 *    this feature in your application, because support for it will be
 *    removed in a future version of the OS.
 *  
 *  Summary:
 *    Obtains folder routing information from the global routing list.
 *  
 *  Discussion:
 *    The folder routing information in the global routing list
 *    determines how the Finder routes files.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    requestedRoutingCount:
 *      An unsigned 32-bit value. Pass the number of folder routing
 *      structures that can fit in the buffer pointed to by the
 *      theRoutings parameter.
 *    
 *    totalRoutingCount:
 *      A pointer to an unsigned 32-bit value. On return, the value is
 *      set to the number of folder routing structures in the global
 *      list. If this value is less than or equal to
 *      requestedRoutingCount , all folder routing structures were
 *      returned to the caller.
 *    
 *    routingSize:
 *      Pass the size (in bytes) of the FolderRouting structure.
 *    
 *    theRoutings:
 *      Pass a pointer to an array of FolderRouting structures. On
 *      return the structure(s) contain the requested routing
 *      information. You may pass null if you do not wish this
 *      information.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in FoldersLib 1.0 and later
 }
function GetFolderRoutings( requestedRoutingCount: UInt32; var totalRoutingCount: UInt32; routingSize: Size; var theRoutings: FolderRouting ): OSErr; external name '_GetFolderRoutings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  FSpDetermineIfSpecIsEnclosedByFolder()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSDetemineIfRefIsEnclosedByFolder
 *  
 *  Summary:
 *    Determine whether the given FSSpec is enclosed inside the given
 *    special folder type for the given domain.
 *  
 *  Discussion:
 *    This is a fairly fast call which can determine whether a given
 *    FSSpec on disk is 'inside' the given special folder type for the
 *    given domain.  This call will be more efficient than the
 *    equivalent client code which walked up the file list, checking
 *    each parent with IdentifyFolder() to see if it matches. One use
 *    for this call is to determine if a given file or folder is inside
 *    the trash on a volume, with something like
 *    
 *    err = FSpDetermineIfRefIsEnclosedByFolder( kOnAppropriateDisk,
 *    kTrashFolderType, & spec, & result );
 *    if ( err == noErr && result ) (
 *    //  FSSpec is inside trash on the volume.<br> )
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    domainOrVRefNum:
 *      The domain or vRefNum to check.  You can also pass
 *      kOnAppropriateDisk to check whatever vRefNum is appropriate for
 *      the given FSSpec, or the value 0 to check all vRefNums and
 *      domains.
 *    
 *    folderType:
 *      The folder type to check
 *    
 *    inSpec:
 *      The FSSpec to look for.
 *    
 *    outResult:
 *      If non-NULL, this will be filled in with true if the given
 *      FSSpec is enclosed inside the special folder type for the given
 *      domain, or false otherwise.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function FSpDetermineIfSpecIsEnclosedByFolder( domainOrVRefNum: FSVolumeRefNum; folderType: OSType; const (*var*) inSpec: FSSpec; var outResult: Boolean ): OSErr; external name '_FSpDetermineIfSpecIsEnclosedByFolder';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{$endc} {not TARGET_CPU_64}

type
	FolderManagerNotificationProcPtr = function( message: OSType; arg: UnivPtr; userRefCon: UnivPtr ): OSStatus;
	FolderManagerNotificationUPP = FolderManagerNotificationProcPtr;
{
 *  NewFolderManagerNotificationUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewFolderManagerNotificationUPP( userRoutine: FolderManagerNotificationProcPtr ): FolderManagerNotificationUPP; external name '_NewFolderManagerNotificationUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeFolderManagerNotificationUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeFolderManagerNotificationUPP( userUPP: FolderManagerNotificationUPP ); external name '_DisposeFolderManagerNotificationUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeFolderManagerNotificationUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0.2 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeFolderManagerNotificationUPP( message: OSType; arg: UnivPtr; userRefCon: UnivPtr; userUPP: FolderManagerNotificationUPP ): OSStatus; external name '_InvokeFolderManagerNotificationUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{$ifc not TARGET_CPU_64}
{
 *  FolderManagerRegisterNotificationProc()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    This function is deprecated on Mac OS X.
 *  
 *  Summary:
 *    Register a function to be called at certain times
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    notificationProc:
 *    
 *    refCon:
 *    
 *    options:
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.0 and later
 }
function FolderManagerRegisterNotificationProc( notificationProc: FolderManagerNotificationUPP; refCon: UnivPtr; options: UInt32 ): OSErr; external name '_FolderManagerRegisterNotificationProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


{
 *  FolderManagerUnregisterNotificationProc()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    This function is deprecated on Mac OS X.
 *  
 *  Summary:
 *    Unregister a function to be called at certain times
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    notificationProc:
 *    
 *    refCon:
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.0 and later
 }
function FolderManagerUnregisterNotificationProc( notificationProc: FolderManagerNotificationUPP; refCon: UnivPtr ): OSErr; external name '_FolderManagerUnregisterNotificationProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


{
 *  FolderManagerRegisterCallNotificationProcs()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    This function is deprecated on Mac OS X.
 *  
 *  Summary:
 *    Call the registered Folder Manager notification procs.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    message:
 *    
 *    arg:
 *    
 *    options:
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.0 and later
 }
function FolderManagerRegisterCallNotificationProcs( message: OSType; arg: UnivPtr; options: UInt32 ): OSStatus; external name '_FolderManagerRegisterCallNotificationProcs';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


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
