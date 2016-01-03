{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       CarbonCore/Aliases.h
 
     Contains:   Alias Manager Interfaces.
                 The contents of this header file are deprecated.
                 Use Foundation or CoreFoundation URL Bookmarks instead.
 
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
 
     Copyright:  © 1989-2008 by Apple Computer, Inc., all rights reserved
=======
=======
>>>>>>> origin/fixes_2_2
     File:       Aliases.p
=======
     File:       CarbonCore/Aliases.h
>>>>>>> origin/fixes_2.4
 
     Contains:   Alias Manager Interfaces.
 
     Version:    CarbonCore-859.2~1
 
<<<<<<< HEAD
     Copyright:  © 1989-2002 by Apple Computer, Inc., all rights reserved
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  © 1989-2008 by Apple Computer, Inc., all rights reserved
>>>>>>> origin/fixes_2.4
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
                     http://www.freepascal.org/bugs.html
 
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{	  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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
{	  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit Aliases;
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
=======
{$elsec}
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
uses MacTypes,Files,UTCUtils,CFBase;
{$endc} {not MACOSALLINCLUDE}
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/cpstrnew

=======
>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}

{$ifc TARGET_OS_MAC}

{$ifc TARGET_OS_MAC}
=======
uses MacTypes,Files;

>>>>>>> graemeg/fixes_2_2
=======
uses MacTypes,Files;
=======
uses MacTypes,Files,UTCUtils,CFBase;
{$endc} {not MACOSALLINCLUDE}
>>>>>>> origin/fixes_2.4

>>>>>>> origin/fixes_2_2

{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
type
	FSAliasInfoBitmap = UInt32;
	FSAliasInfoBitmapPtr = ^FSAliasInfoBitmap;
const
	kFSAliasInfoNone = $00000000; { no valid info}
	kFSAliasInfoVolumeCreateDate = $00000001; { volume creation date is valid}
	kFSAliasInfoTargetCreateDate = $00000002; { target creation date is valid}
	kFSAliasInfoFinderInfo = $00000004; { file type and creator are valid}
	kFSAliasInfoIsDirectory = $00000008; { isDirectory boolean is valid}
	kFSAliasInfoIDs = $00000010; { parentDirID and nodeID are valid}
	kFSAliasInfoFSInfo = $00000020; { filesystemID and signature are valid}
	kFSAliasInfoVolumeFlags = $00000040; { volumeIsBootVolume, volumeIsAutomounted, volumeIsEjectable and volumeHasPersistentFileIDs are valid}

const
	rAliasType = FourCharCode('alis'); { Aliases are stored as resources of this type }

const
{ define alias resolution action rules mask }
	kARMMountVol = $00000001; { mount the volume automatically }
	kARMNoUI = $00000002; { no user interface allowed during resolution }
	kARMMultVols = $00000008; { search on multiple volumes }
	kARMSearch = $00000100; { search quickly }
	kARMSearchMore = $00000200; { search further }
	kARMSearchRelFirst = $00000400; { search target on a relative path first }
	kARMTryFileIDFirst = $00000800; { search by file id before path }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
const
{ define alias record information types }
	asiZoneName = -3;   { get zone name }
	asiServerName = -2;   { get server name }
	asiVolumeName = -1;   { get volume name }
	asiAliasName = 0;    { get aliased file/folder/volume name }
	asiParentName = 1;     { get parent folder name }

{ ResolveAliasFileWithMountFlags options }
const
=======
const
{ define alias record information types }
	asiZoneName = -3;   { get zone name }
	asiServerName = -2;   { get server name }
	asiVolumeName = -1;   { get volume name }
	asiAliasName = 0;    { get aliased file/folder/volume name }
	asiParentName = 1;     { get parent folder name }

{ ResolveAliasFileWithMountFlags options }
const
>>>>>>> graemeg/cpstrnew
=======
const
=======
const
>>>>>>> graemeg/cpstrnew
{ define alias record information types }
	asiZoneName = -3;   { get zone name }
	asiServerName = -2;   { get server name }
	asiVolumeName = -1;   { get volume name }
	asiAliasName = 0;    { get aliased file/folder/volume name }
	asiParentName = 1;     { get parent folder name }

{ ResolveAliasFileWithMountFlags options }
const
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
const
{ define alias record information types }
	asiZoneName = -3;   { get zone name }
	asiServerName = -2;   { get server name }
	asiVolumeName = -1;   { get volume name }
	asiAliasName = 0;    { get aliased file/folder/volume name }
	asiParentName = 1;     { get parent folder name }

{ ResolveAliasFileWithMountFlags options }
const
>>>>>>> origin/cpstrnew
	kResolveAliasFileNoUI = $00000001; { no user interaction during resolution }
	kResolveAliasTryFileIDFirst = $00000002; { search by file id before path }
=======
=======
>>>>>>> origin/fixes_2_2
const
	rAliasType					= FourCharCode('alis');						{  Aliases are stored as resources of this type  }

																{  define alias resolution action rules mask  }
	kARMMountVol				= $00000001;					{  mount the volume automatically  }
	kARMNoUI					= $00000002;					{  no user interface allowed during resolution  }
	kARMMultVols				= $00000008;					{  search on multiple volumes  }
	kARMSearch					= $00000100;					{  search quickly  }
	kARMSearchMore				= $00000200;					{  search further  }
	kARMSearchRelFirst			= $00000400;					{  search target on a relative path first  }

																{  define alias record information types  }
	asiZoneName					= -3;							{  get zone name  }
	asiServerName				= -2;							{  get server name  }
	asiVolumeName				= -1;							{  get volume name  }
	asiAliasName				= 0;							{  get aliased file/folder/volume name  }
	asiParentName				= 1;							{  get parent folder name  }

	{	 ResolveAliasFileWithMountFlags options 	}
	kResolveAliasFileNoUI		= $00000001;					{  no user interaction during resolution  }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
const
{ define alias record information types }
	asiZoneName = -3;   { get zone name }
	asiServerName = -2;   { get server name }
	asiVolumeName = -1;   { get volume name }
	asiAliasName = 0;    { get aliased file/folder/volume name }
	asiParentName = 1;     { get parent folder name }

{ ResolveAliasFileWithMountFlags options }
const
	kResolveAliasFileNoUI = $00000001; { no user interaction during resolution }
	kResolveAliasTryFileIDFirst = $00000002; { search by file id before path }
>>>>>>> origin/fixes_2.4

	{	 define the alias record that will be the blackbox for the caller 	}

type
	AliasRecordPtr = ^AliasRecord;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  { Opaque as of Mac OS X 10.4 ... }
	AliasRecord = record
		userType: OSType;               { appl stored type like creator type }
		aliasSize: UInt16;              { alias record size in bytes, for appl usage }
	end;

type
	AliasPtr = AliasRecordPtr;
	AliasHandle = ^AliasPtr;
{ info block to pass to FSCopyAliasInfo }
type
	FSAliasInfo = record
		volumeCreateDate: UTCDateTime;
		targetCreateDate: UTCDateTime;
		fileType: OSType;
		fileCreator: OSType;
		parentDirID: UInt32;
		nodeID: UInt32;
		filesystemID: UInt16;
		signature: UInt16;
		volumeIsBootVolume: Boolean;
		volumeIsAutomounted: Boolean;
		volumeIsEjectable: Boolean;
		volumeHasPersistentFileIDs: Boolean;
		isDirectory: Boolean;
	end;
	FSAliasInfoPtr = ^FSAliasInfo;
{ alias record information type }
type
	AliasInfoType = SInt16;
{$ifc not TARGET_CPU_64}
type
	AliasFilterProcPtr = function( cpbPtr: CInfoPBPtr; var quitFlag: Boolean; myDataPtr: Ptr ): Boolean;
	AliasFilterUPP = AliasFilterProcPtr;
<<<<<<< HEAD
<<<<<<< HEAD
{
 *  NewAliasFilterUPP()
<<<<<<< HEAD
=======
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
{ old name was NewAliasFilterProc }
function NewAliasFilterUPP( userRoutine: AliasFilterProcPtr ): AliasFilterUPP; external name '_NewAliasFilterUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeAliasFilterUPP()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
<<<<<<< HEAD
 }
{ old name was NewAliasFilterProc }
function NewAliasFilterUPP( userRoutine: AliasFilterProcPtr ): AliasFilterUPP; external name '_NewAliasFilterUPP';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> graemeg/cpstrnew
{
 *  DisposeAliasFilterUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
<<<<<<< HEAD
 }
procedure DisposeAliasFilterUPP( userUPP: AliasFilterUPP ); external name '_DisposeAliasFilterUPP';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)

{
 *  InvokeAliasFilterUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeAliasFilterUPP( cpbPtr: CInfoPBPtr; var quitFlag: Boolean; myDataPtr: Ptr; userUPP: AliasFilterUPP ): Boolean; external name '_InvokeAliasFilterUPP';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)

{#if __MACH__
    #define NewAliasFilterUPP(userRoutine)                      ((AliasFilterUPP)userRoutine)
    #define DisposeAliasFilterUPP(userUPP)
    #define InvokeAliasFilterUPP(cpbPtr, quitFlag, myDataPtr, userUPP) (*userUPP)(cpbPtr, quitFlag, myDataPtr)
#endif}

{$endc} {not TARGET_CPU_64}

type
	FSAliasFilterProcPtr = function( const (*var*) ref: FSRef; var quitFlag: Boolean; myDataPtr: Ptr ): Boolean;
	FSAliasFilterUPP = FSAliasFilterProcPtr;

{
 *  FSNewAlias()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSNewAlias( {const} fromFile: FSRefPtr { can be NULL }; const (*var*) target: FSRef; var inAlias: AliasHandle ): OSErr; external name '_FSNewAlias';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)


{
 *  FSNewAliasMinimal()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSNewAliasMinimal( const (*var*) target: FSRef; var inAlias: AliasHandle ): OSErr; external name '_FSNewAliasMinimal';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)


{
 *  FSIsAliasFile()
 *  
=======
{
 *  NewAliasFilterUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
{ old name was NewAliasFilterProc }
function NewAliasFilterUPP( userRoutine: AliasFilterProcPtr ): AliasFilterUPP; external name '_NewAliasFilterUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
=======
  { Opaque as of Mac OS X 10.4 ... }
>>>>>>> origin/fixes_2.4
	AliasRecord = record
		userType: OSType;               { appl stored type like creator type }
		aliasSize: UInt16;              { alias record size in bytes, for appl usage }
	end;

type
	AliasPtr = AliasRecordPtr;
	AliasHandle = ^AliasPtr;
{ info block to pass to FSCopyAliasInfo }
type
	FSAliasInfo = record
		volumeCreateDate: UTCDateTime;
		targetCreateDate: UTCDateTime;
		fileType: OSType;
		fileCreator: OSType;
		parentDirID: UInt32;
		nodeID: UInt32;
		filesystemID: UInt16;
		signature: UInt16;
		volumeIsBootVolume: Boolean;
		volumeIsAutomounted: Boolean;
		volumeIsEjectable: Boolean;
		volumeHasPersistentFileIDs: Boolean;
		isDirectory: Boolean;
	end;
	FSAliasInfoPtr = ^FSAliasInfo;
{ alias record information type }
type
	AliasInfoType = SInt16;
{$ifc not TARGET_CPU_64}
type
	AliasFilterProcPtr = function( cpbPtr: CInfoPBPtr; var quitFlag: Boolean; myDataPtr: Ptr ): Boolean;
	AliasFilterUPP = AliasFilterProcPtr;
{
 *  NewAliasFilterUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
{ old name was NewAliasFilterProc }
function NewAliasFilterUPP( userRoutine: AliasFilterProcPtr ): AliasFilterUPP; external name '_NewAliasFilterUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeAliasFilterUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeAliasFilterUPP( userUPP: AliasFilterUPP ); external name '_DisposeAliasFilterUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeAliasFilterUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeAliasFilterUPP( cpbPtr: CInfoPBPtr; var quitFlag: Boolean; myDataPtr: Ptr; userUPP: AliasFilterUPP ): Boolean; external name '_InvokeAliasFilterUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{#if __MACH__
    #define NewAliasFilterUPP(userRoutine)                      ((AliasFilterUPP)userRoutine)
    #define DisposeAliasFilterUPP(userUPP)
    #define InvokeAliasFilterUPP(cpbPtr, quitFlag, myDataPtr, userUPP) (*userUPP)(cpbPtr, quitFlag, myDataPtr)
#endif}

{$endc} {not TARGET_CPU_64}

type
	FSAliasFilterProcPtr = function( const (*var*) ref: FSRef; var quitFlag: Boolean; myDataPtr: Ptr ): Boolean;
	FSAliasFilterUPP = FSAliasFilterProcPtr;

{
 *  FSNewAlias()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSNewAlias( {const} fromFile: FSRefPtr { can be NULL }; const (*var*) target: FSRef; var inAlias: AliasHandle ): OSErr; external name '_FSNewAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  FSNewAliasMinimal()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSNewAliasMinimal( const (*var*) target: FSRef; var inAlias: AliasHandle ): OSErr; external name '_FSNewAliasMinimal';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  FSIsAliasFile()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSIsAliasFile( const (*var*) fileRef: FSRef; var aliasFileFlag: Boolean; var folderFlag: Boolean ): OSErr; external name '_FSIsAliasFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  FSResolveAliasWithMountFlags()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSResolveAliasWithMountFlags( {const} fromFile: FSRefPtr { can be NULL }; inAlias: AliasHandle; var target: FSRef; var wasChanged: Boolean; mountFlags: UNSIGNEDLONG ): OSErr; external name '_FSResolveAliasWithMountFlags';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  FSResolveAlias()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSResolveAlias( {const} fromFile: FSRefPtr { can be NULL }; alias: AliasHandle; var target: FSRef; var wasChanged: Boolean ): OSErr; external name '_FSResolveAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  FSResolveAliasFileWithMountFlags()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSResolveAliasFileWithMountFlags( var theRef: FSRef; resolveAliasChains: Boolean; var targetIsFolder: Boolean; var wasAliased: Boolean; mountFlags: UNSIGNEDLONG ): OSErr; external name '_FSResolveAliasFileWithMountFlags';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  FSResolveAliasFile()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSResolveAliasFile( var theRef: FSRef; resolveAliasChains: Boolean; var targetIsFolder: Boolean; var wasAliased: Boolean ): OSErr; external name '_FSResolveAliasFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  FSFollowFinderAlias()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSFollowFinderAlias( fromFile: FSRefPtr { can be NULL }; alias: AliasHandle; logon: Boolean; var target: FSRef; var wasChanged: Boolean ): OSErr; external name '_FSFollowFinderAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  FSUpdateAlias()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSUpdateAlias( {const} fromFile: FSRefPtr { can be NULL }; const (*var*) target: FSRef; alias: AliasHandle; var wasChanged: Boolean ): OSErr; external name '_FSUpdateAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  FSNewAliasUnicode()
 *  
 *  Summary:
 *    Creates an alias given a ref to the target's parent directory and
 *    the target's unicode name.  If the target does not exist fnfErr
 *    will be returned but the alias will still be created.  This
 *    allows the creation of aliases to targets that do not exist.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    fromFile:
 *      The starting point for a relative search.
 *    
 *    targetParentRef:
 *      An FSRef to the parent directory of the target.
 *    
 *    targetNameLength:
 *      Number of Unicode characters in the target's name.
 *    
 *    targetName:
 *      A pointer to the Unicode name.
 *    
 *    inAlias:
 *      A Handle to the newly created alias record.
 *    
 *    isDirectory:
 *      On input, if target does not exist, a flag to indicate whether
 *      or not the target is a directory.  On output, if the target did
 *      exist, a flag indicating if the target is a directory.  Pass
 *      NULL in the non-existant case if unsure.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function FSNewAliasUnicode( {const} fromFile: FSRefPtr { can be NULL }; const (*var*) targetParentRef: FSRef; targetNameLength: UniCharCount; targetName: ConstUniCharPtr; var inAlias: AliasHandle; isDirectory: BooleanPtr { can be NULL } ): OSErr; external name '_FSNewAliasUnicode';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  FSNewAliasMinimalUnicode()
 *  
 *  Summary:
 *    Creates a minimal alias given a ref to the target's parent
 *    directory and the target's unicode name.  If the target does not
 *    exist fnfErr will be returned but the alias will still be created.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    targetParentRef:
 *      An FSRef to the parent directory of the target.
 *    
 *    targetNameLength:
 *      Number of Unicode characters in the target's name.
 *    
 *    targetName:
 *      A pointer to the Unicode name.
 *    
 *    inAlias:
 *      A Handle to the newly created alias record.
 *    
 *    isDirectory:
 *      On input, if target does not exist, a flag to indicate whether
 *      or not the target is a directory.  On output, if the target did
 *      exist, a flag indicating if the target is a directory.  Pass
 *      NULL in the non-existant case if unsure.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function FSNewAliasMinimalUnicode( const (*var*) targetParentRef: FSRef; targetNameLength: UniCharCount; targetName: ConstUniCharPtr; var inAlias: AliasHandle; isDirectory: BooleanPtr { can be NULL } ): OSErr; external name '_FSNewAliasMinimalUnicode';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  FSNewAliasFromPath()
 *  
 *  Summary:
 *    Creates an alias given a POSIX style utf-8 path to the target. 
 *    If the target file does not exist but the path up to the leaf
 *    does then fnfErr will be returned but the alias will still be
 *    created.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    fromFilePath:
 *      The starting point for a relative search.
 *    
 *    targetPath:
 *      POSIX style UTF-8 path to target.
 *    
 *    flags:
 *      Options for future use.  Pass in 0.
 *    
 *    inAlias:
 *      A Handle to the newly created alias record.
 *    
 *    isDirectory:
 *      On input, if target does not exist, a flag to indicate whether
 *      or not the target is a directory.  On output, if the target did
 *      exist, a flag indicating if the target is a directory.  Pass
 *      NULL in the non-existant case if unsure.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.5 and later
 *    Non-Carbon CFM:   not available
 }
function FSNewAliasFromPath( fromFilePath: ConstCStringPtr { can be NULL }; targetPath: ConstCStringPtr; flags: OptionBits; var inAlias: AliasHandle; isDirectory: BooleanPtr { can be NULL } ): OSStatus; external name '_FSNewAliasFromPath';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  FSMatchAliasBulk()
 *  
 *  Summary:
 *    Given an alias handle and fromFile, match the alias and return
 *    FSRefs to the aliased file(s) and needsUpdate flag
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function FSMatchAliasBulk( {const} fromFile: FSRefPtr { can be NULL }; rulesMask: UNSIGNEDLONG; inAlias: AliasHandle; var aliasCount: SInt16; var aliasList: FSRef; var needsUpdate: Boolean; aliasFilter: FSAliasFilterProcPtr { can be NULL }; yourDataPtr: UnivPtr { can be NULL } ): OSStatus; external name '_FSMatchAliasBulk';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  FSCopyAliasInfo()
 *  
 *  Discussion:
 *    This routine will return the requested information from the
 *    passed in aliasHandle.  The information is gathered only from the
 *    alias record so it may not match what is on disk (no disk i/o is
 *    performed).  The whichInfo paramter is an output parameter that
 *    signifies which fields in the info record contain valid data.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    inAlias:
 *      A handle to the alias record to get the information from.
 *    
 *    targetName:
 *      The name of the target item.
 *    
 *    volumeName:
 *      The name of the volume the target resides on.
 *    
 *    pathString:
 *      POSIX path to target.
 *    
 *    whichInfo:
 *      An indication of which fields in the info block contain valid
 *      data.
 *    
 *    info:
 *      Returned information about the alias.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function FSCopyAliasInfo( inAlias: AliasHandle; targetName: HFSUniStr255Ptr { can be NULL }; volumeName: HFSUniStr255Ptr { can be NULL }; pathString: CFStringRefPtr { can be NULL }; whichInfo: FSAliasInfoBitmapPtr { can be NULL }; info: FSAliasInfoPtr { can be NULL } ): OSStatus; external name '_FSCopyAliasInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  GetAliasSize()
 *  
 *  Discussion:
 *    This routine will return the size of the alias record referenced
 *    by the AliasHandle alias.  This will be smaller than the size
 *    returned by GetHandleSize if any custom data has been added (IM
 *    Files 4-13).
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    alias:
 *      A handle to the alias record to get the information from.
 *  
 *  Result:
 *    The size of the private section of the alias record.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function GetAliasSize( alias: AliasHandle ): Size; external name '_GetAliasSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  GetAliasUserType()
 *  
 *  Discussion:
 *    This routine will return the usertype associated with the alias
 *    record referenced by the AliasHandle alias.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    alias:
 *      A handle to the alias record to get the userType from.
 *  
 *  Result:
 *    The userType associated with the alias as an OSType
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function GetAliasUserType( alias: AliasHandle ): OSType; external name '_GetAliasUserType';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  SetAliasUserType()
 *  
 *  Discussion:
 *    This routine will set the userType associated with an alias
 *    record.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    alias:
 *      A handle to the alias record to set the userType for.
 *    
 *    userType:
 *      The OSType to set the userType to.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
procedure SetAliasUserType( alias: AliasHandle; userType: OSType ); external name '_SetAliasUserType';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  GetAliasSizeFromPtr()
 *  
 *  Discussion:
 *    This routine will return the size of the alias record referenced
 *    by a pointer to the AliasRecord.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    alias:
 *      A pointer to the alias record to get the information from.
 *  
 *  Result:
 *    The size of the private section of the alias record.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function GetAliasSizeFromPtr( const (*var*) alias: AliasRecord ): Size; external name '_GetAliasSizeFromPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  GetAliasUserTypeFromPtr()
 *  
 *  Discussion:
 *    This routine will return the usertype associated withthe alias
 *    record pointed to by alias.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    alias:
 *      A pointer to the alias record to get the userType from.
 *  
 *  Result:
 *    The userType associated with the alias as an OSType
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function GetAliasUserTypeFromPtr( const (*var*) alias: AliasRecord ): OSType; external name '_GetAliasUserTypeFromPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  SetAliasUserTypeWithPtr()
 *  
 *  Discussion:
 *    This routine will set the userType associated with an alias
 *    record.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    alias:
 *      A pointer to the alias record to set the userType for.
 *    
 *    userType:
 *      The OSType to set the userType to.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
procedure SetAliasUserTypeWithPtr( alias: AliasPtr; userType: OSType ); external name '_SetAliasUserTypeWithPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{ Functions beyond this point are deprecated}

{$ifc not TARGET_CPU_64}
{
 *  FSMatchAlias()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSMatchAliasBulk instead
 *  
 *  Summary:
 *    Given an alias handle and fromFile, match the alias and return
 *    FSRefs to the aliased file(s) and needsUpdate flag
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function FSMatchAlias( {const} fromFile: FSRefPtr { can be NULL }; rulesMask: UNSIGNEDLONG; inAlias: AliasHandle; var aliasCount: SInt16; var aliasList: FSRef; var needsUpdate: Boolean; aliasFilter: AliasFilterUPP; yourDataPtr: UnivPtr ): OSErr; external name '_FSMatchAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


<<<<<<< HEAD
const
	uppAliasFilterProcInfo = $00000FD0;
	{
	 *  NewAliasFilterUPP()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   available as macro/inline
	 *    CarbonLib:        in CarbonLib 1.0 and later
	 *    Mac OS X:         in version 10.0 and later
	 	}
function NewAliasFilterUPP(userRoutine: AliasFilterProcPtr): AliasFilterUPP; external name '_NewAliasFilterUPP'; { old name was NewAliasFilterProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{
 *  FSMatchAliasNoUI()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSMatchAliasBulk with the kARMNoUI flag instead
 *  
 *  Summary:
 *    variation on FSMatchAlias that does not prompt user with a dialog
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeAliasFilterUPP( userUPP: AliasFilterUPP ); external name '_DisposeAliasFilterUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure DisposeAliasFilterUPP(userUPP: AliasFilterUPP); external name '_DisposeAliasFilterUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.2 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function FSMatchAliasNoUI( {const} fromFile: FSRefPtr { can be NULL }; rulesMask: UNSIGNEDLONG; inAlias: AliasHandle; var aliasCount: SInt16; var aliasList: FSRef; var needsUpdate: Boolean; aliasFilter: AliasFilterUPP; yourDataPtr: UnivPtr ): OSErr; external name '_FSMatchAliasNoUI';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


>>>>>>> origin/fixes_2.4
{
 *  NewAlias()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSNewAlias
 *  
 *  Summary:
 *    create a new alias between fromFile and target, returns alias
 *    record handle
 *  
 *  Discussion:
 *    Create an alias betwen fromFile and target, and return it in an
 *    AliasHandle. This function is deprecated in Mac OS X 10.4;
 *    instead, you should use FSNewAliasUnicode() because NewAlias()
 *    has problems creating aliases to certain files, including those
 *    which are impossible to represent in an FSSpec.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeAliasFilterUPP( cpbPtr: CInfoPBPtr; var quitFlag: Boolean; myDataPtr: Ptr; userUPP: AliasFilterUPP ): Boolean; external name '_InvokeAliasFilterUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{#if __MACH__
    #define NewAliasFilterUPP(userRoutine)                      ((AliasFilterUPP)userRoutine)
    #define DisposeAliasFilterUPP(userUPP)
    #define InvokeAliasFilterUPP(cpbPtr, quitFlag, myDataPtr, userUPP) (*userUPP)(cpbPtr, quitFlag, myDataPtr)
#endif}

{$endc} {not TARGET_CPU_64}

type
	FSAliasFilterProcPtr = function( const (*var*) ref: FSRef; var quitFlag: Boolean; myDataPtr: Ptr ): Boolean;
	FSAliasFilterUPP = FSAliasFilterProcPtr;

{
 *  FSNewAlias()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSNewAlias( {const} fromFile: FSRefPtr { can be NULL }; const (*var*) target: FSRef; var inAlias: AliasHandle ): OSErr; external name '_FSNewAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  FSNewAliasMinimal()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSNewAliasMinimal( const (*var*) target: FSRef; var inAlias: AliasHandle ): OSErr; external name '_FSNewAliasMinimal';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  FSIsAliasFile()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSIsAliasFile( const (*var*) fileRef: FSRef; var aliasFileFlag: Boolean; var folderFlag: Boolean ): OSErr; external name '_FSIsAliasFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  FSResolveAliasWithMountFlags()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSResolveAliasWithMountFlags( {const} fromFile: FSRefPtr { can be NULL }; inAlias: AliasHandle; var target: FSRef; var wasChanged: Boolean; mountFlags: UNSIGNEDLONG ): OSErr; external name '_FSResolveAliasWithMountFlags';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  FSResolveAlias()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSResolveAlias( {const} fromFile: FSRefPtr { can be NULL }; alias: AliasHandle; var target: FSRef; var wasChanged: Boolean ): OSErr; external name '_FSResolveAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  FSResolveAliasFileWithMountFlags()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSResolveAliasFileWithMountFlags( var theRef: FSRef; resolveAliasChains: Boolean; var targetIsFolder: Boolean; var wasAliased: Boolean; mountFlags: UNSIGNEDLONG ): OSErr; external name '_FSResolveAliasFileWithMountFlags';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  FSResolveAliasFile()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSResolveAliasFile( var theRef: FSRef; resolveAliasChains: Boolean; var targetIsFolder: Boolean; var wasAliased: Boolean ): OSErr; external name '_FSResolveAliasFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  FSFollowFinderAlias()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSFollowFinderAlias( fromFile: FSRefPtr { can be NULL }; alias: AliasHandle; logon: Boolean; var target: FSRef; var wasChanged: Boolean ): OSErr; external name '_FSFollowFinderAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  FSUpdateAlias()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSUpdateAlias( {const} fromFile: FSRefPtr { can be NULL }; const (*var*) target: FSRef; alias: AliasHandle; var wasChanged: Boolean ): OSErr; external name '_FSUpdateAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  FSNewAliasUnicode()
 *  
 *  Summary:
 *    Creates an alias given a ref to the target's parent directory and
 *    the target's unicode name.  If the target does not exist fnfErr
 *    will be returned but the alias will still be created.  This
 *    allows the creation of aliases to targets that do not exist.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    fromFile:
 *      The starting point for a relative search.
 *    
 *    targetParentRef:
 *      An FSRef to the parent directory of the target.
 *    
 *    targetNameLength:
 *      Number of Unicode characters in the target's name.
 *    
 *    targetName:
 *      A pointer to the Unicode name.
 *    
 *    inAlias:
 *      A Handle to the newly created alias record.
 *    
 *    isDirectory:
 *      On input, if target does not exist, a flag to indicate whether
 *      or not the target is a directory.  On output, if the target did
 *      exist, a flag indicating if the target is a directory.  Pass
 *      NULL in the non-existant case if unsure.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function FSNewAliasUnicode( {const} fromFile: FSRefPtr { can be NULL }; const (*var*) targetParentRef: FSRef; targetNameLength: UniCharCount; targetName: ConstUniCharPtr; var inAlias: AliasHandle; isDirectory: BooleanPtr { can be NULL } ): OSErr; external name '_FSNewAliasUnicode';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  FSNewAliasMinimalUnicode()
 *  
 *  Summary:
 *    Creates a minimal alias given a ref to the target's parent
 *    directory and the target's unicode name.  If the target does not
 *    exist fnfErr will be returned but the alias will still be created.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    targetParentRef:
 *      An FSRef to the parent directory of the target.
 *    
 *    targetNameLength:
 *      Number of Unicode characters in the target's name.
 *    
 *    targetName:
 *      A pointer to the Unicode name.
 *    
 *    inAlias:
 *      A Handle to the newly created alias record.
 *    
 *    isDirectory:
 *      On input, if target does not exist, a flag to indicate whether
 *      or not the target is a directory.  On output, if the target did
 *      exist, a flag indicating if the target is a directory.  Pass
 *      NULL in the non-existant case if unsure.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function FSNewAliasMinimalUnicode( const (*var*) targetParentRef: FSRef; targetNameLength: UniCharCount; targetName: ConstUniCharPtr; var inAlias: AliasHandle; isDirectory: BooleanPtr { can be NULL } ): OSErr; external name '_FSNewAliasMinimalUnicode';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  FSNewAliasFromPath()
 *  
 *  Summary:
 *    Creates an alias given a POSIX style utf-8 path to the target. 
 *    If the target file does not exist but the path up to the leaf
 *    does then fnfErr will be returned but the alias will still be
 *    created.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    fromFilePath:
 *      The starting point for a relative search.
 *    
 *    targetPath:
 *      POSIX style UTF-8 path to target.
 *    
 *    flags:
 *      Options for future use.  Pass in 0.
 *    
 *    inAlias:
 *      A Handle to the newly created alias record.
 *    
 *    isDirectory:
 *      On input, if target does not exist, a flag to indicate whether
 *      or not the target is a directory.  On output, if the target did
 *      exist, a flag indicating if the target is a directory.  Pass
 *      NULL in the non-existant case if unsure.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.5 and later
 *    Non-Carbon CFM:   not available
 }
function FSNewAliasFromPath( fromFilePath: ConstCStringPtr { can be NULL }; targetPath: ConstCStringPtr; flags: OptionBits; var inAlias: AliasHandle; isDirectory: BooleanPtr { can be NULL } ): OSStatus; external name '_FSNewAliasFromPath';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  FSMatchAliasBulk()
 *  
 *  Summary:
 *    Given an alias handle and fromFile, match the alias and return
 *    FSRefs to the aliased file(s) and needsUpdate flag
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function FSMatchAliasBulk( {const} fromFile: FSRefPtr { can be NULL }; rulesMask: UNSIGNEDLONG; inAlias: AliasHandle; var aliasCount: SInt16; var aliasList: FSRef; var needsUpdate: Boolean; aliasFilter: FSAliasFilterProcPtr { can be NULL }; yourDataPtr: UnivPtr { can be NULL } ): OSStatus; external name '_FSMatchAliasBulk';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  FSCopyAliasInfo()
 *  
 *  Discussion:
 *    This routine will return the requested information from the
 *    passed in aliasHandle.  The information is gathered only from the
 *    alias record so it may not match what is on disk (no disk i/o is
 *    performed).  The whichInfo paramter is an output parameter that
 *    signifies which fields in the info record contain valid data.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    inAlias:
 *      A handle to the alias record to get the information from.
 *    
 *    targetName:
 *      The name of the target item.
 *    
 *    volumeName:
 *      The name of the volume the target resides on.
 *    
 *    pathString:
 *      POSIX path to target.
 *    
 *    whichInfo:
 *      An indication of which fields in the info block contain valid
 *      data.
 *    
 *    info:
 *      Returned information about the alias.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function FSCopyAliasInfo( inAlias: AliasHandle; targetName: HFSUniStr255Ptr { can be NULL }; volumeName: HFSUniStr255Ptr { can be NULL }; pathString: CFStringRefPtr { can be NULL }; whichInfo: FSAliasInfoBitmapPtr { can be NULL }; info: FSAliasInfoPtr { can be NULL } ): OSStatus; external name '_FSCopyAliasInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  GetAliasSize()
 *  
 *  Discussion:
 *    This routine will return the size of the alias record referenced
 *    by the AliasHandle alias.  This will be smaller than the size
 *    returned by GetHandleSize if any custom data has been added (IM
 *    Files 4-13).
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    alias:
 *      A handle to the alias record to get the information from.
 *  
 *  Result:
 *    The size of the private section of the alias record.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function GetAliasSize( alias: AliasHandle ): Size; external name '_GetAliasSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  GetAliasUserType()
 *  
 *  Discussion:
 *    This routine will return the usertype associated with the alias
 *    record referenced by the AliasHandle alias.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    alias:
 *      A handle to the alias record to get the userType from.
 *  
 *  Result:
 *    The userType associated with the alias as an OSType
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function GetAliasUserType( alias: AliasHandle ): OSType; external name '_GetAliasUserType';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  SetAliasUserType()
 *  
 *  Discussion:
 *    This routine will set the userType associated with an alias
 *    record.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    alias:
 *      A handle to the alias record to set the userType for.
 *    
 *    userType:
 *      The OSType to set the userType to.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
procedure SetAliasUserType( alias: AliasHandle; userType: OSType ); external name '_SetAliasUserType';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  GetAliasSizeFromPtr()
 *  
 *  Discussion:
 *    This routine will return the size of the alias record referenced
 *    by a pointer to the AliasRecord.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    alias:
 *      A pointer to the alias record to get the information from.
 *  
 *  Result:
 *    The size of the private section of the alias record.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function GetAliasSizeFromPtr( const (*var*) alias: AliasRecord ): Size; external name '_GetAliasSizeFromPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  GetAliasUserTypeFromPtr()
 *  
 *  Discussion:
 *    This routine will return the usertype associated withthe alias
 *    record pointed to by alias.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    alias:
 *      A pointer to the alias record to get the userType from.
 *  
 *  Result:
 *    The userType associated with the alias as an OSType
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function GetAliasUserTypeFromPtr( const (*var*) alias: AliasRecord ): OSType; external name '_GetAliasUserTypeFromPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  SetAliasUserTypeWithPtr()
 *  
 *  Discussion:
 *    This routine will set the userType associated with an alias
 *    record.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    alias:
 *      A pointer to the alias record to set the userType for.
 *    
 *    userType:
 *      The OSType to set the userType to.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
procedure SetAliasUserTypeWithPtr( alias: AliasPtr; userType: OSType ); external name '_SetAliasUserTypeWithPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{ Functions beyond this point are deprecated}

{$ifc not TARGET_CPU_64}
{
 *  FSMatchAlias()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSMatchAliasBulk instead
 *  
 *  Summary:
 *    Given an alias handle and fromFile, match the alias and return
 *    FSRefs to the aliased file(s) and needsUpdate flag
 *  
>>>>>>> origin/cpstrnew
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSIsAliasFile( const (*var*) fileRef: FSRef; var aliasFileFlag: Boolean; var folderFlag: Boolean ): OSErr; external name '_FSIsAliasFile';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)


{
 *  FSResolveAliasWithMountFlags()
 *  
=======
{
 *  NewAliasFilterUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
{ old name was NewAliasFilterProc }
function NewAliasFilterUPP( userRoutine: AliasFilterProcPtr ): AliasFilterUPP; external name '_NewAliasFilterUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeAliasFilterUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeAliasFilterUPP( userUPP: AliasFilterUPP ); external name '_DisposeAliasFilterUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeAliasFilterUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeAliasFilterUPP( cpbPtr: CInfoPBPtr; var quitFlag: Boolean; myDataPtr: Ptr; userUPP: AliasFilterUPP ): Boolean; external name '_InvokeAliasFilterUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{#if __MACH__
    #define NewAliasFilterUPP(userRoutine)                      ((AliasFilterUPP)userRoutine)
    #define DisposeAliasFilterUPP(userUPP)
    #define InvokeAliasFilterUPP(cpbPtr, quitFlag, myDataPtr, userUPP) (*userUPP)(cpbPtr, quitFlag, myDataPtr)
#endif}

{$endc} {not TARGET_CPU_64}

type
	FSAliasFilterProcPtr = function( const (*var*) ref: FSRef; var quitFlag: Boolean; myDataPtr: Ptr ): Boolean;
	FSAliasFilterUPP = FSAliasFilterProcPtr;
=======
 *    Mac OS X:         in version 10.2 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function FSMatchAlias( {const} fromFile: FSRefPtr { can be NULL }; rulesMask: UNSIGNEDLONG; inAlias: AliasHandle; var aliasCount: SInt16; var aliasList: FSRef; var needsUpdate: Boolean; aliasFilter: AliasFilterUPP; yourDataPtr: UnivPtr ): OSErr; external name '_FSMatchAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FSMatchAliasNoUI()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSMatchAliasBulk with the kARMNoUI flag instead
 *  
 *  Summary:
 *    variation on FSMatchAlias that does not prompt user with a dialog
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function FSMatchAliasNoUI( {const} fromFile: FSRefPtr { can be NULL }; rulesMask: UNSIGNEDLONG; inAlias: AliasHandle; var aliasCount: SInt16; var aliasList: FSRef; var needsUpdate: Boolean; aliasFilter: AliasFilterUPP; yourDataPtr: UnivPtr ): OSErr; external name '_FSMatchAliasNoUI';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NewAlias()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSNewAlias
 *  
 *  Summary:
 *    create a new alias between fromFile and target, returns alias
 *    record handle
 *  
 *  Discussion:
 *    Create an alias betwen fromFile and target, and return it in an
 *    AliasHandle. This function is deprecated in Mac OS X 10.4;
 *    instead, you should use FSNewAliasUnicode() because NewAlias()
 *    has problems creating aliases to certain files, including those
 *    which are impossible to represent in an FSSpec.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function NewAlias( {const} fromFile: FSSpecPtr { can be NULL }; const (*var*) target: FSSpec; var alias: AliasHandle ): OSErr; external name '_NewAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  NewAliasMinimal()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSNewAliasMinimalUnicode
 *  
 *  Summary:
 *    create a minimal new alias for a target and return alias record
 *    handle
 *  
 *  Discussion:
 *    Create a minimal alias for a target, and return it in an
 *    AliasHandle. This function is deprecated in Mac OS X 10.4;
 *    instead, you should use FSNewAliasMinimalUnicode() because
 *    NewAliasMinimalAlias() has problems creating aliases to certain
 *    files, including those which are impossible to represent in an
 *    FSSpec.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function NewAliasMinimal( const (*var*) target: FSSpec; var alias: AliasHandle ): OSErr; external name '_NewAliasMinimal';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  NewAliasMinimalFromFullPath()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSNewAliasMinimalUnicode
 *  
 *  Summary:
 *    create a minimal new alias from a target fullpath (optional zone
 *    and server name) and return alias record handle
 *  
 *  Discussion:
 *    Create a minimal alias for a target fullpath, and return it in an
 *    AliasHandle. This function is deprecated in Mac OS X 10.4;
 *    instead, you should use FSNewAliasMinimalUnicode() because
 *    NewAliasMinimalFromFullPath() has problems creating aliases to
 *    certain files, including those which are impossible to represent
 *    in an FSSpec.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function NewAliasMinimalFromFullPath( fullPathLength: SInt16; fullPath: {const} UnivPtr; const (*var*) zoneName: Str32; const (*var*) serverName: Str31; var alias: AliasHandle ): OSErr; external name '_NewAliasMinimalFromFullPath';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  ResolveAlias()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSResolveAlias() or FSResolveAliasWithMountFlags() instead.
 *  
 *  Summary:
 *    given an alias handle and fromFile, resolve the alias, update the
 *    alias record and return aliased filename and wasChanged flag.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function ResolveAlias( {const} fromFile: FSSpecPtr { can be NULL }; alias: AliasHandle; var target: FSSpec; var wasChanged: Boolean ): OSErr; external name '_ResolveAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  GetAliasInfo()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSCopyAliasInfo instead.
 *  
 *  Summary:
 *    This call does not work on all aliases. Given an alias handle and
 *    an index specifying requested alias information type, return the
 *    information from alias record as a string. An empty string is
 *    returned when the index is greater than the number of levels
 *    between the target and root.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetAliasInfo( alias: AliasHandle; itemIndex: AliasInfoType; var theString: Str63 ): OSErr; external name '_GetAliasInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)

>>>>>>> origin/cpstrnew

{
 *  IsAliasFile()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSIsAliasFile() instead.
 *  
 *  Summary:
 *    Return true if the file pointed to by fileFSSpec is an alias file.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSNewAlias( {const} fromFile: FSRefPtr { can be NULL }; const (*var*) target: FSRef; var inAlias: AliasHandle ): OSErr; external name '_FSNewAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
function IsAliasFile( const (*var*) fileFSSpec: FSSpec; var aliasFileFlag: Boolean; var folderFlag: Boolean ): OSErr; external name '_IsAliasFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{
 *  ResolveAliasWithMountFlags()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSResolveAliasWithMountFlags() instead.
 *  
 *  Summary:
 *    Given an AliasHandle, return target file spec. It resolves the
 *    entire alias chain or one step of the chain.  It returns info
 *    about whether the target is a folder or file; and whether the
 *    input file spec was an alias or not.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSNewAliasMinimal( const (*var*) target: FSRef; var inAlias: AliasHandle ): OSErr; external name '_FSNewAliasMinimal';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
function ResolveAliasWithMountFlags( {const} fromFile: FSSpecPtr { can be NULL }; alias: AliasHandle; var target: FSSpec; var wasChanged: Boolean; mountFlags: UNSIGNEDLONG ): OSErr; external name '_ResolveAliasWithMountFlags';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{
 *  ResolveAliasFile()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSResolveAliasFile
 *  
 *  Summary:
 *    Given a file spec, return target file spec if input file spec is
 *    an alias. It resolves the entire alias chain or one step of the
 *    chain.  It returns info about whether the target is a folder or
 *    file; and whether the input file spec was an alias or not.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSIsAliasFile( const (*var*) fileRef: FSRef; var aliasFileFlag: Boolean; var folderFlag: Boolean ): OSErr; external name '_FSIsAliasFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function ResolveAliasFile( var theSpec: FSSpec; resolveAliasChains: Boolean; var targetIsFolder: Boolean; var wasAliased: Boolean ): OSErr; external name '_ResolveAliasFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{ Deprecated: Use FSResolveAliasFileWithMountFlags instead}
>>>>>>> origin/cpstrnew
{
 *  ResolveAliasFileWithMountFlags()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSResolveAliasWithMountFlags( {const} fromFile: FSRefPtr { can be NULL }; inAlias: AliasHandle; var target: FSRef; var wasChanged: Boolean; mountFlags: UNSIGNEDLONG ): OSErr; external name '_FSResolveAliasWithMountFlags';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
function ResolveAliasFileWithMountFlags( var theSpec: FSSpec; resolveAliasChains: Boolean; var targetIsFolder: Boolean; var wasAliased: Boolean; mountFlags: UNSIGNEDLONG ): OSErr; external name '_ResolveAliasFileWithMountFlags';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ Deprecated:  Use FSFollowFinderAlias instead}
>>>>>>> origin/cpstrnew
{
 *  FollowFinderAlias()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSResolveAlias( {const} fromFile: FSRefPtr { can be NULL }; alias: AliasHandle; var target: FSRef; var wasChanged: Boolean ): OSErr; external name '_FSResolveAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
function FollowFinderAlias( {const} fromFile: FSSpecPtr { can be NULL }; alias: AliasHandle; logon: Boolean; var target: FSSpec; var wasChanged: Boolean ): OSErr; external name '_FollowFinderAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ 
   Low Level Routines 
}
>>>>>>> origin/cpstrnew
{
 *  UpdateAlias()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    UseFSUpdateAlias
 *  
 *  Summary:
 *    given a fromFile-target pair and an alias handle, update the
 *    alias record pointed to by alias handle to represent target as
 *    the new alias.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSResolveAliasFileWithMountFlags( var theRef: FSRef; resolveAliasChains: Boolean; var targetIsFolder: Boolean; var wasAliased: Boolean; mountFlags: UNSIGNEDLONG ): OSErr; external name '_FSResolveAliasFileWithMountFlags';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function UpdateAlias( {const} fromFile: FSSpecPtr { can be NULL }; const (*var*) target: FSSpec; alias: AliasHandle; var wasChanged: Boolean ): OSErr; external name '_UpdateAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{
 *  MatchAlias()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSMatchAliasBulk instead
 *  
 *  Summary:
 *    Given an alias handle and fromFile, match the alias and return
 *    FSSpecs to the aliased file(s) and needsUpdate flag
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSResolveAliasFile( var theRef: FSRef; resolveAliasChains: Boolean; var targetIsFolder: Boolean; var wasAliased: Boolean ): OSErr; external name '_FSResolveAliasFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function MatchAlias( {const} fromFile: FSSpecPtr { can be NULL }; rulesMask: UNSIGNEDLONG; alias: AliasHandle; var aliasCount: SInt16; aliasList: FSSpecArrayPtr; var needsUpdate: Boolean; aliasFilter: AliasFilterUPP; yourDataPtr: UnivPtr ): OSErr; external name '_MatchAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{
 *  ResolveAliasFileWithMountFlagsNoUI()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSResolveAliasFileWithMountFlags passing in the
 *    kResolveAliasFileNoUI flag
 *  
 *  Summary:
 *    variation on ResolveAliasFile that does not prompt user with a
 *    dialog
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSFollowFinderAlias( fromFile: FSRefPtr { can be NULL }; alias: AliasHandle; logon: Boolean; var target: FSRef; var wasChanged: Boolean ): OSErr; external name '_FSFollowFinderAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function ResolveAliasFileWithMountFlagsNoUI( var theSpec: FSSpec; resolveAliasChains: Boolean; var targetIsFolder: Boolean; var wasAliased: Boolean; mountFlags: UNSIGNEDLONG ): OSErr; external name '_ResolveAliasFileWithMountFlagsNoUI';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> origin/cpstrnew


{
 *  MatchAliasNoUI()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSMatchAliasBulk with the kARMNoUI flag instead
 *  
 *  Summary:
 *    variation on MatchAlias that does not prompt user with a dialog
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSUpdateAlias( {const} fromFile: FSRefPtr { can be NULL }; const (*var*) target: FSRef; alias: AliasHandle; var wasChanged: Boolean ): OSErr; external name '_FSUpdateAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  FSNewAliasUnicode()
 *  
 *  Summary:
 *    Creates an alias given a ref to the target's parent directory and
 *    the target's unicode name.  If the target does not exist fnfErr
 *    will be returned but the alias will still be created.  This
 *    allows the creation of aliases to targets that do not exist.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    fromFile:
 *      The starting point for a relative search.
 *    
 *    targetParentRef:
 *      An FSRef to the parent directory of the target.
 *    
 *    targetNameLength:
 *      Number of Unicode characters in the target's name.
 *    
 *    targetName:
 *      A pointer to the Unicode name.
 *    
 *    inAlias:
 *      A Handle to the newly created alias record.
 *    
 *    isDirectory:
 *      On input, if target does not exist, a flag to indicate whether
 *      or not the target is a directory.  On output, if the target did
 *      exist, a flag indicating if the target is a directory.  Pass
 *      NULL in the non-existant case if unsure.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function FSNewAliasUnicode( {const} fromFile: FSRefPtr { can be NULL }; const (*var*) targetParentRef: FSRef; targetNameLength: UniCharCount; targetName: ConstUniCharPtr; var inAlias: AliasHandle; isDirectory: BooleanPtr { can be NULL } ): OSErr; external name '_FSNewAliasUnicode';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  FSNewAliasMinimalUnicode()
 *  
 *  Summary:
 *    Creates a minimal alias given a ref to the target's parent
 *    directory and the target's unicode name.  If the target does not
 *    exist fnfErr will be returned but the alias will still be created.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    targetParentRef:
 *      An FSRef to the parent directory of the target.
 *    
 *    targetNameLength:
 *      Number of Unicode characters in the target's name.
 *    
 *    targetName:
 *      A pointer to the Unicode name.
 *    
 *    inAlias:
 *      A Handle to the newly created alias record.
 *    
 *    isDirectory:
 *      On input, if target does not exist, a flag to indicate whether
 *      or not the target is a directory.  On output, if the target did
 *      exist, a flag indicating if the target is a directory.  Pass
 *      NULL in the non-existant case if unsure.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function FSNewAliasMinimalUnicode( const (*var*) targetParentRef: FSRef; targetNameLength: UniCharCount; targetName: ConstUniCharPtr; var inAlias: AliasHandle; isDirectory: BooleanPtr { can be NULL } ): OSErr; external name '_FSNewAliasMinimalUnicode';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  FSNewAliasFromPath()
 *  
 *  Summary:
 *    Creates an alias given a POSIX style utf-8 path to the target. 
 *    If the target file does not exist but the path up to the leaf
 *    does then fnfErr will be returned but the alias will still be
 *    created.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    fromFilePath:
 *      The starting point for a relative search.
 *    
 *    targetPath:
 *      POSIX style UTF-8 path to target.
 *    
 *    flags:
 *      Options for future use.  Pass in 0.
 *    
 *    inAlias:
 *      A Handle to the newly created alias record.
 *    
 *    isDirectory:
 *      On input, if target does not exist, a flag to indicate whether
 *      or not the target is a directory.  On output, if the target did
 *      exist, a flag indicating if the target is a directory.  Pass
 *      NULL in the non-existant case if unsure.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.5 and later
 *    Non-Carbon CFM:   not available
 }
function FSNewAliasFromPath( fromFilePath: ConstCStringPtr { can be NULL }; targetPath: ConstCStringPtr; flags: OptionBits; var inAlias: AliasHandle; isDirectory: BooleanPtr { can be NULL } ): OSStatus; external name '_FSNewAliasFromPath';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  FSMatchAliasBulk()
 *  
 *  Summary:
 *    Given an alias handle and fromFile, match the alias and return
 *    FSRefs to the aliased file(s) and needsUpdate flag
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function FSMatchAliasBulk( {const} fromFile: FSRefPtr { can be NULL }; rulesMask: UNSIGNEDLONG; inAlias: AliasHandle; var aliasCount: SInt16; var aliasList: FSRef; var needsUpdate: Boolean; aliasFilter: FSAliasFilterProcPtr { can be NULL }; yourDataPtr: UnivPtr { can be NULL } ): OSStatus; external name '_FSMatchAliasBulk';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  FSCopyAliasInfo()
 *  
 *  Discussion:
 *    This routine will return the requested information from the
 *    passed in aliasHandle.  The information is gathered only from the
 *    alias record so it may not match what is on disk (no disk i/o is
 *    performed).  The whichInfo paramter is an output parameter that
 *    signifies which fields in the info record contain valid data.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    inAlias:
 *      A handle to the alias record to get the information from.
 *    
 *    targetName:
 *      The name of the target item.
 *    
 *    volumeName:
 *      The name of the volume the target resides on.
 *    
 *    pathString:
 *      POSIX path to target.
 *    
 *    whichInfo:
 *      An indication of which fields in the info block contain valid
 *      data.
 *    
 *    info:
 *      Returned information about the alias.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function FSCopyAliasInfo( inAlias: AliasHandle; targetName: HFSUniStr255Ptr { can be NULL }; volumeName: HFSUniStr255Ptr { can be NULL }; pathString: CFStringRefPtr { can be NULL }; whichInfo: FSAliasInfoBitmapPtr { can be NULL }; info: FSAliasInfoPtr { can be NULL } ): OSStatus; external name '_FSCopyAliasInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  GetAliasSize()
 *  
 *  Discussion:
 *    This routine will return the size of the alias record referenced
 *    by the AliasHandle alias.  This will be smaller than the size
 *    returned by GetHandleSize if any custom data has been added (IM
 *    Files 4-13).
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    alias:
 *      A handle to the alias record to get the information from.
 *  
 *  Result:
 *    The size of the private section of the alias record.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function GetAliasSize( alias: AliasHandle ): Size; external name '_GetAliasSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  GetAliasUserType()
 *  
 *  Discussion:
 *    This routine will return the usertype associated with the alias
 *    record referenced by the AliasHandle alias.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    alias:
 *      A handle to the alias record to get the userType from.
 *  
 *  Result:
 *    The userType associated with the alias as an OSType
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function GetAliasUserType( alias: AliasHandle ): OSType; external name '_GetAliasUserType';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  SetAliasUserType()
 *  
 *  Discussion:
 *    This routine will set the userType associated with an alias
 *    record.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    alias:
 *      A handle to the alias record to set the userType for.
 *    
 *    userType:
 *      The OSType to set the userType to.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
procedure SetAliasUserType( alias: AliasHandle; userType: OSType ); external name '_SetAliasUserType';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  GetAliasSizeFromPtr()
 *  
 *  Discussion:
 *    This routine will return the size of the alias record referenced
 *    by a pointer to the AliasRecord.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    alias:
 *      A pointer to the alias record to get the information from.
 *  
 *  Result:
 *    The size of the private section of the alias record.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function GetAliasSizeFromPtr( const (*var*) alias: AliasRecord ): Size; external name '_GetAliasSizeFromPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  GetAliasUserTypeFromPtr()
 *  
 *  Discussion:
 *    This routine will return the usertype associated withthe alias
 *    record pointed to by alias.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    alias:
 *      A pointer to the alias record to get the userType from.
 *  
 *  Result:
 *    The userType associated with the alias as an OSType
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function GetAliasUserTypeFromPtr( const (*var*) alias: AliasRecord ): OSType; external name '_GetAliasUserTypeFromPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  SetAliasUserTypeWithPtr()
 *  
 *  Discussion:
 *    This routine will set the userType associated with an alias
 *    record.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    alias:
 *      A pointer to the alias record to set the userType for.
 *    
 *    userType:
 *      The OSType to set the userType to.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
procedure SetAliasUserTypeWithPtr( alias: AliasPtr; userType: OSType ); external name '_SetAliasUserTypeWithPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{ Functions beyond this point are deprecated}

{$ifc not TARGET_CPU_64}
{
 *  FSMatchAlias()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSMatchAliasBulk instead
 *  
 *  Summary:
 *    Given an alias handle and fromFile, match the alias and return
 *    FSRefs to the aliased file(s) and needsUpdate flag
 *  
>>>>>>> graemeg/cpstrnew
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSResolveAliasWithMountFlags( {const} fromFile: FSRefPtr { can be NULL }; inAlias: AliasHandle; var target: FSRef; var wasChanged: Boolean; mountFlags: UNSIGNEDLONG ): OSErr; external name '_FSResolveAliasWithMountFlags';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)


{
 *  FSResolveAlias()
 *  
=======
 }
procedure DisposeAliasFilterUPP( userUPP: AliasFilterUPP ); external name '_DisposeAliasFilterUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
=======
 *    Mac OS X:         in version 10.2 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function FSMatchAlias( {const} fromFile: FSRefPtr { can be NULL }; rulesMask: UNSIGNEDLONG; inAlias: AliasHandle; var aliasCount: SInt16; var aliasList: FSRef; var needsUpdate: Boolean; aliasFilter: AliasFilterUPP; yourDataPtr: UnivPtr ): OSErr; external name '_FSMatchAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FSMatchAliasNoUI()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSMatchAliasBulk with the kARMNoUI flag instead
 *  
 *  Summary:
 *    variation on FSMatchAlias that does not prompt user with a dialog
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function FSMatchAliasNoUI( {const} fromFile: FSRefPtr { can be NULL }; rulesMask: UNSIGNEDLONG; inAlias: AliasHandle; var aliasCount: SInt16; var aliasList: FSRef; var needsUpdate: Boolean; aliasFilter: AliasFilterUPP; yourDataPtr: UnivPtr ): OSErr; external name '_FSMatchAliasNoUI';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

>>>>>>> graemeg/cpstrnew

{
 *  NewAlias()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSNewAlias
 *  
 *  Summary:
 *    create a new alias between fromFile and target, returns alias
 *    record handle
 *  
 *  Discussion:
 *    Create an alias betwen fromFile and target, and return it in an
 *    AliasHandle. This function is deprecated in Mac OS X 10.4;
 *    instead, you should use FSNewAliasUnicode() because NewAlias()
 *    has problems creating aliases to certain files, including those
 *    which are impossible to represent in an FSSpec.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeAliasFilterUPP( cpbPtr: CInfoPBPtr; var quitFlag: Boolean; myDataPtr: Ptr; userUPP: AliasFilterUPP ): Boolean; external name '_InvokeAliasFilterUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{#if __MACH__
    #define NewAliasFilterUPP(userRoutine)                      ((AliasFilterUPP)userRoutine)
    #define DisposeAliasFilterUPP(userUPP)
    #define InvokeAliasFilterUPP(cpbPtr, quitFlag, myDataPtr, userUPP) (*userUPP)(cpbPtr, quitFlag, myDataPtr)
#endif}

{$endc} {not TARGET_CPU_64}

type
	FSAliasFilterProcPtr = function( const (*var*) ref: FSRef; var quitFlag: Boolean; myDataPtr: Ptr ): Boolean;
	FSAliasFilterUPP = FSAliasFilterProcPtr;

{
 *  FSNewAlias()
 *  
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function NewAlias( {const} fromFile: FSSpecPtr { can be NULL }; const (*var*) target: FSSpec; var alias: AliasHandle ): OSErr; external name '_NewAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  NewAliasMinimal()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSNewAliasMinimalUnicode
 *  
 *  Summary:
 *    create a minimal new alias for a target and return alias record
 *    handle
 *  
 *  Discussion:
 *    Create a minimal alias for a target, and return it in an
 *    AliasHandle. This function is deprecated in Mac OS X 10.4;
 *    instead, you should use FSNewAliasMinimalUnicode() because
 *    NewAliasMinimalAlias() has problems creating aliases to certain
 *    files, including those which are impossible to represent in an
 *    FSSpec.
 *  
>>>>>>> graemeg/cpstrnew
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSNewAlias( {const} fromFile: FSRefPtr { can be NULL }; const (*var*) target: FSRef; var inAlias: AliasHandle ): OSErr; external name '_FSNewAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  FSNewAliasMinimal()
 *  
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function NewAliasMinimal( const (*var*) target: FSSpec; var alias: AliasHandle ): OSErr; external name '_NewAliasMinimal';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  NewAliasMinimalFromFullPath()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSNewAliasMinimalUnicode
 *  
 *  Summary:
 *    create a minimal new alias from a target fullpath (optional zone
 *    and server name) and return alias record handle
 *  
 *  Discussion:
 *    Create a minimal alias for a target fullpath, and return it in an
 *    AliasHandle. This function is deprecated in Mac OS X 10.4;
 *    instead, you should use FSNewAliasMinimalUnicode() because
 *    NewAliasMinimalFromFullPath() has problems creating aliases to
 *    certain files, including those which are impossible to represent
 *    in an FSSpec.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function NewAliasMinimalFromFullPath( fullPathLength: SInt16; fullPath: {const} UnivPtr; const (*var*) zoneName: Str32; const (*var*) serverName: Str31; var alias: AliasHandle ): OSErr; external name '_NewAliasMinimalFromFullPath';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  ResolveAlias()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSResolveAlias() or FSResolveAliasWithMountFlags() instead.
 *  
 *  Summary:
 *    given an alias handle and fromFile, resolve the alias, update the
 *    alias record and return aliased filename and wasChanged flag.
 *  
>>>>>>> graemeg/cpstrnew
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSNewAliasMinimal( const (*var*) target: FSRef; var inAlias: AliasHandle ): OSErr; external name '_FSNewAliasMinimal';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  FSIsAliasFile()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSIsAliasFile( const (*var*) fileRef: FSRef; var aliasFileFlag: Boolean; var folderFlag: Boolean ): OSErr; external name '_FSIsAliasFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  FSResolveAliasWithMountFlags()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSResolveAliasWithMountFlags( {const} fromFile: FSRefPtr { can be NULL }; inAlias: AliasHandle; var target: FSRef; var wasChanged: Boolean; mountFlags: UNSIGNEDLONG ): OSErr; external name '_FSResolveAliasWithMountFlags';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  FSResolveAlias()
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function ResolveAlias( {const} fromFile: FSSpecPtr { can be NULL }; alias: AliasHandle; var target: FSSpec; var wasChanged: Boolean ): OSErr; external name '_ResolveAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  GetAliasInfo()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSCopyAliasInfo instead.
 *  
 *  Summary:
 *    This call does not work on all aliases. Given an alias handle and
 *    an index specifying requested alias information type, return the
 *    information from alias record as a string. An empty string is
 *    returned when the index is greater than the number of levels
 *    between the target and root.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetAliasInfo( alias: AliasHandle; itemIndex: AliasInfoType; var theString: Str63 ): OSErr; external name '_GetAliasInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


{
 *  IsAliasFile()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSIsAliasFile() instead.
 *  
 *  Summary:
 *    Return true if the file pointed to by fileFSSpec is an alias file.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
function IsAliasFile( const (*var*) fileFSSpec: FSSpec; var aliasFileFlag: Boolean; var folderFlag: Boolean ): OSErr; external name '_IsAliasFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  ResolveAliasWithMountFlags()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSResolveAliasWithMountFlags() instead.
 *  
 *  Summary:
 *    Given an AliasHandle, return target file spec. It resolves the
 *    entire alias chain or one step of the chain.  It returns info
 *    about whether the target is a folder or file; and whether the
 *    input file spec was an alias or not.
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSResolveAlias( {const} fromFile: FSRefPtr { can be NULL }; alias: AliasHandle; var target: FSRef; var wasChanged: Boolean ): OSErr; external name '_FSResolveAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  FSResolveAliasFileWithMountFlags()
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
function ResolveAliasWithMountFlags( {const} fromFile: FSSpecPtr { can be NULL }; alias: AliasHandle; var target: FSSpec; var wasChanged: Boolean; mountFlags: UNSIGNEDLONG ): OSErr; external name '_ResolveAliasWithMountFlags';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  ResolveAliasFile()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSResolveAliasFile
 *  
 *  Summary:
 *    Given a file spec, return target file spec if input file spec is
 *    an alias. It resolves the entire alias chain or one step of the
 *    chain.  It returns info about whether the target is a folder or
 *    file; and whether the input file spec was an alias or not.
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSResolveAliasFileWithMountFlags( var theRef: FSRef; resolveAliasChains: Boolean; var targetIsFolder: Boolean; var wasAliased: Boolean; mountFlags: UNSIGNEDLONG ): OSErr; external name '_FSResolveAliasFileWithMountFlags';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  FSResolveAliasFile()
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function ResolveAliasFile( var theSpec: FSSpec; resolveAliasChains: Boolean; var targetIsFolder: Boolean; var wasAliased: Boolean ): OSErr; external name '_ResolveAliasFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{ Deprecated: Use FSResolveAliasFileWithMountFlags instead}
{
 *  ResolveAliasFileWithMountFlags()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSResolveAliasFile( var theRef: FSRef; resolveAliasChains: Boolean; var targetIsFolder: Boolean; var wasAliased: Boolean ): OSErr; external name '_FSResolveAliasFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  FSFollowFinderAlias()
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
function ResolveAliasFileWithMountFlags( var theSpec: FSSpec; resolveAliasChains: Boolean; var targetIsFolder: Boolean; var wasAliased: Boolean; mountFlags: UNSIGNEDLONG ): OSErr; external name '_ResolveAliasFileWithMountFlags';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ Deprecated:  Use FSFollowFinderAlias instead}
{
 *  FollowFinderAlias()   *** DEPRECATED ***
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSFollowFinderAlias( fromFile: FSRefPtr { can be NULL }; alias: AliasHandle; logon: Boolean; var target: FSRef; var wasChanged: Boolean ): OSErr; external name '_FSFollowFinderAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  FSUpdateAlias()
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
function FollowFinderAlias( {const} fromFile: FSSpecPtr { can be NULL }; alias: AliasHandle; logon: Boolean; var target: FSSpec; var wasChanged: Boolean ): OSErr; external name '_FollowFinderAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ 
   Low Level Routines 
}
{
 *  UpdateAlias()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    UseFSUpdateAlias
 *  
 *  Summary:
 *    given a fromFile-target pair and an alias handle, update the
 *    alias record pointed to by alias handle to represent target as
 *    the new alias.
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSUpdateAlias( {const} fromFile: FSRefPtr { can be NULL }; const (*var*) target: FSRef; alias: AliasHandle; var wasChanged: Boolean ): OSErr; external name '_FSUpdateAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  FSNewAliasUnicode()
 *  
 *  Summary:
 *    Creates an alias given a ref to the target's parent directory and
 *    the target's unicode name.  If the target does not exist fnfErr
 *    will be returned but the alias will still be created.  This
 *    allows the creation of aliases to targets that do not exist.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    fromFile:
 *      The starting point for a relative search.
 *    
 *    targetParentRef:
 *      An FSRef to the parent directory of the target.
 *    
 *    targetNameLength:
 *      Number of Unicode characters in the target's name.
 *    
 *    targetName:
 *      A pointer to the Unicode name.
 *    
 *    inAlias:
 *      A Handle to the newly created alias record.
 *    
 *    isDirectory:
 *      On input, if target does not exist, a flag to indicate whether
 *      or not the target is a directory.  On output, if the target did
 *      exist, a flag indicating if the target is a directory.  Pass
 *      NULL in the non-existant case if unsure.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function FSNewAliasUnicode( {const} fromFile: FSRefPtr { can be NULL }; const (*var*) targetParentRef: FSRef; targetNameLength: UniCharCount; targetName: ConstUniCharPtr; var inAlias: AliasHandle; isDirectory: BooleanPtr { can be NULL } ): OSErr; external name '_FSNewAliasUnicode';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  FSNewAliasMinimalUnicode()
 *  
 *  Summary:
 *    Creates a minimal alias given a ref to the target's parent
 *    directory and the target's unicode name.  If the target does not
 *    exist fnfErr will be returned but the alias will still be created.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    targetParentRef:
 *      An FSRef to the parent directory of the target.
 *    
 *    targetNameLength:
 *      Number of Unicode characters in the target's name.
 *    
 *    targetName:
 *      A pointer to the Unicode name.
 *    
 *    inAlias:
 *      A Handle to the newly created alias record.
 *    
 *    isDirectory:
 *      On input, if target does not exist, a flag to indicate whether
 *      or not the target is a directory.  On output, if the target did
 *      exist, a flag indicating if the target is a directory.  Pass
 *      NULL in the non-existant case if unsure.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function FSNewAliasMinimalUnicode( const (*var*) targetParentRef: FSRef; targetNameLength: UniCharCount; targetName: ConstUniCharPtr; var inAlias: AliasHandle; isDirectory: BooleanPtr { can be NULL } ): OSErr; external name '_FSNewAliasMinimalUnicode';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  FSNewAliasFromPath()
 *  
 *  Summary:
 *    Creates an alias given a POSIX style utf-8 path to the target. 
 *    If the target file does not exist but the path up to the leaf
 *    does then fnfErr will be returned but the alias will still be
 *    created.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    fromFilePath:
 *      The starting point for a relative search.
 *    
 *    targetPath:
 *      POSIX style UTF-8 path to target.
 *    
 *    flags:
 *      Options for future use.  Pass in 0.
 *    
 *    inAlias:
 *      A Handle to the newly created alias record.
 *    
 *    isDirectory:
 *      On input, if target does not exist, a flag to indicate whether
 *      or not the target is a directory.  On output, if the target did
 *      exist, a flag indicating if the target is a directory.  Pass
 *      NULL in the non-existant case if unsure.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.5 and later
 *    Non-Carbon CFM:   not available
 }
function FSNewAliasFromPath( fromFilePath: ConstCStringPtr { can be NULL }; targetPath: ConstCStringPtr; flags: OptionBits; var inAlias: AliasHandle; isDirectory: BooleanPtr { can be NULL } ): OSStatus; external name '_FSNewAliasFromPath';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  FSMatchAliasBulk()
 *  
 *  Summary:
 *    Given an alias handle and fromFile, match the alias and return
 *    FSRefs to the aliased file(s) and needsUpdate flag
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function FSMatchAliasBulk( {const} fromFile: FSRefPtr { can be NULL }; rulesMask: UNSIGNEDLONG; inAlias: AliasHandle; var aliasCount: SInt16; var aliasList: FSRef; var needsUpdate: Boolean; aliasFilter: FSAliasFilterProcPtr { can be NULL }; yourDataPtr: UnivPtr { can be NULL } ): OSStatus; external name '_FSMatchAliasBulk';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{
 *  FSCopyAliasInfo()
 *  
 *  Discussion:
 *    This routine will return the requested information from the
 *    passed in aliasHandle.  The information is gathered only from the
 *    alias record so it may not match what is on disk (no disk i/o is
 *    performed).  The whichInfo paramter is an output parameter that
 *    signifies which fields in the info record contain valid data.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    inAlias:
 *      A handle to the alias record to get the information from.
 *    
 *    targetName:
 *      The name of the target item.
 *    
 *    volumeName:
 *      The name of the volume the target resides on.
 *    
 *    pathString:
 *      POSIX path to target.
 *    
 *    whichInfo:
 *      An indication of which fields in the info block contain valid
 *      data.
 *    
 *    info:
 *      Returned information about the alias.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function FSCopyAliasInfo( inAlias: AliasHandle; targetName: HFSUniStr255Ptr { can be NULL }; volumeName: HFSUniStr255Ptr { can be NULL }; pathString: CFStringRefPtr { can be NULL }; whichInfo: FSAliasInfoBitmapPtr { can be NULL }; info: FSAliasInfoPtr { can be NULL } ): OSStatus; external name '_FSCopyAliasInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
 *  GetAliasSize()
 *  
 *  Discussion:
 *    This routine will return the size of the alias record referenced
 *    by the AliasHandle alias.  This will be smaller than the size
 *    returned by GetHandleSize if any custom data has been added (IM
 *    Files 4-13).
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    alias:
 *      A handle to the alias record to get the information from.
 *  
 *  Result:
 *    The size of the private section of the alias record.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function GetAliasSize( alias: AliasHandle ): Size; external name '_GetAliasSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  GetAliasUserType()
 *  
 *  Discussion:
 *    This routine will return the usertype associated with the alias
 *    record referenced by the AliasHandle alias.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    alias:
 *      A handle to the alias record to get the userType from.
 *  
 *  Result:
 *    The userType associated with the alias as an OSType
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function GetAliasUserType( alias: AliasHandle ): OSType; external name '_GetAliasUserType';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  SetAliasUserType()
 *  
 *  Discussion:
 *    This routine will set the userType associated with an alias
 *    record.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    alias:
 *      A handle to the alias record to set the userType for.
 *    
 *    userType:
 *      The OSType to set the userType to.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
procedure SetAliasUserType( alias: AliasHandle; userType: OSType ); external name '_SetAliasUserType';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  GetAliasSizeFromPtr()
 *  
 *  Discussion:
 *    This routine will return the size of the alias record referenced
 *    by a pointer to the AliasRecord.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    alias:
 *      A pointer to the alias record to get the information from.
 *  
 *  Result:
 *    The size of the private section of the alias record.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function GetAliasSizeFromPtr( const (*var*) alias: AliasRecord ): Size; external name '_GetAliasSizeFromPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  GetAliasUserTypeFromPtr()
 *  
 *  Discussion:
 *    This routine will return the usertype associated withthe alias
 *    record pointed to by alias.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    alias:
 *      A pointer to the alias record to get the userType from.
 *  
 *  Result:
 *    The userType associated with the alias as an OSType
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function GetAliasUserTypeFromPtr( const (*var*) alias: AliasRecord ): OSType; external name '_GetAliasUserTypeFromPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  SetAliasUserTypeWithPtr()
 *  
 *  Discussion:
 *    This routine will set the userType associated with an alias
 *    record.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    alias:
 *      A pointer to the alias record to set the userType for.
 *    
 *    userType:
 *      The OSType to set the userType to.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
procedure SetAliasUserTypeWithPtr( alias: AliasPtr; userType: OSType ); external name '_SetAliasUserTypeWithPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{ Functions beyond this point are deprecated}

{$ifc not TARGET_CPU_64}
{
 *  FSMatchAlias()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSMatchAliasBulk instead
 *  
 *  Summary:
 *    Given an alias handle and fromFile, match the alias and return
 *    FSRefs to the aliased file(s) and needsUpdate flag
 *  
>>>>>>> graemeg/cpstrnew
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSResolveAlias( {const} fromFile: FSRefPtr { can be NULL }; alias: AliasHandle; var target: FSRef; var wasChanged: Boolean ): OSErr; external name '_FSResolveAlias';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)


{
 *  FSResolveAliasFileWithMountFlags()
=======
 *    Mac OS X:         in version 10.2 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function FSMatchAlias( {const} fromFile: FSRefPtr { can be NULL }; rulesMask: UNSIGNEDLONG; inAlias: AliasHandle; var aliasCount: SInt16; var aliasList: FSRef; var needsUpdate: Boolean; aliasFilter: AliasFilterUPP; yourDataPtr: UnivPtr ): OSErr; external name '_FSMatchAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  FSMatchAliasNoUI()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSMatchAliasBulk with the kARMNoUI flag instead
 *  
 *  Summary:
 *    variation on FSMatchAlias that does not prompt user with a dialog
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSResolveAliasFileWithMountFlags( var theRef: FSRef; resolveAliasChains: Boolean; var targetIsFolder: Boolean; var wasAliased: Boolean; mountFlags: UNSIGNEDLONG ): OSErr; external name '_FSResolveAliasFileWithMountFlags';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)


{
 *  FSResolveAliasFile()
=======
 *    Mac OS X:         in version 10.2 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function FSMatchAliasNoUI( {const} fromFile: FSRefPtr { can be NULL }; rulesMask: UNSIGNEDLONG; inAlias: AliasHandle; var aliasCount: SInt16; var aliasList: FSRef; var needsUpdate: Boolean; aliasFilter: AliasFilterUPP; yourDataPtr: UnivPtr ): OSErr; external name '_FSMatchAliasNoUI';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  NewAlias()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSNewAlias
 *  
 *  Summary:
 *    create a new alias between fromFile and target, returns alias
 *    record handle
 *  
 *  Discussion:
 *    Create an alias betwen fromFile and target, and return it in an
 *    AliasHandle. This function is deprecated in Mac OS X 10.4;
 *    instead, you should use FSNewAliasUnicode() because NewAlias()
 *    has problems creating aliases to certain files, including those
 *    which are impossible to represent in an FSSpec.
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSResolveAliasFile( var theRef: FSRef; resolveAliasChains: Boolean; var targetIsFolder: Boolean; var wasAliased: Boolean ): OSErr; external name '_FSResolveAliasFile';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)


{
 *  FSFollowFinderAlias()
 *  
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function NewAlias( {const} fromFile: FSSpecPtr { can be NULL }; const (*var*) target: FSSpec; var alias: AliasHandle ): OSErr; external name '_NewAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  NewAliasMinimal()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSNewAliasMinimalUnicode
 *  
 *  Summary:
 *    create a minimal new alias for a target and return alias record
 *    handle
 *  
 *  Discussion:
 *    Create a minimal alias for a target, and return it in an
 *    AliasHandle. This function is deprecated in Mac OS X 10.4;
 *    instead, you should use FSNewAliasMinimalUnicode() because
 *    NewAliasMinimalAlias() has problems creating aliases to certain
 *    files, including those which are impossible to represent in an
 *    FSSpec.
 *  
>>>>>>> graemeg/cpstrnew
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSFollowFinderAlias( fromFile: FSRefPtr { can be NULL }; alias: AliasHandle; logon: Boolean; var target: FSRef; var wasChanged: Boolean ): OSErr; external name '_FSFollowFinderAlias';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)


{
 *  FSUpdateAlias()
 *  
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function NewAliasMinimal( const (*var*) target: FSSpec; var alias: AliasHandle ): OSErr; external name '_NewAliasMinimal';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  NewAliasMinimalFromFullPath()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSNewAliasMinimalUnicode
 *  
 *  Summary:
 *    create a minimal new alias from a target fullpath (optional zone
 *    and server name) and return alias record handle
 *  
 *  Discussion:
 *    Create a minimal alias for a target fullpath, and return it in an
 *    AliasHandle. This function is deprecated in Mac OS X 10.4;
 *    instead, you should use FSNewAliasMinimalUnicode() because
 *    NewAliasMinimalFromFullPath() has problems creating aliases to
 *    certain files, including those which are impossible to represent
 *    in an FSSpec.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function NewAliasMinimalFromFullPath( fullPathLength: SInt16; fullPath: {const} UnivPtr; const (*var*) zoneName: Str32; const (*var*) serverName: Str31; var alias: AliasHandle ): OSErr; external name '_NewAliasMinimalFromFullPath';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  ResolveAlias()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSResolveAlias() or FSResolveAliasWithMountFlags() instead.
 *  
 *  Summary:
 *    given an alias handle and fromFile, resolve the alias, update the
 *    alias record and return aliased filename and wasChanged flag.
 *  
>>>>>>> graemeg/cpstrnew
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSUpdateAlias( {const} fromFile: FSRefPtr { can be NULL }; const (*var*) target: FSRef; alias: AliasHandle; var wasChanged: Boolean ): OSErr; external name '_FSUpdateAlias';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)


{
=======
 }
procedure DisposeAliasFilterUPP( userUPP: AliasFilterUPP ); external name '_DisposeAliasFilterUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeAliasFilterUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeAliasFilterUPP( cpbPtr: CInfoPBPtr; var quitFlag: Boolean; myDataPtr: Ptr; userUPP: AliasFilterUPP ): Boolean; external name '_InvokeAliasFilterUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{#if __MACH__
    #define NewAliasFilterUPP(userRoutine)                      ((AliasFilterUPP)userRoutine)
    #define DisposeAliasFilterUPP(userUPP)
    #define InvokeAliasFilterUPP(cpbPtr, quitFlag, myDataPtr, userUPP) (*userUPP)(cpbPtr, quitFlag, myDataPtr)
#endif}

{$endc} {not TARGET_CPU_64}

type
	FSAliasFilterProcPtr = function( const (*var*) ref: FSRef; var quitFlag: Boolean; myDataPtr: Ptr ): Boolean;
	FSAliasFilterUPP = FSAliasFilterProcPtr;
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function ResolveAlias( {const} fromFile: FSSpecPtr { can be NULL }; alias: AliasHandle; var target: FSSpec; var wasChanged: Boolean ): OSErr; external name '_ResolveAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  GetAliasInfo()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSCopyAliasInfo instead.
 *  
 *  Summary:
 *    This call does not work on all aliases. Given an alias handle and
 *    an index specifying requested alias information type, return the
 *    information from alias record as a string. An empty string is
 *    returned when the index is greater than the number of levels
 *    between the target and root.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetAliasInfo( alias: AliasHandle; itemIndex: AliasInfoType; var theString: Str63 ): OSErr; external name '_GetAliasInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)

>>>>>>> graemeg/cpstrnew

{
 *  IsAliasFile()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSIsAliasFile() instead.
 *  
 *  Summary:
 *    Return true if the file pointed to by fileFSSpec is an alias file.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSNewAlias( {const} fromFile: FSRefPtr { can be NULL }; const (*var*) target: FSRef; var inAlias: AliasHandle ): OSErr; external name '_FSNewAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
function IsAliasFile( const (*var*) fileFSSpec: FSSpec; var aliasFileFlag: Boolean; var folderFlag: Boolean ): OSErr; external name '_IsAliasFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  ResolveAliasWithMountFlags()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSResolveAliasWithMountFlags() instead.
 *  
 *  Summary:
 *    Given an AliasHandle, return target file spec. It resolves the
 *    entire alias chain or one step of the chain.  It returns info
 *    about whether the target is a folder or file; and whether the
 *    input file spec was an alias or not.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSNewAliasMinimal( const (*var*) target: FSRef; var inAlias: AliasHandle ): OSErr; external name '_FSNewAliasMinimal';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
function ResolveAliasWithMountFlags( {const} fromFile: FSSpecPtr { can be NULL }; alias: AliasHandle; var target: FSSpec; var wasChanged: Boolean; mountFlags: UNSIGNEDLONG ): OSErr; external name '_ResolveAliasWithMountFlags';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  ResolveAliasFile()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSResolveAliasFile
 *  
 *  Summary:
 *    Given a file spec, return target file spec if input file spec is
 *    an alias. It resolves the entire alias chain or one step of the
 *    chain.  It returns info about whether the target is a folder or
 *    file; and whether the input file spec was an alias or not.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSIsAliasFile( const (*var*) fileRef: FSRef; var aliasFileFlag: Boolean; var folderFlag: Boolean ): OSErr; external name '_FSIsAliasFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function ResolveAliasFile( var theSpec: FSSpec; resolveAliasChains: Boolean; var targetIsFolder: Boolean; var wasAliased: Boolean ): OSErr; external name '_ResolveAliasFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{ Deprecated: Use FSResolveAliasFileWithMountFlags instead}
>>>>>>> graemeg/cpstrnew
{
 *  ResolveAliasFileWithMountFlags()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSResolveAliasWithMountFlags( {const} fromFile: FSRefPtr { can be NULL }; inAlias: AliasHandle; var target: FSRef; var wasChanged: Boolean; mountFlags: UNSIGNEDLONG ): OSErr; external name '_FSResolveAliasWithMountFlags';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
function ResolveAliasFileWithMountFlags( var theSpec: FSSpec; resolveAliasChains: Boolean; var targetIsFolder: Boolean; var wasAliased: Boolean; mountFlags: UNSIGNEDLONG ): OSErr; external name '_ResolveAliasFileWithMountFlags';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ Deprecated:  Use FSFollowFinderAlias instead}
>>>>>>> graemeg/cpstrnew
{
 *  FollowFinderAlias()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSResolveAlias( {const} fromFile: FSRefPtr { can be NULL }; alias: AliasHandle; var target: FSRef; var wasChanged: Boolean ): OSErr; external name '_FSResolveAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
function FollowFinderAlias( {const} fromFile: FSSpecPtr { can be NULL }; alias: AliasHandle; logon: Boolean; var target: FSSpec; var wasChanged: Boolean ): OSErr; external name '_FollowFinderAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ 
   Low Level Routines 
}
>>>>>>> graemeg/cpstrnew
{
 *  UpdateAlias()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    UseFSUpdateAlias
 *  
 *  Summary:
 *    given a fromFile-target pair and an alias handle, update the
 *    alias record pointed to by alias handle to represent target as
 *    the new alias.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSResolveAliasFileWithMountFlags( var theRef: FSRef; resolveAliasChains: Boolean; var targetIsFolder: Boolean; var wasAliased: Boolean; mountFlags: UNSIGNEDLONG ): OSErr; external name '_FSResolveAliasFileWithMountFlags';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function UpdateAlias( {const} fromFile: FSSpecPtr { can be NULL }; const (*var*) target: FSSpec; alias: AliasHandle; var wasChanged: Boolean ): OSErr; external name '_UpdateAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  MatchAlias()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSMatchAliasBulk instead
 *  
 *  Summary:
 *    Given an alias handle and fromFile, match the alias and return
 *    FSSpecs to the aliased file(s) and needsUpdate flag
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSResolveAliasFile( var theRef: FSRef; resolveAliasChains: Boolean; var targetIsFolder: Boolean; var wasAliased: Boolean ): OSErr; external name '_FSResolveAliasFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function MatchAlias( {const} fromFile: FSSpecPtr { can be NULL }; rulesMask: UNSIGNEDLONG; alias: AliasHandle; var aliasCount: SInt16; aliasList: FSSpecArrayPtr; var needsUpdate: Boolean; aliasFilter: AliasFilterUPP; yourDataPtr: UnivPtr ): OSErr; external name '_MatchAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  ResolveAliasFileWithMountFlagsNoUI()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSResolveAliasFileWithMountFlags passing in the
 *    kResolveAliasFileNoUI flag
 *  
 *  Summary:
 *    variation on ResolveAliasFile that does not prompt user with a
 *    dialog
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSFollowFinderAlias( fromFile: FSRefPtr { can be NULL }; alias: AliasHandle; logon: Boolean; var target: FSRef; var wasChanged: Boolean ): OSErr; external name '_FSFollowFinderAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function ResolveAliasFileWithMountFlagsNoUI( var theSpec: FSSpec; resolveAliasChains: Boolean; var targetIsFolder: Boolean; var wasAliased: Boolean; mountFlags: UNSIGNEDLONG ): OSErr; external name '_ResolveAliasFileWithMountFlagsNoUI';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  MatchAliasNoUI()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSMatchAliasBulk with the kARMNoUI flag instead
 *  
 *  Summary:
 *    variation on MatchAlias that does not prompt user with a dialog
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in InterfaceLib 9.1 and later
 }
function FSUpdateAlias( {const} fromFile: FSRefPtr { can be NULL }; const (*var*) target: FSRef; alias: AliasHandle; var wasChanged: Boolean ): OSErr; external name '_FSUpdateAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
>>>>>>> graemeg/cpstrnew
 *  FSNewAliasUnicode()
 *  
 *  Summary:
 *    Creates an alias given a ref to the target's parent directory and
 *    the target's unicode name.  If the target does not exist fnfErr
 *    will be returned but the alias will still be created.  This
 *    allows the creation of aliases to targets that do not exist.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    fromFile:
 *      The starting point for a relative search.
 *    
 *    targetParentRef:
 *      An FSRef to the parent directory of the target.
 *    
 *    targetNameLength:
 *      Number of Unicode characters in the target's name.
 *    
 *    targetName:
 *      A pointer to the Unicode name.
 *    
 *    inAlias:
 *      A Handle to the newly created alias record.
 *    
 *    isDirectory:
 *      On input, if target does not exist, a flag to indicate whether
 *      or not the target is a directory.  On output, if the target did
 *      exist, a flag indicating if the target is a directory.  Pass
 *      NULL in the non-existant case if unsure.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function FSNewAliasUnicode( {const} fromFile: FSRefPtr { can be NULL }; const (*var*) targetParentRef: FSRef; targetNameLength: UniCharCount; targetName: ConstUniCharPtr; var inAlias: AliasHandle; isDirectory: BooleanPtr { can be NULL } ): OSErr; external name '_FSNewAliasUnicode';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew


{
 *  FSNewAliasMinimalUnicode()
 *  
 *  Summary:
 *    Creates a minimal alias given a ref to the target's parent
 *    directory and the target's unicode name.  If the target does not
 *    exist fnfErr will be returned but the alias will still be created.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Parameters:
 *    
 *    targetParentRef:
 *      An FSRef to the parent directory of the target.
 *    
 *    targetNameLength:
 *      Number of Unicode characters in the target's name.
 *    
 *    targetName:
 *      A pointer to the Unicode name.
 *    
 *    inAlias:
 *      A Handle to the newly created alias record.
 *    
 *    isDirectory:
 *      On input, if target does not exist, a flag to indicate whether
 *      or not the target is a directory.  On output, if the target did
 *      exist, a flag indicating if the target is a directory.  Pass
 *      NULL in the non-existant case if unsure.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function FSNewAliasMinimalUnicode( const (*var*) targetParentRef: FSRef; targetNameLength: UniCharCount; targetName: ConstUniCharPtr; var inAlias: AliasHandle; isDirectory: BooleanPtr { can be NULL } ): OSErr; external name '_FSNewAliasMinimalUnicode';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew


{
 *  FSNewAliasFromPath()
 *  
 *  Summary:
 *    Creates an alias given a POSIX style utf-8 path to the target. 
 *    If the target file does not exist but the path up to the leaf
 *    does then fnfErr will be returned but the alias will still be
 *    created.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    fromFilePath:
 *      The starting point for a relative search.
 *    
 *    targetPath:
 *      POSIX style UTF-8 path to target.
 *    
 *    flags:
 *      Options for future use.  Pass in 0.
 *    
 *    inAlias:
 *      A Handle to the newly created alias record.
 *    
 *    isDirectory:
 *      On input, if target does not exist, a flag to indicate whether
 *      or not the target is a directory.  On output, if the target did
 *      exist, a flag indicating if the target is a directory.  Pass
 *      NULL in the non-existant case if unsure.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.5 and later
 *    Non-Carbon CFM:   not available
 }
function FSNewAliasFromPath( fromFilePath: ConstCStringPtr { can be NULL }; targetPath: ConstCStringPtr; flags: OptionBits; var inAlias: AliasHandle; isDirectory: BooleanPtr { can be NULL } ): OSStatus; external name '_FSNewAliasFromPath';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_5, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew


{
 *  FSMatchAliasBulk()
 *  
 *  Summary:
 *    Given an alias handle and fromFile, match the alias and return
 *    FSRefs to the aliased file(s) and needsUpdate flag
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function FSMatchAliasBulk( {const} fromFile: FSRefPtr { can be NULL }; rulesMask: UNSIGNEDLONG; inAlias: AliasHandle; var aliasCount: SInt16; var aliasList: FSRef; var needsUpdate: Boolean; aliasFilter: FSAliasFilterProcPtr { can be NULL }; yourDataPtr: UnivPtr { can be NULL } ): OSStatus; external name '_FSMatchAliasBulk';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_5, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew


{
 *  FSCopyAliasInfo()
 *  
 *  Discussion:
 *    This routine will return the requested information from the
 *    passed in aliasHandle.  The information is gathered only from the
 *    alias record so it may not match what is on disk (no disk i/o is
 *    performed).  The whichInfo paramter is an output parameter that
 *    signifies which fields in the info record contain valid data.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    inAlias:
 *      A handle to the alias record to get the information from.
 *    
 *    targetName:
 *      The name of the target item.
 *    
 *    volumeName:
 *      The name of the volume the target resides on.
 *    
 *    pathString:
 *      POSIX path to target.
 *    
 *    whichInfo:
 *      An indication of which fields in the info block contain valid
 *      data.
 *    
 *    info:
 *      Returned information about the alias.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function FSCopyAliasInfo( inAlias: AliasHandle; targetName: HFSUniStr255Ptr { can be NULL }; volumeName: HFSUniStr255Ptr { can be NULL }; pathString: CFStringRefPtr { can be NULL }; whichInfo: FSAliasInfoBitmapPtr { can be NULL }; info: FSAliasInfoPtr { can be NULL } ): OSStatus; external name '_FSCopyAliasInfo';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew


{
 *  GetAliasSize()
 *  
 *  Discussion:
 *    This routine will return the size of the alias record referenced
 *    by the AliasHandle alias.  This will be smaller than the size
 *    returned by GetHandleSize if any custom data has been added (IM
 *    Files 4-13).
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    alias:
 *      A handle to the alias record to get the information from.
 *  
 *  Result:
 *    The size of the private section of the alias record.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function GetAliasSize( alias: AliasHandle ): Size; external name '_GetAliasSize';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_4, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew


{
 *  GetAliasUserType()
 *  
 *  Discussion:
 *    This routine will return the usertype associated with the alias
 *    record referenced by the AliasHandle alias.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    alias:
 *      A handle to the alias record to get the userType from.
 *  
 *  Result:
 *    The userType associated with the alias as an OSType
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function GetAliasUserType( alias: AliasHandle ): OSType; external name '_GetAliasUserType';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_4, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> graemeg/cpstrnew


{
 *  SetAliasUserType()
 *  
 *  Discussion:
 *    This routine will set the userType associated with an alias
 *    record.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    alias:
 *      A handle to the alias record to set the userType for.
 *    
 *    userType:
 *      The OSType to set the userType to.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
procedure SetAliasUserType( alias: AliasHandle; userType: OSType ); external name '_SetAliasUserType';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_4, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)


{
 *  GetAliasSizeFromPtr()
 *  
 *  Discussion:
 *    This routine will return the size of the alias record referenced
 *    by a pointer to the AliasRecord.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    alias:
 *      A pointer to the alias record to get the information from.
 *  
 *  Result:
 *    The size of the private section of the alias record.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function GetAliasSizeFromPtr( const (*var*) alias: AliasRecord ): Size; external name '_GetAliasSizeFromPtr';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_4, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)


{
 *  GetAliasUserTypeFromPtr()
 *  
 *  Discussion:
 *    This routine will return the usertype associated withthe alias
 *    record pointed to by alias.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    alias:
 *      A pointer to the alias record to get the userType from.
 *  
 *  Result:
 *    The userType associated with the alias as an OSType
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function GetAliasUserTypeFromPtr( const (*var*) alias: AliasRecord ): OSType; external name '_GetAliasUserTypeFromPtr';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_4, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)


{
 *  SetAliasUserTypeWithPtr()
 *  
 *  Discussion:
 *    This routine will set the userType associated with an alias
 *    record.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    alias:
 *      A pointer to the alias record to set the userType for.
 *    
 *    userType:
 *      The OSType to set the userType to.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
procedure SetAliasUserTypeWithPtr( alias: AliasPtr; userType: OSType ); external name '_SetAliasUserTypeWithPtr';
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_4, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)


{ Functions beyond this point are deprecated}

{$ifc not TARGET_CPU_64}
{
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  GetAliasSizeFromPtr()
 *  
 *  Discussion:
 *    This routine will return the size of the alias record referenced
 *    by a pointer to the AliasRecord.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    alias:
 *      A pointer to the alias record to get the information from.
 *  
 *  Result:
 *    The size of the private section of the alias record.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function GetAliasSizeFromPtr( const (*var*) alias: AliasRecord ): Size; external name '_GetAliasSizeFromPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  GetAliasUserTypeFromPtr()
 *  
 *  Discussion:
 *    This routine will return the usertype associated withthe alias
 *    record pointed to by alias.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    alias:
 *      A pointer to the alias record to get the userType from.
 *  
 *  Result:
 *    The userType associated with the alias as an OSType
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function GetAliasUserTypeFromPtr( const (*var*) alias: AliasRecord ): OSType; external name '_GetAliasUserTypeFromPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{
 *  SetAliasUserTypeWithPtr()
 *  
 *  Discussion:
 *    This routine will set the userType associated with an alias
 *    record.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Parameters:
 *    
 *    alias:
 *      A pointer to the alias record to set the userType for.
 *    
 *    userType:
 *      The OSType to set the userType to.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
procedure SetAliasUserTypeWithPtr( alias: AliasPtr; userType: OSType ); external name '_SetAliasUserTypeWithPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)


{ Functions beyond this point are deprecated}

{$ifc not TARGET_CPU_64}
{
>>>>>>> graemeg/cpstrnew
 *  FSMatchAlias()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSMatchAliasBulk instead
 *  
 *  Summary:
 *    Given an alias handle and fromFile, match the alias and return
 *    FSRefs to the aliased file(s) and needsUpdate flag
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function FSMatchAlias( {const} fromFile: FSRefPtr { can be NULL }; rulesMask: UNSIGNEDLONG; inAlias: AliasHandle; var aliasCount: SInt16; var aliasList: FSRef; var needsUpdate: Boolean; aliasFilter: AliasFilterUPP; yourDataPtr: UnivPtr ): OSErr; external name '_FSMatchAlias';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew


{
 *  FSMatchAliasNoUI()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSMatchAliasBulk with the kARMNoUI flag instead
 *  
 *  Summary:
 *    variation on FSMatchAlias that does not prompt user with a dialog
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function FSMatchAliasNoUI( {const} fromFile: FSRefPtr { can be NULL }; rulesMask: UNSIGNEDLONG; inAlias: AliasHandle; var aliasCount: SInt16; var aliasList: FSRef; var needsUpdate: Boolean; aliasFilter: AliasFilterUPP; yourDataPtr: UnivPtr ): OSErr; external name '_FSMatchAliasNoUI';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_2, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew


{
 *  NewAlias()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSNewAlias
 *  
 *  Summary:
 *    create a new alias between fromFile and target, returns alias
 *    record handle
 *  
 *  Discussion:
 *    Create an alias betwen fromFile and target, and return it in an
 *    AliasHandle. This function is deprecated in Mac OS X 10.4;
 *    instead, you should use FSNewAliasUnicode() because NewAlias()
 *    has problems creating aliases to certain files, including those
 *    which are impossible to represent in an FSSpec.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function NewAlias( {const} fromFile: FSSpecPtr { can be NULL }; const (*var*) target: FSSpec; var alias: AliasHandle ): OSErr; external name '_NewAlias';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  NewAliasMinimal()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSNewAliasMinimalUnicode
 *  
 *  Summary:
 *    create a minimal new alias for a target and return alias record
 *    handle
 *  
 *  Discussion:
 *    Create a minimal alias for a target, and return it in an
 *    AliasHandle. This function is deprecated in Mac OS X 10.4;
 *    instead, you should use FSNewAliasMinimalUnicode() because
 *    NewAliasMinimalAlias() has problems creating aliases to certain
 *    files, including those which are impossible to represent in an
 *    FSSpec.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function NewAliasMinimal( const (*var*) target: FSSpec; var alias: AliasHandle ): OSErr; external name '_NewAliasMinimal';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  NewAliasMinimalFromFullPath()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSNewAliasMinimalUnicode
 *  
 *  Summary:
 *    create a minimal new alias from a target fullpath (optional zone
 *    and server name) and return alias record handle
 *  
 *  Discussion:
 *    Create a minimal alias for a target fullpath, and return it in an
 *    AliasHandle. This function is deprecated in Mac OS X 10.4;
 *    instead, you should use FSNewAliasMinimalUnicode() because
 *    NewAliasMinimalFromFullPath() has problems creating aliases to
 *    certain files, including those which are impossible to represent
 *    in an FSSpec.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function NewAliasMinimalFromFullPath( fullPathLength: SInt16; fullPath: {const} UnivPtr; const (*var*) zoneName: Str32; const (*var*) serverName: Str31; var alias: AliasHandle ): OSErr; external name '_NewAliasMinimalFromFullPath';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  ResolveAlias()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSResolveAlias() or FSResolveAliasWithMountFlags() instead.
 *  
 *  Summary:
 *    given an alias handle and fromFile, resolve the alias, update the
 *    alias record and return aliased filename and wasChanged flag.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function ResolveAlias( {const} fromFile: FSSpecPtr { can be NULL }; alias: AliasHandle; var target: FSSpec; var wasChanged: Boolean ): OSErr; external name '_ResolveAlias';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  GetAliasInfo()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSCopyAliasInfo instead.
 *  
 *  Summary:
 *    This call does not work on all aliases. Given an alias handle and
 *    an index specifying requested alias information type, return the
 *    information from alias record as a string. An empty string is
 *    returned when the index is greater than the number of levels
 *    between the target and root.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetAliasInfo( alias: AliasHandle; itemIndex: AliasInfoType; var theString: Str63 ): OSErr; external name '_GetAliasInfo';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_3, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)
>>>>>>> graemeg/cpstrnew


{
 *  IsAliasFile()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSIsAliasFile() instead.
 *  
 *  Summary:
 *    Return true if the file pointed to by fileFSSpec is an alias file.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
function IsAliasFile( const (*var*) fileFSSpec: FSSpec; var aliasFileFlag: Boolean; var folderFlag: Boolean ): OSErr; external name '_IsAliasFile';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  ResolveAliasWithMountFlags()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSResolveAliasWithMountFlags() instead.
 *  
 *  Summary:
 *    Given an AliasHandle, return target file spec. It resolves the
 *    entire alias chain or one step of the chain.  It returns info
 *    about whether the target is a folder or file; and whether the
 *    input file spec was an alias or not.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
function ResolveAliasWithMountFlags( {const} fromFile: FSSpecPtr { can be NULL }; alias: AliasHandle; var target: FSSpec; var wasChanged: Boolean; mountFlags: UNSIGNEDLONG ): OSErr; external name '_ResolveAliasWithMountFlags';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  ResolveAliasFile()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSResolveAliasFile
 *  
 *  Summary:
 *    Given a file spec, return target file spec if input file spec is
 *    an alias. It resolves the entire alias chain or one step of the
 *    chain.  It returns info about whether the target is a folder or
 *    file; and whether the input file spec was an alias or not.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function ResolveAliasFile( var theSpec: FSSpec; resolveAliasChains: Boolean; var targetIsFolder: Boolean; var wasAliased: Boolean ): OSErr; external name '_ResolveAliasFile';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{ Deprecated: Use FSResolveAliasFileWithMountFlags instead}
{
 *  ResolveAliasFileWithMountFlags()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
function ResolveAliasFileWithMountFlags( var theSpec: FSSpec; resolveAliasChains: Boolean; var targetIsFolder: Boolean; var wasAliased: Boolean; mountFlags: UNSIGNEDLONG ): OSErr; external name '_ResolveAliasFileWithMountFlags';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew


{ Deprecated:  Use FSFollowFinderAlias instead}
{
 *  FollowFinderAlias()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
function FollowFinderAlias( {const} fromFile: FSSpecPtr { can be NULL }; alias: AliasHandle; logon: Boolean; var target: FSSpec; var wasChanged: Boolean ): OSErr; external name '_FollowFinderAlias';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew


{ 
   Low Level Routines 
}
{
 *  UpdateAlias()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    UseFSUpdateAlias
 *  
 *  Summary:
 *    given a fromFile-target pair and an alias handle, update the
 *    alias record pointed to by alias handle to represent target as
 *    the new alias.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function UpdateAlias( {const} fromFile: FSSpecPtr { can be NULL }; const (*var*) target: FSSpec; alias: AliasHandle; var wasChanged: Boolean ): OSErr; external name '_UpdateAlias';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  MatchAlias()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSMatchAliasBulk instead
 *  
 *  Summary:
 *    Given an alias handle and fromFile, match the alias and return
 *    FSSpecs to the aliased file(s) and needsUpdate flag
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function MatchAlias( {const} fromFile: FSSpecPtr { can be NULL }; rulesMask: UNSIGNEDLONG; alias: AliasHandle; var aliasCount: SInt16; aliasList: FSSpecArrayPtr; var needsUpdate: Boolean; aliasFilter: AliasFilterUPP; yourDataPtr: UnivPtr ): OSErr; external name '_MatchAlias';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  ResolveAliasFileWithMountFlagsNoUI()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSResolveAliasFileWithMountFlags passing in the
 *    kResolveAliasFileNoUI flag
 *  
 *  Summary:
 *    variation on ResolveAliasFile that does not prompt user with a
 *    dialog
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function ResolveAliasFileWithMountFlagsNoUI( var theSpec: FSSpec; resolveAliasChains: Boolean; var targetIsFolder: Boolean; var wasAliased: Boolean; mountFlags: UNSIGNEDLONG ): OSErr; external name '_ResolveAliasFileWithMountFlagsNoUI';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_4, __IPHONE_NA, __IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
>>>>>>> graemeg/cpstrnew


{
 *  MatchAliasNoUI()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSMatchAliasBulk with the kARMNoUI flag instead
 *  
 *  Summary:
 *    variation on MatchAlias that does not prompt user with a dialog
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function MatchAliasNoUI( {const} fromFile: FSSpecPtr { can be NULL }; rulesMask: UNSIGNEDLONG; alias: AliasHandle; var aliasCount: SInt16; aliasList: FSSpecArrayPtr; var needsUpdate: Boolean; aliasFilter: AliasFilterUPP; yourDataPtr: UnivPtr ): OSErr; external name '_MatchAliasNoUI';
<<<<<<< HEAD
(* __OSX_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5, __IPHONE_NA, __IPHONE_NA) *)

=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function MatchAliasNoUI( {const} fromFile: FSSpecPtr { can be NULL }; rulesMask: UNSIGNEDLONG; alias: AliasHandle; var aliasCount: SInt16; aliasList: FSSpecArrayPtr; var needsUpdate: Boolean; aliasFilter: AliasFilterUPP; yourDataPtr: UnivPtr ): OSErr; external name '_MatchAliasNoUI';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function UpdateAlias( {const} fromFile: FSSpecPtr { can be NULL }; const (*var*) target: FSSpec; alias: AliasHandle; var wasChanged: Boolean ): OSErr; external name '_UpdateAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  MatchAlias()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSMatchAliasBulk instead
 *  
 *  Summary:
 *    Given an alias handle and fromFile, match the alias and return
 *    FSSpecs to the aliased file(s) and needsUpdate flag
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function MatchAlias( {const} fromFile: FSSpecPtr { can be NULL }; rulesMask: UNSIGNEDLONG; alias: AliasHandle; var aliasCount: SInt16; aliasList: FSSpecArrayPtr; var needsUpdate: Boolean; aliasFilter: AliasFilterUPP; yourDataPtr: UnivPtr ): OSErr; external name '_MatchAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  ResolveAliasFileWithMountFlagsNoUI()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSResolveAliasFileWithMountFlags passing in the
 *    kResolveAliasFileNoUI flag
 *  
 *  Summary:
 *    variation on ResolveAliasFile that does not prompt user with a
 *    dialog
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function ResolveAliasFileWithMountFlagsNoUI( var theSpec: FSSpec; resolveAliasChains: Boolean; var targetIsFolder: Boolean; var wasAliased: Boolean; mountFlags: UNSIGNEDLONG ): OSErr; external name '_ResolveAliasFileWithMountFlagsNoUI';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  MatchAliasNoUI()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSMatchAliasBulk with the kARMNoUI flag instead
 *  
 *  Summary:
 *    variation on MatchAlias that does not prompt user with a dialog
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function MatchAliasNoUI( {const} fromFile: FSSpecPtr { can be NULL }; rulesMask: UNSIGNEDLONG; alias: AliasHandle; var aliasCount: SInt16; aliasList: FSSpecArrayPtr; var needsUpdate: Boolean; aliasFilter: AliasFilterUPP; yourDataPtr: UnivPtr ): OSErr; external name '_MatchAliasNoUI';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function MatchAliasNoUI( {const} fromFile: FSSpecPtr { can be NULL }; rulesMask: UNSIGNEDLONG; alias: AliasHandle; var aliasCount: SInt16; aliasList: FSSpecArrayPtr; var needsUpdate: Boolean; aliasFilter: AliasFilterUPP; yourDataPtr: UnivPtr ): OSErr; external name '_MatchAliasNoUI';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/cpstrnew


{$endc} {not TARGET_CPU_64}

{$endc} {TARGET_OS_MAC}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function NewAlias( {const} fromFile: FSSpecPtr { can be NULL }; const (*var*) target: FSSpec; var alias: AliasHandle ): OSErr; external name '_NewAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  NewAliasMinimal()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSNewAliasMinimalUnicode
 *  
 *  Summary:
 *    create a minimal new alias for a target and return alias record
 *    handle
 *  
 *  Discussion:
 *    Create a minimal alias for a target, and return it in an
 *    AliasHandle. This function is deprecated in Mac OS X 10.4;
 *    instead, you should use FSNewAliasMinimalUnicode() because
 *    NewAliasMinimalAlias() has problems creating aliases to certain
 *    files, including those which are impossible to represent in an
 *    FSSpec.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function NewAliasMinimal( const (*var*) target: FSSpec; var alias: AliasHandle ): OSErr; external name '_NewAliasMinimal';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  NewAliasMinimalFromFullPath()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSNewAliasMinimalUnicode
 *  
 *  Summary:
 *    create a minimal new alias from a target fullpath (optional zone
 *    and server name) and return alias record handle
 *  
 *  Discussion:
 *    Create a minimal alias for a target fullpath, and return it in an
 *    AliasHandle. This function is deprecated in Mac OS X 10.4;
 *    instead, you should use FSNewAliasMinimalUnicode() because
 *    NewAliasMinimalFromFullPath() has problems creating aliases to
 *    certain files, including those which are impossible to represent
 *    in an FSSpec.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function NewAliasMinimalFromFullPath( fullPathLength: SInt16; fullPath: {const} UnivPtr; const (*var*) zoneName: Str32; const (*var*) serverName: Str31; var alias: AliasHandle ): OSErr; external name '_NewAliasMinimalFromFullPath';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  ResolveAlias()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSResolveAlias() or FSResolveAliasWithMountFlags() instead.
 *  
 *  Summary:
 *    given an alias handle and fromFile, resolve the alias, update the
 *    alias record and return aliased filename and wasChanged flag.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function ResolveAlias( {const} fromFile: FSSpecPtr { can be NULL }; alias: AliasHandle; var target: FSSpec; var wasChanged: Boolean ): OSErr; external name '_ResolveAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  GetAliasInfo()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSCopyAliasInfo instead.
 *  
 *  Summary:
 *    This call does not work on all aliases. Given an alias handle and
 *    an index specifying requested alias information type, return the
 *    information from alias record as a string. An empty string is
 *    returned when the index is greater than the number of levels
 *    between the target and root.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function GetAliasInfo( alias: AliasHandle; itemIndex: AliasInfoType; var theString: Str63 ): OSErr; external name '_GetAliasInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


{
 *  IsAliasFile()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSIsAliasFile() instead.
 *  
 *  Summary:
 *    Return true if the file pointed to by fileFSSpec is an alias file.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.0
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
function IsAliasFile( const (*var*) fileFSSpec: FSSpec; var aliasFileFlag: Boolean; var folderFlag: Boolean ): OSErr; external name '_IsAliasFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  ResolveAliasWithMountFlags()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSResolveAliasWithMountFlags() instead.
 *  
 *  Summary:
 *    Given an AliasHandle, return target file spec. It resolves the
 *    entire alias chain or one step of the chain.  It returns info
 *    about whether the target is a folder or file; and whether the
 *    input file spec was an alias or not.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
function ResolveAliasWithMountFlags( {const} fromFile: FSSpecPtr { can be NULL }; alias: AliasHandle; var target: FSSpec; var wasChanged: Boolean; mountFlags: UNSIGNEDLONG ): OSErr; external name '_ResolveAliasWithMountFlags';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  ResolveAliasFile()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSResolveAliasFile
 *  
 *  Summary:
 *    Given a file spec, return target file spec if input file spec is
 *    an alias. It resolves the entire alias chain or one step of the
 *    chain.  It returns info about whether the target is a folder or
 *    file; and whether the input file spec was an alias or not.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function ResolveAliasFile( var theSpec: FSSpec; resolveAliasChains: Boolean; var targetIsFolder: Boolean; var wasAliased: Boolean ): OSErr; external name '_ResolveAliasFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{ Deprecated: Use FSResolveAliasFileWithMountFlags instead}
{
 *  ResolveAliasFileWithMountFlags()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
function ResolveAliasFileWithMountFlags( var theSpec: FSSpec; resolveAliasChains: Boolean; var targetIsFolder: Boolean; var wasAliased: Boolean; mountFlags: UNSIGNEDLONG ): OSErr; external name '_ResolveAliasFileWithMountFlags';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ Deprecated:  Use FSFollowFinderAlias instead}
{
 *  FollowFinderAlias()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 8.5 and later
 }
function FollowFinderAlias( {const} fromFile: FSSpecPtr { can be NULL }; alias: AliasHandle; logon: Boolean; var target: FSSpec; var wasChanged: Boolean ): OSErr; external name '_FollowFinderAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ 
   Low Level Routines 
}
{
 *  UpdateAlias()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    UseFSUpdateAlias
 *  
 *  Summary:
 *    given a fromFile-target pair and an alias handle, update the
 *    alias record pointed to by alias handle to represent target as
 *    the new alias.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function UpdateAlias( {const} fromFile: FSSpecPtr { can be NULL }; const (*var*) target: FSSpec; alias: AliasHandle; var wasChanged: Boolean ): OSErr; external name '_UpdateAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  MatchAlias()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSMatchAliasBulk instead
 *  
 *  Summary:
 *    Given an alias handle and fromFile, match the alias and return
 *    FSSpecs to the aliased file(s) and needsUpdate flag
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function MatchAlias( {const} fromFile: FSSpecPtr { can be NULL }; rulesMask: UNSIGNEDLONG; alias: AliasHandle; var aliasCount: SInt16; aliasList: FSSpecArrayPtr; var needsUpdate: Boolean; aliasFilter: AliasFilterUPP; yourDataPtr: UnivPtr ): OSErr; external name '_MatchAlias';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  ResolveAliasFileWithMountFlagsNoUI()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSResolveAliasFileWithMountFlags passing in the
 *    kResolveAliasFileNoUI flag
 *  
 *  Summary:
 *    variation on ResolveAliasFile that does not prompt user with a
 *    dialog
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function ResolveAliasFileWithMountFlagsNoUI( var theSpec: FSSpec; resolveAliasChains: Boolean; var targetIsFolder: Boolean; var wasAliased: Boolean; mountFlags: UNSIGNEDLONG ): OSErr; external name '_ResolveAliasFileWithMountFlagsNoUI';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  MatchAliasNoUI()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use FSMatchAliasBulk with the kARMNoUI flag instead
 *  
 *  Summary:
 *    variation on MatchAlias that does not prompt user with a dialog
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.3
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function MatchAliasNoUI( {const} fromFile: FSSpecPtr { can be NULL }; rulesMask: UNSIGNEDLONG; alias: AliasHandle; var aliasCount: SInt16; aliasList: FSSpecArrayPtr; var needsUpdate: Boolean; aliasFilter: AliasFilterUPP; yourDataPtr: UnivPtr ): OSErr; external name '_MatchAliasNoUI';
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
