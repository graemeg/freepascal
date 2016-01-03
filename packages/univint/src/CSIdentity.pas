{
     File:       OSServices/CSIdentity.h
 
     Contains:   CSIdentity APIs
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Copyright:  (c) 2006-2011 Apple Inc. All rights reserved.
=======
     Version:    OSServices-352~2
 
     Copyright:  © 2006-2008 by Apple Computer, Inc., all rights reserved.
>>>>>>> graemeg/cpstrnew
=======
     Version:    OSServices-352~2
 
     Copyright:  © 2006-2008 by Apple Computer, Inc., all rights reserved.
>>>>>>> graemeg/cpstrnew
=======
     Version:    OSServices-352~2
 
     Copyright:  © 2006-2008 by Apple Computer, Inc., all rights reserved.
>>>>>>> graemeg/cpstrnew
=======
     Version:    OSServices-352~2
 
     Copyright:  © 2006-2008 by Apple Computer, Inc., all rights reserved.
>>>>>>> origin/cpstrnew
=======
     Version:    OSServices-352~2
 
     Copyright:  © 2006-2008 by Apple Computer, Inc., all rights reserved.
>>>>>>> origin/fixes_2.4
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
                     http://www.freepascal.org/bugs.html
 
}
{      Pascal Translation:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
<<<<<<< HEAD
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
=======
>>>>>>> origin/fixes_2.4
{
    Modified for use with Free Pascal
    Version 308
    Please report any bugs to <gpc@microbizz.nl>
}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
{$mode macpas}
{$packenum 1}
{$macro on}
{$inline on}
{$calling mwpascal}

unit CSIdentity;
interface
{$setc UNIVERSAL_INTERFACES_VERSION := $0400}
{$setc GAP_INTERFACES_VERSION := $0308}

{$ifc not defined USE_CFSTR_CONSTANT_MACROS}
    {$setc USE_CFSTR_CONSTANT_MACROS := TRUE}
{$endc}

{$ifc defined CPUPOWERPC and defined CPUI386}
	{$error Conflicting initial definitions for CPUPOWERPC and CPUI386}
{$endc}
{$ifc defined FPC_BIG_ENDIAN and defined FPC_LITTLE_ENDIAN}
	{$error Conflicting initial definitions for FPC_BIG_ENDIAN and FPC_LITTLE_ENDIAN}
{$endc}

{$ifc not defined __ppc__ and defined CPUPOWERPC32}
	{$setc __ppc__ := 1}
{$elsec}
	{$setc __ppc__ := 0}
{$endc}
{$ifc not defined __ppc64__ and defined CPUPOWERPC64}
	{$setc __ppc64__ := 1}
{$elsec}
	{$setc __ppc64__ := 0}
{$endc}
{$ifc not defined __i386__ and defined CPUI386}
	{$setc __i386__ := 1}
{$elsec}
	{$setc __i386__ := 0}
{$endc}
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


{$ifc defined __ppc__ and __ppc__ and defined __i386__ and __i386__}
	{$error Conflicting definitions for __ppc__ and __i386__}
{$endc}

{$ifc defined __ppc__ and __ppc__}
	{$setc TARGET_CPU_PPC := TRUE}
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
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/fixes_2.4
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
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/fixes_2.4
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
=======
>>>>>>> origin/fixes_2.4
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
=======
>>>>>>> origin/fixes_2.4
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
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/fixes_2.4
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
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
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
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> origin/fixes_2.4
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
  {$setc TARGET_CPU_64 := FALSE}
{$endc}

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
<<<<<<< HEAD
<<<<<<< HEAD
uses MacTypes,MacOSXPosix,CSIdentityAuthority,SecBase,CSIdentityBase,Authorization,CFBase,CFArray,CFData,CFError,CFRunLoop,CFUUID;
{$endc} {not MACOSALLINCLUDE}


=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
uses MacTypes,MacOSXPosix,CSIdentityAuthority,SecBase,Authorization,CFBase,CFArray,CFData,CFError,CFRunLoop,CFUUID;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}
<<<<<<< HEAD
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
=======
>>>>>>> origin/fixes_2.4

{$ALIGN MAC68K}


{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
    The error domain of all CFErrors reported by Identity Services 
}
{
 *  kCSIdentityErrorDomain
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kCSIdentityErrorDomain: CFStringRef; external name '_kCSIdentityErrorDomain'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
 *  CSIdentity error codes
 *  
 *  Discussion:
 *    Error codes in the CSIdentity error domain
 }
const
{
   * The specified authority is not recognized
   }
	kCSIdentityUnknownAuthorityErr = -1;

  {
   * The specified authority is currently not accessible
   }
	kCSIdentityAuthorityNotAccessibleErr = -2;

  {
   * The caller does not have permission to perform the operation
   }
	kCSIdentityPermissionErr = -3;

  {
   * The requested identity has been deteled
   }
	kCSIdentityDeletedErr = -4;

  {
   * The full name is not valid (length: [1-255])
   }
	kCSIdentityInvalidFullNameErr = -5;

  {
   * The full name is aleady assigned to another identity
   }
	kCSIdentityDuplicateFullNameErr = -6;

  {
   * The Posix name is not valid (char set: [a-zA-Z0-9_-] length:
   * [1-255])
   }
	kCSIdentityInvalidPosixNameErr = -7;

  {
   * The Posix name is aleady assigned to another identity
   }
	kCSIdentityDuplicatePosixNameErr = -8;


{
<<<<<<< HEAD
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
=======
>>>>>>> origin/fixes_2.4
    CSIdentity

    A CSIdentity object represents a user or group entity known to the system. An
    identity object has the following required attributes: a class (user
    or group), a unique identitfier (UUID), a full name, a Posix ID
    (UID or GID), and a Posix name (a.k.a. "short" name). There are also a number
    of optional attributes such as email address, image data, etc.
    
    Group identities have a membership which may include both users as well as
    other groups. An identity can be tested for membership in a specific group.

    A CSIdentity object is a private copy of the identity information. It can be
    modified in memory, but requires authorization to commit changes back to
    the identity authority database. On Mac OS X version 10.5, only local identities
    can be created, modified or deleted, and only by users with Administrator 
    credentials.

    Changes may be committed synchronously or asynchronously. All data validation
    occurs at commit time.

    Two identities are CFEqual if they have the same class and UUID.
}


{
 *  CSIdentityRef
 *  
 *  Discussion:
 *    A reference to an identity object. Can be either a user or group.
 }
type
	CSIdentityRef = ^__CSIdentity; { an opaque type }
	__CSIdentity = record end;

{
 *  CSIdentityQueryRef
 *  
 *  Discussion:
 *    A reference to an identity query object, used to lookup
 *    identities in an identity authority's database.
 }
type
	CSIdentityQueryRef = ^__CSIdentityQuery; { an opaque type }
	__CSIdentityQuery = record end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{$ifc not TARGET_OS_IPHONE and not TARGET_IPHONE_SIMULATOR}
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
{
    kCSIdentityGeneratePosixName
    Passing this constant as the Posix name when creating an indentity
    will generate a unique Posix name, based on the full name. The Posix 
    name is generated at commit time.
}
{
 *  kCSIdentityGeneratePosixName
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
var kCSIdentityGeneratePosixName: CFStringRef; external name '_kCSIdentityGeneratePosixName'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)
{$endc} {!TARGET_OS_IPHONE && !TARGET_IPHONE_SIMULATOR}
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  CSIdentityClass
 *  
 *  Discussion:
 *    Enum specifying an identity class
 }
const
{
   * The class value for user identities
   }
	kCSIdentityClassUser = 1;

  {
   * The class value for group identities
   }
	kCSIdentityClassGroup = 2;

type
	CSIdentityClass = CFIndex;

{
 *  CSIdentityFlags
 *  
 *  Discussion:
 *    Flags used when creating new identities
 }
const
{
   * Use this flag to set no optional attributes for a new identity
   }
	kCSIdentityFlagNone = 0;

  {
   * This flag causes the identity to be "hidden," that is, excluded
   * from most user-visible identity lists. Hidden identities include
   * administrative users and groups such as root, www, and mysql.
   * System service access control groups should be created with the
   * hidden flag.
   }
	kCSIdentityFlagHidden = 1;

type
	CSIdentityFlags = CFOptionFlags;
{
 *  CSIdentityGetTypeID()
 *  
 *  Summary:
 *    Returns the CSIdentity type identifier
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Result:
 *    The CFTypeID of the CSIdentity Core Foundation type
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function CSIdentityGetTypeID: CFTypeID; external name '_CSIdentityGetTypeID';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_5_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *
 *  Creating Identities
 *
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc not TARGET_OS_IPHONE and not TARGET_IPHONE_SIMULATOR}
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
{
 *  CSIdentityCreate()
 *  
 *  Summary:
 *    Creates a new identity
 *  
 *  Discussion:
 *    The new identity is allocated but is not committed to the
 *    identity authority's database. It will become persistent and
 *    available to other clients after being committed using
 *    CSIdentityCommit or CSIdentityCommitAsynchronously.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    allocator:
 *      The allocator to use when creating the object. NULL is
 *      equivalent to specifying kCFAllocatorDefault.
 *    
 *    identityClass:
 *      The type of identity to be created. Specifying
 *      kCSIdentityClassUser creates a user, while
 *      kCSIdentityClassGroup creates a group.
 *    
 *    fullName:
 *      The primary name of the new identity.
 *    
 *    posixName:
 *      The POSIX name of the new identity. Specify
 *      kCSIdentityGeneratePosixName to have a name generated
 *      autmatically from the full name.
 *    
 *    flags:
 *      A CSIdentityFlags mask defining attributes of the new identity
 *    
 *    authority:
 *      The identity authority to host the identity. Caller must have
 *      write access to the identity authority or commit will fail.
 *      Currently, only local identities may be created, so callers
 *      must specify the local identity authority for this argument.
 *  
 *  Result:
 *    The CSIdentityRef of the newly created identity object. Returns
 *    NULL only if allocation fails.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function CSIdentityCreate( allocator: CFAllocatorRef; identityClass: CSIdentityClass; fullName: CFStringRef; posixName: CFStringRef; flags: CSIdentityFlags; authority: CSIdentityAuthorityRef ): CSIdentityRef; external name '_CSIdentityCreate';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)
{$endc} {!TARGET_OS_IPHONE && !TARGET_IPHONE_SIMULATOR}

{$ifc not TARGET_OS_IPHONE and not TARGET_IPHONE_SIMULATOR}
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  CSIdentityCreateCopy()
 *  
 *  Summary:
 *    Creates a copy of an identity
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    allocator:
 *      The allocator to use for the new identity. NULL is equivalent
 *      to specifying kCFAllocatorDefault.
 *    
 *    identity:
 *      The identity to copy
 *  
 *  Result:
 *    The CSIdentityRef of the newly created identity object
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function CSIdentityCreateCopy( allocator: CFAllocatorRef; identity: CSIdentityRef ): CSIdentityRef; external name '_CSIdentityCreateCopy';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_5_0) *)
{$endc} {!TARGET_OS_IPHONE && !TARGET_IPHONE_SIMULATOR}
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *
 *  Getting Identity Attributes
 *
 }
{
 *  CSIdentityGetClass()
 *  
 *  Summary:
 *    Returns an identity's class
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    identity:
 *      The identity object to access
 *  
 *  Result:
 *    The CSIdentityClass of an identity
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function CSIdentityGetClass( identity: CSIdentityRef ): CSIdentityClass; external name '_CSIdentityGetClass';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_5_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  CSIdentityGetAuthority()
 *  
 *  Summary:
 *    Returns the identity authority of an identity
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    identity:
 *      The identity object to access
 *  
 *  Result:
 *    A CSIdentityAuthorityRef object
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function CSIdentityGetAuthority( identity: CSIdentityRef ): CSIdentityAuthorityRef; external name '_CSIdentityGetAuthority';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_5_0) *)


{$ifc not TARGET_OS_IPHONE and not TARGET_IPHONE_SIMULATOR}
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  CSIdentityGetUUID()
 *  
 *  Summary:
 *    Returns an identity's UUID.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    identity:
 *      The identity object to access
 *  
 *  Result:
 *    A CFUUID object containing identity's UUID. Will never return
 *    NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function CSIdentityGetUUID( identity: CSIdentityRef ): CFUUIDRef; external name '_CSIdentityGetUUID';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)
{$endc} {!TARGET_OS_IPHONE && !TARGET_IPHONE_SIMULATOR}
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  CSIdentityGetFullName()
 *  
 *  Summary:
 *    Retrieve the full name of an identity
 *  
 *  Discussion:
 *    The full name is the name that is displayed in the user interface.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    identity:
 *      The identity object to access
 *  
 *  Result:
 *    Returns an identity's full name as a CFStringRef. This attribute
 *    is always non-NULL. The identity object may release its reference
 *    to the return value when the identity is modified.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function CSIdentityGetFullName( identity: CSIdentityRef ): CFStringRef; external name '_CSIdentityGetFullName';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_5_0) *)

{$ifc not TARGET_OS_IPHONE and not TARGET_IPHONE_SIMULATOR}
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  CSIdentityGetPosixID()
 *  
 *  Summary:
 *    Retrieve POSIX ID of an identity.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    identity:
 *      The identity to access
 *  
 *  Result:
 *    Returns an identity's POSIX identifier (a UID or GID).
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function CSIdentityGetPosixID( identity: CSIdentityRef ): id_t; external name '_CSIdentityGetPosixID';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)
{$endc} {!TARGET_OS_IPHONE && !TARGET_IPHONE_SIMULATOR}
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  CSIdentityGetPosixName()
 *  
 *  Summary:
 *    Retrieve the POSIX name (short name) of an identity.
 *  
 *  Discussion:
 *    The POSIX name cannot be changed after an identity has been
 *    created.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    identity:
 *      The identity object to access.
 *  
 *  Result:
 *    Returns an identity's POSIX name. This attribute is always
 *    non-NULL. The identity object may release its reference to the
 *    return value when the identity is modified.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function CSIdentityGetPosixName( identity: CSIdentityRef ): CFStringRef; external name '_CSIdentityGetPosixName';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_5_0) *)


{$ifc not TARGET_OS_IPHONE and not TARGET_IPHONE_SIMULATOR}
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  CSIdentityGetEmailAddress()
 *  
 *  Summary:
 *    Retrieve the email address of a user identity
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    identity:
 *      The identity to access
 *  
 *  Result:
 *    Returns the email address of the identity or NULL if there is no
 *    email address. The identity object may release its reference to
 *    the return value when the identity is modified.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function CSIdentityGetEmailAddress( identity: CSIdentityRef ): CFStringRef; external name '_CSIdentityGetEmailAddress';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  CSIdentityGetImageURL()
 *  
 *  Summary:
 *    Retrieve the URL to an identity's image file
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    identity:
 *      The identity to access
 *  
 *  Result:
 *    Returns a CFURLRef that contains the location of the user's image
 *    file, or NULL if there is no image URL. The identity object may
 *    release its reference to the return value when the identity is
 *    modified.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function CSIdentityGetImageURL( identity: CSIdentityRef ): CFURLRef; external name '_CSIdentityGetImageURL';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  CSIdentityGetImageData()
 *  
 *  Summary:
 *    Retrieve the image associated with a user identity
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    identity:
 *      The identity to access
 *  
 *  Result:
 *    Returns the identity's image data as a CFDataRef or NULL if there
 *    is no image data. The identity object may release its reference
 *    to the return value when the identity is modified.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function CSIdentityGetImageData( identity: CSIdentityRef ): CFDataRef; external name '_CSIdentityGetImageData';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  CSIdentityGetImageDataType()
 *  
 *  Summary:
 *    Retrieve the uniform type identifier (UTI) of an identity's image
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    identity:
 *      The identity to access
 *  
 *  Result:
 *    Returns a UTI as a CFStringRef for this identity's image data or
 *    NULL if there is no image data. The identity object may release
 *    its reference to the return value when the identity is modified.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function CSIdentityGetImageDataType( identity: CSIdentityRef ): CFStringRef; external name '_CSIdentityGetImageDataType';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  CSIdentityGetAliases()
 *  
 *  Summary:
 *    Retrieve the aliases of an identity.
 *  
 *  Discussion:
 *    Aliases are alternate names for identities. As with all identity
 *    names, aliases must be unique within the entire namespace of of
 *    the identity authority.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    identity:
 *      The identity to access
 *  
 *  Result:
 *    Returns an array containing the identity's name aliases as
 *    CFStringRefs. The array may be empty. The identity object may
 *    release its reference to the return value when the identity is
 *    modified.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function CSIdentityGetAliases( identity: CSIdentityRef ): CFArrayRef; external name '_CSIdentityGetAliases';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  CSIdentityIsMemberOfGroup()
 *  
 *  Summary:
 *    Check if an identity is a memeber of a group
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    identity:
 *      The identity whose membership is in question
 *    
 *    group:
 *      The group identity whose membership is to be checked
 *  
 *  Result:
 *    Returns true if the identity is a member (directly or indirectly)
 *    of the specified group
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function CSIdentityIsMemberOfGroup( identity: CSIdentityRef; group: CSIdentityRef ): Boolean; external name '_CSIdentityIsMemberOfGroup';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  CSIdentityIsHidden()
 *  
 *  Summary:
 *    Determine if a identity's hidden attribute is enabled
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    identity:
 *      The identity object to access
 *  
 *  Result:
 *    Returns true if the identity was created with the hidden attribute
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function CSIdentityIsHidden( identity: CSIdentityRef ): Boolean; external name '_CSIdentityIsHidden';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)
{$endc} {!TARGET_OS_IPHONE && !TARGET_IPHONE_SIMULATOR}
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  CSIdentityCreatePersistentReference()
 *  
 *  Summary:
 *    Create an opaque, persistent data reference to an identity
 *  
 *  Discussion:
 *    A persistent identity reference is an opaque data object from
 *    which an identity object may queried the future (see
 *    CSIdentityQueryCreateForPersistentReference). A persistent
 *    reference is suitable for storage in an external data store, for
 *    example, as an entry in an application-specific access control
 *    list associated with a shared resource. Use of a persistent
 *    identity reference is preferred over a pure UUID-based identity
 *    reference because the peristent reference contains additional
 *    information needed to optimize the identity query and to improve
 *    the user experience when working in a distributed identity
 *    environment (LDAP, Active Directory, etc.).
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    allocator:
 *      The allocator for the data
 *    
 *    identity:
 *      The identity to reference
 *  
 *  Result:
 *    Returns a new persistent reference for the identity
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function CSIdentityCreatePersistentReference( allocator: CFAllocatorRef; identity: CSIdentityRef ): CFDataRef; external name '_CSIdentityCreatePersistentReference';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_5_0) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *
 *  Methods which apply only to users
 *
 }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc not TARGET_OS_IPHONE and not TARGET_IPHONE_SIMULATOR}
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
{
 *  CSIdentityIsEnabled()
 *  
 *  Summary:
 *    Determine if a user is enabled
 *  
 *  Discussion:
 *    A user that is not enabled cannot authenticate. This setting may
 *    be used to temporarily allow a user's access to all services and
 *    resources.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    user:
 *      The user identity to access
 *  
 *  Result:
 *    Returns true if the user is enabled. A user that is not enabled
 *    cannot authenticate.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function CSIdentityIsEnabled( user: CSIdentityRef ): Boolean; external name '_CSIdentityIsEnabled';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  CSIdentityAuthenticateUsingPassword()
 *  
 *  Summary:
 *    Attempt to autenticate a password for a user identity
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    user:
 *      The user identity to access
 *    
 *    password:
 *      The password to authenticate
 *  
 *  Result:
 *    Returns true if the passord is correct for the specified user
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function CSIdentityAuthenticateUsingPassword( user: CSIdentityRef; password: CFStringRef ): Boolean; external name '_CSIdentityAuthenticateUsingPassword';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  CSIdentityGetCertificate()
 *  
 *  Summary:
 *    Get a user's authentication certificate
 *  
 *  Discussion:
 *    The authentication certificate can be used in PKI-based protocols
 *    to authenticate users.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    user:
 *      The user identity to access
 *  
 *  Result:
 *    The identity's certificate, or NULL if there is no certificate. 
 *    The identity object may release its reference to the return value
 *    when the identity is modified.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function CSIdentityGetCertificate( user: CSIdentityRef ): SecCertificateRef; external name '_CSIdentityGetCertificate';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *
 *  Methods which apply only to groups
 *
 }

{
 *  CSIdentityCreateGroupMembershipQuery()
 *  
 *  Summary:
 *    Creates a query to find a group's members
 *  
 *  Discussion:
 *    Using a query to lookup group membership allows the caller to
 *    execute the query synchronously or asynchronously.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    allocator:
 *      The allocator to use for the query
 *    
 *    group:
 *      The group identity whose members are to be queried
 *  
 *  Result:
 *    The CSIdentityQueryRef of the newly created object. The query is
 *    ready to be executed.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function CSIdentityCreateGroupMembershipQuery( allocator: CFAllocatorRef; group: CSIdentityRef ): CSIdentityQueryRef; external name '_CSIdentityCreateGroupMembershipQuery';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *
 *  Methods that modify identities
 *
 }

{
 *  CSIdentitySetFullName()
 *  
 *  Summary:
 *    Sets an identity's full name.
 *  
 *  Discussion:
 *    This change must be committed.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    identity:
 *      The identity object to access
 *    
 *    fullName:
 *      The new full name of the identity
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
procedure CSIdentitySetFullName( identity: CSIdentityRef; fullName: CFStringRef ); external name '_CSIdentitySetFullName';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  CSIdentitySetEmailAddress()
 *  
 *  Summary:
 *    Set an identity's email address
 *  
 *  Discussion:
 *    This change must be committed.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    identity:
 *      The user identity to access
 *    
 *    emailAddress:
 *      The user's new email address value. Pass NULL to remove an
 *      email address.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
procedure CSIdentitySetEmailAddress( identity: CSIdentityRef; emailAddress: CFStringRef { can be NULL } ); external name '_CSIdentitySetEmailAddress';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  CSIdentitySetImageURL()
 *  
 *  Summary:
 *    Set the URL of an identity's external image storage
 *  
 *  Discussion:
 *    This change must be committed.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    identity:
 *      The identity to access
 *    
 *    url:
 *      The URL file of the image. For local identities, this must be a
 *      file URL. Pass NULL to remove the image URL from the identity.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
procedure CSIdentitySetImageURL( identity: CSIdentityRef; url: CFURLRef { can be NULL } ); external name '_CSIdentitySetImageURL';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  CSIdentitySetImageData()
 *  
 *  Summary:
 *    Set the internally-stored image data and data type for an identity
 *  
 *  Discussion:
 *    This change must be committed.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    identity:
 *      The identity to access
 *    
 *    imageData:
 *      The image data. Pass NULL to remove image data.
 *    
 *    imageDataType:
 *      The uniform type identitier (UTI) of the image data. Currently,
 *      kUTTypeJPEG ("public.jpeg") is the only type supported.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
procedure CSIdentitySetImageData( identity: CSIdentityRef; imageData: CFDataRef { can be NULL }; imageDataType: CFStringRef { can be NULL } ); external name '_CSIdentitySetImageData';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  CSIdentityAddAlias()
 *  
 *  Summary:
 *    Add a name alias to an identity
 *  
 *  Discussion:
 *    This change must be committed.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    identity:
 *      The identity to access
 *    
 *    alias:
 *      The alias to add
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
procedure CSIdentityAddAlias( identity: CSIdentityRef; alias: CFStringRef ); external name '_CSIdentityAddAlias';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  CSIdentityRemoveAlias()
 *  
 *  Summary:
 *    Remove an alias name from an identity
 *  
 *  Discussion:
 *    This change must be committed.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    identity:
 *      The identity to access
 *    
 *    alias:
 *      The alias name to remove
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
procedure CSIdentityRemoveAlias( identity: CSIdentityRef; alias: CFStringRef ); external name '_CSIdentityRemoveAlias';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *
 *  Methods that modify group membership
 *
 }

{
 *  CSIdentityAddMember()
 *  
 *  Summary:
 *    Add an identity to a group
 *  
 *  Discussion:
 *    This change to the group must be committed.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    group:
 *      The group identity to access
 *    
 *    member:
 *      The identity to add to the group. Can be a user or group
 *      identity.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
procedure CSIdentityAddMember( group: CSIdentityRef; member: CSIdentityRef ); external name '_CSIdentityAddMember';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  CSIdentityRemoveMember()
 *  
 *  Summary:
 *    Remove a member from a group
 *  
 *  Discussion:
 *    This change to the group must be committed.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    group:
 *      The group identity to access
 *    
 *    member:
 *      The member identity to remove
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
procedure CSIdentityRemoveMember( group: CSIdentityRef; member: CSIdentityRef ); external name '_CSIdentityRemoveMember';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{ 


    Methods that modfify user credentials


}

{
 *  CSIdentitySetIsEnabled()
 *  
 *  Summary:
 *    Enable or disable a user
 *  
 *  Discussion:
 *    A disabled user account cannot authenticate. Credentials
 *    (password and certificate) are not affected. This change must be
 *    committed.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    user:
 *      The identity object to access
 *    
 *    isEnabled:
 *      The new value of the isEnabled attribute
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
procedure CSIdentitySetIsEnabled( user: CSIdentityRef; isEnabled: Boolean ); external name '_CSIdentitySetIsEnabled';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  CSIdentitySetPassword()
 *  
 *  Summary:
 *    Set a user password
 *  
 *  Discussion:
 *    Setting the password to NULL removes the current password and
 *    disables password authentication for the user. Setting the
 *    password to a zero-length string allows authentication with a
 *    blank password. This change must be committed.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    user:
 *      The user identity to access
 *    
 *    password:
 *      The new password, or NULL to remove the current password and
 *      disable password-based authentication
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
procedure CSIdentitySetPassword( user: CSIdentityRef; password: CFStringRef { can be NULL } ); external name '_CSIdentitySetPassword';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  CSIdentitySetCertificate()
 *  
 *  Summary:
 *    Set a user's authentication certificate
 *  
 *  Discussion:
 *    The subject name in the certificate will function as an alias for
 *    the identity. As with all identity names, the subject name must
 *    be unique within the entire name space of the identity authority.
 *    This change must be submitted.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    user:
 *      The user identity to access
 *    
 *    certificate:
 *      The user's certificate, or NULL to remove the current
 *      certificate
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
procedure CSIdentitySetCertificate( user: CSIdentityRef; certificate: SecCertificateRef { can be NULL } ); external name '_CSIdentitySetCertificate';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *
 *  Permanent Deletion
 *
 }
{
 *  CSIdentityDelete()
 *  
 *  Summary:
 *    Permanently delete an identity from the identity database
 *  
 *  Discussion:
 *    Sets an identity to deleted state. This change must be committed.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    identity:
 *      The identity to delete
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
procedure CSIdentityDelete( identity: CSIdentityRef ); external name '_CSIdentityDelete';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)
{$endc} {!TARGET_OS_IPHONE && !TARGET_IPHONE_SIMULATOR}
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *
 *
 *  Committing changes
 *
 }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc not TARGET_OS_IPHONE and not TARGET_IPHONE_SIMULATOR}
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
{
 *  CSIdentityCommit()
 *  
 *  Summary:
 *    Synchronously commit all pending changes to the identity
 *    authority database
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    identity:
 *      The identity to commit
 *    
 *    authorization:
 *      The authorization object holding credentials necessary to allow
 *      modification to the identity database. As a convenience,
 *      callers may pass NULL for the authorization, and the
 *      implmentation will attempt to acquire the necessary credentials
 *      from Authorization Services.
 *    
 *    error:
 *      Optional pointer to a CFErrorRef which will be set if this
 *      function returns false. When this occurs, the caller is
 *      responsible for releasing the error.
 *  
 *  Result:
 *    Returns true if successful, false if an error occurred
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function CSIdentityCommit( identity: CSIdentityRef; authorization: AuthorizationRef { can be NULL }; error: CFErrorRefPtr { can be NULL } ): Boolean; external name '_CSIdentityCommit';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)
	
{$endc} {!TARGET_OS_IPHONE && !TARGET_IPHONE_SIMULATOR}
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  Status values
 *  
 }
const
{
   * The identity has been committed to the authority database
   }
	kCSIdentityCommitCompleted = 1;


type
	CSIdentityStatusUpdatedCallback = procedure( identity: CSIdentityRef; status: CFIndex; error: CFErrorRef; info: UnivPtr );

{
 *  CSIdentityClientContext
 *  
 *  Discussion:
 *    Structure containing the user-defined data and callbacks used
 *    during asynchronous commits
 }
type
	CSIdentityClientContext = record
{
   * The version number of the client structure type.  The current
   * version number is 0.
   }
		version: CFIndex;

  {
   * An arbitrary pointer to client-defined data, which can be
   * associated with the client and is passed to the callbacks.
   }
		info: UnivPtr;

  {
   * The callback used to add a retain for the on the client object for
   * the life of the asynchronous operation, and may be used for
   * temporary references the identity needs to take. This callback
   * returns the actual info pointer to be passed to the statusUpdated
   * callback. May be NULL.
   }
		retain: CFAllocatorRetainCallBack;

  {
   * The callback used to remove a retain previously acquired for the
   * client object. May be NULL.
   }
		release: CFAllocatorReleaseCallBack;

  {
   * The callback used to create a descriptive string representation of
   * the client object for debugging purposes. This is used by the
   * CFCopyDescription() function. May be NULL.
   }
		copyDescription: CFAllocatorCopyDescriptionCallBack;

  {
   * The client callback invoked when the status of an asnchronous
   * operation changes
   }
		statusUpdated: CSIdentityStatusUpdatedCallback;
	end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD


{$ifc not TARGET_OS_IPHONE and not TARGET_IPHONE_SIMULATOR}
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
{
 *  CSIdentityCommitAsynchronously()
 *  
 *  Summary:
 *    Asychronously commit all pending changes to the identity
 *    authority's database
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    identity:
 *      The identity to commit
 *    
 *    clientContext:
 *      The client structure specifying context and callbacks for the
 *      asynchronous operation
 *    
 *    runLoop:
 *      The run loop on which to schedule the statusUpdated callback
 *    
 *    runLoopMode:
 *      The run loop mode in which the callback can be scheduled
 *    
 *    authorization:
 *      The authorization object holding credentials necessary to allow
 *      modification to the identity database. As a convenience,
 *      callers may pass NULL for the authorization, and the
 *      implmentation will attempt to acquire the necessary credentials
 *      from Authorization Services. Modifying the local system
 *      identity database requires Admin credentials.
 *  
 *  Result:
 *    Returns true if the commit operation is started, indicated that
 *    an statusUpdated callback will follow. Returns false if the
 *    identity has no uncommitted changes or a commit is already in
 *    progress
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function CSIdentityCommitAsynchronously( identity: CSIdentityRef; const (*var*) clientContext: CSIdentityClientContext; runLoop: CFRunLoopRef; runLoopMode: CFStringRef; authorization: AuthorizationRef { can be NULL } ): Boolean; external name '_CSIdentityCommitAsynchronously';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)
{$endc} {!TARGET_OS_IPHONE && !TARGET_IPHONE_SIMULATOR}


{$ifc not TARGET_OS_IPHONE and not TARGET_IPHONE_SIMULATOR}
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


>>>>>>> origin/fixes_2.4
{
 *  CSIdentityIsCommitting()
 *  
 *  Summary:
 *    Determine if a commit operation is in progress
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    identity:
 *      The identity to access
 *  
 *  Result:
 *    Returns true if a commit operation is in progress
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function CSIdentityIsCommitting( identity: CSIdentityRef ): Boolean; external name '_CSIdentityIsCommitting';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  CSIdentityRemoveClient()
 *  
 *  Summary:
 *    Invalidate an identity's client structure to stop client callbacks
 *  
 *  Discussion:
 *    After returning, this function guarantees that client callbacks
 *    will never be invoked again. Use this function when releasing an
 *    identity which may have an outstanding asynchronous request. This
 *    function does not cancel an outstanding commit operation because
 *    a commit cannot be interrupted.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.5
 *  
 *  Parameters:
 *    
 *    identity:
 *      The identity to access
 *  
 *  Availability:
 *    Mac OS X:         in version 10.5 and later in CoreServices.framework
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
procedure CSIdentityRemoveClient( identity: CSIdentityRef ); external name '_CSIdentityRemoveClient';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_5,__IPHONE_NA) *)
{$endc} {!TARGET_OS_IPHONE && !TARGET_IPHONE_SIMULATOR}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)


{$endc} {TARGET_OS_MAC}
<<<<<<< HEAD
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
=======
>>>>>>> origin/fixes_2.4

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
