{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       LSOpen.h
 
     Contains:   Public interfaces for LaunchServices.framework
 
     Copyright:  (c) 2003-2012 by Apple Inc. All rights reserved.
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
     File:       LaunchServices/LSOpen.h
 
     Contains:   Public interfaces for LaunchServices.framework
 
     Version:    LaunchServices-360.3~1
 
     Copyright:  © 2001-2008 by Apple Computer, Inc., all rights reserved.
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
{	 Pascal Translation: Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }
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

unit LSOpen;
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
uses MacTypes,CFBase,CFArray,CFDictionary,CFURL,Files,Processes,LSInfo,AEDataModel;
{$endc} {not MACOSALLINCLUDE}



{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}

{ ======================================================================================================== }
{ LaunchServices Type & Constants                                                                          }
{ ======================================================================================================== }

type
	LSLaunchFlags = OptionBits;
const
	kLSLaunchDefaults = $00000001; { Defaults = open, async, use Info.plist, start Classic}
	kLSLaunchAndPrint = $00000002; { Print items instead of open them}
	kLSLaunchReserved2 = $00000004;
	kLSLaunchReserved3 = $00000008;
	kLSLaunchReserved4 = $00000010;
	kLSLaunchReserved5 = $00000020;
	kLSLaunchAndDisplayErrors = $00000040; { Report launch/open failures in the UI}
	kLSLaunchInhibitBGOnly = $00000080; { Causes launch to fail if target is background-only.}
	kLSLaunchDontAddToRecents = $00000100; { Do not add app or documents to recents menus.}
	kLSLaunchDontSwitch = $00000200; { Do not bring new app to the foreground.}
	kLSLaunchNoParams = $00000800; { Use Info.plist to determine launch parameters}
	kLSLaunchAsync = $00010000; { Asynchronous launch; return as soon as the app starts launching.}
	kLSLaunchStartClassic = $00020000; { Start up Classic environment if required for app.}
	kLSLaunchInClassic = $00040000; { Force app to launch in Classic environment.}
	kLSLaunchNewInstance = $00080000; { Instantiate app even if it is already running.}
	kLSLaunchAndHide = $00100000; { Send child a "hide" request as soon as it checks in.}
	kLSLaunchAndHideOthers = $00200000; { Hide all other apps when the app checks in.}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kLSLaunchHasUntrustedContents = $00400000;  { Mark items to be opened as untrusted}
=======
	kLSLaunchHasUntrustedContents = $00400000; { Mark items to be opened as untrusted}
>>>>>>> graemeg/cpstrnew
=======
	kLSLaunchHasUntrustedContents = $00400000; { Mark items to be opened as untrusted}
>>>>>>> graemeg/cpstrnew
=======
	kLSLaunchHasUntrustedContents = $00400000; { Mark items to be opened as untrusted}
>>>>>>> graemeg/cpstrnew
=======
	kLSLaunchHasUntrustedContents = $00400000; { Mark items to be opened as untrusted}
>>>>>>> origin/cpstrnew
=======
	kLSLaunchHasUntrustedContents = $00400000; { Mark items to be opened as untrusted}
>>>>>>> origin/fixes_2.4

type
	LSLaunchFSRefSpec = record
		appRef: {const} FSRefPtr;                 { app to use, can be NULL}
		numDocs: ItemCount;                { items to open/print, can be zero}
		itemRefs: {const} FSRefPtr;               { array of FSRefs, ignored when numDocs is zero}
		passThruParams: {const} AEDescPtr;         { passed untouched to application as optional event parameter, }
                                              { with keyword keyAEPropData (can be NULL)}
		launchFlags: LSLaunchFlags;
		asyncRefCon: UnivPtr;            { used if you register for app birth/death notification}
	end;
type
	LSLaunchURLSpec = record
		appURL: CFURLRef;                 { app to use, can be NULL}
		itemURLs: CFArrayRef;               { items to open/print, can be NULL}
		passThruParams: {const} AEDescPtr;         { passed untouched to application as optional parameter (can be NULL)}
		launchFlags: LSLaunchFlags;
		asyncRefCon: UnivPtr;            { used if you register for app birth/death notification}
	end;


{ ======================================================================================================== }
{ LaunchServices API                                                                                       }
{ ======================================================================================================== }

{
 *  LSOpenFSRef()
 *  
 *  Summary:
 *    Open an application, document, or folder.
 *  
 *  Discussion:
 *    Opens applications, documents, and folders. Applications are
 *    opened via an 'oapp' or 'rapp' event. Documents are opened in
 *    their user-overridden or default applications as appropriate.
 *    Folders are opened in the Finder. Use the more specific
 *    LSOpenFromRefSpec for more control over launching.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    inRef:
 *      The FSRef of the item to launch.
 *    
 *    outLaunchedRef:
 *      The FSRef of the item actually launched. For inRefs that are
 *      documents, outLaunchedRef will be the application used to
 *      launch the document. Can be NULL.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
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
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function LSOpenFSRef( const (*var*) inRef: FSRef; outLaunchedRef: FSRefPtr { can be NULL } ): OSStatus; external name '_LSOpenFSRef';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
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
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  LSOpenCFURLRef()
 *  
 *  Summary:
 *    Open an application, document, or folder.
 *  
 *  Discussion:
 *    Opens applications, documents, and folders. Applications are
 *    opened via an 'oapp' or 'rapp' event. Documents are opened in
 *    their user-overridden or default applications as appropriate.
 *    Folders are opened in the Finder. Use the more specific
 *    LSOpenFromURLSpec for more control over launching.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    inURL:
 *      The CFURLRef of the item to launch.
 *    
 *    outLaunchedURL:
 *      The CFURLRef of the item actually launched. For inURLs that are
 *      documents, outLaunchedURL will be the application used to
 *      launch the document. Can be NULL. THIS FUNCTION, DESPITE ITS
 *      NAME, RETAINS THE URL REFERENCE ON BEHALF OF THE CALLER. THE
 *      CALLER MUST EVENTUALLY RELEASE THE RETURNED URL REFERENCE.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
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
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function LSOpenCFURLRef( inURL: CFURLRef; outLaunchedURL: CFURLRefPtr { can be NULL } ): OSStatus; external name '_LSOpenCFURLRef';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
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
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  LSOpenFromRefSpec()
 *  
 *  Summary:
 *    Opens an application or one or more documents or folders.
 *  
 *  Discussion:
 *    Opens applications, documents, and folders.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    inLaunchSpec:
 *      The specification of what to launch and how to launch it.
 *    
 *    outLaunchedRef:
 *      The FSRef of the item actually launched. For inRefs that are
 *      documents, outLaunchedRef will be the application used to
 *      launch the document. Can be NULL.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
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
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function LSOpenFromRefSpec( const (*var*) inLaunchSpec: LSLaunchFSRefSpec; outLaunchedRef: FSRefPtr { can be NULL } ): OSStatus; external name '_LSOpenFromRefSpec';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
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
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  LSOpenFromURLSpec()
 *  
 *  Summary:
 *    Opens an application or one or more documents or folders.
 *  
 *  Discussion:
 *    Opens applications, documents, and folders.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    inLaunchSpec:
 *      The specification of what to launch and how to launch it.
 *    
 *    outLaunchedURL:
 *      The CFURLRef of the item actually launched. For inURLs that are
 *      documents, outLaunchedURL will be the application used to
 *      launch the document. Can be NULL. THIS FUNCTION, DESPITE ITS
 *      NAME, RETAINS THE URL REFERENCE ON BEHALF OF THE CALLER. THE
 *      CALLER MUST EVENTUALLY RELEASE THE RETURNED URL REFERENCE.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in CoreServices.framework
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
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function LSOpenFromURLSpec( const (*var*) inLaunchSpec: LSLaunchURLSpec; outLaunchedURL: CFURLRefPtr { can be NULL } ): OSStatus; external name '_LSOpenFromURLSpec';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_NA) *)
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
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{ ================================================================================== }
{   API for opening with a specific role and additional parameters                   }
{ ================================================================================== }

{
 * LSApplicationParameters
 *
 *    This structure is used by the new LSOpen functions to specify
 *    an application, launch flags, and additional parameters
 *    controlling how an application is launched.
 *
 *    A version field allows the structure to be extended in 
 *    future releases. 
 }
type
	LSApplicationParameters = record
		version: CFIndex;                { This must be set to zero by the client }
		flags: LSLaunchFlags;                  { See the LSLaunchFlags enum }
		application: {const} FSRefPtr;            { The application to open (and possibly handle documents/URLs) }
		asyncLaunchRefCon: UnivPtr;      { The client refCon which will appear in subsequent launch notifications }
		environment: CFDictionaryRef;            { Environment variables to set in the launched process }
                                              { (a dictionary of CFStringRef keys and values). Can be NULL. }
		argv: CFArrayRef;                   { Note: argv is ignored on 10.4. On 10.5 and later, the array elements }
                                              { (which must be CFStringRefs) are passed as arguments to main() in the launched process. }
		initialEvent: AppleEventPtr;           { The first Apple Event to be sent to the launched process. Can be NULL. }
	end;
type
	LSApplicationParametersPtr = ^LSApplicationParameters;
	
{
 *  LSOpenApplication()
 *  
 *    LSOpenApplication launches one application. This function
 *    is an updated alternative to the Process Manager's LaunchApplication().
 *
 *    Launch arguments are specified in the inAppParams argument, which
 *    must be supplied. If the application is already running in the current
 *    session, it will be made the front process (unless the kLSLaunchNewInstance
 *    flag is used, which will always cause a new process to be created).
 *
 *    If outPSN is not NULL, the structure it points to will contain the process 
 *    serial number of the launched (or activated) process. Note that for 
 *    asynchronous launches, the application may not have finished launching
 *    when this function returns.
 }
{
 *  LSOpenApplication()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function LSOpenApplication( const (*var*) appParams: LSApplicationParameters; outPSN: ProcessSerialNumberPtr { can be NULL } ): OSStatus; external name '_LSOpenApplication';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
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
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  LSOpenItemsWithRole()
 *  
 *    Opens the items specified as an array of FSRefs with the role
 *    specified by inRoleMask. If the role doesn't matter, use kLSRolesAll.
 *
 *    Clients can optionally specify the application and launch parameters 
 *    in inAppParams. If a specific application is given in inAppParams, then 
 *    inRoleMask is ignored and the application is launched (if necessary). 
 *    Otherwise, an application will be selected which can handle each input 
 *    item in the specified role(s).
 *
 *    Each launched application will receive an 'odoc' Apple Event specifying
 *    which items are to be opened.
 *
 *    Note that if the input items array contains any applications, this 
 *    function will not launch them unless the kLSRolesShell bit is set
 *    in the inRolesMask (in which case the application is its own shell).
 *
 *    The optional inAEParam argument specifies an AEDesc to be attached to
 *    the Apple Event(s) generated by Launch Services with the specified
 *    AEKeyword.
 *
 *    If not NULL, the outPSNs buffer will be filled with the PSN which
 *    was used to open each item at the same index of the input FSRef array. The
 *    PSN capacity of the output buffer is specified by inMaxPSNCount.
 }
{
 *  LSOpenItemsWithRole()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function LSOpenItemsWithRole( const (*var*) inItems: FSRef; inItemCount: CFIndex; inRole: LSRolesMask; {const} inAEParam: AEKeyDescPtr { can be NULL }; {const} inAppParams: LSApplicationParametersPtr { can be NULL }; outPSNs: ProcessSerialNumberPtr { can be NULL }; inMaxPSNCount: CFIndex ): OSStatus; external name '_LSOpenItemsWithRole';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
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
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  LSOpenURLsWithRole()
 *  
 *    Opens the URLs specified by inURLs (an array of CFURLRefs) with the role
 *    specified by inRoleMask. If the role doesn't matter, use kLSRolesAll.
 *
 *    Clients can optionally specify the application and launch parameters 
 *    in inAppParams. If a specific application is given in inAppParams, then 
 *    inRoleMask is ignored and the application is launched (if necessary). 
 *    Otherwise, an application will be selected which can handle each input 
 *    URL in at least one of the specified role(s).
 *
 *    Each launched application will receive one or more 'GURL' Apple Event 
 *    specifying the URLs to be opened. Clients may also pass file URLs, which
 *    will be interpreted as file system items and opened in the manner of 
 *    LSOpenItemsWithRole (i.e., a handler will be selected base on the item's
 *    metadata).
 *
 *    Note that if the input array contains any application URLs, this 
 *    function will not launch them unless the kLSRolesShell bit is set 
 *    in the inRolesMask (in which case the application is its own shell).
 *
 *    The optional inAEParam argument specifies an AEDesc to be attached to
 *    the Apple Event(s) generated by Launch Services with the specified
 *    AEKeyword.
 *
 *    If not NULL, the outPSNs buffer will be filled with the PSN which
 *    was used to open each URL at the same index of the input URL array. The
 *    PSN capacity of the output buffer is specified by inMaxPSNCount.
 }
{
 *  LSOpenURLsWithRole()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.4
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.4 and later in CoreServices.framework
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/cpstrnew
=======
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }
function LSOpenURLsWithRole( inURLs: CFArrayRef; inRole: LSRolesMask; {const} inAEParam: AEKeyDescPtr { can be NULL }; {const} inAppParams: LSApplicationParametersPtr { can be NULL }; outPSNs: ProcessSerialNumberPtr { can be NULL }; inMaxPSNCount: CFIndex ): OSStatus; external name '_LSOpenURLsWithRole';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)
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
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
>>>>>>> origin/fixes_2.4


{$endc} {TARGET_OS_MAC}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
