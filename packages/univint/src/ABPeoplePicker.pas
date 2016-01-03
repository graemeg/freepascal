<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{
>>>>>>> origin/fixes_2.4
//
//  ABPeoplePickerC.h
//  AddressBook Framework
//
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
//  Copyright (c) 2003-2007 Apple Inc.  All rights reserved.
//
}
{	  Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, 2004 }
{	  Pascal Translation Updated:  Gorazd Krosl, <gorazd_1957@yahoo.ca>, November 2009 }

{
    Modified for use with Free Pascal
    Version 308
    Please report any bugs to <gpc@microbizz.nl>
}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
//  Copyright (c) 2003 Apple Computer. All rights reserved.
=======
//  Copyright (c) 2003-2007 Apple Inc.  All rights reserved.
>>>>>>> origin/fixes_2.4
//
}
{	  Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, 2004 }
{	  Pascal Translation Updated:  Gorazd Krosl, <gorazd_1957@yahoo.ca>, November 2009 }

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

unit ABPeoplePicker;
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
<<<<<<< HEAD
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
=======
=======
>>>>>>> origin/cpstrnew
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
<<<<<<< HEAD
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
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
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
=======
>>>>>>> origin/fixes_2.4
uses MacTypes,ABAddressBook,CFBase,CFArray,CGGeometry,HIGeometry,CarbonEventsCore;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}

<<<<<<< HEAD

type
	ABPickerRef = ^OpaqueABPicker; { an opaque type }
	OpaqueABPicker = record end;
=======
=======
>>>>>>> origin/fixes_2_2
uses MacTypes,ABAddressBook,CFBase,CFArray,CGGeometry,Drag,HIObjectCore,HIGeometry;
{$ALIGN MAC68K}

type
	ABPickerRef							= ^SInt32;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

type
	ABPickerRef = ^OpaqueABPicker; { an opaque type }
	OpaqueABPicker = record end;
>>>>>>> origin/fixes_2.4

{
 * Picker creation and manipulation
 }

// Creates an ABPickerRef. Release with CFRelease(). The window is created hidden. Call
// ABPickerSetVisibility() to show it.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function ABPickerCreate: ABPickerRef; external name '_ABPickerCreate';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

// Change the structural frame of the window.
procedure ABPickerSetFrame( inPicker: ABPickerRef; const (*var*) inFrame: HIRect ); external name '_ABPickerSetFrame';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
procedure ABPickerGetFrame( inPicker: ABPickerRef; var outFrame: HIRect ); external name '_ABPickerGetFrame';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

procedure ABPickerSetVisibility( inPicker: ABPickerRef; visible: CBool ); external name '_ABPickerSetVisibility';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
function ABPickerIsVisible( inPicker: ABPickerRef ): CBool; external name '_ABPickerIsVisible';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
=======
=======
>>>>>>> origin/fixes_2_2
// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
=======
>>>>>>> origin/fixes_2.4
function ABPickerCreate: ABPickerRef; external name '_ABPickerCreate';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

// Change the structural frame of the window.
procedure ABPickerSetFrame( inPicker: ABPickerRef; const (*var*) inFrame: HIRect ); external name '_ABPickerSetFrame';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
procedure ABPickerGetFrame( inPicker: ABPickerRef; var outFrame: HIRect ); external name '_ABPickerGetFrame';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

<<<<<<< HEAD
// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
procedure ABPickerSetVisibility( inPicker: ABPickerRef; visible: Boolean ); external name '_ABPickerSetVisibility';
// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
function ABPickerIsVisible( inPicker: ABPickerRef ): Boolean; external name '_ABPickerIsVisible';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure ABPickerSetVisibility( inPicker: ABPickerRef; visible: CBool ); external name '_ABPickerSetVisibility';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
function ABPickerIsVisible( inPicker: ABPickerRef ): CBool; external name '_ABPickerIsVisible';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/fixes_2.4

{
 * Look and Feel
 }

const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
// Choose the selection behavior for the value column. If multiple behaviors are selected,
    // the most restrictive behavior will be used. Defaults to kABPickerSingleValueSelection set
    // to TRUE.
	kABPickerSingleValueSelection = 1 shl 0; // Allow user to choose a single value for a person.
	kABPickerMultipleValueSelection = 1 shl 1; // Allow user to choose multiple values for a person.

    // Allow the user to select entire groups in the group column. If false, at least one
    // person in the group will be selected. Defaults to FALSE.
	kABPickerAllowGroupSelection = 1 shl 2;

    // Allow the user to select more than one group/record at a time. Defaults to TRUE.
	kABPickerAllowMultipleSelection = 1 shl 3;

type
	ABPickerAttributes = OptionBits;

function ABPickerGetAttributes( inPicker: ABPickerRef ): ABPickerAttributes; external name '_ABPickerGetAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
procedure ABPickerChangeAttributes( inPicker: ABPickerRef; inAttributesToSet: ABPickerAttributes; inAttributesToClear: ABPickerAttributes ); external name '_ABPickerChangeAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
=======
=======
>>>>>>> origin/fixes_2_2
    // Choose the selection behavior for the value column. If multiple behaviors are selected,
=======
// Choose the selection behavior for the value column. If multiple behaviors are selected,
>>>>>>> origin/fixes_2.4
    // the most restrictive behavior will be used. Defaults to kABPickerSingleValueSelection set
    // to TRUE.
	kABPickerSingleValueSelection = 1 shl 0; // Allow user to choose a single value for a person.
	kABPickerMultipleValueSelection = 1 shl 1; // Allow user to choose multiple values for a person.

    // Allow the user to select entire groups in the group column. If false, at least one
    // person in the group will be selected. Defaults to FALSE.
	kABPickerAllowGroupSelection = 1 shl 2;

    // Allow the user to select more than one group/record at a time. Defaults to TRUE.
	kABPickerAllowMultipleSelection = 1 shl 3;

type
	ABPickerAttributes = OptionBits;

function ABPickerGetAttributes( inPicker: ABPickerRef ): ABPickerAttributes; external name '_ABPickerGetAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
procedure ABPickerChangeAttributes( inPicker: ABPickerRef; inAttributesToSet: ABPickerAttributes; inAttributesToClear: ABPickerAttributes ); external name '_ABPickerChangeAttributes';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/fixes_2.4

{
 * Value column
 }

    // These methods control what data (if any) is shown in the values column. The column will only
    // display if an AB property is added. A popup button in the column header will be used if more
    // than one property is added. Titles for built in properties will localized automatically. A
    // list of AB properties can be found in <AddressBook/ABGlobals.h>.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure ABPickerAddProperty( inPicker: ABPickerRef; inProperty: CFStringRef ); external name '_ABPickerAddProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
procedure ABPickerRemoveProperty( inPicker: ABPickerRef; inProperty: CFStringRef ); external name '_ABPickerRemoveProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
    // Returns an array of AB Properties as CFStringRefs.
function ABPickerCopyProperties( inPicker: ABPickerRef ): CFArrayRef; external name '_ABPickerCopyProperties';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

    // Localized titles for third party properties should be set with these methods.
procedure ABPickerSetColumnTitle( inPicker: ABPickerRef; inTitle: CFStringRef; inProperty: CFStringRef ); external name '_ABPickerSetColumnTitle';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
function ABPickerCopyColumnTitle( inPicker: ABPickerRef; inProperty: CFStringRef ): CFStringRef; external name '_ABPickerCopyColumnTitle';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

    // Display one of the properties added above in the values column.
procedure ABPickerSetDisplayedProperty( inPicker: ABPickerRef; inProperty: CFStringRef ); external name '_ABPickerSetDisplayedProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
function ABPickerCopyDisplayedProperty( inPicker: ABPickerRef ): CFStringRef; external name '_ABPickerCopyDisplayedProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
=======
=======
>>>>>>> origin/fixes_2_2
// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
=======
>>>>>>> origin/fixes_2.4
procedure ABPickerAddProperty( inPicker: ABPickerRef; inProperty: CFStringRef ); external name '_ABPickerAddProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
procedure ABPickerRemoveProperty( inPicker: ABPickerRef; inProperty: CFStringRef ); external name '_ABPickerRemoveProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
    // Returns an array of AB Properties as CFStringRefs.
function ABPickerCopyProperties( inPicker: ABPickerRef ): CFArrayRef; external name '_ABPickerCopyProperties';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

    // Localized titles for third party properties should be set with these methods.
procedure ABPickerSetColumnTitle( inPicker: ABPickerRef; inTitle: CFStringRef; inProperty: CFStringRef ); external name '_ABPickerSetColumnTitle';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
function ABPickerCopyColumnTitle( inPicker: ABPickerRef; inProperty: CFStringRef ): CFStringRef; external name '_ABPickerCopyColumnTitle';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

    // Display one of the properties added above in the values column.
procedure ABPickerSetDisplayedProperty( inPicker: ABPickerRef; inProperty: CFStringRef ); external name '_ABPickerSetDisplayedProperty';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
function ABPickerCopyDisplayedProperty( inPicker: ABPickerRef ): CFStringRef; external name '_ABPickerCopyDisplayedProperty';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/fixes_2.4

{
 * Selection
 }

    // Returns group column selection as an array of ABGroupRef objects.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function ABPickerCopySelectedGroups( inPicker: ABPickerRef ): CFArrayRef; external name '_ABPickerCopySelectedGroups';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

    // Returns names column selection as an array of ABGroupRef or ABPersonRef objects.
function ABPickerCopySelectedRecords( inPicker: ABPickerRef ): CFArrayRef; external name '_ABPickerCopySelectedRecords';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

    // This method returns an array of selected multi-value identifiers. Returns nil if the displayed
    // property is a single value type.
function ABPickerCopySelectedIdentifiers( inPicker: ABPickerRef; inPerson: ABPersonRef ): CFArrayRef; external name '_ABPickerCopySelectedIdentifiers';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

    // Returns an array containing CFStringRefs for each item selected in the values column.
function ABPickerCopySelectedValues( inPicker: ABPickerRef ): CFArrayRef; external name '_ABPickerCopySelectedValues';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

    // Select group/name/value programatically.
procedure ABPickerSelectGroup( inPicker: ABPickerRef; inGroup: ABGroupRef; inExtendSelection: CBool ); external name '_ABPickerSelectGroup';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
procedure ABPickerSelectRecord( inPicker: ABPickerRef; inRecord: ABRecordRef; inExtendSelection: CBool ); external name '_ABPickerSelectRecord';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
    // Individual values contained within an multi-value property can be selected with this method.
procedure ABPickerSelectIdentifier( inPicker: ABPickerRef; inPerson: ABPersonRef; inIdentifier: CFStringRef; inExtendSelection: CBool ); external name '_ABPickerSelectIdentifier';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

    // Remove selection
procedure ABPickerDeselectGroup( inPicker: ABPickerRef; inGroup: ABGroupRef ); external name '_ABPickerDeselectGroup';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
procedure ABPickerDeselectRecord( inPicker: ABPickerRef; inRecord: ABRecordRef ); external name '_ABPickerDeselectRecord';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
procedure ABPickerDeselectIdentifier( inPicker: ABPickerRef; inPerson: ABPersonRef; inIdentifier: CFStringRef ); external name '_ABPickerDeselectIdentifier';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

procedure ABPickerDeselectAll( inPicker: ABPickerRef ); external name '_ABPickerDeselectAll';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
=======
=======
>>>>>>> origin/fixes_2_2
// AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER
=======
>>>>>>> origin/fixes_2.4
function ABPickerCopySelectedGroups( inPicker: ABPickerRef ): CFArrayRef; external name '_ABPickerCopySelectedGroups';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

    // Returns names column selection as an array of ABGroupRef or ABPersonRef objects.
function ABPickerCopySelectedRecords( inPicker: ABPickerRef ): CFArrayRef; external name '_ABPickerCopySelectedRecords';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

    // This method returns an array of selected multi-value identifiers. Returns nil if the displayed
    // property is a single value type.
function ABPickerCopySelectedIdentifiers( inPicker: ABPickerRef; inPerson: ABPersonRef ): CFArrayRef; external name '_ABPickerCopySelectedIdentifiers';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

    // Returns an array containing CFStringRefs for each item selected in the values column.
function ABPickerCopySelectedValues( inPicker: ABPickerRef ): CFArrayRef; external name '_ABPickerCopySelectedValues';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

    // Select group/name/value programatically.
procedure ABPickerSelectGroup( inPicker: ABPickerRef; inGroup: ABGroupRef; inExtendSelection: CBool ); external name '_ABPickerSelectGroup';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
procedure ABPickerSelectRecord( inPicker: ABPickerRef; inRecord: ABRecordRef; inExtendSelection: CBool ); external name '_ABPickerSelectRecord';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
    // Individual values contained within an multi-value property can be selected with this method.
procedure ABPickerSelectIdentifier( inPicker: ABPickerRef; inPerson: ABPersonRef; inIdentifier: CFStringRef; inExtendSelection: CBool ); external name '_ABPickerSelectIdentifier';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

    // Remove selection
procedure ABPickerDeselectGroup( inPicker: ABPickerRef; inGroup: ABGroupRef ); external name '_ABPickerDeselectGroup';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
procedure ABPickerDeselectRecord( inPicker: ABPickerRef; inRecord: ABRecordRef ); external name '_ABPickerDeselectRecord';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
procedure ABPickerDeselectIdentifier( inPicker: ABPickerRef; inPerson: ABPersonRef; inIdentifier: CFStringRef ); external name '_ABPickerDeselectIdentifier';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

procedure ABPickerDeselectAll( inPicker: ABPickerRef ); external name '_ABPickerDeselectAll';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
>>>>>>> origin/fixes_2.4

{
 * Events and Actions
 *
 * Your delegate will be notified when the user changes the selection or displayed property of the picker.
 * Picker events have an event class of kEventClassABPeoplePicker and one of the kinds listed below. Picker
 * events contain an event parameter which contains the ABPickerRef. To obtain this:
 *
 * GetEventParameter(inEvent, kEventParamABPickerRef,
 *                   typeCFTypeRef, NULL, sizeof(ABPickerRef),
 *                   NULL, &outPickerRef);
 *
 }

const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
// Carbon Event class for People Picker
	kEventClassABPeoplePicker = FourCharCode('abpp');
=======
    // Carbon Event class for People Picker
    kEventClassABPeoplePicker = FourCharCode('abpp');
>>>>>>> graemeg/fixes_2_2
=======
    // Carbon Event class for People Picker
    kEventClassABPeoplePicker = FourCharCode('abpp');
>>>>>>> origin/fixes_2_2
=======
// Carbon Event class for People Picker
	kEventClassABPeoplePicker = FourCharCode('abpp');
>>>>>>> origin/fixes_2.4

const
    // Carbon Event kinds for People Picker
    kEventABPeoplePickerGroupSelectionChanged     = 1;
    kEventABPeoplePickerNameSelectionChanged      = 2;
    kEventABPeoplePickerValueSelectionChanged     = 3;
    kEventABPeoplePickerDisplayedPropertyChanged  = 4;

    kEventABPeoplePickerGroupDoubleClicked        = 5;
    kEventABPeoplePickerNameDoubleClicked         = 6;

const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
// Carbon Event parameter name
	kEventParamABPickerRef = FourCharCode('abpp');

    // Set the event handler for People Picker events.
procedure ABPickerSetDelegate( inPicker: ABPickerRef; inDelegate: EventTargetRef ); external name '_ABPickerSetDelegate';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
function ABPickerGetDelegate( inPicker: ABPickerRef ): EventTargetRef; external name '_ABPickerGetDelegate';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

    // Clear the search field and reset the list of displayed names.
procedure ABPickerClearSearchField( inPicker: ABPickerRef ); external name '_ABPickerClearSearchField';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

    // Launch AddressBook and edit the current selection
procedure ABPickerEditInAddressBook( inPicker: ABPickerRef ); external name '_ABPickerEditInAddressBook';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
procedure ABPickerSelectInAddressBook( inPicker: ABPickerRef ); external name '_ABPickerSelectInAddressBook';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
    // Carbon Event parameter name
    kEventParamABPickerRef	=  FourCharCode('abpp');
=======
// Carbon Event parameter name
	kEventParamABPickerRef = FourCharCode('abpp');
>>>>>>> origin/fixes_2.4

    // Set the event handler for People Picker events.
procedure ABPickerSetDelegate( inPicker: ABPickerRef; inDelegate: EventTargetRef ); external name '_ABPickerSetDelegate';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
function ABPickerGetDelegate( inPicker: ABPickerRef ): EventTargetRef; external name '_ABPickerGetDelegate';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

    // Clear the search field and reset the list of displayed names.
procedure ABPickerClearSearchField( inPicker: ABPickerRef ); external name '_ABPickerClearSearchField';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

    // Launch AddressBook and edit the current selection
procedure ABPickerEditInAddressBook( inPicker: ABPickerRef ); external name '_ABPickerEditInAddressBook';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
procedure ABPickerSelectInAddressBook( inPicker: ABPickerRef ); external name '_ABPickerSelectInAddressBook';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

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
