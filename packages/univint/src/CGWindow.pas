{ CoreGraphics - CGWindow.h
   Copyright (c) 2006-2008 Apple Inc.
   All rights reserved. }
{       Pascal Translation:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, August 2015 }
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

unit CGWindow;
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
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
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
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
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
uses MacTypes,CGBase,CGGeometry,CGImage,CFBase,CFDictionary,CFArray,CFNumber;
{$endc} {not MACOSALLINCLUDE}

{$ALIGN POWER}


{ The CGWindowID contains a unique value within the user session
   representing a window }

type
	CGWindowID = UInt32;

{ Options for use with these APIs. }
type
	CGWindowListOption = UInt32;
	CGWindowImageOption = UInt32;

{ Values used for `CGWindowSharingType'. }
const
	kCGWindowSharingNone = 0;
	kCGWindowSharingReadOnly = 1;
	kCGWindowSharingReadWrite = 2;
type
	CGWindowSharingType = UInt32;

{ Values used for `CGWindowBackingType'. }
const
	kCGBackingStoreRetained = 0;
	kCGBackingStoreNonretained = 1;
	kCGBackingStoreBuffered = 2;
type
	CGWindowBackingType = UInt32;


{ A value which is never the window ID of any window. }

const
  kCGNullWindowID = CGWindowID(0);

{$ifc TARGET_OS_MAC}
{ Keys for window dictionaries. }

{ The window ID, a unique value within the user session representing the
   window. The value of this key is a CFNumber 32-bit signed integer
   value. }

var kCGWindowNumber: CFStringRef; external name '_kCGWindowNumber'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)

{ The backing store type of the window, one of `kCGBackingStoreRetained',
   `kCGBackingStoreNonretained', or `kCGBackingStoreBuffered'. The value of
   this key is a CFNumber 32-bit signed integer value. }

var kCGWindowStoreType: CFStringRef; external name '_kCGWindowStoreType'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)

{ The window layer number of the window. The value of this key is a
   CFNumber 32-bit signed integer value. }

var kCGWindowLayer: CFStringRef; external name '_kCGWindowLayer'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)

{ The bounds of the window in screen space, with the origin at the
   upper-left corner of the main display. The value of this key is a
   CFDictionary; use `CGRectMakeWithDictionaryRepresentation' to obtain the
   bounds as a CGRect value. }

var kCGWindowBounds: CFStringRef; external name '_kCGWindowBounds'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)

{ The sharing state of the window, one of `kCGWindowSharingNone',
   `kCGWindowSharingReadOnly', or `kCGWindowSharingReadWrite'. The value of
   this key is a CFNumber 32-bit signed integer value. }

var kCGWindowSharingState: CFStringRef; external name '_kCGWindowSharingState'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)

{ The alpha fade of the window. The value of this key is a CFNumber
   floating-point value. The value 1.0 is normal (opaque); the value 0.0 is
   fully transparent (invisible). }

var kCGWindowAlpha: CFStringRef; external name '_kCGWindowAlpha'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)

{ The process ID of the process that owns the window. The value of this key
   is a CFNumber 32-bit signed integer value. }

var kCGWindowOwnerPID: CFStringRef; external name '_kCGWindowOwnerPID'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)

{ An estimate of the memory in bytes currently used by the window and its
   supporting data structures. The value of this key is a CFNumber 64-bit
   signed integer value. }

var kCGWindowMemoryUsage: CFStringRef; external name '_kCGWindowMemoryUsage'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)

{ Optional keys for window dictionaries. }

{ If present, the workspace ID of the workspace associated with the window.
   The value of this key is a CFNumber 32-bit signed integer value. }

var kCGWindowWorkspace: CFStringRef; external name '_kCGWindowWorkspace'; (* attribute const *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_5, __MAC_10_8, __IPHONE_NA, __IPHONE_NA) *)
=======
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)
>>>>>>> origin/cpstrnew
=======
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)
>>>>>>> origin/fixes_2.4

{ If present, the name of the application process which owns the window.
   The value of this key is a CFString. }

var kCGWindowOwnerName: CFStringRef; external name '_kCGWindowOwnerName'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)

{ If present, the name of the window. The value of this key is a
   CFString. }

var kCGWindowName: CFStringRef; external name '_kCGWindowName'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)

{ If present, true if the window is ordered on screen, false otherwise. If
   the key is not present, then the window is not ordered on screen. The
   value of this key is a CFBoolean. }

var kCGWindowIsOnscreen: CFStringRef; external name '_kCGWindowIsOnscreen'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)

{ If present, true if the window backing store is in video memory, false
   otherwise. If the key is not present, then the window backing store is in
   main memory. The value of this key is a CFBoolean. }

var kCGWindowBackingLocationVideoMemory: CFStringRef; external name '_kCGWindowBackingLocationVideoMemory'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)

{$endc} {TARGET_OS_MAC}

{ Flags for CGWindowListOption values.  These may be ORed together. }

const
{ List all windows in this user session, including both on- and
     off-screen windows. The parameter `relativeToWindow' should be
     `kCGNullWindowID'. }
	kCGWindowListOptionAll = 0;

  { List all on-screen windows in this user session, ordered from front to
     back. The parameter `relativeToWindow' should be `kCGNullWindowID'. }
	kCGWindowListOptionOnScreenOnly = 1 shl 0;

  { List all on-screen windows above the window specified by
     `relativeToWindow', ordered from front to back. }
	kCGWindowListOptionOnScreenAboveWindow = 1 shl 1;

  { List all on-screen windows below the window specified by
     `relativeToWindow', ordered from front to back. }
	kCGWindowListOptionOnScreenBelowWindow = 1 shl 2;

  { Include the window specified by `relativeToWindow' in any list,
     effectively creating `at-or-above' or `at-or-below' lists. }
	kCGWindowListOptionIncludingWindow = 1 shl 3;
    
  { Exclude any windows from the list that are elements of the desktop. }
	kCGWindowListExcludeDesktopElements = 1 shl 4;

{$ifc TARGET_OS_MAC}
{ Return an array of window dictionaries for windows within the user
   session.

   This function returns NULL if the caller is not running within a Quartz
   GUI session or the window server is disabled. You should release the
   array when you are finished using it. }

function CGWindowListCopyWindowInfo( option: CGWindowListOption; relativeToWindow: CGWindowID ): CFArrayRef; external name '_CGWindowListCopyWindowInfo';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)

{ Return an array of CGWindowID values for windows within the user session.

   This function returns NULL if the caller is not running within a Quartz
   GUI session or the window server is disabled. You should release the
   array when you are finished using it. }

function CGWindowListCreate( option: CGWindowListOption; relativeToWindow: CGWindowID ): CFArrayRef; external name '_CGWindowListCreate';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)

{ Return an array of window dictionaries, each corresponding to a window ID
   specified in `windowArray'.

   This function returns NULL if the caller is not running within a Quartz
   GUI session or the window server is disabled. You should release the
   array when you are finished using it. }

function CGWindowListCreateDescriptionFromArray( windowArray: CFArrayRef ): CFArrayRef; external name '_CGWindowListCreateDescriptionFromArray';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)

{$endc} {TARGET_OS_MAC}

{ Flags for CGWindowImageOption values.  These may be ORed together. }

const
{ If `CGRectNull' is passed as the screen bounds, then then bounds
     computation includes window frame ornamentation, such as a shadow. }
	kCGWindowImageDefault = 0;

  { If `CGRectNull' is passed as the screen bounds, then then bounds
     computation excludes window frame ornamentation, such as a shadow. }
	kCGWindowImageBoundsIgnoreFraming = 1 shl 0;

  { Force the created image to be opaque.  Empty areas are white }
	kCGWindowImageShouldBeOpaque = 1 shl 1;

  { Only draw the windows' shadows, not the windows themselves. }
	kCGWindowImageOnlyShadows = 1 shl 2;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  { Return the best image resolution. The screen size may be
     different than the returned image size. }
	kCGWindowImageBestResolution = 1 shl 3;

  { Return the nominal image resolution. The screen size
     equals the returned image size. }
	kCGWindowImageNominalResolution = 1 shl 4;

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
{$ifc TARGET_OS_MAC}

{ Create an image containing a composite of the specified set of windows
   contained within a rectangular area. The set of windows is specified
   using options from `CGWindowListOption', along with an optional
   additional window ID.

   The windows list options are:

   --- kCGWindowListOptionAll, kCGWindowListOptionOnScreenOnly: Use all
   on-screen windows in this user session to construct the image. The
   parameter `windowID' should be `kCGNullWindowID'.

   --- kCGWindowListOptionOnScreenAboveWindow: Use all on-screen windows in
   this user session above the window specified by `windowID', ordered from
   front to back, to construct the image. To include the window specified by
   `windowID', add the flag `kCGWindowListOptionIncludingWindow'.

   --- kCGWindowListOptionOnScreenBelowWindow: Use all on-screen windows in
   this user session below the window specified by `windowID', ordered from
   front to back, to construct the image. To include the window specified by
   `windowID', add the flag `kCGWindowListOptionIncludingWindow'.

   --- kCGWindowListOptionIncludingWindow: Use only the window specified by
   `windowID' to construct the image.

   The parameter `screenBounds' specifies the rectangle in screen space
   (origin at the upper-left; y-value increasing downward). Setting
   `screenBounds' to `CGRectInfinite' will include all the windows on the
   entire desktop. Setting `screenBounds' to `CGRectNull' will use the
   bounding box of the specified windows as the screen space rectangle.

   The parameter `imageOptions' allows you to specify whether the window
   frame ornamentation, such as a shadow or similar effect, should be
   included or excluded in the bounds calculation when `CGRectNull' is
   specified for the window bounds.

   If no windows meet the specified criteria, or the windows can't be read,
   then a transparent black image will be returned.

   Any on-screen window with sharing type `kCGWindowSharingNone' will not
   be included in the image.

   This function returns NULL if the caller is not running within a Quartz
   GUI session or the window server is disabled. }

function CGWindowListCreateImage( screenBounds: CGRect; listOption: CGWindowListOption; windowID: CGWindowID; imageOption: CGWindowImageOption ): CGImageRef; external name '_CGWindowListCreateImage';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)

{ Create an image containing a composite of the specified set of windows
   contained within a rectangular area à la `CGWindowListCreateImage'. The
   set of windows is specified by `windowArray', an array of window IDs. }

function CGWindowListCreateImageFromArray( screenBounds: CGRect; windowArray: CFArrayRef; imageOption: CGWindowImageOption ): CGImageRef; external name '_CGWindowListCreateImageFromArray';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_NA) *)

{$endc} {TARGET_OS_MAC}

{ A CFNumberRef encoding appropriate for use with a CGWindowID. }
const
  kCGWindowIDCFNumberType = kCFNumberSInt32Type;

{ CFNumberRef encoding appropriate for use with CGWindowSharingType }
const
  kCGWindowSharingCFNumberType = kCFNumberSInt32Type;

{ CFNumberRef encoding appropriate for use with CGWindowBackingType }
const
  kCGWindowBackingCFNumberType = kCFNumberSInt32Type;

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
