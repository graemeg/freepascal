{
     File:       HIToolbox/Drag.h
 
     Contains:   Drag and Drop Interfaces.
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Version:    HIToolbox-624~3
=======
     Version:    HIToolbox-437~1
>>>>>>> graemeg/cpstrnew
=======
     Version:    HIToolbox-437~1
>>>>>>> graemeg/cpstrnew
=======
     Version:    HIToolbox-437~1
>>>>>>> graemeg/cpstrnew
=======
     Version:    HIToolbox-437~1
>>>>>>> origin/cpstrnew
 
     Copyright:  © 1992-2008 by Apple Computer, Inc., all rights reserved.
=======
     Version:    HIToolbox-219.4.81~2
 
     Copyright:  © 1992-2005 by Apple Computer, Inc., all rights reserved.
>>>>>>> graemeg/fixes_2_2
=======
     Version:    HIToolbox-219.4.81~2
 
     Copyright:  © 1992-2005 by Apple Computer, Inc., all rights reserved.
>>>>>>> origin/fixes_2_2
=======
     Version:    HIToolbox-437~1
 
     Copyright:  © 1992-2008 by Apple Computer, Inc., all rights reserved.
>>>>>>> origin/fixes_2.4
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}
{       Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
=======
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
                     http://www.freepascal.org/bugs.html
 
}
{       Pascal Translation Updated:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
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

unit Drag;
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
=======
>>>>>>> origin/cpstrnew
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
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
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
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
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
uses MacTypes,AEDataModel,CGImage,CGGeometry,Events,Files,AppleEvents,QuickdrawTypes,Pasteboard,HIGeometry;
{$endc} {not MACOSALLINCLUDE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew

{$ifc TARGET_OS_MAC}
=======
uses MacTypes,AEDataModel,CGImage,CGGeometry,Events,Files,AppleEvents,Quickdraw,Pasteboard,HIGeometry;

>>>>>>> graemeg/fixes_2_2
=======
uses MacTypes,AEDataModel,CGImage,CGGeometry,Events,Files,AppleEvents,Quickdraw,Pasteboard,HIGeometry;

>>>>>>> origin/fixes_2_2
=======
uses MacTypes,AEDataModel,CGImage,CGGeometry,Events,Files,AppleEvents,QuickdrawTypes,Pasteboard,HIGeometry;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}
>>>>>>> origin/fixes_2.4

{$ALIGN MAC68K}

{
  _________________________________________________________________________________________________________
      
   ¥ DRAG MANAGER DATA TYPES
  _________________________________________________________________________________________________________
}
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	DragRef = ^OpaqueDragRef; { an opaque type }
	OpaqueDragRef = record end;
=======
	DragRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	DragRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	DragRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	DragRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
	DragRefPtr = ^DragRef;  { when a var xx:DragRef parameter can be nil, it is changed to xx: DragRefPtr }
	DragItemRef = URefCon;
=======
=======
>>>>>>> origin/fixes_2_2
	DragRef = ^SInt32; { an opaque 32-bit type }
	DragRefPtr = ^DragRef;  { when a var xx:DragRef parameter can be nil, it is changed to xx: DragRefPtr }
type
	DragItemRef = UInt32;
type
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	DragRef = ^SInt32; { an opaque type }
	DragRefPtr = ^DragRef;  { when a var xx:DragRef parameter can be nil, it is changed to xx: DragRefPtr }
	DragItemRef = URefCon;
>>>>>>> origin/fixes_2.4
	FlavorType = OSType;
{
  _________________________________________________________________________________________________________
      
   ¥ DRAG ATTRIBUTES
  _________________________________________________________________________________________________________
}
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	DragAttributes = OptionBits;
=======
	DragAttributes = UInt32;
>>>>>>> graemeg/fixes_2_2
=======
	DragAttributes = UInt32;
>>>>>>> origin/fixes_2_2
=======
	DragAttributes = OptionBits;
>>>>>>> origin/fixes_2.4
const
	kDragHasLeftSenderWindow = 1 shl 0; { drag has left the source window since TrackDrag}
	kDragInsideSenderApplication = 1 shl 1; { drag is occurring within the sender application}
	kDragInsideSenderWindow = 1 shl 2; { drag is occurring within the sender window}

{
  _________________________________________________________________________________________________________
      
   ¥ DRAG BEHAVIORS
  _________________________________________________________________________________________________________
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

type
	DragBehaviors = OptionBits;
=======
type
	DragBehaviors = UInt32;
>>>>>>> graemeg/fixes_2_2
=======
type
	DragBehaviors = UInt32;
>>>>>>> origin/fixes_2_2
=======

type
	DragBehaviors = OptionBits;
>>>>>>> origin/fixes_2.4
const
	kDragBehaviorNone = 0;
	kDragBehaviorZoomBackAnimation = 1 shl 0; { do zoomback animation for failed drags (normally enabled).}

{
  _________________________________________________________________________________________________________
      
   ¥ DRAG IMAGE FLAGS
  _________________________________________________________________________________________________________
}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  DragImageFlags
 *  
 *  Summary:
 *    Parameters to SetDragImage and SetDragImageWithCGImage.
 }
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	DragImageFlags = OptionBits;
const
{
   * Indicates that the outline region passed to TrackDrag should be
   * drawn onscreen, in addition to the translucent drag image. Do not
   * set this flag when passing your own drag reference to
   * HIWindowTrackProxyDrag.
=======
=======
>>>>>>> origin/fixes_2_2
	DragImageFlags = UInt32;
const
{
   * Indicates that the outline region passed to TrackDrag should be
   * drawn onscreen, in addition to the translucent drag image.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	DragImageFlags = OptionBits;
const
{
   * Indicates that the outline region passed to TrackDrag should be
   * drawn onscreen, in addition to the translucent drag image. Do not
   * set this flag when passing your own drag reference to
   * HIWindowTrackProxyDrag.
>>>>>>> origin/fixes_2.4
   }
	kDragRegionAndImage = 1 shl 4;

  {
   * Indicates that the image and offset being passed in are already at
   * device resolution, and the image should be drawn as-is. If this
   * option is not specified, the image will be scaled according to the
   * user's current scaling factor. Available in Mac OS X 10.4 and
   * later.
   }
	kDragDoNotScaleImage = 1 shl 5;

{
  _________________________________________________________________________________________________________
      
   ¥ DRAG IMAGE TRANSLUCENCY LEVELS
  _________________________________________________________________________________________________________
}

const
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
	kDragStandardTranslucency = 0;    { 65% image translucency (standard)}
	kDragDarkTranslucency = 1;    { 50% image translucency}
	kDragDarkerTranslucency = 2;    { 25% image translucency}
	kDragOpaqueTranslucency = 3;     { 0% image translucency (opaque)}
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
	kDragStandardTranslucency = 0;   { 65% image translucency (standard)}
	kDragDarkTranslucency = 1;   { 50% image translucency}
	kDragDarkerTranslucency = 2;   { 25% image translucency}
	kDragOpaqueTranslucency = 3;    { 0% image translucency (opaque)}

{
  _________________________________________________________________________________________________________
      
   ¥ DRAG DRAWING PROCEDURE MESSAGES
  _________________________________________________________________________________________________________
}

type
	DragRegionMessage = SInt16;
const
	kDragRegionBegin = 1;    { initialize drawing}
	kDragRegionDraw = 2;    { draw drag feedback}
	kDragRegionHide = 3;    { hide drag feedback}
	kDragRegionIdle = 4;    { drag feedback idle time}
	kDragRegionEnd = 5;     { end of drawing}

{
  _________________________________________________________________________________________________________
      
   ¥ ZOOM ACCELERATION
  _________________________________________________________________________________________________________
}

type
	ZoomAcceleration = SInt16;
const
	kZoomNoAcceleration = 0;    { use linear interpolation}
	kZoomAccelerate = 1;    { ramp up step size}
	kZoomDecelerate = 2;     { ramp down step size}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{
  _________________________________________________________________________________________________________
      
   ¥ FLAVOR FLAGS
  _________________________________________________________________________________________________________
}

type
	FlavorFlags = UInt32;
const
	flavorSenderOnly = 1 shl 0; { flavor is available to sender only}
	flavorSenderTranslated = 1 shl 1; { flavor is translated by sender}
	flavorNotSaved = 1 shl 2; { flavor should not be saved}
	flavorSystemTranslated = 1 shl 8; { flavor is translated by system}
	flavorDataPromised = 1 shl 9; { flavor data is promised by sender}

{
  _________________________________________________________________________________________________________
      
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
   ¥ FILE SYSTEM CONSTANTS
  _________________________________________________________________________________________________________
}

const
	kDragFlavorTypeHFS = FourCharCode('hfs '); { flavor type for HFS data}
	kDragFlavorTypePromiseHFS = FourCharCode('phfs'); { flavor type for promised HFS data}
	flavorTypeHFS = kDragFlavorTypeHFS; { old name}
	flavorTypePromiseHFS = kDragFlavorTypePromiseHFS; { old name}

const
	kDragPromisedFlavorFindFile = FourCharCode('rWm1'); { promisedFlavor value for Find File}
	kDragPromisedFlavor = FourCharCode('fssP'); { promisedFlavor value for everything else}

const
	kDragPseudoCreatorVolumeOrDirectory = FourCharCode('MACS'); { "creator code" for volume or directory}
	kDragPseudoFileTypeVolume = FourCharCode('disk'); { "file type" for volume}
	kDragPseudoFileTypeDirectory = FourCharCode('fold'); { "file type" for directory}

{
  _________________________________________________________________________________________________________
      
   ¥ SPECIAL FLAVORS
  _________________________________________________________________________________________________________
}

const
	flavorTypeDirectory = FourCharCode('diry'); { flavor type for AOCE directories}

{
  _________________________________________________________________________________________________________
      
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
   ¥ FLAVORS FOR FINDER 8.0 AND LATER
  _________________________________________________________________________________________________________
}

const
	kFlavorTypeClippingName = FourCharCode('clnm'); { name hint for clipping file (preferred over 'clfn')}
	kFlavorTypeClippingFilename = FourCharCode('clfn'); { name for clipping file}
	kFlavorTypeUnicodeClippingName = FourCharCode('ucln'); { unicode name hint for clipping file (preferred over 'uclf')}
	kFlavorTypeUnicodeClippingFilename = FourCharCode('uclf'); { unicode name for clipping file}
	kFlavorTypeDragToTrashOnly = FourCharCode('fdtt'); { for apps that want to allow dragging private data to the trash}
	kFlavorTypeFinderNoTrackingBehavior = FourCharCode('fntb'); { Finder completely ignores any drag containing this flavor}

{
  _________________________________________________________________________________________________________
      
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   ¥ DRAG ACTIONS
=======
   ¥ DRAG TRACKING HANDLER MESSAGES
>>>>>>> graemeg/fixes_2_2
=======
   ¥ DRAG TRACKING HANDLER MESSAGES
>>>>>>> origin/fixes_2_2
=======
   ¥ DRAG ACTIONS
>>>>>>> origin/fixes_2.4
  _________________________________________________________________________________________________________
}

type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	DragActions = OptionBits;
=======
=======
>>>>>>> origin/fixes_2_2
	DragTrackingMessage = SInt16;
const
	kDragTrackingEnterHandler = 1;    { drag has entered handler}
	kDragTrackingEnterWindow = 2;    { drag has entered window}
	kDragTrackingInWindow = 3;    { drag is moving within window}
	kDragTrackingLeaveWindow = 4;    { drag has exited window}
	kDragTrackingLeaveHandler = 5;     { drag has exited handler}

{
  ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ
   ¥ STANDARD DROP LOCATIONS
  ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ
}


{
 *  Summary:
 *    Standard Drop Location constants
 *  
 *  Discussion:
 *    The following constants define common "meta" drop locations.
 }
const
{
   * The drop location was in the trash.  This is set when a drag is
   * dropped on the trash icon.  Setting this standard drop location
   * sets the traditional drop location to an alias to the trash folder
   * automatically.
   }
	kDragStandardDropLocationTrash = FourCharCode('trsh');

  {
   * The receiver did not specify a drop lcoation. This is the default.
   }
	kDragStandardDropLocationUnknown = FourCharCode('unkn');

type
	StandardDropLocation = OSType;
{
  _________________________________________________________________________________________________________
      
   ¥ DRAG ACTIONS
  _________________________________________________________________________________________________________
}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	DragActions = OptionBits;
>>>>>>> origin/fixes_2.4

{
 *  Summary:
 *    Drag Action constants
 *  
 *  Discussion:
 *    The following constants define, in a general way, what actions a
 *    drag should or has performed.  Some drag actions enforce a mode
 *    of operation while others are flexible suggestions.  These
 *    constants are used in conjunction with the
 *    Get/SetDragAllowableActions() and Get/SetDragDropAction() APIs. 
 *    Adopting the Drag Action APIs increases compatability with the
 *    Cocoa drag operation model.
 }
const
{
   * Suggests nothing should be/was done with the data in a drag.  When
   * set as an allowable action for remote drags, the drag will not be
   * sent to apps other than the sender.
   }
	kDragActionNothing = 0;

  {
   * Suggests the data contained within the drag can be/was copied.
   }
	kDragActionCopy = 1;

  {
   * Suggests the data contained within the drag can be/is shared.
   }
	kDragActionAlias = 1 shl 1;

  {
   * Suggests the drag action is can be defined by the drag destination
   * or was not defined by the drag destination.
   }
	kDragActionGeneric = 1 shl 2;

  {
   * Suggests the drag action should be negotiated privately between
   * the drag source and destination.
   }
	kDragActionPrivate = 1 shl 3;

  {
   * Description forthcoming.
   }
	kDragActionMove = 1 shl 4;

  {
   * Description forthcoming.
   }
	kDragActionDelete = 1 shl 5;

  {
   * All of the above drag actions are allowed.
   }
	kDragActionAll = $FFFFFFFF;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{
  _________________________________________________________________________________________________________
      
   ¥ PROVIDING CALLBACK PROCEDURES
  _________________________________________________________________________________________________________
}

type
	DragInputProcPtr = function( var mouse: Point; var modifiers: SInt16; dragInputRefCon: UnivPtr; theDrag: DragRef ): OSErr;
	DragInputUPP = DragInputProcPtr;
{
 *  NewDragInputUPP()
=======
=======
>>>>>>> origin/fixes_2_2
type
	DragActions = UInt32;
{
  _________________________________________________________________________________________________________
      
   ¥ HFS FLAVORS
  _________________________________________________________________________________________________________
}
type
	HFSFlavorPtr = ^HFSFlavor;
	HFSFlavor = record
		fileType: OSType;               { file type }
		fileCreator: OSType;            { file creator }
		fdFlags: UInt16;                { Finder flags }
		fileSpec: FSSpec;               { file system specification }
	end;
type
	PromiseHFSFlavorPtr = ^PromiseHFSFlavor;
	PromiseHFSFlavor = record
		fileType: OSType;               { file type }
		fileCreator: OSType;            { file creator }
		fdFlags: UInt16;                { Finder flags }
		promisedFlavor: FlavorType;         { promised flavor containing an FSSpec }
	end;
=======
>>>>>>> origin/fixes_2.4
{
  _________________________________________________________________________________________________________
      
   ¥ PROVIDING CALLBACK PROCEDURES
  _________________________________________________________________________________________________________
}

type
	DragInputProcPtr = function( var mouse: Point; var modifiers: SInt16; dragInputRefCon: UnivPtr; theDrag: DragRef ): OSErr;
	DragInputUPP = DragInputProcPtr;
{
<<<<<<< HEAD
 *  NewDragTrackingHandlerUPP()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  NewDragInputUPP()
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function NewDragInputUPP( userRoutine: DragInputProcPtr ): DragInputUPP; external name '_NewDragInputUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeDragInputUPP()
=======
=======
>>>>>>> origin/fixes_2_2
function NewDragTrackingHandlerUPP( userRoutine: DragTrackingHandlerProcPtr ): DragTrackingHandlerUPP; external name '_NewDragTrackingHandlerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  NewDragReceiveHandlerUPP()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function NewDragInputUPP( userRoutine: DragInputProcPtr ): DragInputUPP; external name '_NewDragInputUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeDragInputUPP()
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure DisposeDragInputUPP( userUPP: DragInputUPP ); external name '_DisposeDragInputUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeDragInputUPP()
=======
=======
>>>>>>> origin/fixes_2_2
function NewDragReceiveHandlerUPP( userRoutine: DragReceiveHandlerProcPtr ): DragReceiveHandlerUPP; external name '_NewDragReceiveHandlerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeDragTrackingHandlerUPP()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeDragInputUPP( userUPP: DragInputUPP ); external name '_DisposeDragInputUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeDragInputUPP()
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function InvokeDragInputUPP( var mouse: Point; var modifiers: SInt16; dragInputRefCon: UnivPtr; theDrag: DragRef; userUPP: DragInputUPP ): OSErr; external name '_InvokeDragInputUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{$ifc not TARGET_CPU_64}
{
 *  SetDragInputProc()
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function SetDragInputProc( theDrag: DragRef; inputProc: DragInputUPP; dragInputRefCon: UnivPtr ): OSErr; external name '_SetDragInputProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  NewDrag()
 *  
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
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function SetDragInputProc( theDrag: DragRef; inputProc: DragInputUPP; dragInputRefCon: UnivPtr ): OSErr; external name '_SetDragInputProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function NewDrag( var theDrag: DragRef ): OSErr; external name '_NewDrag';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DisposeDrag()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function DisposeDrag( theDrag: DragRef ): OSErr; external name '_DisposeDrag';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew




=======
procedure DisposeDragTrackingHandlerUPP( userUPP: DragTrackingHandlerUPP ); external name '_DisposeDragTrackingHandlerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeDragReceiveHandlerUPP()
=======
procedure DisposeDragTrackingHandlerUPP( userUPP: DragTrackingHandlerUPP ); external name '_DisposeDragTrackingHandlerUPP';
=======
function InvokeDragInputUPP( var mouse: Point; var modifiers: SInt16; dragInputRefCon: UnivPtr; theDrag: DragRef; userUPP: DragInputUPP ): OSErr; external name '_InvokeDragInputUPP';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{$ifc not TARGET_CPU_64}
{
 *  SetDragInputProc()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function SetDragInputProc( theDrag: DragRef; inputProc: DragInputUPP; dragInputRefCon: UnivPtr ): OSErr; external name '_SetDragInputProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  NewDrag()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function NewDrag( var theDrag: DragRef ): OSErr; external name '_NewDrag';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  DisposeDrag()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function DisposeDrag( theDrag: DragRef ): OSErr; external name '_DisposeDrag';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
  _________________________________________________________________________________________________________
      
   ¥ DRAG PASTEBOARD
  _________________________________________________________________________________________________________
}

{
 *  NewDragWithPasteboard()
 *  
 *  Discussion:
 *    Creates a new Drag reference containing the pasteboard reference
 *    provided.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inPasteboard:
 *      A pasteboard created by the drag sender for use with the drag.
 *      Items may be added to the pasteboard via the Pasteboard Manager
 *      API either before or after this routine is called. It is still
 *      possible to add data via the Drag Manager API, but only after
 *      this routine is called. It is the drag sender's responsibility
 *      to clear the pasteboard before adding items. It is also the
 *      drag sender's responsibility to release the pasteboard.  This
 *      may be done at any time after this routine is called. The
 *      pasteboard is retained by the Drag Manager for the duration of
 *      the drag.
 *    
 *    outDrag:
 *      A drag reference which receives the newly created drag.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function NewDragWithPasteboard( inPasteboard: PasteboardRef; var outDrag: DragRef ): OSStatus; external name '_NewDragWithPasteboard';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  GetDragPasteboard()
 *  
 *  Discussion:
 *    Returns the pasteboard reference contained within the provided
 *    drag reference. This routine may be called by a drag sender or
 *    receiver at any point after a valid drag reference has been
 *    created/received.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inDrag:
 *      The drag reference containing the requested pasteboard.
 *    
 *    outPasteboard:
 *      A pasteboard reference which receives the pasteboard contained
 *      by the drag.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function GetDragPasteboard( inDrag: DragRef; var outPasteboard: PasteboardRef ): OSStatus; external name '_GetDragPasteboard';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
  _________________________________________________________________________________________________________
      
   ¥ SETTING THE DRAG IMAGE
  _________________________________________________________________________________________________________
}

{
 *  SetDragImageWithCGImage()
 *  
 *  Discussion:
 *    Used by the sender of the drag to set the image, in CGImage
 *    format, to be displayed as user feedback during the drag.  This
 *    API may be called  at any point during the drag to update the
 *    image.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inDrag:
 *      The drag reference for which the image will be displayed.
 *    
 *    inCGImage:
 *      The CGImageRef for the image to be displayed during the drag. 
 *      The image is retained internally by the Drag Manager for the
 *      duration of the drag so it may be released by the client
 *      immediately after setting. For Mac OS X 10.4 and earlier only
 *      images with an alpha info value between
 *      kCGImageAlphaPremultipliedLast and kCGImageAlphaFirst inclusive
 *      are properly accepted. All alpha info values are accepted in
 *      later versions.
 *    
 *    inImageOffsetPt:
 *      A pointer to the offset from the mouse to the upper left of the
 *      image (normally expressed in negative values).  This differs
 *      from the usage of the offset passed to SetDragImage().  Here,
 *      an offset of ( -30, -30 ) will center a 60x60 pixel image on
 *      the drag mouse. In order to support resolution independence the
 *      kDragDoNotScaleImage flag must be set. In this case, the
 *      inImageOffsetPt parameter will be interpreted in pixels rather
 *      than points.
 *    
 *    inImageFlags:
 *      The flags determining image drawing during the drag.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function SetDragImageWithCGImage( inDrag: DragRef; inCGImage: CGImageRef; const (*var*) inImageOffsetPt: HIPoint; inImageFlags: DragImageFlags ): OSStatus; external name '_SetDragImageWithCGImage';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)


{
  _________________________________________________________________________________________________________
      
   ¥ ALTERING THE BEHAVIOR OF A DRAG
  _________________________________________________________________________________________________________
}

{
<<<<<<< HEAD
 *  InvokeDragSendDataUPP()
>>>>>>> origin/fixes_2_2
=======
 *  ChangeDragBehaviors()
 *  
 *  Mac OS X threading:
 *    Not thread safe
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 9.0 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
procedure DisposeDragReceiveHandlerUPP( userUPP: DragReceiveHandlerUPP ); external name '_DisposeDragReceiveHandlerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeDragTrackingHandlerUPP()
=======
function InvokeDragSendDataUPP( theType: FlavorType; dragSendRefCon: UnivPtr; theItemRef: DragItemRef; theDrag: DragRef; userUPP: DragSendDataUPP ): OSErr; external name '_InvokeDragSendDataUPP';
=======
function ChangeDragBehaviors( theDrag: DragRef; inBehaviorsToSet: DragBehaviors; inBehaviorsToClear: DragBehaviors ): OSErr; external name '_ChangeDragBehaviors';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  InvokeDragInputUPP()
>>>>>>> origin/fixes_2_2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function InvokeDragTrackingHandlerUPP( message: DragTrackingMessage; theWindow: WindowRef; handlerRefCon: UnivPtr; theDrag: DragRef; userUPP: DragTrackingHandlerUPP ): OSErr; external name '_InvokeDragTrackingHandlerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeDragReceiveHandlerUPP()
=======
function InvokeDragInputUPP( var mouse: Point; var modifiers: SInt16; dragInputRefCon: UnivPtr; theDrag: DragRef; userUPP: DragInputUPP ): OSErr; external name '_InvokeDragInputUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeDragDrawingUPP()
>>>>>>> origin/fixes_2_2
=======
  _________________________________________________________________________________________________________
      
   ¥ PERFORMING A DRAG
  _________________________________________________________________________________________________________
}

{
 *  TrackDrag()
 *  
 *  Summary:
 *    Tracks a drag across the displays, returning when the drag has
 *    completed, either via a drop action or a user cancel operation.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    theDrag:
 *      A drag reference describing the data to be dragged.
 *    
 *    theEvent:
 *      An event describing the start location for the drag and
 *      modifier keys.
 *    
 *    theRegion:
 *      A region describing the visible outline of the dragged data. If
 *      no drag image was supplied, or if a drag image was supplied
 *      along with the kDragRegionAndImage flag, this region is moved
 *      across the displays as the user moves the mouse during the drag
 *      operation. On Mac OS X 10.5 and later, you may pass NULL if you
 *      only want the drag image to be shown.
 *  
 *  Result:
 *    An operating system result code.
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
function InvokeDragReceiveHandlerUPP( theWindow: WindowRef; handlerRefCon: UnivPtr; theDrag: DragRef; userUPP: DragReceiveHandlerUPP ): OSErr; external name '_InvokeDragReceiveHandlerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
function InvokeDragDrawingUPP( message: DragRegionMessage; showRegion: RgnHandle; showOrigin: Point; hideRegion: RgnHandle; hideOrigin: Point; dragDrawingRefCon: UnivPtr; theDrag: DragRef; userUPP: DragDrawingUPP ): OSErr; external name '_InvokeDragDrawingUPP';
=======
function TrackDrag( theDrag: DragRef; const (*var*) theEvent: EventRecord; theRegion: RgnHandle ): OSErr; external name '_TrackDrag';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2_2


{
  _________________________________________________________________________________________________________
      
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   ¥ CREATING & DISPOSING
=======
   ¥ DRAG PASTEBOARD
>>>>>>> graemeg/cpstrnew
=======
   ¥ DRAG PASTEBOARD
>>>>>>> graemeg/cpstrnew
=======
   ¥ DRAG PASTEBOARD
>>>>>>> origin/cpstrnew
=======
   ¥ INSTALLING AND REMOVING HANDLERS API'S
>>>>>>> origin/fixes_2_2
=======
   ¥ GETTING INFORMATION ABOUT A DRAG
>>>>>>> origin/fixes_2.4
  _________________________________________________________________________________________________________
}

{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  NewDrag()
=======
 *  InstallTrackingHandler()
>>>>>>> origin/fixes_2_2
=======
 *  GetDragAttributes()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
<<<<<<< HEAD
=======
   ¥ DRAG PASTEBOARD
  _________________________________________________________________________________________________________
}

{
 *  NewDragWithPasteboard()
 *  
 *  Discussion:
 *    Creates a new Drag reference containing the pasteboard reference
 *    provided.
 *  
=======
 *  NewDragWithPasteboard()
=======
 *  NewDragWithPasteboard()
 *  
 *  Discussion:
 *    Creates a new Drag reference containing the pasteboard reference
 *    provided.
=======
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function GetDragAttributes( theDrag: DragRef; var flags: DragAttributes ): OSErr; external name '_GetDragAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDragMouse()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function GetDragMouse( theDrag: DragRef; var mouse: Point; var globalPinnedMouse: Point ): OSErr; external name '_GetDragMouse';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  RemoveTrackingHandler()
>>>>>>> origin/fixes_2_2
=======
 *  SetDragMouse()
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
<<<<<<< HEAD
 *  Parameters:
 *    
 *    inPasteboard:
 *      A pasteboard created by the drag sender for use with the drag.
 *      Items may be added to the pasteboard via the Pasteboard Manager
 *      API either before or after this routine is called. It is still
 *      possible to add data via the Drag Manager API, but only after
 *      this routine is called. It is the drag sender's responsibility
=======
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function SetDragMouse( theDrag: DragRef; globalPinnedMouse: Point ): OSErr; external name '_SetDragMouse';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDragOrigin()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function GetDragOrigin( theDrag: DragRef; var globalInitialMouse: Point ): OSErr; external name '_GetDragOrigin';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDragModifiers()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function GetDragModifiers( theDrag: DragRef; var modifiers: SInt16; var mouseDownModifiers: SInt16; var mouseUpModifiers: SInt16 ): OSErr; external name '_GetDragModifiers';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDragItemBounds()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function GetDragItemBounds( theDrag: DragRef; theItemRef: DragItemRef; var itemBounds: Rect ): OSErr; external name '_GetDragItemBounds';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetDragItemBounds()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function SetDragItemBounds( theDrag: DragRef; theItemRef: DragItemRef; const (*var*) itemBounds: Rect ): OSErr; external name '_SetDragItemBounds';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
  _________________________________________________________________________________________________________
      
   ¥ ACCESSING DRAG ACTIONS
  _________________________________________________________________________________________________________
}

{
 *  GetDragAllowableActions()
 *  
 *  Discussion:
 *    Gets the actions the drag sender has allowed the receiver to
 *    perform. These are not requirements, but they highly suggested
 *    actions which allows the drag receiver to improve harmony across
 *    the system.  The allowable actions received are always those
 *    local to the caller's process.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
<<<<<<< HEAD
 *    inPasteboard:
 *      A pasteboard created by the drag sender for use with the drag.
 *      Items may be added to the pasteboard via the Pasteboard Manager
 *      API either before or after this routine is called. It is still
 *      possible to add data via the Drag Manager API, but only after
 *      this routine is called. It is the drag sender's responsibility
>>>>>>> origin/fixes_2_2
 *      to clear the pasteboard before adding items. It is also the
 *      drag sender's responsibility to release the pasteboard.  This
 *      may be done at any time after this routine is called. The
 *      pasteboard is retained by the Drag Manager for the duration of
 *      the drag.
=======
 *    theDrag:
 *      The drag reference from which to retreive the allowable drag
 *      actions.
>>>>>>> origin/fixes_2.4
 *    
 *    outActions:
 *      A pointer to receive the field of allowable drag actions.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
=======
 *    Mac OS X:         in version 10.1 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
>>>>>>> origin/fixes_2.4
 *    Non-Carbon CFM:   not available
 }
function GetDragAllowableActions( theDrag: DragRef; var outActions: DragActions ): OSStatus; external name '_GetDragAllowableActions';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


{
 *  SetDragAllowableActions()
 *  
 *  Discussion:
 *    Sets the actions the receiver of the drag is allowed to perform. 
 *    These are not requirements, but they highly suggested actions
 *    which allows the drag receiver to improve harmony across the
 *    system.  The caller may select wether these drag actions apply to
 *    a local or remote process.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    theDrag:
 *      The drag reference in which to set the allowable drag actions.
 *    
 *    inActions:
 *      A field of allowable drag actions to be set.
 *    
 *    isLocal:
 *      A boolean value allowing the drag sender to distinguish between
 *      those drag actions allowable by the local receiver versus a
 *      remote one.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
=======
 *    Mac OS X:         in version 10.1 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
>>>>>>> origin/fixes_2.4
 *    Non-Carbon CFM:   not available
 }
function SetDragAllowableActions( theDrag: DragRef; inActions: DragActions; isLocal: Boolean ): OSStatus; external name '_SetDragAllowableActions';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

<<<<<<< HEAD
{
  _________________________________________________________________________________________________________
      
<<<<<<< HEAD
=======
   ¥ ADDING DRAG ITEM FLAVORS
  _________________________________________________________________________________________________________
}
=======
>>>>>>> origin/fixes_2.4

{
 *  GetDragDropAction()
 *  
 *  Discussion:
 *    Gets the action performed by the receiver of the drag.  More than
 *    one action may have been performed.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    theDrag:
 *      The drag reference from which to retreive the performed drop
 *      action.
 *    
 *    outAction:
 *      A pointer to receive the drag action performed.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDragDropAction( theDrag: DragRef; var outAction: DragActions ): OSStatus; external name '_GetDragDropAction';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


{
 *  SetDragDropAction()
 *  
 *  Discussion:
 *    Sets the action performed by the receiver of the drag.  More than
 *    one action may be performed.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    theDrag:
 *      The drag reference in which to set the performed drop action.
 *    
 *    inAction:
 *      The drop action performed.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDragDropAction( theDrag: DragRef; inAction: DragActions ): OSStatus; external name '_SetDragDropAction';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


{
  _________________________________________________________________________________________________________
      
   ¥ UTILITIES
  _________________________________________________________________________________________________________
}

{
 *  WaitMouseMoved()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function WaitMouseMoved( initialGlobalMouse: Point ): Boolean; external name '_WaitMouseMoved';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{--------------------------------------------------------------------------------------}
{  ¥ DEPRECATED                                                                        }
{  All functions below this point are either deprecated (they continue to function     }
{  but are not the most modern nor most efficient solution to a problem), or they are  }
{  completely unavailable on Mac OS X.                                                 }
{--------------------------------------------------------------------------------------}
{
   The file dragging constants and structures have been deprecated in Leopard. Use kUTTypeFileURL,
   kPasteboardTypeFileURLPromise and kPasteboardTypeFilePromiseContent from UTCoreTypes.h and Pasteboard.h instead.
}

{$endc} {not TARGET_CPU_64}

const
	kDragFlavorTypeHFS = FourCharCode('hfs '); { flavor type for HFS data}
	kDragFlavorTypePromiseHFS = FourCharCode('phfs'); { flavor type for promised HFS data}
	flavorTypeHFS = kDragFlavorTypeHFS; { old name}
	flavorTypePromiseHFS = kDragFlavorTypePromiseHFS; { old name}

const
	kDragPromisedFlavorFindFile = FourCharCode('rWm1'); { promisedFlavor value for Find File}
	kDragPromisedFlavor = FourCharCode('fssP'); { promisedFlavor value for everything else}

const
	kDragPseudoCreatorVolumeOrDirectory = FourCharCode('MACS'); { "creator code" for volume or directory}
	kDragPseudoFileTypeVolume = FourCharCode('disk'); { "file type" for volume}
	kDragPseudoFileTypeDirectory = FourCharCode('fold'); { "file type" for directory}

{
   The file dragging constants and structures have been deprecated in Leopard. Use kUTTypeFileURL,
   kPasteboardTypeFileURLPromise and kPasteboardTypeFilePromiseContent from UTCoreTypes.h and Pasteboard.h instead.
}

type
	HFSFlavorPtr = ^HFSFlavor;
	HFSFlavor = record
		fileType: OSType;               { file type }
		fileCreator: OSType;            { file creator }
		fdFlags: UInt16;                { Finder flags }
		fileSpec: FSSpec;               { file system specification }
	end;
type
	PromiseHFSFlavorPtr = ^PromiseHFSFlavor;
	PromiseHFSFlavor = record
		fileType: OSType;               { file type }
		fileCreator: OSType;            { file creator }
		fdFlags: UInt16;                { Finder flags }
		promisedFlavor: FlavorType;         { promised flavor containing an FSSpec }
	end;
type
	DragTrackingMessage = SInt16;
const
	kDragTrackingEnterHandler = 1;    { drag has entered handler}
	kDragTrackingEnterWindow = 2;    { drag has entered window}
	kDragTrackingInWindow = 3;    { drag is moving within window}
	kDragTrackingLeaveWindow = 4;    { drag has exited window}
	kDragTrackingLeaveHandler = 5;     { drag has exited handler}


type
	DragRegionMessage = SInt16;
const
	kDragRegionBegin = 1;    { initialize drawing}
	kDragRegionDraw = 2;    { draw drag feedback}
	kDragRegionHide = 3;    { hide drag feedback}
	kDragRegionIdle = 4;    { drag feedback idle time}
	kDragRegionEnd = 5;     { end of drawing}


type
	ZoomAcceleration = SInt16;
const
	kZoomNoAcceleration = 0;    { use linear interpolation}
	kZoomAccelerate = 1;    { ramp up step size}
	kZoomDecelerate = 2;     { ramp down step size}


{
 *  Summary:
 *    Standard Drop Location constants
 *  
 *  Discussion:
 *    The following constants define common "meta" drop locations.
 }
const
{
   * The drop location was in the trash.  This is set when a drag is
   * dropped on the trash icon.  Setting this standard drop location
   * sets the traditional drop location to an alias to the trash folder
   * automatically.
   }
	kDragStandardDropLocationTrash = FourCharCode('trsh');

  {
   * The receiver did not specify a drop lcoation. This is the default.
   }
	kDragStandardDropLocationUnknown = FourCharCode('unkn');


type
	StandardDropLocation = OSType;
	DragSendDataProcPtr = function( theType: FlavorType; dragSendRefCon: UnivPtr; theItemRef: DragItemRef; theDrag: DragRef ): OSErr;
	DragTrackingHandlerProcPtr = function( message: DragTrackingMessage; theWindow: WindowRef; handlerRefCon: UnivPtr; theDrag: DragRef ): OSErr;
	DragReceiveHandlerProcPtr = function( theWindow: WindowRef; handlerRefCon: UnivPtr; theDrag: DragRef ): OSErr;
	DragDrawingProcPtr = function( message: DragRegionMessage; showRegion: RgnHandle; showOrigin: Point; hideRegion: RgnHandle; hideOrigin: Point; dragDrawingRefCon: UnivPtr; theDrag: DragRef ): OSErr;
	DragSendDataUPP = DragSendDataProcPtr;
	DragTrackingHandlerUPP = DragTrackingHandlerProcPtr;
	DragReceiveHandlerUPP = DragReceiveHandlerProcPtr;
	DragDrawingUPP = DragDrawingProcPtr;
{
 *  NewDragSendDataUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDragSendDataUPP( userRoutine: DragSendDataProcPtr ): DragSendDataUPP; external name '_NewDragSendDataUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  NewDragTrackingHandlerUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDragTrackingHandlerUPP( userRoutine: DragTrackingHandlerProcPtr ): DragTrackingHandlerUPP; external name '_NewDragTrackingHandlerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
<<<<<<< HEAD
  _________________________________________________________________________________________________________
      
>>>>>>> origin/fixes_2_2
   ¥ SETTING THE DRAG IMAGE
  _________________________________________________________________________________________________________
}

{
 *  SetDragImageWithCGImage()
 *  
 *  Discussion:
 *    Used by the sender of the drag to set the image, in CGImage
 *    format, to be displayed as user feedback during the drag.  This
 *    API may be called  at any point during the drag to update the
 *    image.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inDrag:
 *      The drag reference for which the image will be displayed.
 *    
 *    inCGImage:
 *      The CGImageRef for the image to be displayed during the drag. 
 *      The image is retained internally by the Drag Manager for the
 *      duration of the drag so it may be released by the client
<<<<<<< HEAD
 *      immediately after setting. For Mac OS X 10.4 and earlier only
 *      images with an alpha info value between
 *      kCGImageAlphaPremultipliedLast and kCGImageAlphaFirst inclusive
 *      are properly accepted. All alpha info values are accepted in
 *      later versions.
=======
 *      immediately after setting.
>>>>>>> origin/fixes_2_2
 *    
 *    inImageOffsetPt:
 *      A pointer to the offset from the mouse to the upper left of the
 *      image (normally expressed in negative values).  This differs
 *      from the usage of the offset passed to SetDragImage().  Here,
 *      an offset of ( -30, -30 ) will center a 60x60 pixel image on
<<<<<<< HEAD
 *      the drag mouse. In order to support resolution independence the
 *      kDragDoNotScaleImage flag must be set. In this case, the
 *      inImageOffsetPt parameter will be interpreted in pixels rather
 *      than points.
=======
 *      the drag mouse.
>>>>>>> origin/fixes_2_2
 *    
 *    inImageFlags:
 *      The flags determining image drawing during the drag.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
=======
 *  NewDragReceiveHandlerUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDragReceiveHandlerUPP( userRoutine: DragReceiveHandlerProcPtr ): DragReceiveHandlerUPP; external name '_NewDragReceiveHandlerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  NewDragDrawingUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
>>>>>>> origin/fixes_2.4
 }
function NewDragDrawingUPP( userRoutine: DragDrawingProcPtr ): DragDrawingUPP; external name '_NewDragDrawingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
<<<<<<< HEAD
  _________________________________________________________________________________________________________
      
   ¥ ALTERING THE BEHAVIOR OF A DRAG
  _________________________________________________________________________________________________________
}
<<<<<<< HEAD

=======
>>>>>>> origin/fixes_2_2
{
 *  ChangeDragBehaviors()
 *  
 *  Mac OS X threading:
 *    Not thread safe
=======
 *  DisposeDragSendDataUPP()
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDragSendDataUPP( userUPP: DragSendDataUPP ); external name '_DisposeDragSendDataUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
<<<<<<< HEAD
  _________________________________________________________________________________________________________
      
   ¥ PERFORMING A DRAG
  _________________________________________________________________________________________________________
}
<<<<<<< HEAD

{
 *  TrackDrag()
 *  
 *  Summary:
 *    Tracks a drag across the displays, returning when the drag has
 *    completed, either via a drop action or a user cancel operation.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    theDrag:
 *      A drag reference describing the data to be dragged.
 *    
 *    theEvent:
 *      An event describing the start location for the drag and
 *      modifier keys.
 *    
 *    theRegion:
 *      A region describing the visible outline of the dragged data. If
 *      no drag image was supplied, or if a drag image was supplied
 *      along with the kDragRegionAndImage flag, this region is moved
 *      across the displays as the user moves the mouse during the drag
 *      operation. On Mac OS X 10.5 and later, you may pass NULL if you
 *      only want the drag image to be shown.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
=======
{
 *  TrackDrag()
 *  
 *  Mac OS X threading:
 *    Not thread safe
=======
 *  DisposeDragTrackingHandlerUPP()
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
>>>>>>> origin/fixes_2_2
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDragTrackingHandlerUPP( userUPP: DragTrackingHandlerUPP ); external name '_DisposeDragTrackingHandlerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  DisposeDragReceiveHandlerUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDragReceiveHandlerUPP( userUPP: DragReceiveHandlerUPP ); external name '_DisposeDragReceiveHandlerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
<<<<<<< HEAD
  _________________________________________________________________________________________________________
      
<<<<<<< HEAD
   ¥ GETTING INFORMATION ABOUT A DRAG
=======
   ¥ APPLICATION-DEFINED ROUTINES
  _________________________________________________________________________________________________________
}
type
	DragSendDataProcPtr = function( theType: FlavorType; dragSendRefCon: UnivPtr; theItemRef: DragItemRef; theDrag: DragRef ): OSErr;
type
	DragInputProcPtr = function( var mouse: Point; var modifiers: SInt16; dragInputRefCon: UnivPtr; theDrag: DragRef ): OSErr;
type
	DragDrawingProcPtr = function( message: DragRegionMessage; showRegion: RgnHandle; showOrigin: Point; hideRegion: RgnHandle; hideOrigin: Point; dragDrawingRefCon: UnivPtr; theDrag: DragRef ): OSErr;
type
	DragSendDataUPP = DragSendDataProcPtr;
type
	DragInputUPP = DragInputProcPtr;
type
	DragDrawingUPP = DragDrawingProcPtr;
{
 *  NewDragSendDataUPP()
=======
   ¥ GETTING DRAG ITEM INFORMATION
  _________________________________________________________________________________________________________
}

{
 *  CountDragItems()
 *  
 *  Mac OS X threading:
 *    Not thread safe
>>>>>>> origin/fixes_2_2
=======
 *  DisposeDragDrawingUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDragDrawingUPP( userUPP: DragDrawingUPP ); external name '_DisposeDragDrawingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  InvokeDragSendDataUPP()
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
<<<<<<< HEAD
<<<<<<< HEAD
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDragSendDataUPP( userRoutine: DragSendDataProcPtr ): DragSendDataUPP; external name '_NewDragSendDataUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  NewDragInputUPP()
=======
 *    Non-Carbon CFM:   in DragLib 1.1 and later
=======
 *    Non-Carbon CFM:   available as macro/inline
>>>>>>> origin/fixes_2.4
 }
function InvokeDragSendDataUPP( theType: FlavorType; dragSendRefCon: UnivPtr; theItemRef: DragItemRef; theDrag: DragRef; userUPP: DragSendDataUPP ): OSErr; external name '_InvokeDragSendDataUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
<<<<<<< HEAD
 *  GetDragItemReferenceNumber()
 *  
 *  Mac OS X threading:
 *    Not thread safe
>>>>>>> origin/fixes_2_2
=======
 *  InvokeDragTrackingHandlerUPP()
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
<<<<<<< HEAD
<<<<<<< HEAD
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDragInputUPP( userRoutine: DragInputProcPtr ): DragInputUPP; external name '_NewDragInputUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  NewDragDrawingUPP()
=======
 *    Non-Carbon CFM:   in DragLib 1.1 and later
=======
 *    Non-Carbon CFM:   available as macro/inline
>>>>>>> origin/fixes_2.4
 }
function InvokeDragTrackingHandlerUPP( message: DragTrackingMessage; theWindow: WindowRef; handlerRefCon: UnivPtr; theDrag: DragRef; userUPP: DragTrackingHandlerUPP ): OSErr; external name '_InvokeDragTrackingHandlerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
<<<<<<< HEAD
 *  CountDragItemFlavors()
 *  
 *  Mac OS X threading:
 *    Not thread safe
>>>>>>> origin/fixes_2_2
=======
 *  InvokeDragReceiveHandlerUPP()
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
<<<<<<< HEAD
<<<<<<< HEAD
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDragDrawingUPP( userRoutine: DragDrawingProcPtr ): DragDrawingUPP; external name '_NewDragDrawingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeDragSendDataUPP()
=======
 *    Non-Carbon CFM:   in DragLib 1.1 and later
=======
 *    Non-Carbon CFM:   available as macro/inline
>>>>>>> origin/fixes_2.4
 }
function InvokeDragReceiveHandlerUPP( theWindow: WindowRef; handlerRefCon: UnivPtr; theDrag: DragRef; userUPP: DragReceiveHandlerUPP ): OSErr; external name '_InvokeDragReceiveHandlerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
<<<<<<< HEAD
 *  GetFlavorType()
 *  
 *  Mac OS X threading:
 *    Not thread safe
>>>>>>> origin/fixes_2_2
=======
 *  InvokeDragDrawingUPP()
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
<<<<<<< HEAD
<<<<<<< HEAD
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDragSendDataUPP( userUPP: DragSendDataUPP ); external name '_DisposeDragSendDataUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeDragInputUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDragInputUPP( userUPP: DragInputUPP ); external name '_DisposeDragInputUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeDragDrawingUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDragDrawingUPP( userUPP: DragDrawingUPP ); external name '_DisposeDragDrawingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeDragSendDataUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeDragSendDataUPP( theType: FlavorType; dragSendRefCon: UnivPtr; theItemRef: DragItemRef; theDrag: DragRef; userUPP: DragSendDataUPP ): OSErr; external name '_InvokeDragSendDataUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeDragInputUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeDragInputUPP( var mouse: Point; var modifiers: SInt16; dragInputRefCon: UnivPtr; theDrag: DragRef; userUPP: DragInputUPP ): OSErr; external name '_InvokeDragInputUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeDragDrawingUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeDragDrawingUPP( message: DragRegionMessage; showRegion: RgnHandle; showOrigin: Point; hideRegion: RgnHandle; hideOrigin: Point; dragDrawingRefCon: UnivPtr; theDrag: DragRef; userUPP: DragDrawingUPP ): OSErr; external name '_InvokeDragDrawingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
  _________________________________________________________________________________________________________
      
   ¥ INSTALLING AND REMOVING HANDLERS API'S
>>>>>>> graemeg/fixes_2_2
  _________________________________________________________________________________________________________
}

{
<<<<<<< HEAD
 *  GetDragAttributes()
=======
 *  InstallTrackingHandler()
>>>>>>> graemeg/fixes_2_2
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function GetDragAttributes( theDrag: DragRef; var flags: DragAttributes ): OSErr; external name '_GetDragAttributes';
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function InstallTrackingHandler( trackingHandler: DragTrackingHandlerUPP; theWindow: WindowRef; handlerRefCon: UnivPtr ): OSErr; external name '_InstallTrackingHandler';
>>>>>>> graemeg/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  GetDragMouse()
=======
 *  InstallReceiveHandler()
>>>>>>> graemeg/fixes_2_2
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function GetDragMouse( theDrag: DragRef; var mouse: Point; var globalPinnedMouse: Point ): OSErr; external name '_GetDragMouse';
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function InstallReceiveHandler( receiveHandler: DragReceiveHandlerUPP; theWindow: WindowRef; handlerRefCon: UnivPtr ): OSErr; external name '_InstallReceiveHandler';
>>>>>>> graemeg/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  SetDragMouse()
=======
 *  RemoveTrackingHandler()
>>>>>>> graemeg/fixes_2_2
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function SetDragMouse( theDrag: DragRef; globalPinnedMouse: Point ): OSErr; external name '_SetDragMouse';
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function RemoveTrackingHandler( trackingHandler: DragTrackingHandlerUPP; theWindow: WindowRef ): OSErr; external name '_RemoveTrackingHandler';
>>>>>>> graemeg/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  GetDragOrigin()
=======
 *  RemoveReceiveHandler()
>>>>>>> graemeg/fixes_2_2
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function GetDragOrigin( theDrag: DragRef; var globalInitialMouse: Point ): OSErr; external name '_GetDragOrigin';
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function RemoveReceiveHandler( receiveHandler: DragReceiveHandlerUPP; theWindow: WindowRef ): OSErr; external name '_RemoveReceiveHandler';
>>>>>>> graemeg/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  GetDragModifiers()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function GetDragModifiers( theDrag: DragRef; var modifiers: SInt16; var mouseDownModifiers: SInt16; var mouseUpModifiers: SInt16 ): OSErr; external name '_GetDragModifiers';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDragItemBounds()
=======
  _________________________________________________________________________________________________________
      
   ¥ CREATING & DISPOSING
  _________________________________________________________________________________________________________
}

{
 *  NewDrag()
>>>>>>> graemeg/fixes_2_2
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function GetDragItemBounds( theDrag: DragRef; theItemRef: DragItemRef; var itemBounds: Rect ): OSErr; external name '_GetDragItemBounds';
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function NewDrag( var theDrag: DragRef ): OSErr; external name '_NewDrag';
>>>>>>> graemeg/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  SetDragItemBounds()
=======
 *  DisposeDrag()
>>>>>>> graemeg/fixes_2_2
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function SetDragItemBounds( theDrag: DragRef; theItemRef: DragItemRef; const (*var*) itemBounds: Rect ): OSErr; external name '_SetDragItemBounds';
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function DisposeDrag( theDrag: DragRef ): OSErr; external name '_DisposeDrag';
>>>>>>> graemeg/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
  _________________________________________________________________________________________________________
      
<<<<<<< HEAD
   ¥ ACCESSING DRAG ACTIONS
=======
   ¥ DRAG PASTEBOARD
>>>>>>> graemeg/fixes_2_2
  _________________________________________________________________________________________________________
}

{
<<<<<<< HEAD
 *  GetDragAllowableActions()
>>>>>>> origin/cpstrnew
 *  
 *  Discussion:
 *    Gets the actions the drag sender has allowed the receiver to
 *    perform. These are not requirements, but they highly suggested
 *    actions which allows the drag receiver to improve harmony across
 *    the system.  The allowable actions received are always those
 *    local to the caller's process.
=======
 *  NewDragWithPasteboard()
 *  
 *  Discussion:
 *    Creates a new Drag reference containing the pasteboard reference
 *    provided.
>>>>>>> graemeg/fixes_2_2
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
<<<<<<< HEAD
 *    theDrag:
 *      The drag reference from which to retreive the allowable drag
 *      actions.
 *    
 *    outActions:
 *      A pointer to receive the field of allowable drag actions.
=======
 *    inPasteboard:
 *      A pasteboard created by the drag sender for use with the drag.
 *      Items may be added to the pasteboard via the Pasteboard Manager
 *      API either before or after this routine is called. It is still
 *      possible to add data via the Drag Manager API, but only after
 *      this routine is called. It is the drag sender's responsibility
 *      to clear the pasteboard before adding items. It is also the
 *      drag sender's responsibility to release the pasteboard.  This
 *      may be done at any time after this routine is called. The
 *      pasteboard is retained by the Drag Manager for the duration of
 *      the drag.
 *    
 *    outDrag:
 *      A drag reference which receives the newly created drag.
>>>>>>> graemeg/fixes_2_2
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
=======
 *    Mac OS X:         in version 10.1 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
>>>>>>> origin/cpstrnew
 *    Non-Carbon CFM:   not available
 }
function GetDragAllowableActions( theDrag: DragRef; var outActions: DragActions ): OSStatus; external name '_GetDragAllowableActions';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


{
 *  SetDragAllowableActions()
 *  
 *  Discussion:
 *    Sets the actions the receiver of the drag is allowed to perform. 
 *    These are not requirements, but they highly suggested actions
 *    which allows the drag receiver to improve harmony across the
 *    system.  The caller may select wether these drag actions apply to
 *    a local or remote process.
=======
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function NewDragWithPasteboard( inPasteboard: PasteboardRef; var outDrag: DragRef ): OSStatus; external name '_NewDragWithPasteboard';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  GetDragPasteboard()
 *  
 *  Discussion:
 *    Returns the pasteboard reference contained within the provided
 *    drag reference. This routine may be called by a drag sender or
 *    receiver at any point after a valid drag reference has been
 *    created/received.
>>>>>>> graemeg/fixes_2_2
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
<<<<<<< HEAD
 *    theDrag:
 *      The drag reference in which to set the allowable drag actions.
 *    
 *    inActions:
 *      A field of allowable drag actions to be set.
 *    
 *    isLocal:
 *      A boolean value allowing the drag sender to distinguish between
 *      those drag actions allowable by the local receiver versus a
 *      remote one.
=======
 *    inDrag:
 *      The drag reference containing the requested pasteboard.
 *    
 *    outPasteboard:
 *      A pasteboard reference which receives the pasteboard contained
 *      by the drag.
>>>>>>> graemeg/fixes_2_2
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
=======
 *    Mac OS X:         in version 10.1 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
>>>>>>> origin/cpstrnew
 *    Non-Carbon CFM:   not available
 }
function SetDragAllowableActions( theDrag: DragRef; inActions: DragActions; isLocal: Boolean ): OSStatus; external name '_SetDragAllowableActions';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

<<<<<<< HEAD
{
  _________________________________________________________________________________________________________
      
   ¥ SETTING THE DRAG IMAGE
=======
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function GetDragPasteboard( inDrag: DragRef; var outPasteboard: PasteboardRef ): OSStatus; external name '_GetDragPasteboard';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
  _________________________________________________________________________________________________________
      
   ¥ ADDING DRAG ITEM FLAVORS
>>>>>>> graemeg/fixes_2_2
  _________________________________________________________________________________________________________
}

{
<<<<<<< HEAD
 *  SetDragImageWithCGImage()
 *  
 *  Discussion:
 *    Used by the sender of the drag to set the image, in CGImage
 *    format, to be displayed as user feedback during the drag.  This
 *    API may be called  at any point during the drag to update the
 *    image.
=======

{
 *  GetDragDropAction()
 *  
 *  Discussion:
 *    Gets the action performed by the receiver of the drag.  More than
 *    one action may have been performed.
>>>>>>> origin/cpstrnew
 *  
>>>>>>> graemeg/cpstrnew
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
<<<<<<< HEAD
<<<<<<< HEAD
 *    inPasteboard:
 *      A pasteboard created by the drag sender for use with the drag.
 *      Items may be added to the pasteboard via the Pasteboard Manager
 *      API either before or after this routine is called. It is still
 *      possible to add data via the Drag Manager API, but only after
 *      this routine is called. It is the drag sender's responsibility
 *      to clear the pasteboard before adding items. It is also the
 *      drag sender's responsibility to release the pasteboard.  This
 *      may be done at any time after this routine is called. The
 *      pasteboard is retained by the Drag Manager for the duration of
 *      the drag.
 *    
 *    outDrag:
 *      A drag reference which receives the newly created drag.
=======
 *    theDrag:
 *      The drag reference from which to retreive the performed drop
 *      action.
 *    
 *    outAction:
 *      A pointer to receive the drag action performed.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDragDropAction( theDrag: DragRef; var outAction: DragActions ): OSStatus; external name '_GetDragDropAction';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


{
 *  SetDragDropAction()
 *  
 *  Discussion:
 *    Sets the action performed by the receiver of the drag.  More than
 *    one action may be performed.
>>>>>>> origin/cpstrnew
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Parameters:
 *    
 *    theDrag:
 *      The drag reference in which to set the performed drop action.
 *    
 *    inAction:
 *      The drop action performed.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function NewDragWithPasteboard( inPasteboard: PasteboardRef; var outDrag: DragRef ): OSStatus; external name '_NewDragWithPasteboard';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

=======
=======
    The method for setting Drag Manager promises differs from that for Scrap Manger promises.  This chart
    describes the method for setting drag promises via AddDragItemFlavor().
    
        dataPtr         dataSize                                result
     pointer value  actual data size    The data of size dataSize pointed to by dataPtr is added to the drag.
        NULL             ignored        A promise is placed on the drag.
}
{
 *  AddDragItemFlavor()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function AddDragItemFlavor( theDrag: DragRef; theItemRef: DragItemRef; theType: FlavorType; dataPtr: {const} UnivPtr; dataSize: Size; theFlags: FlavorFlags ): OSErr; external name '_AddDragItemFlavor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetDragItemFlavorData()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function SetDragItemFlavorData( theDrag: DragRef; theItemRef: DragItemRef; theType: FlavorType; dataPtr: {const} UnivPtr; dataSize: Size; dataOffset: UInt32 ): OSErr; external name '_SetDragItemFlavorData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
  _________________________________________________________________________________________________________
      
   ¥ PROVIDING CALLBACK PROCEDURES
  _________________________________________________________________________________________________________
}

{
 *  SetDragSendProc()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function SetDragSendProc( theDrag: DragRef; sendProc: DragSendDataUPP; dragSendRefCon: UnivPtr ): OSErr; external name '_SetDragSendProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetDragInputProc()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function SetDragInputProc( theDrag: DragRef; inputProc: DragInputUPP; dragInputRefCon: UnivPtr ): OSErr; external name '_SetDragInputProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetDragDrawingProc()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function SetDragDrawingProc( theDrag: DragRef; drawingProc: DragDrawingUPP; dragDrawingRefCon: UnivPtr ): OSErr; external name '_SetDragDrawingProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
  _________________________________________________________________________________________________________
      
   ¥ SETTING THE DRAG IMAGE
  _________________________________________________________________________________________________________
}

{
 *  SetDragImageWithCGImage()
 *  
 *  Discussion:
 *    Used by the sender of the drag to set the image, in CGImage
 *    format, to be displayed as user feedback during the drag.  This
 *    API may be called  at any point during the drag to update the
 *    image.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
>>>>>>> graemeg/fixes_2_2
 *    inDrag:
 *      The drag reference for which the image will be displayed.
 *    
 *    inCGImage:
 *      The CGImageRef for the image to be displayed during the drag. 
 *      The image is retained internally by the Drag Manager for the
 *      duration of the drag so it may be released by the client
<<<<<<< HEAD
 *      immediately after setting. For Mac OS X 10.4 and earlier only
 *      images with an alpha info value between
 *      kCGImageAlphaPremultipliedLast and kCGImageAlphaFirst inclusive
 *      are properly accepted. All alpha info values are accepted in
 *      later versions.
=======
 *      immediately after setting.
>>>>>>> graemeg/fixes_2_2
 *    
 *    inImageOffsetPt:
 *      A pointer to the offset from the mouse to the upper left of the
 *      image (normally expressed in negative values).  This differs
 *      from the usage of the offset passed to SetDragImage().  Here,
 *      an offset of ( -30, -30 ) will center a 60x60 pixel image on
<<<<<<< HEAD
 *      the drag mouse. In order to support resolution independence the
 *      kDragDoNotScaleImage flag must be set. In this case, the
 *      inImageOffsetPt parameter will be interpreted in pixels rather
 *      than points.
 *    
 *    inImageFlags:
 *      The flags determining image drawing during the drag.
=======
 *    Mac OS X:         in version 10.1 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDragDropAction( theDrag: DragRef; inAction: DragActions ): OSStatus; external name '_SetDragDropAction';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
=======
 *      the drag mouse.
 *    
 *    inImageFlags:
 *      The flags determining image drawing during the drag.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function SetDragImageWithCGImage( inDrag: DragRef; inCGImage: CGImageRef; const (*var*) inImageOffsetPt: HIPoint; inImageFlags: DragImageFlags ): OSStatus; external name '_SetDragImageWithCGImage';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/fixes_2_2


{
  _________________________________________________________________________________________________________
      
<<<<<<< HEAD
   ¥ UTILITIES
  _________________________________________________________________________________________________________
}

{
 *  WaitMouseMoved()
>>>>>>> origin/cpstrnew
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function SetDragImageWithCGImage( inDrag: DragRef; inCGImage: CGImageRef; const (*var*) inImageOffsetPt: HIPoint; inImageFlags: DragImageFlags ): OSStatus; external name '_SetDragImageWithCGImage';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function WaitMouseMoved( initialGlobalMouse: Point ): Boolean; external name '_WaitMouseMoved';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/cpstrnew


{--------------------------------------------------------------------------------------}
{  ¥ DEPRECATED                                                                        }
{  All functions below this point are either deprecated (they continue to function     }
{  but are not the most modern nor most efficient solution to a problem), or they are  }
{  completely unavailable on Mac OS X.                                                 }
{--------------------------------------------------------------------------------------}
{
<<<<<<< HEAD
  _________________________________________________________________________________________________________
      
   ¥ ALTERING THE BEHAVIOR OF A DRAG
  _________________________________________________________________________________________________________
}

{
 *  ChangeDragBehaviors()
=======
   The file dragging constants and structures have been deprecated in Leopard. Use kUTTypeFileURL,
   kPasteboardTypeFileURLPromise and kPasteboardTypeFilePromiseContent from UTCoreTypes.h and Pasteboard.h instead.
}

{$endc} {not TARGET_CPU_64}

const
	kDragFlavorTypeHFS = FourCharCode('hfs '); { flavor type for HFS data}
	kDragFlavorTypePromiseHFS = FourCharCode('phfs'); { flavor type for promised HFS data}
	flavorTypeHFS = kDragFlavorTypeHFS; { old name}
	flavorTypePromiseHFS = kDragFlavorTypePromiseHFS; { old name}

const
	kDragPromisedFlavorFindFile = FourCharCode('rWm1'); { promisedFlavor value for Find File}
	kDragPromisedFlavor = FourCharCode('fssP'); { promisedFlavor value for everything else}

const
	kDragPseudoCreatorVolumeOrDirectory = FourCharCode('MACS'); { "creator code" for volume or directory}
	kDragPseudoFileTypeVolume = FourCharCode('disk'); { "file type" for volume}
	kDragPseudoFileTypeDirectory = FourCharCode('fold'); { "file type" for directory}

{
   The file dragging constants and structures have been deprecated in Leopard. Use kUTTypeFileURL,
   kPasteboardTypeFileURLPromise and kPasteboardTypeFilePromiseContent from UTCoreTypes.h and Pasteboard.h instead.
}

type
	HFSFlavorPtr = ^HFSFlavor;
	HFSFlavor = record
		fileType: OSType;               { file type }
		fileCreator: OSType;            { file creator }
		fdFlags: UInt16;                { Finder flags }
		fileSpec: FSSpec;               { file system specification }
	end;
type
	PromiseHFSFlavorPtr = ^PromiseHFSFlavor;
	PromiseHFSFlavor = record
		fileType: OSType;               { file type }
		fileCreator: OSType;            { file creator }
		fdFlags: UInt16;                { Finder flags }
		promisedFlavor: FlavorType;         { promised flavor containing an FSSpec }
	end;
type
	DragTrackingMessage = SInt16;
const
	kDragTrackingEnterHandler = 1;    { drag has entered handler}
	kDragTrackingEnterWindow = 2;    { drag has entered window}
	kDragTrackingInWindow = 3;    { drag is moving within window}
	kDragTrackingLeaveWindow = 4;    { drag has exited window}
	kDragTrackingLeaveHandler = 5;     { drag has exited handler}


type
	DragRegionMessage = SInt16;
const
	kDragRegionBegin = 1;    { initialize drawing}
	kDragRegionDraw = 2;    { draw drag feedback}
	kDragRegionHide = 3;    { hide drag feedback}
	kDragRegionIdle = 4;    { drag feedback idle time}
	kDragRegionEnd = 5;     { end of drawing}


type
	ZoomAcceleration = SInt16;
const
	kZoomNoAcceleration = 0;    { use linear interpolation}
	kZoomAccelerate = 1;    { ramp up step size}
	kZoomDecelerate = 2;     { ramp down step size}


{
 *  Summary:
 *    Standard Drop Location constants
>>>>>>> origin/cpstrnew
 *  
 *  Discussion:
 *    The following constants define common "meta" drop locations.
 }
const
{
   * The drop location was in the trash.  This is set when a drag is
   * dropped on the trash icon.  Setting this standard drop location
   * sets the traditional drop location to an alias to the trash folder
   * automatically.
   }
	kDragStandardDropLocationTrash = FourCharCode('trsh');

  {
   * The receiver did not specify a drop lcoation. This is the default.
   }
	kDragStandardDropLocationUnknown = FourCharCode('unkn');


type
	StandardDropLocation = OSType;
	DragSendDataProcPtr = function( theType: FlavorType; dragSendRefCon: UnivPtr; theItemRef: DragItemRef; theDrag: DragRef ): OSErr;
	DragTrackingHandlerProcPtr = function( message: DragTrackingMessage; theWindow: WindowRef; handlerRefCon: UnivPtr; theDrag: DragRef ): OSErr;
	DragReceiveHandlerProcPtr = function( theWindow: WindowRef; handlerRefCon: UnivPtr; theDrag: DragRef ): OSErr;
	DragDrawingProcPtr = function( message: DragRegionMessage; showRegion: RgnHandle; showOrigin: Point; hideRegion: RgnHandle; hideOrigin: Point; dragDrawingRefCon: UnivPtr; theDrag: DragRef ): OSErr;
	DragSendDataUPP = DragSendDataProcPtr;
	DragTrackingHandlerUPP = DragTrackingHandlerProcPtr;
	DragReceiveHandlerUPP = DragReceiveHandlerProcPtr;
	DragDrawingUPP = DragDrawingProcPtr;
{
 *  NewDragSendDataUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
<<<<<<< HEAD
 *    Non-Carbon CFM:   in DragLib 9.0 and later
 }
function ChangeDragBehaviors( theDrag: DragRef; inBehaviorsToSet: DragBehaviors; inBehaviorsToClear: DragBehaviors ): OSErr; external name '_ChangeDragBehaviors';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew


{
<<<<<<< HEAD
 *  GetDragPasteboard()
 *  
 *  Discussion:
 *    Returns the pasteboard reference contained within the provided
 *    drag reference. This routine may be called by a drag sender or
 *    receiver at any point after a valid drag reference has been
 *    created/received.
=======
  _________________________________________________________________________________________________________
      
   ¥ PERFORMING A DRAG
  _________________________________________________________________________________________________________
}

{
 *  TrackDrag()
 *  
 *  Summary:
 *    Tracks a drag across the displays, returning when the drag has
 *    completed, either via a drop action or a user cancel operation.
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
=======
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDragSendDataUPP( userRoutine: DragSendDataProcPtr ): DragSendDataUPP; external name '_NewDragSendDataUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  NewDragTrackingHandlerUPP()
>>>>>>> origin/cpstrnew
 *  
 *  Parameters:
 *    
<<<<<<< HEAD
 *    inDrag:
 *      The drag reference containing the requested pasteboard.
 *    
 *    outPasteboard:
 *      A pasteboard reference which receives the pasteboard contained
 *      by the drag.
=======
 *    theDrag:
 *      A drag reference describing the data to be dragged.
 *    
 *    theEvent:
 *      An event describing the start location for the drag and
 *      modifier keys.
 *    
 *    theRegion:
 *      A region describing the visible outline of the dragged data. If
 *      no drag image was supplied, or if a drag image was supplied
 *      along with the kDragRegionAndImage flag, this region is moved
 *      across the displays as the user moves the mouse during the drag
 *      operation. On Mac OS X 10.5 and later, you may pass NULL if you
 *      only want the drag image to be shown.
>>>>>>> graemeg/cpstrnew
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function GetDragPasteboard( inDrag: DragRef; var outPasteboard: PasteboardRef ): OSStatus; external name '_GetDragPasteboard';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function TrackDrag( theDrag: DragRef; const (*var*) theEvent: EventRecord; theRegion: RgnHandle ): OSErr; external name '_TrackDrag';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew


{
  _________________________________________________________________________________________________________
      
<<<<<<< HEAD
   ¥ SETTING THE DRAG IMAGE
=======
   ¥ GETTING INFORMATION ABOUT A DRAG
>>>>>>> graemeg/cpstrnew
  _________________________________________________________________________________________________________
}

{
<<<<<<< HEAD
 *  SetDragImageWithCGImage()
 *  
 *  Discussion:
 *    Used by the sender of the drag to set the image, in CGImage
 *    format, to be displayed as user feedback during the drag.  This
 *    API may be called  at any point during the drag to update the
 *    image.
=======
 *  GetDragAttributes()
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
<<<<<<< HEAD
 *  Parameters:
 *    
 *    inDrag:
 *      The drag reference for which the image will be displayed.
 *    
 *    inCGImage:
 *      The CGImageRef for the image to be displayed during the drag. 
 *      The image is retained internally by the Drag Manager for the
 *      duration of the drag so it may be released by the client
 *      immediately after setting. For Mac OS X 10.4 and earlier only
 *      images with an alpha info value between
 *      kCGImageAlphaPremultipliedLast and kCGImageAlphaFirst inclusive
 *      are properly accepted. All alpha info values are accepted in
 *      later versions.
 *    
 *    inImageOffsetPt:
 *      A pointer to the offset from the mouse to the upper left of the
 *      image (normally expressed in negative values).  This differs
 *      from the usage of the offset passed to SetDragImage().  Here,
 *      an offset of ( -30, -30 ) will center a 60x60 pixel image on
 *      the drag mouse. In order to support resolution independence the
 *      kDragDoNotScaleImage flag must be set. In this case, the
 *      inImageOffsetPt parameter will be interpreted in pixels rather
 *      than points.
 *    
 *    inImageFlags:
 *      The flags determining image drawing during the drag.
=======
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function GetDragAttributes( theDrag: DragRef; var flags: DragAttributes ): OSErr; external name '_GetDragAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDragMouse()
>>>>>>> graemeg/cpstrnew
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function SetDragImageWithCGImage( inDrag: DragRef; inCGImage: CGImageRef; const (*var*) inImageOffsetPt: HIPoint; inImageFlags: DragImageFlags ): OSStatus; external name '_SetDragImageWithCGImage';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)

=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function GetDragMouse( theDrag: DragRef; var mouse: Point; var globalPinnedMouse: Point ): OSErr; external name '_GetDragMouse';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew

{
  _________________________________________________________________________________________________________
      
   ¥ ALTERING THE BEHAVIOR OF A DRAG
  _________________________________________________________________________________________________________
}

{
<<<<<<< HEAD
 *  ChangeDragBehaviors()
=======
 *  SetDragMouse()
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
=======
function NewDragTrackingHandlerUPP( userRoutine: DragTrackingHandlerProcPtr ): DragTrackingHandlerUPP; external name '_NewDragTrackingHandlerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  NewDragReceiveHandlerUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDragReceiveHandlerUPP( userRoutine: DragReceiveHandlerProcPtr ): DragReceiveHandlerUPP; external name '_NewDragReceiveHandlerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  NewDragDrawingUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDragDrawingUPP( userRoutine: DragDrawingProcPtr ): DragDrawingUPP; external name '_NewDragDrawingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  DisposeDragSendDataUPP()
>>>>>>> origin/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
function ChangeDragBehaviors( theDrag: DragRef; inBehaviorsToSet: DragBehaviors; inBehaviorsToClear: DragBehaviors ): OSErr; external name '_ChangeDragBehaviors';
=======
function SetDragMouse( theDrag: DragRef; globalPinnedMouse: Point ): OSErr; external name '_SetDragMouse';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
  _________________________________________________________________________________________________________
      
   ¥ PERFORMING A DRAG
  _________________________________________________________________________________________________________
}

{
 *  TrackDrag()
 *  
 *  Summary:
 *    Tracks a drag across the displays, returning when the drag has
 *    completed, either via a drop action or a user cancel operation.
=======
 *  GetDragOrigin()
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
=======
procedure DisposeDragSendDataUPP( userUPP: DragSendDataUPP ); external name '_DisposeDragSendDataUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  DisposeDragTrackingHandlerUPP()
>>>>>>> origin/cpstrnew
 *  
 *  Parameters:
 *    
 *    theDrag:
 *      A drag reference describing the data to be dragged.
 *    
 *    theEvent:
 *      An event describing the start location for the drag and
 *      modifier keys.
 *    
 *    theRegion:
 *      A region describing the visible outline of the dragged data. If
 *      no drag image was supplied, or if a drag image was supplied
 *      along with the kDragRegionAndImage flag, this region is moved
 *      across the displays as the user moves the mouse during the drag
 *      operation. On Mac OS X 10.5 and later, you may pass NULL if you
 *      only want the drag image to be shown.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
function TrackDrag( theDrag: DragRef; const (*var*) theEvent: EventRecord; theRegion: RgnHandle ): OSErr; external name '_TrackDrag';
=======
function GetDragOrigin( theDrag: DragRef; var globalInitialMouse: Point ): OSErr; external name '_GetDragOrigin';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
  _________________________________________________________________________________________________________
      
   ¥ GETTING INFORMATION ABOUT A DRAG
  _________________________________________________________________________________________________________
}

{
 *  GetDragAttributes()
=======
 *  GetDragModifiers()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function GetDragModifiers( theDrag: DragRef; var modifiers: SInt16; var mouseDownModifiers: SInt16; var mouseUpModifiers: SInt16 ): OSErr; external name '_GetDragModifiers';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDragItemBounds()
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
>>>>>>> graemeg/cpstrnew
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
function NewDrag( var theDrag: DragRef ): OSErr; external name '_NewDrag';
=======
function GetDragItemBounds( theDrag: DragRef; theItemRef: DragItemRef; var itemBounds: Rect ): OSErr; external name '_GetDragItemBounds';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
=======
 *  NewDragWithPasteboard()
 *  
 *  Discussion:
 *    Creates a new Drag reference containing the pasteboard reference
 *    provided.
=======
function GetDragAttributes( theDrag: DragRef; var flags: DragAttributes ): OSErr; external name '_GetDragAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  GetDragMouse()
=======
 *  SetDragItemBounds()
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
<<<<<<< HEAD
function GetDragMouse( theDrag: DragRef; var mouse: Point; var globalPinnedMouse: Point ): OSErr; external name '_GetDragMouse';
=======
function SetDragItemBounds( theDrag: DragRef; theItemRef: DragItemRef; const (*var*) itemBounds: Rect ): OSErr; external name '_SetDragItemBounds';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
=======
procedure DisposeDragTrackingHandlerUPP( userUPP: DragTrackingHandlerUPP ); external name '_DisposeDragTrackingHandlerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/cpstrnew

{
 *  DisposeDragReceiveHandlerUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDragReceiveHandlerUPP( userUPP: DragReceiveHandlerUPP ); external name '_DisposeDragReceiveHandlerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
<<<<<<< HEAD
<<<<<<< HEAD
 *  SetDragMouse()
>>>>>>> graemeg/cpstrnew
=======
  _________________________________________________________________________________________________________
      
   ¥ ACCESSING DRAG ACTIONS
  _________________________________________________________________________________________________________
}

{
 *  GetDragAllowableActions()
 *  
 *  Discussion:
 *    Gets the actions the drag sender has allowed the receiver to
 *    perform. These are not requirements, but they highly suggested
 *    actions which allows the drag receiver to improve harmony across
 *    the system.  The allowable actions received are always those
 *    local to the caller's process.
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
<<<<<<< HEAD
 *  Parameters:
 *    
 *    theDrag:
 *      The drag reference from which to retreive the allowable drag
 *      actions.
 *    
<<<<<<< HEAD
 *    outDrag:
 *      A drag reference which receives the newly created drag.
=======
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function SetDragMouse( theDrag: DragRef; globalPinnedMouse: Point ): OSErr; external name '_SetDragMouse';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDragOrigin()
>>>>>>> graemeg/cpstrnew
=======
 *    outActions:
 *      A pointer to receive the field of allowable drag actions.
>>>>>>> graemeg/cpstrnew
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
=======
 *    Mac OS X:         in version 10.1 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
>>>>>>> graemeg/cpstrnew
 *    Non-Carbon CFM:   not available
 }
function GetDragAllowableActions( theDrag: DragRef; var outActions: DragActions ): OSStatus; external name '_GetDragAllowableActions';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)

>>>>>>> graemeg/cpstrnew


{
<<<<<<< HEAD
<<<<<<< HEAD
 *  DisposeDrag()
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function GetDragOrigin( theDrag: DragRef; var globalInitialMouse: Point ): OSErr; external name '_GetDragOrigin';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDragModifiers()
=======
 *  SetDragAllowableActions()
 *  
 *  Discussion:
 *    Sets the actions the receiver of the drag is allowed to perform. 
 *    These are not requirements, but they highly suggested actions
 *    which allows the drag receiver to improve harmony across the
 *    system.  The caller may select wether these drag actions apply to
 *    a local or remote process.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    theDrag:
 *      The drag reference in which to set the allowable drag actions.
 *    
 *    inActions:
 *      A field of allowable drag actions to be set.
 *    
 *    isLocal:
 *      A boolean value allowing the drag sender to distinguish between
 *      those drag actions allowable by the local receiver versus a
 *      remote one.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDragAllowableActions( theDrag: DragRef; inActions: DragActions; isLocal: Boolean ): OSStatus; external name '_SetDragAllowableActions';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


{
 *  GetDragDropAction()
 *  
 *  Discussion:
 *    Gets the action performed by the receiver of the drag.  More than
 *    one action may have been performed.
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    theDrag:
 *      The drag reference from which to retreive the performed drop
 *      action.
 *    
 *    outAction:
 *      A pointer to receive the drag action performed.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function GetDragModifiers( theDrag: DragRef; var modifiers: SInt16; var mouseDownModifiers: SInt16; var mouseUpModifiers: SInt16 ): OSErr; external name '_GetDragModifiers';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDragItemBounds()
=======
 *    Mac OS X:         in version 10.1 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDragDropAction( theDrag: DragRef; var outAction: DragActions ): OSStatus; external name '_GetDragDropAction';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


{
 *  SetDragDropAction()
 *  
 *  Discussion:
 *    Sets the action performed by the receiver of the drag.  More than
 *    one action may be performed.
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    theDrag:
 *      The drag reference in which to set the performed drop action.
 *    
 *    inAction:
 *      The drop action performed.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function GetDragItemBounds( theDrag: DragRef; theItemRef: DragItemRef; var itemBounds: Rect ): OSErr; external name '_GetDragItemBounds';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetDragItemBounds()
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.1 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDragDropAction( theDrag: DragRef; inAction: DragActions ): OSStatus; external name '_SetDragDropAction';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


{
  _________________________________________________________________________________________________________
      
   ¥ UTILITIES
  _________________________________________________________________________________________________________
}

{
 *  WaitMouseMoved()
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
<<<<<<< HEAD
function SetDragItemBounds( theDrag: DragRef; theItemRef: DragItemRef; const (*var*) itemBounds: Rect ): OSErr; external name '_SetDragItemBounds';
=======
function WaitMouseMoved( initialGlobalMouse: Point ): Boolean; external name '_WaitMouseMoved';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{--------------------------------------------------------------------------------------}
{  ¥ DEPRECATED                                                                        }
{  All functions below this point are either deprecated (they continue to function     }
{  but are not the most modern nor most efficient solution to a problem), or they are  }
{  completely unavailable on Mac OS X.                                                 }
{--------------------------------------------------------------------------------------}
{
<<<<<<< HEAD
  _________________________________________________________________________________________________________
      
   ¥ ACCESSING DRAG ACTIONS
  _________________________________________________________________________________________________________
}

{
 *  GetDragAllowableActions()
 *  
 *  Discussion:
 *    Gets the actions the drag sender has allowed the receiver to
 *    perform. These are not requirements, but they highly suggested
 *    actions which allows the drag receiver to improve harmony across
 *    the system.  The allowable actions received are always those
 *    local to the caller's process.
=======
   The file dragging constants and structures have been deprecated in Leopard. Use kUTTypeFileURL,
   kPasteboardTypeFileURLPromise and kPasteboardTypeFilePromiseContent from UTCoreTypes.h and Pasteboard.h instead.
}

{$endc} {not TARGET_CPU_64}

const
	kDragFlavorTypeHFS = FourCharCode('hfs '); { flavor type for HFS data}
	kDragFlavorTypePromiseHFS = FourCharCode('phfs'); { flavor type for promised HFS data}
	flavorTypeHFS = kDragFlavorTypeHFS; { old name}
	flavorTypePromiseHFS = kDragFlavorTypePromiseHFS; { old name}

const
	kDragPromisedFlavorFindFile = FourCharCode('rWm1'); { promisedFlavor value for Find File}
	kDragPromisedFlavor = FourCharCode('fssP'); { promisedFlavor value for everything else}

const
	kDragPseudoCreatorVolumeOrDirectory = FourCharCode('MACS'); { "creator code" for volume or directory}
	kDragPseudoFileTypeVolume = FourCharCode('disk'); { "file type" for volume}
	kDragPseudoFileTypeDirectory = FourCharCode('fold'); { "file type" for directory}

{
   The file dragging constants and structures have been deprecated in Leopard. Use kUTTypeFileURL,
   kPasteboardTypeFileURLPromise and kPasteboardTypeFilePromiseContent from UTCoreTypes.h and Pasteboard.h instead.
}

type
	HFSFlavorPtr = ^HFSFlavor;
	HFSFlavor = record
		fileType: OSType;               { file type }
		fileCreator: OSType;            { file creator }
		fdFlags: UInt16;                { Finder flags }
		fileSpec: FSSpec;               { file system specification }
	end;
type
	PromiseHFSFlavorPtr = ^PromiseHFSFlavor;
	PromiseHFSFlavor = record
		fileType: OSType;               { file type }
		fileCreator: OSType;            { file creator }
		fdFlags: UInt16;                { Finder flags }
		promisedFlavor: FlavorType;         { promised flavor containing an FSSpec }
	end;
type
	DragTrackingMessage = SInt16;
const
	kDragTrackingEnterHandler = 1;    { drag has entered handler}
	kDragTrackingEnterWindow = 2;    { drag has entered window}
	kDragTrackingInWindow = 3;    { drag is moving within window}
	kDragTrackingLeaveWindow = 4;    { drag has exited window}
	kDragTrackingLeaveHandler = 5;     { drag has exited handler}


type
	DragRegionMessage = SInt16;
const
	kDragRegionBegin = 1;    { initialize drawing}
	kDragRegionDraw = 2;    { draw drag feedback}
	kDragRegionHide = 3;    { hide drag feedback}
	kDragRegionIdle = 4;    { drag feedback idle time}
	kDragRegionEnd = 5;     { end of drawing}


type
	ZoomAcceleration = SInt16;
const
	kZoomNoAcceleration = 0;    { use linear interpolation}
	kZoomAccelerate = 1;    { ramp up step size}
	kZoomDecelerate = 2;     { ramp down step size}


{
 *  Summary:
 *    Standard Drop Location constants
>>>>>>> graemeg/cpstrnew
 *  
 *  Discussion:
 *    The following constants define common "meta" drop locations.
 }
const
{
   * The drop location was in the trash.  This is set when a drag is
   * dropped on the trash icon.  Setting this standard drop location
   * sets the traditional drop location to an alias to the trash folder
   * automatically.
   }
	kDragStandardDropLocationTrash = FourCharCode('trsh');

  {
   * The receiver did not specify a drop lcoation. This is the default.
   }
	kDragStandardDropLocationUnknown = FourCharCode('unkn');


type
	StandardDropLocation = OSType;
	DragSendDataProcPtr = function( theType: FlavorType; dragSendRefCon: UnivPtr; theItemRef: DragItemRef; theDrag: DragRef ): OSErr;
	DragTrackingHandlerProcPtr = function( message: DragTrackingMessage; theWindow: WindowRef; handlerRefCon: UnivPtr; theDrag: DragRef ): OSErr;
	DragReceiveHandlerProcPtr = function( theWindow: WindowRef; handlerRefCon: UnivPtr; theDrag: DragRef ): OSErr;
	DragDrawingProcPtr = function( message: DragRegionMessage; showRegion: RgnHandle; showOrigin: Point; hideRegion: RgnHandle; hideOrigin: Point; dragDrawingRefCon: UnivPtr; theDrag: DragRef ): OSErr;
	DragSendDataUPP = DragSendDataProcPtr;
	DragTrackingHandlerUPP = DragTrackingHandlerProcPtr;
	DragReceiveHandlerUPP = DragReceiveHandlerProcPtr;
	DragDrawingUPP = DragDrawingProcPtr;
{
 *  NewDragSendDataUPP()
 *  
 *  Parameters:
 *    
 *    theDrag:
 *      The drag reference from which to retreive the allowable drag
 *      actions.
 *    
 *    outActions:
 *      A pointer to receive the field of allowable drag actions.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
=======
 *    Mac OS X:         in version 10.1 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
>>>>>>> graemeg/cpstrnew
 *    Non-Carbon CFM:   not available
 }
function GetDragAllowableActions( theDrag: DragRef; var outActions: DragActions ): OSStatus; external name '_GetDragAllowableActions';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


{
 *  SetDragAllowableActions()
 *  
 *  Discussion:
<<<<<<< HEAD
 *    Returns the pasteboard reference contained within the provided
 *    drag reference. This routine may be called by a drag sender or
 *    receiver at any point after a valid drag reference has been
 *    created/received.
=======
 *  GetDragPasteboard()
 *  
 *  Discussion:
 *    Returns the pasteboard reference contained within the provided
 *    drag reference. This routine may be called by a drag sender or
 *    receiver at any point after a valid drag reference has been
 *    created/received.
=======
 *    Sets the actions the receiver of the drag is allowed to perform. 
 *    These are not requirements, but they highly suggested actions
 *    which allows the drag receiver to improve harmony across the
 *    system.  The caller may select wether these drag actions apply to
 *    a local or remote process.
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDragSendDataUPP( userRoutine: DragSendDataProcPtr ): DragSendDataUPP; external name '_NewDragSendDataUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  NewDragTrackingHandlerUPP()
>>>>>>> graemeg/cpstrnew
 *  
 *  Parameters:
 *    
 *    theDrag:
 *      The drag reference in which to set the allowable drag actions.
 *    
 *    inActions:
 *      A field of allowable drag actions to be set.
 *    
 *    isLocal:
 *      A boolean value allowing the drag sender to distinguish between
 *      those drag actions allowable by the local receiver versus a
 *      remote one.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
=======
 *    Mac OS X:         in version 10.1 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
>>>>>>> graemeg/cpstrnew
 *    Non-Carbon CFM:   not available
 }
function SetDragAllowableActions( theDrag: DragRef; inActions: DragActions; isLocal: Boolean ): OSStatus; external name '_SetDragAllowableActions';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDragTrackingHandlerUPP( userRoutine: DragTrackingHandlerProcPtr ): DragTrackingHandlerUPP; external name '_NewDragTrackingHandlerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew

<<<<<<< HEAD
{
 *  NewDragReceiveHandlerUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDragReceiveHandlerUPP( userRoutine: DragReceiveHandlerProcPtr ): DragReceiveHandlerUPP; external name '_NewDragReceiveHandlerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
<<<<<<< HEAD
 *  SetDragImageWithCGImage()
 *  
 *  Discussion:
 *    Used by the sender of the drag to set the image, in CGImage
 *    format, to be displayed as user feedback during the drag.  This
 *    API may be called  at any point during the drag to update the
 *    image.
>>>>>>> graemeg/cpstrnew
=======

{
 *  GetDragDropAction()
 *  
 *  Discussion:
 *    Gets the action performed by the receiver of the drag.  More than
 *    one action may have been performed.
>>>>>>> graemeg/cpstrnew
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
<<<<<<< HEAD
 *    inDrag:
<<<<<<< HEAD
 *      The drag reference containing the requested pasteboard.
 *    
 *    outPasteboard:
 *      A pasteboard reference which receives the pasteboard contained
 *      by the drag.
=======
 *      The drag reference for which the image will be displayed.
 *    
 *    inCGImage:
 *      The CGImageRef for the image to be displayed during the drag. 
 *      The image is retained internally by the Drag Manager for the
 *      duration of the drag so it may be released by the client
 *      immediately after setting. For Mac OS X 10.4 and earlier only
 *      images with an alpha info value between
 *      kCGImageAlphaPremultipliedLast and kCGImageAlphaFirst inclusive
 *      are properly accepted. All alpha info values are accepted in
 *      later versions.
 *    
 *    inImageOffsetPt:
 *      A pointer to the offset from the mouse to the upper left of the
 *      image (normally expressed in negative values).  This differs
 *      from the usage of the offset passed to SetDragImage().  Here,
 *      an offset of ( -30, -30 ) will center a 60x60 pixel image on
 *      the drag mouse. In order to support resolution independence the
 *      kDragDoNotScaleImage flag must be set. In this case, the
 *      inImageOffsetPt parameter will be interpreted in pixels rather
 *      than points.
 *    
 *    inImageFlags:
 *      The flags determining image drawing during the drag.
>>>>>>> graemeg/cpstrnew
=======
 *    theDrag:
 *      The drag reference from which to retreive the performed drop
 *      action.
 *    
 *    outAction:
 *      A pointer to receive the drag action performed.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDragDropAction( theDrag: DragRef; var outAction: DragActions ): OSStatus; external name '_GetDragDropAction';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


{
 *  SetDragDropAction()
 *  
 *  Discussion:
 *    Sets the action performed by the receiver of the drag.  More than
 *    one action may be performed.
>>>>>>> graemeg/cpstrnew
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Parameters:
 *    
 *    theDrag:
 *      The drag reference in which to set the performed drop action.
 *    
 *    inAction:
 *      The drop action performed.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function GetDragPasteboard( inDrag: DragRef; var outPasteboard: PasteboardRef ): OSStatus; external name '_GetDragPasteboard';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function SetDragImageWithCGImage( inDrag: DragRef; inCGImage: CGImageRef; const (*var*) inImageOffsetPt: HIPoint; inImageFlags: DragImageFlags ): OSStatus; external name '_SetDragImageWithCGImage';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
 *    Mac OS X:         in version 10.1 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDragDropAction( theDrag: DragRef; inAction: DragActions ): OSStatus; external name '_SetDragDropAction';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)
>>>>>>> graemeg/cpstrnew


{
  _________________________________________________________________________________________________________
      
<<<<<<< HEAD
<<<<<<< HEAD
   ¥ SETTING THE DRAG IMAGE
=======
   ¥ ALTERING THE BEHAVIOR OF A DRAG
>>>>>>> graemeg/cpstrnew
=======
   ¥ UTILITIES
>>>>>>> graemeg/cpstrnew
  _________________________________________________________________________________________________________
}

{
<<<<<<< HEAD
<<<<<<< HEAD
 *  SetDragImageWithCGImage()
 *  
 *  Discussion:
 *    Used by the sender of the drag to set the image, in CGImage
 *    format, to be displayed as user feedback during the drag.  This
 *    API may be called  at any point during the drag to update the
 *    image.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inDrag:
 *      The drag reference for which the image will be displayed.
 *    
 *    inCGImage:
 *      The CGImageRef for the image to be displayed during the drag. 
 *      The image is retained internally by the Drag Manager for the
 *      duration of the drag so it may be released by the client
 *      immediately after setting. For Mac OS X 10.4 and earlier only
 *      images with an alpha info value between
 *      kCGImageAlphaPremultipliedLast and kCGImageAlphaFirst inclusive
 *      are properly accepted. All alpha info values are accepted in
 *      later versions.
 *    
 *    inImageOffsetPt:
 *      A pointer to the offset from the mouse to the upper left of the
 *      image (normally expressed in negative values).  This differs
 *      from the usage of the offset passed to SetDragImage().  Here,
 *      an offset of ( -30, -30 ) will center a 60x60 pixel image on
 *      the drag mouse. In order to support resolution independence the
 *      kDragDoNotScaleImage flag must be set. In this case, the
 *      inImageOffsetPt parameter will be interpreted in pixels rather
 *      than points.
 *    
 *    inImageFlags:
 *      The flags determining image drawing during the drag.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x
 *    Non-Carbon CFM:   not available
 }
function SetDragImageWithCGImage( inDrag: DragRef; inCGImage: CGImageRef; const (*var*) inImageOffsetPt: HIPoint; inImageFlags: DragImageFlags ): OSStatus; external name '_SetDragImageWithCGImage';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)

=======
 *  ChangeDragBehaviors()
 *  
 *  Mac OS X threading:
 *    Not thread safe
=======
 *  NewDragDrawingUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDragDrawingUPP( userRoutine: DragDrawingProcPtr ): DragDrawingUPP; external name '_NewDragDrawingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  DisposeDragSendDataUPP()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function ChangeDragBehaviors( theDrag: DragRef; inBehaviorsToSet: DragBehaviors; inBehaviorsToClear: DragBehaviors ): OSErr; external name '_ChangeDragBehaviors';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> graemeg/cpstrnew


{
  _________________________________________________________________________________________________________
      
<<<<<<< HEAD
   ¥ ALTERING THE BEHAVIOR OF A DRAG
  _________________________________________________________________________________________________________
}

{
 *  ChangeDragBehaviors()
 *  
 *  Mac OS X threading:
 *    Not thread safe
=======
procedure DisposeDragSendDataUPP( userUPP: DragSendDataUPP ); external name '_DisposeDragSendDataUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  DisposeDragTrackingHandlerUPP()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
<<<<<<< HEAD
 *    Non-Carbon CFM:   in DragLib 9.0 and later
 }
function ChangeDragBehaviors( theDrag: DragRef; inBehaviorsToSet: DragBehaviors; inBehaviorsToClear: DragBehaviors ): OSErr; external name '_ChangeDragBehaviors';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
=======
   ¥ PERFORMING A DRAG
  _________________________________________________________________________________________________________
}
>>>>>>> graemeg/cpstrnew
=======
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDragTrackingHandlerUPP( userUPP: DragTrackingHandlerUPP ); external name '_DisposeDragTrackingHandlerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew

{
 *  DisposeDragReceiveHandlerUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDragReceiveHandlerUPP( userUPP: DragReceiveHandlerUPP ); external name '_DisposeDragReceiveHandlerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
<<<<<<< HEAD
<<<<<<< HEAD
  _________________________________________________________________________________________________________
      
   ¥ PERFORMING A DRAG
  _________________________________________________________________________________________________________
}

{
=======
>>>>>>> graemeg/cpstrnew
 *  TrackDrag()
 *  
 *  Summary:
 *    Tracks a drag across the displays, returning when the drag has
 *    completed, either via a drop action or a user cancel operation.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    theDrag:
 *      A drag reference describing the data to be dragged.
 *    
 *    theEvent:
 *      An event describing the start location for the drag and
 *      modifier keys.
 *    
 *    theRegion:
 *      A region describing the visible outline of the dragged data. If
 *      no drag image was supplied, or if a drag image was supplied
 *      along with the kDragRegionAndImage flag, this region is moved
 *      across the displays as the user moves the mouse during the drag
 *      operation. On Mac OS X 10.5 and later, you may pass NULL if you
 *      only want the drag image to be shown.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
=======
 *  WaitMouseMoved()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
>>>>>>> graemeg/cpstrnew
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
<<<<<<< HEAD
function TrackDrag( theDrag: DragRef; const (*var*) theEvent: EventRecord; theRegion: RgnHandle ): OSErr; external name '_TrackDrag';
=======
function WaitMouseMoved( initialGlobalMouse: Point ): Boolean; external name '_WaitMouseMoved';
>>>>>>> graemeg/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{--------------------------------------------------------------------------------------}
{  ¥ DEPRECATED                                                                        }
{  All functions below this point are either deprecated (they continue to function     }
{  but are not the most modern nor most efficient solution to a problem), or they are  }
{  completely unavailable on Mac OS X.                                                 }
{--------------------------------------------------------------------------------------}
{
<<<<<<< HEAD
  _________________________________________________________________________________________________________
      
   ¥ GETTING INFORMATION ABOUT A DRAG
  _________________________________________________________________________________________________________
}

{
 *  GetDragAttributes()
=======
   The file dragging constants and structures have been deprecated in Leopard. Use kUTTypeFileURL,
   kPasteboardTypeFileURLPromise and kPasteboardTypeFilePromiseContent from UTCoreTypes.h and Pasteboard.h instead.
}

{$endc} {not TARGET_CPU_64}

const
	kDragFlavorTypeHFS = FourCharCode('hfs '); { flavor type for HFS data}
	kDragFlavorTypePromiseHFS = FourCharCode('phfs'); { flavor type for promised HFS data}
	flavorTypeHFS = kDragFlavorTypeHFS; { old name}
	flavorTypePromiseHFS = kDragFlavorTypePromiseHFS; { old name}

const
	kDragPromisedFlavorFindFile = FourCharCode('rWm1'); { promisedFlavor value for Find File}
	kDragPromisedFlavor = FourCharCode('fssP'); { promisedFlavor value for everything else}

const
	kDragPseudoCreatorVolumeOrDirectory = FourCharCode('MACS'); { "creator code" for volume or directory}
	kDragPseudoFileTypeVolume = FourCharCode('disk'); { "file type" for volume}
	kDragPseudoFileTypeDirectory = FourCharCode('fold'); { "file type" for directory}

{
   The file dragging constants and structures have been deprecated in Leopard. Use kUTTypeFileURL,
   kPasteboardTypeFileURLPromise and kPasteboardTypeFilePromiseContent from UTCoreTypes.h and Pasteboard.h instead.
}

type
	HFSFlavorPtr = ^HFSFlavor;
	HFSFlavor = record
		fileType: OSType;               { file type }
		fileCreator: OSType;            { file creator }
		fdFlags: UInt16;                { Finder flags }
		fileSpec: FSSpec;               { file system specification }
	end;
type
	PromiseHFSFlavorPtr = ^PromiseHFSFlavor;
	PromiseHFSFlavor = record
		fileType: OSType;               { file type }
		fileCreator: OSType;            { file creator }
		fdFlags: UInt16;                { Finder flags }
		promisedFlavor: FlavorType;         { promised flavor containing an FSSpec }
	end;
type
	DragTrackingMessage = SInt16;
const
	kDragTrackingEnterHandler = 1;    { drag has entered handler}
	kDragTrackingEnterWindow = 2;    { drag has entered window}
	kDragTrackingInWindow = 3;    { drag is moving within window}
	kDragTrackingLeaveWindow = 4;    { drag has exited window}
	kDragTrackingLeaveHandler = 5;     { drag has exited handler}


type
	DragRegionMessage = SInt16;
const
	kDragRegionBegin = 1;    { initialize drawing}
	kDragRegionDraw = 2;    { draw drag feedback}
	kDragRegionHide = 3;    { hide drag feedback}
	kDragRegionIdle = 4;    { drag feedback idle time}
	kDragRegionEnd = 5;     { end of drawing}


type
	ZoomAcceleration = SInt16;
const
	kZoomNoAcceleration = 0;    { use linear interpolation}
	kZoomAccelerate = 1;    { ramp up step size}
	kZoomDecelerate = 2;     { ramp down step size}


{
 *  Summary:
 *    Standard Drop Location constants
>>>>>>> graemeg/cpstrnew
 *  
 *  Discussion:
 *    The following constants define common "meta" drop locations.
 }
const
{
   * The drop location was in the trash.  This is set when a drag is
   * dropped on the trash icon.  Setting this standard drop location
   * sets the traditional drop location to an alias to the trash folder
   * automatically.
   }
	kDragStandardDropLocationTrash = FourCharCode('trsh');

  {
   * The receiver did not specify a drop lcoation. This is the default.
   }
	kDragStandardDropLocationUnknown = FourCharCode('unkn');


type
	StandardDropLocation = OSType;
	DragSendDataProcPtr = function( theType: FlavorType; dragSendRefCon: UnivPtr; theItemRef: DragItemRef; theDrag: DragRef ): OSErr;
	DragTrackingHandlerProcPtr = function( message: DragTrackingMessage; theWindow: WindowRef; handlerRefCon: UnivPtr; theDrag: DragRef ): OSErr;
	DragReceiveHandlerProcPtr = function( theWindow: WindowRef; handlerRefCon: UnivPtr; theDrag: DragRef ): OSErr;
	DragDrawingProcPtr = function( message: DragRegionMessage; showRegion: RgnHandle; showOrigin: Point; hideRegion: RgnHandle; hideOrigin: Point; dragDrawingRefCon: UnivPtr; theDrag: DragRef ): OSErr;
	DragSendDataUPP = DragSendDataProcPtr;
	DragTrackingHandlerUPP = DragTrackingHandlerProcPtr;
	DragReceiveHandlerUPP = DragReceiveHandlerProcPtr;
	DragDrawingUPP = DragDrawingProcPtr;
{
 *  NewDragSendDataUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function GetDragAttributes( theDrag: DragRef; var flags: DragAttributes ): OSErr; external name '_GetDragAttributes';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDragMouse()
 *  
 *  Mac OS X threading:
 *    Not thread safe
=======
function NewDragSendDataUPP( userRoutine: DragSendDataProcPtr ): DragSendDataUPP; external name '_NewDragSendDataUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  NewDragTrackingHandlerUPP()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function GetDragMouse( theDrag: DragRef; var mouse: Point; var globalPinnedMouse: Point ): OSErr; external name '_GetDragMouse';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetDragMouse()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function SetDragMouse( theDrag: DragRef; globalPinnedMouse: Point ): OSErr; external name '_SetDragMouse';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDragOrigin()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function GetDragOrigin( theDrag: DragRef; var globalInitialMouse: Point ): OSErr; external name '_GetDragOrigin';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDragModifiers()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function GetDragModifiers( theDrag: DragRef; var modifiers: SInt16; var mouseDownModifiers: SInt16; var mouseUpModifiers: SInt16 ): OSErr; external name '_GetDragModifiers';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDragItemBounds()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function GetDragItemBounds( theDrag: DragRef; theItemRef: DragItemRef; var itemBounds: Rect ): OSErr; external name '_GetDragItemBounds';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetDragItemBounds()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function SetDragItemBounds( theDrag: DragRef; theItemRef: DragItemRef; const (*var*) itemBounds: Rect ): OSErr; external name '_SetDragItemBounds';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
  _________________________________________________________________________________________________________
      
   ¥ ACCESSING DRAG ACTIONS
  _________________________________________________________________________________________________________
}

{
 *  GetDragAllowableActions()
 *  
 *  Discussion:
 *    Gets the actions the drag sender has allowed the receiver to
 *    perform. These are not requirements, but they highly suggested
 *    actions which allows the drag receiver to improve harmony across
 *    the system.  The allowable actions received are always those
 *    local to the caller's process.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    theDrag:
 *      The drag reference from which to retreive the allowable drag
 *      actions.
 *    
 *    outActions:
 *      A pointer to receive the field of allowable drag actions.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDragAllowableActions( theDrag: DragRef; var outActions: DragActions ): OSStatus; external name '_GetDragAllowableActions';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


{
 *  SetDragAllowableActions()
 *  
 *  Discussion:
 *    Sets the actions the receiver of the drag is allowed to perform. 
 *    These are not requirements, but they highly suggested actions
 *    which allows the drag receiver to improve harmony across the
 *    system.  The caller may select wether these drag actions apply to
 *    a local or remote process.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    theDrag:
 *      The drag reference in which to set the allowable drag actions.
 *    
 *    inActions:
 *      A field of allowable drag actions to be set.
 *    
 *    isLocal:
 *      A boolean value allowing the drag sender to distinguish between
 *      those drag actions allowable by the local receiver versus a
 *      remote one.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDragAllowableActions( theDrag: DragRef; inActions: DragActions; isLocal: Boolean ): OSStatus; external name '_SetDragAllowableActions';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


{
 *  GetDragDropAction()
 *  
 *  Discussion:
 *    Gets the action performed by the receiver of the drag.  More than
 *    one action may have been performed.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    theDrag:
 *      The drag reference from which to retreive the performed drop
 *      action.
 *    
 *    outAction:
 *      A pointer to receive the drag action performed.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   not available
 }
function GetDragDropAction( theDrag: DragRef; var outAction: DragActions ): OSStatus; external name '_GetDragDropAction';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


{
 *  SetDragDropAction()
 *  
 *  Discussion:
 *    Sets the action performed by the receiver of the drag.  More than
 *    one action may be performed.
 *  
 *  Mac OS X threading:
 *    Not thread safe
=======
function NewDragTrackingHandlerUPP( userRoutine: DragTrackingHandlerProcPtr ): DragTrackingHandlerUPP; external name '_NewDragTrackingHandlerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  NewDragReceiveHandlerUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDragReceiveHandlerUPP( userRoutine: DragReceiveHandlerProcPtr ): DragReceiveHandlerUPP; external name '_NewDragReceiveHandlerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  NewDragDrawingUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDragDrawingUPP( userRoutine: DragDrawingProcPtr ): DragDrawingUPP; external name '_NewDragDrawingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  DisposeDragSendDataUPP()
>>>>>>> graemeg/cpstrnew
 *  
 *  Parameters:
 *    
 *    theDrag:
 *      The drag reference in which to set the performed drop action.
 *    
 *    inAction:
 *      The drop action performed.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.1 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.1 and later
 *    Non-Carbon CFM:   not available
 }
function SetDragDropAction( theDrag: DragRef; inAction: DragActions ): OSStatus; external name '_SetDragDropAction';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER *)


{
  _________________________________________________________________________________________________________
      
   ¥ UTILITIES
  _________________________________________________________________________________________________________
}

{
 *  WaitMouseMoved()
 *  
 *  Mac OS X threading:
 *    Not thread safe
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDragSendDataUPP( userUPP: DragSendDataUPP ); external name '_DisposeDragSendDataUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  DisposeDragTrackingHandlerUPP()
>>>>>>> graemeg/cpstrnew
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function WaitMouseMoved( initialGlobalMouse: Point ): Boolean; external name '_WaitMouseMoved';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
=======
procedure DisposeDragTrackingHandlerUPP( userUPP: DragTrackingHandlerUPP ); external name '_DisposeDragTrackingHandlerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew

{
 *  DisposeDragReceiveHandlerUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDragReceiveHandlerUPP( userUPP: DragReceiveHandlerUPP ); external name '_DisposeDragReceiveHandlerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{--------------------------------------------------------------------------------------}
{  ¥ DEPRECATED                                                                        }
{  All functions below this point are either deprecated (they continue to function     }
{  but are not the most modern nor most efficient solution to a problem), or they are  }
{  completely unavailable on Mac OS X.                                                 }
{--------------------------------------------------------------------------------------}
{
<<<<<<< HEAD
   The file dragging constants and structures have been deprecated in Leopard. Use kUTTypeFileURL,
   kPasteboardTypeFileURLPromise and kPasteboardTypeFilePromiseContent from UTCoreTypes.h and Pasteboard.h instead.
}
=======
 *  DisposeDragDrawingUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDragDrawingUPP( userUPP: DragDrawingUPP ); external name '_DisposeDragDrawingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> graemeg/cpstrnew

{$endc} {not TARGET_CPU_64}

const
	kDragFlavorTypeHFS = FourCharCode('hfs '); { flavor type for HFS data}
	kDragFlavorTypePromiseHFS = FourCharCode('phfs'); { flavor type for promised HFS data}
	flavorTypeHFS = kDragFlavorTypeHFS; { old name}
	flavorTypePromiseHFS = kDragFlavorTypePromiseHFS; { old name}

const
	kDragPromisedFlavorFindFile = FourCharCode('rWm1'); { promisedFlavor value for Find File}
	kDragPromisedFlavor = FourCharCode('fssP'); { promisedFlavor value for everything else}

const
	kDragPseudoCreatorVolumeOrDirectory = FourCharCode('MACS'); { "creator code" for volume or directory}
	kDragPseudoFileTypeVolume = FourCharCode('disk'); { "file type" for volume}
	kDragPseudoFileTypeDirectory = FourCharCode('fold'); { "file type" for directory}

{
   The file dragging constants and structures have been deprecated in Leopard. Use kUTTypeFileURL,
   kPasteboardTypeFileURLPromise and kPasteboardTypeFilePromiseContent from UTCoreTypes.h and Pasteboard.h instead.
}

type
	HFSFlavorPtr = ^HFSFlavor;
	HFSFlavor = record
		fileType: OSType;               { file type }
		fileCreator: OSType;            { file creator }
		fdFlags: UInt16;                { Finder flags }
		fileSpec: FSSpec;               { file system specification }
	end;
type
	PromiseHFSFlavorPtr = ^PromiseHFSFlavor;
	PromiseHFSFlavor = record
		fileType: OSType;               { file type }
		fileCreator: OSType;            { file creator }
		fdFlags: UInt16;                { Finder flags }
		promisedFlavor: FlavorType;         { promised flavor containing an FSSpec }
	end;
type
	DragTrackingMessage = SInt16;
const
	kDragTrackingEnterHandler = 1;    { drag has entered handler}
	kDragTrackingEnterWindow = 2;    { drag has entered window}
	kDragTrackingInWindow = 3;    { drag is moving within window}
	kDragTrackingLeaveWindow = 4;    { drag has exited window}
	kDragTrackingLeaveHandler = 5;     { drag has exited handler}


type
	DragRegionMessage = SInt16;
const
	kDragRegionBegin = 1;    { initialize drawing}
	kDragRegionDraw = 2;    { draw drag feedback}
	kDragRegionHide = 3;    { hide drag feedback}
	kDragRegionIdle = 4;    { drag feedback idle time}
	kDragRegionEnd = 5;     { end of drawing}


type
	ZoomAcceleration = SInt16;
const
	kZoomNoAcceleration = 0;    { use linear interpolation}
	kZoomAccelerate = 1;    { ramp up step size}
	kZoomDecelerate = 2;     { ramp down step size}


{
 *  Summary:
 *    Standard Drop Location constants
 *  
 *  Discussion:
 *    The following constants define common "meta" drop locations.
 }
const
{
   * The drop location was in the trash.  This is set when a drag is
   * dropped on the trash icon.  Setting this standard drop location
   * sets the traditional drop location to an alias to the trash folder
   * automatically.
   }
	kDragStandardDropLocationTrash = FourCharCode('trsh');

  {
   * The receiver did not specify a drop lcoation. This is the default.
   }
	kDragStandardDropLocationUnknown = FourCharCode('unkn');


type
	StandardDropLocation = OSType;
	DragSendDataProcPtr = function( theType: FlavorType; dragSendRefCon: UnivPtr; theItemRef: DragItemRef; theDrag: DragRef ): OSErr;
	DragTrackingHandlerProcPtr = function( message: DragTrackingMessage; theWindow: WindowRef; handlerRefCon: UnivPtr; theDrag: DragRef ): OSErr;
	DragReceiveHandlerProcPtr = function( theWindow: WindowRef; handlerRefCon: UnivPtr; theDrag: DragRef ): OSErr;
	DragDrawingProcPtr = function( message: DragRegionMessage; showRegion: RgnHandle; showOrigin: Point; hideRegion: RgnHandle; hideOrigin: Point; dragDrawingRefCon: UnivPtr; theDrag: DragRef ): OSErr;
	DragSendDataUPP = DragSendDataProcPtr;
	DragTrackingHandlerUPP = DragTrackingHandlerProcPtr;
	DragReceiveHandlerUPP = DragReceiveHandlerProcPtr;
	DragDrawingUPP = DragDrawingProcPtr;
{
 *  NewDragSendDataUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDragSendDataUPP( userRoutine: DragSendDataProcPtr ): DragSendDataUPP; external name '_NewDragSendDataUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  NewDragTrackingHandlerUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDragTrackingHandlerUPP( userRoutine: DragTrackingHandlerProcPtr ): DragTrackingHandlerUPP; external name '_NewDragTrackingHandlerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  NewDragReceiveHandlerUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDragReceiveHandlerUPP( userRoutine: DragReceiveHandlerProcPtr ): DragReceiveHandlerUPP; external name '_NewDragReceiveHandlerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  NewDragDrawingUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDragDrawingUPP( userRoutine: DragDrawingProcPtr ): DragDrawingUPP; external name '_NewDragDrawingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  DisposeDragSendDataUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDragSendDataUPP( userUPP: DragSendDataUPP ); external name '_DisposeDragSendDataUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  DisposeDragTrackingHandlerUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDragTrackingHandlerUPP( userUPP: DragTrackingHandlerUPP ); external name '_DisposeDragTrackingHandlerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  DisposeDragReceiveHandlerUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDragReceiveHandlerUPP( userUPP: DragReceiveHandlerUPP ); external name '_DisposeDragReceiveHandlerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  DisposeDragDrawingUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDragDrawingUPP( userUPP: DragDrawingUPP ); external name '_DisposeDragDrawingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
=======
 *  DisposeDragDrawingUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDragDrawingUPP( userUPP: DragDrawingUPP ); external name '_DisposeDragDrawingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
>>>>>>> graemeg/cpstrnew
=======
 *  DisposeDragDrawingUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDragDrawingUPP( userUPP: DragDrawingUPP ); external name '_DisposeDragDrawingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
>>>>>>> origin/cpstrnew
 *  InvokeDragSendDataUPP()
=======
   ¥ ALTERING THE BEHAVIOR OF A DRAG
  _________________________________________________________________________________________________________
}
{
 *  ChangeDragBehaviors()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 9.0 and later
 }
function ChangeDragBehaviors( theDrag: DragRef; inBehaviorsToSet: DragBehaviors; inBehaviorsToClear: DragBehaviors ): OSErr; external name '_ChangeDragBehaviors';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
  _________________________________________________________________________________________________________
      
   ¥ PERFORMING A DRAG
  _________________________________________________________________________________________________________
}
{
 *  TrackDrag()
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function TrackDrag( theDrag: DragRef; const (*var*) theEvent: EventRecord; theRegion: RgnHandle ): OSErr; external name '_TrackDrag';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
  _________________________________________________________________________________________________________
      
   ¥ GETTING DRAG ITEM INFORMATION
  _________________________________________________________________________________________________________
}

{
 *  CountDragItems()
 *  
 *  Mac OS X threading:
 *    Not thread safe
>>>>>>> graemeg/fixes_2_2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
<<<<<<< HEAD
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeDragSendDataUPP( theType: FlavorType; dragSendRefCon: UnivPtr; theItemRef: DragItemRef; theDrag: DragRef; userUPP: DragSendDataUPP ): OSErr; external name '_InvokeDragSendDataUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  InvokeDragTrackingHandlerUPP()
=======
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function CountDragItems( theDrag: DragRef; var numItems: UInt16 ): OSErr; external name '_CountDragItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetDragItemReferenceNumber()
 *  
 *  Mac OS X threading:
 *    Not thread safe
>>>>>>> graemeg/fixes_2_2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
<<<<<<< HEAD
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeDragTrackingHandlerUPP( message: DragTrackingMessage; theWindow: WindowRef; handlerRefCon: UnivPtr; theDrag: DragRef; userUPP: DragTrackingHandlerUPP ): OSErr; external name '_InvokeDragTrackingHandlerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  InvokeDragReceiveHandlerUPP()
=======
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function GetDragItemReferenceNumber( theDrag: DragRef; index: UInt16; var theItemRef: DragItemRef ): OSErr; external name '_GetDragItemReferenceNumber';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  CountDragItemFlavors()
 *  
 *  Mac OS X threading:
 *    Not thread safe
>>>>>>> graemeg/fixes_2_2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
<<<<<<< HEAD
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeDragReceiveHandlerUPP( theWindow: WindowRef; handlerRefCon: UnivPtr; theDrag: DragRef; userUPP: DragReceiveHandlerUPP ): OSErr; external name '_InvokeDragReceiveHandlerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  InvokeDragDrawingUPP()
=======
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function CountDragItemFlavors( theDrag: DragRef; theItemRef: DragItemRef; var numFlavors: UInt16 ): OSErr; external name '_CountDragItemFlavors';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetFlavorType()
 *  
 *  Mac OS X threading:
 *    Not thread safe
>>>>>>> graemeg/fixes_2_2
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
<<<<<<< HEAD
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeDragDrawingUPP( message: DragRegionMessage; showRegion: RgnHandle; showOrigin: Point; hideRegion: RgnHandle; hideOrigin: Point; dragDrawingRefCon: UnivPtr; theDrag: DragRef; userUPP: DragDrawingUPP ): OSErr; external name '_InvokeDragDrawingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{$ifc not TARGET_CPU_64}
{
 *  GetStandardDropLocation()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use PasteboardGetStandardPasteLocation instead.
 *  
 *  Discussion:
 *    Gets the standard drop location that was set by the receiver of
 *    the drag.
=======
 *    Non-Carbon CFM:   in DragLib 1.1 and later
=======
 *    Non-Carbon CFM:   in DragLib 1.1 and later
>>>>>>> origin/fixes_2_2
 }
function GetFlavorType( theDrag: DragRef; theItemRef: DragItemRef; index: UInt16; var theType: FlavorType ): OSErr; external name '_GetFlavorType';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

=======
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeDragDrawingUPP( message: DragRegionMessage; showRegion: RgnHandle; showOrigin: Point; hideRegion: RgnHandle; hideOrigin: Point; dragDrawingRefCon: UnivPtr; theDrag: DragRef; userUPP: DragDrawingUPP ): OSErr; external name '_InvokeDragDrawingUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4

{$ifc not TARGET_CPU_64}
{
<<<<<<< HEAD
 *  GetFlavorFlags()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
<<<<<<< HEAD
<<<<<<< HEAD
 *  Parameters:
 *    
 *    theDrag:
 *      The drag reference from which to retrieve the allowable drag
 *      actions.
 *    
 *    outDropLocation:
 *      A pointer to the standard drop location, set by the receiver,
 *      representing the location where the drag was dropped.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function GetStandardDropLocation( theDrag: DragRef; var outDropLocation: StandardDropLocation ): OSStatus; external name '_GetStandardDropLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  SetStandardDropLocation()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use PasteboardSetStandardPasteLocation instead.
 *  
 *  Discussion:
 *    Used by the receiver of the drag to set the standard drop
 *    location.
=======
=======
>>>>>>> origin/fixes_2_2
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function GetFlavorFlags( theDrag: DragRef; theItemRef: DragItemRef; theType: FlavorType; var theFlags: FlavorFlags ): OSErr; external name '_GetFlavorFlags';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  GetFlavorDataSize()
>>>>>>> graemeg/fixes_2_2
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
<<<<<<< HEAD
 *  Parameters:
 *    
 *    theDrag:
 *      The drag reference from which to retrieve the allowable drag
 *      actions.
 *    
 *    dropLocation:
 *      The standard drop location representing the location where the
 *      drag was dropped.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function SetStandardDropLocation( theDrag: DragRef; dropLocation: StandardDropLocation ): OSStatus; external name '_SetStandardDropLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  ZoomRects()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use window transitions or custom drawing in an overlay window
 *    instead.
=======
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function GetFlavorDataSize( theDrag: DragRef; theItemRef: DragItemRef; theType: FlavorType; var dataSize: Size ): OSErr; external name '_GetFlavorDataSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  GetFlavorData()
>>>>>>> graemeg/fixes_2_2
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function ZoomRects( const (*var*) fromRect: Rect; const (*var*) toRect: Rect; zoomSteps: SInt16; acceleration: ZoomAcceleration ): OSErr; external name '_ZoomRects';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  ZoomRegion()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use window transitions or custom drawing in an overlay window
 *    instead.
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function GetFlavorData( theDrag: DragRef; theItemRef: DragItemRef; theType: FlavorType; dataPtr: UnivPtr; var dataSize: Size; dataOffset: UInt32 ): OSErr; external name '_GetFlavorData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
  _________________________________________________________________________________________________________
      
   ¥ DRAG ITEM BOUNDS
  _________________________________________________________________________________________________________
}

{
 *  GetDragItemBounds()
>>>>>>> graemeg/fixes_2_2
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function ZoomRegion( region: RgnHandle; zoomDistance: Point; zoomSteps: SInt16; acceleration: ZoomAcceleration ): OSErr; external name '_ZoomRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  SetDragImage()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Applications should use SetDragImageWithCGImage instead.
 *  
 *  Summary:
 *    Associates an image with a drag reference.
 *  
 *  Discussion:
 *    Used by the sender of the drag to set the image, in PixMapHandle
 *    format, to be displayed as user feedback during the drag.  This
 *    API may be called  at any point during the drag to update the
 *    image.
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function GetDragItemBounds( theDrag: DragRef; theItemRef: DragItemRef; var itemBounds: Rect ): OSErr; external name '_GetDragItemBounds';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  SetDragItemBounds()
>>>>>>> graemeg/fixes_2_2
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
<<<<<<< HEAD
 *  Parameters:
 *    
 *    inDrag:
 *      The drag reference for which the image will be displayed.
 *    
 *    inImagePixMap:
 *      The PixMapHandle for the image to be displayed during the drag.
 *    
 *    inImageRgn:
 *      A mask describing the portion of the PixMap contained in the
 *      imagePixMap parameter which contains the drag image. Pass NULL
 *      for inImageRgn if the entire PixMap, including white space,
 *      should be dragged.
 *    
 *    inImageOffsetPt:
 *      The offset required to move the PixMap specified in the
 *      imagePixMap parameter to the global coordinates where the image
 *      initially appears. If this parameter is (0,0), the PixMap
 *      should already be in global coordinates.
 *    
 *    inImageFlags:
 *      Flags controlling the appearance of the drag image.
=======
 *  GetStandardDropLocation()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use PasteboardGetStandardPasteLocation instead.
 *  
 *  Discussion:
 *    Gets the standard drop location that was set by the receiver of
 *    the drag.
>>>>>>> origin/fixes_2.4
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Parameters:
 *    
 *    theDrag:
 *      The drag reference from which to retrieve the allowable drag
 *      actions.
 *    
 *    outDropLocation:
 *      A pointer to the standard drop location, set by the receiver,
 *      representing the location where the drag was dropped.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 7.5 and later
 }
function SetDragImage( inDrag: DragRef; inImagePixMap: PixMapHandle; inImageRgn: RgnHandle; inImageOffsetPt: Point; inImageFlags: DragImageFlags ): OSErr; external name '_SetDragImage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
    The method for setting Drag Manager promises differs from that for Scrap Manger promises.  This chart
    describes the method for setting drag promises via AddDragItemFlavor().
    
        dataPtr         dataSize                                result
     pointer value  actual data size    The data of size dataSize pointed to by dataPtr is added to the drag.
        NULL             ignored        A promise is placed on the drag.
}
{
 *  AddDragItemFlavor()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The Drag Flavor APIs are deprecated. Use PasteboardPutItemFlavor
 *    instead.
=======
=======
 *  GetFlavorDataSize()
=======
 *    Mac OS X:         in version 10.2 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function GetStandardDropLocation( theDrag: DragRef; var outDropLocation: StandardDropLocation ): OSStatus; external name '_GetStandardDropLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  SetStandardDropLocation()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use PasteboardSetStandardPasteLocation instead.
 *  
 *  Discussion:
 *    Used by the receiver of the drag to set the standard drop
 *    location.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    theDrag:
 *      The drag reference from which to retrieve the allowable drag
 *      actions.
 *    
 *    dropLocation:
 *      The standard drop location representing the location where the
 *      drag was dropped.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function SetStandardDropLocation( theDrag: DragRef; dropLocation: StandardDropLocation ): OSStatus; external name '_SetStandardDropLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  ZoomRects()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use window transitions or custom drawing in an overlay window
 *    instead.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
>>>>>>> origin/fixes_2_2
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
function SetDragItemBounds( theDrag: DragRef; theItemRef: DragItemRef; const (*var*) itemBounds: Rect ): OSErr; external name '_SetDragItemBounds';
=======
function GetFlavorData( theDrag: DragRef; theItemRef: DragItemRef; theType: FlavorType; dataPtr: UnivPtr; var dataSize: Size; dataOffset: UInt32 ): OSErr; external name '_GetFlavorData';
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
  _________________________________________________________________________________________________________
      
<<<<<<< HEAD
   ¥ DROP LOCATIONS
=======
   ¥ DRAG ITEM BOUNDS
>>>>>>> origin/fixes_2_2
  _________________________________________________________________________________________________________
}

{
<<<<<<< HEAD
 *  GetDropLocation()
>>>>>>> graemeg/fixes_2_2
=======
 *  GetDragItemBounds()
>>>>>>> origin/fixes_2_2
=======
function ZoomRects( const (*var*) fromRect: Rect; const (*var*) toRect: Rect; zoomSteps: SInt16; acceleration: ZoomAcceleration ): OSErr; external name '_ZoomRects';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  ZoomRegion()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use window transitions or custom drawing in an overlay window
 *    instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function AddDragItemFlavor( theDrag: DragRef; theItemRef: DragItemRef; theType: FlavorType; dataPtr: {const} UnivPtr; dataSize: Size; theFlags: FlavorFlags ): OSErr; external name '_AddDragItemFlavor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  SetDragItemFlavorData()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The Drag Flavor APIs are deprecated. Use PasteboardPutItemFlavor
 *    instead.
=======
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function ZoomRegion( region: RgnHandle; zoomDistance: Point; zoomSteps: SInt16; acceleration: ZoomAcceleration ): OSErr; external name '_ZoomRegion';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  SetDragImage()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Applications should use SetDragImageWithCGImage instead.
 *  
 *  Summary:
 *    Associates an image with a drag reference.
 *  
 *  Discussion:
 *    Used by the sender of the drag to set the image, in PixMapHandle
 *    format, to be displayed as user feedback during the drag.  This
 *    API may be called  at any point during the drag to update the
 *    image.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inDrag:
 *      The drag reference for which the image will be displayed.
 *    
 *    inImagePixMap:
 *      The PixMapHandle for the image to be displayed during the drag.
 *    
 *    inImageRgn:
 *      A mask describing the portion of the PixMap contained in the
 *      imagePixMap parameter which contains the drag image. Pass NULL
 *      for inImageRgn if the entire PixMap, including white space,
 *      should be dragged.
 *    
 *    inImageOffsetPt:
 *      The offset required to move the PixMap specified in the
 *      imagePixMap parameter to the global coordinates where the image
 *      initially appears. If this parameter is (0,0), the PixMap
 *      should already be in global coordinates.
 *    
 *    inImageFlags:
 *      Flags controlling the appearance of the drag image.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
<<<<<<< HEAD
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 7.5 and later
 }
<<<<<<< HEAD
<<<<<<< HEAD
function GetDropLocation( theDrag: DragRef; var dropLocation: AEDesc ): OSErr; external name '_GetDropLocation';
=======
function SetDragItemBounds( theDrag: DragRef; theItemRef: DragItemRef; const (*var*) itemBounds: Rect ): OSErr; external name '_SetDragItemBounds';
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
<<<<<<< HEAD
 *  SetDropLocation()
>>>>>>> graemeg/fixes_2_2
=======
  _________________________________________________________________________________________________________
      
   ¥ DROP LOCATIONS
  _________________________________________________________________________________________________________
=======
function SetDragImage( inDrag: DragRef; inImagePixMap: PixMapHandle; inImageRgn: RgnHandle; inImageOffsetPt: Point; inImageFlags: DragImageFlags ): OSErr; external name '_SetDragImage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
    The method for setting Drag Manager promises differs from that for Scrap Manger promises.  This chart
    describes the method for setting drag promises via AddDragItemFlavor().
    
        dataPtr         dataSize                                result
     pointer value  actual data size    The data of size dataSize pointed to by dataPtr is added to the drag.
        NULL             ignored        A promise is placed on the drag.
>>>>>>> origin/fixes_2.4
}
{
<<<<<<< HEAD
 *  GetDropLocation()
>>>>>>> origin/fixes_2_2
=======
 *  AddDragItemFlavor()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The Drag Flavor APIs are deprecated. Use PasteboardPutItemFlavor
 *    instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function SetDragItemFlavorData( theDrag: DragRef; theItemRef: DragItemRef; theType: FlavorType; dataPtr: {const} UnivPtr; dataSize: Size; dataOffset: UInt32 ): OSErr; external name '_SetDragItemFlavorData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  InstallTrackingHandler()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Install drag suite event handlers on a drag tracking HIView
 *    instead.
=======
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function AddDragItemFlavor( theDrag: DragRef; theItemRef: DragItemRef; theType: FlavorType; dataPtr: {const} UnivPtr; dataSize: Size; theFlags: FlavorFlags ): OSErr; external name '_AddDragItemFlavor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  SetDragItemFlavorData()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The Drag Flavor APIs are deprecated. Use PasteboardPutItemFlavor
 *    instead.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function SetDragItemFlavorData( theDrag: DragRef; theItemRef: DragItemRef; theType: FlavorType; dataPtr: {const} UnivPtr; dataSize: Size; dataOffset: UInt32 ): OSErr; external name '_SetDragItemFlavorData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  InstallTrackingHandler()   *** DEPRECATED ***
 *  
<<<<<<< HEAD
 *  Discussion:
 *    Gets the standard drop location that was set by the receiver of
 *    the drag.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  Deprecated:
 *    Install drag suite event handlers on a drag tracking HIView
 *    instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function InstallTrackingHandler( trackingHandler: DragTrackingHandlerUPP; theWindow: WindowRef; handlerRefCon: UnivPtr ): OSErr; external name '_InstallTrackingHandler';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  InstallReceiveHandler()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Install drag suite event handlers on a drag tracking HIView
 *    instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *  Parameters:
 *    
 *    theDrag:
 *      The drag reference from which to retrieve the allowable drag
 *      actions.
 *    
 *    outDropLocation:
 *      A pointer to the standard drop location, set by the receiver,
 *      representing the location where the drag was dropped.
 *  
 *  Result:
 *    An operating system result code.
 *  
=======
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function InstallTrackingHandler( trackingHandler: DragTrackingHandlerUPP; theWindow: WindowRef; handlerRefCon: UnivPtr ): OSErr; external name '_InstallTrackingHandler';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  InstallReceiveHandler()   *** DEPRECATED ***
 *  
<<<<<<< HEAD
 *  Discussion:
 *    Used by the receiver of the drag to set the standard drop
 *    location.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  Deprecated:
 *    Install drag suite event handlers on a drag tracking HIView
 *    instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function InstallReceiveHandler( receiveHandler: DragReceiveHandlerUPP; theWindow: WindowRef; handlerRefCon: UnivPtr ): OSErr; external name '_InstallReceiveHandler';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  RemoveTrackingHandler()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Remove drag suite event handlers from a drag tracking HIView
 *    instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *  Parameters:
 *    
 *    theDrag:
 *      The drag reference from which to retrieve the allowable drag
 *      actions.
 *    
 *    dropLocation:
 *      The standard drop location representing the location where the
 *      drag was dropped.
 *  
 *  Result:
 *    An operating system result code.
 *  
=======
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function InstallReceiveHandler( receiveHandler: DragReceiveHandlerUPP; theWindow: WindowRef; handlerRefCon: UnivPtr ): OSErr; external name '_InstallReceiveHandler';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  GetDragAttributes()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  RemoveTrackingHandler()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Remove drag suite event handlers from a drag tracking HIView
 *    instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function RemoveTrackingHandler( trackingHandler: DragTrackingHandlerUPP; theWindow: WindowRef ): OSErr; external name '_RemoveTrackingHandler';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  RemoveReceiveHandler()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Remove drag suite event handlers from a drag tracking HIView
 *    instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function RemoveTrackingHandler( trackingHandler: DragTrackingHandlerUPP; theWindow: WindowRef ): OSErr; external name '_RemoveTrackingHandler';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  GetDragMouse()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  RemoveReceiveHandler()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Remove drag suite event handlers from a drag tracking HIView
 *    instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function RemoveReceiveHandler( receiveHandler: DragReceiveHandlerUPP; theWindow: WindowRef ): OSErr; external name '_RemoveReceiveHandler';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  SetDragSendProc()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The Drag Flavor APIs are deprecated. Use
 *    PasteboardSetPromiseKeeper instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function RemoveReceiveHandler( receiveHandler: DragReceiveHandlerUPP; theWindow: WindowRef ): OSErr; external name '_RemoveReceiveHandler';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  SetDragMouse()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  SetDragSendProc()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The Drag Flavor APIs are deprecated. Use
 *    PasteboardSetPromiseKeeper instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function SetDragSendProc( theDrag: DragRef; sendProc: DragSendDataUPP; dragSendRefCon: UnivPtr ): OSErr; external name '_SetDragSendProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  SetDragDrawingProc()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SetDragImageWithCGImage instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function SetDragSendProc( theDrag: DragRef; sendProc: DragSendDataUPP; dragSendRefCon: UnivPtr ): OSErr; external name '_SetDragSendProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  GetDragOrigin()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  SetDragDrawingProc()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use SetDragImageWithCGImage instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function SetDragDrawingProc( theDrag: DragRef; drawingProc: DragDrawingUPP; dragDrawingRefCon: UnivPtr ): OSErr; external name '_SetDragDrawingProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  CountDragItems()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The Drag Flavor APIs are deprecated. Use PasteboardGetItemCount
 *    instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function SetDragDrawingProc( theDrag: DragRef; drawingProc: DragDrawingUPP; dragDrawingRefCon: UnivPtr ): OSErr; external name '_SetDragDrawingProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  GetDragModifiers()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  CountDragItems()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The Drag Flavor APIs are deprecated. Use PasteboardGetItemCount
 *    instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function CountDragItems( theDrag: DragRef; var numItems: UInt16 ): OSErr; external name '_CountDragItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetDragItemReferenceNumber()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The Drag Flavor APIs are deprecated. Use
 *    PasteboardGetItemIdentifier instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function CountDragItems( theDrag: DragRef; var numItems: UInt16 ): OSErr; external name '_CountDragItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetDragItemReferenceNumber()   *** DEPRECATED ***
 *  
<<<<<<< HEAD
 *  Discussion:
 *    Gets the actions the drag sender has allowed the receiver to
 *    perform. These are not requirements, but they highly suggested
 *    actions which allows the drag receiver to improve harmony across
 *    the system.  The allowable actions received are always those
 *    local to the caller's process.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  Deprecated:
 *    The Drag Flavor APIs are deprecated. Use
 *    PasteboardGetItemIdentifier instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function GetDragItemReferenceNumber( theDrag: DragRef; theIndex: UInt16; var theItemRef: DragItemRef ): OSErr; external name '_GetDragItemReferenceNumber';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  CountDragItemFlavors()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The Drag Flavor APIs are deprecated. Use
 *    PasteboardCopyItemFlavors instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *  Parameters:
 *    
 *    theDrag:
 *      The drag reference from which to retreive the allowable drag
 *      actions.
 *    
 *    outActions:
 *      A pointer to receive the field of allowable drag actions.
 *  
 *  Result:
 *    An operating system result code.
 *  
=======
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function GetDragItemReferenceNumber( theDrag: DragRef; theIndex: UInt16; var theItemRef: DragItemRef ): OSErr; external name '_GetDragItemReferenceNumber';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  CountDragItemFlavors()   *** DEPRECATED ***
 *  
<<<<<<< HEAD
 *  Discussion:
 *    Sets the actions the receiver of the drag is allowed to perform. 
 *    These are not requirements, but they highly suggested actions
 *    which allows the drag receiver to improve harmony across the
 *    system.  The caller may select wether these drag actions apply to
 *    a local or remote process.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  Deprecated:
 *    The Drag Flavor APIs are deprecated. Use
 *    PasteboardCopyItemFlavors instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function CountDragItemFlavors( theDrag: DragRef; theItemRef: DragItemRef; var numFlavors: UInt16 ): OSErr; external name '_CountDragItemFlavors';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetFlavorType()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The Drag Flavor APIs are deprecated. Use
 *    PasteboardCopyItemFlavors instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *  Parameters:
 *    
 *    theDrag:
 *      The drag reference in which to set the allowable drag actions.
 *    
 *    inActions:
 *      A field of allowable drag actions to be set.
 *    
 *    isLocal:
 *      A boolean value allowing the drag sender to distinguish between
 *      those drag actions allowable by the local receiver versus a
 *      remote one.
 *  
 *  Result:
 *    An operating system result code.
 *  
=======
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function CountDragItemFlavors( theDrag: DragRef; theItemRef: DragItemRef; var numFlavors: UInt16 ): OSErr; external name '_CountDragItemFlavors';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetFlavorType()   *** DEPRECATED ***
 *  
<<<<<<< HEAD
 *  Discussion:
 *    Gets the action performed by the receiver of the drag.  More than
 *    one action may have been performed.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  Deprecated:
 *    The Drag Flavor APIs are deprecated. Use
 *    PasteboardCopyItemFlavors instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function GetFlavorType( theDrag: DragRef; theItemRef: DragItemRef; theIndex: UInt16; var theType: FlavorType ): OSErr; external name '_GetFlavorType';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetFlavorFlags()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The Drag Flavor APIs are deprecated. Use
 *    PasteboardGetItemFlavorFlags instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *  Parameters:
 *    
 *    theDrag:
 *      The drag reference from which to retreive the performed drop
 *      action.
 *    
 *    outAction:
 *      A pointer to receive the drag action performed.
 *  
 *  Result:
 *    An operating system result code.
 *  
=======
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function GetFlavorType( theDrag: DragRef; theItemRef: DragItemRef; theIndex: UInt16; var theType: FlavorType ): OSErr; external name '_GetFlavorType';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetFlavorFlags()   *** DEPRECATED ***
 *  
<<<<<<< HEAD
 *  Discussion:
 *    Sets the action performed by the receiver of the drag.  More than
 *    one action may be performed.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  Deprecated:
 *    The Drag Flavor APIs are deprecated. Use
 *    PasteboardGetItemFlavorFlags instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function GetFlavorFlags( theDrag: DragRef; theItemRef: DragItemRef; theType: FlavorType; var theFlags: FlavorFlags ): OSErr; external name '_GetFlavorFlags';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetFlavorDataSize()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The Drag Flavor APIs are deprecated. Use
 *    PasteboardCopyItemFlavorData instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *  Parameters:
 *    
 *    theDrag:
 *      The drag reference in which to set the performed drop action.
 *    
 *    inAction:
 *      The drop action performed.
 *  
 *  Result:
 *    An operating system result code.
 *  
=======
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function GetFlavorFlags( theDrag: DragRef; theItemRef: DragItemRef; theType: FlavorType; var theFlags: FlavorFlags ): OSErr; external name '_GetFlavorFlags';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  ShowDragHilite()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  GetFlavorDataSize()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The Drag Flavor APIs are deprecated. Use
 *    PasteboardCopyItemFlavorData instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function GetFlavorDataSize( theDrag: DragRef; theItemRef: DragItemRef; theType: FlavorType; var dataSize: Size ): OSErr; external name '_GetFlavorDataSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetFlavorData()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The Drag Flavor APIs are deprecated. Use
 *    PasteboardCopyItemFlavorData instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function GetFlavorDataSize( theDrag: DragRef; theItemRef: DragItemRef; theType: FlavorType; var dataSize: Size ): OSErr; external name '_GetFlavorDataSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  HideDragHilite()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  GetFlavorData()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The Drag Flavor APIs are deprecated. Use
 *    PasteboardCopyItemFlavorData instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function GetFlavorData( theDrag: DragRef; theItemRef: DragItemRef; theType: FlavorType; dataPtr: UnivPtr; var dataSize: Size; dataOffset: UInt32 ): OSErr; external name '_GetFlavorData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetDropLocation()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use PasteboardCopyPasteLocation instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function GetFlavorData( theDrag: DragRef; theItemRef: DragItemRef; theType: FlavorType; dataPtr: UnivPtr; var dataSize: Size; dataOffset: UInt32 ): OSErr; external name '_GetFlavorData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  DragPreScroll()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  GetDropLocation()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use PasteboardCopyPasteLocation instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function GetDropLocation( theDrag: DragRef; var dropLocation: AEDesc ): OSErr; external name '_GetDropLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  SetDropLocation()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use PasteboardSetPasteLocation instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function GetDropLocation( theDrag: DragRef; var dropLocation: AEDesc ): OSErr; external name '_GetDropLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  DragPostScroll()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  SetDropLocation()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use PasteboardSetPasteLocation instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function SetDropLocation( theDrag: DragRef; const (*var*) dropLocation: AEDesc ): OSErr; external name '_SetDropLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  ShowDragHilite()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The Drag scroll and hilite APIs are deprecated. Use the
 *    kThemeBrushDragHilite theme brush and draw the hilight as part of
 *    your custom window or control drawing instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function SetDropLocation( theDrag: DragRef; const (*var*) dropLocation: AEDesc ): OSErr; external name '_SetDropLocation';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  UpdateDragHilite()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ShowDragHilite()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The Drag scroll and hilite APIs are deprecated. Use the
 *    kThemeBrushDragHilite theme brush and draw the hilight as part of
 *    your custom window or control drawing instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function ShowDragHilite( theDrag: DragRef; hiliteFrame: RgnHandle; inside: Boolean ): OSErr; external name '_ShowDragHilite';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  HideDragHilite()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The Drag scroll and hilite APIs are deprecated. Use the
 *    kThemeBrushDragHilite theme brush and draw the hilight as part of
 *    your custom window or control drawing instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function ShowDragHilite( theDrag: DragRef; hiliteFrame: RgnHandle; inside: Boolean ): OSErr; external name '_ShowDragHilite';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  GetDragHiliteColor()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  HideDragHilite()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The Drag scroll and hilite APIs are deprecated. Use the
 *    kThemeBrushDragHilite theme brush and draw the hilight as part of
 *    your custom window or control drawing instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function HideDragHilite( theDrag: DragRef ): OSErr; external name '_HideDragHilite';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DragPreScroll()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The Drag scroll and hilite APIs are deprecated. Redraw as part of
 *    your custom window or control drawing instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function HideDragHilite( theDrag: DragRef ): OSErr; external name '_HideDragHilite';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  WaitMouseMoved()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  DragPreScroll()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The Drag scroll and hilite APIs are deprecated. Redraw as part of
 *    your custom window or control drawing instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function DragPreScroll( theDrag: DragRef; dH: SInt16; dV: SInt16 ): OSErr; external name '_DragPreScroll';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  DragPostScroll()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The Drag scroll and hilite APIs are deprecated. Redraw as part of
 *    your custom window or control drawing instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function DragPreScroll( theDrag: DragRef; dH: SInt16; dV: SInt16 ): OSErr; external name '_DragPreScroll';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  ZoomRects()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  DragPostScroll()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The Drag scroll and hilite APIs are deprecated. Redraw as part of
 *    your custom window or control drawing instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function DragPostScroll( theDrag: DragRef ): OSErr; external name '_DragPostScroll';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  UpdateDragHilite()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The Drag scroll and hilite APIs are deprecated. Use the
 *    kThemeBrushDragHilite theme brush and draw the hilight as part of
 *    your custom window or control drawing instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function DragPostScroll( theDrag: DragRef ): OSErr; external name '_DragPostScroll';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
<<<<<<< HEAD
 *  ZoomRegion()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  UpdateDragHilite()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The Drag scroll and hilite APIs are deprecated. Use the
 *    kThemeBrushDragHilite theme brush and draw the hilight as part of
 *    your custom window or control drawing instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function UpdateDragHilite( theDrag: DragRef; updateRgn: RgnHandle ): OSErr; external name '_UpdateDragHilite';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetDragHiliteColor()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    The Drag scroll and hilite APIs are deprecated. Use the
 *    kThemeBrushDragHilite theme brush and draw the hilight as part of
 *    your custom window or control drawing instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function UpdateDragHilite( theDrag: DragRef; updateRgn: RgnHandle ): OSErr; external name '_UpdateDragHilite';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  GetDragHiliteColor()   *** DEPRECATED ***
 *  
 *  Deprecated:
<<<<<<< HEAD
 *    Applications should use SetDragImageWithCGImage instead.
 *  
 *  Summary:
 *    Associates an image with a drag reference.
 *  
 *  Discussion:
 *    Used by the sender of the drag to set the image, in PixMapHandle
 *    format, to be displayed as user feedback during the drag.  This
 *    API may be called  at any point during the drag to update the
 *    image.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    The Drag scroll and hilite APIs are deprecated. Use the
 *    kThemeBrushDragHilite theme brush and draw the hilight as part of
 *    your custom window or control drawing instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 7.5 and later
 }
function GetDragHiliteColor( window: WindowRef; var color: RGBColor ): OSErr; external name '_GetDragHiliteColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
=======
=======
>>>>>>> origin/fixes_2_2
 *  Parameters:
 *    
 *    inDrag:
 *      The drag reference for which the image will be displayed.
 *    
 *    inImagePixMap:
 *      The PixMapHandle for the image to be displayed during the drag.
 *    
 *    inImageRgn:
 *      A mask describing the portion of the PixMap contained in the
 *      imagePixMap parameter which contains the drag image. Pass NULL
 *      for inImageRgn if the entire PixMap, including white space,
 *      should be dragged.
 *    
 *    inImageOffsetPt:
 *      The offset required to move the PixMap specified in the
 *      imagePixMap parameter to the global coordinates where the image
 *      initially appears. If this parameter is (0,0), the PixMap
 *      should already be in global coordinates.
 *    
 *    inImageFlags:
 *      Flags controlling the appearance of the drag image.
 *  
 *  Result:
 *    An operating system result code.
 *  
=======
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 7.5 and later
 }
<<<<<<< HEAD
function SetDragImage( inDrag: DragRef; inImagePixMap: PixMapHandle; inImageRgn: RgnHandle; inImageOffsetPt: Point; inImageFlags: DragImageFlags ): OSErr; external name '_SetDragImage';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function GetDragHiliteColor( window: WindowRef; var color: RGBColor ): OSErr; external name '_GetDragHiliteColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
>>>>>>> origin/fixes_2.4


{
  _________________________________________________________________________________________________________
   ¥ OLD NAMES
     These are provided for compatiblity with older source bases.  It is recommended to not use them since
     they may removed from this interface file at any time.
  _________________________________________________________________________________________________________
}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not TARGET_CPU_64}

type
	DragReference = DragRef;
=======
=======
>>>>>>> origin/fixes_2_2

type
	DragReference = DragRef;
type
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {not TARGET_CPU_64}

type
	DragReference = DragRef;
>>>>>>> origin/fixes_2.4
	ItemReference = DragItemRef;
(*
#if OLDROUTINENAMES
const
	dragHasLeftSenderWindow = kDragHasLeftSenderWindow; { drag has left the source window since TrackDrag }
	dragInsideSenderApplication = kDragInsideSenderApplication; { drag is occurring within the sender application }
	dragInsideSenderWindow = kDragInsideSenderWindow; { drag is occurring within the sender window }

const
	dragTrackingEnterHandler = kDragTrackingEnterHandler; { drag has entered handler }
	dragTrackingEnterWindow = kDragTrackingEnterWindow; { drag has entered window }
	dragTrackingInWindow = kDragTrackingInWindow; { drag is moving within window }
	dragTrackingLeaveWindow = kDragTrackingLeaveWindow; { drag has exited window }
	dragTrackingLeaveHandler = kDragTrackingLeaveHandler; { drag has exited handler }

const
	dragRegionBegin = kDragRegionBegin; { initialize drawing }
	dragRegionDraw = kDragRegionDraw; { draw drag feedback }
	dragRegionHide = kDragRegionHide; { hide drag feedback }
	dragRegionIdle = kDragRegionIdle; { drag feedback idle time }
	dragRegionEnd = kDragRegionEnd; { end of drawing }

const
	zoomNoAcceleration = kZoomNoAcceleration; { use linear interpolation }
	zoomAccelerate = kZoomAccelerate; { ramp up step size }
	zoomDecelerate = kZoomDecelerate; { ramp down step size }

const
	kDragStandardImage = kDragStandardTranslucency; { 65% image translucency (standard)}
	kDragDarkImage = kDragDarkTranslucency; { 50% image translucency}
	kDragDarkerImage = kDragDarkerTranslucency; { 25% image translucency}
	kDragOpaqueImage = kDragOpaqueTranslucency; { 0% image translucency (opaque)}

#endif  { OLDROUTINENAMES }
*)

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
