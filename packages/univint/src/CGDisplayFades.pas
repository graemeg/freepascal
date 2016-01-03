<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ CoreGraphics - CGDisplayFade.h
   Copyright (c) 2002-2008 Apple Inc.
   All rights reserved. }
{       Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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
{
    Modified for use with Free Pascal
    Version 308
    Please report any bugs to <gpc@microbizz.nl>
}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
{
 *  CGDisplayFade.h
 *  CoreGraphics
 *
 *  API to fade displays to and from a solid color, without resorting
 *  to playing with the gamma table APIs and losing ColorSync calibration.
 *
 *  These APIs should be used in perference to manipulating the gamma tables
 *  for purposes of performing fade effects.
 *
 *  Copyright (c) 2002 Apple Computer, Inc. All rights reserved.
 *
 }
=======
{ CoreGraphics - CGDisplayFade.h
   Copyright (c) 2002-2008 Apple Inc.
   All rights reserved. }
>>>>>>> origin/fixes_2.4
{       Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
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

unit CGDisplayFades;
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
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
=======
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
>>>>>>> graemeg/cpstrnew
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
>>>>>>> graemeg/cpstrnew
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
<<<<<<< HEAD
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
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> graemeg/cpstrnew
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
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
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
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
uses MacTypes,CGBase,CGErrors,CGDirectDisplay,CGDisplayConfiguration;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not MACOSALLINCLUDE}

{$ALIGN POWER}


{ These APIs are used to fade displays to and from a solid color without
   resorting to playing with the gamma table APIs and losing ColorSync
   calibration.

   These APIs should be used in preference to manipulating the gamma tables
   for purposes of performing fade effects. }

=======
{$ALIGN POWER}


>>>>>>> graemeg/fixes_2_2
=======
{$ALIGN POWER}


>>>>>>> origin/fixes_2_2
=======
{$endc} {not MACOSALLINCLUDE}

{$ALIGN POWER}


{ These APIs are used to fade displays to and from a solid color without
   resorting to playing with the gamma table APIs and losing ColorSync
   calibration.

   These APIs should be used in preference to manipulating the gamma tables
   for purposes of performing fade effects. }

>>>>>>> origin/fixes_2.4
type
	CGDisplayFadeReservationToken = UInt32;
const
	kCGDisplayFadeReservationInvalidToken = 0;

type
	CGDisplayBlendFraction = Float32;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4

{ Values for the limits of the fade. `kCGDisplayBlendNormal' represents a
   normal display state. `kCGDisplayBlendSolidColor' represents a display
   blended to a solid color }

<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
{
 * Values for the limits of the fade.
 *	kCGDisplayBlendNormal represents a normal display state
 *	kCGDisplayBlendSolidColor represents a display blended to a solid color
 }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
const
	kCGDisplayBlendNormal = 0.0;
const
	kCGDisplayBlendSolidColor = 1.0;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ Time in seconds to perform a fade operation. }

type
	CGDisplayFadeInterval = Float32;

{$ifc TARGET_OS_MAC}

{ Set the display fade time and color for a display reconfigure operation.
   Call this function after `CGBeginDisplayConfiguration' and before
   `CGCompleteDisplayConfiguration'.

   When `CGCompleteDisplayConfiguration' is called, a fade-out effect will
   occur prior to the display reconfiguration. When the reconfiguration is
   complete, control returns to the calling program; the fade-in effect runs
   asynchronously. }

function CGConfigureDisplayFadeEffect( config: CGDisplayConfigRef; fadeOutSeconds: CGDisplayFadeInterval; fadeInSeconds: CGDisplayFadeInterval; fadeRed: Float32; fadeGreen: Float32; fadeBlue: Float32 ): CGError; external name '_CGConfigureDisplayFadeEffect';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ It may be desirable to perform fade operations at other times, such as
   when transitioning between game play and cinematic sequences. The
   following API provides a mechanism for controlling display fade
   operations outside of display mode reconfigurations. }

{ Return a reservation token for the display hardware. Before performing a
   fade operation, the caller must reserve the hardware for the expected
   period of time that the program will be doing fades.

   Failing to release the hardware by the end of the reservation interval
   will result in the reservation token becomingn invalid, and the hardware
   being unfaded back to a normal state. The reservation interval is limited
   to a maximum of 15 seconds and should be greater than zero. }

=======
=======
>>>>>>> origin/fixes_2_2
{
 * Time in seconds to perform a fade operation.
 }
=======
{ Time in seconds to perform a fade operation. }

>>>>>>> origin/fixes_2.4
type
	CGDisplayFadeInterval = Float32;

{$ifc TARGET_OS_MAC}

{ Set the display fade time and color for a display reconfigure operation.
   Call this function after `CGBeginDisplayConfiguration' and before
   `CGCompleteDisplayConfiguration'.

   When `CGCompleteDisplayConfiguration' is called, a fade-out effect will
   occur prior to the display reconfiguration. When the reconfiguration is
   complete, control returns to the calling program; the fade-in effect runs
   asynchronously. }

function CGConfigureDisplayFadeEffect( config: CGDisplayConfigRef; fadeOutSeconds: CGDisplayFadeInterval; fadeInSeconds: CGDisplayFadeInterval; fadeRed: Float32; fadeGreen: Float32; fadeBlue: Float32 ): CGError; external name '_CGConfigureDisplayFadeEffect';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ It may be desirable to perform fade operations at other times, such as
   when transitioning between game play and cinematic sequences. The
   following API provides a mechanism for controlling display fade
   operations outside of display mode reconfigurations. }

{ Return a reservation token for the display hardware. Before performing a
   fade operation, the caller must reserve the hardware for the expected
   period of time that the program will be doing fades.

   Failing to release the hardware by the end of the reservation interval
   will result in the reservation token becomingn invalid, and the hardware
   being unfaded back to a normal state. The reservation interval is limited
   to a maximum of 15 seconds and should be greater than zero. }

<<<<<<< HEAD
{
 * It may also be desirable to perform fade operations at other times, as when
 * transitioning between game play and cinematic sequences.  The following API
 * provides a mechanism for controlling display fade operations outside of display
 * mode reconfigurations.
 }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
type
	CGDisplayReservationInterval = Float32;
const
	kCGMaxDisplayReservationInterval = 15.0;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

function CGAcquireDisplayFadeReservation( seconds: CGDisplayReservationInterval; var token: CGDisplayFadeReservationToken ): CGError; external name '_CGAcquireDisplayFadeReservation';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Release a display fade reservation, and unfades the display if needed.
   The reservation token is no longer valid after this operation.
 
   `CGReleaseDisplayFadeReservation' may be safely called while an
   asynchronous fade operation is running; if the ending blend value is
   `kCGDisplayBlendNormal', this will not disturb the running operation. The
   reservation is dropped when the fade operation completes. }

function CGReleaseDisplayFadeReservation( token: CGDisplayFadeReservationToken ): CGError; external name '_CGReleaseDisplayFadeReservation';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Perform a fade operation. The reservation token `token' must have been
   previously acquired from `CGAcquireDisplayFadeReservation'. The duration
   of the fade is specified by `duration'. The starting and ending values of
   the alpha component of the desired blend color are specified by
   `startBlend' and `endBlend', respectively. The RGB blend color is
   specified by the three components `(redBlend, greenBlend, blueBlend)'. If
   the operation should be synchronous, specify true for `synchronous',
   otherwise specify false.

   Over the fade operation time interval, the system interpolates the
   blending coefficient between the specified starting and ending values,
   applying a nonlinear (sine-based) bias term, and blends the video output
   with the specified color based on the resulting value.

   If the time interval is specifed as 0, then the ending state blend value
   is applied at once and the function returns.

   The maximum allowable time interval is 15 seconds.

   If the `synchronous' is true, the function does not return until the fade
   operation is complete. If false, the function returns at once, and the
   fade operation runs asynchronously.
   
   To perform a two-second fade-to-black, waiting until complete:

     CGDisplayFade(token,
		   2,				// 2 seconds 
		   kCGDisplayBlendNormal,	// Starting state 
		   kCGDisplayBlendSolidColor,	// Ending state 
		   0, 0, 0,			// black 
		   true);			// Wait for completion 

   To perform a two-second fade-from-black to normal, without waiting for
   completion:

     CGDisplayFade(token,
		   2,				// 2 seconds 
		   kCGDisplayBlendSolidColor,	// Starting state 
		   kCGDisplayBlendNormal,	// Ending state 
		   0, 0, 0,			// black 
		   false);			// Don't wait for completion }

function CGDisplayFade( token: CGDisplayFadeReservationToken; duration: CGDisplayFadeInterval; startBlend: CGDisplayBlendFraction; endBlend: CGDisplayBlendFraction; redBlend: Float32; greenBlend: Float32; blueBlend: Float32; synchronous: boolean_t ): CGError; external name '_CGDisplayFade';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Return true if a fade operation is currently in progress, false
   otherwise. }

function CGDisplayFadeOperationInProgress: boolean_t; external name '_CGDisplayFadeOperationInProgress';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_2, __MAC_10_9, __IPHONE_NA, __IPHONE_NA) *)
=======
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)
>>>>>>> graemeg/cpstrnew
=======
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)
>>>>>>> origin/cpstrnew

{$endc}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
{
 * Before performing fade operation, the caller must reserve the hardware
 * for the expected period of time that the program will be doing fades
 *
 * A reservation token is returned that must be passed in on subsequent calls.
 *
 * Failing to release the hardware by the end of the reservation interval will
 * result in the reservation token becomingn invalid, and the hardware being
 * unfaded back to a normal state.  The reservation interval is limited (clipped)
 * to 15 seconds maximum, and should be greater than zero.
 *
 * Returns kCGErrorNoneAvailable if another reservation is in effect,
 * and kCGErrorSuccess on success.
 }
function CGAcquireDisplayFadeReservation( seconds: CGDisplayReservationInterval; var pNewToken: CGDisplayFadeReservationToken ): CGError; external name '_CGAcquireDisplayFadeReservation';
=======
>>>>>>> origin/fixes_2.4

function CGAcquireDisplayFadeReservation( seconds: CGDisplayReservationInterval; var token: CGDisplayFadeReservationToken ): CGError; external name '_CGAcquireDisplayFadeReservation';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Release a display fade reservation, and unfades the display if needed.
   The reservation token is no longer valid after this operation.
 
   `CGReleaseDisplayFadeReservation' may be safely called while an
   asynchronous fade operation is running; if the ending blend value is
   `kCGDisplayBlendNormal', this will not disturb the running operation. The
   reservation is dropped when the fade operation completes. }

function CGReleaseDisplayFadeReservation( token: CGDisplayFadeReservationToken ): CGError; external name '_CGReleaseDisplayFadeReservation';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Perform a fade operation. The reservation token `token' must have been
   previously acquired from `CGAcquireDisplayFadeReservation'. The duration
   of the fade is specified by `duration'. The starting and ending values of
   the alpha component of the desired blend color are specified by
   `startBlend' and `endBlend', respectively. The RGB blend color is
   specified by the three components `(redBlend, greenBlend, blueBlend)'. If
   the operation should be synchronous, specify true for `synchronous',
   otherwise specify false.

   Over the fade operation time interval, the system interpolates the
   blending coefficient between the specified starting and ending values,
   applying a nonlinear (sine-based) bias term, and blends the video output
   with the specified color based on the resulting value.

   If the time interval is specifed as 0, then the ending state blend value
   is applied at once and the function returns.

   The maximum allowable time interval is 15 seconds.

   If the `synchronous' is true, the function does not return until the fade
   operation is complete. If false, the function returns at once, and the
   fade operation runs asynchronously.
   
   To perform a two-second fade-to-black, waiting until complete:

     CGDisplayFade(token,
		   2,				// 2 seconds 
		   kCGDisplayBlendNormal,	// Starting state 
		   kCGDisplayBlendSolidColor,	// Ending state 
		   0, 0, 0,			// black 
		   true);			// Wait for completion 

   To perform a two-second fade-from-black to normal, without waiting for
   completion:

     CGDisplayFade(token,
		   2,				// 2 seconds 
		   kCGDisplayBlendSolidColor,	// Starting state 
		   kCGDisplayBlendNormal,	// Ending state 
		   0, 0, 0,			// black 
		   false);			// Don't wait for completion }

function CGDisplayFade( token: CGDisplayFadeReservationToken; duration: CGDisplayFadeInterval; startBlend: CGDisplayBlendFraction; endBlend: CGDisplayBlendFraction; redBlend: Float32; greenBlend: Float32; blueBlend: Float32; synchronous: boolean_t ): CGError; external name '_CGDisplayFade';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{ Return true if a fade operation is currently in progress, false
   otherwise. }

function CGDisplayFadeOperationInProgress: boolean_t; external name '_CGDisplayFadeOperationInProgress';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_NA) *)

{$endc}
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
