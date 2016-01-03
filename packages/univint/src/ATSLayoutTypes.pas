{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       ATS/ATSLayoutTypes.h
 
     Contains:   Apple Type Services layout public structures and constants.
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Version:    ATS
 
     Copyright:  © 1994-2012 by Apple Inc., all rights reserved.
=======
     Copyright:  © 1994-2008 by Apple Inc., all rights reserved.
 
     Warning:    *** APPLE INTERNAL USE ONLY ***
                 This file may contain unreleased API's
>>>>>>> graemeg/cpstrnew
 
     BuildInfo:  Built by:            root
                 On:                  Fri Jul 24 22:21:51 2009
                 With Interfacer:     3.0d46   (Mac OS X for PowerPC)
                 From:                ATSLayoutTypes.i
                     Revision:        1.5
                     Dated:           2007/01/15 23:28:25
                     Last change by:  kurita
                     Last comment:    <rdar://problem/4916090> updated copyright.
 
<<<<<<< HEAD
                     http://bugs.freepascal.org
=======
     Bugs:       Report bugs to Radar component "System Interfaces", "Latest"
                 List the version information (from above) in the Problem Description.
>>>>>>> graemeg/cpstrnew
=======
     Copyright:  © 1994-2008 by Apple Inc., all rights reserved.
 
     Warning:    *** APPLE INTERNAL USE ONLY ***
                 This file may contain unreleased API's
 
     BuildInfo:  Built by:            root
                 On:                  Fri Jul 24 22:21:51 2009
                 With Interfacer:     3.0d46   (Mac OS X for PowerPC)
                 From:                ATSLayoutTypes.i
                     Revision:        1.5
                     Dated:           2007/01/15 23:28:25
                     Last change by:  kurita
                     Last comment:    <rdar://problem/4916090> updated copyright.
 
     Bugs:       Report bugs to Radar component "System Interfaces", "Latest"
                 List the version information (from above) in the Problem Description.
>>>>>>> graemeg/cpstrnew
=======
     Copyright:  © 1994-2008 by Apple Inc., all rights reserved.
 
     Warning:    *** APPLE INTERNAL USE ONLY ***
                 This file may contain unreleased API's
 
     BuildInfo:  Built by:            root
                 On:                  Fri Jul 24 22:21:51 2009
                 With Interfacer:     3.0d46   (Mac OS X for PowerPC)
                 From:                ATSLayoutTypes.i
                     Revision:        1.5
                     Dated:           2007/01/15 23:28:25
                     Last change by:  kurita
                     Last comment:    <rdar://problem/4916090> updated copyright.
 
     Bugs:       Report bugs to Radar component "System Interfaces", "Latest"
                 List the version information (from above) in the Problem Description.
>>>>>>> graemeg/cpstrnew
=======
     Copyright:  © 1994-2008 by Apple Inc., all rights reserved.
 
     Warning:    *** APPLE INTERNAL USE ONLY ***
                 This file may contain unreleased API's
 
     BuildInfo:  Built by:            root
                 On:                  Fri Jul 24 22:21:51 2009
                 With Interfacer:     3.0d46   (Mac OS X for PowerPC)
                 From:                ATSLayoutTypes.i
                     Revision:        1.5
                     Dated:           2007/01/15 23:28:25
                     Last change by:  kurita
                     Last comment:    <rdar://problem/4916090> updated copyright.
 
     Bugs:       Report bugs to Radar component "System Interfaces", "Latest"
                 List the version information (from above) in the Problem Description.
>>>>>>> origin/cpstrnew
 
}

{ Pascal Translation Updated: Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
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
     File:       ATSLayoutTypes.p
=======
     File:       ATS/ATSLayoutTypes.h
>>>>>>> origin/fixes_2.4
 
     Contains:   Apple Type Services layout public structures and constants.
 
     Copyright:  © 1994-2008 by Apple Inc., all rights reserved.
 
     Warning:    *** APPLE INTERNAL USE ONLY ***
                 This file may contain unreleased API's
 
     BuildInfo:  Built by:            root
                 On:                  Fri Jul 24 22:21:51 2009
                 With Interfacer:     3.0d46   (Mac OS X for PowerPC)
                 From:                ATSLayoutTypes.i
                     Revision:        1.5
                     Dated:           2007/01/15 23:28:25
                     Last change by:  kurita
                     Last comment:    <rdar://problem/4916090> updated copyright.
 
     Bugs:       Report bugs to Radar component "System Interfaces", "Latest"
                 List the version information (from above) in the Problem Description.
 
}

{ Pascal Translation Updated: Gorazd Krosl <gorazd_1957@yahoo.ca>, October 2009 }

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

unit ATSLayoutTypes;
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
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
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
uses MacTypes,SFNTLayoutTypes,ATSTypes;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not MACOSALLINCLUDE}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{$ifc TARGET_OS_MAC}
<<<<<<< HEAD
=======

{$ALIGN POWER}
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}

=======

{$ifc TARGET_OS_MAC}

>>>>>>> graemeg/cpstrnew
=======

{$ifc TARGET_OS_MAC}

>>>>>>> origin/cpstrnew
{$ALIGN POWER}

=======
=======
{$endc} {not MACOSALLINCLUDE}

>>>>>>> origin/fixes_2.4

{$ifc TARGET_OS_MAC}

{$ALIGN POWER}

<<<<<<< HEAD
{$ALIGN MAC68K}
>>>>>>> graemeg/fixes_2_2
=======


{$ALIGN MAC68K}
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{ --------------------------------------------------------------------------- }
{ CONSTANTS and related scalar types }
{ --------------------------------------------------------------------------- }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{ --------------------------------------------------------------------------- }
{ Miscellaneous Constants }
{ --------------------------------------------------------------------------- }
const
	kATSUseGlyphAdvance = $7FFFFFFF; { assignment to use natural glyph advance value }
	kATSUseLineHeight = $7FFFFFFF; { assignment to use natural line ascent/descent values }
	kATSNoTracking = $80000000; { negativeInfinity }
<<<<<<< HEAD
<<<<<<< HEAD

{ --------------------------------------------------------------------------- }

=======

{ --------------------------------------------------------------------------- }

>>>>>>> origin/cpstrnew
{
 *  Summary:
 *    These values are passed into the ATSUGetGlyphBounds function to
 *    indicate whether the width of the resulting typographic glyph
 *    bounds will be determined using the caret origin, glyph origin in
 *    device space, or glyph origin in fractional absolute positions
 }
const
{
   * Specifies that the width of the typographic glyph bounds will be
   * determined using the caret origin. The caret origin is halfway
   * between two characters.
   }
	kATSUseCaretOrigins = 0;

  {
   * Specifies that the width of the typographic glyph bounds will be
   * determined using the glyph origin in device space. This is useful
   * for adjusting text on the screen.
   }
	kATSUseDeviceOrigins = 1;

  {
   * Specifies that the width of the typographic glyph bounds will be
   * determined using the glyph origin in fractional absolute
   * positions, which are uncorrected for device display. This provides
   * the ideal position of laid-out text and is useful for scaling text
   * on the screen. This origin is also used to get the width of the
   * typographic bounding rectangle when you call ATSUMeasureText.
   }
	kATSUseFractionalOrigins = 2;
	kATSUseOriginFlags = 3;

{ ---------------------------------------------------------------------------- }
=======
=======
>>>>>>> origin/fixes_2_2
{ Miscellaneous constants }
=======
>>>>>>> origin/fixes_2.4

{ --------------------------------------------------------------------------- }

{
 *  Summary:
 *    These values are passed into the ATSUGetGlyphBounds function to
 *    indicate whether the width of the resulting typographic glyph
 *    bounds will be determined using the caret origin, glyph origin in
 *    device space, or glyph origin in fractional absolute positions
 }
const
<<<<<<< HEAD
  kATSUseGlyphAdvance   = $7FFFFFFF;					{ assignment to use natural glyph advance value }
	kATSUseLineHeight			= $7FFFFFFF;					{  assignment to use natural line ascent/descent values  }
	kATSNoTracking				= $80000000;					{  negativeInfinity  }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{
   * Specifies that the width of the typographic glyph bounds will be
   * determined using the caret origin. The caret origin is halfway
   * between two characters.
   }
	kATSUseCaretOrigins = 0;

  {
   * Specifies that the width of the typographic glyph bounds will be
   * determined using the glyph origin in device space. This is useful
   * for adjusting text on the screen.
   }
	kATSUseDeviceOrigins = 1;

  {
   * Specifies that the width of the typographic glyph bounds will be
   * determined using the glyph origin in fractional absolute
   * positions, which are uncorrected for device display. This provides
   * the ideal position of laid-out text and is useful for scaling text
   * on the screen. This origin is also used to get the width of the
   * typographic bounding rectangle when you call ATSUMeasureText.
   }
	kATSUseFractionalOrigins = 2;
	kATSUseOriginFlags = 3;

{ ---------------------------------------------------------------------------- }
>>>>>>> origin/fixes_2.4

{
 *  ATSULayoutOperationSelector
 *  
 *  Summary:
 *    This is used to select which operations to override, or which
 *    operation is currently being run.
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
	ATSULayoutOperationSelector = UInt32;
const
{
   * No Layout operation is currently selected.
   }
	kATSULayoutOperationNone = $00000000;
=======
=======
>>>>>>> origin/fixes_2_2
type ATSULayoutOperationSelector = UInt32;
=======
type
	ATSULayoutOperationSelector = UInt32;
>>>>>>> origin/fixes_2.4
const
{
   * No Layout operation is currently selected.
   }
<<<<<<< HEAD
  kATSULayoutOperationNone      = $00000000;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kATSULayoutOperationNone = $00000000;
>>>>>>> origin/fixes_2.4

  {
   * Select the Justification operation.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSULayoutOperationJustification = $00000001;
=======
  kATSULayoutOperationJustification = $00000001;
>>>>>>> graemeg/fixes_2_2
=======
  kATSULayoutOperationJustification = $00000001;
>>>>>>> origin/fixes_2_2
=======
	kATSULayoutOperationJustification = $00000001;
>>>>>>> origin/fixes_2.4

  {
   * Select the character morphing operation.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSULayoutOperationMorph = $00000002;
=======
  kATSULayoutOperationMorph     = $00000002;
>>>>>>> graemeg/fixes_2_2
=======
  kATSULayoutOperationMorph     = $00000002;
>>>>>>> origin/fixes_2_2
=======
	kATSULayoutOperationMorph = $00000002;
>>>>>>> origin/fixes_2.4

  {
   * Select the kerning adjustment operation.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSULayoutOperationKerningAdjustment = $00000004;
=======
  kATSULayoutOperationKerningAdjustment = $00000004;
>>>>>>> graemeg/fixes_2_2
=======
  kATSULayoutOperationKerningAdjustment = $00000004;
>>>>>>> origin/fixes_2_2
=======
	kATSULayoutOperationKerningAdjustment = $00000004;
>>>>>>> origin/fixes_2.4

  {
   * Select the baseline adjustment operation.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSULayoutOperationBaselineAdjustment = $00000008;
=======
  kATSULayoutOperationBaselineAdjustment = $00000008;
>>>>>>> graemeg/fixes_2_2
=======
  kATSULayoutOperationBaselineAdjustment = $00000008;
>>>>>>> origin/fixes_2_2
=======
	kATSULayoutOperationBaselineAdjustment = $00000008;
>>>>>>> origin/fixes_2.4

  {
   * Select the tracking adjustment operation.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSULayoutOperationTrackingAdjustment = $00000010;
=======
  kATSULayoutOperationTrackingAdjustment = $00000010;
>>>>>>> graemeg/fixes_2_2
=======
  kATSULayoutOperationTrackingAdjustment = $00000010;
>>>>>>> origin/fixes_2_2
=======
	kATSULayoutOperationTrackingAdjustment = $00000010;
>>>>>>> origin/fixes_2.4

  {
   * Select the period of time after ATSUI has finished all of it's
   * layout operations.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSULayoutOperationPostLayoutAdjustment = $00000020;
	kATSULayoutOperationAppleReserved = $FFFFFFC0;
=======
  kATSULayoutOperationPostLayoutAdjustment = $00000020;
  kATSULayoutOperationAppleReserved = $FFFFFFC0;
>>>>>>> graemeg/fixes_2_2
=======
  kATSULayoutOperationPostLayoutAdjustment = $00000020;
  kATSULayoutOperationAppleReserved = $FFFFFFC0;
>>>>>>> origin/fixes_2_2
=======
	kATSULayoutOperationPostLayoutAdjustment = $00000020;
	kATSULayoutOperationAppleReserved = $FFFFFFC0;
>>>>>>> origin/fixes_2.4

{ ---------------------------------------------------------------------------- }

{
 *  ATSULayoutOperationCallbackStatus
 *  
 *  Summary:
 *    One of these must be returned by a
 *    ATSUDLayoutOperationOverrideUPP callback function in order to
 *    indicate ATSUI's status.
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
	ATSULayoutOperationCallbackStatus = UInt32;
const
{
=======
=======
>>>>>>> origin/fixes_2_2
type ATSULayoutOperationCallbackStatus = UInt32;
const

  {
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
type
	ATSULayoutOperationCallbackStatus = UInt32;
const
{
>>>>>>> origin/fixes_2.4
   * Return this if the callback function has totally handled the
   * operation which triggered the callback and does not need ATSUI to
   * run any further processing for the operation.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSULayoutOperationCallbackStatusHandled = $00000000;
=======
  kATSULayoutOperationCallbackStatusHandled = $00000000;
>>>>>>> graemeg/fixes_2_2
=======
  kATSULayoutOperationCallbackStatusHandled = $00000000;
>>>>>>> origin/fixes_2_2
=======
	kATSULayoutOperationCallbackStatusHandled = $00000000;
>>>>>>> origin/fixes_2.4

  {
   * Return this if the callback function has not totally handled the
   * operation which triggered the callback and needs ATSUI to run it's
   * own processing.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSULayoutOperationCallbackStatusContinue = $00000001;
=======
  kATSULayoutOperationCallbackStatusContinue = $00000001;

>>>>>>> graemeg/fixes_2_2
=======
  kATSULayoutOperationCallbackStatusContinue = $00000001;

>>>>>>> origin/fixes_2_2
=======
	kATSULayoutOperationCallbackStatusContinue = $00000001;
>>>>>>> origin/fixes_2.4

{ ---------------------------------------------------------------------------- }

{
 *  ATSLineLayoutOptions
 *  
 *  Summary:
 *    ATSLineLayoutOptions are normally set in an ATSUTextLayout object
 *    via the kATSULineLayoutOptionsTag layout control attribute. They
 *    can also be set in an ATSLineLayoutParams structure.
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
	ATSLineLayoutOptions = UInt32;
const
{
   * No options specified.
   }
	kATSLineNoLayoutOptions = $00000000;
=======
=======
>>>>>>> origin/fixes_2_2
type ATSLineLayoutOptions = UInt32;
=======
type
	ATSLineLayoutOptions = UInt32;
>>>>>>> origin/fixes_2.4
const
{
   * No options specified.
   }
<<<<<<< HEAD
  kATSLineNoLayoutOptions       = $00000000;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kATSLineNoLayoutOptions = $00000000;
>>>>>>> origin/fixes_2.4

  {
   * This line option is no longer used.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSLineIsDisplayOnly = $00000001; { obsolete option}
=======
  kATSLineIsDisplayOnly         = $00000001; { obsolete option }
>>>>>>> graemeg/fixes_2_2
=======
  kATSLineIsDisplayOnly         = $00000001; { obsolete option }
>>>>>>> origin/fixes_2_2
=======
	kATSLineIsDisplayOnly = $00000001; { obsolete option}
>>>>>>> origin/fixes_2.4

  {
   * Specifies that no hangers to be formed on the line.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSLineHasNoHangers = $00000002;
=======
  kATSLineHasNoHangers          = $00000002;
>>>>>>> graemeg/fixes_2_2
=======
  kATSLineHasNoHangers          = $00000002;
>>>>>>> origin/fixes_2_2
=======
	kATSLineHasNoHangers = $00000002;
>>>>>>> origin/fixes_2.4

  {
   * Specifies that no optical alignment to be performed on the line.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSLineHasNoOpticalAlignment = $00000004;
=======
  kATSLineHasNoOpticalAlignment = $00000004;
>>>>>>> graemeg/fixes_2_2
=======
  kATSLineHasNoOpticalAlignment = $00000004;
>>>>>>> origin/fixes_2_2
=======
	kATSLineHasNoOpticalAlignment = $00000004;
>>>>>>> origin/fixes_2.4

  {
   * Specifies that space charcters should not be treated as hangers.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSLineKeepSpacesOutOfMargin = $00000008;
=======
  kATSLineKeepSpacesOutOfMargin = $00000008;
>>>>>>> graemeg/fixes_2_2
=======
  kATSLineKeepSpacesOutOfMargin = $00000008;
>>>>>>> origin/fixes_2_2
=======
	kATSLineKeepSpacesOutOfMargin = $00000008;
>>>>>>> origin/fixes_2.4

  {
   * Specifies no post-compensation justification is to be performed.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSLineNoSpecialJustification = $00000010;
=======
  kATSLineNoSpecialJustification = $00000010;
>>>>>>> graemeg/fixes_2_2
=======
  kATSLineNoSpecialJustification = $00000010;
>>>>>>> origin/fixes_2_2
=======
	kATSLineNoSpecialJustification = $00000010;
>>>>>>> origin/fixes_2.4

  {
   * Specifies that if the line is the last of a paragraph, it will not
   * get justified.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSLineLastNoJustification = $00000020;
=======
  kATSLineLastNoJustification   = $00000020;
>>>>>>> graemeg/fixes_2_2
=======
  kATSLineLastNoJustification   = $00000020;
>>>>>>> origin/fixes_2_2
=======
	kATSLineLastNoJustification = $00000020;
>>>>>>> origin/fixes_2.4

  {
   * Specifies that the displayed line glyphs will adjust for device
   * metrics.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSLineFractDisable = $00000040;
=======
  kATSLineFractDisable          = $00000040;
>>>>>>> graemeg/fixes_2_2
=======
  kATSLineFractDisable          = $00000040;
>>>>>>> origin/fixes_2_2
=======
	kATSLineFractDisable = $00000040;
>>>>>>> origin/fixes_2.4

  {
   * Specifies that the carets at the ends of the line will be
   * guarenteed to be perpendicular to the baseline.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSLineImposeNoAngleForEnds = $00000080;
=======
  kATSLineImposeNoAngleForEnds  = $00000080;
>>>>>>> graemeg/fixes_2_2
=======
  kATSLineImposeNoAngleForEnds  = $00000080;
>>>>>>> origin/fixes_2_2
=======
	kATSLineImposeNoAngleForEnds = $00000080;
>>>>>>> origin/fixes_2.4

  {
   * Highlights for the line end characters will be extended to 0 and
   * the specified line width.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSLineFillOutToWidth = $00000100;
=======
  kATSLineFillOutToWidth        = $00000100;
>>>>>>> graemeg/fixes_2_2
=======
  kATSLineFillOutToWidth        = $00000100;
>>>>>>> origin/fixes_2_2
=======
	kATSLineFillOutToWidth = $00000100;
>>>>>>> origin/fixes_2.4

  {
   * Specifies that the tab character width will be automatically
   * adjusted to fit the specified line width.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSLineTabAdjustEnabled = $00000200;
=======
  kATSLineTabAdjustEnabled      = $00000200;
>>>>>>> graemeg/fixes_2_2
=======
  kATSLineTabAdjustEnabled      = $00000200;
>>>>>>> origin/fixes_2_2
=======
	kATSLineTabAdjustEnabled = $00000200;
>>>>>>> origin/fixes_2.4

  {
   * Specifies that any leading value specified by a font will be
   * ignored.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSLineIgnoreFontLeading = $00000400;

  {
   * Specifies that ATS produce antialiased glyph images despite system
   * preferences (negates kATSLineNoAntiAliasing bit if set).
   }
	kATSLineApplyAntiAliasing = $00000800;

  {
   * Specifies that ATS turn-off antialiasing glyph imaging despite
   * system preferences (negates kATSLineApplyAntiAliasing bit if set).
   }
	kATSLineNoAntiAliasing = $00001000;
=======
=======
>>>>>>> origin/fixes_2_2
  kATSLineIgnoreFontLeading     = $00000400;
=======
	kATSLineIgnoreFontLeading = $00000400;
>>>>>>> origin/fixes_2.4

  {
   * Specifies that ATS produce antialiased glyph images despite system
   * preferences (negates kATSLineNoAntiAliasing bit if set).
   }
	kATSLineApplyAntiAliasing = $00000800;

  {
   * Specifies that ATS turn-off antialiasing glyph imaging despite
   * system preferences (negates kATSLineApplyAntiAliasing bit if set).
   }
<<<<<<< HEAD
  kATSLineNoAntiAliasing        = $00001000;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kATSLineNoAntiAliasing = $00001000;
>>>>>>> origin/fixes_2.4

  {
   * Specifies that if the line width is not sufficient to hold all its
   * glyphs, glyph positions are allowed to extend beyond the line's
   * assigned width so negative justification is not used.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSLineDisableNegativeJustification = $00002000;
=======
  kATSLineDisableNegativeJustification = $00002000;
>>>>>>> graemeg/fixes_2_2
=======
  kATSLineDisableNegativeJustification = $00002000;
>>>>>>> origin/fixes_2_2
=======
	kATSLineDisableNegativeJustification = $00002000;
>>>>>>> origin/fixes_2.4

  {
   * Specifies that lines with any integer glyph positioning (due to
   * either any character non-antialiased or kATSLineFractDisable
   * specified), not automatically esthetically adjust individual
   * character positions while rendering to display.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSLineDisableAutoAdjustDisplayPos = $00004000;

  {
   * Specifies that rendering be done simulating Quickdraw rendering
   * (4-bit pixel aligned antialiasing).
   }
	kATSLineUseQDRendering = $00008000;
=======
=======
>>>>>>> origin/fixes_2_2
  kATSLineDisableAutoAdjustDisplayPos = $00004000;
=======
	kATSLineDisableAutoAdjustDisplayPos = $00004000;
>>>>>>> origin/fixes_2.4

  {
   * Specifies that rendering be done simulating Quickdraw rendering
   * (4-bit pixel aligned antialiasing).
   }
<<<<<<< HEAD
  kATSLineUseQDRendering        = $00008000;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kATSLineUseQDRendering = $00008000;
>>>>>>> origin/fixes_2.4

  {
   * Specifies that any Justification operations will not be run.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSLineDisableAllJustification = $00010000;
=======
  kATSLineDisableAllJustification = $00010000;
>>>>>>> graemeg/fixes_2_2
=======
  kATSLineDisableAllJustification = $00010000;
>>>>>>> origin/fixes_2_2
=======
	kATSLineDisableAllJustification = $00010000;
>>>>>>> origin/fixes_2.4

  {
   * Specifies that any glyph morphing operations will not be run.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSLineDisableAllGlyphMorphing = $00020000;
=======
  kATSLineDisableAllGlyphMorphing = $00020000;
>>>>>>> graemeg/fixes_2_2
=======
  kATSLineDisableAllGlyphMorphing = $00020000;
>>>>>>> origin/fixes_2_2
=======
	kATSLineDisableAllGlyphMorphing = $00020000;
>>>>>>> origin/fixes_2.4

  {
   * Specifies that any kerning adjustment operations will not be run.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSLineDisableAllKerningAdjustments = $00040000;
=======
  kATSLineDisableAllKerningAdjustments = $00040000;
>>>>>>> graemeg/fixes_2_2
=======
  kATSLineDisableAllKerningAdjustments = $00040000;
>>>>>>> origin/fixes_2_2
=======
	kATSLineDisableAllKerningAdjustments = $00040000;
>>>>>>> origin/fixes_2.4

  {
   * Specifies that any baseline adjustment operations will not be run.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSLineDisableAllBaselineAdjustments = $00080000;
=======
  kATSLineDisableAllBaselineAdjustments = $00080000;
>>>>>>> graemeg/fixes_2_2
=======
  kATSLineDisableAllBaselineAdjustments = $00080000;
>>>>>>> origin/fixes_2_2
=======
	kATSLineDisableAllBaselineAdjustments = $00080000;
>>>>>>> origin/fixes_2.4

  {
   * Specifies that any tracking adjustment operations will not be run.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSLineDisableAllTrackingAdjustments = $00100000;
=======
  kATSLineDisableAllTrackingAdjustments = $00100000;
>>>>>>> graemeg/fixes_2_2
=======
  kATSLineDisableAllTrackingAdjustments = $00100000;
>>>>>>> origin/fixes_2_2
=======
	kATSLineDisableAllTrackingAdjustments = $00100000;
>>>>>>> origin/fixes_2.4

  {
   * Convenience constant for turning-off all adjustments.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSLineDisableAllLayoutOperations = kATSLineDisableAllJustification or kATSLineDisableAllGlyphMorphing or kATSLineDisableAllKerningAdjustments or kATSLineDisableAllBaselineAdjustments or kATSLineDisableAllTrackingAdjustments;
=======
  kATSLineDisableAllLayoutOperations = kATSLineDisableAllJustification or kATSLineDisableAllGlyphMorphing or kATSLineDisableAllKerningAdjustments or kATSLineDisableAllBaselineAdjustments or kATSLineDisableAllTrackingAdjustments;
>>>>>>> graemeg/fixes_2_2
=======
  kATSLineDisableAllLayoutOperations = kATSLineDisableAllJustification or kATSLineDisableAllGlyphMorphing or kATSLineDisableAllKerningAdjustments or kATSLineDisableAllBaselineAdjustments or kATSLineDisableAllTrackingAdjustments;
>>>>>>> origin/fixes_2_2
=======
	kATSLineDisableAllLayoutOperations = kATSLineDisableAllJustification or kATSLineDisableAllGlyphMorphing or kATSLineDisableAllKerningAdjustments or kATSLineDisableAllBaselineAdjustments or kATSLineDisableAllTrackingAdjustments;
>>>>>>> origin/fixes_2.4

  {
   * Specifies to optimize for displaying text only.  Note, rounded
   * device metrics will be used instead of fractional path metrics.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSLineUseDeviceMetrics = $01000000;
=======
  kATSLineUseDeviceMetrics      = $01000000;
>>>>>>> graemeg/fixes_2_2
=======
  kATSLineUseDeviceMetrics      = $01000000;
>>>>>>> origin/fixes_2_2
=======
	kATSLineUseDeviceMetrics = $01000000;
>>>>>>> origin/fixes_2.4

  {
   * Specifies that line breaking should occur at the nearest
   * character, not word.  This could cause a word to be split among
   * multiple lines.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSLineBreakToNearestCharacter = $02000000;
=======
  kATSLineBreakToNearestCharacter = $02000000;
>>>>>>> graemeg/fixes_2_2
=======
  kATSLineBreakToNearestCharacter = $02000000;
>>>>>>> origin/fixes_2_2
=======
	kATSLineBreakToNearestCharacter = $02000000;
>>>>>>> origin/fixes_2.4

  {
   * These bits are reserved by Apple and will result in a invalid
   * value error if attemped to set. Obsolete constants:
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSLineAppleReserved = $FCE00000;
=======
  kATSLineAppleReserved         = $FCE00000;

>>>>>>> graemeg/fixes_2_2
=======
  kATSLineAppleReserved         = $FCE00000;

>>>>>>> origin/fixes_2_2
=======
	kATSLineAppleReserved = $FCE00000;
>>>>>>> origin/fixes_2.4

{ --------------------------------------------------------------------------- }

{
 *  ATSStyleRenderingOptions
 *  
 *  Summary:
 *    ATSStyleRenderingOptions are set in the ATSUStyle object via the
 *    attribute tag kATSUStyleRenderingOptions. They provide finer
 *    control over how the style is rendered.
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
	ATSStyleRenderingOptions = UInt32;
const
{
   * No options specified.
   }
	kATSStyleNoOptions = $00000000;
=======
=======
>>>>>>> origin/fixes_2_2
type ATSStyleRenderingOptions = UInt32;
=======
type
	ATSStyleRenderingOptions = UInt32;
>>>>>>> origin/fixes_2.4
const
{
   * No options specified.
   }
<<<<<<< HEAD
  kATSStyleNoOptions            = $00000000;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kATSStyleNoOptions = $00000000;
>>>>>>> origin/fixes_2.4

  {
   * Specifies that ATS produce "unhinted" glyph outlines (default is
   * hinted glyph outlines).
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSStyleNoHinting = $00000001;

  {
   * Specifies that ATS produce antialiased glyph images despite system
   * preferences or LineLayoutOptions (negates kATSStyleNoAntiAliasing
   * bit if set).
   }
	kATSStyleApplyAntiAliasing = $00000002;

  {
   * Specifies that ATS turn-off antialiasing glyph imaging despite
   * system preferences or LineLayoutOptions (negates
   * kATSStyleApplyAntiAliasing bit if set).
   }
	kATSStyleNoAntiAliasing = $00000004;
=======
=======
>>>>>>> origin/fixes_2_2
  kATSStyleNoHinting            = $00000001;
=======
	kATSStyleNoHinting = $00000001;
>>>>>>> origin/fixes_2.4

  {
   * Specifies that ATS produce antialiased glyph images despite system
   * preferences or LineLayoutOptions (negates kATSStyleNoAntiAliasing
   * bit if set).
   }
	kATSStyleApplyAntiAliasing = $00000002;

  {
   * Specifies that ATS turn-off antialiasing glyph imaging despite
   * system preferences or LineLayoutOptions (negates
   * kATSStyleApplyAntiAliasing bit if set).
   }
<<<<<<< HEAD
  kATSStyleNoAntiAliasing       = $00000004;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kATSStyleNoAntiAliasing = $00000004;
>>>>>>> origin/fixes_2.4

  {
   * These bits are reserved by Apple and will result in a invalid
   * value error if attemped to set.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSStyleAppleReserved = $FFFFFFF8;
=======
  kATSStyleAppleReserved        = $FFFFFFF8;
>>>>>>> graemeg/fixes_2_2
=======
  kATSStyleAppleReserved        = $FFFFFFF8;
>>>>>>> origin/fixes_2_2
=======
	kATSStyleAppleReserved = $FFFFFFF8;
>>>>>>> origin/fixes_2.4

  {
   * (OBSOLETE) Specifies that ATS produce "hinted" glyph outlines (the
   * default behavior). THIS NAME IS OBSOLETE. DO NOT USE. It's only
   * left in for backwards compatibility.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
	kATSStyleApplyHints = kATSStyleNoOptions;

{ --------------------------------------------------------------------------- }

<<<<<<< HEAD
=======
  kATSStyleApplyHints           = kATSStyleNoOptions;
>>>>>>> graemeg/fixes_2_2
=======
  kATSStyleApplyHints           = kATSStyleNoOptions;
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{
 *  ATSGlyphInfoFlags
 *  
 *  Summary:
 *    ATSGlyphInfoFlags are set in the individual ATSLayoutRecord
 *    structures and apply only to the ATSGlyphRef in that structure.
 *    The are used by the layout engine to flag a glyph with specific
 *    properties.
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
	ATSGlyphInfoFlags = UInt32;
const
{
   * These bits are Apple reserved and may result in an invalid value
   * error if attempted to set.
   }
	kATSGlyphInfoAppleReserved = $1FFBFFE8;
=======
=======
>>>>>>> origin/fixes_2_2
type ATSGlyphInfoFlags = UInt32;
=======
type
	ATSGlyphInfoFlags = UInt32;
>>>>>>> origin/fixes_2.4
const
{
   * These bits are Apple reserved and may result in an invalid value
   * error if attempted to set.
   }
<<<<<<< HEAD
  kATSGlyphInfoAppleReserved    = $1FFBFFE8;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
	kATSGlyphInfoAppleReserved = $1FFBFFE8;
>>>>>>> origin/fixes_2.4

  {
   * The glyph attaches to another glyph.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSGlyphInfoIsAttachment = $80000000;
=======
  kATSGlyphInfoIsAttachment     = $80000000;
>>>>>>> graemeg/fixes_2_2
=======
  kATSGlyphInfoIsAttachment     = $80000000;
>>>>>>> origin/fixes_2_2
=======
	kATSGlyphInfoIsAttachment = $80000000;
>>>>>>> origin/fixes_2.4

  {
   * The glyph can hang off left/top edge of line.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSGlyphInfoIsLTHanger = $40000000;
=======
  kATSGlyphInfoIsLTHanger       = $40000000;
>>>>>>> graemeg/fixes_2_2
=======
  kATSGlyphInfoIsLTHanger       = $40000000;
>>>>>>> origin/fixes_2_2
=======
	kATSGlyphInfoIsLTHanger = $40000000;
>>>>>>> origin/fixes_2.4

  {
   * The glyph can hang off right/bottom edge of line.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSGlyphInfoIsRBHanger = $20000000;
=======
  kATSGlyphInfoIsRBHanger       = $20000000;
>>>>>>> graemeg/fixes_2_2
=======
  kATSGlyphInfoIsRBHanger       = $20000000;
>>>>>>> origin/fixes_2_2
=======
	kATSGlyphInfoIsRBHanger = $20000000;
>>>>>>> origin/fixes_2.4

  {
   * The glyph is not really a glyph at all, but an end-marker designed
   * to allow the calculation of the previous glyph's advance.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSGlyphInfoTerminatorGlyph = $00080000;
=======
  kATSGlyphInfoTerminatorGlyph  = $00080000;
>>>>>>> graemeg/fixes_2_2
=======
  kATSGlyphInfoTerminatorGlyph  = $00080000;
>>>>>>> origin/fixes_2_2
=======
	kATSGlyphInfoTerminatorGlyph = $00080000;
>>>>>>> origin/fixes_2.4

  {
   * The glyph is a white space glyph.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSGlyphInfoIsWhiteSpace = $00040000;
=======
  kATSGlyphInfoIsWhiteSpace     = $00040000;
>>>>>>> graemeg/fixes_2_2
=======
  kATSGlyphInfoIsWhiteSpace     = $00040000;
>>>>>>> origin/fixes_2_2
=======
	kATSGlyphInfoIsWhiteSpace = $00040000;
>>>>>>> origin/fixes_2.4

  {
   * Glyph has a style specified imposed width (i.e. advance)
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSGlyphInfoHasImposedWidth = $00000010;
=======
  kATSGlyphInfoHasImposedWidth  = $00000010;
>>>>>>> graemeg/fixes_2_2
=======
  kATSGlyphInfoHasImposedWidth  = $00000010;
>>>>>>> origin/fixes_2_2
=======
	kATSGlyphInfoHasImposedWidth = $00000010;
>>>>>>> origin/fixes_2.4

  {
   * A three-bit mask, that can be used to get the size of the original
   * character that spawned this glyph. When a logical 'and' operation
   * with this mask and an ATSGlyphInfoFlags variable, it will yield
   * the size in bytes of the original character (0 - 7 bytes possible).
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSGlyphInfoByteSizeMask = $00000007;

{ --------------------------------------------------------------------------- }
{ STRUCTURED TYPES and related constants }
{ --------------------------------------------------------------------------- }

{
 *  ATSLayoutRecord
 *  
 *  Summary:
 *    The ATSLayoutRecord structure defines all the needed info for a
 *    single glyph during the layout process.  This struct must be
 *    declared as the first element of an enclosing glyph record struct
 *    defined by ATSUI DirectAccess clients.
 }
type
	ATSLayoutRecord = record
{
   * The glyph ID reference.
   }
		glyphID: ATSGlyphRef;

  {
   * These flags describe the individual state of the glyph (see above).
   }
		flags: ATSGlyphInfoFlags;

  {
   * The byte offset of the original character that spawned this glyph.
   }
		originalOffset: ByteCount;

  {
   * This is the real position that the glyph sits.
   }
		realPos: Fixed;
	end;
	ATSLayoutRecordPtr = ^ATSLayoutRecord;
	
{ --------------------------------------------------------------------------- }

{
 *  ATSTrapezoid
 *  
 *  Summary:
 *    The ATSTrapezoid structure supplies a convenient container for
 *    glyph bounds in trapezoidal form.
 }
type
	ATSTrapezoid = record
		upperLeft: FixedPoint;
		upperRight: FixedPoint;
		lowerRight: FixedPoint;
		lowerLeft: FixedPoint;
	end;
	ATSTrapezoidPtr = ^ATSTrapezoid;
	
{ --------------------------------------------------------------------------- }

{
 *  ATSJustWidthDeltaEntryOverride
 *  
 *  Summary:
 *    The JustWidthDeltaEntryOverride structure specifies values for
 *    the grow and shrink case during justification, both on the left
 *    and on the right. It also contains flags.  This particular
 *    structure is used for passing justification overrides to LLC. For
 *    further sfnt resource 'just' table constants and structures, see
 *    SFNTLayoutTypes.h.
 }
type
	ATSJustWidthDeltaEntryOverride = record
{
   * ems AW can grow by at most on LT
   }
		beforeGrowLimit: Fixed;

  {
   * ems AW can shrink by at most on LT
   }
		beforeShrinkLimit: Fixed;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

  {
   * ems AW can grow by at most on RB
   }
		afterGrowLimit: Fixed;
<<<<<<< HEAD

  {
   * ems AW can shrink by at most on RB
   }
		afterShrinkLimit: Fixed;
<<<<<<< HEAD
<<<<<<< HEAD

  {
   * flags controlling grow case
   }
		growFlags: JustificationFlags;

  {
=======

  {
   * ems AW can grow by at most on RB
   }
		afterGrowLimit: Fixed;

  {
   * ems AW can shrink by at most on RB
   }
		afterShrinkLimit: Fixed;

  {
   * flags controlling grow case
   }
		growFlags: JustificationFlags;

  {
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> graemeg/cpstrnew

  {
   * flags controlling grow case
   }
		growFlags: JustificationFlags;

  {
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======

  {
   * ems AW can shrink by at most on RB
   }
		afterShrinkLimit: Fixed;

  {
   * flags controlling grow case
   }
		growFlags: JustificationFlags;

  {
>>>>>>> origin/cpstrnew
   * flags controlling shrink case
   }
		shrinkFlags: JustificationFlags;
	end;
	ATSJustWidthDeltaEntryOverridePtr = ^ATSJustWidthDeltaEntryOverride;
	
{ The JustPriorityOverrides type is an array of 4 width delta records, one per priority level override. }
	ATSJustPriorityWidthDeltaOverrides = array[0..3] of ATSJustWidthDeltaEntryOverride;
	
{ ---------------------------------------------------------------------------- }
=======
=======
>>>>>>> origin/fixes_2_2
  kATSGlyphInfoByteSizeMask     = $00000007;

=======
	kATSGlyphInfoByteSizeMask = $00000007;
>>>>>>> origin/fixes_2.4

{ --------------------------------------------------------------------------- }
{ STRUCTURED TYPES and related constants }
{ --------------------------------------------------------------------------- }

{
 *  ATSLayoutRecord
 *  
 *  Summary:
 *    The ATSLayoutRecord structure defines all the needed info for a
 *    single glyph during the layout process.  This struct must be
 *    declared as the first element of an enclosing glyph record struct
 *    defined by ATSUI DirectAccess clients.
 }
type
	ATSLayoutRecord = record
{
   * The glyph ID reference.
   }
		glyphID: ATSGlyphRef;

  {
   * These flags describe the individual state of the glyph (see above).
   }
		flags: ATSGlyphInfoFlags;

  {
   * The byte offset of the original character that spawned this glyph.
   }
		originalOffset: ByteCount;

  {
   * This is the real position that the glyph sits.
   }
		realPos: Fixed;
	end;
	ATSLayoutRecordPtr = ^ATSLayoutRecord;
	
{ --------------------------------------------------------------------------- }

{
 *  ATSTrapezoid
 *  
 *  Summary:
 *    The ATSTrapezoid structure supplies a convenient container for
 *    glyph bounds in trapezoidal form.
 }
type
	ATSTrapezoid = record
		upperLeft: FixedPoint;
		upperRight: FixedPoint;
		lowerRight: FixedPoint;
		lowerLeft: FixedPoint;
	end;
	ATSTrapezoidPtr = ^ATSTrapezoid;
	
{ --------------------------------------------------------------------------- }

{
 *  ATSJustWidthDeltaEntryOverride
 *  
 *  Summary:
 *    The JustWidthDeltaEntryOverride structure specifies values for
 *    the grow and shrink case during justification, both on the left
 *    and on the right. It also contains flags.  This particular
 *    structure is used for passing justification overrides to LLC. For
 *    further sfnt resource 'just' table constants and structures, see
 *    SFNTLayoutTypes.h.
 }
type
	ATSJustWidthDeltaEntryOverride = record
{
   * ems AW can grow by at most on LT
   }
		beforeGrowLimit: Fixed;

  {
   * ems AW can shrink by at most on LT
   }
		beforeShrinkLimit: Fixed;

  {
   * ems AW can grow by at most on RB
   }
		afterGrowLimit: Fixed;

  {
   * ems AW can shrink by at most on RB
   }
		afterShrinkLimit: Fixed;

  {
   * flags controlling grow case
   }
		growFlags: JustificationFlags;

<<<<<<< HEAD
	{	 The JustPriorityOverrides type is an array of 4 width delta records, one per priority level override. 	}
	ATSJustPriorityWidthDeltaOverrides	= array [0..3] of ATSJustWidthDeltaEntryOverride;
	{	 --------------------------------------------------------------------------- 	}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
  {
   * flags controlling shrink case
   }
		shrinkFlags: JustificationFlags;
	end;
	ATSJustWidthDeltaEntryOverridePtr = ^ATSJustWidthDeltaEntryOverride;
	
{ The JustPriorityOverrides type is an array of 4 width delta records, one per priority level override. }
	ATSJustPriorityWidthDeltaOverrides = array[0..3] of ATSJustWidthDeltaEntryOverride;
	
{ ---------------------------------------------------------------------------- }
>>>>>>> origin/fixes_2.4

{
 *  ATSULineRef
 *  
 *  Summary:
 *    A reference to a line that is being laid out. This is passed into
 *    the ATSUDirectLayoutOperationOverrideUPP callback function to be
 *    used by the ATSUDirectGetLayoutDataArrayPtrFromLineRef function.
 *    The only way to get a line ref is inside of the callback. The
 *    only time the line ref is valid is inside of the callback.
 }
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	ATSGlyphVector = record end;
	ATSULineRef = ^ATSGlyphVector;
=======
	ATSULineRef    = ^SInt32; { an opaque 32-bit type }
>>>>>>> graemeg/fixes_2_2
=======
	ATSULineRef    = ^SInt32; { an opaque 32-bit type }
>>>>>>> origin/fixes_2_2
=======
	ATSGlyphVector = record end;
	ATSULineRef = ^ATSGlyphVector;
>>>>>>> origin/fixes_2.4
{ ---------------------------------------------------------------------------- }
{ DirectAccess Layout Callback Definitions                                     }
{ ---------------------------------------------------------------------------- }

{
 *  ATSUDirectLayoutOperationOverrideProcPtr
 *  
 *  Summary:
 *    Callback definition for a low-level adjustment routine hook.
 *  
 *  Discussion:
 *    This callback can be set in an ATSUTextLayout object by setting
 *    the attribute tag kATSULayoutOperationOverrideTag and passing in
 *    a ATSULayoutOperationOverrideSpecifier structure into
 *    ATSUSetLayoutAttribute. This callback will be called whenever an
 *    ATSUI call triggers a re-layout for each operation it is
 *    installed for. The operation that triggered the callback will be
 *    set in the iCurrentOperation parameter. The callback function
 *    defined by the developer is only required to do one thing: return
 *    it's status to ATSUI as to what it has done. This is done via the
 *    oCallbackStatus parameter. It needs to tell ATSUI if it had
 *    handled the layout operation or if it still needs ATSUI to run
 *    it's own processes. iOperationCallbackParameterPtr is there in
 *    case there are ever any ATSUDirectLayoutOperationSelector which
 *    require extra parameters to be passed into the callback function.
 *    It is currently unused and will always be set to NULL. iRefCon is
 *    the constant that is set in the ATSUTextLayout object that
 *    spawned the operation by the ATSUSetTextLayoutRefCon() API.
 *    Within the context of the callback itself, only a limited subset
 *    of ATSUI APIs may be called. Basically, only the APIs that have
 *    no chance of triggering a re-layout are allowed to be called. The
 *    reason for this restriction is to prevent runaway recursion. Most
 *    of the APIs that have "create", "get", or "copy" are safe. Any
 *    attempt to call one of the restricted APIs will result in an
 *    immediate return with the kATSUInvalidCallInsideCallbackErr
 *    error. ATSULayoutOperationSelector and
 *    ATSULayoutOperationCallbackStatus are defined in ATSLayoutTypes.i.
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
type
	ATSUDirectLayoutOperationOverrideProcPtr = function( iCurrentOperation: ATSULayoutOperationSelector; iLineRef: ATSULineRef; iRefCon: URefCon; iOperationCallbackParameterPtr: UnivPtr; var oCallbackStatus: ATSULayoutOperationCallbackStatus ): OSStatus;
	ATSUDirectLayoutOperationOverrideUPP = ATSUDirectLayoutOperationOverrideProcPtr;

<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
type ATSUDirectLayoutOperationOverrideProcPtr = function( iCurrentOperation: ATSULayoutOperationSelector; iLineRef: ATSULineRef; iRefCon: UInt32; iOperationCallbackParameterPtr: UnivPtr; var oCallbackStatus: ATSULayoutOperationCallbackStatus ): OSStatus;
// Beats me what this translates to.  If someone finds out they can tell me and we'll update it
// typedef STACK_UPP_TYPE(ATSUDirectLayoutOperationOverrideProcPtr)  ATSUDirectLayoutOperationOverrideUPP;
type ATSUDirectLayoutOperationOverrideUPP = Ptr;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{
 *  NewATSUDirectLayoutOperationOverrideUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function NewATSUDirectLayoutOperationOverrideUPP( userRoutine: ATSUDirectLayoutOperationOverrideProcPtr ): ATSUDirectLayoutOperationOverrideUPP; external name '_NewATSUDirectLayoutOperationOverrideUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
// AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER
function NewATSUDirectLayoutOperationOverrideUPP( userRoutine: ATSUDirectLayoutOperationOverrideProcPtr ): ATSUDirectLayoutOperationOverrideUPP; external name '_NewATSUDirectLayoutOperationOverrideUPP';
>>>>>>> graemeg/fixes_2_2
=======
// AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER
function NewATSUDirectLayoutOperationOverrideUPP( userRoutine: ATSUDirectLayoutOperationOverrideProcPtr ): ATSUDirectLayoutOperationOverrideUPP; external name '_NewATSUDirectLayoutOperationOverrideUPP';
>>>>>>> origin/fixes_2_2
=======
function NewATSUDirectLayoutOperationOverrideUPP( userRoutine: ATSUDirectLayoutOperationOverrideProcPtr ): ATSUDirectLayoutOperationOverrideUPP; external name '_NewATSUDirectLayoutOperationOverrideUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> origin/fixes_2.4

{
 *  DisposeATSUDirectLayoutOperationOverrideUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure DisposeATSUDirectLayoutOperationOverrideUPP( userUPP: ATSUDirectLayoutOperationOverrideUPP ); external name '_DisposeATSUDirectLayoutOperationOverrideUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
// AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER
procedure DisposeATSUDirectLayoutOperationOverrideUPP( userUPP: ATSUDirectLayoutOperationOverrideUPP ); external name '_DisposeATSUDirectLayoutOperationOverrideUPP';
>>>>>>> graemeg/fixes_2_2
=======
// AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER
procedure DisposeATSUDirectLayoutOperationOverrideUPP( userUPP: ATSUDirectLayoutOperationOverrideUPP ); external name '_DisposeATSUDirectLayoutOperationOverrideUPP';
>>>>>>> origin/fixes_2_2
=======
procedure DisposeATSUDirectLayoutOperationOverrideUPP( userUPP: ATSUDirectLayoutOperationOverrideUPP ); external name '_DisposeATSUDirectLayoutOperationOverrideUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> origin/fixes_2.4

{
 *  InvokeATSUDirectLayoutOperationOverrideUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function InvokeATSUDirectLayoutOperationOverrideUPP( iCurrentOperation: ATSULayoutOperationSelector; iLineRef: ATSULineRef; iRefCon: URefCon; iOperationCallbackParameterPtr: UnivPtr; var oCallbackStatus: ATSULayoutOperationCallbackStatus; userUPP: ATSUDirectLayoutOperationOverrideUPP ): OSStatus; external name '_InvokeATSUDirectLayoutOperationOverrideUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> origin/cpstrnew
=======
function InvokeATSUDirectLayoutOperationOverrideUPP( iCurrentOperation: ATSULayoutOperationSelector; iLineRef: ATSULineRef; iRefCon: URefCon; iOperationCallbackParameterPtr: UnivPtr; var oCallbackStatus: ATSULayoutOperationCallbackStatus; userUPP: ATSUDirectLayoutOperationOverrideUPP ): OSStatus; external name '_InvokeATSUDirectLayoutOperationOverrideUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> origin/fixes_2.4

{
#if __MACH__
    #define NewATSUDirectLayoutOperationOverrideUPP(userRoutine) ((ATSUDirectLayoutOperationOverrideUPP)userRoutine)
    #define DisposeATSUDirectLayoutOperationOverrideUPP(userUPP)
    #define InvokeATSUDirectLayoutOperationOverrideUPP(iCurrentOperation, iLineRef, iRefCon, iOperationCallbackParameterPtr, oCallbackStatus, userUPP) (*userUPP)(iCurrentOperation, iLineRef, iRefCon, iOperationCallbackParameterPtr, oCallbackStatus)
#endif
}
<<<<<<< HEAD
=======
// AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER
function InvokeATSUDirectLayoutOperationOverrideUPP( iCurrentOperation: ATSULayoutOperationSelector; iLineRef: ATSULineRef; iRefCon: UInt32; iOperationCallbackParameterPtr: UnivPtr; var oCallbackStatus: ATSULayoutOperationCallbackStatus; userUPP: ATSUDirectLayoutOperationOverrideUPP ): OSStatus; external name '_InvokeATSUDirectLayoutOperationOverrideUPP';
>>>>>>> graemeg/fixes_2_2
=======
// AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER
function InvokeATSUDirectLayoutOperationOverrideUPP( iCurrentOperation: ATSULayoutOperationSelector; iLineRef: ATSULineRef; iRefCon: UInt32; iOperationCallbackParameterPtr: UnivPtr; var oCallbackStatus: ATSULayoutOperationCallbackStatus; userUPP: ATSUDirectLayoutOperationOverrideUPP ): OSStatus; external name '_InvokeATSUDirectLayoutOperationOverrideUPP';
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{ ---------------------------------------------------------------------------- }

{
 *  ATSULayoutOperationOverrideSpecifier
 *  
 *  Summary:
 *    This structure is used to install a callback for one or more
 *    ATSUI operations. To do this, simply passed one of these
 *    structure into the ATSUSetLayoutControls call with the
 *    kATSULayoutOperationOverrideTag tag.
 }
type
	ATSULayoutOperationOverrideSpecifier = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{
   * A bitfield containing the selector for the operations in which the
   * callback will be installed for.
   }
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
			{
			 * A bitfield containing the selector for the operations in which the
			 * callback will be installed for.
			 }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
		operationSelector: ATSULayoutOperationSelector;
		overrideUPP: ATSUDirectLayoutOperationOverrideUPP;
	end;
	ATSULayoutOperationOverrideSpecifierPtr = ^ATSULayoutOperationOverrideSpecifier;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	
//#pragma pack(pop)

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
=======
	
//#pragma pack(pop)
>>>>>>> origin/fixes_2.4

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
