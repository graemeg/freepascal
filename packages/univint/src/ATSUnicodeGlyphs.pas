{
     File:       QD/ATSUnicodeGlyphs.h
 
     Contains:   ATSUI glyph handling functions.
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Version:    Quickdraw-285~150
=======
     Version:    Quickdraw-262~1
>>>>>>> graemeg/cpstrnew
=======
     Version:    Quickdraw-262~1
>>>>>>> graemeg/cpstrnew
=======
     Version:    Quickdraw-262~1
>>>>>>> graemeg/cpstrnew
=======
     Version:    Quickdraw-262~1
>>>>>>> origin/cpstrnew
 
     Copyright:  © 2003-2008 by Apple Inc. all rights reserved.
=======
     Version:    Quickdraw-150~1
 
     Copyright:  © 2003 by Apple Computer, Inc., all rights reserved.
>>>>>>> graemeg/fixes_2_2
=======
     Version:    Quickdraw-150~1
 
     Copyright:  © 2003 by Apple Computer, Inc., all rights reserved.
>>>>>>> origin/fixes_2_2
=======
     Version:    Quickdraw-262~1
 
     Copyright:  © 2003-2008 by Apple Inc. all rights reserved.
>>>>>>> origin/fixes_2.4
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}
<<<<<<< HEAD
{  Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, 2004 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
=======
{	  Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, 2004 }
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
                     http://www.freepascal.org/bugs.html
 
}
{	  Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, 2004 }
{	  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit ATSUnicodeGlyphs;
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
>>>>>>> origin/cpstrnew
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
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
=======
>>>>>>> graemeg/cpstrnew
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
>>>>>>> graemeg/cpstrnew
=======
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
>>>>>>> origin/cpstrnew
{$ifc defined(iphonesim)}
 	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := TRUE}
<<<<<<< HEAD
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
=======
>>>>>>> graemeg/cpstrnew
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
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
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
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
uses MacTypes,ATSUnicodeTypes,TextCommon,ATSTypes;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
{$ALIGN MAC68K}
>>>>>>> graemeg/fixes_2_2
=======
{$ALIGN MAC68K}
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{ ---------------------------------------------------------------------------- }
{ ATSUI glyph metrics                                                          }
{ ---------------------------------------------------------------------------- }


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc not TARGET_CPU_64}
{
 *  ATSUGlyphGetIdealMetrics()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontGetGlyphWithName,
 *    CTFontGetVerticalTranslationsForGlyphs,
 *    CTFontGetBoundingRectsForGlyphs, or CTFontGetAdvancesForGlyphs
 *    instead.
=======
{
 *  ATSUGlyphGetIdealMetrics()
>>>>>>> graemeg/fixes_2_2
=======
{
 *  ATSUGlyphGetIdealMetrics()
>>>>>>> origin/fixes_2_2
=======
{$ifc not TARGET_CPU_64}
{
 *  ATSUGlyphGetIdealMetrics()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontGetGlyphWithName,
 *    CTFontGetVerticalTranslationsForGlyphs,
 *    CTFontGetBoundingRectsForGlyphs, or CTFontGetAdvancesForGlyphs
 *    instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Obtains resolution-independent font metric information for glyphs
 *    associated with a given style object.
 *  
 *  Discussion:
 *    The advance width is the full horizontal width of the glyph as
 *    measured from its origin to the origin of the next glyph on the
 *    line, including the left-side and right-side bearings. For
 *    vertical text, the advance height is the sum of the top-side
 *    bearing, the bounding-box height, and the bottom-side bearing.
 *    You can call the ATSUGlyphGetIdealMetrics function to obtain an
 *    array of ATSGlyphIdealMetrics structures containing values for
 *    the specified glyphs' advance and side bearings.
 *    ATSUGlyphGetIdealMetrics can analyze both horizontal and vertical
 *    text, automatically producing the appropriate bearing values
 *    (oriented for width or height, respectively) for each. You should
 *    call ATSUGlyphGetIdealMetrics to obtain resolution-independent
 *    glyph metrics. To obtain device-adjusted (that is,
 *    resolution-dependent) glyph metrics, call the function
 *    ATSUGlyphGetScreenMetrics.
 *  
 *  Parameters:
 *    
 *    iATSUStyle:
 *      A style referring to a font you wish to obtain glyph metrics
 *      from.
 *    
 *    iNumOfGlyphs:
 *      The number of glyph IDs you are passing in to be examined. This
 *      value should be equal to the size of the array you are passing
 *      in for the iGlyphIDs parameter.
 *    
 *    iGlyphIDs:
 *      An array of glyph IDs referring to glyphs for which you wish to
 *      obtain metrics.
 *    
 *    iInputOffset:
 *      A ByteOffset value specifying the offset in bytes between glyph
 *      IDs in the iGlyphIDs array.
 *    
 *    oIdealMetrics:
 *      A pointer to memory you have allocated for an array of
 *      ATSGlyphIdealMetrics structures. On return, each structure
 *      contains advance and side-bearing values for a glyph. See
 *      ATSTypes.h for more information regarding the
 *      ATSGlyphIdealMetrics structure.
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 9.1 and later
 }
function ATSUGlyphGetIdealMetrics( iATSUStyle: ATSUStyle; iNumOfGlyphs: ItemCount; iGlyphIDs: {variable-size-array} GlyphIDPtr; iInputOffset: ByteOffset; oIdealMetrics: {variable-size-array} ATSGlyphIdealMetricsPtr ): OSStatus; external name '_ATSUGlyphGetIdealMetrics';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUGlyphGetScreenMetrics()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontGetBoundingBox, CTFontGetUnderlinePosition,
 *    CTFontGetUnderlineThickness, CTFontGetSlantAngle,
 *    CTFontGetCapHeight, or CTFontGetXHeight iinstead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 9.1 and later
 }
function ATSUGlyphGetIdealMetrics( iATSUStyle: ATSUStyle; iNumOfGlyphs: ItemCount; iGlyphIDs: {variable-size-array} GlyphIDPtr; iInputOffset: ByteOffset; oIdealMetrics: {variable-size-array} ATSGlyphIdealMetricsPtr ): OSStatus; external name '_ATSUGlyphGetIdealMetrics';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUGlyphGetScreenMetrics()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUGlyphGetScreenMetrics()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontGetBoundingBox, CTFontGetUnderlinePosition,
 *    CTFontGetUnderlineThickness, CTFontGetSlantAngle,
 *    CTFontGetCapHeight, or CTFontGetXHeight iinstead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Obtains device-adjusted font metric information for glyphs
 *    associated with a given style object.
 *  
 *  Discussion:
 *    You can call the ATSUGlyphGetScreenMetrics function to obtain an
 *    array of ATSGlyphScreenMetrics structures containing values for
 *    the specified glyphs' advance and side bearings, top left,
 *    height, and width. You should call ATSUGlyphGetScreenMetrics to
 *    obtain device-adjusted (that is, resolution-dependent) glyph
 *    metrics. To obtain resolution-independent glyph metrics, call the
 *    function ATSUGlyphGetIdealMetrics.
 *  
 *  Parameters:
 *    
 *    iATSUStyle:
 *      A style referring to a font you wish to obtain glyph metrics
 *      from.
 *    
 *    iNumOfGlyphs:
 *      The number of glyph IDs you are passing in to be examined. This
 *      value should be equal to the size of the array you are passing
 *      in for the iGlyphIDs parameter.
 *    
 *    iGlyphIDs:
 *      An array of glyph IDs referring to glyphs for which you wish to
 *      obtain metrics.
 *    
 *    iInputOffset:
 *      A ByteOffset value specifying the offset in bytes between glyph
 *      IDs in the iGlyphIDs array.
 *    
 *    iForcingAntiAlias:
 *      A Boolean value indicating whether anti-aliasing is forced for
 *      the style object.
 *    
 *    iAntiAliasSwitch:
 *      A Boolean value indicating whether anti-aliasing is currently
 *      on or off.
 *    
 *    oScreenMetrics:
 *      A pointer to memory you have allocated for an array of
 *      ATSGlyphScreenMetrics structures. On return, each structure
 *      contains advance and side-bearing values for a glyph. See
 *      ATSTypes.h for more information regarding the
 *      ATSGlyphScreenMetrics structure.
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 9.1 and later
 }
function ATSUGlyphGetScreenMetrics( iATSUStyle: ATSUStyle; iNumOfGlyphs: ItemCount; iGlyphIDs: {variable-size-array} GlyphIDPtr; iInputOffset: ByteOffset; iForcingAntiAlias: Boolean; iAntiAliasSwitch: Boolean; oScreenMetrics: {variable-size-array} ATSGlyphScreenMetricsPtr ): OSStatus; external name '_ATSUGlyphGetScreenMetrics';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 9.1 and later
 }
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function ATSUGlyphGetScreenMetrics( iATSUStyle: ATSUStyle; iNumOfGlyphs: ItemCount; iGlyphIDs: GlyphIDPtr; iInputOffset: ByteOffset; iForcingAntiAlias: Boolean; iAntiAliasSwitch: Boolean; oScreenMetrics: ATSGlyphScreenMetricsPtr ): OSStatus; external name '_ATSUGlyphGetScreenMetrics';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 9.1 and later
 }
function ATSUGlyphGetScreenMetrics( iATSUStyle: ATSUStyle; iNumOfGlyphs: ItemCount; iGlyphIDs: {variable-size-array} GlyphIDPtr; iInputOffset: ByteOffset; iForcingAntiAlias: Boolean; iAntiAliasSwitch: Boolean; oScreenMetrics: {variable-size-array} ATSGlyphScreenMetricsPtr ): OSStatus; external name '_ATSUGlyphGetScreenMetrics';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4


{ ---------------------------------------------------------------------------- }
{ ATSUI glyph curve access functions and callbacks                             }
{ ---------------------------------------------------------------------------- }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  ATSUGetNativeCurveType()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCreatePathForGlyph instead.
<<<<<<< HEAD
=======
 *  ATSUGetNativeCurveType()
>>>>>>> graemeg/fixes_2_2
=======
 *  ATSUGetNativeCurveType()
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Returns the native curve format for a specific font.
 *  
 *  Discussion:
 *    Use this function to decide whether to call
 *    ATSUGlyphGetQuadraticPaths or ATSUGlyphGetCubicPaths. Both
 *    functions will return curves for all valid ATSUI fonts, but if
 *    the curve type you request is not the native curve type of the
 *    font, the curves you get back will be mathematically converted,
 *    rather than native font data. See the definition of ATSCurveType
 *    in ATSTypes.h for possible return values from this function.
 *  
 *  Parameters:
 *    
 *    iATSUStyle:
 *      A style referencing a font for which you wish to obtain the
 *      native curve type.
 *    
 *    oCurveType:
 *      On return, a value indicating the native curve type of the font
 *      referenced by iATSUStyle. See the definition of ATSCurveType in
 *      ATSTypes.h for a list of possible return values for this
 *      parameter.
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 9.1 and later
 }
function ATSUGetNativeCurveType( iATSUStyle: ATSUStyle; var oCurveType: ATSCurveType ): OSStatus; external name '_ATSUGetNativeCurveType';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{$endc} {not TARGET_CPU_64}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 9.1 and later
 }
function ATSUGetNativeCurveType( iATSUStyle: ATSUStyle; var oCurveType: ATSCurveType ): OSStatus; external name '_ATSUGetNativeCurveType';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{$endc} {not TARGET_CPU_64}
>>>>>>> origin/fixes_2.4


{
 *  ATSQuadraticNewPathProcPtr
 *  
 *  Discussion:
 *    A pointer to a client supplied callback function for handling
 *    glyph curve drawing operations. This callback handles operations
 *    to start a new drawing path.
 *  
 *  Parameters:
 *    
 *    callBackDataPtr:
 *      A pointer to any application specific data that may have been
 *      passed to the callbacks through the iCallbackDataPtr parameter
 *      of the ATSUGlyphGetQuadraticPaths function.
 *  
 *  Result:
 *    Return status. Pass any errors you wish to propagate back to the
 *    original caller of ATSUGlyphGetQuadraticPaths through this return
 *    value. Note that any nonzero result from this callback will halt
 *    the curve drawing process.
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
	ATSQuadraticNewPathProcPtr = function( callBackDataPtr: UnivPtr ): OSStatus;
	ATSQuadraticNewPathUPP = ATSQuadraticNewPathProcPtr;
=======
=======
>>>>>>> origin/fixes_2_2
type ATSQuadraticNewPathProcPtr = function( callBackDataPtr: UnivPtr ): OSStatus;
// Beats me what this translates to.  If someone finds out they can tell me and we'll update it
// typedef STACK_UPP_TYPE(ATSQuadraticNewPathProcPtr)              ATSQuadraticNewPathUPP;
type ATSQuadraticNewPathUPP = Ptr;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
type
	ATSQuadraticNewPathProcPtr = function( callBackDataPtr: UnivPtr ): OSStatus;
	ATSQuadraticNewPathUPP = ATSQuadraticNewPathProcPtr;
>>>>>>> origin/fixes_2.4
{
 *  NewATSQuadraticNewPathUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function NewATSQuadraticNewPathUPP( userRoutine: ATSQuadraticNewPathProcPtr ): ATSQuadraticNewPathUPP; external name '_NewATSQuadraticNewPathUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function NewATSQuadraticNewPathUPP( userRoutine: ATSQuadraticNewPathProcPtr ): ATSQuadraticNewPathUPP; external name '_NewATSQuadraticNewPathUPP';
>>>>>>> graemeg/fixes_2_2
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function NewATSQuadraticNewPathUPP( userRoutine: ATSQuadraticNewPathProcPtr ): ATSQuadraticNewPathUPP; external name '_NewATSQuadraticNewPathUPP';
>>>>>>> origin/fixes_2_2
=======
function NewATSQuadraticNewPathUPP( userRoutine: ATSQuadraticNewPathProcPtr ): ATSQuadraticNewPathUPP; external name '_NewATSQuadraticNewPathUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4

{
 *  DisposeATSQuadraticNewPathUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure DisposeATSQuadraticNewPathUPP( userUPP: ATSQuadraticNewPathUPP ); external name '_DisposeATSQuadraticNewPathUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
procedure DisposeATSQuadraticNewPathUPP( userUPP: ATSQuadraticNewPathUPP ); external name '_DisposeATSQuadraticNewPathUPP';
>>>>>>> graemeg/fixes_2_2
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
procedure DisposeATSQuadraticNewPathUPP( userUPP: ATSQuadraticNewPathUPP ); external name '_DisposeATSQuadraticNewPathUPP';
>>>>>>> origin/fixes_2_2
=======
procedure DisposeATSQuadraticNewPathUPP( userUPP: ATSQuadraticNewPathUPP ); external name '_DisposeATSQuadraticNewPathUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4

{
 *  InvokeATSQuadraticNewPathUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function InvokeATSQuadraticNewPathUPP( callBackDataPtr: UnivPtr; userUPP: ATSQuadraticNewPathUPP ): OSStatus; external name '_InvokeATSQuadraticNewPathUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function InvokeATSQuadraticNewPathUPP( callBackDataPtr: UnivPtr; userUPP: ATSQuadraticNewPathUPP ): OSStatus; external name '_InvokeATSQuadraticNewPathUPP';

>>>>>>> graemeg/fixes_2_2
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function InvokeATSQuadraticNewPathUPP( callBackDataPtr: UnivPtr; userUPP: ATSQuadraticNewPathUPP ): OSStatus; external name '_InvokeATSQuadraticNewPathUPP';

>>>>>>> origin/fixes_2_2
=======
function InvokeATSQuadraticNewPathUPP( callBackDataPtr: UnivPtr; userUPP: ATSQuadraticNewPathUPP ): OSStatus; external name '_InvokeATSQuadraticNewPathUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4

{
 *  ATSQuadraticLineProcPtr
 *  
 *  Discussion:
 *    A pointer to a client supplied callback function for handling
 *    glyph curve drawing operations. This callback handles operations
 *    to draw straight lines.
 *  
 *  Parameters:
 *    
 *    pt1:
 *      The starting point of the line.
 *    
 *    pt2:
 *      The end point of the line.
 *    
 *    callBackDataPtr:
 *      A pointer to any application specific data that may have been
 *      passed to the callbacks through the iCallbackDataPtr parameter
 *      of the ATSUGlyphGetQuadraticPaths function.
 *  
 *  Result:
 *    Return status. Pass any errors you wish to propagate back to the
 *    original caller of ATSUGlyphGetQuadraticPaths through this return
 *    value. Note that any nonzero result from this callback will halt
 *    the curve drawing process.
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
	ATSQuadraticLineProcPtr = function( const (*var*) pt1: Float32Point; const (*var*) pt2: Float32Point; callBackDataPtr: UnivPtr ): OSStatus;
	ATSQuadraticLineUPP = ATSQuadraticLineProcPtr;
=======
=======
>>>>>>> origin/fixes_2_2
type ATSQuadraticLineProcPtr = function( const (*var*) pt1, pt2: Float32Point; callBackDataPtr: UnivPtr ): OSStatus;
// Beats me what this translates to.  If someone finds out they can tell me and we'll update it
// typedef STACK_UPP_TYPE(ATSQuadraticLineProcPtr)                 ATSQuadraticLineUPP;
type ATSQuadraticLineUPP = Ptr;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
type
	ATSQuadraticLineProcPtr = function( const (*var*) pt1: Float32Point; const (*var*) pt2: Float32Point; callBackDataPtr: UnivPtr ): OSStatus;
	ATSQuadraticLineUPP = ATSQuadraticLineProcPtr;
>>>>>>> origin/fixes_2.4
{
 *  NewATSQuadraticLineUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function NewATSQuadraticLineUPP( userRoutine: ATSQuadraticLineProcPtr ): ATSQuadraticLineUPP; external name '_NewATSQuadraticLineUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function NewATSQuadraticLineUPP( userRoutine: ATSQuadraticLineProcPtr ): ATSQuadraticLineUPP; external name '_NewATSQuadraticLineUPP';
>>>>>>> graemeg/fixes_2_2
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function NewATSQuadraticLineUPP( userRoutine: ATSQuadraticLineProcPtr ): ATSQuadraticLineUPP; external name '_NewATSQuadraticLineUPP';
>>>>>>> origin/fixes_2_2
=======
function NewATSQuadraticLineUPP( userRoutine: ATSQuadraticLineProcPtr ): ATSQuadraticLineUPP; external name '_NewATSQuadraticLineUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4

{
 *  DisposeATSQuadraticLineUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure DisposeATSQuadraticLineUPP( userUPP: ATSQuadraticLineUPP ); external name '_DisposeATSQuadraticLineUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
procedure DisposeATSQuadraticLineUPP( userUPP: ATSQuadraticLineUPP ); external name '_DisposeATSQuadraticLineUPP';
>>>>>>> graemeg/fixes_2_2
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
procedure DisposeATSQuadraticLineUPP( userUPP: ATSQuadraticLineUPP ); external name '_DisposeATSQuadraticLineUPP';
>>>>>>> origin/fixes_2_2
=======
procedure DisposeATSQuadraticLineUPP( userUPP: ATSQuadraticLineUPP ); external name '_DisposeATSQuadraticLineUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4

{
 *  InvokeATSQuadraticLineUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function InvokeATSQuadraticLineUPP( const (*var*) pt1: Float32Point; const (*var*) pt2: Float32Point; callBackDataPtr: UnivPtr; userUPP: ATSQuadraticLineUPP ): OSStatus; external name '_InvokeATSQuadraticLineUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function InvokeATSQuadraticLineUPP( const (*var*) pt1, pt2: Float32Point; callBackDataPtr: UnivPtr; userUPP: ATSQuadraticLineUPP ): OSStatus; external name '_InvokeATSQuadraticLineUPP';

>>>>>>> graemeg/fixes_2_2
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function InvokeATSQuadraticLineUPP( const (*var*) pt1, pt2: Float32Point; callBackDataPtr: UnivPtr; userUPP: ATSQuadraticLineUPP ): OSStatus; external name '_InvokeATSQuadraticLineUPP';

>>>>>>> origin/fixes_2_2
=======
function InvokeATSQuadraticLineUPP( const (*var*) pt1: Float32Point; const (*var*) pt2: Float32Point; callBackDataPtr: UnivPtr; userUPP: ATSQuadraticLineUPP ): OSStatus; external name '_InvokeATSQuadraticLineUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4

{
 *  ATSQuadraticCurveProcPtr
 *  
 *  Discussion:
 *    A pointer to a client supplied callback function for handling
 *    glyph curve drawing operations. This callback handles operations
 *    to draw curves. The curve is a quadratic patch specified by a
 *    start point (pt1), and end point (pt2), and a single control
 *    point (controlPt).
 *  
 *  Parameters:
 *    
 *    pt1:
 *      The starting point of the curve.
 *    
 *    controlPt:
 *      The off-curve control point.
 *    
 *    pt2:
 *      The end point of the curve.
 *    
 *    callBackDataPtr:
 *      A pointer to any application specific data that may have been
 *      passed to the callbacks through the iCallbackDataPtr parameter
 *      of the ATSUGlyphGetQuadraticPaths function.
 *  
 *  Result:
 *    Return status. Pass any errors you wish to propagate back to the
 *    original caller of ATSUGlyphGetQuadraticPaths through this return
 *    value. Note that any nonzero result from this callback will halt
 *    the curve drawing process.
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
	ATSQuadraticCurveProcPtr = function( const (*var*) pt1: Float32Point; const (*var*) controlPt: Float32Point; const (*var*) pt2: Float32Point; callBackDataPtr: UnivPtr ): OSStatus;
	ATSQuadraticCurveUPP = ATSQuadraticCurveProcPtr;
=======
=======
>>>>>>> origin/fixes_2_2
type ATSQuadraticCurveProcPtr = function( const (*var*) pt1, controlPt, pt2: Float32Point; callBackDataPtr: UnivPtr ): OSStatus;
// Beats me what this translates to.  If someone finds out they can tell me and we'll update it
// typedef STACK_UPP_TYPE(ATSQuadraticCurveProcPtr)                ATSQuadraticCurveUPP;
type ATSQuadraticCurveUPP = Ptr;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
type
	ATSQuadraticCurveProcPtr = function( const (*var*) pt1: Float32Point; const (*var*) controlPt: Float32Point; const (*var*) pt2: Float32Point; callBackDataPtr: UnivPtr ): OSStatus;
	ATSQuadraticCurveUPP = ATSQuadraticCurveProcPtr;
>>>>>>> origin/fixes_2.4
{
 *  NewATSQuadraticCurveUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function NewATSQuadraticCurveUPP( userRoutine: ATSQuadraticCurveProcPtr ): ATSQuadraticCurveUPP; external name '_NewATSQuadraticCurveUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function NewATSQuadraticCurveUPP( userRoutine: ATSQuadraticCurveProcPtr ): ATSQuadraticCurveUPP; external name '_NewATSQuadraticCurveUPP';
>>>>>>> graemeg/fixes_2_2
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function NewATSQuadraticCurveUPP( userRoutine: ATSQuadraticCurveProcPtr ): ATSQuadraticCurveUPP; external name '_NewATSQuadraticCurveUPP';
>>>>>>> origin/fixes_2_2
=======
function NewATSQuadraticCurveUPP( userRoutine: ATSQuadraticCurveProcPtr ): ATSQuadraticCurveUPP; external name '_NewATSQuadraticCurveUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4

{
 *  DisposeATSQuadraticCurveUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure DisposeATSQuadraticCurveUPP( userUPP: ATSQuadraticCurveUPP ); external name '_DisposeATSQuadraticCurveUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
procedure DisposeATSQuadraticCurveUPP( userUPP: ATSQuadraticCurveUPP ); external name '_DisposeATSQuadraticCurveUPP';
>>>>>>> graemeg/fixes_2_2
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
procedure DisposeATSQuadraticCurveUPP( userUPP: ATSQuadraticCurveUPP ); external name '_DisposeATSQuadraticCurveUPP';
>>>>>>> origin/fixes_2_2
=======
procedure DisposeATSQuadraticCurveUPP( userUPP: ATSQuadraticCurveUPP ); external name '_DisposeATSQuadraticCurveUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4

{
 *  InvokeATSQuadraticCurveUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function InvokeATSQuadraticCurveUPP( const (*var*) pt1: Float32Point; const (*var*) controlPt: Float32Point; const (*var*) pt2: Float32Point; callBackDataPtr: UnivPtr; userUPP: ATSQuadraticCurveUPP ): OSStatus; external name '_InvokeATSQuadraticCurveUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function InvokeATSQuadraticCurveUPP( const (*var*) pt1, controlPt, pt2: Float32Point; callBackDataPtr: UnivPtr; userUPP: ATSQuadraticCurveUPP ): OSStatus; external name '_InvokeATSQuadraticCurveUPP';

>>>>>>> graemeg/fixes_2_2
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function InvokeATSQuadraticCurveUPP( const (*var*) pt1, controlPt, pt2: Float32Point; callBackDataPtr: UnivPtr; userUPP: ATSQuadraticCurveUPP ): OSStatus; external name '_InvokeATSQuadraticCurveUPP';

>>>>>>> origin/fixes_2_2
=======
function InvokeATSQuadraticCurveUPP( const (*var*) pt1: Float32Point; const (*var*) controlPt: Float32Point; const (*var*) pt2: Float32Point; callBackDataPtr: UnivPtr; userUPP: ATSQuadraticCurveUPP ): OSStatus; external name '_InvokeATSQuadraticCurveUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4

{
 *  ATSQuadraticClosePathProcPtr
 *  
 *  Discussion:
 *    A pointer to a client supplied callback function for handling
 *    glyph curve drawing operations. This callback handles operations
 *    to close the current drawing path.
 *  
 *  Parameters:
 *    
 *    callBackDataPtr:
 *      A pointer to any application specific data that may have been
 *      passed to the callbacks through the iCallbackDataPtr parameter
 *      of the ATSUGlyphGetQuadraticPaths function.
 *  
 *  Result:
 *    Return status. Pass any errors you wish to propagate back to the
 *    original caller of ATSUGlyphGetQuadraticPaths through this return
 *    value. Note that any nonzero result from this callback will halt
 *    the curve drawing process.
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
	ATSQuadraticClosePathProcPtr = function( callBackDataPtr: UnivPtr ): OSStatus;
	ATSQuadraticClosePathUPP = ATSQuadraticClosePathProcPtr;
=======
=======
>>>>>>> origin/fixes_2_2
type ATSQuadraticClosePathProcPtr = function( callBackDataPtr: UnivPtr ): OSStatus;
// Beats me what this translates to.  If someone finds out they can tell me and we'll update it
// typedef STACK_UPP_TYPE(ATSQuadraticClosePathProcPtr)            ATSQuadraticClosePathUPP;
type ATSQuadraticClosePathUPP = Ptr;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
type
	ATSQuadraticClosePathProcPtr = function( callBackDataPtr: UnivPtr ): OSStatus;
	ATSQuadraticClosePathUPP = ATSQuadraticClosePathProcPtr;
>>>>>>> origin/fixes_2.4
{
 *  NewATSQuadraticClosePathUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function NewATSQuadraticClosePathUPP( userRoutine: ATSQuadraticClosePathProcPtr ): ATSQuadraticClosePathUPP; external name '_NewATSQuadraticClosePathUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function NewATSQuadraticClosePathUPP( userRoutine: ATSQuadraticClosePathProcPtr ): ATSQuadraticClosePathUPP; external name '_NewATSQuadraticClosePathUPP';
>>>>>>> graemeg/fixes_2_2
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function NewATSQuadraticClosePathUPP( userRoutine: ATSQuadraticClosePathProcPtr ): ATSQuadraticClosePathUPP; external name '_NewATSQuadraticClosePathUPP';
>>>>>>> origin/fixes_2_2
=======
function NewATSQuadraticClosePathUPP( userRoutine: ATSQuadraticClosePathProcPtr ): ATSQuadraticClosePathUPP; external name '_NewATSQuadraticClosePathUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4

{
 *  DisposeATSQuadraticClosePathUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure DisposeATSQuadraticClosePathUPP( userUPP: ATSQuadraticClosePathUPP ); external name '_DisposeATSQuadraticClosePathUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
procedure DisposeATSQuadraticClosePathUPP( userUPP: ATSQuadraticClosePathUPP ); external name '_DisposeATSQuadraticClosePathUPP';
>>>>>>> graemeg/fixes_2_2
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
procedure DisposeATSQuadraticClosePathUPP( userUPP: ATSQuadraticClosePathUPP ); external name '_DisposeATSQuadraticClosePathUPP';
>>>>>>> origin/fixes_2_2
=======
procedure DisposeATSQuadraticClosePathUPP( userUPP: ATSQuadraticClosePathUPP ); external name '_DisposeATSQuadraticClosePathUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4

{
 *  InvokeATSQuadraticClosePathUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function InvokeATSQuadraticClosePathUPP( callBackDataPtr: UnivPtr; userUPP: ATSQuadraticClosePathUPP ): OSStatus; external name '_InvokeATSQuadraticClosePathUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)

{$ifc not TARGET_CPU_64}
{
 *  ATSUGlyphGetQuadraticPaths()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCreatePathForGlyph instead.
=======
=======
>>>>>>> origin/fixes_2_2
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
=======
>>>>>>> origin/fixes_2.4
function InvokeATSQuadraticClosePathUPP( callBackDataPtr: UnivPtr; userUPP: ATSQuadraticClosePathUPP ): OSStatus; external name '_InvokeATSQuadraticClosePathUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)

{$ifc not TARGET_CPU_64}
{
<<<<<<< HEAD
 *  ATSUGlyphGetQuadraticPaths()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUGlyphGetQuadraticPaths()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCreatePathForGlyph instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Uses a callback mechanism to obtain a set of Quadratic outlines
 *    for a specified glyph in a specified font.
 *  
 *  Discussion:
 *    This function will allow you to use callbacks to obtain the exact
 *    outline of a specified glyph, in quadratic form. Although this
 *    function will always return results for any valid ATSUI font, you
 *    should first use the function ATSUGetNativeCurveType to determine
 *    the native format of the glyph you are interested in. Then,
 *    either call ATSUGlyphGetQuadraticPaths or ATSUGlyphGetCubicPaths
 *    based on the result. Otherwise, you may end up with curves that
 *    are mathematically converted from cubic to quadratic (or vice
 *    versa), instead of getting native font data. See the definitions
 *    of ATSQuadraticNewPathProcPtr, ATSQuadraticLineProcPtr,
 *    ATSQuadraticCurveProcPtr, and ATSQuadraticClosePathProcPtr for
 *    more information about setting up the callbacks.
 *  
 *  Parameters:
 *    
 *    iATSUStyle:
 *      A style referring to a font you wish to obtain a set of glyph
 *      outlines from.
 *    
 *    iGlyphID:
 *      A ID number referring to the glyph for which you wish to obtain
 *      outline data. Use the ATSUI direct access functions in
 *      ATSUnicodeDirectAccess.h to obtain values to pass for this
 *      parameter.
 *    
 *    iNewPathProc:
 *      A pointer to a callback function for quadratic new path
 *      operations. See the definition of ATSQuadraticNewPathProcPtr
 *      for more information about creating, disposing, and invoking
 *      this type of Universal Procedure Pointer.
 *    
 *    iLineProc:
 *      A pointer to a callback function for quadratic LineTo
 *      operations. See the definition of ATSQuadraticLineProcPtr for
 *      more information about creating, disposing, and invoking this
 *      type of Universal Procedure Pointer.
 *    
 *    iCurveProc:
 *      A pointer to a callback function for quadratic curve
 *      operations. See the definition of ATSQuadraticCurveProcPtr for
 *      more information about creating, disposing, and invoking this
 *      type of Universal Procedure Pointer.
 *    
 *    iClosePathProc:
 *      A pointer to a callback function for quadratic close path
 *      operations. See the definition of ATSQuadraticClosePathProcPtr
 *      for more information about creating, disposing, and invoking
 *      this type of Universal Procedure Pointer.
 *    
 *    iCallbackDataPtr:
 *      Any valid pointer. Any application specific data you wish to
 *      pass to your callbacks may be sent through this parameter.
 *    
 *    oCallbackResult:
 *      On return, status returned by callback functions. If an error
 *      occurs, callbacks may communicate it through this parameter.
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 9.1 and later
 }
function ATSUGlyphGetQuadraticPaths( iATSUStyle: ATSUStyle; iGlyphID: GlyphID; iNewPathProc: ATSQuadraticNewPathUPP; iLineProc: ATSQuadraticLineUPP; iCurveProc: ATSQuadraticCurveUPP; iClosePathProc: ATSQuadraticClosePathUPP; iCallbackDataPtr: UnivPtr; var oCallbackResult: OSStatus ): OSStatus; external name '_ATSUGlyphGetQuadraticPaths';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{$endc} {not TARGET_CPU_64}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 9.1 and later
 }
function ATSUGlyphGetQuadraticPaths( iATSUStyle: ATSUStyle; iGlyphID: GlyphID; iNewPathProc: ATSQuadraticNewPathUPP; iLineProc: ATSQuadraticLineUPP; iCurveProc: ATSQuadraticCurveUPP; iClosePathProc: ATSQuadraticClosePathUPP; iCallbackDataPtr: UnivPtr; var oCallbackResult: OSStatus ): OSStatus; external name '_ATSUGlyphGetQuadraticPaths';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{$endc} {not TARGET_CPU_64}
>>>>>>> origin/fixes_2.4


{
 *  ATSCubicMoveToProcPtr
 *  
 *  Discussion:
 *    A pointer to a client supplied callback function for handling
 *    glyph curve drawing operations. This callback handles operations
 *    to move the current pen location.
 *  
 *  Parameters:
 *    
 *    pt:
 *      The point to which to move the current pen location.
 *    
 *    callBackDataPtr:
 *      A pointer to any application specific data that may have been
 *      passed to the callbacks through the iCallbackDataPtr parameter
 *      of the ATSUGlyphGetCubicPaths function.
 *  
 *  Result:
 *    Return status. Pass any errors you wish to propagate back to the
 *    original caller of ATSUGlyphGetCubicPaths through this return
 *    value. Note that any nonzero result from this callback will halt
 *    the curve drawing process.
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
	ATSCubicMoveToProcPtr = function( const (*var*) pt: Float32Point; callBackDataPtr: UnivPtr ): OSStatus;
	ATSCubicMoveToUPP = ATSCubicMoveToProcPtr;
=======
=======
>>>>>>> origin/fixes_2_2
type ATSCubicMoveToProcPtr = function( const (*var*) pt: Float32Point; callBackDataPtr: UnivPtr ): OSStatus;
// Beats me what this translates to.  If someone finds out they can tell me and we'll update it
// typedef STACK_UPP_TYPE(ATSCubicMoveToProcPtr)                   ATSCubicMoveToUPP;
type ATSCubicMoveToUPP = Ptr;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
type
	ATSCubicMoveToProcPtr = function( const (*var*) pt: Float32Point; callBackDataPtr: UnivPtr ): OSStatus;
	ATSCubicMoveToUPP = ATSCubicMoveToProcPtr;
>>>>>>> origin/fixes_2.4
{
 *  NewATSCubicMoveToUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function NewATSCubicMoveToUPP( userRoutine: ATSCubicMoveToProcPtr ): ATSCubicMoveToUPP; external name '_NewATSCubicMoveToUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function NewATSCubicMoveToUPP( userRoutine: ATSCubicMoveToProcPtr ): ATSCubicMoveToUPP; external name '_NewATSCubicMoveToUPP';
>>>>>>> graemeg/fixes_2_2
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function NewATSCubicMoveToUPP( userRoutine: ATSCubicMoveToProcPtr ): ATSCubicMoveToUPP; external name '_NewATSCubicMoveToUPP';
>>>>>>> origin/fixes_2_2
=======
function NewATSCubicMoveToUPP( userRoutine: ATSCubicMoveToProcPtr ): ATSCubicMoveToUPP; external name '_NewATSCubicMoveToUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4

{
 *  DisposeATSCubicMoveToUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure DisposeATSCubicMoveToUPP( userUPP: ATSCubicMoveToUPP ); external name '_DisposeATSCubicMoveToUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
procedure DisposeATSCubicMoveToUPP( userUPP: ATSCubicMoveToUPP ); external name '_DisposeATSCubicMoveToUPP';
>>>>>>> graemeg/fixes_2_2
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
procedure DisposeATSCubicMoveToUPP( userUPP: ATSCubicMoveToUPP ); external name '_DisposeATSCubicMoveToUPP';
>>>>>>> origin/fixes_2_2
=======
procedure DisposeATSCubicMoveToUPP( userUPP: ATSCubicMoveToUPP ); external name '_DisposeATSCubicMoveToUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4

{
 *  InvokeATSCubicMoveToUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function InvokeATSCubicMoveToUPP( const (*var*) pt: Float32Point; callBackDataPtr: UnivPtr; userUPP: ATSCubicMoveToUPP ): OSStatus; external name '_InvokeATSCubicMoveToUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function InvokeATSCubicMoveToUPP( const (*var*) pt: Float32Point; callBackDataPtr: UnivPtr; userUPP: ATSCubicMoveToUPP ): OSStatus; external name '_InvokeATSCubicMoveToUPP';

>>>>>>> graemeg/fixes_2_2
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function InvokeATSCubicMoveToUPP( const (*var*) pt: Float32Point; callBackDataPtr: UnivPtr; userUPP: ATSCubicMoveToUPP ): OSStatus; external name '_InvokeATSCubicMoveToUPP';

>>>>>>> origin/fixes_2_2
=======
function InvokeATSCubicMoveToUPP( const (*var*) pt: Float32Point; callBackDataPtr: UnivPtr; userUPP: ATSCubicMoveToUPP ): OSStatus; external name '_InvokeATSCubicMoveToUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4

{
 *  ATSCubicLineToProcPtr
 *  
 *  Discussion:
 *    A pointer to a client supplied callback function for handling
 *    glyph curve drawing operations. This callback handles operations
 *    to draw straight lines.
 *  
 *  Parameters:
 *    
 *    pt:
 *      The end point of the line to be drawn. The starting point is
 *      whatever the current pen position is.
 *    
 *    callBackDataPtr:
 *      A pointer to any application specific data that may have been
 *      passed to the callbacks through the iCallbackDataPtr parameter
 *      of the ATSUGlyphGetCubicPaths function.
 *  
 *  Result:
 *    Return status. Pass any errors you wish to propagate back to the
 *    original caller of ATSUGlyphGetCubicPaths through this return
 *    value. Note that any nonzero result from this callback will halt
 *    the curve drawing process.
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
	ATSCubicLineToProcPtr = function( const (*var*) pt: Float32Point; callBackDataPtr: UnivPtr ): OSStatus;
	ATSCubicLineToUPP = ATSCubicLineToProcPtr;
=======
=======
>>>>>>> origin/fixes_2_2
type ATSCubicLineToProcPtr = function( const (*var*) pt: Float32Point; callBackDataPtr: UnivPtr ): OSStatus;
// Beats me what this translates to.  If someone finds out they can tell me and we'll update it
// typedef STACK_UPP_TYPE(ATSCubicLineToProcPtr)                   ATSCubicLineToUPP;
type ATSCubicLineToUPP = Ptr;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
type
	ATSCubicLineToProcPtr = function( const (*var*) pt: Float32Point; callBackDataPtr: UnivPtr ): OSStatus;
	ATSCubicLineToUPP = ATSCubicLineToProcPtr;
>>>>>>> origin/fixes_2.4
{
 *  NewATSCubicLineToUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function NewATSCubicLineToUPP( userRoutine: ATSCubicLineToProcPtr ): ATSCubicLineToUPP; external name '_NewATSCubicLineToUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function NewATSCubicLineToUPP( userRoutine: ATSCubicLineToProcPtr ): ATSCubicLineToUPP; external name '_NewATSCubicLineToUPP';
>>>>>>> graemeg/fixes_2_2
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function NewATSCubicLineToUPP( userRoutine: ATSCubicLineToProcPtr ): ATSCubicLineToUPP; external name '_NewATSCubicLineToUPP';
>>>>>>> origin/fixes_2_2
=======
function NewATSCubicLineToUPP( userRoutine: ATSCubicLineToProcPtr ): ATSCubicLineToUPP; external name '_NewATSCubicLineToUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4

{
 *  DisposeATSCubicLineToUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure DisposeATSCubicLineToUPP( userUPP: ATSCubicLineToUPP ); external name '_DisposeATSCubicLineToUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
procedure DisposeATSCubicLineToUPP( userUPP: ATSCubicLineToUPP ); external name '_DisposeATSCubicLineToUPP';
>>>>>>> graemeg/fixes_2_2
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
procedure DisposeATSCubicLineToUPP( userUPP: ATSCubicLineToUPP ); external name '_DisposeATSCubicLineToUPP';
>>>>>>> origin/fixes_2_2
=======
procedure DisposeATSCubicLineToUPP( userUPP: ATSCubicLineToUPP ); external name '_DisposeATSCubicLineToUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4

{
 *  InvokeATSCubicLineToUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function InvokeATSCubicLineToUPP( const (*var*) pt: Float32Point; callBackDataPtr: UnivPtr; userUPP: ATSCubicLineToUPP ): OSStatus; external name '_InvokeATSCubicLineToUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function InvokeATSCubicLineToUPP( const (*var*) pt: Float32Point; callBackDataPtr: UnivPtr; userUPP: ATSCubicLineToUPP ): OSStatus; external name '_InvokeATSCubicLineToUPP';

>>>>>>> graemeg/fixes_2_2
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function InvokeATSCubicLineToUPP( const (*var*) pt: Float32Point; callBackDataPtr: UnivPtr; userUPP: ATSCubicLineToUPP ): OSStatus; external name '_InvokeATSCubicLineToUPP';

>>>>>>> origin/fixes_2_2
=======
function InvokeATSCubicLineToUPP( const (*var*) pt: Float32Point; callBackDataPtr: UnivPtr; userUPP: ATSCubicLineToUPP ): OSStatus; external name '_InvokeATSCubicLineToUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4

{
 *  ATSCubicCurveToProcPtr
 *  
 *  Discussion:
 *    A pointer to a client supplied callback function for handling
 *    glyph curve drawing operations. This callback handles operations
 *    to draw a curve. The curve is a Bezier patch defined by two
 *    off-curve control points (pt1 and pt2), and an endpoint (pt3).
 *    The starting point is whatever the current pen position is.
 *  
 *  Parameters:
 *    
 *    pt1:
 *      The first off-curve control point.
 *    
 *    pt2:
 *      The second off-curve control point.
 *    
 *    pt3:
 *      The end point of the curve.
 *    
 *    callBackDataPtr:
 *      A pointer to any application specific data that may have been
 *      passed to the callbacks through the iCallbackDataPtr parameter
 *      of the ATSUGlyphGetCubicPaths function.
 *  
 *  Result:
 *    Return status. Pass any errors you wish to propagate back to the
 *    original caller of ATSUGlyphGetCubicPaths through this return
 *    value. Note that any nonzero result from this callback will halt
 *    the curve drawing process.
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
	ATSCubicCurveToProcPtr = function( const (*var*) pt1: Float32Point; const (*var*) pt2: Float32Point; const (*var*) pt3: Float32Point; callBackDataPtr: UnivPtr ): OSStatus;
	ATSCubicCurveToUPP = ATSCubicCurveToProcPtr;
=======
=======
>>>>>>> origin/fixes_2_2
type ATSCubicCurveToProcPtr = function( const (*var*) pt, pt2, pt3: Float32Point; callBackDataPtr: UnivPtr ): OSStatus;
// Beats me what this translates to.  If someone finds out they can tell me and we'll update it
// typedef STACK_UPP_TYPE(ATSCubicCurveToProcPtr)                  ATSCubicCurveToUPP;
type ATSCubicCurveToUPP = Ptr;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
type
	ATSCubicCurveToProcPtr = function( const (*var*) pt1: Float32Point; const (*var*) pt2: Float32Point; const (*var*) pt3: Float32Point; callBackDataPtr: UnivPtr ): OSStatus;
	ATSCubicCurveToUPP = ATSCubicCurveToProcPtr;
>>>>>>> origin/fixes_2.4
{
 *  NewATSCubicCurveToUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function NewATSCubicCurveToUPP( userRoutine: ATSCubicCurveToProcPtr ): ATSCubicCurveToUPP; external name '_NewATSCubicCurveToUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function NewATSCubicCurveToUPP( userRoutine: ATSCubicCurveToProcPtr ): ATSCubicCurveToUPP; external name '_NewATSCubicCurveToUPP';
>>>>>>> graemeg/fixes_2_2
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function NewATSCubicCurveToUPP( userRoutine: ATSCubicCurveToProcPtr ): ATSCubicCurveToUPP; external name '_NewATSCubicCurveToUPP';
>>>>>>> origin/fixes_2_2
=======
function NewATSCubicCurveToUPP( userRoutine: ATSCubicCurveToProcPtr ): ATSCubicCurveToUPP; external name '_NewATSCubicCurveToUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4

{
 *  DisposeATSCubicCurveToUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure DisposeATSCubicCurveToUPP( userUPP: ATSCubicCurveToUPP ); external name '_DisposeATSCubicCurveToUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
procedure DisposeATSCubicCurveToUPP( userUPP: ATSCubicCurveToUPP ); external name '_DisposeATSCubicCurveToUPP';
>>>>>>> graemeg/fixes_2_2
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
procedure DisposeATSCubicCurveToUPP( userUPP: ATSCubicCurveToUPP ); external name '_DisposeATSCubicCurveToUPP';
>>>>>>> origin/fixes_2_2
=======
procedure DisposeATSCubicCurveToUPP( userUPP: ATSCubicCurveToUPP ); external name '_DisposeATSCubicCurveToUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4

{
 *  InvokeATSCubicCurveToUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function InvokeATSCubicCurveToUPP( const (*var*) pt1: Float32Point; const (*var*) pt2: Float32Point; const (*var*) pt3: Float32Point; callBackDataPtr: UnivPtr; userUPP: ATSCubicCurveToUPP ): OSStatus; external name '_InvokeATSCubicCurveToUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function InvokeATSCubicCurveToUPP( const (*var*) pt, pt2, pt3: Float32Point; callBackDataPtr: UnivPtr; userUPP: ATSCubicCurveToUPP ): OSStatus; external name '_InvokeATSCubicCurveToUPP';

>>>>>>> graemeg/fixes_2_2
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function InvokeATSCubicCurveToUPP( const (*var*) pt, pt2, pt3: Float32Point; callBackDataPtr: UnivPtr; userUPP: ATSCubicCurveToUPP ): OSStatus; external name '_InvokeATSCubicCurveToUPP';

>>>>>>> origin/fixes_2_2
=======
function InvokeATSCubicCurveToUPP( const (*var*) pt1: Float32Point; const (*var*) pt2: Float32Point; const (*var*) pt3: Float32Point; callBackDataPtr: UnivPtr; userUPP: ATSCubicCurveToUPP ): OSStatus; external name '_InvokeATSCubicCurveToUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4

{
 *  ATSCubicClosePathProcPtr
 *  
 *  Discussion:
 *    A pointer to a client supplied callback function for handling
 *    glyph curve drawing operations. This callback handles operations
 *    to close the current drawing path.
 *  
 *  Parameters:
 *    
 *    callBackDataPtr:
 *      A pointer to any application specific data that may have been
 *      passed to the callbacks through the iCallbackDataPtr parameter
 *      of the ATSUGlyphGetCubicPaths function.
 *  
 *  Result:
 *    Return status. Pass any errors you wish to propagate back to the
 *    original caller of ATSUGlyphGetCubicPaths through this return
 *    value. Note that any nonzero result from this callback will halt
 *    the curve drawing process.
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
	ATSCubicClosePathProcPtr = function( callBackDataPtr: UnivPtr ): OSStatus;
	ATSCubicClosePathUPP = ATSCubicClosePathProcPtr;
=======
=======
>>>>>>> origin/fixes_2_2
type ATSCubicClosePathProcPtr = function( callBackDataPtr: UnivPtr ): OSStatus;
// Beats me what this translates to.  If someone finds out they can tell me and we'll update it
// typedef STACK_UPP_TYPE(ATSCubicClosePathProcPtr)                ATSCubicClosePathUPP;
type ATSCubicClosePathUPP = Ptr;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
type
	ATSCubicClosePathProcPtr = function( callBackDataPtr: UnivPtr ): OSStatus;
	ATSCubicClosePathUPP = ATSCubicClosePathProcPtr;
>>>>>>> origin/fixes_2.4
{
 *  NewATSCubicClosePathUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function NewATSCubicClosePathUPP( userRoutine: ATSCubicClosePathProcPtr ): ATSCubicClosePathUPP; external name '_NewATSCubicClosePathUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function NewATSCubicClosePathUPP( userRoutine: ATSCubicClosePathProcPtr ): ATSCubicClosePathUPP; external name '_NewATSCubicClosePathUPP';
>>>>>>> graemeg/fixes_2_2
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function NewATSCubicClosePathUPP( userRoutine: ATSCubicClosePathProcPtr ): ATSCubicClosePathUPP; external name '_NewATSCubicClosePathUPP';
>>>>>>> origin/fixes_2_2
=======
function NewATSCubicClosePathUPP( userRoutine: ATSCubicClosePathProcPtr ): ATSCubicClosePathUPP; external name '_NewATSCubicClosePathUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4

{
 *  DisposeATSCubicClosePathUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure DisposeATSCubicClosePathUPP( userUPP: ATSCubicClosePathUPP ); external name '_DisposeATSCubicClosePathUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
procedure DisposeATSCubicClosePathUPP( userUPP: ATSCubicClosePathUPP ); external name '_DisposeATSCubicClosePathUPP';
>>>>>>> graemeg/fixes_2_2
=======
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
procedure DisposeATSCubicClosePathUPP( userUPP: ATSCubicClosePathUPP ); external name '_DisposeATSCubicClosePathUPP';
>>>>>>> origin/fixes_2_2
=======
procedure DisposeATSCubicClosePathUPP( userUPP: ATSCubicClosePathUPP ); external name '_DisposeATSCubicClosePathUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4

{
 *  InvokeATSCubicClosePathUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function InvokeATSCubicClosePathUPP( callBackDataPtr: UnivPtr; userUPP: ATSCubicClosePathUPP ): OSStatus; external name '_InvokeATSCubicClosePathUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)

{$ifc not TARGET_CPU_64}
{
 *  ATSUGlyphGetCubicPaths()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCreatePathForGlyph instead.
=======
=======
>>>>>>> origin/fixes_2_2
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
=======
>>>>>>> origin/fixes_2.4
function InvokeATSCubicClosePathUPP( callBackDataPtr: UnivPtr; userUPP: ATSCubicClosePathUPP ): OSStatus; external name '_InvokeATSCubicClosePathUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)

{$ifc not TARGET_CPU_64}
{
<<<<<<< HEAD
 *  ATSUGlyphGetCubicPaths()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUGlyphGetCubicPaths()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCreatePathForGlyph instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Uses a callback mechanism to obtain a set of Cubic outlines for a
 *    specified glyph in a specified font.
 *  
 *  Discussion:
 *    This function will allow you to use callbacks to obtain the exact
 *    outline of a specified glyph, in cubic form. Although this
 *    function will always return results for any valid ATSUI font, you
 *    should first use the function ATSUGetNativeCurveType to determine
 *    the native format of the glyph you are interested in. Then,
 *    either call ATSUGlyphGetQuadraticPaths or ATSUGlyphGetCubicPaths
 *    based on the result. Otherwise, you may end up with curves that
 *    are mathematically converted from quadratic to cubic (or vice
 *    versa), instead of getting native font data. See the definitions
 *    of ATSCubicMoveToProcPtr, ATSCubicLineToProcPtr,
 *    ATSCubicCurveToProcPtr, and ATSCubicClosePathProcPtr for more
 *    information about setting up the callbacks.
 *  
 *  Parameters:
 *    
 *    iATSUStyle:
 *      A style referring to a font you wish to obtain a set of glyph
 *      outlines from.
 *    
 *    iGlyphID:
 *      A ID number referring to the glyph for which you wish to obtain
 *      outline data. Use the ATSUI direct access functions in
 *      ATSUnicodeDirectAccess.h to obtain values to pass for this
 *      parameter.
 *    
 *    iMoveToProc:
 *      A pointer to a callback function for cubic MoveTo operations.
 *      See the definition of ATSCubicMoveToProcPtr for more
 *      information about creating, disposing, and invoking this type
 *      of Universal Procedure Pointer.
 *    
 *    iLineToProc:
 *      A pointer to a callback function for cubic LineTo operations.
 *      See the definition of ATSCubicLineToProcPtr for more
 *      information about creating, disposing, and invoking this type
 *      of Universal Procedure Pointer.
 *    
 *    iCurveToProc:
 *      A pointer to a callback function for cubic CurveTo operations.
 *      See the definition of ATSCubicCurveToProcPtr for more
 *      information about creating, disposing, and invoking this type
 *      of Universal Procedure Pointer.
 *    
 *    iClosePathProc:
 *      A pointer to a callback function for cubic MoveTo operations.
 *      See the definition of ATSCubicClosePathProcPtr for more
 *      information about creating, disposing, and invoking this type
 *      of Universal Procedure Pointer.
 *    
 *    iCallbackDataPtr:
 *      Any valid pointer. Any application specific data you wish to
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      pass to your callbacks may be sent through this parameter.
=======
 *      pass to your callbacks may be sent through this parameter. can be NULL
>>>>>>> graemeg/fixes_2_2
=======
 *      pass to your callbacks may be sent through this parameter. can be NULL
>>>>>>> origin/fixes_2_2
=======
 *      pass to your callbacks may be sent through this parameter.
>>>>>>> origin/fixes_2.4
 *    
 *    oCallbackResult:
 *      On return, status returned by callback functions. If an error
 *      occurs, callbacks may communicate it through this parameter.
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 9.1 and later
 }
function ATSUGlyphGetCubicPaths( iATSUStyle: ATSUStyle; iGlyphID: GlyphID; iMoveToProc: ATSCubicMoveToUPP; iLineToProc: ATSCubicLineToUPP; iCurveToProc: ATSCubicCurveToUPP; iClosePathProc: ATSCubicClosePathUPP; iCallbackDataPtr: UnivPtr { can be NULL }; var oCallbackResult: OSStatus ): OSStatus; external name '_ATSUGlyphGetCubicPaths';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
 *  ATSUGlyphGetCurvePaths()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCreatePathForGlyph instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 9.1 and later
 }
function ATSUGlyphGetCubicPaths( iATSUStyle: ATSUStyle; iGlyphID: GlyphID; iMoveToProc: ATSCubicMoveToUPP; iLineToProc: ATSCubicLineToUPP; iCurveToProc: ATSCubicCurveToUPP; iClosePathProc: ATSCubicClosePathUPP; iCallbackDataPtr: UnivPtr { can be NULL }; var oCallbackResult: OSStatus ): OSStatus; external name '_ATSUGlyphGetCubicPaths';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{
<<<<<<< HEAD
 *  ATSUGlyphGetCurvePaths()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUGlyphGetCurvePaths()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTFontCreatePathForGlyph instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Obtains glyph curve data without the use of callbacks.
 *  
 *  Discussion:
 *    This function will return glyph curve data in a single data
 *    structure rather than through the use of callbacks, but you must
 *    parse the data structure yourself. ATSUGlyphGetCubicPaths and
 *    ATSUGlyphGetQuadraticPaths will parse the glyph data for you and
 *    use the callbacks you provide them to give you access to the
 *    individual points on the curves. Typically you use the function
 *    ATSUGlyphGetCurvePaths by calling it twice, as follows: (1) Pass
 *    a valid style and glyphID into the iATSUStyle and iGlyphID
 *    parameters, respectively, 0 for the ioBufferSize parameter, and
 *    NULL for the oPaths parameter. ATSUGlyphGetCurvePaths returns the
 *    size to use for the oPaths array in the ioBufferSize parameter.
 *    (2) Allocate enough space an array of the returned size, then
 *    call the ATSUGlyphGetCurvePaths again, passing a pointer to the
 *    array in the oPaths parameter. On return, the array contains the
 *    glyph outline data.
 *  
 *  Parameters:
 *    
 *    iATSUStyle:
 *      A style referring to a font you wish to obtain a set of glyph
 *      outlines from.
 *    
 *    iGlyphID:
 *      A ID number referring to the glyph for which you wish to obtain
 *      outline data. Use the ATSUI direct access functions in
 *      ATSUnicodeDirectAccess.h to obtain values to pass for this
 *      parameter.
 *    
 *    ioBufferSize:
 *      On input, the size of the buffer you have allocated for the
 *      oPaths parameter. On return, the actual size of the data
 *      structure that has been copied into the oPaths parameter.
 *    
 *    oPaths:
 *      On return, a data structure containing glyph outline
 *      information. See ATSTypes.h for a definition of this data
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      structure.
=======
 *      structure. can be NULL
>>>>>>> graemeg/fixes_2_2
=======
 *      structure. can be NULL
>>>>>>> origin/fixes_2_2
=======
 *      structure.
>>>>>>> origin/fixes_2.4
 *  
 *  Result:
 *    On success, noErr is returned. See MacErrors.h for possible error
 *    codes.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 9.1 and later
 }
function ATSUGlyphGetCurvePaths( iATSUStyle: ATSUStyle; iGlyphID: GlyphID; ioBufferSize: ByteCountPtr; oPaths: ATSUCurvePathsPtr { can be NULL } ): OSStatus; external name '_ATSUGlyphGetCurvePaths';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 9.1 and later
 }
// AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER
function ATSUGlyphGetCurvePaths( iATSUStyle: ATSUStyle; iGlyphID: GlyphID; ioBufferSize: ByteCountPtr; oPaths: ATSUCurvePathsPtr ): OSStatus; external name '_ATSUGlyphGetCurvePaths';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 9.1 and later
 }
function ATSUGlyphGetCurvePaths( iATSUStyle: ATSUStyle; iGlyphID: GlyphID; ioBufferSize: ByteCountPtr; oPaths: ATSUCurvePathsPtr { can be NULL } ): OSStatus; external name '_ATSUGlyphGetCurvePaths';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)
>>>>>>> origin/fixes_2.4


{ Functions listed beyond this point are either deprecated or not recommended }

{ ---------------------------------------------------------------------------- }
{ ATSUI glyphInfo access (deprecated)                                          }
{ ---------------------------------------------------------------------------- }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  ATSUGetGlyphInfo()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTRunGetGlyphsPtr,CTRunGetGlyphs, CTRunGetPositionsPtr,
 *    CTRunGetPositions, CTRunGetStringIndicesPtr,
 *    CTRunGetStringIndices, CTRunGetStringRange instead.
<<<<<<< HEAD
=======
 *  ATSUGetGlyphInfo()
>>>>>>> graemeg/fixes_2_2
=======
 *  ATSUGetGlyphInfo()
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Obtains a copy of the style and layout information for each glyph
 *    in a line.
 *  
 *  Discussion:
 *    Please see ATSUnicodeDirectAccess.h for replacement functions.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 9.1 and later
 }
function ATSUGetGlyphInfo( iTextLayout: ATSUTextLayout; iLineStart: UniCharArrayOffset; iLineLength: UniCharCount; ioBufferSize: ByteCountPtr; oGlyphInfoPtr: ATSUGlyphInfoArrayPtr ): OSStatus; external name '_ATSUGetGlyphInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


{
 *  ATSUDrawGlyphInfo()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTRunGetGlyphsPtr,CTRunGetGlyphs, CTRunGetPositionsPtr,
 *    CTRunGetPositions, CTRunGetStringIndicesPtr,
 *    CTRunGetStringIndices, CTRunGetStringRange instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.3
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 9.1 and later
 }
function ATSUGetGlyphInfo( iTextLayout: ATSUTextLayout; iLineStart: UniCharArrayOffset; iLineLength: UniCharCount; ioBufferSize: ByteCountPtr; oGlyphInfoPtr: ATSUGlyphInfoArrayPtr ): OSStatus; external name '_ATSUGetGlyphInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


{
<<<<<<< HEAD
 *  ATSUDrawGlyphInfo()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUDrawGlyphInfo()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTRunGetGlyphsPtr,CTRunGetGlyphs, CTRunGetPositionsPtr,
 *    CTRunGetPositions, CTRunGetStringIndicesPtr,
 *    CTRunGetStringIndices, CTRunGetStringRange instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Draws glyphs at the specified location, based on style and layout
 *    information specified for each glyph.
 *  
 *  Discussion:
 *    Please see ATSUnicodeDirectAccess.h for replacement functions.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.3
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 9.1 and later
 }
function ATSUDrawGlyphInfo( iGlyphInfoArray: ATSUGlyphInfoArrayPtr; iLocation: Float32Point ): OSStatus; external name '_ATSUDrawGlyphInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_3 *)


{$endc} {not TARGET_CPU_64}

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.3
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   in ATSUnicodeLib 9.1 and later
 }
function ATSUDrawGlyphInfo( iGlyphInfoArray: ATSUGlyphInfoArrayPtr; iLocation: Float32Point ): OSStatus; external name '_ATSUDrawGlyphInfo';
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
