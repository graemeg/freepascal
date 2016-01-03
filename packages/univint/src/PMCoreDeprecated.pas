{
     File:       PrintCore/PMCoreDeprecated.h
 
     Contains:   Deprecated Carbon Printing Manager Interfaces.
 
     Copyright (c) 1998-2006,2008 by Apple Inc. All Rights Reserved.
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}
{  Pascal Translation:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
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
{    Pascal Translation:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit PMCoreDeprecated;
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
>>>>>>> graemeg/cpstrnew
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
uses MacTypes,Files,QuickdrawTypes,PMDefinitions,PMDefinitionsDeprecated,ColorSyncDeprecated,CFBase,CFArray,CFData;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}


{
 *  PMFlattenPageFormatToCFData() *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMPageFormatCreateDataRepresentation instead.
 *
 *  Summary:
 *    Returns a flattened representation of the page format object in a CFDataRef.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMFlattenPageFormatToCFData( pageFormat: PMPageFormat; var flatFormat: CFDataRef ): OSStatus; external name '_PMFlattenPageFormatToCFData';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  PMFlattenPageFormatToURL() *** DEPRECATED ***
 *  
 *  Discussion:
 *    Instead use PMPageFormatCreateDataRepresentation and write the resulting data to your destination.
 *
 *  Summary:
 *    Writes a flattened representation of the print settings to the
 *    URL specified by flattenFileURL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMFlattenPageFormatToURL( pageFormat: PMPageFormat; flattenFileURL: CFURLRef ): OSStatus; external name '_PMFlattenPageFormatToURL';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  PMUnflattenPageFormatWithCFData()	*** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMPageFormatCreateWithDataRepresentation instead.
 *
 *  Summary:
 *    Returns a page format object given a CFDataRef to a previously
 *    flattened page format.
 *  
 *  Parameters:
 *    
 *    flattenCFData:
 *      A CFDataRef containing a flattened print settings.
 *    
 *    pageFormat:
 *      On return, a newly created page format object create with the
 *      data contained in 'flattenCFData'. The printing framework will
 *      create the object. The app is responsible for disposing of it.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMUnflattenPageFormatWithCFData( flattenCFData: CFDataRef; var pageFormat: PMPageFormat ): OSStatus; external name '_PMUnflattenPageFormatWithCFData';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  PMUnflattenPageFormatWithURL()	*** DEPRECATED ***
 *  
 *  Discussion:
 *    Instead read the data into a CFData object and use PMPageFormatCreateWithDataRepresentation.
 *
 *  Summary:
 *    Returns a page format object given a CFURLRef referencing a file
 *    containing a previously flattened page format.
 *  
 *  Parameters:
 *    
 *    flattenFileURL:
 *      A CFURLRef referencing a file that contains a flattened page
 *      format.
 *    
 *    pageFormat:
 *      On return, a newly created page format object created from the
 *      flattened page format data specified by 'flattenFileURL'. The
 *      caller is responsible for releasing the page format created.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMUnflattenPageFormatWithURL( flattenFileURL: CFURLRef; var pageFormat: PMPageFormat ): OSStatus; external name '_PMUnflattenPageFormatWithURL';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  PMFlattenPrintSettingsToCFData()	*** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMPrintSettingsCreateDataRepresentation instead.
 *
 *  Summary:
 *    Returns a flattened print settings object as a CFDataRef.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMFlattenPrintSettingsToCFData( printSettings: PMPrintSettings; var flatSetting: CFDataRef ): OSStatus; external name '_PMFlattenPrintSettingsToCFData';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  PMFlattenPrintSettingsToURL()	*** DEPRECATED ***
 *  
 *  Discussion:
 *    Instead use PMPrintSettingsCreateDataRepresentation and write the resulting data to your destination.
 *
 *  Summary:
 *    Writes a flattened representation of the print settings to the
 *    URL specified by flattenFileURL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMFlattenPrintSettingsToURL( printSettings: PMPrintSettings; flattenFileURL: CFURLRef ): OSStatus; external name '_PMFlattenPrintSettingsToURL';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  PMUnflattenPrintSettingsWithCFData()	*** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMPrintSettingsCreateWithDataRepresentation instead.
 *
 *  Summary:
 *    Returns a print settings object given a CFDataRef to a previously
 *    flatten print settings.
 *  
 *  Parameters:
 *    
 *    flattenCFData:
 *      A CFDataRef containing a flatten print settings.
 *    
 *    printSettings:
 *      On return, a newly created print settings object create with
 *      the data contained in 'flattenCFData'. The printing framework
 *      will create the object. The app is responsible for disposing of
 *      it.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMUnflattenPrintSettingsWithCFData( flattenCFData: CFDataRef; var printSettings: PMPrintSettings ): OSStatus; external name '_PMUnflattenPrintSettingsWithCFData';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  PMUnflattenPrintSettingsWithURL()	*** DEPRECATED ***
 *  
 *  Discussion:
 *    Instead read the data into a CFData object and use PMPrintSettingsCreateWithDataRepresentation.
 *
 *  Summary:
 *    Returns a print settings object given a CFURLRef referencing a
 *    file containing a previously flattened print settings.
 *  
 *  Parameters:
 *    
 *    flattenFileURL:
 *      A CFURLRef referencing a file that contains a flattened print
 *      settings.
 *    
 *    printSettings:
 *      On return, a newly created print settings object created with
 *      the data specified by 'flattenFileURL'. The caller is
 *      responsible for releasing the print settings created.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in ApplicationServices.framework but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMUnflattenPrintSettingsWithURL( flattenFileURL: CFURLRef; var printSettings: PMPrintSettings ): OSStatus; external name '_PMUnflattenPrintSettingsWithURL';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)
  
{
 *  PMGetPrintSettingsExtendedData()	*** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMPrintSettingsGetValue instead.
 *
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetPrintSettingsExtendedData( printSettings: PMPrintSettings; dataID: OSType; var size: UInt32; extendedData: UnivPtr ): OSStatus; external name '_PMGetPrintSettingsExtendedData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)

{
 *  PMSetPrintSettingsExtendedData()	*** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMPrintSettingsSetValue instead.
 *
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetPrintSettingsExtendedData( printSettings: PMPrintSettings; dataID: OSType; size: UInt32; extendedData: UnivPtr ): OSStatus; external name '_PMSetPrintSettingsExtendedData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{
 *  PMPaperGetName()
 *  
 *  Summary:
 *    Returns the name for a given paper.
 *  
 *  Discussion:
 *		Use PMPaperCreateLocalizedName instead
 *
 *  Parameters:
 *    
 *    paper:
 *      Obtain the name for this paper.
 *    
 *    paperName:
 *      If successful noErr is returned and *paperName is set to the
 *      name of the paper. The returned paper name is that appropriate
 *      to display to a user.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework but deprecated in 10.7
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 *  
 *  Compatibility:
 *  	Not appropriate for CUPS filters, drivers, and backends.
 *  
 }
function PMPaperGetName( paper: PMPaper; var paperName: CFStringRef ): OSStatus; external name '_PMPaperGetName';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)

{
 *  PMPrinterCreatePaperInfoListForMenu()
 *  
 *  Summary:
 *    Hand back a list of paper info items that can be used to create a
 *    paper size menu. Each item is a dictionary describing the menu item.
 *  
 *  Parameters:
 *    
 *    printer:
 *      The printer whose page size list should be enumerated. To get
 *      the session's current printer, see PMSessionGetCurrentPrinter().
 *    
 *    pageFormat:
 *      The page format object that contains the current paper size. 
 *    
 *    paperInfoList:
 *      If this function is successful then noErr will be returned and
 *      *'paperInfoList' will be set to a newly created CFArray. Each
 *      element in the array will be a CFDictionary describing the
 *      menu item that needs to be added for the paper size menu. If this
 *      function fails then a non-zero error code will be returned and
 *      *'paperInfoList' will be set to NULL.
 *
 }
function PMPrinterCreatePaperInfoListForMenu( printer: PMPrinter; pageFormat: PMPageFormat; paperInfoList: CFArrayRefPtr ): OSStatus; external name '_PMPrinterCreatePaperInfoListForMenu';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_7 *)

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
{$ifc not TARGET_CPU_64}

{
 * SPECIAL AVAILABILITY note: This routine is available in ApplicationsServices.framework in
 * Mac OS X version 10.0 and later. On Mac OS X it is available to CFM applications through CarbonLib
 * starting with Mac OS X version 10.2 and later.
 *
 * On Mac OS 8/9 using CarbonLib, this routine returns kPMNotImplemented
 }

{
 *  PMSetProfile()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Application must be rendering with QuickDraw for this to have
 *    effect. Use Quartz drawing instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetProfile( printSettings: PMPrintSettings; tag: PMTag; const (*var*) profile: CMProfileLocation ): OSStatus; external name '_PMSetProfile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{ Callbacks }
{***************************************}
{  All Idle UPP routines are deprecated }
{***************************************}
type
	PMIdleProcPtr = procedure;
	PMIdleUPP = PMIdleProcPtr;

{
 *  NewPMIdleUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function NewPMIdleUPP( userRoutine: PMIdleProcPtr ): PMIdleUPP; external name '_NewPMIdleUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposePMIdleUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure DisposePMIdleUPP( userUPP: PMIdleUPP ); external name '_DisposePMIdleUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokePMIdleUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure InvokePMIdleUPP( userUPP: PMIdleUPP ); external name '_InvokePMIdleUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{$ifc PM_USE_SESSION_APIS}

{
 *      @function PMSessionBeginDocumentNoDialog
 *      @discussion Use PMSessionBeginCGDocumentNoDialog instead.
 }
{
 *  PMSessionBeginDocumentNoDialog()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.6 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionBeginDocumentNoDialog( printSession: PMPrintSession; printSettings: PMPrintSettings; pageFormat: PMPageFormat ): OSStatus; external name '_PMSessionBeginDocumentNoDialog';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  PMSessionGetGraphicsContext()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMSessionGetCGGraphicsContext instead.
 *  
 *  Parameters:
 *    
 *    printSession:
 *      the session
 *    
 *    graphicsContextType:
 *      either kPMGraphicsContextQuickdraw or
 *      kPMGraphicsContextCoreGraphics
 *    
 *    graphicsContext:
 *      returns a GrafPtr or a CGContextRef
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionGetGraphicsContext( printSession: PMPrintSession; graphicsContextType: CFStringRef; var graphicsContext: UnivPtr ): OSStatus; external name '_PMSessionGetGraphicsContext';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  PMSessionEnableColorSync()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Application must be rendering with QuickDraw for this to have
 *    effect. Use Quartz drawing instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionEnableColorSync( printSession: PMPrintSession ): OSStatus; external name '_PMSessionEnableColorSync';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  PMSessionDisableColorSync()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Application must be rendering with QuickDraw for this to have
 *    effect. Use Quartz drawing instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionDisableColorSync( printSession: PMPrintSession ): OSStatus; external name '_PMSessionDisableColorSync';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  PMSessionSetIdleProc()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionSetIdleProc( printSession: PMPrintSession; idleProc: PMIdleUPP ): OSStatus; external name '_PMSessionSetIdleProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMSessionGeneral()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMPrinterGetCommInfo instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionGeneral( printSession: PMPrintSession; pData: Ptr ): OSStatus; external name '_PMSessionGeneral';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMSessionConvertOldPrintRecord()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionConvertOldPrintRecord( printSession: PMPrintSession; printRecordHandle: Handle; var printSettings: PMPrintSettings; var pageFormat: PMPageFormat ): OSStatus; external name '_PMSessionConvertOldPrintRecord';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMSessionMakeOldPrintRecord()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionMakeOldPrintRecord( printSession: PMPrintSession; printSettings: PMPrintSettings; pageFormat: PMPageFormat; var printRecordHandle: Handle ): OSStatus; external name '_PMSessionMakeOldPrintRecord';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMSessionSetCurrentPrinter()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMSessionSetCurrentPMPrinter after creating a PMPrinter with
 *    PMPrinterCreateFromPrinterID.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.1 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.4 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionSetCurrentPrinter( session: PMPrintSession; printerName: CFStringRef ): OSStatus; external name '_PMSessionSetCurrentPrinter';
(* AVAILABLE_MAC_OS_X_VERSION_10_1_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMSessionPostScriptBegin()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMPrinterPrintWithFile or PMPrinterPrintWithProvider instead.
 *
 *    For using EPS data together with other application drawing
 *    using Quartz, see PMCGImageCreateWithEPSDataProvider.
 *
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionPostScriptBegin( printSession: PMPrintSession ): OSStatus; external name '_PMSessionPostScriptBegin';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMSessionPostScriptEnd()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMPrinterPrintWithFile or PMPrinterPrintWithProvider instead.
 *
 *    For using EPS data together with other application drawing
 *    using Quartz, see PMCGImageCreateWithEPSDataProvider.
 *
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionPostScriptEnd( printSession: PMPrintSession ): OSStatus; external name '_PMSessionPostScriptEnd';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMSessionPostScriptHandle()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMPrinterPrintWithFile or PMPrinterPrintWithProvider instead.
 *
 *    For using EPS data together with other application drawing
 *    using Quartz, see PMCGImageCreateWithEPSDataProvider.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionPostScriptHandle( printSession: PMPrintSession; psHandle: Handle ): OSStatus; external name '_PMSessionPostScriptHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMSessionPostScriptData()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMPrinterPrintWithFile or PMPrinterPrintWithProvider instead.
 *
 *    For using EPS data together with other application drawing
 *    using Quartz, see PMCGImageCreateWithEPSDataProvider.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionPostScriptData( printSession: PMPrintSession; psPtr: Ptr; len: Size ): OSStatus; external name '_PMSessionPostScriptData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMSessionPostScriptFile()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMPrinterPrintWithFile or PMPrinterPrintWithProvider instead.
 *
 *    For using EPS data together with other application drawing
 *    using Quartz, see PMCGImageCreateWithEPSDataProvider.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionPostScriptFile( printSession: PMPrintSession; var psFile: FSSpec ): OSStatus; external name '_PMSessionPostScriptFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMSessionSetPSInjectionData()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMPrinterPrintWithFile or PMPrinterPrintWithProvider instead.
 *
 *    For using EPS data together with other application drawing
 *    using Quartz, see PMCGImageCreateWithEPSDataProvider.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionSetPSInjectionData( printSession: PMPrintSession; printSettings: PMPrintSettings; injectionDictArray: CFArrayRef ): OSStatus; external name '_PMSessionSetPSInjectionData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMSessionGetDocumentFormatSupported()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMPrinterPrintWithFile or PMPrinterPrintWithProvider instead.
 *
 *    For using EPS data together with other application drawing
 *    using Quartz, see PMCGImageCreateWithEPSDataProvider.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionGetDocumentFormatSupported( printSession: PMPrintSession; var docFormats: CFArrayRef; limit: UInt32 ): OSStatus; external name '_PMSessionGetDocumentFormatSupported';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMSessionGetDocumentFormatGeneration()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMPrinterPrintWithFile or PMPrinterPrintWithProvider instead.
 *
 *    For using EPS data together with other application drawing
 *    using Quartz, see PMCGImageCreateWithEPSDataProvider.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionGetDocumentFormatGeneration( printSession: PMPrintSession; var docFormats: CFArrayRef ): OSStatus; external name '_PMSessionGetDocumentFormatGeneration';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMSessionSetDocumentFormatGeneration()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Carbon applications using CoreGraphics instead of QuickDraw for
 *    all their drawing should use PMSessionBeginCGDocument or
 *    PMSessionBeginCGDocumentNoDialog. For generating PostScript data
 *    you should be using: PMPrinterPrintWithFile or
 *    PMPrinterPrintWithProvider. For mixing EPS data with Quartz drawing
 *    use PMCGImageCreateWithEPSDataProvider.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionSetDocumentFormatGeneration( printSession: PMPrintSession; docFormat: CFStringRef; graphicsContextTypes: CFArrayRef; options: CFTypeRef ): OSStatus; external name '_PMSessionSetDocumentFormatGeneration';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMSessionIsDocumentFormatSupported()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMPrinterPrintWithFile or PMPrinterPrintWithProvider instead.
 *
 *    For using EPS data together with other application drawing
 *    using Quartz, see PMCGImageCreateWithEPSDataProvider.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSessionIsDocumentFormatSupported( printSession: PMPrintSession; docFormat: CFStringRef; var supported: Boolean ): OSStatus; external name '_PMSessionIsDocumentFormatSupported';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{$elsec} {PM_USE_SESSION_APIS}

{
 *  PMSetIdleProc()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetIdleProc( idleProc: PMIdleUPP ): OSStatus; external name '_PMSetIdleProc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{ Print loop }
{
 *  PMBegin()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMCreateSession instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMBegin: OSStatus; external name '_PMBegin';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMEnd()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMEnd: OSStatus; external name '_PMEnd';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMGetGrafPtr()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMSessionGetCGGraphicsContext instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetGrafPtr( printContext: PMPrintContext; var grafPort: GrafPtr ): OSStatus; external name '_PMGetGrafPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{ PMPageFormat }
{
 *  PMNewPageFormat()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMCreatePageFormat instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMNewPageFormat( var pageFormat: PMPageFormat ): OSStatus; external name '_PMNewPageFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMDisposePageFormat()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMRelease instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMDisposePageFormat( pageFormat: PMPageFormat ): OSStatus; external name '_PMDisposePageFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMDefaultPageFormat()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMSessionDefaultPageFormat instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMDefaultPageFormat( pageFormat: PMPageFormat ): OSStatus; external name '_PMDefaultPageFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMValidatePageFormat()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMSessionValidatePageFormat instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMValidatePageFormat( pageFormat: PMPageFormat; var result: Boolean ): OSStatus; external name '_PMValidatePageFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{ PMPrintSettings }
{
 *  PMNewPrintSettings()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMCreatePrintSettings instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMNewPrintSettings( var printSettings: PMPrintSettings ): OSStatus; external name '_PMNewPrintSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMDisposePrintSettings()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMRelease instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMDisposePrintSettings( printSettings: PMPrintSettings ): OSStatus; external name '_PMDisposePrintSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMDefaultPrintSettings()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMSessionDefaultPrintSettings instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMDefaultPrintSettings( printSettings: PMPrintSettings ): OSStatus; external name '_PMDefaultPrintSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMValidatePrintSettings()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMSessionValidatePrintSettings instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMValidatePrintSettings( printSettings: PMPrintSettings; result: BooleanPtr ): OSStatus; external name '_PMValidatePrintSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{ Mac OS 9 Support }
{
 *  PMGeneral()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGeneral( pData: Ptr ): OSStatus; external name '_PMGeneral';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMConvertOldPrintRecord()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMConvertOldPrintRecord( printRecordHandle: Handle; var printSettings: PMPrintSettings; var pageFormat: PMPageFormat ): OSStatus; external name '_PMConvertOldPrintRecord';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMMakeOldPrintRecord()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMMakeOldPrintRecord( printSettings: PMPrintSettings; pageFormat: PMPageFormat; var printRecordHandle: Handle ): OSStatus; external name '_PMMakeOldPrintRecord';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{ Driver Information }
{
 *  PMIsPostScriptDriver()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMPrinterIsPostScriptCapable or PMPrinterIsPostScriptPrinter instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMIsPostScriptDriver( var isPostScript: Boolean ): OSStatus; external name '_PMIsPostScriptDriver';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMGetLanguageInfo()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMPrinterGetLanguageInfo instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetLanguageInfo( var info: PMLanguageInfo ): OSStatus; external name '_PMGetLanguageInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMGetDriverCreator()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMPrinterGetDriverCreator instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetDriverCreator( var creator: OSType ): OSStatus; external name '_PMGetDriverCreator';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMGetDriverReleaseInfo()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMPrinterGetDriverReleaseInfo instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetDriverReleaseInfo( var release: VersRec ): OSStatus; external name '_PMGetDriverReleaseInfo';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMGetPrinterResolutionCount()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMPrinterGetPrinterResolutionCount instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetPrinterResolutionCount( var count: UInt32 ): OSStatus; external name '_PMGetPrinterResolutionCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMGetPrinterResolution()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMPrinterGetPrinterResolution instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetPrinterResolution( tag: PMTag; var res: PMResolution ): OSStatus; external name '_PMGetPrinterResolution';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMGetIndexedPrinterResolution()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMPrinterGetIndexedPrinterResolution instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetIndexedPrinterResolution( index: UInt32; var res: PMResolution ): OSStatus; external name '_PMGetIndexedPrinterResolution';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{ ColorSync & PostScript Support }
{
 *  PMEnableColorSync()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use Quartz drawing instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMEnableColorSync: OSStatus; external name '_PMEnableColorSync';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMDisableColorSync()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use Quartz drawing instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMDisableColorSync: OSStatus; external name '_PMDisableColorSync';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMPostScriptBegin()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMPrinterPrintWithFile or PMPrinterPrintWithProvider instead.
 *
 *    For using EPS data together with other application drawing
 *    using Quartz, see PMCGImageCreateWithEPSDataProvider.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMPostScriptBegin: OSStatus; external name '_PMPostScriptBegin';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMPostScriptEnd()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMPrinterPrintWithFile or PMPrinterPrintWithProvider instead.
 *
 *    For using EPS data together with other application drawing
 *    using Quartz, see PMCGImageCreateWithEPSDataProvider.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMPostScriptEnd: OSStatus; external name '_PMPostScriptEnd';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMPostScriptHandle()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMPrinterPrintWithFile or PMPrinterPrintWithProvider instead.
 *
 *    For using EPS data together with other application drawing
 *    using Quartz, see PMCGImageCreateWithEPSDataProvider.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMPostScriptHandle( psHandle: Handle ): OSStatus; external name '_PMPostScriptHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMPostScriptData()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMPrinterPrintWithFile or PMPrinterPrintWithProvider instead.
 *
 *    For using EPS data together with other application drawing
 *    using Quartz, see PMCGImageCreateWithEPSDataProvider.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMPostScriptData( psPtr: Ptr; len: Size ): OSStatus; external name '_PMPostScriptData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMPostScriptFile()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMPrinterPrintWithFile or PMPrinterPrintWithProvider instead.
 *
 *    For using EPS data together with other application drawing
 *    using Quartz, see PMCGImageCreateWithEPSDataProvider.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMPostScriptFile( var psFile: FSSpec ): OSStatus; external name '_PMPostScriptFile';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{ Error }
{
 *  PMError()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMSessionError instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMError: OSStatus; external name '_PMError';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMSetError()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMSessionSetError instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetError( printError: OSStatus ): OSStatus; external name '_PMSetError';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{$endc} {PM_USE_SESSION_APIS}

{
 *  PMGetJobName()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMPrintSettingsGetJobName instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetJobName( printSettings: PMPrintSettings; name: StringPtr ): OSStatus; external name '_PMGetJobName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  PMSetJobName()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMPrintSettingsSetJobName instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetJobName( printSettings: PMPrintSettings; const name: Str255 ): OSStatus; external name '_PMSetJobName';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMGetPhysicalPaperSize()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMGetUnadjustedPaperRect instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetPhysicalPaperSize( pageFormat: PMPageFormat; var paperSize: PMRect ): OSStatus; external name '_PMGetPhysicalPaperSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMSetAdjustedPageRect() *** DEPRECATED ***
 *  
 *  Discussion:
 *    To set a particular paper size and margins create a PMPaper first then call
 *    PMCreatePageFormatWithPMPaper.
 *
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetAdjustedPageRect( pageFormat: PMPageFormat; const (*var*) pageRect: PMRect ): OSStatus; external name '_PMSetAdjustedPageRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  PMSetPhysicalPaperSize()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    To set a particular paper size create a PMPaper first then call
 *    PMCreatePageFormatWithPMPaper.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetPhysicalPaperSize( pageFormat: PMPageFormat; const (*var*) paperSize: PMRect ): OSStatus; external name '_PMSetPhysicalPaperSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  PMSetUnadjustedPaperRect()	 *** DEPRECATED ***
 *  
 *  Discussion:
 *    To set a particular paper size create a PMPaper first then call
 *    PMCreatePageFormatWithPMPaper.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetUnadjustedPaperRect( pageFormat: PMPageFormat; const (*var*) paperRect: PMRect ): OSStatus; external name '_PMSetUnadjustedPaperRect';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  PMGetPhysicalPageSize()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMGetUnadjustedPageRect instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetPhysicalPageSize( pageFormat: PMPageFormat; var pageSize: PMRect ): OSStatus; external name '_PMGetPhysicalPageSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMGetColorMode()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetColorMode( printSettings: PMPrintSettings; var colorMode: PMColorMode ): OSStatus; external name '_PMGetColorMode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMSetColorMode()   *** DEPRECATED ***
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetColorMode( printSettings: PMPrintSettings; colorMode: PMColorMode ): OSStatus; external name '_PMSetColorMode';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  PMGetDestination()	*** DEPRECATED ***
 *
 *  Discussion:
 *    Use PMSessionGetDestinationType, PMSessionCopyDestinationFormat, and/or PMSessionCopyDestinationLocation instead.
 *
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetDestination( printSettings: PMPrintSettings; var destType: PMDestinationType; var fileURL: CFURLRef ): OSStatus; external name '_PMGetDestination';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  PMPrinterGetDescriptionURL()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMPrinterCopyDescriptionURL instead. Please be aware that
 *    PMPrinterGetDescriptionURL will return a copy of the URL which
 *    you'll need to release.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetDescriptionURL( printer: PMPrinter; descriptionType: CFStringRef; var fileURL: CFURLRef ): OSStatus; external name '_PMPrinterGetDescriptionURL';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMPrinterGetDeviceURI()   *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMPrinterCopyDeviceURI instead. Please be aware that
 *    PMPrinterGetDeviceURI will return a copy of the URI which you'll
 *    need to release.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetDeviceURI( printer: PMPrinter; var deviceURI: CFURLRef ): OSStatus; external name '_PMPrinterGetDeviceURI';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  PMGetResolution()	*** DEPRECATED ***
 *  
 *  Summary:
 *    Obtains the current application drawing resolution.
 *  
 *  Discussion:
 *
 *    Use Quartz drawing and call CGContextScaleCTM instead.
 *
 *    This is the drawing resolution of an app. This should not be
 *    confused with the resolution of the printer. You can call
 *    PMGetPrinterResolution to see what resolutions are avaliable for
 *    the current printer.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetResolution( pageFormat: PMPageFormat; var res: PMResolution ): OSStatus; external name '_PMGetResolution';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  PMSetResolution()	*** DEPRECATED ***
 *  
 *  Discussion:
 *    Use Quartz drawing and call CGContextScaleCTM instead.
 *
 *    This sets the drawing resolution of an app. This should not be
 *    confused with the resolution of the printer.
 *
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetResolution( pageFormat: PMPageFormat; const (*var*) res: PMResolution ): OSStatus; external name '_PMSetResolution';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  PMFlattenPageFormat()    *** DEPRECATED ***
 *  
 *  Summary:
 *    Use PMPageFormatCreateDataRepresentation instead.
 *
 *    Flattens a PMPageFormat object for storage in a user document.
 *  
 *  Discussion:
 *    Flattening a page format should only be necessary if you intend
 *    to preserve the object settings along with a document. A page
 *    format will persist outside of a
 *    PMSessionBeginDocxxx/PMSessionEndDocxxx block. This will allow
 *    you to use any accessors on the object without the need to
 *    flatten and unflatten. Keep in mind accessors make no assumption
 *    on the validity of the value you set. This can only be done thru
 *    PMSessionValidatePageFormat in a
 *    PMSessionBeginDocxxx/PMSessionEndDocxxx block with a valid
 *    session.
 *  
 *  Parameters:
 *    
 *    pageFormat:
 *      A page format object.
 *    
 *    flatFormat:
 *      On return, a handle to a flattened PMPageFormat object. The
 *      handle is allocated by the function. You are responsible for
 *      disposing of the handle.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMFlattenPageFormat( pageFormat: PMPageFormat; var flatFormat: Handle ): OSStatus; external name '_PMFlattenPageFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  PMUnflattenPageFormat()    *** DEPRECATED ***
 *  
 *  Summary:
 *    Use PMPageFormatCreateWithDataRepresentation instead.
 *
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMUnflattenPageFormat( flatFormat: Handle; var pageFormat: PMPageFormat ): OSStatus; external name '_PMUnflattenPageFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  PMFlattenPrintSettings()    *** DEPRECATED ***
 *  
 *  Summary:
 *    Use PMPrintSettingsCreateDataRepresentation instead.
 *
 *    Flattens a PMPrintSettings object for storage in a user document.
 *  
 *  Discussion:
 *    Flattening a print settings should only be necessary if you
 *    intend to preserve the object settings along with a document. A
 *    print settings will persist outside of a
 *    PMSessionBeginDocxxx/PMSessionEndDocxxx block. This allows you to
 *    use any accessors on the object without the need to flatten and
 *    unflatten. Keep in mind the accessors make no assumption on the
 *    validity of the value. This can only be done thru
 *    PMSessionValidatePrintSettings in a
 *    PMSessionBeginDocxxx/PMSessionEndDocxxx block with a valid
 *    session.
 *  
 *  Parameters:
 *    
 *    printSettings:
 *      A print settings object.
 *    
 *    flatSettings:
 *      On return, a handle to a flattened PMPrintSettings object. The
 *      handle is allocated by the function. You are responsible for
 *      disposing of the handle.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMFlattenPrintSettings( printSettings: PMPrintSettings; var flatSettings: Handle ): OSStatus; external name '_PMFlattenPrintSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  PMUnflattenPrintSettings()    *** DEPRECATED ***
 *  
 *  Summary:
 *    Use PMPrintSettingsCreateWithDataRepresentation instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function PMUnflattenPrintSettings( flatSettings: Handle; var printSettings: PMPrintSettings ): OSStatus; external name '_PMUnflattenPrintSettings';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  PMGetJobNameCFString()    *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMPrintSettingsGetJobName instead.
 *    
 *    Please be aware that PMGetJobNameCFString will return a copy of 
 *    the job name which you'll need to release.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMGetJobNameCFString( printSettings: PMPrintSettings; var name: CFStringRef ): OSStatus; external name '_PMGetJobNameCFString';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  PMSetJobNameCFString()    *** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMPrintSettingsSetJobName instead.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMSetJobNameCFString( printSettings: PMPrintSettings; name: CFStringRef ): OSStatus; external name '_PMSetJobNameCFString';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)


{
 *  PMPrinterGetPrinterResolution()    *** DEPRECATED ***
 *
 *  Discussion:
 *    Use PMPrinterGetPrinterResolutionCount and PMPrinterGetIndexedPrinterResolution to examine the available printer resolutions.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function PMPrinterGetPrinterResolution( printer: PMPrinter; tag: PMTag; var res: PMResolution ): OSStatus; external name '_PMPrinterGetPrinterResolution';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{
 *  PMPaperCreate()			*** DEPRECATED ***
 *  
 *  Discussion:
 *    Use PMPrinterGetPaperList to list the PMPaper instances available for a given printer or use PMPaperCreateCustom
 *    to create a PMPaper that represents a custom paper instance.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.5
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function PMPaperCreate( printer: PMPrinter; id: CFStringRef; name: CFStringRef; width: Float64; height: Float64; const (*var*) margins: PMPaperMargins; var paperP: PMPaper ): OSStatus; external name '_PMPaperCreate';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_5 *)

{$endc} {not TARGET_CPU_64}

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
