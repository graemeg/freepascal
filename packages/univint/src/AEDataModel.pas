{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     File:       AE/AEDataModel.h
 
     Contains:   AppleEvent Data Model Interfaces.
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    
=======
     Version:    AppleEvents-496~1
>>>>>>> graemeg/cpstrnew
=======
     Version:    AppleEvents-496~1
>>>>>>> graemeg/cpstrnew
=======
     Version:    AppleEvents-496~1
>>>>>>> graemeg/cpstrnew
=======
     Version:    AppleEvents-496~1
>>>>>>> origin/cpstrnew
 
     Copyright:  © 1996-2008 by Apple Computer, Inc., all rights reserved
=======
=======
>>>>>>> origin/fixes_2_2
     File:       AEDataModel.p
=======
     File:       AE/AEDataModel.h
>>>>>>> origin/fixes_2.4
 
     Contains:   AppleEvent Data Model Interfaces.
 
     Version:    AppleEvents-496~1
 
<<<<<<< HEAD
     Copyright:  © 1996-2002 by Apple Computer, Inc., all rights reserved
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
     Copyright:  © 1996-2008 by Apple Computer, Inc., all rights reserved
>>>>>>> origin/fixes_2.4
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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
{  Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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

unit AEDataModel;
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
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
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
	{$setc TARGET_CPU_ARM64 := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
=======
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
>>>>>>> graemeg/cpstrnew
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
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
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
<<<<<<< HEAD
=======
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
uses MacTypes,MixedMode;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not MACOSALLINCLUDE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}
=======

>>>>>>> graemeg/cpstrnew

{$ifc TARGET_OS_MAC}
=======


{$ifc TARGET_OS_MAC}
>>>>>>> graemeg/cpstrnew
=======
=======
{$endc} {not MACOSALLINCLUDE}

>>>>>>> origin/fixes_2.4

{$ifc TARGET_OS_MAC}

{$ifc TARGET_OS_MAC}
>>>>>>> origin/cpstrnew
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2

{$ALIGN MAC68K}

{ Apple event descriptor types }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
const
	typeBoolean = FourCharCode('bool');
	typeChar = FourCharCode('TEXT'); { Deprecated, use typeUTF8Text instead. }

{
 * The following descriptor types are deprecated due to their lack of
 * explicit encoding or byte order definition.  Please use
 * typeUTF16ExternalRepresentation or typeUTF8Text instead. }
const
	typeStyledUnicodeText = FourCharCode('sutx'); { Not implemented }
	typeEncodedString = FourCharCode('encs'); { Not implemented }
	typeUnicodeText = FourCharCode('utxt'); { native byte ordering, optional BOM }
	typeCString = FourCharCode('cstr'); { MacRoman characters followed by a NULL byte }
	typePString = FourCharCode('pstr'); { Unsigned length byte followed by MacRoman characters }

{
 * The preferred unicode text types.  In both cases, there is no explicit null termination or length byte.
 }
const
	typeUTF16ExternalRepresentation = FourCharCode('ut16'); { big-endian 16 bit unicode with optional byte-order-mark, or little-endian 16 bit unicode with required byte-order-mark. }
	typeUTF8Text = FourCharCode('utf8'); { 8 bit unicode }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{ Preferred numeric Apple event descriptor types }
const
	typeSInt16 = FourCharCode('shor'); { SInt16 : signed, 16 bit integer }
	typeUInt16 = FourCharCode('ushr'); { UInt16 : unsigned, 16 bit integer }
	typeSInt32 = FourCharCode('long'); { SInt32 : signed, 32 bit integer }
	typeUInt32 = FourCharCode('magn'); { UInt32 : unsigned, 32 bit integer }
	typeSInt64 = FourCharCode('comp'); { SInt64 : signed, 64 bit integer }
	typeUInt64 = FourCharCode('ucom'); { UInt64 : unsigned, 64 bit integer }
	typeIEEE32BitFloatingPoint = FourCharCode('sing'); { float }
	typeIEEE64BitFloatingPoint = FourCharCode('doub'); { double }
	type128BitFloatingPoint = FourCharCode('ldbl');
	typeDecimalStruct = FourCharCode('decm');

{ Non-preferred Apple event descriptor types }
{$ifc TARGET_CPU_64}
{    On Mac OS X 64 bit, the following types have been removed because their meaning is unclear or confusing.
    For example, people have assumed that the appropriate data type for typeLongInteger would be long; but
    on 64 bit 'long' is a 64 bit value and the appropriate type should be typeComp.

    You should change your existing code to not use the following types, and use the more specific ones.  Check
    through your code to make sure that the datasize referenced by these types is the correct type.

        type constant       change to                   data type should be
        -------------       ---------                   -------------------
        typeSMInt           typeSInt16                  SInt16
        typeShortInteger    typeSInt16                  SInt16
        typeInteger         typeSInt32                  SInt32
        typeLongInteger     typeSInt32                  SInt32
        typeComp            typeSInt64                  SInt64 or Wide
        typeSMFloat         typeIEEE32BitFloatingPoint  Float32
        typeShortFloat      typeIEEE32BitFloatingPoint  Float32
        typeLongFloat       typeIEEE64BitFloatingPoint  Float64

    There is no good type on 64 bit to use for typeEntended; either change your code to pass typeIEEE64BitFloatingPoint
    for typeExtended and live with the reduction in range or use type128BitFloatingPoint.

=======

{ Preferred numeric Apple event descriptor types }
const
	typeSInt16 = FourCharCode('shor'); { SInt16 : signed, 16 bit integer }
	typeUInt16 = FourCharCode('ushr'); { UInt16 : unsigned, 16 bit integer }
	typeSInt32 = FourCharCode('long'); { SInt32 : signed, 32 bit integer }
	typeUInt32 = FourCharCode('magn'); { UInt32 : unsigned, 32 bit integer }
	typeSInt64 = FourCharCode('comp'); { SInt64 : signed, 64 bit integer }
	typeUInt64 = FourCharCode('ucom'); { UInt64 : unsigned, 64 bit integer }
	typeIEEE32BitFloatingPoint = FourCharCode('sing'); { float }
	typeIEEE64BitFloatingPoint = FourCharCode('doub'); { double }
	type128BitFloatingPoint = FourCharCode('ldbl');
	typeDecimalStruct = FourCharCode('decm');

{ Non-preferred Apple event descriptor types }
{$ifc TARGET_CPU_64}
{    On Mac OS X 64 bit, the following types have been removed because their meaning is unclear or confusing.
    For example, people have assumed that the appropriate data type for typeLongInteger would be long; but
    on 64 bit 'long' is a 64 bit value and the appropriate type should be typeComp.

    You should change your existing code to not use the following types, and use the more specific ones.  Check
    through your code to make sure that the datasize referenced by these types is the correct type.

        type constant       change to                   data type should be
        -------------       ---------                   -------------------
        typeSMInt           typeSInt16                  SInt16
        typeShortInteger    typeSInt16                  SInt16
        typeInteger         typeSInt32                  SInt32
        typeLongInteger     typeSInt32                  SInt32
        typeComp            typeSInt64                  SInt64 or Wide
        typeSMFloat         typeIEEE32BitFloatingPoint  Float32
        typeShortFloat      typeIEEE32BitFloatingPoint  Float32
        typeLongFloat       typeIEEE64BitFloatingPoint  Float64

    There is no good type on 64 bit to use for typeEntended; either change your code to pass typeIEEE64BitFloatingPoint
    for typeExtended and live with the reduction in range or use type128BitFloatingPoint.

>>>>>>> graemeg/cpstrnew
=======

{ Preferred numeric Apple event descriptor types }
const
	typeSInt16 = FourCharCode('shor'); { SInt16 : signed, 16 bit integer }
	typeUInt16 = FourCharCode('ushr'); { UInt16 : unsigned, 16 bit integer }
	typeSInt32 = FourCharCode('long'); { SInt32 : signed, 32 bit integer }
	typeUInt32 = FourCharCode('magn'); { UInt32 : unsigned, 32 bit integer }
	typeSInt64 = FourCharCode('comp'); { SInt64 : signed, 64 bit integer }
	typeUInt64 = FourCharCode('ucom'); { UInt64 : unsigned, 64 bit integer }
	typeIEEE32BitFloatingPoint = FourCharCode('sing'); { float }
	typeIEEE64BitFloatingPoint = FourCharCode('doub'); { double }
	type128BitFloatingPoint = FourCharCode('ldbl');
	typeDecimalStruct = FourCharCode('decm');

{ Non-preferred Apple event descriptor types }
{$ifc TARGET_CPU_64}
{    On Mac OS X 64 bit, the following types have been removed because their meaning is unclear or confusing.
    For example, people have assumed that the appropriate data type for typeLongInteger would be long; but
    on 64 bit 'long' is a 64 bit value and the appropriate type should be typeComp.

    You should change your existing code to not use the following types, and use the more specific ones.  Check
    through your code to make sure that the datasize referenced by these types is the correct type.

        type constant       change to                   data type should be
        -------------       ---------                   -------------------
        typeSMInt           typeSInt16                  SInt16
        typeShortInteger    typeSInt16                  SInt16
        typeInteger         typeSInt32                  SInt32
        typeLongInteger     typeSInt32                  SInt32
        typeComp            typeSInt64                  SInt64 or Wide
        typeSMFloat         typeIEEE32BitFloatingPoint  Float32
        typeShortFloat      typeIEEE32BitFloatingPoint  Float32
        typeLongFloat       typeIEEE64BitFloatingPoint  Float64

    There is no good type on 64 bit to use for typeEntended; either change your code to pass typeIEEE64BitFloatingPoint
    for typeExtended and live with the reduction in range or use type128BitFloatingPoint.

>>>>>>> graemeg/cpstrnew
=======

{ Preferred numeric Apple event descriptor types }
const
	typeSInt16 = FourCharCode('shor'); { SInt16 : signed, 16 bit integer }
	typeUInt16 = FourCharCode('ushr'); { UInt16 : unsigned, 16 bit integer }
	typeSInt32 = FourCharCode('long'); { SInt32 : signed, 32 bit integer }
	typeUInt32 = FourCharCode('magn'); { UInt32 : unsigned, 32 bit integer }
	typeSInt64 = FourCharCode('comp'); { SInt64 : signed, 64 bit integer }
	typeUInt64 = FourCharCode('ucom'); { UInt64 : unsigned, 64 bit integer }
	typeIEEE32BitFloatingPoint = FourCharCode('sing'); { float }
	typeIEEE64BitFloatingPoint = FourCharCode('doub'); { double }
	type128BitFloatingPoint = FourCharCode('ldbl');
	typeDecimalStruct = FourCharCode('decm');

{ Non-preferred Apple event descriptor types }
{$ifc TARGET_CPU_64}
{    On Mac OS X 64 bit, the following types have been removed because their meaning is unclear or confusing.
    For example, people have assumed that the appropriate data type for typeLongInteger would be long; but
    on 64 bit 'long' is a 64 bit value and the appropriate type should be typeComp.

    You should change your existing code to not use the following types, and use the more specific ones.  Check
    through your code to make sure that the datasize referenced by these types is the correct type.

        type constant       change to                   data type should be
        -------------       ---------                   -------------------
        typeSMInt           typeSInt16                  SInt16
        typeShortInteger    typeSInt16                  SInt16
        typeInteger         typeSInt32                  SInt32
        typeLongInteger     typeSInt32                  SInt32
        typeComp            typeSInt64                  SInt64 or Wide
        typeSMFloat         typeIEEE32BitFloatingPoint  Float32
        typeShortFloat      typeIEEE32BitFloatingPoint  Float32
        typeLongFloat       typeIEEE64BitFloatingPoint  Float64

    There is no good type on 64 bit to use for typeEntended; either change your code to pass typeIEEE64BitFloatingPoint
    for typeExtended and live with the reduction in range or use type128BitFloatingPoint.

>>>>>>> origin/cpstrnew
}
{$elsec} {TARGET_CPU_64}
=======
=======
>>>>>>> origin/fixes_2_2

=======
>>>>>>> origin/fixes_2.4
const
	typeBoolean = FourCharCode('bool');
	typeChar = FourCharCode('TEXT'); { Deprecated, use typeUTF8Text instead. }

{
 * The following descriptor types are deprecated due to their lack of
 * explicit encoding or byte order definition.  Please use
 * typeUTF16ExternalRepresentation or typeUTF8Text instead. }
const
	typeStyledUnicodeText = FourCharCode('sutx'); { Not implemented }
	typeEncodedString = FourCharCode('encs'); { Not implemented }
	typeUnicodeText = FourCharCode('utxt'); { native byte ordering, optional BOM }
	typeCString = FourCharCode('cstr'); { MacRoman characters followed by a NULL byte }
	typePString = FourCharCode('pstr'); { Unsigned length byte followed by MacRoman characters }

{
 * The preferred unicode text types.  In both cases, there is no explicit null termination or length byte.
 }
const
	typeUTF16ExternalRepresentation = FourCharCode('ut16'); { big-endian 16 bit unicode with optional byte-order-mark, or little-endian 16 bit unicode with required byte-order-mark. }
	typeUTF8Text = FourCharCode('utf8'); { 8 bit unicode }

{ Preferred numeric Apple event descriptor types }
const
	typeSInt16 = FourCharCode('shor'); { SInt16 : signed, 16 bit integer }
	typeUInt16 = FourCharCode('ushr'); { UInt16 : unsigned, 16 bit integer }
	typeSInt32 = FourCharCode('long'); { SInt32 : signed, 32 bit integer }
	typeUInt32 = FourCharCode('magn'); { UInt32 : unsigned, 32 bit integer }
	typeSInt64 = FourCharCode('comp'); { SInt64 : signed, 64 bit integer }
	typeUInt64 = FourCharCode('ucom'); { UInt64 : unsigned, 64 bit integer }
	typeIEEE32BitFloatingPoint = FourCharCode('sing'); { float }
	typeIEEE64BitFloatingPoint = FourCharCode('doub'); { double }
	type128BitFloatingPoint = FourCharCode('ldbl');
	typeDecimalStruct = FourCharCode('decm');

{ Non-preferred Apple event descriptor types }
{$ifc TARGET_CPU_64}
{    On Mac OS X 64 bit, the following types have been removed because their meaning is unclear or confusing.
    For example, people have assumed that the appropriate data type for typeLongInteger would be long; but
    on 64 bit 'long' is a 64 bit value and the appropriate type should be typeComp.

    You should change your existing code to not use the following types, and use the more specific ones.  Check
    through your code to make sure that the datasize referenced by these types is the correct type.

        type constant       change to                   data type should be
        -------------       ---------                   -------------------
        typeSMInt           typeSInt16                  SInt16
        typeShortInteger    typeSInt16                  SInt16
        typeInteger         typeSInt32                  SInt32
        typeLongInteger     typeSInt32                  SInt32
        typeComp            typeSInt64                  SInt64 or Wide
        typeSMFloat         typeIEEE32BitFloatingPoint  Float32
        typeShortFloat      typeIEEE32BitFloatingPoint  Float32
        typeLongFloat       typeIEEE64BitFloatingPoint  Float64

    There is no good type on 64 bit to use for typeEntended; either change your code to pass typeIEEE64BitFloatingPoint
    for typeExtended and live with the reduction in range or use type128BitFloatingPoint.

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
}
{$elsec} {TARGET_CPU_64}
>>>>>>> origin/fixes_2.4
	{	 Non-preferred Apple event descriptor types 	}
	typeSMInt					= FourCharCode('shor');
	typeShortInteger			= FourCharCode('shor');
	typeInteger					= FourCharCode('long');
	typeLongInteger				= FourCharCode('long');
	typeMagnitude				= FourCharCode('magn');
	typeComp					= FourCharCode('comp');
	typeSMFloat					= FourCharCode('sing');
	typeShortFloat				= FourCharCode('sing');
	typeFloat					= FourCharCode('doub');
	typeLongFloat				= FourCharCode('doub');
	typeExtended				= FourCharCode('exte');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {TARGET_CPU_64}
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/cpstrnew
=======
{$endc} {TARGET_CPU_64}
>>>>>>> origin/fixes_2.4

{ More Apple event descriptor types }
const
	typeAEList = FourCharCode('list');
	typeAERecord = FourCharCode('reco');
	typeAppleEvent = FourCharCode('aevt');
	typeEventRecord = FourCharCode('evrc');
	typeTrue = FourCharCode('true');
	typeFalse = FourCharCode('fals');
	typeAlias = FourCharCode('alis'); { AliasPtr, from a valid AliasHandle }
	typeEnumerated = FourCharCode('enum');
	typeType = FourCharCode('type'); { OSType }
	typeAppParameters = FourCharCode('appa');
	typeProperty = FourCharCode('prop');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	typeFSRef = FourCharCode('fsrf'); { FSRef.  Deprecated; use typeFileURL or typeBookmark data to refer to files in AppleEvents }
	typeFileURL = FourCharCode('furl'); { a UTF-8 encoded full path, using native path separators }
	typeBookmarkData = FourCharCode('bmrk'); { the bytes of a CFURLBookmarkData }
=======
	typeFSRef = FourCharCode('fsrf'); { FSRef }
	typeFileURL = FourCharCode('furl');
	typeBookmarkData = FourCharCode('bmrk');
>>>>>>> graemeg/cpstrnew
=======
	typeFSRef = FourCharCode('fsrf'); { FSRef }
	typeFileURL = FourCharCode('furl');
	typeBookmarkData = FourCharCode('bmrk');
>>>>>>> graemeg/cpstrnew
=======
	typeFSRef = FourCharCode('fsrf'); { FSRef }
	typeFileURL = FourCharCode('furl');
	typeBookmarkData = FourCharCode('bmrk');
>>>>>>> origin/cpstrnew
=======
	typeFSRef = FourCharCode('fsrf'); { FSRef }
	typeFileURL = FourCharCode('furl');
	typeBookmarkData = FourCharCode('bmrk');
>>>>>>> origin/fixes_2.4
	typeKeyword = FourCharCode('keyw'); { OSType }
	typeSectionH = FourCharCode('sect');
	typeWildCard = FourCharCode('****');
	typeApplSignature = FourCharCode('sign'); { OSType }
	typeQDRectangle = FourCharCode('qdrt');
	typeFixed = FourCharCode('fixd');
	typeProcessSerialNumber = FourCharCode('psn '); { ProcessSerialNumber }
	typeApplicationURL = FourCharCode('aprl');
	typeNull = FourCharCode('null'); { null or nonexistent data }

{$ifc not TARGET_CPU_64}
{
    FSSpecs are deprecated on Mac OS X, and their use in AppleEvents is discouraged.  You should change
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    your code to use typeFileURL or typeFileBookmark.  In __LP64__ code, coercions into typeFSS is not supported,
=======
    your code to use FSRefs.  In __LP64__ code, coercions into typeFSS is not supported,
>>>>>>> graemeg/cpstrnew
=======
    your code to use FSRefs.  In __LP64__ code, coercions into typeFSS is not supported,
>>>>>>> graemeg/cpstrnew
=======
    your code to use FSRefs.  In __LP64__ code, coercions into typeFSS is not supported,
>>>>>>> origin/cpstrnew
=======
    your code to use FSRefs.  In __LP64__ code, coercions into typeFSS is not supported,
>>>>>>> origin/fixes_2.4
    and coercion from typeFSS is not guaranteed to work correctly in all cases.
}
const
	typeFSS = FourCharCode('fss '); { FSSpec }

{$endc} {not TARGET_CPU_64}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

const
	typeCFAttributedStringRef = FourCharCode('cfas');
	typeCFMutableAttributedStringRef = FourCharCode('cfaa');
	typeCFStringRef = FourCharCode('cfst');
	typeCFMutableStringRef = FourCharCode('cfms');
	typeCFArrayRef = FourCharCode('cfar');
	typeCFMutableArrayRef = FourCharCode('cfma');
	typeCFDictionaryRef = FourCharCode('cfdc');
	typeCFMutableDictionaryRef = FourCharCode('cfmd');
	typeCFNumberRef = FourCharCode('cfnb');
	typeCFBooleanRef = FourCharCode('cftf');
	typeCFTypeRef = FourCharCode('cfty');

{ New addressing modes for MacOS X }
const
	typeKernelProcessID = FourCharCode('kpid');
	typeMachPort = FourCharCode('port');

	
const
	typeAuditToken = FourCharCode('tokn');	{ Mac OS X 10.8, returned as keyAuditTokenAttr and is a typedef audit_token_t }
										 
{ Targeting applications by bundle ID is only available in Mac OS X 10.3 or later. }
const
	typeApplicationBundleID = FourCharCode('bund');

{ Keywords for Apple event attributes }
const
	keyTransactionIDAttr = FourCharCode('tran'); { AETransactionID }
	keyReturnIDAttr = FourCharCode('rtid'); { AEReturnID }
	keyEventClassAttr = FourCharCode('evcl'); { AEEventClass }
	keyEventIDAttr = FourCharCode('evid'); { AEEventID }
	keyAddressAttr = FourCharCode('addr');
	keyOptionalKeywordAttr = FourCharCode('optk');
	keyTimeoutAttr = FourCharCode('timo'); { SInt32 }
	keyInteractLevelAttr = FourCharCode('inte'); { this attribute is read only - will be set in AESend }
	keyEventSourceAttr = FourCharCode('esrc'); { this attribute is read only - returned as typeShortInteger }
	keyMissedKeywordAttr = FourCharCode('miss'); { this attribute is read only }
	keyOriginalAddressAttr = FourCharCode('from'); { new in 1.0.1 }
	keyAcceptTimeoutAttr = FourCharCode('actm'); { new for Mac OS X }
	keyReplyRequestedAttr = FourCharCode('repq'); { Was a reply requested for this event - returned as typeBoolean }
	keySenderEUIDAttr = FourCharCode('seid'); { read only, returned as typeSInt32.  Will be the euid of the sender of this event. } { Mac OS X 10.6 or later }
	keySenderEGIDAttr = FourCharCode('sgid'); { read only, returned as typeSInt32.  Will be the egid of the sender of this event. } { Mac OS X 10.6 or later }
	keySenderUIDAttr = FourCharCode('uids'); { read only, returned as typeSInt32.  Will be the uid of the sender of this event. } { Mac OS X 10.6 or later }
	keySenderGIDAttr = FourCharCode('gids'); { read only, returned as typeSInt32.  Will be the gid of the sender of this event. } { Mac OS X 10.6 or later }
	keySenderPIDAttr = FourCharCode('spid'); { read only, returned as typeSInt32.  Will be the pid of the sender of this event. } { Mac OS X 10.6 or later }
	keySenderAuditTokenAttr = FourCharCode('tokn'); { read only, returned as an audit_token_t.  Will be the audit token of the sender of this event. } { Mac OS X 10.6 or later }
	keySenderApplescriptEntitlementsAttr = FourCharCode('entl'); { read only, an AEDesc containing opaque data representing the entitlements held by the sender. Interpreted by sandbox routines. } { Mac OS X 10.8 or later }
	keySenderApplicationIdentifierEntitlementAttr = FourCharCode('aiea');
	keySenderApplicationSandboxed = FourCharCode('sssb'); { read-only, an AEDesc typeBoolean, true if the sender application was in an application sandbox } { Mac OS X 10.8 or later }
	keyActualSenderAuditToken = FourCharCode('acat'); { read-only, an AEDesc typeAuditToken of the acual ( possibly over-ridden ) audit token for the sender of this event } { Mac OS X 10.8 or later }

{ These bits are specified in the keyXMLDebuggingAttr (an SInt32) }
const
	kAEDebugPOSTHeader = 1 shl 0; { headers of the HTTP post we sent - typeChar }
	kAEDebugReplyHeader = 1 shl 1; { headers returned by the server }
	kAEDebugXMLRequest = 1 shl 2; { the XML request we sent }
	kAEDebugXMLResponse = 1 shl 3; { the XML reply from the server }
	kAEDebugXMLDebugAll = $FFFFFFFF; { everything! }

{ These values can be added as a parameter to the direct object of a
   SOAP message to specify the serialization schema.  If not
   specified, kSOAP1999Schema is the default. These should be added as
   typeType. }
const
	kSOAP1999Schema = FourCharCode('ss99');
	kSOAP2001Schema = FourCharCode('ss01');

const
=======

=======

>>>>>>> graemeg/cpstrnew
=======

{ More Apple event descriptor types }
const
	typeAEList = FourCharCode('list');
	typeAERecord = FourCharCode('reco');
	typeAppleEvent = FourCharCode('aevt');
	typeEventRecord = FourCharCode('evrc');
	typeTrue = FourCharCode('true');
	typeFalse = FourCharCode('fals');
	typeAlias = FourCharCode('alis'); { AliasPtr, from a valid AliasHandle }
	typeEnumerated = FourCharCode('enum');
	typeType = FourCharCode('type'); { OSType }
	typeAppParameters = FourCharCode('appa');
	typeProperty = FourCharCode('prop');
	typeFSRef = FourCharCode('fsrf'); { FSRef }
	typeFileURL = FourCharCode('furl');
	typeBookmarkData = FourCharCode('bmrk');
	typeKeyword = FourCharCode('keyw'); { OSType }
	typeSectionH = FourCharCode('sect');
	typeWildCard = FourCharCode('****');
	typeApplSignature = FourCharCode('sign'); { OSType }
	typeQDRectangle = FourCharCode('qdrt');
	typeFixed = FourCharCode('fixd');
	typeProcessSerialNumber = FourCharCode('psn '); { ProcessSerialNumber }
	typeApplicationURL = FourCharCode('aprl');
	typeNull = FourCharCode('null'); { null or nonexistent data }

{$ifc not TARGET_CPU_64}
{
    FSSpecs are deprecated on Mac OS X, and their use in AppleEvents is discouraged.  You should change
    your code to use FSRefs.  In __LP64__ code, coercions into typeFSS is not supported,
    and coercion from typeFSS is not guaranteed to work correctly in all cases.
}
const
	typeFSS = FourCharCode('fss '); { FSSpec }

{$endc} {not TARGET_CPU_64}

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
const
	typeCFAttributedStringRef = FourCharCode('cfas');
	typeCFMutableAttributedStringRef = FourCharCode('cfaa');
	typeCFStringRef = FourCharCode('cfst');
	typeCFMutableStringRef = FourCharCode('cfms');
	typeCFArrayRef = FourCharCode('cfar');
	typeCFMutableArrayRef = FourCharCode('cfma');
	typeCFDictionaryRef = FourCharCode('cfdc');
	typeCFMutableDictionaryRef = FourCharCode('cfmd');
	typeCFNumberRef = FourCharCode('cfnb');
	typeCFBooleanRef = FourCharCode('cftf');
	typeCFTypeRef = FourCharCode('cfty');

{ New addressing modes for MacOS X }
const
	typeKernelProcessID = FourCharCode('kpid');
	typeMachPort = FourCharCode('port');

{ Targeting applications by bundle ID is only available in Mac OS X 10.3 or later. }
const
	typeApplicationBundleID = FourCharCode('bund');

{ Keywords for Apple event attributes }
const
	keyTransactionIDAttr = FourCharCode('tran'); { AETransactionID }
	keyReturnIDAttr = FourCharCode('rtid'); { AEReturnID }
	keyEventClassAttr = FourCharCode('evcl'); { AEEventClass }
	keyEventIDAttr = FourCharCode('evid'); { AEEventID }
	keyAddressAttr = FourCharCode('addr');
	keyOptionalKeywordAttr = FourCharCode('optk');
	keyTimeoutAttr = FourCharCode('timo'); { SInt32 }
	keyInteractLevelAttr = FourCharCode('inte'); { this attribute is read only - will be set in AESend }
	keyEventSourceAttr = FourCharCode('esrc'); { this attribute is read only - returned as typeShortInteger }
	keyMissedKeywordAttr = FourCharCode('miss'); { this attribute is read only }
	keyOriginalAddressAttr = FourCharCode('from'); { new in 1.0.1 }
	keyAcceptTimeoutAttr = FourCharCode('actm'); { new for Mac OS X }
	keyReplyRequestedAttr = FourCharCode('repq'); { Was a reply requested for this event - returned as typeBoolean }
	keySenderEUIDAttr = FourCharCode('seid'); { read only, returned as typeSInt32.  Will be the euid of the sender of this event. }
	keySenderEGIDAttr = FourCharCode('sgid'); { read only, returned as typeSInt32.  Will be the egid of the sender of this event. }
	keySenderUIDAttr = FourCharCode('uids'); { read only, returned as typeSInt32.  Will be the uid of the sender of this event. }
	keySenderGIDAttr = FourCharCode('gids'); { read only, returned as typeSInt32.  Will be the gid of the sender of this event. }
	keySenderPIDAttr = FourCharCode('spid'); { read only, returned as typeSInt32.  Will be the pid of the sender of this event. }

{ These bits are specified in the keyXMLDebuggingAttr (an SInt32) }
const
	kAEDebugPOSTHeader = 1 shl 0; { headers of the HTTP post we sent - typeChar }
	kAEDebugReplyHeader = 1 shl 1; { headers returned by the server }
	kAEDebugXMLRequest = 1 shl 2; { the XML request we sent }
	kAEDebugXMLResponse = 1 shl 3; { the XML reply from the server }
	kAEDebugXMLDebugAll = $FFFFFFFF; { everything! }

{ These values can be added as a parameter to the direct object of a
   SOAP message to specify the serialization schema.  If not
   specified, kSOAP1999Schema is the default. These should be added as
   typeType. }
const
	kSOAP1999Schema = FourCharCode('ss99');
	kSOAP2001Schema = FourCharCode('ss01');

const
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
{ outgoing event attributes }
	keyUserNameAttr = FourCharCode('unam');
	keyUserPasswordAttr = FourCharCode('pass'); { not sent with the event }
	keyDisableAuthenticationAttr = FourCharCode('auth'); { When present and with a non zero value (that is, false, or integer 0), }
                                        { AESend will not authenticate the user.  If not present, or with a non-zero}
                                        { value, AESend will prompt for authentication information from the user if the interaction level allows. }
	keyXMLDebuggingAttr = FourCharCode('xdbg'); { a bitfield of specifying which XML debugging data is to be returned with the event }
                                        { Event class / id }
	kAERPCClass = FourCharCode('rpc '); { for outgoing XML events }
	kAEXMLRPCScheme = FourCharCode('RPC2'); { event ID: event should be sent to an XMLRPC endpoint }
	kAESOAPScheme = FourCharCode('SOAP'); { event ID: event should be sent to a SOAP endpoint }
	kAESharedScriptHandler = FourCharCode('wscp'); { event ID: handler for incoming XML requests }
                                        { these parameters exist as part of the direct object of the event for both incoming and outgoing requests }
	keyRPCMethodName = FourCharCode('meth'); { name of the method to call }
	keyRPCMethodParam = FourCharCode('parm'); { the list (or structure) of parameters }
	keyRPCMethodParamOrder = FourCharCode('/ord'); { if a structure, the order of parameters (a list) }
                                        { when keyXMLDebugginAttr so specifies, these additional parameters will be part of the reply. }
	keyAEPOSTHeaderData = FourCharCode('phed'); { what we sent to the server }
	keyAEReplyHeaderData = FourCharCode('rhed'); { what the server sent to us }
	keyAEXMLRequestData = FourCharCode('xreq'); { what we sent to the server }
	keyAEXMLReplyData = FourCharCode('xrep'); { what the server sent to us }
                                        { additional parameters that can be specified in the direct object of the event }
	keyAdditionalHTTPHeaders = FourCharCode('ahed'); { list of additional HTTP headers (a list of 2 element lists) }
	keySOAPAction = FourCharCode('sact'); { the SOAPAction header (required for SOAP messages) }
	keySOAPMethodNameSpace = FourCharCode('mspc'); { Optional namespace (defaults to m:) }
	keySOAPMethodNameSpaceURI = FourCharCode('mspu'); { Required namespace URI }
	keySOAPSchemaVersion = FourCharCode('ssch'); { Optional XML Schema version, defaults to kSOAP1999Schama }
=======
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

const
	typeCFAttributedStringRef = FourCharCode('cfas');
	typeCFMutableAttributedStringRef = FourCharCode('cfaa');
	typeCFStringRef = FourCharCode('cfst');
	typeCFMutableStringRef = FourCharCode('cfms');
	typeCFArrayRef = FourCharCode('cfar');
	typeCFMutableArrayRef = FourCharCode('cfma');
	typeCFDictionaryRef = FourCharCode('cfdc');
	typeCFMutableDictionaryRef = FourCharCode('cfmd');
	typeCFNumberRef = FourCharCode('cfnb');
	typeCFBooleanRef = FourCharCode('cftf');
	typeCFTypeRef = FourCharCode('cfty');

{ New addressing modes for MacOS X }
const
	typeKernelProcessID = FourCharCode('kpid');
	typeMachPort = FourCharCode('port');

{ Targeting applications by bundle ID is only available in Mac OS X 10.3 or later. }
const
	typeApplicationBundleID = FourCharCode('bund');

{ Keywords for Apple event attributes }
const
	keyTransactionIDAttr = FourCharCode('tran'); { AETransactionID }
	keyReturnIDAttr = FourCharCode('rtid'); { AEReturnID }
	keyEventClassAttr = FourCharCode('evcl'); { AEEventClass }
	keyEventIDAttr = FourCharCode('evid'); { AEEventID }
	keyAddressAttr = FourCharCode('addr');
	keyOptionalKeywordAttr = FourCharCode('optk');
	keyTimeoutAttr = FourCharCode('timo'); { SInt32 }
	keyInteractLevelAttr = FourCharCode('inte'); { this attribute is read only - will be set in AESend }
	keyEventSourceAttr = FourCharCode('esrc'); { this attribute is read only - returned as typeShortInteger }
	keyMissedKeywordAttr = FourCharCode('miss'); { this attribute is read only }
	keyOriginalAddressAttr = FourCharCode('from'); { new in 1.0.1 }
	keyAcceptTimeoutAttr = FourCharCode('actm'); { new for Mac OS X }
	keyReplyRequestedAttr = FourCharCode('repq'); { Was a reply requested for this event - returned as typeBoolean }
	keySenderEUIDAttr = FourCharCode('seid'); { read only, returned as typeSInt32.  Will be the euid of the sender of this event. }
	keySenderEGIDAttr = FourCharCode('sgid'); { read only, returned as typeSInt32.  Will be the egid of the sender of this event. }
	keySenderUIDAttr = FourCharCode('uids'); { read only, returned as typeSInt32.  Will be the uid of the sender of this event. }
	keySenderGIDAttr = FourCharCode('gids'); { read only, returned as typeSInt32.  Will be the gid of the sender of this event. }
	keySenderPIDAttr = FourCharCode('spid'); { read only, returned as typeSInt32.  Will be the pid of the sender of this event. }

{ These bits are specified in the keyXMLDebuggingAttr (an SInt32) }
const
	kAEDebugPOSTHeader = 1 shl 0; { headers of the HTTP post we sent - typeChar }
	kAEDebugReplyHeader = 1 shl 1; { headers returned by the server }
	kAEDebugXMLRequest = 1 shl 2; { the XML request we sent }
	kAEDebugXMLResponse = 1 shl 3; { the XML reply from the server }
	kAEDebugXMLDebugAll = $FFFFFFFF; { everything! }

{ These values can be added as a parameter to the direct object of a
   SOAP message to specify the serialization schema.  If not
   specified, kSOAP1999Schema is the default. These should be added as
   typeType. }
const
	kSOAP1999Schema = FourCharCode('ss99');
	kSOAP2001Schema = FourCharCode('ss01');

const
{ outgoing event attributes }
	keyUserNameAttr = FourCharCode('unam');
	keyUserPasswordAttr = FourCharCode('pass'); { not sent with the event }
	keyDisableAuthenticationAttr = FourCharCode('auth'); { When present and with a non zero value (that is, false, or integer 0), }
<<<<<<< HEAD
                                                           { AESend will not authenticate the user.  If not present, or with a non-zero}
                                                           { value, AESend will prompt for authentication information from the user if the interaction level allows. }
	keyXMLDebuggingAttr         = FourCharCode('xdbg'); { a bitfield of specifying which XML debugging data is to be returned with the event }
                                                          { Event class / id }
	kAERPCClass                 = FourCharCode('rpc '); { for outgoing XML events }
	kAEXMLRPCScheme             = FourCharCode('RPC2'); { event ID: event should be sent to an XMLRPC endpoint }
	kAESOAPScheme               = FourCharCode('SOAP'); { event ID: event should be sent to a SOAP endpoint }
	kAESharedScriptHandler      = FourCharCode('wscp'); { event ID: handler for incoming XML requests }
                                                          { these parameters exist as part of the direct object of the event for both incoming and outgoing requests }
	keyRPCMethodName            = FourCharCode('meth'); { name of the method to call }
	keyRPCMethodParam           = FourCharCode('parm'); { the list (or structure) of parameters }
	keyRPCMethodParamOrder      = FourCharCode('/ord'); { if a structure, the order of parameters (a list) }
                                                          { when keyXMLDebugginAttr so specifies, these additional parameters will be part of the reply. }
	keyAEPOSTHeaderData         = FourCharCode('phed'); { what we sent to the server }
	keyAEReplyHeaderData        = FourCharCode('rhed'); { what the server sent to us }
	keyAEXMLRequestData         = FourCharCode('xreq'); { what we sent to the server }
	keyAEXMLReplyData           = FourCharCode('xrep'); { what the server sent to us }
                                                          { additional parameters that can be specified in the direct object of the event }
	keyAdditionalHTTPHeaders    = FourCharCode('ahed'); { list of additional HTTP headers (a list of 2 element lists) }
	keySOAPAction               = FourCharCode('sact'); { the SOAPAction header (required for SOAP messages) }
	keySOAPMethodNameSpace      = FourCharCode('mspc'); { Optional namespace (defaults to m:) }
	keySOAPMethodNameSpaceURI   = FourCharCode('mspu'); { Required namespace URI }
	keySOAPSchemaVersion        = FourCharCode('ssch'); { Optional XML Schema version, defaults to kSOAP1999Schama }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
                                        { AESend will not authenticate the user.  If not present, or with a non-zero}
                                        { value, AESend will prompt for authentication information from the user if the interaction level allows. }
	keyXMLDebuggingAttr = FourCharCode('xdbg'); { a bitfield of specifying which XML debugging data is to be returned with the event }
                                        { Event class / id }
	kAERPCClass = FourCharCode('rpc '); { for outgoing XML events }
	kAEXMLRPCScheme = FourCharCode('RPC2'); { event ID: event should be sent to an XMLRPC endpoint }
	kAESOAPScheme = FourCharCode('SOAP'); { event ID: event should be sent to a SOAP endpoint }
	kAESharedScriptHandler = FourCharCode('wscp'); { event ID: handler for incoming XML requests }
                                        { these parameters exist as part of the direct object of the event for both incoming and outgoing requests }
	keyRPCMethodName = FourCharCode('meth'); { name of the method to call }
	keyRPCMethodParam = FourCharCode('parm'); { the list (or structure) of parameters }
	keyRPCMethodParamOrder = FourCharCode('/ord'); { if a structure, the order of parameters (a list) }
                                        { when keyXMLDebugginAttr so specifies, these additional parameters will be part of the reply. }
	keyAEPOSTHeaderData = FourCharCode('phed'); { what we sent to the server }
	keyAEReplyHeaderData = FourCharCode('rhed'); { what the server sent to us }
	keyAEXMLRequestData = FourCharCode('xreq'); { what we sent to the server }
	keyAEXMLReplyData = FourCharCode('xrep'); { what the server sent to us }
                                        { additional parameters that can be specified in the direct object of the event }
	keyAdditionalHTTPHeaders = FourCharCode('ahed'); { list of additional HTTP headers (a list of 2 element lists) }
	keySOAPAction = FourCharCode('sact'); { the SOAPAction header (required for SOAP messages) }
	keySOAPMethodNameSpace = FourCharCode('mspc'); { Optional namespace (defaults to m:) }
	keySOAPMethodNameSpaceURI = FourCharCode('mspu'); { Required namespace URI }
	keySOAPSchemaVersion = FourCharCode('ssch'); { Optional XML Schema version, defaults to kSOAP1999Schama }
>>>>>>> origin/fixes_2.4

{ 
   When serializing AERecords as SOAP structures, it is possible
   to specify the namespace and type of the structure.  To do this,
   add a keySOAPStructureMetaData record to the top level of the
   record to be serialized.  If present, this will be used to specify
   the structure namespace.  This will produce a structure elment that
   looks like:

    <myStruct
        xmlns:myNamespace="http://myUri.org/xsd",
        xsi:type="myNamespace:MyStructType">
        ...
    </myStruct>

}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
const
	keySOAPStructureMetaData = FourCharCode('/smd');
	keySOAPSMDNamespace = FourCharCode('ssns'); { "myNamespace"}
	keySOAPSMDNamespaceURI = FourCharCode('ssnu'); { "http://myUri.org/xsd"}
	keySOAPSMDType = FourCharCode('sstp'); { "MyStructType"}
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
	keySOAPStructureMetaData    = FourCharCode('/smd');
	keySOAPSMDNamespace         = FourCharCode('ssns'); { "myNamespace"}
	keySOAPSMDNamespaceURI      = FourCharCode('ssnu'); { "http://myUri.org/xsd"}
	keySOAPSMDType              = FourCharCode('sstp'); { "MyStructType"}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{ 
 * Web Services Proxy support.  Available only on Mac OS X 10.2 or later.
 * These constants should be added as attributes on the event that is
 * being sent (not part of the direct object.)
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
const
{ Automatically configure the proxy based on System Configuration }
	kAEUseHTTPProxyAttr = FourCharCode('xupr'); { a typeBoolean.  Defaults to true.}
                                        { manually specify the proxy host and port. }
	kAEHTTPProxyPortAttr = FourCharCode('xhtp'); { a typeSInt32}
	kAEHTTPProxyHostAttr = FourCharCode('xhth'); { a typeChar}
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
   { Automatically configure the proxy based on System Configuration }
	kAEUseHTTPProxyAttr         = FourCharCode('xupr'); { a typeBoolean.  Defaults to true.}
                                                          { manually specify the proxy host and port. }
	kAEHTTPProxyPortAttr        = FourCharCode('xhtp'); { a typeSInt32}
	kAEHTTPProxyHostAttr        = FourCharCode('xhth'); { a typeChar}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{
 * Web Services SOCKS support.  kAEUseSocksAttr is a boolean that
 * specifies whether to automatically configure SOCKS proxies by
 * querying System Configuration.
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
const
	kAESocks4Protocol = 4;
	kAESocks5Protocol = 5;

const
	kAEUseSocksAttr = FourCharCode('xscs'); { a typeBoolean.  Defaults to true.}
                                        { This attribute specifies a specific SOCKS protocol to be used }
	kAESocksProxyAttr = FourCharCode('xsok'); { a typeSInt32}
                                        { if version >= 4 }
	kAESocksHostAttr = FourCharCode('xshs'); { a typeChar}
	kAESocksPortAttr = FourCharCode('xshp'); { a typeSInt32}
	kAESocksUserAttr = FourCharCode('xshu'); { a typeChar}
                                        { if version >= 5 }
	kAESocksPasswordAttr = FourCharCode('xshw'); { a typeChar}


{  Constants used for specifying the factoring of AEDescLists. }
const
	kAEDescListFactorNone = 0;
	kAEDescListFactorType = 4;
	kAEDescListFactorTypeAndSize = 8;

{ Constants used creating an AppleEvent }
const
{ Constant for the returnID param of AECreateAppleEvent }
	kAutoGenerateReturnID = -1;   { AECreateAppleEvent will generate a session-unique ID }
                                        { Constant for transaction IDs }
	kAnyTransactionID = 0;     { no transaction is in use }

{ Apple event manager data types }
=======
=======
>>>>>>> origin/fixes_2_2
	kAESocks4Protocol           = 4;
	kAESocks5Protocol           = 5;

	kAEUseSocksAttr             = FourCharCode('xscs'); { a typeBoolean.  Defaults to true.}
                                                          { This attribute specifies a specific SOCKS protocol to be used }
	kAESocksProxyAttr           = FourCharCode('xsok'); { a typeSInt32}
                                                     { if version >= 4 }
	kAESocksHostAttr            = FourCharCode('xshs'); { a typeChar}
	kAESocksPortAttr            = FourCharCode('xshp'); { a typeSInt32}
	kAESocksUserAttr            = FourCharCode('xshu'); { a typeChar}
                                                     { if version >= 5 }
	kAESocksPasswordAttr        = FourCharCode('xshw'); { a typeChar}


=======
const
	kAESocks4Protocol = 4;
	kAESocks5Protocol = 5;
>>>>>>> origin/fixes_2.4

const
	kAEUseSocksAttr = FourCharCode('xscs'); { a typeBoolean.  Defaults to true.}
                                        { This attribute specifies a specific SOCKS protocol to be used }
	kAESocksProxyAttr = FourCharCode('xsok'); { a typeSInt32}
                                        { if version >= 4 }
	kAESocksHostAttr = FourCharCode('xshs'); { a typeChar}
	kAESocksPortAttr = FourCharCode('xshp'); { a typeSInt32}
	kAESocksUserAttr = FourCharCode('xshu'); { a typeChar}
                                        { if version >= 5 }
	kAESocksPasswordAttr = FourCharCode('xshw'); { a typeChar}


{  Constants used for specifying the factoring of AEDescLists. }
const
	kAEDescListFactorNone = 0;
	kAEDescListFactorType = 4;
	kAEDescListFactorTypeAndSize = 8;

{ Constants used creating an AppleEvent }
const
{ Constant for the returnID param of AECreateAppleEvent }
	kAutoGenerateReturnID = -1;   { AECreateAppleEvent will generate a session-unique ID }
                                        { Constant for transaction IDs }
	kAnyTransactionID = 0;     { no transaction is in use }

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ Apple event manager data types }
>>>>>>> origin/fixes_2.4
type
	DescType							= ResType;
	DescTypePtr					= ^DescType;
	AEKeyword							= FourCharCode;
	AEKeywordPtr					= ^AEKeyword;
{$ifc OPAQUE_TOOLBOX_STRUCTS}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	AEDataStorage = ^OpaqueAEDataStorage; { an opaque type }
	OpaqueAEDataStorage = record end;
=======
	AEDataStorage    = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	AEDataStorage    = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	AEDataStorage    = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	AEDataStorage    = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	AEDataStorage    = ^SInt32; { an opaque 32-bit type }
>>>>>>> graemeg/fixes_2_2
=======
	AEDataStorage    = ^SInt32; { an opaque 32-bit type }
>>>>>>> origin/fixes_2_2
=======
	AEDataStorage    = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4
	AEDataStoragePtr = ^AEDataStorage;  { when a var xx:AEDataStorage parameter can be nil, it is changed to xx: AEDataStoragePtr }
{$elsec}
	AEDataStorage						= Handle;
{$endc}  {OPAQUE_TOOLBOX_STRUCTS}

	AEDescPtr = ^AEDesc;
	AEDesc = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		descriptorType: DescType;
		{ No alignment dummy here for 64 bit, also compiled with m68k alignment in C! }
		dataHandle: AEDataStorage;
	end;
type
	AEKeyDescPtr = ^AEKeyDesc;
	AEKeyDesc = record
		descKey: AEKeyword;
		descContent: AEDesc;
	end;
{ a list of AEDesc's is a special kind of AEDesc }

<<<<<<< HEAD
<<<<<<< HEAD
=======
{ a list of AEDesc's is a special kind of AEDesc }

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
{ a list of AEDesc's is a special kind of AEDesc }

>>>>>>> graemeg/cpstrnew
=======
{ a list of AEDesc's is a special kind of AEDesc }

>>>>>>> graemeg/cpstrnew
=======
{ a list of AEDesc's is a special kind of AEDesc }

>>>>>>> origin/cpstrnew
type
	AEDescList = AEDesc;
	AEDescListPtr = ^AEDescList;
{ AERecord is a list of keyworded AEDesc's }
type
	AERecord = AEDescList;
	AERecordPtr = ^AERecord;
{ an AEDesc which contains address data }
type
	AEAddressDesc = AEDesc;
	AEAddressDescPtr = ^AEAddressDesc;
{ an AERecord that contains an AppleEvent, and related data types }
type
	AppleEvent = AERecord;
	AppleEventPtr = ^AppleEvent;
	AEReturnID = SInt16;
	AETransactionID = SInt32;
	AEEventClass = FourCharCode;
	AEEventID = FourCharCode;
	AEArrayType = SInt8;
const
	kAEDataArray = 0;
	kAEPackedArray = 1;
	kAEDescArray = 3;
	kAEKeyDescArray = 4;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD


=======


>>>>>>> graemeg/cpstrnew
=======
=======

>>>>>>> graemeg/cpstrnew

const
	kAEHandleArray = 2;

>>>>>>> graemeg/cpstrnew
=======


>>>>>>> origin/cpstrnew
const
	kAEHandleArray = 2;
=======
=======
>>>>>>> origin/fixes_2_2
		descriptorType:			DescType;
		dataHandle:				AEDataStorage;
=======
		descriptorType: DescType;
		{ No alignment dummy here for 64 bit, also compiled with m68k alignment in C! }
		dataHandle: AEDataStorage;
>>>>>>> origin/fixes_2.4
	end;
type
	AEKeyDescPtr = ^AEKeyDesc;
	AEKeyDesc = record
		descKey: AEKeyword;
		descContent: AEDesc;
	end;

{ a list of AEDesc's is a special kind of AEDesc }

type
	AEDescList = AEDesc;
	AEDescListPtr = ^AEDescList;
{ AERecord is a list of keyworded AEDesc's }
type
	AERecord = AEDescList;
	AERecordPtr = ^AERecord;
{ an AEDesc which contains address data }
type
	AEAddressDesc = AEDesc;
	AEAddressDescPtr = ^AEAddressDesc;
{ an AERecord that contains an AppleEvent, and related data types }
type
	AppleEvent = AERecord;
	AppleEventPtr = ^AppleEvent;
	AEReturnID = SInt16;
	AETransactionID = SInt32;
	AEEventClass = FourCharCode;
	AEEventID = FourCharCode;
	AEArrayType = SInt8;
const
	kAEDataArray = 0;
	kAEPackedArray = 1;
	kAEDescArray = 3;
	kAEKeyDescArray = 4;


<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
const
	kAEHandleArray = 2;
>>>>>>> origin/fixes_2.4

type
	AEArrayDataPtr = ^AEArrayData;
	AEArrayData = record
		case SInt16 of
		0: (
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
			kAEDataArray: array [0..0] of SInt16;
			);
		1: (
			kAEPackedArray: SInt8;
			);
		2: (
			kAEHandleArray:	array [0..0] of Handle;
			);
		3: (
			kAEDescArray: array [0..0] of AEDesc;
			);
		4: (
			kAEKeyDescArray: array [0..0] of AEKeyDesc;
			);
	end;
type
	AEArrayDataPointer = ^AEArrayData;
	AEArrayDataPointerPtr = ^AEArrayDataPointer;
{*************************************************************************
  These constants are used by AEMach and AEInteraction APIs.  They are not
  strictly part of the data format, but are declared here due to layering.
*************************************************************************}
type
	AESendPriority = SInt16;
const
	kAENormalPriority = $00000000; { post message at the end of the event queue }
	kAEHighPriority = $00000001; { post message at the front of the event queue (same as nAttnMsg) }


type
	AESendMode = SInt32;
const
	kAENoReply = $00000001; { sender doesn't want a reply to event }
	kAEQueueReply = $00000002; { sender wants a reply but won't wait }
	kAEWaitReply = $00000003; { sender wants a reply and will wait }
	kAEDontReconnect = $00000080; { don't reconnect if there is a sessClosedErr from PPCToolbox }
	kAEWantReceipt = $00000200; { (nReturnReceipt) sender wants a receipt of message }
	kAENeverInteract = $00000010; { server should not interact with user }
	kAECanInteract = $00000020; { server may try to interact with user }
	kAEAlwaysInteract = $00000030; { server should always interact with user where appropriate }
	kAECanSwitchLayer = $00000040; { interaction may switch layer }
	kAEDontRecord = $00001000; { don't record this event - available only in vers 1.0.1 and greater }
	kAEDontExecute = $00002000; { don't send the event for recording - available only in vers 1.0.1 and greater }
	kAEProcessNonReplyEvents = $00008000; { allow processing of non-reply events while awaiting synchronous AppleEvent reply }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kAEDoNotAutomaticallyAddAnnotationsToEvent = $00010000; { if set, don't automatically add any sandbox or other annotations to the event }
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew


{ Constants for timeout durations }
const
	kAEDefaultTimeout = -1;   { timeout value determined by AEM }
	kNoTimeOut = -2;    { wait until reply comes back, however long it takes }


{*************************************************************************
  These calls are used to set up and modify the coercion dispatch table.
*************************************************************************}
type
	AECoerceDescProcPtr = function( const (*var*) fromDesc: AEDesc; toType: DescType; handlerRefcon: SRefCon; var toDesc: AEDesc ): OSErr;
	AECoercePtrProcPtr = function( typeCode: DescType; dataPtr: {const} UnivPtr; dataSize: Size; toType: DescType; handlerRefcon: SRefCon; var result: AEDesc ): OSErr;
	AECoerceDescUPP = UniversalProcPtr;
	AECoercePtrUPP = UniversalProcPtr;
{
 *  NewAECoerceDescUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewAECoerceDescUPP( userRoutine: AECoerceDescProcPtr ): AECoerceDescUPP; external name '_NewAECoerceDescUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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
=======
>>>>>>> origin/fixes_2_2
			kAEDataArray:		array [0..0] of SInt16;
=======
			kAEDataArray: array [0..0] of SInt16;
>>>>>>> origin/fixes_2.4
			);
		1: (
			kAEPackedArray: SInt8;
			);
		2: (
			kAEHandleArray:	array [0..0] of Handle;
			);
		3: (
			kAEDescArray: array [0..0] of AEDesc;
			);
		4: (
			kAEKeyDescArray: array [0..0] of AEKeyDesc;
			);
	end;
type
	AEArrayDataPointer = ^AEArrayData;
	AEArrayDataPointerPtr = ^AEArrayDataPointer;
{*************************************************************************
  These constants are used by AEMach and AEInteraction APIs.  They are not
  strictly part of the data format, but are declared here due to layering.
*************************************************************************}
type
	AESendPriority = SInt16;
const
	kAENormalPriority = $00000000; { post message at the end of the event queue }
	kAEHighPriority = $00000001; { post message at the front of the event queue (same as nAttnMsg) }


type
	AESendMode = SInt32;
const
	kAENoReply = $00000001; { sender doesn't want a reply to event }
	kAEQueueReply = $00000002; { sender wants a reply but won't wait }
	kAEWaitReply = $00000003; { sender wants a reply and will wait }
	kAEDontReconnect = $00000080; { don't reconnect if there is a sessClosedErr from PPCToolbox }
	kAEWantReceipt = $00000200; { (nReturnReceipt) sender wants a receipt of message }
	kAENeverInteract = $00000010; { server should not interact with user }
	kAECanInteract = $00000020; { server may try to interact with user }
	kAEAlwaysInteract = $00000030; { server should always interact with user where appropriate }
	kAECanSwitchLayer = $00000040; { interaction may switch layer }
	kAEDontRecord = $00001000; { don't record this event - available only in vers 1.0.1 and greater }
	kAEDontExecute = $00002000; { don't send the event for recording - available only in vers 1.0.1 and greater }
	kAEProcessNonReplyEvents = $00008000; { allow processing of non-reply events while awaiting synchronous AppleEvent reply }


{ Constants for timeout durations }
const
	kAEDefaultTimeout = -1;   { timeout value determined by AEM }
	kNoTimeOut = -2;    { wait until reply comes back, however long it takes }


{*************************************************************************
  These calls are used to set up and modify the coercion dispatch table.
*************************************************************************}
type
	AECoerceDescProcPtr = function( const (*var*) fromDesc: AEDesc; toType: DescType; handlerRefcon: SRefCon; var toDesc: AEDesc ): OSErr;
	AECoercePtrProcPtr = function( typeCode: DescType; dataPtr: {const} UnivPtr; dataSize: Size; toType: DescType; handlerRefcon: SRefCon; var result: AEDesc ): OSErr;
	AECoerceDescUPP = UniversalProcPtr;
	AECoercePtrUPP = UniversalProcPtr;
{
 *  NewAECoerceDescUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewAECoerceDescUPP( userRoutine: AECoerceDescProcPtr ): AECoerceDescUPP; external name '_NewAECoerceDescUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

<<<<<<< HEAD
const
	uppAECoerceDescProcInfo = $00003FE0;
	uppAECoercePtrProcInfo = $0003FFE0;
	{
	 *  NewAECoerceDescUPP()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   available as macro/inline
	 *    CarbonLib:        in CarbonLib 1.0 and later
	 *    Mac OS X:         in version 10.0 and later
	 	}
function NewAECoerceDescUPP(userRoutine: AECoerceDescProcPtr): AECoerceDescUPP; external name '_NewAECoerceDescUPP'; { old name was NewAECoerceDescProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{
 *  NewAECoercePtrUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewAECoercePtrUPP( userRoutine: AECoercePtrProcPtr ): AECoercePtrUPP; external name '_NewAECoercePtrUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function NewAECoercePtrUPP(userRoutine: AECoercePtrProcPtr): AECoercePtrUPP; external name '_NewAECoercePtrUPP'; { old name was NewAECoercePtrProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function NewAECoercePtrUPP( userRoutine: AECoercePtrProcPtr ): AECoercePtrUPP; external name '_NewAECoercePtrUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  DisposeAECoerceDescUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeAECoerceDescUPP( userUPP: AECoerceDescUPP ); external name '_DisposeAECoerceDescUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure DisposeAECoerceDescUPP(userUPP: AECoerceDescUPP); external name '_DisposeAECoerceDescUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeAECoerceDescUPP( userUPP: AECoerceDescUPP ); external name '_DisposeAECoerceDescUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  DisposeAECoercePtrUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeAECoercePtrUPP( userUPP: AECoercePtrUPP ); external name '_DisposeAECoercePtrUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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
 *    Non-Carbon CFM:   available as macro/inline
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
procedure DisposeAECoercePtrUPP(userUPP: AECoercePtrUPP); external name '_DisposeAECoercePtrUPP';
>>>>>>> graemeg/fixes_2_2
=======
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure DisposeAECoercePtrUPP(userUPP: AECoercePtrUPP); external name '_DisposeAECoercePtrUPP';
>>>>>>> origin/fixes_2_2
=======
procedure DisposeAECoercePtrUPP( userUPP: AECoercePtrUPP ); external name '_DisposeAECoercePtrUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  InvokeAECoerceDescUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeAECoerceDescUPP( const (*var*) fromDesc: AEDesc; toType: DescType; handlerRefcon: SRefCon; var toDesc: AEDesc; userUPP: AECoerceDescUPP ): OSErr; external name '_InvokeAECoerceDescUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function InvokeAECoerceDescUPP(const (*var*) fromDesc: AEDesc; toType: DescType; handlerRefcon: SInt32; var toDesc: AEDesc; userRoutine: AECoerceDescUPP): OSErr; external name '_InvokeAECoerceDescUPP'; { old name was CallAECoerceDescProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function InvokeAECoerceDescUPP( const (*var*) fromDesc: AEDesc; toType: DescType; handlerRefcon: SRefCon; var toDesc: AEDesc; userUPP: AECoerceDescUPP ): OSErr; external name '_InvokeAECoerceDescUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{
 *  InvokeAECoercePtrUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeAECoercePtrUPP( typeCode: DescType; dataPtr: {const} UnivPtr; dataSize: Size; toType: DescType; handlerRefcon: SRefCon; var result: AEDesc; userUPP: AECoercePtrUPP ): OSErr; external name '_InvokeAECoercePtrUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
function InvokeAECoercePtrUPP(typeCode: DescType; dataPtr: UnivPtr; dataSize: Size; toType: DescType; handlerRefcon: SInt32; var result: AEDesc; userRoutine: AECoercePtrUPP): OSErr; external name '_InvokeAECoercePtrUPP'; { old name was CallAECoercePtrProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function InvokeAECoercePtrUPP( typeCode: DescType; dataPtr: {const} UnivPtr; dataSize: Size; toType: DescType; handlerRefcon: SRefCon; var result: AEDesc; userUPP: AECoercePtrUPP ): OSErr; external name '_InvokeAECoercePtrUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4
{ a AECoercionHandlerUPP is by default a AECoerceDescUPP.  If you are registering a 
    Ptr based coercion handler you will have to add a cast to AECoerceDescUPP from 
    your AECoercePtrUPP type.  A future release of the interfaces will fix this by
    introducing seperate Desc and Ptr coercion handler installation/remove/query routines. }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
	AECoercionHandlerUPP = AECoerceDescUPP;
{
 *  AEInstallCoercionHandler()
 *  
 *  Summary:
 *    Installs a coercion handler in either the application or system
 *    coercion handler dispatch table.
 *  
 *  Discussion:
 *    Before using AEInstallCoercionHandler to install a handler for a
 *    particular descriptor type, you can use the AEGetCoercionHandler
 *    function to determine whether the table already contains a
 *    coercion handler for that type. 
 *    Version Notes
 *    See the Version Notes section for the AECoercePtr function for
 *    information on when to use descriptor-based versus pointer-based
 *    coercion handlers starting in Mac OS X version 10.2.
 *    Your application should not install a coercion handler in a
 *    system coercion handler dispatch table with the goal that the
 *    handler will get called when other applications perform coercions
 *    - this won't work in Mac OS X. For more information, see "Writing
 *    and Installing Coercion Handlers" in Apple Events Programming
 *    Guide.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    fromType:
 *      The descriptor type of the data coerced by the handler.
 *    
 *    toType:
 *      The descriptor type of the resulting data.
 *      If there was already an entry in the specified coercion handler
 *      table for the same source descriptor type and result descriptor
 *      type, the existing entry is replaced.
 *    
 *    handler:
 *      A universal procedure pointer to the coercion handler function
 *      to install.
 *    
 *    handlerRefcon:
 *      A reference constant. The Apple Event Manager passes this value
 *      to the handler each time it calls it. If your handler doesn't
 *      require a reference constant, pass 0 for this parameter.
 *    
 *    fromTypeIsDesc:
 *      Specifies the form of the data to coerce. Pass TRUE if the
 *      coercion handler expects the data as a descriptor or FALSE if
 *      the coercion handler expects a pointer to the data. The Apple
 *      Event Manager can provide a pointer to data more efficiently
 *      than it can provide a descriptor, so all coercion functions
 *      should accept a pointer to data if possible.
 *    
 *    isSysHandler:
 *      Specifies the coercion table to add the handler to. Pass TRUE
 *      to add the handler to the system coercion table or FALSE to add
 *      the handler to your application's coercion table. Use of the
 *      system coercion table is not recommended.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AEInstallCoercionHandler( fromType: DescType; toType: DescType; handler: AECoercionHandlerUPP; handlerRefcon: SRefCon; fromTypeIsDesc: Boolean; isSysHandler: Boolean ): OSErr; external name '_AEInstallCoercionHandler';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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


{
 *  AERemoveCoercionHandler()
 *  
 *  Discussion:
 *    Removes a coercion handler from a coercion handler dispatch table.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    fromType:
 *      The descriptor type of the data coerced by the handler.
 *    
 *    toType:
 *      The descriptor type of the resulting data.
 *    
 *    handler:
 *      A universal procedure pointer to the coercion handler to
 *      remove. Although the parameters fromType and toType are
 *      sufficient to identify the handler, you can identify the
 *      handler explicitly as a safeguard. If you pass NULL for this
 *      parameter, the Apple Event Manager relies solely on the event
 *      class and event ID to identify the handler.
 *    
 *    isSysHandler:
 *      Specifies the coercion table to remove the handler from. Pass
 *      TRUE to remove the handler from the system coercion table or
 *      FALSE to remove the handler from your application's coercion
 *      table. Use of the system coercion table is not recommended.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AERemoveCoercionHandler( fromType: DescType; toType: DescType; handler: AECoercionHandlerUPP; isSysHandler: Boolean ): OSErr; external name '_AERemoveCoercionHandler';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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


{
 *  AEGetCoercionHandler()
 *  
 *  Discussion:
 *    Gets the coercion handler for a specified descriptor type.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    fromType:
 *      The descriptor type of the data coerced by the handler.
 *    
 *    toType:
 *      The descriptor type of the resulting data.
 *    
 *    handler:
 *      A universal procedure pointer. On return, a pointer to the
 *      specified handler, if a coercion table entry exists that
 *      exactly matches the values supplied in the parameters fromType
 *      and toType. See AECoercionHandlerUPP.
 *    
 *    handlerRefcon:
 *      A pointer to a reference constant. On return, the reference
 *      constant from the coercion table entry for the specified
 *      coercion handler. The Apple Event Manager passes this reference
 *      constant to the handler each time it calls the handler. The
 *      reference constant may have a value of 0.
 *    
 *    fromTypeIsDesc:
 *      A pointer to a Boolean value. The AEGetCoercionHandler function
 *      returns a value of TRUE in this parameter if the coercion
 *      handler expects the data as a descriptor or FALSE, if the
 *      coercion handler expects a pointer to the data.
 *    
 *    isSysHandler:
 *      Specifies the coercion table to get the handler from. Pass TRUE
 *      to get the handler from the system coercion table or FALSE to
 *      get the handler from your application's coercion table. Use of
 *      the system coercion table is not recommended.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AEGetCoercionHandler( fromType: DescType; toType: DescType; var handler: AECoercionHandlerUPP; var handlerRefcon: SRefCon; var fromTypeIsDesc: Boolean; isSysHandler: Boolean ): OSErr; external name '_AEGetCoercionHandler';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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
=======
>>>>>>> origin/fixes_2_2

=======
>>>>>>> origin/fixes_2.4
type
	AECoercionHandlerUPP = AECoerceDescUPP;
{
 *  AEInstallCoercionHandler()
 *  
 *  Summary:
 *    Installs a coercion handler in either the application or system
 *    coercion handler dispatch table.
 *  
 *  Discussion:
 *    Before using AEInstallCoercionHandler to install a handler for a
 *    particular descriptor type, you can use the AEGetCoercionHandler
 *    function to determine whether the table already contains a
 *    coercion handler for that type. 
 *    Version Notes
 *    See the Version Notes section for the AECoercePtr function for
 *    information on when to use descriptor-based versus pointer-based
 *    coercion handlers starting in Mac OS X version 10.2.
 *    Your application should not install a coercion handler in a
 *    system coercion handler dispatch table with the goal that the
 *    handler will get called when other applications perform coercions
 *    - this won't work in Mac OS X. For more information, see "Writing
 *    and Installing Coercion Handlers" in Apple Events Programming
 *    Guide.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    fromType:
 *      The descriptor type of the data coerced by the handler.
 *    
 *    toType:
 *      The descriptor type of the resulting data.
 *      If there was already an entry in the specified coercion handler
 *      table for the same source descriptor type and result descriptor
 *      type, the existing entry is replaced.
 *    
 *    handler:
 *      A universal procedure pointer to the coercion handler function
 *      to install.
 *    
 *    handlerRefcon:
 *      A reference constant. The Apple Event Manager passes this value
 *      to the handler each time it calls it. If your handler doesn't
 *      require a reference constant, pass 0 for this parameter.
 *    
 *    fromTypeIsDesc:
 *      Specifies the form of the data to coerce. Pass TRUE if the
 *      coercion handler expects the data as a descriptor or FALSE if
 *      the coercion handler expects a pointer to the data. The Apple
 *      Event Manager can provide a pointer to data more efficiently
 *      than it can provide a descriptor, so all coercion functions
 *      should accept a pointer to data if possible.
 *    
 *    isSysHandler:
 *      Specifies the coercion table to add the handler to. Pass TRUE
 *      to add the handler to the system coercion table or FALSE to add
 *      the handler to your application's coercion table. Use of the
 *      system coercion table is not recommended.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AEInstallCoercionHandler( fromType: DescType; toType: DescType; handler: AECoercionHandlerUPP; handlerRefcon: SRefCon; fromTypeIsDesc: Boolean; isSysHandler: Boolean ): OSErr; external name '_AEInstallCoercionHandler';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  AERemoveCoercionHandler()
 *  
 *  Discussion:
 *    Removes a coercion handler from a coercion handler dispatch table.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    fromType:
 *      The descriptor type of the data coerced by the handler.
 *    
 *    toType:
 *      The descriptor type of the resulting data.
 *    
 *    handler:
 *      A universal procedure pointer to the coercion handler to
 *      remove. Although the parameters fromType and toType are
 *      sufficient to identify the handler, you can identify the
 *      handler explicitly as a safeguard. If you pass NULL for this
 *      parameter, the Apple Event Manager relies solely on the event
 *      class and event ID to identify the handler.
 *    
 *    isSysHandler:
 *      Specifies the coercion table to remove the handler from. Pass
 *      TRUE to remove the handler from the system coercion table or
 *      FALSE to remove the handler from your application's coercion
 *      table. Use of the system coercion table is not recommended.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AERemoveCoercionHandler( fromType: DescType; toType: DescType; handler: AECoercionHandlerUPP; isSysHandler: Boolean ): OSErr; external name '_AERemoveCoercionHandler';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  AEGetCoercionHandler()
 *  
 *  Discussion:
 *    Gets the coercion handler for a specified descriptor type.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    fromType:
 *      The descriptor type of the data coerced by the handler.
 *    
 *    toType:
 *      The descriptor type of the resulting data.
 *    
 *    handler:
 *      A universal procedure pointer. On return, a pointer to the
 *      specified handler, if a coercion table entry exists that
 *      exactly matches the values supplied in the parameters fromType
 *      and toType. See AECoercionHandlerUPP.
 *    
 *    handlerRefcon:
 *      A pointer to a reference constant. On return, the reference
 *      constant from the coercion table entry for the specified
 *      coercion handler. The Apple Event Manager passes this reference
 *      constant to the handler each time it calls the handler. The
 *      reference constant may have a value of 0.
 *    
 *    fromTypeIsDesc:
 *      A pointer to a Boolean value. The AEGetCoercionHandler function
 *      returns a value of TRUE in this parameter if the coercion
 *      handler expects the data as a descriptor or FALSE, if the
 *      coercion handler expects a pointer to the data.
 *    
 *    isSysHandler:
 *      Specifies the coercion table to get the handler from. Pass TRUE
 *      to get the handler from the system coercion table or FALSE to
 *      get the handler from your application's coercion table. Use of
 *      the system coercion table is not recommended.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function AEGetCoercionHandler(fromType: DescType; toType: DescType; var handler: AECoercionHandlerUPP; var handlerRefcon: SInt32; var fromTypeIsDesc: boolean; isSysHandler: boolean): OSErr; external name '_AEGetCoercionHandler';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function AEGetCoercionHandler( fromType: DescType; toType: DescType; var handler: AECoercionHandlerUPP; var handlerRefcon: SRefCon; var fromTypeIsDesc: Boolean; isSysHandler: Boolean ): OSErr; external name '_AEGetCoercionHandler';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{*************************************************************************
  The following calls provide for a coercion interface.
*************************************************************************}
{
 *  AECoercePtr()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  Discussion:
 *    Coerces data to a desired descriptor type and creates a
 *    descriptor containing the newly coerced data.
 *  
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    typeCode:
 *      The descriptor type of the source data.
 *    
 *    dataPtr:
 *      A pointer to the data to coerce.
 *    
 *    dataSize:
 *      The length, in bytes, of the data to coerce.
 *    
 *    toType:
 *      The desired descriptor type of the resulting descriptor.
 *    
 *    result:
 *      A pointer to a descriptor. On successful return, a descriptor
 *      containing the coerced data and matching the descriptor type
 *      specified in toType. On error, a null descriptor. If the
 *      function returns successfully, your application should call the
 *      AEDisposeDesc function to dispose of the resulting descriptor
 *      after it has finished using it. See AEDesc.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AECoercePtr( typeCode: DescType; dataPtr: {const} UnivPtr; dataSize: Size; toType: DescType; var result: AEDesc ): OSErr; external name '_AECoercePtr';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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


{
 *  AECoerceDesc()
 *  
 *  Discussion:
 *    Coerces the data in a descriptor to another descriptor type and
 *    creates a descriptor containing the newly coerced data.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAEDesc:
 *      A pointer to the descriptor containing the data to coerce.
 *    
 *    toType:
 *      The desired descriptor type of the resulting descriptor.
 *    
 *    result:
 *      A pointer to a descriptor. On successful return, a descriptor
 *      containing the coerced data and matching the descriptor type
 *      specified in toType. On error, a null descriptor. If the
 *      function returns successfully, your application should call the
 *      AEDisposeDesc function to dispose of the resulting descriptor
 *      after it has finished using it.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AECoerceDesc( const (*var*) theAEDesc: AEDesc; toType: DescType; var result: AEDesc ): OSErr; external name '_AECoerceDesc';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    typeCode:
 *      The descriptor type of the source data.
 *    
 *    dataPtr:
 *      A pointer to the data to coerce.
 *    
 *    dataSize:
 *      The length, in bytes, of the data to coerce.
 *    
 *    toType:
 *      The desired descriptor type of the resulting descriptor.
 *    
 *    result:
 *      A pointer to a descriptor. On successful return, a descriptor
 *      containing the coerced data and matching the descriptor type
 *      specified in toType. On error, a null descriptor. If the
 *      function returns successfully, your application should call the
 *      AEDisposeDesc function to dispose of the resulting descriptor
 *      after it has finished using it. See AEDesc.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AECoercePtr( typeCode: DescType; dataPtr: {const} UnivPtr; dataSize: Size; toType: DescType; var result: AEDesc ): OSErr; external name '_AECoercePtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  AECoerceDesc()
 *  
 *  Discussion:
 *    Coerces the data in a descriptor to another descriptor type and
 *    creates a descriptor containing the newly coerced data.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAEDesc:
 *      A pointer to the descriptor containing the data to coerce.
 *    
 *    toType:
 *      The desired descriptor type of the resulting descriptor.
 *    
 *    result:
 *      A pointer to a descriptor. On successful return, a descriptor
 *      containing the coerced data and matching the descriptor type
 *      specified in toType. On error, a null descriptor. If the
 *      function returns successfully, your application should call the
 *      AEDisposeDesc function to dispose of the resulting descriptor
 *      after it has finished using it.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function AECoerceDesc(const (*var*) theAEDesc: AEDesc; toType: DescType; var result: AEDesc): OSErr; external name '_AECoerceDesc';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function AECoerceDesc( const (*var*) theAEDesc: AEDesc; toType: DescType; var result: AEDesc ): OSErr; external name '_AECoerceDesc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{*************************************************************************
 The following calls apply to any AEDesc. Every 'result' descriptor is
 created for you, so you will be responsible for memory management
 (including disposing) of the descriptors so created.  
*************************************************************************}
{ because AEDescs are opaque under Carbon, this AEInitializeDesc provides a
   'clean' way of initializating them to be empty. }
{
 *  AEInitializeDesc()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  Discussion:
 *    The function sets the type of the descriptor to typeNull and sets
 *    the data handle to NULL. If you need to initialize a descriptor
 *    that already has some data in it, use AEDisposeDesc to deallocate
 *    the memory and initialize the descriptor.
 *  
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    desc:
 *      A pointer to a new descriptor.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.4 and later
 *    Non-Carbon CFM:   not available
 }
procedure AEInitializeDesc( var desc: AEDesc ); external name '_AEInitializeDesc';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    desc:
 *      A pointer to a new descriptor.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.4 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
procedure AEInitializeDesc(var desc: AEDesc); external name '_AEInitializeDesc';

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure AEInitializeDesc( var desc: AEDesc ); external name '_AEInitializeDesc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4

{
 *  AECreateDesc()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  Summary:
 *    Creates a new descriptor that incorporates the specified data.
 *  
 *  Discussion:
 *    While it is possible to create an Apple event descriptor or a
 *    descriptor list or a descriptor with the AECreateDesc function
 *    (assuming you have access to the raw data for an Apple event,
 *    list, or descriptor), you typically create these structured
 *    objects with their specific creation routines -
 *    AECreateAppleEvent, AECreateList, or AECreateDesc.
 *  
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    typeCode:
 *      The descriptor type for the new descriptor.
 *    
 *    dataPtr:
 *      A pointer to the data for the new descriptor. This data is
 *      copied into a newly-allocated block of memory for the
 *      descriptor that is created. To minimize copying overhead,
 *      consider using AECreateDescFromExternalPtr.
 *    
 *    dataSize:
 *      The length, in bytes, of the data for the new descriptor.
 *    
 *    result:
 *      A pointer to a descriptor. On successful return, a descriptor
 *      that incorporates the data specified by the dataPtr parameter.
 *      On error, a null descriptor. If the function returns
 *      successfully, your application should call the AEDisposeDesc
 *      function to dispose of the resulting descriptor after it has
 *      finished using it.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AECreateDesc( typeCode: DescType; dataPtr: {const} UnivPtr; dataSize: Size; var result: AEDesc ): OSErr; external name '_AECreateDesc';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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


{
 *  AEDisposeDesc()
 *  
 *  Summary:
 *    Deallocates the memory used by a descriptor.
 *  
 *  Discussion:
 *    The AEDisposeDesc function deallocates the memory used by a
 *    descriptor. After calling this method, the descriptor becomes an
 *    empty descriptor with a type of typeNULL. Because all Apple event
 *    structures (except for keyword-specified descriptors) are
 *    descriptors, you can use AEDisposeDesc for any of them.
 *    Do not call AEDisposeDesc on a descriptor obtained from another
 *    Apple Event Manager function (such as the reply event from a call
 *    to AESend) unless that function returns successfully.
 *    Special Considerations
 *    If the AEDesc might contain an OSL token, dispose of it with
 *    AEDisposeToken.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAEDesc:
 *      A pointer to the descriptor to deallocate. On return, a null
 *      descriptor. If you pass a null descriptor in this parameter,
 *      AEDisposeDesc returns noErr.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AEDisposeDesc( var theAEDesc: AEDesc ): OSErr; external name '_AEDisposeDesc';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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


{
 *  AEDuplicateDesc()
 *  
 *  Summary:
 *    Creates a copy of a descriptor.
 *  
 *  Discussion:
 *    It is common for applications to send Apple events that have one
 *    or more attributes or parameters in common. For example, if you
 *    send a series of Apple events to the same application, the
 *    address attribute is the same. In these cases, the most efficient
 *    way to create the necessary Apple events is to make a template
 *    Apple event that you can then copy - by calling the
 *    AEDuplicateDesc function - as needed. You then fill in or change
 *    the remaining parameters and attributes of the copy, send the
 *    copy by calling the AESend function and, after AESend returns a
 *    result code, dispose of the copy by calling AEDisposeDesc. You
 *    can use this approach to prepare structures of type AEDesc,
 *    AEDescList, AERecord, and AppleEvent.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAEDesc:
 *      A pointer to the descriptor to duplicate. See AEDesc.
 *    
 *    result:
 *      A pointer to a descriptor. On return, the descriptor contains a
 *      copy of the descriptor specified by the theAEDesc parameter. If
 *      the function returns successfully, your application should call
 *      the AEDisposeDesc function to dispose of the resulting
 *      descriptor after it has finished using it.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AEDuplicateDesc( const (*var*) theAEDesc: AEDesc; var result: AEDesc ): OSErr; external name '_AEDuplicateDesc';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)

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
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    typeCode:
 *      The descriptor type for the new descriptor.
 *    
 *    dataPtr:
 *      A pointer to the data for the new descriptor. This data is
 *      copied into a newly-allocated block of memory for the
 *      descriptor that is created. To minimize copying overhead,
 *      consider using AECreateDescFromExternalPtr.
 *    
 *    dataSize:
 *      The length, in bytes, of the data for the new descriptor.
 *    
 *    result:
 *      A pointer to a descriptor. On successful return, a descriptor
 *      that incorporates the data specified by the dataPtr parameter.
 *      On error, a null descriptor. If the function returns
 *      successfully, your application should call the AEDisposeDesc
 *      function to dispose of the resulting descriptor after it has
 *      finished using it.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AECreateDesc( typeCode: DescType; dataPtr: {const} UnivPtr; dataSize: Size; var result: AEDesc ): OSErr; external name '_AECreateDesc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  AEDisposeDesc()
 *  
 *  Summary:
 *    Deallocates the memory used by a descriptor.
 *  
 *  Discussion:
 *    The AEDisposeDesc function deallocates the memory used by a
 *    descriptor. After calling this method, the descriptor becomes an
 *    empty descriptor with a type of typeNULL. Because all Apple event
 *    structures (except for keyword-specified descriptors) are
 *    descriptors, you can use AEDisposeDesc for any of them.
 *    Do not call AEDisposeDesc on a descriptor obtained from another
 *    Apple Event Manager function (such as the reply event from a call
 *    to AESend) unless that function returns successfully.
 *    Special Considerations
 *    If the AEDesc might contain an OSL token, dispose of it with
 *    AEDisposeToken.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAEDesc:
 *      A pointer to the descriptor to deallocate. On return, a null
 *      descriptor. If you pass a null descriptor in this parameter,
 *      AEDisposeDesc returns noErr.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AEDisposeDesc( var theAEDesc: AEDesc ): OSErr; external name '_AEDisposeDesc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  AEDuplicateDesc()
 *  
 *  Summary:
 *    Creates a copy of a descriptor.
 *  
 *  Discussion:
 *    It is common for applications to send Apple events that have one
 *    or more attributes or parameters in common. For example, if you
 *    send a series of Apple events to the same application, the
 *    address attribute is the same. In these cases, the most efficient
 *    way to create the necessary Apple events is to make a template
 *    Apple event that you can then copy - by calling the
 *    AEDuplicateDesc function - as needed. You then fill in or change
 *    the remaining parameters and attributes of the copy, send the
 *    copy by calling the AESend function and, after AESend returns a
 *    result code, dispose of the copy by calling AEDisposeDesc. You
 *    can use this approach to prepare structures of type AEDesc,
 *    AEDescList, AERecord, and AppleEvent.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAEDesc:
 *      A pointer to the descriptor to duplicate. See AEDesc.
 *    
 *    result:
 *      A pointer to a descriptor. On return, the descriptor contains a
 *      copy of the descriptor specified by the theAEDesc parameter. If
 *      the function returns successfully, your application should call
 *      the AEDisposeDesc function to dispose of the resulting
 *      descriptor after it has finished using it.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function AEDuplicateDesc(const (*var*) theAEDesc: AEDesc; var result: AEDesc): OSErr; external name '_AEDuplicateDesc';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function AEDuplicateDesc( const (*var*) theAEDesc: AEDesc; var result: AEDesc ): OSErr; external name '_AEDuplicateDesc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4

{
 * Create an AEDesc with memory "borrowed" from the application. The
 * data passed in *must* be immutable and not freed until the Dispose
 * callback is made.
 * The dispose callback may be made at any time, including during the
 * creation of the descriptor.
 * If possible, the descriptor will be copied to the address space of
 * any recipient process using virtual memory APIs and avoid an
 * actual memory copy.
 }
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	AEDisposeExternalProcPtr = procedure( dataPtr: {const} UnivPtr; dataLength: Size; refcon: SRefCon );
	AEDisposeExternalUPP = AEDisposeExternalProcPtr;
{
 *  AECreateDescFromExternalPtr()
 *  
 *  Summary:
 *    Creates a new descriptor that uses a memory buffer supplied by
 *    the caller.
 *  
 *  Discussion:
 *    This function is different than AECreateDesc, in that it creates
 *    a descriptor that uses the data block provided by the caller "in
 *    place," rather than allocate a block of memory and copy the data
 *    to it. This function can provide dramatically improved
 *    performance if you're working with large chunks of data. It
 *    attempts to copy the descriptor to the address space of any
 *    recipient process using virtual memory APIs, avoiding an actual
 *    memory copy. For example, you might want to use this function to
 *    pass a large image in an Apple event.
 *    You can use the AEGetDescDataRange function to access a specific
 *    section of a large block of data.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    descriptorType:
 *      The descriptor type for the new descriptor.
 *    
 *    dataPtr:
 *      A pointer to the data for the new descriptor. The memory that
 *      is pointed to cannot be a Handle (which may move in memory),
 *      cannot be modified by the caller, and must be preserved in
 *      place (and not freed), until the disposeCallback function is
 *      called.
 *      If possible, the descriptor will be mapped into the address
 *      space of the recipient using shared memory, avoiding an actual
 *      memory copy.
 *      The pointer that is passed in does not need to be aligned to
 *      any particular boundary, but is optimized to transfer data on a
 *      page boundary. You can get the current page size (4096 on all
 *      current Mac OS X systems) with the getpagesize(3) call. (Type
 *      man 3 getpagesize in a Terminal window for documentation.)
 *    
 *    dataLength:
 *      The length, in bytes, of the data for the new descriptor.
 *    
 *    disposeCallback:
 *      A universal procedure pointer to a dispose callback function of
 *      type AEDisposeExternalProcPtr. Your callback function will be
 *      called when the block of memory provided by dataPtr is no
 *      longer needed by the Apple Event Manager. The function can be
 *      called at any time, including during creation of the descriptor.
 *    
 *    disposeRefcon:
 *      A reference constant the Apple Event Manager passes to the
 *      disposeCallback function whenever it calls the function. If
 *      your dispose function doesn't require a reference constant,
 *      pass 0 for this parameter.
 *    
 *    theDesc:
 *      A pointer to a descriptor. On successful return, a descriptor
 *      that incorporates the data specified by the dataPtr parameter.
 *      On error, a null descriptor. If the function returns
 *      successfully, your application should call the AEDisposeDesc
 *      function to dispose of the resulting descriptor after it has
 *      finished using it.
 *  
=======
=======
>>>>>>> origin/fixes_2_2
	AEDisposeExternalProcPtr = procedure( dataPtr: {const} UnivPtr; dataLength: Size; refcon: SInt32 );

{$ifc OPAQUE_UPP_TYPES}
	AEDisposeExternalUPP = ^SInt32; { an opaque UPP }
{$elsec}
	AEDisposeExternalUPP = UniversalProcPtr;
{$endc}	

=======
	AEDisposeExternalProcPtr = procedure( dataPtr: {const} UnivPtr; dataLength: Size; refcon: SRefCon );
	AEDisposeExternalUPP = AEDisposeExternalProcPtr;
>>>>>>> origin/fixes_2.4
{
 *  AECreateDescFromExternalPtr()
 *  
 *  Summary:
 *    Creates a new descriptor that uses a memory buffer supplied by
 *    the caller.
 *  
 *  Discussion:
 *    This function is different than AECreateDesc, in that it creates
 *    a descriptor that uses the data block provided by the caller "in
 *    place," rather than allocate a block of memory and copy the data
 *    to it. This function can provide dramatically improved
 *    performance if you're working with large chunks of data. It
 *    attempts to copy the descriptor to the address space of any
 *    recipient process using virtual memory APIs, avoiding an actual
 *    memory copy. For example, you might want to use this function to
 *    pass a large image in an Apple event.
 *    You can use the AEGetDescDataRange function to access a specific
 *    section of a large block of data.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  Parameters:
 *    
 *    descriptorType:
 *      The descriptor type for the new descriptor.
 *    
 *    dataPtr:
 *      A pointer to the data for the new descriptor. The memory that
 *      is pointed to cannot be a Handle (which may move in memory),
 *      cannot be modified by the caller, and must be preserved in
 *      place (and not freed), until the disposeCallback function is
 *      called.
 *      If possible, the descriptor will be mapped into the address
 *      space of the recipient using shared memory, avoiding an actual
 *      memory copy.
 *      The pointer that is passed in does not need to be aligned to
 *      any particular boundary, but is optimized to transfer data on a
 *      page boundary. You can get the current page size (4096 on all
 *      current Mac OS X systems) with the getpagesize(3) call. (Type
 *      man 3 getpagesize in a Terminal window for documentation.)
 *    
 *    dataLength:
 *      The length, in bytes, of the data for the new descriptor.
 *    
 *    disposeCallback:
 *      A universal procedure pointer to a dispose callback function of
 *      type AEDisposeExternalProcPtr. Your callback function will be
 *      called when the block of memory provided by dataPtr is no
 *      longer needed by the Apple Event Manager. The function can be
 *      called at any time, including during creation of the descriptor.
 *    
 *    disposeRefcon:
 *      A reference constant the Apple Event Manager passes to the
 *      disposeCallback function whenever it calls the function. If
 *      your dispose function doesn't require a reference constant,
 *      pass 0 for this parameter.
 *    
 *    theDesc:
 *      A pointer to a descriptor. On successful return, a descriptor
 *      that incorporates the data specified by the dataPtr parameter.
 *      On error, a null descriptor. If the function returns
 *      successfully, your application should call the AEDisposeDesc
 *      function to dispose of the resulting descriptor after it has
 *      finished using it.
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function AECreateDescFromExternalPtr( descriptorType: OSType; dataPtr: {const} UnivPtr; dataLength: Size; disposeCallback: AEDisposeExternalUPP; disposeRefcon: SRefCon; var theDesc: AEDesc ): OSStatus; external name '_AECreateDescFromExternalPtr';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_2, __IPHONE_NA ) *)

	

	
	{
	 *  AECompareDesc()
	 *  
	 *  Discussion:
	 *    Compare two AEDesc descriptors and return whether they are
	 *	  identical or not.
	 *  
	 *  Mac OS X threading:
	 *    Thread safe since version 10.8
	 *  
	 *  Parameters:
	 *    
	 *    desc1, desc2:
	 *      A pointer to an AEDesc to be compared.
	 *
	 *	  resultP:
	 *		If non-NULL, on a noErr return will be filled in with
	 *		true or false indicating whether the descriptors are
	 *		equilavent or not.
	 *
	 *  Two descriptors are identical if they are the same type and have
	 *  the same data; typeAEList descriptors must contain the same number
	 *  of items and every item in each list must itself be identical;
	 *  typeAERecord descriptors must contain the same number of keys
	 *  and values and each key/value must match between the two.
	 *  typeAppleEvents match like typeAERecord and also require that
	 *	most attributes of the two events are identical.
	 *  
	 *  Availability:
	 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
	 *    CarbonLib:        in CarbonLib 1.0 and later
	 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
	 }
function AECompareDesc( const (*var*) desc1: AEDesc; const (*var*) desc2: AEDesc; var resultP: Boolean ): OSStatus; external name '_AECompareDesc';
(* __OSX_AVAILABLE_STARTING( __MAC_10_8, __IPHONE_NA ) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew




=======
function AECreateDescFromExternalPtr( descriptorType: OSType; dataPtr: {const} UnivPtr; dataLength: Size; disposeCallback: AEDisposeExternalUPP; disposeRefcon: SInt32; var theDesc: AEDesc ): OSStatus; external name '_AECreateDescFromExternalPtr';
=======
function AECreateDescFromExternalPtr( descriptorType: OSType; dataPtr: {const} UnivPtr; dataLength: Size; disposeCallback: AEDisposeExternalUPP; disposeRefcon: SRefCon; var theDesc: AEDesc ): OSStatus; external name '_AECreateDescFromExternalPtr';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
function AECreateDescFromExternalPtr( descriptorType: OSType; dataPtr: {const} UnivPtr; dataLength: Size; disposeCallback: AEDisposeExternalUPP; disposeRefcon: SInt32; var theDesc: AEDesc ): OSStatus; external name '_AECreateDescFromExternalPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> origin/fixes_2_2


{*************************************************************************
  The following calls apply to AEDescList. Since AEDescList is a subtype of
  AEDesc, the calls in the previous section can also be used for AEDescList.
  All list and array indices are 1-based. If the data was greater than
  maximumSize in the routines below, then actualSize will be greater than
  maximumSize, but only maximumSize bytes will actually be retrieved.
*************************************************************************}
{
 *  AECreateList()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Discussion:
 *    Creates an empty descriptor list or Apple event record.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    factoringPtr:
 *      A pointer to the data at the beginning of each descriptor that
 *      is the same for all descriptors in the list. If there is no
 *      common data, or if you decide not to isolate the common data,
 *      pass NULL as the value of this parameter.
 *    
 *    factoredSize:
 *      The size of the common data. If there is no common data, or if
 *      you decide not to isolate the common data, pass 0 as the value
 *      of this parameter. (See the Discussion section for more
 *      information.)
 *    
 *    isRecord:
 *      A Boolean value that specifies the kind of list to create. Pass
 *      a value of TRUE to create an Apple event record (a data
 *      structure of type AERecord) or FALSE to create a descriptor
 *      list.
 *    
 *    resultList:
 *      A pointer to a descriptor list variable. On successful return,
 *      the descriptor list or Apple event record that the AECreateList
 *      function creates. On error, a null descriptor. See AEDescList.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AECreateList( factoringPtr: {const} UnivPtr; factoredSize: Size; isRecord: Boolean; var resultList: AEDescList ): OSErr; external name '_AECreateList';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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


{
 *  AECountItems()
 *  
 *  Discussion:
 *    Counts the number of descriptors in a descriptor list.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAEDescList:
 *      A pointer to the descriptor list to count
 *    
 *    theCount:
 *      A pointer to a count variable. On return, the number of
 *      descriptors in the specified descriptor list.  Currently an
 *      AEDescList is limited to 2^31 items.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AECountItems( const (*var*) theAEDescList: AEDescList; var theCount: SIGNEDLONG ): OSErr; external name '_AECountItems';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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


{
 *  AEPutPtr()
 *  
 *  Discussion:
 *    Puts data specified in a buffer to a descriptor list as a
 *    descriptor, possibly replacing an existing descriptor in the list.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAEDescList:
 *      A pointer to the descriptor list to add a descriptor to. See
 *      AEDescList.
 *    
 *    index:
 *      A one-based positive integer indicating the position to insert
 *      the descriptor at. If there is already a descriptor in the
 *      specified position, it is replaced.  You can pass a value of
 *      zero or count + 1 to add the descriptor at the end of the list.
 *      AEPutPtr returns an error (AEIllegalIndex) if you pass a
 *      negative number or a value that is out of range. Currently the
 *      upper limit on index is 2^31 items.
 *    
 *    typeCode:
 *      The descriptor type for the descriptor to be put into the list.
 *    
 *    dataPtr:
 *      A pointer to the data for the descriptor to add.
 *    
 *    dataSize:
 *      The length, in bytes, of the data for the descriptor to add.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AEPutPtr( var theAEDescList: AEDescList; index: SIGNEDLONG; typeCode: DescType; dataPtr: {const} UnivPtr; dataSize: Size ): OSErr; external name '_AEPutPtr';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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


{
 *  AEPutDesc()
 *  
 *  Discussion:
 *    Adds a descriptor to any descriptor list, possibly replacing an
 *    existing descriptor in the list.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAEDescList:
 *      A pointer to the descriptor list to add a descriptor to. See
 *      AEDescList.
 *    
 *    index:
 *      A one-based positive integer indicating the position to insert
 *      the descriptor at. If there is already a descriptor in the
 *      specified position, it is replaced.  You can pass a value of
 *      zero or count + 1 to add the descriptor at the end of the list.
 *      AEPutPtr returns an error (AEIllegalIndex) if you pass a
 *      negative number or a value that is out of range. Currently the
 *      upper limit on index is 2^31 items.
 *    
 *    theAEDesc:
 *      A pointer to the descriptor to add to the list.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AEPutDesc( var theAEDescList: AEDescList; index: SIGNEDLONG; const (*var*) theAEDesc: AEDesc ): OSErr; external name '_AEPutDesc';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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


{
 *  AEGetNthPtr()
 *  
 *  Discussion:
 *    Gets a copy of the data from a descriptor at a specified position
 *    in a descriptor list; typically used when your application needs
 *    to work with the extracted data directly.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAEDescList:
 *      A pointer to the descriptor list to add a descriptor to. See
 *      AEDescList.
 *    
 *    index:
 *      A one-based positive integer indicating the position in the
 *      descriptor list of the descriptor to get the data from.
 *      AEGetNthPtr returns an error if you pass zero, a negative
 *      number, or a value that is out of range. Currently the upper
 *      limit on index is 2^31 items.
 *    
 *    desiredType:
 *      The desired descriptor type for the copied data. For a list of
 *      AppleScript's predefined descriptor types. If the descriptor
 *      specified by the index parameter is not of the desired type,
 *      AEGetNthPtr attempts to coerce the data to this type. If you
 *      pass a value of typeWildCard, no coercion is performed, and the
 *      descriptor type of the copied data is the same as the
 *      descriptor type of the original descriptor.
 *    
 *    theAEKeyword:
 *      A pointer to a keyword or NULL. On return, the keyword for the
 *      specified descriptor, if you are getting data from a list of
 *      keyword-specified descriptors; otherwise, AEGetNthPtr returns
 *      the value typeWildCard.
 *    
 *    typeCode:
 *      A pointer to a descriptor type or NULL. On return, specifies
 *      the descriptor type of the data pointed to by dataPtr.
 *    
 *    dataPtr:
 *      A pointer to a buffer, local variable, or other storage
 *      location created and disposed of by your application. The size
 *      in bytes must be at least as large as the value you pass in the
 *      maximumSize parameter. On return, contains the data from the
 *      descriptor at the position in the descriptor list specified by
 *      the index parameter.
 *    
 *    maximumSize:
 *      The maximum length, in bytes, of the expected data. The
 *      AEGetNthPtr function will not return more data than you specify
 *      in this parameter.
 *    
 *    actualSize:
 *      A pointer to a size variable or NULL. On return, the length, in
 *      bytes, of the data for the specified descriptor. If this value
 *      is larger than the value of the maximumSize parameter, the
 *      buffer pointed to by dataPtr was not large enough to contain
 *      all of the data for the descriptor, though AEGetNthPtr does not
 *      write beyond the end of the buffer. If the buffer was too
 *      small, you can resize it and call AEGetNthPtr again.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AEGetNthPtr( const (*var*) theAEDescList: AEDescList; index: SIGNEDLONG; desiredType: DescType; theAEKeyword: AEKeywordPtr { can be NULL }; typeCode: DescTypePtr { can be NULL }; dataPtr: UnivPtr; maximumSize: Size; actualSize: SizePtr { can be NULL } ): OSErr; external name '_AEGetNthPtr';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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


{
 *  AEGetNthDesc()
 *  
 *  Discussion:
 *    Copies a descriptor from a specified position in a descriptor
 *    list into a specified descriptor; typically used when your
 *    application needs to pass the extracted data to another function
 *    as a descriptor.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAEDescList:
 *      A pointer to the descriptor list to add a descriptor to. See
 *      AEDescList.
 *    
 *    index:
 *      A one-based positive integer indicating the position in the
 *      descriptor list of the descriptor to get the data from.
 *      AEGetNthDesc returns an error if you pass zero, a negative
 *      number, or a value that is out of range. Currently the upper
 *      limit on index is 2^31 items.
 *    
 *    desiredType:
 *      The desired descriptor type for the copied data. For a list of
 *      AppleScript's predefined descriptor types.  If the descriptor
 *      specified by the index parameter is not of the desired type,
 *      AEGetNthDesc attempts to coerce the data to this type. If you
 *      pass a value of typeWildCard, no coercion is performed, and the
 *      descriptor type of the copied data is the same as the
 *      descriptor type of the original descriptor.
 *    
 *    theAEKeyword:
 *      A pointer to a keyword or NULL. On return, the keyword for the
 *      specified descriptor, if you are getting data from a list of
 *      keyword-specified descriptors; otherwise, AEGetNthDesc returns
 *      the value typeWildCard.
 *    
 *    result:
 *      A pointer to a descriptor. On successful return, a copy of the
 *      descriptor specified by the index parameter, coerced, if
 *      necessary, to the descriptor type specified by the desiredType
 *      parameter. On error, a null descriptor. If the function returns
 *      successfully, your application should call the AEDisposeDesc
 *      function to dispose of the resulting descriptor after it has
 *      finished using it.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AEGetNthDesc( const (*var*) theAEDescList: AEDescList; index: SIGNEDLONG; desiredType: DescType; theAEKeyword: AEKeywordPtr { can be NULL }; var result: AEDesc ): OSErr; external name '_AEGetNthDesc';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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


{
 *  AESizeOfNthItem()
 *  
 *  Discussion:
 *    Gets the data size and descriptor type of the descriptor at a
 *    specified position in a descriptor list.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAEDescList:
 *      A pointer to the descriptor list to add a descriptor to. See
 *      AEDescList.
 *    
 *    index:
 *      A one-based positive integer indicating the position in the
 *      descriptor list of the descriptor to get the data from.
 *      AESizeOfNthItem returns an error if you pass zero, a negative
 *      number, or a value that is out of range. Currently the upper
 *      limit on index is 2^31 items.
 *    
 *    typeCode:
 *      A pointer to a descriptor type or NULL. On return, specifies
 *      the descriptor type of the descriptor.
 *    
 *    dataSize:
 *      A pointer to a size variable or NULL. On return, the length (in
 *      bytes) of the data in the descriptor.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AESizeOfNthItem( const (*var*) theAEDescList: AEDescList; index: SIGNEDLONG; typeCode: DescTypePtr { can be NULL }; dataSize: SizePtr { can be NULL } ): OSErr; external name '_AESizeOfNthItem';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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


{
 *  AEGetArray()
 *  
 *  Discussion:
 *    Extracts data from an Apple event array created with the
 *    AEPutArray function and stores it as a standard array of fixed
 *    size items in the specified buffer.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAEDescList:
 *      A pointer to the descriptor list to add a descriptor to. See
 *      AEDescList.
 *    
 *    arrayType:
 *      The Apple event array type to convert. Pass one of the
 *      constants:  kAEDataArray, kAEPackedArray, kAEDescArray,
 *      kAEKeyDescArray
 *    
 *    arrayPtr:
 *      A pointer to a buffer, allocated and disposed of by your
 *      application, for storing the array. The size in bytes must be
 *      at least as large as the value you pass in the maximumSize
 *      parameter. On return, the buffer contains the array of
 *      fixed-size items.
 *    
 *    maximumSize:
 *      The maximum length, in bytes, of the expected data. The
 *      AEGetArray function will not return more data than you specify
 *      in this parameter.
 *    
 *    itemType:
 *      A pointer to a descriptor type. On return, for arrays of type
 *      kAEDataArray, kAEPackedArray, or kAEHandleArray, the descriptor
 *      type of the items in the returned array. The AEGetArray
 *      function doesn't supply a value in itemType for arrays of type
 *      kAEDescArray and kAEKeyDescArray because they may contain
 *      descriptors of different types.
 *    
 *    itemSize:
 *      A pointer to a size variable. On return, for arrays of type
 *      kAEDataArray or kAEPackedArray, the size (in bytes) of each
 *      item in the returned array. You don't get an item size for
 *      arrays of type kAEDescArray, kAEKeyDescArray, or kAEHandleArray
 *      because descriptors and handles (though not the data they point
 *      to) have a known size.
 *    
 *    itemCount:
 *      A pointer to a size variable. On return, the number of items in
 *      the returned array.  Currently the upper limit on the size of
 *      an array is 2^31 items.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AEGetArray( const (*var*) theAEDescList: AEDescList; arrayType: AEArrayType; arrayPtr: AEArrayDataPointer; maximumSize: Size; var itemType: DescType; var itemSize: Size; var itemCount: SIGNEDLONG ): OSErr; external name '_AEGetArray';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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


{
 *  AEPutArray()
 *  
 *  Discussion:
 *    Extracts data from an Apple event array created with the
 *    AEPutArray function and stores it as a standard array of fixed
 *    size items in the specified buffer.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAEDescList:
 *      A pointer to the descriptor list to add a descriptor to. See
 *      AEDescList.
 *    
 *    arrayType:
 *      The Apple event array type to convert. Pass one of the
 *      constants:  kAEDataArray, kAEPackedArray, kAEDescArray,
 *      kAEKeyDescArray
 *    
 *    arrayPtr:
 *      A pointer to a buffer, local variable, or other storage
 *      location, created and disposed of by your application, that
 *      contains the array to put into the descriptor list.
 *    
 *    itemType:
 *      For arrays of type kAEDataArray, kAEPackedArray, or
 *      kAEHandleArray, the descriptor type of the array items to
 *      create. Use one of the constants such as typeLongInteger. You
 *      don't need to specify an item type for arrays of type
 *      kAEDescArray or kAEKeyDescArray because the data is already
 *      stored in descriptors which contain a descriptor type.
 *    
 *    itemSize:
 *      For arrays of type kAEDataArray or kAEPackedArray, the size (in
 *      bytes) of the array items to create. You don't need to specify
 *      an item size for arrays of type kAEDescArray, kAEKeyDescArray,
 *      or kAEHandleArray because their descriptors (though not the
 *      data they point to) have a known size.
 *    
 *    itemCount:
 *      A pointer to a size variable. On return, the number of items in
 *      the returned array.  Currently the upper limit on the size of
 *      an array is 2^31 items.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AEPutArray( var theAEDescList: AEDescList; arrayType: AEArrayType; const (*var*) arrayPtr: AEArrayData; itemType: DescType; itemSize: Size; itemCount: SIGNEDLONG ): OSErr; external name '_AEPutArray';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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


{
 *  AEDeleteItem()
 *  
 *  Discussion:
 *    Deletes a descriptor from a descriptor list, causing all
 *    subsequent descriptors to move up one place.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAEDescList:
 *      A pointer to the descriptor list to add a descriptor to. See
 *      AEDescList.
 *    
 *    index:
 *      A one-based positive integer indicating the position in the
 *      descriptor list of the descriptor to delete. AEDeleteItem
 *      returns an error if you pass zero, a negative number, or a
 *      value that is out of range. Currently the upper limit on index
 *      is 2^31 items.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AEDeleteItem( var theAEDescList: AEDescList; index: SIGNEDLONG ): OSErr; external name '_AEDeleteItem';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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
=======
>>>>>>> origin/fixes_2_2
=======
 *  Discussion:
 *    Creates an empty descriptor list or Apple event record.
 *  
>>>>>>> origin/fixes_2.4
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    factoringPtr:
 *      A pointer to the data at the beginning of each descriptor that
 *      is the same for all descriptors in the list. If there is no
 *      common data, or if you decide not to isolate the common data,
 *      pass NULL as the value of this parameter.
 *    
 *    factoredSize:
 *      The size of the common data. If there is no common data, or if
 *      you decide not to isolate the common data, pass 0 as the value
 *      of this parameter. (See the Discussion section for more
 *      information.)
 *    
 *    isRecord:
 *      A Boolean value that specifies the kind of list to create. Pass
 *      a value of TRUE to create an Apple event record (a data
 *      structure of type AERecord) or FALSE to create a descriptor
 *      list.
 *    
 *    resultList:
 *      A pointer to a descriptor list variable. On successful return,
 *      the descriptor list or Apple event record that the AECreateList
 *      function creates. On error, a null descriptor. See AEDescList.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AECreateList( factoringPtr: {const} UnivPtr; factoredSize: Size; isRecord: Boolean; var resultList: AEDescList ): OSErr; external name '_AECreateList';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  AECountItems()
 *  
 *  Discussion:
 *    Counts the number of descriptors in a descriptor list.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAEDescList:
 *      A pointer to the descriptor list to count
 *    
 *    theCount:
 *      A pointer to a count variable. On return, the number of
 *      descriptors in the specified descriptor list.  Currently an
 *      AEDescList is limited to 2^31 items.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AECountItems( const (*var*) theAEDescList: AEDescList; var theCount: SIGNEDLONG ): OSErr; external name '_AECountItems';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  AEPutPtr()
 *  
 *  Discussion:
 *    Puts data specified in a buffer to a descriptor list as a
 *    descriptor, possibly replacing an existing descriptor in the list.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAEDescList:
 *      A pointer to the descriptor list to add a descriptor to. See
 *      AEDescList.
 *    
 *    index:
 *      A one-based positive integer indicating the position to insert
 *      the descriptor at. If there is already a descriptor in the
 *      specified position, it is replaced.  You can pass a value of
 *      zero or count + 1 to add the descriptor at the end of the list.
 *      AEPutPtr returns an error (AEIllegalIndex) if you pass a
 *      negative number or a value that is out of range. Currently the
 *      upper limit on index is 2^31 items.
 *    
 *    typeCode:
 *      The descriptor type for the descriptor to be put into the list.
 *    
 *    dataPtr:
 *      A pointer to the data for the descriptor to add.
 *    
 *    dataSize:
 *      The length, in bytes, of the data for the descriptor to add.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AEPutPtr( var theAEDescList: AEDescList; index: SIGNEDLONG; typeCode: DescType; dataPtr: {const} UnivPtr; dataSize: Size ): OSErr; external name '_AEPutPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  AEPutDesc()
 *  
 *  Discussion:
 *    Adds a descriptor to any descriptor list, possibly replacing an
 *    existing descriptor in the list.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAEDescList:
 *      A pointer to the descriptor list to add a descriptor to. See
 *      AEDescList.
 *    
 *    index:
 *      A one-based positive integer indicating the position to insert
 *      the descriptor at. If there is already a descriptor in the
 *      specified position, it is replaced.  You can pass a value of
 *      zero or count + 1 to add the descriptor at the end of the list.
 *      AEPutPtr returns an error (AEIllegalIndex) if you pass a
 *      negative number or a value that is out of range. Currently the
 *      upper limit on index is 2^31 items.
 *    
 *    theAEDesc:
 *      A pointer to the descriptor to add to the list.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AEPutDesc( var theAEDescList: AEDescList; index: SIGNEDLONG; const (*var*) theAEDesc: AEDesc ): OSErr; external name '_AEPutDesc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  AEGetNthPtr()
 *  
 *  Discussion:
 *    Gets a copy of the data from a descriptor at a specified position
 *    in a descriptor list; typically used when your application needs
 *    to work with the extracted data directly.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAEDescList:
 *      A pointer to the descriptor list to add a descriptor to. See
 *      AEDescList.
 *    
 *    index:
 *      A one-based positive integer indicating the position in the
 *      descriptor list of the descriptor to get the data from.
 *      AEGetNthPtr returns an error if you pass zero, a negative
 *      number, or a value that is out of range. Currently the upper
 *      limit on index is 2^31 items.
 *    
 *    desiredType:
 *      The desired descriptor type for the copied data. For a list of
 *      AppleScript's predefined descriptor types. If the descriptor
 *      specified by the index parameter is not of the desired type,
 *      AEGetNthPtr attempts to coerce the data to this type. If you
 *      pass a value of typeWildCard, no coercion is performed, and the
 *      descriptor type of the copied data is the same as the
 *      descriptor type of the original descriptor.
 *    
 *    theAEKeyword:
 *      A pointer to a keyword or NULL. On return, the keyword for the
 *      specified descriptor, if you are getting data from a list of
 *      keyword-specified descriptors; otherwise, AEGetNthPtr returns
 *      the value typeWildCard.
 *    
 *    typeCode:
 *      A pointer to a descriptor type or NULL. On return, specifies
 *      the descriptor type of the data pointed to by dataPtr.
 *    
 *    dataPtr:
 *      A pointer to a buffer, local variable, or other storage
 *      location created and disposed of by your application. The size
 *      in bytes must be at least as large as the value you pass in the
 *      maximumSize parameter. On return, contains the data from the
 *      descriptor at the position in the descriptor list specified by
 *      the index parameter.
 *    
 *    maximumSize:
 *      The maximum length, in bytes, of the expected data. The
 *      AEGetNthPtr function will not return more data than you specify
 *      in this parameter.
 *    
 *    actualSize:
 *      A pointer to a size variable or NULL. On return, the length, in
 *      bytes, of the data for the specified descriptor. If this value
 *      is larger than the value of the maximumSize parameter, the
 *      buffer pointed to by dataPtr was not large enough to contain
 *      all of the data for the descriptor, though AEGetNthPtr does not
 *      write beyond the end of the buffer. If the buffer was too
 *      small, you can resize it and call AEGetNthPtr again.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AEGetNthPtr( const (*var*) theAEDescList: AEDescList; index: SIGNEDLONG; desiredType: DescType; theAEKeyword: AEKeywordPtr { can be NULL }; typeCode: DescTypePtr { can be NULL }; dataPtr: UnivPtr; maximumSize: Size; actualSize: SizePtr { can be NULL } ): OSErr; external name '_AEGetNthPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  AEGetNthDesc()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function AEGetNthDesc(const (*var*) theAEDescList: AEDescList; index: SInt32; desiredType: DescType; theAEKeywordPtr: AEKeywordPtr; var result: AEDesc): OSErr; external name '_AEGetNthDesc';
{
 *  AESizeOfNthItem()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function AESizeOfNthItem(const (*var*) theAEDescList: AEDescList; index: SInt32; typeCode: DescTypePtr; dataSize: SizePtr): OSErr; external name '_AESizeOfNthItem';
{
 *  AEGetArray()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function AEGetArray(const (*var*) theAEDescList: AEDescList; arrayType: AEArrayType; arrayPtr: AEArrayDataPointer; maximumSize: Size; var itemType: DescType; var itemSize: Size; var itemCount: SInt32): OSErr; external name '_AEGetArray';
{
 *  AEPutArray()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function AEPutArray(var theAEDescList: AEDescList; arrayType: AEArrayType; const (*var*) arrayPtr: AEArrayData; itemType: DescType; itemSize: Size; itemCount: SInt32): OSErr; external name '_AEPutArray';
{
 *  AEDeleteItem()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function AEDeleteItem(var theAEDescList: AEDescList; index: SInt32): OSErr; external name '_AEDeleteItem';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{*************************************************************************
 The following calls apply to AERecord. Since AERecord is a subtype of
 AEDescList, the calls in the previous sections can also be used for
 AERecord an AERecord can be created by using AECreateList with isRecord
 set to true. 
*************************************************************************}
{************************************************************************
 AERecords can have an abitrary descriptorType.  This allows you to
 check if desc is truly an AERecord
***********************************************************************}
{
 *  AECheckIsRecord()
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.4 and later
 *    Non-Carbon CFM:   not available
 }
function AECheckIsRecord( const (*var*) theDesc: AEDesc ): Boolean; external name '_AECheckIsRecord';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        in CarbonLib 1.4 and later
 *    Mac OS X:         in version 10.0 and later
 }
function AECheckIsRecord(const (*var*) theDesc: AEDesc): boolean; external name '_AECheckIsRecord';

{
  Note: none of the “key” calls were available in the PowerPC 7.x IntefaceLib.
  In C, a #define is used to map “key” calls to “param” calls.  In pascal
  this mapping is done in externally linked glue code.
}
{$ifc CALL_NOT_IN_CARBON}
{
 *  AEPutKeyPtr()
 *  
 *  Availability:
<<<<<<< HEAD
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function AEPutKeyPtr(var theAERecord: AERecord; theAEKeyword: AEKeyword; typeCode: DescType; dataPtr: UnivPtr; dataSize: Size): OSErr; external name '_AEPutKeyPtr';
{
 *  AEPutKeyDesc()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function AEPutKeyDesc(var theAERecord: AERecord; theAEKeyword: AEKeyword; const (*var*) theAEDesc: AEDesc): OSErr; external name '_AEPutKeyDesc';
{
 *  AEGetKeyPtr()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function AEGetKeyPtr(const (*var*) theAERecord: AERecord; theAEKeyword: AEKeyword; desiredType: DescType; var typeCode: DescType; dataPtr: UnivPtr; maximumSize: Size; var actualSize: Size): OSErr; external name '_AEGetKeyPtr';
{
 *  AEGetKeyDesc()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function AEGetKeyDesc(const (*var*) theAERecord: AERecord; theAEKeyword: AEKeyword; desiredType: DescType; var result: AEDesc): OSErr; external name '_AEGetKeyDesc';
{
 *  AESizeOfKeyDesc()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function AESizeOfKeyDesc(const (*var*) theAERecord: AERecord; theAEKeyword: AEKeyword; var typeCode: DescType; var dataSize: Size): OSErr; external name '_AESizeOfKeyDesc';
{
 *  AEDeleteKeyDesc()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function AEDeleteKeyDesc(var theAERecord: AERecord; theAEKeyword: AEKeyword): OSErr; external name '_AEDeleteKeyDesc';
{$endc}  {CALL_NOT_IN_CARBON}
>>>>>>> graemeg/fixes_2_2

{*************************************************************************
  The following calls create and manipulate the AppleEvent data type.
*************************************************************************}
{
 *  AECreateAppleEvent()
 *  
<<<<<<< HEAD
 *  Summary:
 *    Creates an Apple event with several important attributes but no
 *    parameters.
 *  
 *  Discussion:
 *    The AECreateAppleEvent function creates an empty Apple event. You
 *    can add parameters to the Apple event after you create it with
 *    the functions described in "Adding Parameters and Attributes to
 *    an Apple Event".
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAEEventClass:
 *      The event class of the Apple event to create. This parameter
 *      becomes accessible through the keyEventClassAttr attribute of
 *      the Apple event. Some event classes are described in "Event
 *      Class Constants".
 *    
 *    theAEEventID:
 *      The event ID of the Apple event to create. This parameter
 *      becomes accessible through the keyEventIDAttr attribute of the
 *      Apple event.
 *    
 *    target:
 *      A pointer to an address descriptor. Before calling
 *      AECreateAppleEvent, you set the descriptor to identify the
 *      target (or server) application for the Apple event. This
 *      parameter becomes accessible through the keyAddressAttr
 *      attribute of the Apple event.
 *    
 *    returnID:
 *      The return ID for the created Apple event. If you pass a value
 *      of kAutoGenerateReturnID, the Apple Event Manager assigns the
 *      created Apple event a return ID that is unique to the current
 *      session. If you pass any other value, the Apple Event Manager
 *      assigns that value for the ID. This parameter becomes
 *      accessible through the keyReturnIDAttr attribute of the Apple
 *      event. The return ID constant is described in "ID Constants for
 *      the AECreateAppleEvent Function".
 *    
 *    transactionID:
 *      The transaction ID for this Apple event. A transaction is a
 *      sequence of Apple events that are sent back and forth between
 *      the client and server applications, beginning with the client's
 *      initial request for a service. All Apple events that are part
 *      of a transaction must have the same transaction ID. You can
 *      specify the kAnyTransactionID constant if the Apple event is
 *      not one of a series of interdependent Apple events. This
 *      parameter becomes accessible through the keyTransactionIDAttr
 *      attribute of the Apple event.
 *    
 *    result:
 *      A pointer to an Apple event. On successful return, the new
 *      Apple event. On error, a null descriptor (one with descriptor
 *      type typeNull). If the function returns successfully, your
 *      application should call the AEDisposeDesc function to dispose
 *      of the resulting Apple event after it has finished using it.
 *      See the AppleEvent data type.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AECreateAppleEvent( theAEEventClass: AEEventClass; theAEEventID: AEEventID; const (*var*) target: AEAddressDesc; returnID: AEReturnID; transactionID: AETransactionID; var result: AppleEvent ): OSErr; external name '_AECreateAppleEvent';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Availability:
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Mac OS X:         in version 10.0 and later
 }
function AECreateAppleEvent(theAEEventClass: AEEventClass; theAEEventID: AEEventID; const (*var*) target: AEAddressDesc; returnID: AEReturnID; transactionID: AETransactionID; var result: AppleEvent): OSErr; external name '_AECreateAppleEvent';
>>>>>>> graemeg/fixes_2_2
{*************************************************************************
  The following calls are used to pack and unpack parameters from records
  of type AppleEvent. Since AppleEvent is a subtype of AERecord, the calls
  in the previous sections can also be used for variables of type
  AppleEvent. The next six calls are in fact identical to the six calls
  for AERecord.
*************************************************************************}
{
 *  AEPutParamPtr()
 *  
<<<<<<< HEAD
 *  Discussion:
 *    Puts a pointer to data, a descriptor type, and a keyword into an
 *    Apple event or Apple event record as an Apple event parameter.
 *  
 *  Discussion:
 *    Copies a descriptor from a specified position in a descriptor
 *    list into a specified descriptor; typically used when your
 *    application needs to pass the extracted data to another function
 *    as a descriptor.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
<<<<<<< HEAD
 *    theAppleEvent:
 *      A pointer to the Apple event to add a parameter to. See the
 *      AppleEvent data type.
 *    
 *    theAEKeyword:
 *      The keyword for the parameter to add. If the Apple event
 *      already includes an parameter with this keyword, the parameter
 *      is replaced.
 *    
 *    typeCode:
 *      The descriptor type for the parameter to add.
 *    
 *    dataPtr:
 *      A pointer to the data for the parameter to add.
 *    
 *    dataSize:
 *      The length, in bytes, of the data for the parameter to add.
=======
 *    theAEDescList:
 *      A pointer to the descriptor list to add a descriptor to. See
 *      AEDescList.
 *    
 *    index:
 *      A one-based positive integer indicating the position in the
 *      descriptor list of the descriptor to get the data from.
 *      AEGetNthDesc returns an error if you pass zero, a negative
 *      number, or a value that is out of range. Currently the upper
 *      limit on index is 2^31 items.
 *    
 *    desiredType:
 *      The desired descriptor type for the copied data. For a list of
 *      AppleScript's predefined descriptor types.  If the descriptor
 *      specified by the index parameter is not of the desired type,
 *      AEGetNthDesc attempts to coerce the data to this type. If you
 *      pass a value of typeWildCard, no coercion is performed, and the
 *      descriptor type of the copied data is the same as the
 *      descriptor type of the original descriptor.
 *    
 *    theAEKeyword:
 *      A pointer to a keyword or NULL. On return, the keyword for the
 *      specified descriptor, if you are getting data from a list of
 *      keyword-specified descriptors; otherwise, AEGetNthDesc returns
 *      the value typeWildCard.
 *    
 *    result:
 *      A pointer to a descriptor. On successful return, a copy of the
 *      descriptor specified by the index parameter, coerced, if
 *      necessary, to the descriptor type specified by the desiredType
 *      parameter. On error, a null descriptor. If the function returns
 *      successfully, your application should call the AEDisposeDesc
 *      function to dispose of the resulting descriptor after it has
 *      finished using it.
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function AEPutParamPtr( var theAppleEvent: AppleEvent; theAEKeyword: AEKeyword; typeCode: DescType; dataPtr: {const} UnivPtr; dataSize: Size ): OSErr; external name '_AEPutParamPtr';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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
function AEGetNthDesc( const (*var*) theAEDescList: AEDescList; index: SIGNEDLONG; desiredType: DescType; theAEKeyword: AEKeywordPtr { can be NULL }; var result: AEDesc ): OSErr; external name '_AEGetNthDesc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  AEPutParamDesc()
 *  
 *  Discussion:
 *    Puts a descriptor and a keyword into an Apple event or Apple
 *    event record as an Apple event parameter.
 *  
 *  Discussion:
 *    Gets the data size and descriptor type of the descriptor at a
 *    specified position in a descriptor list.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
<<<<<<< HEAD
 *    theAppleEvent:
 *      A pointer to the Apple event to add a parameter to.
 *    
 *    theAEKeyword:
 *      The keyword specifying the parameter to add. If the Apple event
 *      already has a parameter with this keyword, the parameter is
 *      replaced.
 *    
 *    theAEDesc:
 *      A pointer to the descriptor for the parameter to add. See
 *      AEDesc.
=======
 *    theAEDescList:
 *      A pointer to the descriptor list to add a descriptor to. See
 *      AEDescList.
 *    
 *    index:
 *      A one-based positive integer indicating the position in the
 *      descriptor list of the descriptor to get the data from.
 *      AESizeOfNthItem returns an error if you pass zero, a negative
 *      number, or a value that is out of range. Currently the upper
 *      limit on index is 2^31 items.
 *    
 *    typeCode:
 *      A pointer to a descriptor type or NULL. On return, specifies
 *      the descriptor type of the descriptor.
 *    
 *    dataSize:
 *      A pointer to a size variable or NULL. On return, the length (in
 *      bytes) of the data in the descriptor.
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function AEPutParamDesc( var theAppleEvent: AppleEvent; theAEKeyword: AEKeyword; const (*var*) theAEDesc: AEDesc ): OSErr; external name '_AEPutParamDesc';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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
function AESizeOfNthItem( const (*var*) theAEDescList: AEDescList; index: SIGNEDLONG; typeCode: DescTypePtr { can be NULL }; dataSize: SizePtr { can be NULL } ): OSErr; external name '_AESizeOfNthItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  AEGetParamPtr()
 *  
<<<<<<< HEAD
 *  Summary:
 *    Gets a copy of the data for a specified Apple event parameter
 *    from an Apple event or an Apple event record (type AERecord);
 *    typically used when your application needs to work with the
 *    extracted data directly.
 *  
 *  Discussion:
 *    You should use this function only to extract data from value
 *    descriptors such as typeUTF8Text.
 *    Because this function allows you to specify a desired type, it
 *    can result in coercion. When used correctly, this has the
 *    positive effect of returning the data in the desired format.
 *    However, it can have side effects you may not be expecting, such
 *    as the overhead of calls to coercion handlers. See also the
 *    Version Notes section below for possible problems with
 *    coercion.
 *    To get Apple event parameter data for your application to use
 *    directly, call AEGetParamPtr. To get a descriptor for an Apple
 *    event parameter to pass on to another Apple Event Manager
 *    routine, call AEGetParamDesc.
 *    Before calling AEGetParamPtr, you can call the AESizeOfParam
 *    function to determine a size for the dataPtr buffer. However,
 *    unless you specify typeWildCard for the desiredType parameter,
 *    AEGetParamPtr may coerce the data, which may cause the size of
 *    the data to change.
 *    In some cases, you may get improved efficiency extracting
 *    information from an Apple event with the AEGetDescDataRange
 *    function.
=======
 *  Discussion:
 *    Extracts data from an Apple event array created with the
 *    AEPutArray function and stores it as a standard array of fixed
 *    size items in the specified buffer.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAEDescList:
 *      A pointer to the descriptor list to add a descriptor to. See
 *      AEDescList.
 *    
 *    arrayType:
 *      The Apple event array type to convert. Pass one of the
 *      constants:  kAEDataArray, kAEPackedArray, kAEDescArray,
 *      kAEKeyDescArray
 *    
 *    arrayPtr:
 *      A pointer to a buffer, allocated and disposed of by your
 *      application, for storing the array. The size in bytes must be
 *      at least as large as the value you pass in the maximumSize
 *      parameter. On return, the buffer contains the array of
 *      fixed-size items.
 *    
 *    maximumSize:
 *      The maximum length, in bytes, of the expected data. The
 *      AEGetArray function will not return more data than you specify
 *      in this parameter.
 *    
 *    itemType:
 *      A pointer to a descriptor type. On return, for arrays of type
 *      kAEDataArray, kAEPackedArray, or kAEHandleArray, the descriptor
 *      type of the items in the returned array. The AEGetArray
 *      function doesn't supply a value in itemType for arrays of type
 *      kAEDescArray and kAEKeyDescArray because they may contain
 *      descriptors of different types.
 *    
 *    itemSize:
 *      A pointer to a size variable. On return, for arrays of type
 *      kAEDataArray or kAEPackedArray, the size (in bytes) of each
 *      item in the returned array. You don't get an item size for
 *      arrays of type kAEDescArray, kAEKeyDescArray, or kAEHandleArray
 *      because descriptors and handles (though not the data they point
 *      to) have a known size.
 *    
 *    itemCount:
 *      A pointer to a size variable. On return, the number of items in
 *      the returned array.  Currently the upper limit on the size of
 *      an array is 2^31 items.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AEGetArray( const (*var*) theAEDescList: AEDescList; arrayType: AEArrayType; arrayPtr: AEArrayDataPointer; maximumSize: Size; var itemType: DescType; var itemSize: Size; var itemCount: SIGNEDLONG ): OSErr; external name '_AEGetArray';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  AEPutArray()
>>>>>>> origin/fixes_2.4
 *  
 *  Discussion:
 *    Extracts data from an Apple event array created with the
 *    AEPutArray function and stores it as a standard array of fixed
 *    size items in the specified buffer.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
<<<<<<< HEAD
 *    theAppleEvent:
 *      A pointer to the Apple event to get the parameter data from.
 *      See AppleEvent.
 *    
 *    theAEKeyword:
 *      The keyword that specifies the desired Apple event parameter.
 *    
 *    desiredType:
 *      The desired descriptor type for the copied data. If the
 *      descriptor specified by the theAEKeyword parameter is not of
 *      the desired type, AEGetParamPtr attempts to coerce the data to
 *      this type. However, if the desired type is typeWildCard, no
 *      coercion is performed. On return, you can determine the actual
 *      descriptor type by examining the typeCode parameter.
 *    
 *    actualType:
 *      A pointer to a descriptor type. On return, specifies the
 *      descriptor type of the data pointed to by dataPtr. The returned
 *      type is either the same as the type specified by the
 *      desiredType parameter or, if the desired type was typeWildcard,
 *      the true type of the descriptor. Specify NULL if you do not
 *      care about this return value.
 *    
 *    dataPtr:
 *      A pointer to a buffer, local variable, or other storage
 *      location created and disposed of by your application. The size
 *      in bytes must be at least as large as the value you pass in the
 *      maximumSize parameter. On return, contains the parameter data.
 *      Specify NULL if you do not care about this return value.
 *    
 *    maximumSize:
 *      The maximum length, in bytes, of the expected Apple event
 *      parameter data. The AEGetParamPtr function will not return more
 *      data than you specify in this parameter.
 *    
 *    actualSize:
 *      A pointer to a variable of type Size. On return, the length, in
 *      bytes, of the data for the specified Apple event parameter. If
 *      this value is larger than the value you passed in the
 *      maximumSize parameter, the buffer pointed to by dataPtr was not
 *      large enough to contain all of the data for the parameter,
 *      though AEGetParamPtr does not write beyond the end of the
 *      buffer. If the buffer was too small, you can resize it and call
 *      AEGetParamPtr again. Specify NULL if you do not care about this
 *      return value.
=======
 *    theAEDescList:
 *      A pointer to the descriptor list to add a descriptor to. See
 *      AEDescList.
 *    
 *    arrayType:
 *      The Apple event array type to convert. Pass one of the
 *      constants:  kAEDataArray, kAEPackedArray, kAEDescArray,
 *      kAEKeyDescArray
 *    
 *    arrayPtr:
 *      A pointer to a buffer, local variable, or other storage
 *      location, created and disposed of by your application, that
 *      contains the array to put into the descriptor list.
 *    
 *    itemType:
 *      For arrays of type kAEDataArray, kAEPackedArray, or
 *      kAEHandleArray, the descriptor type of the array items to
 *      create. Use one of the constants such as typeLongInteger. You
 *      don't need to specify an item type for arrays of type
 *      kAEDescArray or kAEKeyDescArray because the data is already
 *      stored in descriptors which contain a descriptor type.
 *    
 *    itemSize:
 *      For arrays of type kAEDataArray or kAEPackedArray, the size (in
 *      bytes) of the array items to create. You don't need to specify
 *      an item size for arrays of type kAEDescArray, kAEKeyDescArray,
 *      or kAEHandleArray because their descriptors (though not the
 *      data they point to) have a known size.
 *    
 *    itemCount:
 *      A pointer to a size variable. On return, the number of items in
 *      the returned array.  Currently the upper limit on the size of
 *      an array is 2^31 items.
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function AEGetParamPtr( const (*var*) theAppleEvent: AppleEvent; theAEKeyword: AEKeyword; desiredType: DescType; actualType: DescTypePtr { can be NULL }; dataPtr: UnivPtr; maximumSize: Size; actualSize: SizePtr { can be NULL } ): OSErr; external name '_AEGetParamPtr';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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
function AEPutArray( var theAEDescList: AEDescList; arrayType: AEArrayType; const (*var*) arrayPtr: AEArrayData; itemType: DescType; itemSize: Size; itemCount: SIGNEDLONG ): OSErr; external name '_AEPutArray';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4


{
 *  AEGetParamDesc()
 *  
 *  Summary:
 *    Gets a copy of the descriptor for a keyword-specified Apple event
 *    parameter from an Apple event or an Apple event record (type
 *    AERecord); typically used when your application needs to pass the
 *    extracted data to another function as a descriptor.
 *  
 *  Discussion:
 *    To get Apple event parameter data for your application to use
 *    directly, call AEGetParamPtr. To get a descriptor for an Apple
 *    event parameter to pass on to another Apple Event Manager
 *    routine, call AEGetParamDesc.
 *    If the actual parameter you are getting with AEGetParamDesc is a
 *    record, AEGetParamDesc will only allow you to request it as a
 *    typeAERecord, typeAEList, or typeWildcard. For any other type, it
 *    will return errAECoercionFail.
 *  
 *  Discussion:
 *    Deletes a descriptor from a descriptor list, causing all
 *    subsequent descriptors to move up one place.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
<<<<<<< HEAD
 *    theAppleEvent:
 *      A pointer to the Apple event to get the parameter descriptor
 *      from.
 *    
 *    theAEKeyword:
 *      A keyword that specifies the desired Apple event parameter.
 *    
 *    desiredType:
 *      The descriptor type for the desired Apple event parameter. If
 *      the requested Apple event parameter is not of the desired type,
 *      the Apple Event Manager attempts to coerce it to the desired
 *      type. However, if you pass a value of typeWildCard, no coercion
 *      is performed, and the descriptor type of the returned
 *      descriptor is the same as the descriptor type of the Apple
 *      event parameter.
 *    
 *    result:
 *      A pointer to a descriptor. On successful return, a copy of the
 *      descriptor for the specified Apple event parameter, coerced, if
 *      necessary, to the descriptor type specified by the desiredType
 *      parameter. On error, a null descriptor. If the function returns
 *      successfully, your application should call the AEDisposeDesc
 *      function to dispose of the resulting descriptor after it has
 *      finished using it. See AEDesc.
=======
 *    theAEDescList:
 *      A pointer to the descriptor list to add a descriptor to. See
 *      AEDescList.
 *    
 *    index:
 *      A one-based positive integer indicating the position in the
 *      descriptor list of the descriptor to delete. AEDeleteItem
 *      returns an error if you pass zero, a negative number, or a
 *      value that is out of range. Currently the upper limit on index
 *      is 2^31 items.
>>>>>>> origin/fixes_2.4
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function AEGetParamDesc( const (*var*) theAppleEvent: AppleEvent; theAEKeyword: AEKeyword; desiredType: DescType; var result: AEDesc ): OSErr; external name '_AEGetParamDesc';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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
function AEDeleteItem( var theAEDescList: AEDescList; index: SIGNEDLONG ): OSErr; external name '_AEDeleteItem';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{*************************************************************************
 The following calls apply to AERecord. Since AERecord is a subtype of
 AEDescList, the calls in the previous sections can also be used for
 AERecord an AERecord can be created by using AECreateList with isRecord
 set to true. 
*************************************************************************}
{************************************************************************
 AERecords can have an abitrary descriptorType.  This allows you to
 check if desc is truly an AERecord
***********************************************************************}
>>>>>>> origin/fixes_2.4
{
 *  AESizeOfParam()
 *  
 *  Discussion:
 *    Gets the size and descriptor type of an Apple event parameter
 *    from a descriptor of type AERecord or AppleEvent.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAppleEvent:
 *      A pointer to the Apple event to get the parameter data from.
 *      See AppleEvent.
 *    
 *    theAEKeyword:
 *      The keyword that specifies the desired parameter.
 *    
 *    typeCode:
 *      A pointer to a descriptor type. On return, specifies the
 *      descriptor type of the Apple event parameter.
 *    
 *    dataSize:
 *      A pointer to a size variable. On return, the length, in bytes,
 *      of the data in the Apple event parameter.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
<<<<<<< HEAD
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AESizeOfParam( const (*var*) theAppleEvent: AppleEvent; theAEKeyword: AEKeyword; typeCode: DescTypePtr { can be NULL }; dataSize: SizePtr { can be NULL } ): OSErr; external name '_AESizeOfParam';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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


{
 *  AEDeleteParam()
 *  
 *  Discussion:
 *    Deletes a keyword-specified parameter from an Apple event or
 *    Apple event record.
=======
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function AEPutKeyPtr(var theAERecord: AERecord; theAEKeyword: AEKeyword; typeCode: DescType; dataPtr: UnivPtr; dataSize: Size): OSErr; external name '_AEPutKeyPtr';
{
 *  AEPutKeyDesc()
 *  
 *  Availability:
 *    Non-Carbon CFM:   not available
 *    CarbonLib:        not available
 *    Mac OS X:         not available
 }
function AEPutKeyDesc(var theAERecord: AERecord; theAEKeyword: AEKeyword; const (*var*) theAEDesc: AEDesc): OSErr; external name '_AEPutKeyDesc';
{
 *  AEGetKeyPtr()
 *  
 *  Availability:
=======
 *    CarbonLib:        in CarbonLib 1.4 and later
>>>>>>> origin/fixes_2.4
 *    Non-Carbon CFM:   not available
 }
function AECheckIsRecord( const (*var*) theDesc: AEDesc ): Boolean; external name '_AECheckIsRecord';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{*************************************************************************
  The following calls create and manipulate the AppleEvent data type.
*************************************************************************}
{
 *  AECreateAppleEvent()
>>>>>>> origin/fixes_2_2
 *  
 *  Summary:
 *    Creates an Apple event with several important attributes but no
 *    parameters.
 *  
 *  Discussion:
 *    The AECreateAppleEvent function creates an empty Apple event. You
 *    can add parameters to the Apple event after you create it with
 *    the functions described in "Adding Parameters and Attributes to
 *    an Apple Event".
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
<<<<<<< HEAD
<<<<<<< HEAD
 *  Parameters:
 *    
 *    theAppleEvent:
 *      A pointer to the Apple event or Apple event record to delete
 *      the parameter from.
 *    
 *    theAEKeyword:
 *      The keyword that specifies the parameter to delete.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AEDeleteParam( var theAppleEvent: AppleEvent; theAEKeyword: AEKeyword ): OSErr; external name '_AEDeleteParam';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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
=======
=======
 *  Parameters:
 *    
 *    theAEEventClass:
 *      The event class of the Apple event to create. This parameter
 *      becomes accessible through the keyEventClassAttr attribute of
 *      the Apple event. Some event classes are described in "Event
 *      Class Constants".
 *    
 *    theAEEventID:
 *      The event ID of the Apple event to create. This parameter
 *      becomes accessible through the keyEventIDAttr attribute of the
 *      Apple event.
 *    
 *    target:
 *      A pointer to an address descriptor. Before calling
 *      AECreateAppleEvent, you set the descriptor to identify the
 *      target (or server) application for the Apple event. This
 *      parameter becomes accessible through the keyAddressAttr
 *      attribute of the Apple event.
 *    
 *    returnID:
 *      The return ID for the created Apple event. If you pass a value
 *      of kAutoGenerateReturnID, the Apple Event Manager assigns the
 *      created Apple event a return ID that is unique to the current
 *      session. If you pass any other value, the Apple Event Manager
 *      assigns that value for the ID. This parameter becomes
 *      accessible through the keyReturnIDAttr attribute of the Apple
 *      event. The return ID constant is described in "ID Constants for
 *      the AECreateAppleEvent Function".
 *    
 *    transactionID:
 *      The transaction ID for this Apple event. A transaction is a
 *      sequence of Apple events that are sent back and forth between
 *      the client and server applications, beginning with the client's
 *      initial request for a service. All Apple events that are part
 *      of a transaction must have the same transaction ID. You can
 *      specify the kAnyTransactionID constant if the Apple event is
 *      not one of a series of interdependent Apple events. This
 *      parameter becomes accessible through the keyTransactionIDAttr
 *      attribute of the Apple event.
 *    
 *    result:
 *      A pointer to an Apple event. On successful return, the new
 *      Apple event. On error, a null descriptor (one with descriptor
 *      type typeNull). If the function returns successfully, your
 *      application should call the AEDisposeDesc function to dispose
 *      of the resulting Apple event after it has finished using it.
 *      See the AppleEvent data type.
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AECreateAppleEvent( theAEEventClass: AEEventClass; theAEEventID: AEEventID; const (*var*) target: AEAddressDesc; returnID: AEReturnID; transactionID: AETransactionID; var result: AppleEvent ): OSErr; external name '_AECreateAppleEvent';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{*************************************************************************
  The following calls are used to pack and unpack parameters from records
  of type AppleEvent. Since AppleEvent is a subtype of AERecord, the calls
  in the previous sections can also be used for variables of type
  AppleEvent. The next six calls are in fact identical to the six calls
  for AERecord.
*************************************************************************}
{
 *  AEPutParamPtr()
 *  
<<<<<<< HEAD
>>>>>>> origin/fixes_2_2
=======
 *  Discussion:
 *    Puts a pointer to data, a descriptor type, and a keyword into an
 *    Apple event or Apple event record as an Apple event parameter.
 *  
>>>>>>> origin/fixes_2.4
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAppleEvent:
 *      A pointer to the Apple event to add a parameter to. See the
 *      AppleEvent data type.
 *    
 *    theAEKeyword:
 *      The keyword for the parameter to add. If the Apple event
 *      already includes an parameter with this keyword, the parameter
 *      is replaced.
 *    
 *    typeCode:
 *      The descriptor type for the parameter to add.
 *    
 *    dataPtr:
 *      A pointer to the data for the parameter to add.
 *    
 *    dataSize:
 *      The length, in bytes, of the data for the parameter to add.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AEPutParamPtr( var theAppleEvent: AppleEvent; theAEKeyword: AEKeyword; typeCode: DescType; dataPtr: {const} UnivPtr; dataSize: Size ): OSErr; external name '_AEPutParamPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  AEPutParamDesc()
 *  
 *  Discussion:
 *    Puts a descriptor and a keyword into an Apple event or Apple
 *    event record as an Apple event parameter.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAppleEvent:
 *      A pointer to the Apple event to add a parameter to.
 *    
 *    theAEKeyword:
 *      The keyword specifying the parameter to add. If the Apple event
 *      already has a parameter with this keyword, the parameter is
 *      replaced.
 *    
 *    theAEDesc:
 *      A pointer to the descriptor for the parameter to add. See
 *      AEDesc.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AEPutParamDesc( var theAppleEvent: AppleEvent; theAEKeyword: AEKeyword; const (*var*) theAEDesc: AEDesc ): OSErr; external name '_AEPutParamDesc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  AEGetParamPtr()
 *  
 *  Summary:
 *    Gets a copy of the data for a specified Apple event parameter
 *    from an Apple event or an Apple event record (type AERecord);
 *    typically used when your application needs to work with the
 *    extracted data directly.
 *  
 *  Discussion:
 *    You should use this function only to extract data from value
 *    descriptors such as typeUTF8Text.
 *    Because this function allows you to specify a desired type, it
 *    can result in coercion. When used correctly, this has the
 *    positive effect of returning the data in the desired format.
 *    However, it can have side effects you may not be expecting, such
 *    as the overhead of calls to coercion handlers. See also the
 *    Version Notes section below for possible problems with
 *    coercion.
 *    To get Apple event parameter data for your application to use
 *    directly, call AEGetParamPtr. To get a descriptor for an Apple
 *    event parameter to pass on to another Apple Event Manager
 *    routine, call AEGetParamDesc.
 *    Before calling AEGetParamPtr, you can call the AESizeOfParam
 *    function to determine a size for the dataPtr buffer. However,
 *    unless you specify typeWildCard for the desiredType parameter,
 *    AEGetParamPtr may coerce the data, which may cause the size of
 *    the data to change.
 *    In some cases, you may get improved efficiency extracting
 *    information from an Apple event with the AEGetDescDataRange
 *    function.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAppleEvent:
 *      A pointer to the Apple event to get the parameter data from.
 *      See AppleEvent.
 *    
 *    theAEKeyword:
 *      The keyword that specifies the desired Apple event parameter.
 *    
 *    desiredType:
 *      The desired descriptor type for the copied data. If the
 *      descriptor specified by the theAEKeyword parameter is not of
 *      the desired type, AEGetParamPtr attempts to coerce the data to
 *      this type. However, if the desired type is typeWildCard, no
 *      coercion is performed. On return, you can determine the actual
 *      descriptor type by examining the typeCode parameter.
 *    
 *    actualType:
 *      A pointer to a descriptor type. On return, specifies the
 *      descriptor type of the data pointed to by dataPtr. The returned
 *      type is either the same as the type specified by the
 *      desiredType parameter or, if the desired type was typeWildcard,
 *      the true type of the descriptor. Specify NULL if you do not
 *      care about this return value.
 *    
 *    dataPtr:
 *      A pointer to a buffer, local variable, or other storage
 *      location created and disposed of by your application. The size
 *      in bytes must be at least as large as the value you pass in the
 *      maximumSize parameter. On return, contains the parameter data.
 *      Specify NULL if you do not care about this return value.
 *    
 *    maximumSize:
 *      The maximum length, in bytes, of the expected Apple event
 *      parameter data. The AEGetParamPtr function will not return more
 *      data than you specify in this parameter.
 *    
 *    actualSize:
 *      A pointer to a variable of type Size. On return, the length, in
 *      bytes, of the data for the specified Apple event parameter. If
 *      this value is larger than the value you passed in the
 *      maximumSize parameter, the buffer pointed to by dataPtr was not
 *      large enough to contain all of the data for the parameter,
 *      though AEGetParamPtr does not write beyond the end of the
 *      buffer. If the buffer was too small, you can resize it and call
 *      AEGetParamPtr again. Specify NULL if you do not care about this
 *      return value.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AEGetParamPtr( const (*var*) theAppleEvent: AppleEvent; theAEKeyword: AEKeyword; desiredType: DescType; actualType: DescTypePtr { can be NULL }; dataPtr: UnivPtr; maximumSize: Size; actualSize: SizePtr { can be NULL } ): OSErr; external name '_AEGetParamPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  AEGetParamDesc()
 *  
 *  Summary:
 *    Gets a copy of the descriptor for a keyword-specified Apple event
 *    parameter from an Apple event or an Apple event record (type
 *    AERecord); typically used when your application needs to pass the
 *    extracted data to another function as a descriptor.
 *  
 *  Discussion:
 *    To get Apple event parameter data for your application to use
 *    directly, call AEGetParamPtr. To get a descriptor for an Apple
 *    event parameter to pass on to another Apple Event Manager
 *    routine, call AEGetParamDesc.
 *    If the actual parameter you are getting with AEGetParamDesc is a
 *    record, AEGetParamDesc will only allow you to request it as a
 *    typeAERecord, typeAEList, or typeWildcard. For any other type, it
 *    will return errAECoercionFail.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAppleEvent:
 *      A pointer to the Apple event to get the parameter descriptor
 *      from.
 *    
 *    theAEKeyword:
 *      A keyword that specifies the desired Apple event parameter.
 *    
 *    desiredType:
 *      The descriptor type for the desired Apple event parameter. If
 *      the requested Apple event parameter is not of the desired type,
 *      the Apple Event Manager attempts to coerce it to the desired
 *      type. However, if you pass a value of typeWildCard, no coercion
 *      is performed, and the descriptor type of the returned
 *      descriptor is the same as the descriptor type of the Apple
 *      event parameter.
 *    
 *    result:
 *      A pointer to a descriptor. On successful return, a copy of the
 *      descriptor for the specified Apple event parameter, coerced, if
 *      necessary, to the descriptor type specified by the desiredType
 *      parameter. On error, a null descriptor. If the function returns
 *      successfully, your application should call the AEDisposeDesc
 *      function to dispose of the resulting descriptor after it has
 *      finished using it. See AEDesc.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AEGetParamDesc( const (*var*) theAppleEvent: AppleEvent; theAEKeyword: AEKeyword; desiredType: DescType; var result: AEDesc ): OSErr; external name '_AEGetParamDesc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  AESizeOfParam()
 *  
 *  Discussion:
 *    Gets the size and descriptor type of an Apple event parameter
 *    from a descriptor of type AERecord or AppleEvent.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAppleEvent:
 *      A pointer to the Apple event to get the parameter data from.
 *      See AppleEvent.
 *    
 *    theAEKeyword:
 *      The keyword that specifies the desired parameter.
 *    
 *    typeCode:
 *      A pointer to a descriptor type. On return, specifies the
 *      descriptor type of the Apple event parameter.
 *    
 *    dataSize:
 *      A pointer to a size variable. On return, the length, in bytes,
 *      of the data in the Apple event parameter.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AESizeOfParam( const (*var*) theAppleEvent: AppleEvent; theAEKeyword: AEKeyword; typeCode: DescTypePtr { can be NULL }; dataSize: SizePtr { can be NULL } ): OSErr; external name '_AESizeOfParam';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  AEDeleteParam()
 *  
 *  Discussion:
 *    Deletes a keyword-specified parameter from an Apple event or
 *    Apple event record.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAppleEvent:
 *      A pointer to the Apple event or Apple event record to delete
 *      the parameter from.
 *    
 *    theAEKeyword:
 *      The keyword that specifies the parameter to delete.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function AEDeleteParam(var theAppleEvent: AppleEvent; theAEKeyword: AEKeyword): OSErr; external name '_AEDeleteParam';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function AEDeleteParam( var theAppleEvent: AppleEvent; theAEKeyword: AEKeyword ): OSErr; external name '_AEDeleteParam';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{*************************************************************************
 The following calls also apply to type AppleEvent. Message attributes are
 far more restricted, and can only be accessed through the following 5
 calls. The various list and record routines cannot be used to access the
 attributes of an event. 
*************************************************************************}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{
 *  AEGetAttributePtr()
 *  
 *  Summary:
 *    Gets a copy of the data for a specified Apple event attribute
 *    from an Apple event; typically used when your application needs
 *    to work with the data directly.
 *  
 *  Discussion:
 *    To get Apple event attribute data for your application to use
 *    directly, call AEGetAttributePtr. To get a descriptor for an
 *    Apple event attribute to pass on to another Apple Event Manager
 *    routine, call AEGetAttributeDesc.
 *    Before calling AEGetAttributePtr, you can call the
 *    AESizeOfAttribute function to determine a size for the dataPtr
 *    buffer. However, unless you specify typeWildCard for the
 *    desiredType parameter, AEGetAttributePtr may coerce the data,
 *    which may cause the size of the data to change.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAppleEvent:
 *      A pointer to the Apple event to get the attribute data from.
 *    
 *    theAEKeyword:
 *      The keyword that specifies the desired attribute.
 *    
 *    desiredType:
 *      The desired descriptor type for the copied data. If the
 *      descriptor specified by the theAEKeyword parameter is not of
 *      the desired type, AEGetAttributePtr attempts to coerce the data
 *      to this type. However, if you pass a value of typeWildCard, no
 *      coercion is performed, and the descriptor type of the returned
 *      data is the same as the descriptor type of the Apple event
 *      attribute. On return, you can determine the actual descriptor
 *      type by examining the typeCode parameter.
 *    
 *    typeCode:
 *      A pointer to a descriptor type. On return, specifies the
 *      descriptor type of the attribute data pointed to by dataPtr.
 *      The returned type is either the same as the type specified by
 *      the desiredType parameter or, if the desired type was type
 *      wildcard, the true type of the descriptor. For a list of
 *      AppleScript's predefined descriptor types, see "Descriptor Type
 *      Constants". See DescType.
 *    
 *    dataPtr:
 *      A pointer to a buffer, local variable, or other storage
 *      location, created and disposed of by your application. The size
 *      in bytes must be at least as large as the value you pass in the
 *      maximumSize parameter. On return, contains the attribute data.
 *    
 *    maximumSize:
 *      The maximum length, in bytes, of the expected attribute data.
 *      The AEGetAttributePtr function will not return more data than
 *      you specify in this parameter.
 *    
 *    actualSize:
 *      A pointer to a size variable. On return, the length, in bytes,
 *      of the data for the specified Apple event attribute. If this
 *      value is larger than the value you passed in the maximumSize
 *      parameter, the buffer pointed to by dataPtr was not large
 *      enough to contain all of the data for the attribute, though
 *      AEGetAttributePtr does not write beyond the end of the buffer.
 *      If the buffer was too small, you can resize it and call
 *      AEGetAttributePtr again.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AEGetAttributePtr( const (*var*) theAppleEvent: AppleEvent; theAEKeyword: AEKeyword; desiredType: DescType; typeCode: DescTypePtr { can be NULL }; dataPtr: UnivPtr; maximumSize: Size; actualSize: SizePtr { can be NULL } ): OSErr; external name '_AEGetAttributePtr';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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


{
 *  AEGetAttributeDesc()
 *  
 *  Summary:
 *    Gets a copy of the descriptor for a specified Apple event
 *    attribute from an Apple event; typically used when your
 *    application needs to pass the descriptor on to another function.
 *  
 *  Discussion:
 *    To get Apple event attribute data for your application to use
 *    directly, call AEGetAttributePtr. To get a descriptor for an
 *    Apple event attribute to pass on to another Apple Event Manager
 *    routine, call AEGetAttributeDesc.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAppleEvent:
 *      A pointer to the Apple event to get the attribute descriptor
 *      from. See AppleEvent.
 *    
 *    theAEKeyword:
 *      The keyword that specifies the desired attribute.
 *    
 *    desiredType:
 *      The desired descriptor type for the copied data. If the
 *      descriptor specified by the theAEKeyword parameter is not of
 *      the desired type, AEGetAttributePtr attempts to coerce the data
 *      to this type. However, if you pass a value of typeWildCard, no
 *      coercion is performed, and the descriptor type of the returned
 *      data is the same as the descriptor type of the Apple event
 *      attribute. On return, you can determine the actual descriptor
 *      type by examining the typeCode parameter.
 *    
 *    result:
 *      A pointer to a descriptor. On successful return, a copy of the
 *      specified Apple event attribute, coerced, if necessary, to the
 *      descriptor type specified in desiredType. On error, a null
 *      descriptor. If the function returns successfully, your
 *      application should call the AEDisposeDesc function to dispose
 *      of the resulting descriptor after it has finished using it. See
 *      AEDesc.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AEGetAttributeDesc( const (*var*) theAppleEvent: AppleEvent; theAEKeyword: AEKeyword; desiredType: DescType; var result: AEDesc ): OSErr; external name '_AEGetAttributeDesc';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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


{
 *  AESizeOfAttribute()
 *  
 *  Discussion:
 *    Gets the size and descriptor type of an Apple event attribute
 *    from a descriptor of type AppleEvent.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAppleEvent:
 *      A pointer to the Apple event to get the attribute data from.
 *    
 *    theAEKeyword:
 *      The keyword that specifies the attribute.
 *    
 *    typeCode:
 *      A pointer to a descriptor type. On return, specifies the
 *      descriptor type of the attribute. Can be NULL.
 *    
 *    dataSize:
 *      A pointer to a size variable. On return, the length, in bytes,
 *      of the data in the attribute. Can be NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AESizeOfAttribute( const (*var*) theAppleEvent: AppleEvent; theAEKeyword: AEKeyword; typeCode: DescTypePtr { can be NULL }; dataSize: SizePtr { can be NULL } ): OSErr; external name '_AESizeOfAttribute';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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


{
 *  AEPutAttributePtr()
 *  
 *  Discussion:
 *    Adds a pointer to data, a descriptor type, and a keyword to an
 *    Apple event as an attribute.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAppleEvent:
 *      A pointer to the Apple event to add an attribute to. See the
 *      AppleEvent data type.
 *    
 *    theAEKeyword:
 *      The keyword for the attribute to add. If the Apple event
 *      already includes an attribute with this keyword, the attribute
 *      is replaced.
 *    
 *    typeCode:
 *      The descriptor type for the attribute to add.
 *    
 *    dataPtr:
 *      A pointer to the data for the attribute to add.
 *    
 *    dataSize:
 *      The length, in bytes, of the data for the attribute to add.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AEPutAttributePtr( var theAppleEvent: AppleEvent; theAEKeyword: AEKeyword; typeCode: DescType; dataPtr: {const} UnivPtr; dataSize: Size ): OSErr; external name '_AEPutAttributePtr';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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


{
 *  AEPutAttributeDesc()
 *  
 *  Summary:
 *    Adds a descriptor and a keyword to an Apple event as an attribute.
 *  
 *  Discussion:
 *    The AEPutAttributeDesc function takes a descriptor and a keyword
 *    and adds them to an Apple event as an attribute. If the
 *    descriptor type required for the attribute is different from the
 *    descriptor type of the descriptor, the Apple Event Manager
 *    attempts to coerce the descriptor into the required type, with
 *    one exception: the Apple Event Manager does not attempt to coerce
 *    the data for an address attribute, thereby allowing applications
 *    to use their own address types.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAppleEvent:
 *      A pointer to the Apple event to add an attribute to. See the
 *      AppleEvent data type.
 *    
 *    theAEKeyword:
 *      The keyword for the attribute to add. If the Apple event
 *      already includes an attribute with this keyword, the attribute
 *      is replaced.
 *    
 *    theAEDesc:
 *      A pointer to the descriptor to assign to the attribute. The
 *      descriptor type of the specified descriptor should match the
 *      defined descriptor type for that attribute.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AEPutAttributeDesc( var theAppleEvent: AppleEvent; theAEKeyword: AEKeyword; const (*var*) theAEDesc: AEDesc ): OSErr; external name '_AEPutAttributeDesc';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
{
 *  AEGetAttributePtr()
 *  
 *  Summary:
 *    Gets a copy of the data for a specified Apple event attribute
 *    from an Apple event; typically used when your application needs
 *    to work with the data directly.
 *  
 *  Discussion:
 *    To get Apple event attribute data for your application to use
 *    directly, call AEGetAttributePtr. To get a descriptor for an
 *    Apple event attribute to pass on to another Apple Event Manager
 *    routine, call AEGetAttributeDesc.
 *    Before calling AEGetAttributePtr, you can call the
 *    AESizeOfAttribute function to determine a size for the dataPtr
 *    buffer. However, unless you specify typeWildCard for the
 *    desiredType parameter, AEGetAttributePtr may coerce the data,
 *    which may cause the size of the data to change.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAppleEvent:
 *      A pointer to the Apple event to get the attribute data from.
 *    
 *    theAEKeyword:
 *      The keyword that specifies the desired attribute.
 *    
 *    desiredType:
 *      The desired descriptor type for the copied data. If the
 *      descriptor specified by the theAEKeyword parameter is not of
 *      the desired type, AEGetAttributePtr attempts to coerce the data
 *      to this type. However, if you pass a value of typeWildCard, no
 *      coercion is performed, and the descriptor type of the returned
 *      data is the same as the descriptor type of the Apple event
 *      attribute. On return, you can determine the actual descriptor
 *      type by examining the typeCode parameter.
 *    
 *    typeCode:
 *      A pointer to a descriptor type. On return, specifies the
 *      descriptor type of the attribute data pointed to by dataPtr.
 *      The returned type is either the same as the type specified by
 *      the desiredType parameter or, if the desired type was type
 *      wildcard, the true type of the descriptor. For a list of
 *      AppleScript's predefined descriptor types, see "Descriptor Type
 *      Constants". See DescType.
 *    
 *    dataPtr:
 *      A pointer to a buffer, local variable, or other storage
 *      location, created and disposed of by your application. The size
 *      in bytes must be at least as large as the value you pass in the
 *      maximumSize parameter. On return, contains the attribute data.
 *    
 *    maximumSize:
 *      The maximum length, in bytes, of the expected attribute data.
 *      The AEGetAttributePtr function will not return more data than
 *      you specify in this parameter.
 *    
 *    actualSize:
 *      A pointer to a size variable. On return, the length, in bytes,
 *      of the data for the specified Apple event attribute. If this
 *      value is larger than the value you passed in the maximumSize
 *      parameter, the buffer pointed to by dataPtr was not large
 *      enough to contain all of the data for the attribute, though
 *      AEGetAttributePtr does not write beyond the end of the buffer.
 *      If the buffer was too small, you can resize it and call
 *      AEGetAttributePtr again.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AEGetAttributePtr( const (*var*) theAppleEvent: AppleEvent; theAEKeyword: AEKeyword; desiredType: DescType; typeCode: DescTypePtr { can be NULL }; dataPtr: UnivPtr; maximumSize: Size; actualSize: SizePtr { can be NULL } ): OSErr; external name '_AEGetAttributePtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  AEGetAttributeDesc()
 *  
 *  Summary:
 *    Gets a copy of the descriptor for a specified Apple event
 *    attribute from an Apple event; typically used when your
 *    application needs to pass the descriptor on to another function.
 *  
 *  Discussion:
 *    To get Apple event attribute data for your application to use
 *    directly, call AEGetAttributePtr. To get a descriptor for an
 *    Apple event attribute to pass on to another Apple Event Manager
 *    routine, call AEGetAttributeDesc.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAppleEvent:
 *      A pointer to the Apple event to get the attribute descriptor
 *      from. See AppleEvent.
 *    
 *    theAEKeyword:
 *      The keyword that specifies the desired attribute.
 *    
 *    desiredType:
 *      The desired descriptor type for the copied data. If the
 *      descriptor specified by the theAEKeyword parameter is not of
 *      the desired type, AEGetAttributePtr attempts to coerce the data
 *      to this type. However, if you pass a value of typeWildCard, no
 *      coercion is performed, and the descriptor type of the returned
 *      data is the same as the descriptor type of the Apple event
 *      attribute. On return, you can determine the actual descriptor
 *      type by examining the typeCode parameter.
 *    
 *    result:
 *      A pointer to a descriptor. On successful return, a copy of the
 *      specified Apple event attribute, coerced, if necessary, to the
 *      descriptor type specified in desiredType. On error, a null
 *      descriptor. If the function returns successfully, your
 *      application should call the AEDisposeDesc function to dispose
 *      of the resulting descriptor after it has finished using it. See
 *      AEDesc.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AEGetAttributeDesc( const (*var*) theAppleEvent: AppleEvent; theAEKeyword: AEKeyword; desiredType: DescType; var result: AEDesc ): OSErr; external name '_AEGetAttributeDesc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  AESizeOfAttribute()
 *  
 *  Discussion:
 *    Gets the size and descriptor type of an Apple event attribute
 *    from a descriptor of type AppleEvent.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAppleEvent:
 *      A pointer to the Apple event to get the attribute data from.
 *    
 *    theAEKeyword:
 *      The keyword that specifies the attribute.
 *    
 *    typeCode:
 *      A pointer to a descriptor type. On return, specifies the
 *      descriptor type of the attribute. Can be NULL.
 *    
 *    dataSize:
 *      A pointer to a size variable. On return, the length, in bytes,
 *      of the data in the attribute. Can be NULL.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AESizeOfAttribute( const (*var*) theAppleEvent: AppleEvent; theAEKeyword: AEKeyword; typeCode: DescTypePtr { can be NULL }; dataSize: SizePtr { can be NULL } ): OSErr; external name '_AESizeOfAttribute';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  AEPutAttributePtr()
 *  
 *  Discussion:
 *    Adds a pointer to data, a descriptor type, and a keyword to an
 *    Apple event as an attribute.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAppleEvent:
 *      A pointer to the Apple event to add an attribute to. See the
 *      AppleEvent data type.
 *    
 *    theAEKeyword:
 *      The keyword for the attribute to add. If the Apple event
 *      already includes an attribute with this keyword, the attribute
 *      is replaced.
 *    
 *    typeCode:
 *      The descriptor type for the attribute to add.
 *    
 *    dataPtr:
 *      A pointer to the data for the attribute to add.
 *    
 *    dataSize:
 *      The length, in bytes, of the data for the attribute to add.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function AEPutAttributePtr( var theAppleEvent: AppleEvent; theAEKeyword: AEKeyword; typeCode: DescType; dataPtr: {const} UnivPtr; dataSize: Size ): OSErr; external name '_AEPutAttributePtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  AEPutAttributeDesc()
 *  
 *  Summary:
 *    Adds a descriptor and a keyword to an Apple event as an attribute.
 *  
 *  Discussion:
 *    The AEPutAttributeDesc function takes a descriptor and a keyword
 *    and adds them to an Apple event as an attribute. If the
 *    descriptor type required for the attribute is different from the
 *    descriptor type of the descriptor, the Apple Event Manager
 *    attempts to coerce the descriptor into the required type, with
 *    one exception: the Apple Event Manager does not attempt to coerce
 *    the data for an address attribute, thereby allowing applications
 *    to use their own address types.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAppleEvent:
 *      A pointer to the Apple event to add an attribute to. See the
 *      AppleEvent data type.
 *    
 *    theAEKeyword:
 *      The keyword for the attribute to add. If the Apple event
 *      already includes an attribute with this keyword, the attribute
 *      is replaced.
 *    
 *    theAEDesc:
 *      A pointer to the descriptor to assign to the attribute. The
 *      descriptor type of the specified descriptor should match the
 *      defined descriptor type for that attribute.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
<<<<<<< HEAD
function AEPutAttributeDesc(var theAppleEvent: AppleEvent; theAEKeyword: AEKeyword; const (*var*) theAEDesc: AEDesc): OSErr; external name '_AEPutAttributeDesc';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function AEPutAttributeDesc( var theAppleEvent: AppleEvent; theAEKeyword: AEKeyword; const (*var*) theAEDesc: AEDesc ): OSErr; external name '_AEPutAttributeDesc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


>>>>>>> origin/fixes_2.4
{*************************************************************************
 AppleEvent Serialization Support

    AESizeOfFlattenedDesc, AEFlattenDesc, AEUnflattenDesc
    
    These calls will work for all AppleEvent data types and between different
    versions of the OS (including between Mac OS 9 and X)
    
    Basic types, AEDesc, AEList and AERecord are OK, but AppleEvent records
    themselves may not be reliably flattened for storage.
*************************************************************************}
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  AESizeOfFlattenedDesc()
 *  
 *  Discussion:
 *    Returns the amount of buffer space needed to flatten the AEDesc.
 *    Call this before AEFlattenDesc to make sure your buffer has
 *    enough room for the operation.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAEDesc:
 *      A pointer to the descriptor to be flattened.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.4 and later
 *    Non-Carbon CFM:   not available
 }
function AESizeOfFlattenedDesc( const (*var*) theAEDesc: AEDesc ): Size; external name '_AESizeOfFlattenedDesc';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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
=======
>>>>>>> origin/fixes_2_2
   AEFlattenDesc
   Returns the amount of buffer space needed to flatten the
   AEDesc. Call this before AEFlattenDesc to make sure your
   buffer has enough room for the operation.
}

{
=======
>>>>>>> origin/fixes_2.4
 *  AESizeOfFlattenedDesc()
 *  
 *  Discussion:
 *    Returns the amount of buffer space needed to flatten the AEDesc.
 *    Call this before AEFlattenDesc to make sure your buffer has
 *    enough room for the operation.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAEDesc:
 *      A pointer to the descriptor to be flattened.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.4 and later
 *    Non-Carbon CFM:   not available
 }
function AESizeOfFlattenedDesc( const (*var*) theAEDesc: AEDesc ): Size; external name '_AESizeOfFlattenedDesc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

<<<<<<< HEAD
{
   AEFlattenDesc
   Fills a buffer with a flattened representation of the
   AEDesc and returns the amount of buffer used in actualSize.
   If bufferSize was too small it returns errAEBufferTooSmall
   (-1741) and does not fill in any of the buffer. The resulting
   buffer is only useful with an AEUnflattenDesc call.
   
   Note: if you pass a NULL buffer pointer it returns noErr but
   fills in the actualSize field anyway.
}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{
 *  AEFlattenDesc()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  Discussion:
 *    Fills a buffer with a flattened representation of the AEDesc and
 *    returns the amount of buffer used in actualSize. If bufferSize
 *    was too small it returns errAEBufferTooSmall (-1741) and does not
 *    fill in any of the buffer. The resulting buffer is only useful
 *    with an AEUnflattenDesc call. 
 *    Note: if you pass a NULL buffer pointer it returns noErr but
 *    fills in the actualSize field anyway.
 *  
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAEDesc:
 *      A pointer to the descriptor to be flattened.
 *    
 *    buffer:
 *      A pointer to memory, allocated by the application, where the
 *      flattened data will be stored. See the bufferSize parameter for
 *      information on how large a buffer you should allocate.
 *    
 *    bufferSize:
 *      The size of the buffer pointed to by buffer. Prior to calling
 *      AEFlattenDesc, you call the AESizeOfFlattenedDesc function to
 *      determine the required size of the buffer for the flatten
 *      operation.
 *      If bufferSize is too small, AEFlattenDesc returns
 *      errAEBufferTooSmall and doesn't store any data in the buffer.
 *    
 *    actualSize:
 *      A pointer to a size variable. On return, the variable contains
 *      the actual size of the flattened data. You can specify NULL for
 *      this parameter if you do not care about the returned size.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.4 and later
 *    Non-Carbon CFM:   not available
 }
function AEFlattenDesc( const (*var*) theAEDesc: AEDesc; buffer: Ptr; bufferSize: Size; var actualSize: Size ): OSStatus; external name '_AEFlattenDesc';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAEDesc:
 *      A pointer to the descriptor to be flattened.
 *    
 *    buffer:
 *      A pointer to memory, allocated by the application, where the
 *      flattened data will be stored. See the bufferSize parameter for
 *      information on how large a buffer you should allocate.
 *    
 *    bufferSize:
 *      The size of the buffer pointed to by buffer. Prior to calling
 *      AEFlattenDesc, you call the AESizeOfFlattenedDesc function to
 *      determine the required size of the buffer for the flatten
 *      operation.
 *      If bufferSize is too small, AEFlattenDesc returns
 *      errAEBufferTooSmall and doesn't store any data in the buffer.
 *    
 *    actualSize:
 *      A pointer to a size variable. On return, the variable contains
 *      the actual size of the flattened data. You can specify NULL for
 *      this parameter if you do not care about the returned size.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.4 and later
 *    Non-Carbon CFM:   not available
 }
function AEFlattenDesc( const (*var*) theAEDesc: AEDesc; buffer: Ptr; bufferSize: Size; var actualSize: Size ): OSStatus; external name '_AEFlattenDesc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

<<<<<<< HEAD
{
   AEUnflattenDesc
   Allocates an AEDesc (given a Null Desc) given a flattened
   data buffer. It assumes it was given a good buffer filled
   in by AEFlattenDesc. It returns paramErr if it discovers
   something fishy about the buffer.
}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{
 *  AEUnflattenDesc()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  Discussion:
 *    Allocates an AEDesc (given a Null Desc) given a flattened data
 *    buffer. It assumes it was given a good buffer filled in by
 *    AEFlattenDesc. It returns paramErr if it discovers something
 *    fishy about the buffer.
 *  
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    buffer:
 *      A pointer to memory, allocated by the application, that
 *      contains flattened data produced by a previous call to
 *      AEFlattenDesc.
 *    
 *    result:
 *      A null descriptor. On successful completion, points to a
 *      descriptor created from the flattened data. The caller is
 *      responsible for disposing of the descriptor.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.4 and later
 *    Non-Carbon CFM:   not available
 }
function AEUnflattenDesc( buffer: {const} UnivPtr; var result: AEDesc ): OSStatus; external name '_AEUnflattenDesc';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    buffer:
 *      A pointer to memory, allocated by the application, that
 *      contains flattened data produced by a previous call to
 *      AEFlattenDesc.
 *    
 *    result:
 *      A null descriptor. On successful completion, points to a
 *      descriptor created from the flattened data. The caller is
 *      responsible for disposing of the descriptor.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.4 and later
 *    Non-Carbon CFM:   not available
 }
<<<<<<< HEAD
function AEUnflattenDesc(buffer: Ptr; var result: AEDesc): OSStatus; external name '_AEUnflattenDesc';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function AEUnflattenDesc( buffer: {const} UnivPtr; var result: AEDesc ): OSStatus; external name '_AEUnflattenDesc';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{*************************************************************************
 The following calls are necessary to deal with opaque data in AEDescs, because the
 traditional way of dealing with a basic AEDesc has been to dereference the dataHandle
 directly.  This is not supported under Carbon.
*************************************************************************}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
{$ifc ACCESSOR_CALLS_ARE_FUNCTIONS}
>>>>>>> graemeg/fixes_2_2
=======
{$ifc ACCESSOR_CALLS_ARE_FUNCTIONS}
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{
        AEGetDescData no longer supports automatic coercion. If you'd like to
        coerce the descriptor use AECoerceDesc.
    }
{
 *  AEGetDescData()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  Discussion:
 *    Gets the data from the specified descriptor.  AEGetDescData no
 *    longer supports automatic coercion. If you'd like to coerce the
 *    descriptor use AECoerceDesc.
 *  
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAEDesc:
 *      A pointer to the descriptor to get the data from.
 *    
 *    dataPtr:
 *      A pointer to a buffer, local variable, or other storage
 *      location created and disposed of by your application. The size
 *      in bytes should be the same as the value you pass in the
 *      maximumSize parameter. On return, contains the data from the
 *      descriptor.
 *    
 *    maximumSize:
 *      The length, in bytes, of the expected descriptor data. The
 *      AEGetDescData function will not return more data than you
 *      specify in this parameter. You typically determine the maximum
 *      size by calling AEGetDescDataSize.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function AEGetDescData( const (*var*) theAEDesc: AEDesc; dataPtr: UnivPtr; maximumSize: Size ): OSErr; external name '_AEGetDescData';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAEDesc:
 *      A pointer to the descriptor to get the data from.
 *    
 *    dataPtr:
 *      A pointer to a buffer, local variable, or other storage
 *      location created and disposed of by your application. The size
 *      in bytes should be the same as the value you pass in the
 *      maximumSize parameter. On return, contains the data from the
 *      descriptor.
 *    
 *    maximumSize:
 *      The length, in bytes, of the expected descriptor data. The
 *      AEGetDescData function will not return more data than you
 *      specify in this parameter. You typically determine the maximum
 *      size by calling AEGetDescDataSize.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
<<<<<<< HEAD
function AEGetDescData(const (*var*) theAEDesc: AEDesc; dataPtr: UnivPtr; maximumSize: Size): OSErr; external name '_AEGetDescData';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function AEGetDescData( const (*var*) theAEDesc: AEDesc; dataPtr: UnivPtr; maximumSize: Size ): OSErr; external name '_AEGetDescData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  AEGetDescDataSize()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *  Discussion:
 *    Gets the size, in bytes, of the data in the specified descriptor.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAEDesc:
 *      A pointer to the descriptor to obtain the data size for. See
 *      AEDesc.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function AEGetDescDataSize( const (*var*) theAEDesc: AEDesc ): Size; external name '_AEGetDescDataSize';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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
=======
>>>>>>> origin/fixes_2_2
=======
 *  Discussion:
 *    Gets the size, in bytes, of the data in the specified descriptor.
 *  
>>>>>>> origin/fixes_2.4
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    theAEDesc:
 *      A pointer to the descriptor to obtain the data size for. See
 *      AEDesc.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
<<<<<<< HEAD
function AEGetDescDataSize(const (*var*) theAEDesc: AEDesc): Size; external name '_AEGetDescDataSize';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function AEGetDescDataSize( const (*var*) theAEDesc: AEDesc ): Size; external name '_AEGetDescDataSize';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

>>>>>>> origin/fixes_2.4

{
 *  AEReplaceDescData()
 *  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  Discussion:
 *    Copies the specified data into the specified descriptor,
 *    replacing any previous data.
 *  
<<<<<<< HEAD
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    typeCode:
 *      Specifies the descriptor type of the data pointed to by dataPtr.
 *    
 *    dataPtr:
 *      A pointer to the data to store in the specified descriptor.
 *    
 *    dataSize:
 *      The size, in bytes, of the data pointed to by the dataSize
 *      parameter.
 *    
 *    theAEDesc:
 *      A pointer to a descriptor. On return, contains the copied data.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function AEReplaceDescData( typeCode: DescType; dataPtr: {const} UnivPtr; dataSize: Size; var theAEDesc: AEDesc ): OSErr; external name '_AEReplaceDescData';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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


{
 *  AEGetDescDataRange()
 *  
 *  Discussion:
 *    Retrieve a range of bytes from an AEDesc.  This obviates the need
 *    to retrieve the entire data from the event using AEGetDescData.
 *    This is only valid for data type AEDescs.  If the requested
 *    length and offset are such that they do not fit entirely with the
 *    data of the desc, errAEBufferTooSmall is returned.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    dataDesc:
 *      A pointer to the descriptor to get the data from.
 *    
 *    buffer:
 *      A pointer to a buffer, local variable, or other storage
 *      location created and disposed of by your application. The size
 *      in bytes should be at least as large as the value you pass in
 *      the length parameter. On return, contains the specified data
 *      from the descriptor.
 *    
 *    offset:
 *      The zero-based offset to the data to be retrieved from the
 *      descriptor.
 *    
 *    length:
 *      The number of bytes of contiguous data to retrieve.
 *  
=======
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
 *  Parameters:
 *    
 *    typeCode:
 *      Specifies the descriptor type of the data pointed to by dataPtr.
 *    
 *    dataPtr:
 *      A pointer to the data to store in the specified descriptor.
 *    
 *    dataSize:
 *      The size, in bytes, of the data pointed to by the dataSize
 *      parameter.
 *    
 *    theAEDesc:
 *      A pointer to a descriptor. On return, contains the copied data.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in CarbonAccessors.o 1.0 and later
 }
function AEReplaceDescData( typeCode: DescType; dataPtr: {const} UnivPtr; dataSize: Size; var theAEDesc: AEDesc ): OSErr; external name '_AEReplaceDescData';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{
 *  AEGetDescDataRange()
 *  
 *  Discussion:
 *    Retrieve a range of bytes from an AEDesc.  This obviates the need
 *    to retrieve the entire data from the event using AEGetDescData.
 *    This is only valid for data type AEDescs.  If the requested
 *    length and offset are such that they do not fit entirely with the
 *    data of the desc, errAEBufferTooSmall is returned.
 *  
 *  Mac OS X threading:
 *    Thread safe since version 10.2
 *  
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  Parameters:
 *    
 *    dataDesc:
 *      A pointer to the descriptor to get the data from.
 *    
 *    buffer:
 *      A pointer to a buffer, local variable, or other storage
 *      location created and disposed of by your application. The size
 *      in bytes should be at least as large as the value you pass in
 *      the length parameter. On return, contains the specified data
 *      from the descriptor.
 *    
 *    offset:
 *      The zero-based offset to the data to be retrieved from the
 *      descriptor.
 *    
 *    length:
 *      The number of bytes of contiguous data to retrieve.
 *  
>>>>>>> origin/fixes_2.4
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function AEGetDescDataRange( const (*var*) dataDesc: AEDesc; buffer: UnivPtr; offset: Size; length: Size ): OSStatus; external name '_AEGetDescDataRange';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_2, __IPHONE_NA ) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> origin/fixes_2_2


{*************************************************************************
  A AEEventHandler is installed to process an AppleEvent 
*************************************************************************}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
	AEEventHandlerProcPtr = function( const (*var*) theAppleEvent: AppleEvent; var reply: AppleEvent; handlerRefcon: SRefCon ): OSErr;
	AEEventHandlerUPP = AEEventHandlerProcPtr;
=======
=======
>>>>>>> origin/fixes_2_2

type
{$ifc TYPED_FUNCTION_POINTERS}
	AEEventHandlerProcPtr = function(const (*var*) theAppleEvent: AppleEvent; var reply: AppleEvent; handlerRefcon: SInt32): OSErr;
{$elsec}
	AEEventHandlerProcPtr = ProcPtr;
{$endc}

{$ifc OPAQUE_UPP_TYPES}
	AEEventHandlerUPP = ^SInt32; { an opaque UPP }
{$elsec}
	AEEventHandlerUPP = UniversalProcPtr;
{$endc}	

const
	uppAEEventHandlerProcInfo = $00000FE0;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
type
	AEEventHandlerProcPtr = function( const (*var*) theAppleEvent: AppleEvent; var reply: AppleEvent; handlerRefcon: SRefCon ): OSErr;
	AEEventHandlerUPP = AEEventHandlerProcPtr;
>>>>>>> origin/fixes_2.4
{
 *  NewAEDisposeExternalUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewAEDisposeExternalUPP( userRoutine: AEDisposeExternalProcPtr ): AEDisposeExternalUPP; external name '_NewAEDisposeExternalUPP';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_2, __IPHONE_NA ) *)

{
 *  NewAEEventHandlerUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewAEEventHandlerUPP( userRoutine: AEEventHandlerProcPtr ): AEEventHandlerUPP; external name '_NewAEEventHandlerUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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
=======
>>>>>>> origin/fixes_2_2
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)

<<<<<<< HEAD
	{
	 *  NewAEEventHandlerUPP()
	 *  
	 *  Availability:
	 *    Non-Carbon CFM:   available as macro/inline
	 *    CarbonLib:        in CarbonLib 1.0 and later
	 *    Mac OS X:         in version 10.0 and later
	 	}
function NewAEEventHandlerUPP(userRoutine: AEEventHandlerProcPtr): AEEventHandlerUPP; external name '_NewAEEventHandlerUPP'; { old name was NewAEEventHandlerProc }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{
 *  NewAEEventHandlerUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewAEEventHandlerUPP( userRoutine: AEEventHandlerProcPtr ): AEEventHandlerUPP; external name '_NewAEEventHandlerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4

{
 *  DisposeAEDisposeExternalUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeAEDisposeExternalUPP( userUPP: AEDisposeExternalUPP ); external name '_DisposeAEDisposeExternalUPP';
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_2, __IPHONE_NA ) *)
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> origin/fixes_2_2

{
 *  DisposeAEEventHandlerUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeAEEventHandlerUPP( userUPP: AEEventHandlerUPP ); external name '_DisposeAEEventHandlerUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
procedure DisposeAEEventHandlerUPP(userUPP: AEEventHandlerUPP); external name '_DisposeAEEventHandlerUPP';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
procedure DisposeAEEventHandlerUPP( userUPP: AEEventHandlerUPP ); external name '_DisposeAEEventHandlerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)
>>>>>>> origin/fixes_2.4

{
 *  InvokeAEDisposeExternalUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure InvokeAEDisposeExternalUPP( dataPtr: {const} UnivPtr; dataLength: Size; refcon: SRefCon; userUPP: AEDisposeExternalUPP ); external name '_InvokeAEDisposeExternalUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_2, __IPHONE_NA ) *)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/cpstrnew
=======
procedure InvokeAEDisposeExternalUPP( dataPtr: {const} UnivPtr; dataLength: Size; refcon: SInt32; userUPP: AEDisposeExternalUPP ); external name '_InvokeAEDisposeExternalUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> graemeg/fixes_2_2
=======
procedure InvokeAEDisposeExternalUPP( dataPtr: {const} UnivPtr; dataLength: Size; refcon: SInt32; userUPP: AEDisposeExternalUPP ); external name '_InvokeAEDisposeExternalUPP';
=======
procedure InvokeAEDisposeExternalUPP( dataPtr: {const} UnivPtr; dataLength: Size; refcon: SRefCon; userUPP: AEDisposeExternalUPP ); external name '_InvokeAEDisposeExternalUPP';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
>>>>>>> origin/fixes_2_2

{
 *  InvokeAEEventHandlerUPP()
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
 function InvokeAEEventHandlerUPP( const (*var*) theAppleEvent: AppleEvent; var reply: AppleEvent; handlerRefcon: SRefCon; userUPP: AEEventHandlerUPP ): OSErr; external name '_InvokeAEEventHandlerUPP';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* __OSX_AVAILABLE_STARTING( __MAC_10_0, __IPHONE_NA ) *)
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

{$endc} {TARGET_OS_MAC}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Non-Carbon CFM:   available as macro/inline
=======
 *    Mac OS X:         in version 10.0 and later in ApplicationServices.framework
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
 function InvokeAEEventHandlerUPP( const (*var*) theAppleEvent: AppleEvent; var reply: AppleEvent; handlerRefcon: SRefCon; userUPP: AEEventHandlerUPP ): OSErr; external name '_InvokeAEEventHandlerUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

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
