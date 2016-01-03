{
     File:       QD/ATSUnicodeDirectAccess.h
 
     Contains:   Public Interfaces/Types for Low Level ATSUI
 
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
 
     Copyright:  © 2002-2008 by Apple Inc. all rights reserved.
=======
     Version:    Quickdraw-150~1
 
     Copyright:  © 2002-2003 by Apple Computer, Inc., all rights reserved.
>>>>>>> graemeg/fixes_2_2
=======
     Version:    Quickdraw-150~1
 
     Copyright:  © 2002-2003 by Apple Computer, Inc., all rights reserved.
>>>>>>> origin/fixes_2_2
=======
     Version:    Quickdraw-262~1
 
     Copyright:  © 2002-2008 by Apple Inc. all rights reserved.
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

unit ATSUnicodeDirectAccess;
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
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
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
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
	{$setc TARGET_OS_EMBEDDED := TRUE}
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ nor __arm64__ is defined.}
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
=======
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
>>>>>>> graemeg/cpstrnew
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
>>>>>>> graemeg/cpstrnew
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
>>>>>>> origin/cpstrnew
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
uses MacTypes,ATSLayoutTypes,ATSUnicodeTypes,TextCommon;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}

<<<<<<< HEAD
=======
{$ALIGN MAC68K}
>>>>>>> graemeg/fixes_2_2
=======
{$ALIGN MAC68K}
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{ ---------------------------------------------------------------------------- }
{ Constants                                                                    }
{ ---------------------------------------------------------------------------- }

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
 *  ATSUDirectDataSelector
 *  
 *  Summary:
 *    These are the data selectors used in the
 *    ATSUDirectGetLayoutDataArrayPtr function to get the needed layout
 *    data array pointer.
 }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
	ATSUDirectDataSelector = UInt32;
const
{
=======
=======
>>>>>>> origin/fixes_2_2
type ATSUDirectDataSelector = UInt32;
const

  {
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
type
	ATSUDirectDataSelector = UInt32;
const
{
>>>>>>> origin/fixes_2.4
   * Returns the parallel advance delta (delta X) array. (Array Type):
   * Fixed (Return Time): Constant, unless creation is necessary, or
   * unless requested by ATSUDirectGetLayoutDataArrayPtrFromTextLayout.
   * (Creation): This array is created only on demand. Thus, if any
   * changes are to be made iCreate should be set to true. If the array
   * had not been previously allocated it will be allocated and
   * zero-filled when iCreate is set to true.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUDirectDataAdvanceDeltaFixedArray = 0;
=======
  kATSUDirectDataAdvanceDeltaFixedArray = 0;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUDirectDataAdvanceDeltaFixedArray = 0;
>>>>>>> origin/fixes_2_2
=======
	kATSUDirectDataAdvanceDeltaFixedArray = 0;
>>>>>>> origin/fixes_2.4

  {
   * Returns the parallel baseline delta (delta Y) array. (Array Type):
   * Fixed (Return Time): Constant, unless creation is necessary, or
   * unless requested by ATSUDirectGetLayoutDataArrayPtrFromTextLayout.
   * (Creation): This array is created only on demand. Thus, if any
   * changes are to be made iCreate should be set to true. If the array
   * had not been previously allocated it will be allocated and
   * zero-filled when iCreate is set to true.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUDirectDataBaselineDeltaFixedArray = 1;
=======
  kATSUDirectDataBaselineDeltaFixedArray = 1;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUDirectDataBaselineDeltaFixedArray = 1;
>>>>>>> origin/fixes_2_2
=======
	kATSUDirectDataBaselineDeltaFixedArray = 1;
>>>>>>> origin/fixes_2.4

  {
   * Returns the parallel device delta array for device- specific
   * tweaking. This is an array of values which are used to adjust
   * truncated fractional values for devices that do not accept
   * fractional positioning. It is also used to provide precise
   * positioning for connected scripts. (Array Type): SInt16 (Return
   * Time): Constant, unless creation is necessary, or unless requested
   * by ATSUDirectGetLayoutDataArrayPtrFromTextLayout. (Creation): This
   * array is created only on demand. Thus, if any changes are to be
   * made iCreate should be set to true. If the array had not been
   * previously allocated it will be allocated and zero-filled when
   * iCreate is set to true.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUDirectDataDeviceDeltaSInt16Array = 2;
=======
  kATSUDirectDataDeviceDeltaSInt16Array = 2;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUDirectDataDeviceDeltaSInt16Array = 2;
>>>>>>> origin/fixes_2_2
=======
	kATSUDirectDataDeviceDeltaSInt16Array = 2;
>>>>>>> origin/fixes_2.4

  {
   * Returns the parallel style index array. The indexes setting in the
   * array are indexes into the the StyleSetting array, which can be
   * obtained using the
   * kATSUDirectDataStyleSettingATSUStyleSettingRefArray below. (Array
   * Type): UInt16 (Return Time): Constant, unless creation is
   * necessary, or unless requested by
   * ATSUDirectGetLayoutDataArrayPtrFromTextLayout. (Creation): This
   * array is created only on demand. Thus, if any changes are to be
   * made iCreate should be set to true. If the array had not been
   * previously allocated it will be allocated and zero-filled when
   * iCreate is set to true.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUDirectDataStyleIndexUInt16Array = 3;
=======
  kATSUDirectDataStyleIndexUInt16Array = 3;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUDirectDataStyleIndexUInt16Array = 3;
>>>>>>> origin/fixes_2_2
=======
	kATSUDirectDataStyleIndexUInt16Array = 3;
>>>>>>> origin/fixes_2.4

  {
   * Returns the style setting ref array. (Array Type):
   * ATSUStyleSettingRef (Return Time): Linear, based on the number of
   * styles applied to the given line. (Creation): This array is always
   * present if the layout has any text assigned to it at all. Setting
   * iCreate has no effect.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUDirectDataStyleSettingATSUStyleSettingRefArray = 4;
=======
  kATSUDirectDataStyleSettingATSUStyleSettingRefArray = 4;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUDirectDataStyleSettingATSUStyleSettingRefArray = 4;
>>>>>>> origin/fixes_2_2
=======
	kATSUDirectDataStyleSettingATSUStyleSettingRefArray = 4;
>>>>>>> origin/fixes_2.4

  {
   * Returns the ATSLayoutRecord, version 1 array. This should not be
   * used directly at all. Rather, use the
   * kATSUDirectDataLayoutRecordATSLayoutRecordCurrent selector below.
   * This will ensure that the code will always be using the most
   * current version of the ATSLayoutRecord, should there ever be a
   * change. ATSUI will only ensure the most efficient processing will
   * occur for the latest version of ATSLayoutRecord. (Array Type):
   * ATSLayoutRecord, version 1 (Return Time): Constant, unless
   * creation is necessary, or unless requested by
   * ATSUDirectGetLayoutDataArrayPtrFromTextLayout. (Creation): This
   * array is always present if the layout has any text assigned to it
   * at all. Setting iCreate has no effect
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUDirectDataLayoutRecordATSLayoutRecordVersion1 = 100;
=======
  kATSUDirectDataLayoutRecordATSLayoutRecordVersion1 = 100;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUDirectDataLayoutRecordATSLayoutRecordVersion1 = 100;
>>>>>>> origin/fixes_2_2
=======
	kATSUDirectDataLayoutRecordATSLayoutRecordVersion1 = 100;
>>>>>>> origin/fixes_2.4

  {
   * Returns the ATSLayoutRecord. This will return the most current
   * version of the ATSLayoutRecord, and the one that's defined in this
   * file. Always use kATSUDirectDataLayoutRecordATSLayoutRecordCurrent
   * to get the array of ATSLayoutRecords. (Array Type):
   * ATSLayoutRecord (Return Time): Constant, unless creation is
   * necessary, or unless requested by
   * ATSUDirectGetLayoutDataArrayPtrFromTextLayout. (Creation): This
   * array is always present if the layout has any text assigned to it
   * at all. Setting iCreate has no effect.
   }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	kATSUDirectDataLayoutRecordATSLayoutRecordCurrent = kATSUDirectDataLayoutRecordATSLayoutRecordVersion1;
=======
  kATSUDirectDataLayoutRecordATSLayoutRecordCurrent = kATSUDirectDataLayoutRecordATSLayoutRecordVersion1;
>>>>>>> graemeg/fixes_2_2
=======
  kATSUDirectDataLayoutRecordATSLayoutRecordCurrent = kATSUDirectDataLayoutRecordATSLayoutRecordVersion1;
>>>>>>> origin/fixes_2_2
=======
	kATSUDirectDataLayoutRecordATSLayoutRecordCurrent = kATSUDirectDataLayoutRecordATSLayoutRecordVersion1;
>>>>>>> origin/fixes_2.4

{ ---------------------------------------------------------------------------- }
{ Data Types                                                                   }
{ ---------------------------------------------------------------------------- }

{
 *  ATSUStyleSettingRef
 *  
 *  Summary:
 *    A reference to a style setting object that represents an
 *    ATSUStyle plus any cached/set information about that style.
 }
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	ATSUStyleSettingRef = ^LLCStyleInfo; { an opaque type }
	LLCStyleInfo = record end;
=======
	ATSUStyleSettingRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	ATSUStyleSettingRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	ATSUStyleSettingRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	ATSUStyleSettingRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew
=======
	ATSUStyleSettingRef    = ^SInt32; { an opaque 32-bit type }
>>>>>>> graemeg/fixes_2_2
=======
	ATSUStyleSettingRef    = ^SInt32; { an opaque 32-bit type }
>>>>>>> origin/fixes_2_2
=======
	ATSUStyleSettingRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4
{ ---------------------------------------------------------------------------- }
{ Direct Accessors                                                             }
{ ---------------------------------------------------------------------------- }
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
 *  ATSUDirectGetLayoutDataArrayPtrFromLineRef()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTRunGetGlyphsPtr or CTRunGetPositionsPtr instead.
<<<<<<< HEAD
=======
 *  ATSUDirectGetLayoutDataArrayPtrFromLineRef()
>>>>>>> graemeg/fixes_2_2
=======
 *  ATSUDirectGetLayoutDataArrayPtrFromLineRef()
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Returns the data pointer specified by iDataSelector and
 *    referenced by iLineRef.
 *  
 *  Discussion:
 *    This function simply returns the data pointer specified by
 *    iDataSelector and referenced by iLineRef. This data pointer
 *    should not be freed directly after it's been used. Rather, it
 *    should be released using ATSUDirectReleaseLayoutDataArrayPtr.
 *    Doing so serves as a signal to ATSUI that the caller is done with
 *    the data and that it can merge it in smoothly and adjust its
 *    internal processes. Furthermore, it may be the case that the
 *    pointer returned may be dynamically allocated one or contain
 *    dynamically allocated data. If it's not properly freed, a memory
 *    leak may result. This function may only be called within the
 *    context of an ATSUDirectLayoutOperationOverrideUPP callback. The
 *    pointer returned points to the exact data referenced by the
 *    ATSUTextLayout object that triggered the callback call. This is
 *    by far the most efficient way to use the direct access calls
 *    because for most requested data, no allocation and copy is done.
 *    Furthermore, because this a direct pointer to the data that ATSUI
 *    will use for it's layout, the data arrays returned by this can be
 *    tweaked and edited. Many of the requested arrays are created by
 *    ATSUI only when necessary. If these arrays are to be altered,
 *    then be sure to set iCreate to true. This will ensure that this
 *    array is created. If the arrays are not created, ATSUI
 *    automatically assumes that all entries in the array are zero. The
 *    pointer returned by this function is only valid within the
 *    context of the callback. Do not attempt to retain it for later
 *    use.
 *  
 *  Parameters:
 *    
 *    iLineRef:
 *      The ATSULineRef which was passed into a
 *      ATSUDirectLayoutOperationOverrideUPP callback function as a
 *      parameter.
 *    
 *    iDataSelector:
 *      The selector for the data that is being requested.
 *    
 *    iCreate:
 *      If the ATSULineRef passed in iLineRef does not reference the
 *      requested array, then a zero-filled one will be created and
 *      returned in oLayoutDataArray if this is set to true. For some
 *      ATSUDirectDataSelectors, these cannot be simply created. Thus,
 *      this flag will have no affect on these few
 *      ATSUDirectDataSelectors.
 *    
 *    oLayoutDataArrayPtr:
 *      Upon sucessful return, this parameter will contain a pointer to
 *      an array of the requested values if the ATSULineRef passed in
 *      iLineRef references those values. If this is not the case, then
 *      NULL will be returned, unless iCreate is set to true and the
 *      array can be created. This parameter itself may be set to NULL
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      if only a count of the entries is needed.
=======
 *      if only a count of the entries is needed. can be NULL
>>>>>>> graemeg/fixes_2_2
=======
 *      if only a count of the entries is needed. can be NULL
>>>>>>> origin/fixes_2_2
=======
 *      if only a count of the entries is needed.
>>>>>>> origin/fixes_2.4
 *    
 *    oLayoutDataCount:
 *      Upon sucessful return, this parameter will contain a count of
 *      the entries in the array returned in oLayoutDataArray.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework but deprecated in 10.6
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function ATSUDirectGetLayoutDataArrayPtrFromLineRef( iLineRef: ATSULineRef; iDataSelector: ATSUDirectDataSelector; iCreate: Boolean; oLayoutDataArrayPtr: PtrPtr { can be NULL }; var oLayoutDataCount: ItemCount ): OSStatus; external name '_ATSUDirectGetLayoutDataArrayPtrFromLineRef';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{ ---------------------------------------------------------------------------- }
{$ifc not TARGET_CPU_64}
{
 *  ATSUDirectGetLayoutDataArrayPtrFromTextLayout()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTRunGetGlyphs or CTRunGetPositions instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function ATSUDirectGetLayoutDataArrayPtrFromLineRef( iLineRef: ATSULineRef; iDataSelector: ATSUDirectDataSelector; iCreate: Boolean; oLayoutDataArrayPtr: PtrPtr { can be NULL }; var oLayoutDataCount: ItemCount ): OSStatus; external name '_ATSUDirectGetLayoutDataArrayPtrFromLineRef';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{ ---------------------------------------------------------------------------- }
{$ifc not TARGET_CPU_64}
{
<<<<<<< HEAD
 *  ATSUDirectGetLayoutDataArrayPtrFromTextLayout()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUDirectGetLayoutDataArrayPtrFromTextLayout()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTRunGetGlyphs or CTRunGetPositions instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Returns the data pointer specified by iDataSelector and
 *    referenced by iTextLayout for the line starting at iLineOffset.
 *  
 *  Discussion:
 *    This function simply returns the data pointer specified by
 *    iDataSelector and referenced by iTextLayout for the line starting
 *    at iLineOffset. This data pointer should not be freed directly
 *    after it's been used. Rather, it should be released using
 *    ATSUDirectReleaseLayoutDataArrayPtr. Doing so serves as a signal
 *    to ATSUI that the caller is done with the data. Furthermore, it
 *    may be the case that the pointer returned may be dynamically
 *    allocated one or contain dynamically allocated data. If it's not
 *    properly freed, a memory leak may result. This function may not
 *    be called inside the context of an
 *    ATSUDirectLayoutOperationOverrideUPP callback for the
 *    ATSUTextLayout data that triggered the callback.  All data
 *    returned will be a copy of the data in the object requested. This
 *    means two things: first of all, this means that it's a very
 *    inefficient way of using the data. All of the selectors that
 *    would have returned in constant time now would be forced to
 *    return in order-n time. Second of all, this means that the
 *    developer cannot change any of the data. Any changes the
 *    developer makes to the arrays returned by this API will have no
 *    effect on the layout. Using the 
 *    kATSULayoutOperationPostLayoutAdjustment operation selector
 *    override and the ATSUDirectGetLayoutDataArrayPtrFromLineRef is a
 *    great alternative to using this API. Many of the requested arrays
 *    are created by ATSUI only when necessary. This means that it's
 *    possible that this API will return NULL pointer and a count of 0.
 *    In this case, if there's no error returned, the array simply
 *    doesn't exist and the caller should treat all of the entries in
 *    the array that they would have recieved as being 0.
 *  
 *  Parameters:
 *    
 *    iTextLayout:
 *      The ATSUTextLayout object from which the requested data will
 *      come from.
 *    
 *    iLineOffset:
 *      The edge offset that corresponds to the beginning of the range
 *      of text of the line of the requested data. If the text has
 *      multiple lines, then ATSUDirectGetLayoutDataArrayPtrFromLineRef
 *      will need to be called for each of the lines in which the
 *      requested data is needed.
 *    
 *    iDataSelector:
 *      The selector for the data that is being requested.
 *    
 *    oLayoutDataArrayPtr:
 *      Upon sucessful return, this parameter will contain a pointer to
 *      an array of the requested values if the ATSUTextLayout passed
 *      in iTextLayout references those values for the line offset
 *      iLineOffset. If this is not the case, then NULL will be
 *      returned. This parameter itself may be set to NULL if only a
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      count of the entries is needed.
=======
 *      count of the entries is needed. can be NULL
>>>>>>> graemeg/fixes_2_2
=======
 *      count of the entries is needed. can be NULL
>>>>>>> origin/fixes_2_2
=======
 *      count of the entries is needed.
>>>>>>> origin/fixes_2.4
 *    
 *    oLayoutDataCount:
 *      Upon sucessful return, this parameter will contain a count of
 *      the entries in the array returned in oLayoutDataArray.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function ATSUDirectGetLayoutDataArrayPtrFromTextLayout( iTextLayout: ATSUTextLayout; iLineOffset: UniCharArrayOffset; iDataSelector: ATSUDirectDataSelector; oLayoutDataArrayPtr: PtrPtr { can be NULL }; var oLayoutDataCount: ItemCount ): OSStatus; external name '_ATSUDirectGetLayoutDataArrayPtrFromTextLayout';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{ ---------------------------------------------------------------------------- }
{$endc} {not TARGET_CPU_64}

{
 *  ATSUDirectReleaseLayoutDataArrayPtr()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function ATSUDirectGetLayoutDataArrayPtrFromTextLayout( iTextLayout: ATSUTextLayout; iLineOffset: UniCharArrayOffset; iDataSelector: ATSUDirectDataSelector; oLayoutDataArrayPtr: PtrPtr { can be NULL }; var oLayoutDataCount: ItemCount ): OSStatus; external name '_ATSUDirectGetLayoutDataArrayPtrFromTextLayout';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{ ---------------------------------------------------------------------------- }
{$endc} {not TARGET_CPU_64}

{
<<<<<<< HEAD
 *  ATSUDirectReleaseLayoutDataArrayPtr()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUDirectReleaseLayoutDataArrayPtr()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CoreText API instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    Properly releases of an array pointer returned by
 *    ATSUDirectGetLayoutDataArrayPtrFromLineRef() or
 *    ATSUDirectGetLayoutDataArrayPtrFromTextLayout.
 *  
 *  Discussion:
 *    This function is needed to let ATSUI know that the caller is
 *    finished with the pointer that was previously requested by
 *    ATSUDirectGetLayoutDataArrayPtrFromLineRef() or
 *    ATSUDirectGetLayoutDataArrayPtrFromTextLayout(). This is needed
 *    in case ATSUI needs to make any internal adjustments to it's
 *    internal structures.
 *  
 *  Parameters:
 *    
 *    iLineRef:
 *      The lineRef from which the layout data array pointer came from.
 *      If the layout data array pointer did not come from a lineRef,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *      then set this to NULL.
=======
 *      then set this to NULL. can be NULL
>>>>>>> graemeg/fixes_2_2
=======
 *      then set this to NULL. can be NULL
>>>>>>> origin/fixes_2_2
=======
 *      then set this to NULL.
>>>>>>> origin/fixes_2.4
 *    
 *    iDataSelector:
 *      The selector for which iLayoutDataArrayPtr was obtained.
 *    
 *    iLayoutDataArrayPtr:
 *      A pointer to the layout data array which is to be disposed of.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework but deprecated in 10.6
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function ATSUDirectReleaseLayoutDataArrayPtr( iLineRef: ATSULineRef { can be NULL }; iDataSelector: ATSUDirectDataSelector; iLayoutDataArrayPtr: PtrPtr ): OSStatus; external name '_ATSUDirectReleaseLayoutDataArrayPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{ ---------------------------------------------------------------------------- }
{$ifc not TARGET_CPU_64}
{
 *  ATSUDirectAddStyleSettingRef()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTRunGetGlyphsPtr or CTRunGetGlyphs instead.
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function ATSUDirectReleaseLayoutDataArrayPtr( iLineRef: ATSULineRef { can be NULL }; iDataSelector: ATSUDirectDataSelector; iLayoutDataArrayPtr: PtrPtr ): OSStatus; external name '_ATSUDirectReleaseLayoutDataArrayPtr';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{ ---------------------------------------------------------------------------- }
{$ifc not TARGET_CPU_64}
{
<<<<<<< HEAD
 *  ATSUDirectAddStyleSettingRef()
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *  ATSUDirectAddStyleSettingRef()   *** DEPRECATED ***
 *  
 *  Deprecated:
 *    Use CTRunGetGlyphsPtr or CTRunGetGlyphs instead.
>>>>>>> origin/fixes_2.4
 *  
 *  Summary:
 *    This function will fetch a style index for the
 *    ATSUStyleSettingRef passed in.
 *  
 *  Discussion:
 *    This function allows for glyph replacement or substitution from
 *    one layout or line to another layout or line. Not only will it
 *    look up the style index for iStyleSettingRef, but if the
 *    ATSUStyleSettingRef passed in iStyleSettingRef is not yet part of
 *    the line referenced by iLineRef, it will add it. If there is an
 *    outstanding ATSUStyleSettingRef array obtained by using the
 *    kATSUDirectDataStyleSettingATSUStyleSettingRefArray selector, the
 *    pointer obtained for this may no longer be valid after this
 *    function has been called. These pointers should be freed before
 *    calling this function and re-obtained afterwards.
 *  
 *  Parameters:
 *    
 *    iLineRef:
 *      An ATSULineRef which was passed into a
 *      ATSUDirectLayoutOperationOverrideUPP callback function as a
 *      parameter.
 *    
 *    iStyleSettingRef:
 *      The ATSUStyleSettingRef to be looked up or added to the
 *      ATSUTextLayout referenced by iTextLayout for the line starting
 *      at the offset iLineOffset.
 *    
 *    oStyleIndex:
 *      Upon sucessful return, this will parameter will be set to the
 *      index of the ATSUStyleSettingRef passed in iStyleSettingRef for
 *      the line referenced by iLineRef. If the ATSUStyleSettingRef
 *      does not exist, in that context, then it will be added and the
 *      new index will be returned here.
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function ATSUDirectAddStyleSettingRef( iLineRef: ATSULineRef; iStyleSettingRef: ATSUStyleSettingRef; var oStyleIndex: UInt16 ): OSStatus; external name '_ATSUDirectAddStyleSettingRef';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


{$endc} {not TARGET_CPU_64}

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework
=======
 *    Mac OS X:         in version 10.2 and later in ApplicationServices.framework [32-bit only] but deprecated in 10.6
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.2 and later
 *    Non-Carbon CFM:   not available
 }
function ATSUDirectAddStyleSettingRef( iLineRef: ATSULineRef; iStyleSettingRef: ATSUStyleSettingRef; var oStyleIndex: UInt16 ): OSStatus; external name '_ATSUDirectAddStyleSettingRef';
(* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_6 *)


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
