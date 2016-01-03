{ CoreGraphics - CGPDFDictionary.h
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 * Copyright (c) 2002-2008 Apple Inc.
 * All rights reserved. }
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
 * Copyright (c) 2002-2004 Apple Computer, Inc.
 * All rights reserved.
 }
=======
 * Copyright (c) 2002-2008 Apple Inc.
 * All rights reserved. }
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

unit CGPDFDictionary;
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
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
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
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
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
<<<<<<< HEAD
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
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
=======
{$elsec}
=======
{$elsec}
>>>>>>> origin/cpstrnew
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
<<<<<<< HEAD
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
uses MacTypes,CGPDFObject,CGBase;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not MACOSALLINCLUDE}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {not MACOSALLINCLUDE}

>>>>>>> origin/fixes_2.4
{$ALIGN POWER}


// CGPDFDictionaryRef defined in CGBase


{ Return the number of entries in `dictionary'. }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function CGPDFDictionaryGetCount( dict: CGPDFDictionaryRef ): size_t; external name '_CGPDFDictionaryGetCount';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{ Look up the object associated with `key' in `dict' and return the result
   in `value'. Return true on success; false otherwise. }

function CGPDFDictionaryGetObject( dict: CGPDFDictionaryRef; key: ConstCStringPtr; var value: CGPDFObjectRef ): CBool; external name '_CGPDFDictionaryGetObject';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{ Look up the object associated with `key' in `dict' and, if it's a
   boolean, return the result in `value'. Return true on success; false
   otherwise. }

function CGPDFDictionaryGetBoolean( dict: CGPDFDictionaryRef; key: ConstCStringPtr; var value: CGPDFBoolean ): CBool; external name '_CGPDFDictionaryGetBoolean';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{ Look up the object associated with `key' in `dict' and, if it's an
   integer, return the result in `value'. Return true on success; false
   otherwise. }

function CGPDFDictionaryGetInteger( dict: CGPDFDictionaryRef; key: ConstCStringPtr; var value: CGPDFInteger ): CBool; external name '_CGPDFDictionaryGetInteger';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{ Look up the object associated with `key' in `dict' and, if it's a number
   (real or integer), return the result in `value'. Return true on success;
   false otherwise. }

function CGPDFDictionaryGetNumber( dict: CGPDFDictionaryRef; key: ConstCStringPtr; var value: CGPDFReal ): CBool; external name '_CGPDFDictionaryGetNumber';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{ Look up the object associated with `key' in `dict' and, if it's a name,
   return the result in `value'. Return true on success; false otherwise. }

function CGPDFDictionaryGetName( dict: CGPDFDictionaryRef; key: ConstCStringPtr; var value: ConstCStringPtr ): CBool; external name '_CGPDFDictionaryGetName';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{ Look up the object associated with `key' in `dict' and, if it's a string,
   return the result in `value'. Return true on success; false otherwise. }

function CGPDFDictionaryGetString( dict: CGPDFDictionaryRef; key: ConstCStringPtr; var value: CGPDFStringRef ): CBool; external name '_CGPDFDictionaryGetString';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{ Look up the object associated with `key' in `dict' and, if it's an array,
   return the result in `value'. Return true on success; false otherwise. }

function CGPDFDictionaryGetArray( dict: CGPDFDictionaryRef; key: ConstCStringPtr; var value: CGPDFArrayRef ): CBool; external name '_CGPDFDictionaryGetArray';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{ Look up the object associated with `key' in `dict' and, if it's a
   dictionary, return the result in `value'. Return true on success; false
   otherwise. }

function CGPDFDictionaryGetDictionary( dict: CGPDFDictionaryRef; key: ConstCStringPtr; var value: CGPDFDictionaryRef ): CBool; external name '_CGPDFDictionaryGetDictionary';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{ Look up the object associated with `key' in `dict' and, if it's a stream,
   return the result in `value'. Return true on success; false otherwise. }

function CGPDFDictionaryGetStream( dict: CGPDFDictionaryRef; key: ConstCStringPtr; var value: CGPDFStreamRef ): CBool; external name '_CGPDFDictionaryGetStream';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{ The callback for `CGPDFDictionaryApplyFunction'. `key' is the current
   key, `value' is the value for `key', and `info' is the parameter passed
   to `CGPDFDictionaryApplyFunction'. }
=======
=======
>>>>>>> origin/fixes_2_2
function CGPDFDictionaryGetCount( dict: CGPDFDictionaryRef ): size_t; external name '_CGPDFDictionaryGetCount'; (* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
=======
function CGPDFDictionaryGetCount( dict: CGPDFDictionaryRef ): size_t; external name '_CGPDFDictionaryGetCount';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ Look up the object associated with `key' in `dict' and return the result
   in `value'. Return true on success; false otherwise. }

function CGPDFDictionaryGetObject( dict: CGPDFDictionaryRef; key: ConstCStringPtr; var value: CGPDFObjectRef ): CBool; external name '_CGPDFDictionaryGetObject';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{ Look up the object associated with `key' in `dict' and, if it's a
   boolean, return the result in `value'. Return true on success; false
   otherwise. }

function CGPDFDictionaryGetBoolean( dict: CGPDFDictionaryRef; key: ConstCStringPtr; var value: CGPDFBoolean ): CBool; external name '_CGPDFDictionaryGetBoolean';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{ Look up the object associated with `key' in `dict' and, if it's an
   integer, return the result in `value'. Return true on success; false
   otherwise. }

function CGPDFDictionaryGetInteger( dict: CGPDFDictionaryRef; key: ConstCStringPtr; var value: CGPDFInteger ): CBool; external name '_CGPDFDictionaryGetInteger';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{ Look up the object associated with `key' in `dict' and, if it's a number
   (real or integer), return the result in `value'. Return true on success;
   false otherwise. }

function CGPDFDictionaryGetNumber( dict: CGPDFDictionaryRef; key: ConstCStringPtr; var value: CGPDFReal ): CBool; external name '_CGPDFDictionaryGetNumber';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{ Look up the object associated with `key' in `dict' and, if it's a name,
   return the result in `value'. Return true on success; false otherwise. }

function CGPDFDictionaryGetName( dict: CGPDFDictionaryRef; key: ConstCStringPtr; var value: ConstCStringPtr ): CBool; external name '_CGPDFDictionaryGetName';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{ Look up the object associated with `key' in `dict' and, if it's a string,
   return the result in `value'. Return true on success; false otherwise. }

function CGPDFDictionaryGetString( dict: CGPDFDictionaryRef; key: ConstCStringPtr; var value: CGPDFStringRef ): CBool; external name '_CGPDFDictionaryGetString';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{ Look up the object associated with `key' in `dict' and, if it's an array,
   return the result in `value'. Return true on success; false otherwise. }

function CGPDFDictionaryGetArray( dict: CGPDFDictionaryRef; key: ConstCStringPtr; var value: CGPDFArrayRef ): CBool; external name '_CGPDFDictionaryGetArray';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{ Look up the object associated with `key' in `dict' and, if it's a
   dictionary, return the result in `value'. Return true on success; false
   otherwise. }

function CGPDFDictionaryGetDictionary( dict: CGPDFDictionaryRef; key: ConstCStringPtr; var value: CGPDFDictionaryRef ): CBool; external name '_CGPDFDictionaryGetDictionary';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{ Look up the object associated with `key' in `dict' and, if it's a stream,
   return the result in `value'. Return true on success; false otherwise. }

function CGPDFDictionaryGetStream( dict: CGPDFDictionaryRef; key: ConstCStringPtr; var value: CGPDFStreamRef ): CBool; external name '_CGPDFDictionaryGetStream';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

<<<<<<< HEAD
{ The callback for `CGPDFDictionaryApplyFunction'.  `key' is the current
 * key, `value' is the value for `key', and `info' is the parameter passed
 * to `CGPDFDictionaryApplyFunction'. }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{ The callback for `CGPDFDictionaryApplyFunction'. `key' is the current
   key, `value' is the value for `key', and `info' is the parameter passed
   to `CGPDFDictionaryApplyFunction'. }
>>>>>>> origin/fixes_2.4

type
	CGPDFDictionaryApplierFunction = procedure( key: ConstCStringPtr; value: CGPDFObjectRef; info: UnivPtr );

{ Enumerate all of the keys in `dict', calling `function' once for each
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   key/value pair. Passes the current key, the associated value, and `info'
   to `function'. }

procedure CGPDFDictionaryApplyFunction( dict: CGPDFDictionaryRef; func: CGPDFDictionaryApplierFunction; info: UnivPtr ); external name '_CGPDFDictionaryApplyFunction';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
 * key/value pair.  Passes the current key, the associated value, and
 * `info' to `function'. }
=======
   key/value pair. Passes the current key, the associated value, and `info'
   to `function'. }
>>>>>>> origin/fixes_2.4

procedure CGPDFDictionaryApplyFunction( dict: CGPDFDictionaryRef; func: CGPDFDictionaryApplierFunction; info: UnivPtr ); external name '_CGPDFDictionaryApplyFunction';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

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
