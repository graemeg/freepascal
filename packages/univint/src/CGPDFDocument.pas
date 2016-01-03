{ CoreGraphics - CGPDFDocument.h
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   Copyright (c) 2000-2011 Apple Inc.
   All rights reserved. }
{       Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, August 2015 }
=======
 * Copyright (c) 2000-2008 Apple Inc.
 * All rights reserved. }
{       Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
 * Copyright (c) 2000-2008 Apple Inc.
 * All rights reserved. }
{       Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
 * Copyright (c) 2000-2008 Apple Inc.
 * All rights reserved. }
{       Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
>>>>>>> graemeg/cpstrnew
=======
 * Copyright (c) 2000-2008 Apple Inc.
 * All rights reserved. }
{       Pascal Translation:  Peter N Lewis, <peter@stairways.com.au>, August 2005 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
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
 * Copyright (c) 2000-2003 Apple Computer, Inc.
 * All rights reserved.
 }
=======
 * Copyright (c) 2000-2008 Apple Inc.
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

unit CGPDFDocument;
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
=======
>>>>>>> graemeg/cpstrnew
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := TRUE}
{$elifc defined __arm64__ and __arm64__}
=======
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
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_CPU_ARM64 := TRUE}
=======
	{$setc TARGET_CPU_ARM := TRUE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_CPU_ARM := TRUE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_CPU_ARM := TRUE}
>>>>>>> origin/cpstrnew
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
uses MacTypes,CFBase,CGBase,CGDataProvider,CGGeometry,CFURL;
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


type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	CGPDFDocumentRef = ^OpaqueCGPDFDocumentRef; { an opaque type }
	OpaqueCGPDFDocumentRef = record end;
=======
	CGPDFDocumentRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CGPDFDocumentRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CGPDFDocumentRef = ^SInt32; { an opaque type }
>>>>>>> graemeg/cpstrnew
=======
	CGPDFDocumentRef = ^SInt32; { an opaque type }
>>>>>>> origin/cpstrnew


{ Create a PDF document, using `provider' to obtain the document's data. }

function CGPDFDocumentCreateWithProvider( provider: CGDataProviderRef ): CGPDFDocumentRef; external name '_CGPDFDocumentCreateWithProvider';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
	CGPDFDocumentRef = ^SInt32; { an opaque 32-bit type }
=======
	CGPDFDocumentRef = ^SInt32; { an opaque type }
>>>>>>> origin/fixes_2.4


{ Create a PDF document, using `provider' to obtain the document's data. }

function CGPDFDocumentCreateWithProvider( provider: CGDataProviderRef ): CGPDFDocumentRef; external name '_CGPDFDocumentCreateWithProvider';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ Create a PDF document from `url'. }

function CGPDFDocumentCreateWithURL( url: CFURLRef ): CGPDFDocumentRef; external name '_CGPDFDocumentCreateWithURL';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Equivalent to `CFRetain(document)', except it doesn't crash (as CFRetain
   does) if `document' is NULL. }

function CGPDFDocumentRetain( document: CGPDFDocumentRef ): CGPDFDocumentRef; external name '_CGPDFDocumentRetain';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Equivalent to `CFRelease(document)', except it doesn't crash (as
   CFRelease does) if `document' is NULL. }

procedure CGPDFDocumentRelease( document: CGPDFDocumentRef ); external name '_CGPDFDocumentRelease';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the major and minor version numbers of `document'. }

procedure CGPDFDocumentGetVersion( document: CGPDFDocumentRef; var majorVersion: SInt32; var minorVersion: SInt32 ); external name '_CGPDFDocumentGetVersion';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{ Return true if the PDF file associated with `document' is encrypted;
   false otherwise. If the PDF file is encrypted, then a password must be
   supplied before certain operations are enabled; different passwords may
   enable different operations. }

function CGPDFDocumentIsEncrypted( document: CGPDFDocumentRef ): CBool; external name '_CGPDFDocumentIsEncrypted';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Use `password' to decrypt `document' and grant permission for certain
   operations. Returns true if `password' is a valid password; false
   otherwise. }

function CGPDFDocumentUnlockWithPassword( document: CGPDFDocumentRef; password: ConstCStringPtr ): CBool; external name '_CGPDFDocumentUnlockWithPassword';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Return true if `document' is unlocked; false otherwise. A document is
   unlocked if it isn't encrypted, or if it is encrypted and a valid
   password was specified with `CGPDFDocumentUnlockWithPassword'. }

function CGPDFDocumentIsUnlocked( document: CGPDFDocumentRef ): CBool; external name '_CGPDFDocumentIsUnlocked';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Return true if `document' allows printing; false otherwise. Typically,
   this function returns false only if the document is encrypted and the
   document's current password doesn't grant permission to perform
   printing. }

function CGPDFDocumentAllowsPrinting( document: CGPDFDocumentRef ): CBool; external name '_CGPDFDocumentAllowsPrinting';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Return true if `document' allows copying; false otherwise. Typically,
   this function returns false only if the document is encrypted and the
   document's current password doesn't grant permission to perform
   copying. }

function CGPDFDocumentAllowsCopying( document: CGPDFDocumentRef ): CBool; external name '_CGPDFDocumentAllowsCopying';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)
=======
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ Equivalent to `CFRetain(document)', except it doesn't crash (as CFRetain
   does) if `document' is NULL. }

function CGPDFDocumentRetain( document: CGPDFDocumentRef ): CGPDFDocumentRef; external name '_CGPDFDocumentRetain';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Equivalent to `CFRelease(document)', except it doesn't crash (as
   CFRelease does) if `document' is NULL. }

procedure CGPDFDocumentRelease( document: CGPDFDocumentRef ); external name '_CGPDFDocumentRelease';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the major and minor version numbers of `document'. }

procedure CGPDFDocumentGetVersion( document: CGPDFDocumentRef; var majorVersion: SInt32; var minorVersion: SInt32 ); external name '_CGPDFDocumentGetVersion';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{ Return true if the PDF file associated with `document' is encrypted;
   false otherwise. If the PDF file is encrypted, then a password must be
   supplied before certain operations are enabled; different passwords may
   enable different operations. }

function CGPDFDocumentIsEncrypted( document: CGPDFDocumentRef ): CBool; external name '_CGPDFDocumentIsEncrypted';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Use `password' to decrypt `document' and grant permission for certain
   operations. Returns true if `password' is a valid password; false
   otherwise. }

function CGPDFDocumentUnlockWithPassword( document: CGPDFDocumentRef; password: ConstCStringPtr ): CBool; external name '_CGPDFDocumentUnlockWithPassword';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Return true if `document' is unlocked; false otherwise. A document is
   unlocked if it isn't encrypted, or if it is encrypted and a valid
   password was specified with `CGPDFDocumentUnlockWithPassword'. }

function CGPDFDocumentIsUnlocked( document: CGPDFDocumentRef ): CBool; external name '_CGPDFDocumentIsUnlocked';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Return true if `document' allows printing; false otherwise. Typically,
   this function returns false only if the document is encrypted and the
   document's current password doesn't grant permission to perform
   printing. }

function CGPDFDocumentAllowsPrinting( document: CGPDFDocumentRef ): CBool; external name '_CGPDFDocumentAllowsPrinting';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{ Return true if `document' allows copying; false otherwise. Typically,
   this function returns false only if the document is encrypted and the
   document's current password doesn't grant permission to perform
   copying. }

<<<<<<< HEAD
function CGPDFDocumentAllowsCopying( document: CGPDFDocumentRef ): CBool; external name '_CGPDFDocumentAllowsCopying'; (* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CGPDFDocumentAllowsCopying( document: CGPDFDocumentRef ): CBool; external name '_CGPDFDocumentAllowsCopying';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ Return the number of pages in `document'. }

function CGPDFDocumentGetNumberOfPages( document: CGPDFDocumentRef ): size_t; external name '_CGPDFDocumentGetNumberOfPages';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Return the page corresponding to `pageNumber', or NULL if no such page
   exists in the document. Pages are numbered starting at 1. }

function CGPDFDocumentGetPage( document: CGPDFDocumentRef; pageNumber: size_t ): CGPDFPageRef; external name '_CGPDFDocumentGetPage';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{ Return the document catalog of `document'. }

function CGPDFDocumentGetCatalog( document: CGPDFDocumentRef ): CGPDFDictionaryRef; external name '_CGPDFDocumentGetCatalog';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{ Return the info dictionary of `document'. }

function CGPDFDocumentGetInfo( document: CGPDFDocumentRef ): CGPDFDictionaryRef; external name '_CGPDFDocumentGetInfo';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Return the "file identifier" array of `document'. }

function CGPDFDocumentGetID( document: CGPDFDocumentRef ): CGPDFArrayRef; external name '_CGPDFDocumentGetID';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Return the CFTypeID for CGPDFDocumentRefs. }

function CGPDFDocumentGetTypeID: CFTypeID; external name '_CGPDFDocumentGetTypeID';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{$ifc TARGET_OS_MAC}
=======
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)
>>>>>>> origin/fixes_2.4

{ Return the page corresponding to `pageNumber', or NULL if no such page
   exists in the document. Pages are numbered starting at 1. }

function CGPDFDocumentGetPage( document: CGPDFDocumentRef; pageNumber: size_t ): CGPDFPageRef; external name '_CGPDFDocumentGetPage';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{ Return the document catalog of `document'. }

function CGPDFDocumentGetCatalog( document: CGPDFDocumentRef ): CGPDFDictionaryRef; external name '_CGPDFDocumentGetCatalog';
(* CG_AVAILABLE_STARTING(__MAC_10_3, __IPHONE_2_0) *)

{ Return the info dictionary of `document'. }

function CGPDFDocumentGetInfo( document: CGPDFDocumentRef ): CGPDFDictionaryRef; external name '_CGPDFDocumentGetInfo';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Return the "file identifier" array of `document'. }

function CGPDFDocumentGetID( document: CGPDFDocumentRef ): CGPDFArrayRef; external name '_CGPDFDocumentGetID';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Return the CFTypeID for CGPDFDocumentRefs. }

<<<<<<< HEAD
function CGPDFDocumentGetTypeID: CFTypeID; external name '_CGPDFDocumentGetTypeID'; (* AVAILABLE_MAC_OS_X_VERSION_10_2_AND_LATER *)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
function CGPDFDocumentGetTypeID: CFTypeID; external name '_CGPDFDocumentGetTypeID';
(* CG_AVAILABLE_STARTING(__MAC_10_2, __IPHONE_2_0) *)

{$ifc TARGET_OS_MAC}
>>>>>>> origin/fixes_2.4

{ The following functions are deprecated in favor of the CGPDFPage API. }

{ DEPRECATED; return the media box of page number `page' in `document'. }

function CGPDFDocumentGetMediaBox( document: CGPDFDocumentRef; page: SInt32 ): CGRect; external name '_CGPDFDocumentGetMediaBox';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5,
	__IPHONE_NA, __IPHONE_NA) *)
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5,
	__IPHONE_NA, __IPHONE_NA) *)
>>>>>>> origin/fixes_2.4

{ DEPRECATED; return the crop box of page number `page' in `document'. }

function CGPDFDocumentGetCropBox( document: CGPDFDocumentRef; page: SInt32 ): CGRect; external name '_CGPDFDocumentGetCropBox';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5,
	__IPHONE_NA, __IPHONE_NA) *)
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5,
	__IPHONE_NA, __IPHONE_NA) *)
>>>>>>> origin/fixes_2.4

{ DEPRECATED; return the bleed box of page number `page' in `document'. }

function CGPDFDocumentGetBleedBox( document: CGPDFDocumentRef; page: SInt32 ): CGRect; external name '_CGPDFDocumentGetBleedBox';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5,
	__IPHONE_NA, __IPHONE_NA) *)
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5,
	__IPHONE_NA, __IPHONE_NA) *)
>>>>>>> origin/fixes_2.4

{ DEPRECATED; return the trim box of page number `page' in `document'. }

function CGPDFDocumentGetTrimBox( document: CGPDFDocumentRef; page: SInt32 ): CGRect; external name '_CGPDFDocumentGetTrimBox';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5,
	__IPHONE_NA, __IPHONE_NA) *)
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5,
	__IPHONE_NA, __IPHONE_NA) *)
>>>>>>> origin/fixes_2.4

{ DEPRECATED; return the art box of page number `page' in `document'. }

function CGPDFDocumentGetArtBox( document: CGPDFDocumentRef; page: SInt32 ): CGRect; external name '_CGPDFDocumentGetArtBox';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5,
	__IPHONE_NA, __IPHONE_NA) *)

{ DEPRECATED; return the rotation angle (in degrees) of page number `page'
   in `document'. }

function CGPDFDocumentGetRotationAngle( document: CGPDFDocumentRef; page: SInt32 ): SInt32; external name '_CGPDFDocumentGetRotationAngle';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5,
	__IPHONE_NA, __IPHONE_NA) *)

{$endc}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
=======
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5,
	__IPHONE_NA, __IPHONE_NA) *)
>>>>>>> origin/fixes_2.4

{ DEPRECATED; return the rotation angle (in degrees) of page number `page'
   in `document'. }

function CGPDFDocumentGetRotationAngle( document: CGPDFDocumentRef; page: SInt32 ): SInt32; external name '_CGPDFDocumentGetRotationAngle';
(* CG_AVAILABLE_BUT_DEPRECATED(__MAC_10_0, __MAC_10_5,
	__IPHONE_NA, __IPHONE_NA) *)

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
