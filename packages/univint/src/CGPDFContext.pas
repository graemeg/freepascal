{ CoreGraphics - CGPDFContext.h
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
 * Copyright (c) 2000-2004 Apple Computer, Inc.
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

unit CGPDFContext;
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
=======
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
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
<<<<<<< HEAD
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
	{$setc TARGET_OS_EMBEDDED := FALSE}
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
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
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
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
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
	{$setc TARGET_OS_EMBEDDED := TRUE}
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ nor __arm64__ is defined.}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
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
uses MacTypes,CGGeometry,CFBase,CFData,CFDictionary,CFURL,CGBase,CGContext,CGDataConsumer;
=======
uses MacTypes,CGGeometry,CFBase,CFDictionary,CFURL,CGBase,CGContext,CGDataConsumer;
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
{$endc} {not MACOSALLINCLUDE}

=======
uses MacTypes,CGGeometry,CFBase,CFDictionary,CFURL,CGBase,CGContext,CGDataConsumer;
>>>>>>> graemeg/fixes_2_2
=======
uses MacTypes,CGGeometry,CFBase,CFDictionary,CFURL,CGBase,CGContext,CGDataConsumer;
<<<<<<< HEAD
>>>>>>> origin/fixes_2_2
=======
{$endc} {not MACOSALLINCLUDE}

>>>>>>> origin/fixes_2.4
{$ALIGN POWER}


{ Create a PDF context, using `consumer' for output. `mediaBox' is the
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
   default page media bounding box; if NULL, then a default page size is
   used. `auxiliaryInfo' specifies additional information used by the PDF
   context when generating the PDF file. The keys and values in
   `auxiliaryInfo' are described below. If `mediaBox' is non-NULL, then its
   value overrides the value of `kCGPDFContextMediaBox' if specified in the
   `auxiliaryInfo' dictionary. }
<<<<<<< HEAD

function CGPDFContextCreate( consumer: CGDataConsumerRef; const (*var*) mediaBox: CGRect; auxiliaryInfo: CFDictionaryRef ): CGContextRef; external name '_CGPDFContextCreate';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Create a PDF context for writing to `url'. This function behaves in the
   same manner as the above function, except that the output data will be
   written to `url'. }

function CGPDFContextCreateWithURL( url: CFURLRef; const (*var*) mediaBox: CGRect; auxiliaryInfo: CFDictionaryRef ): CGContextRef; external name '_CGPDFContextCreateWithURL';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Close a PDF context. After closing the context, all pending data is
   written to the context's destination, and the PDF file is completed. No
   additional data will be written to the context's destionation after
   closing. }

procedure CGPDFContextClose( context: CGContextRef ); external name '_CGPDFContextClose';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Begin a new page in the PDF context `context'. }

procedure CGPDFContextBeginPage( context: CGContextRef; pageInfo: CFDictionaryRef ); external name '_CGPDFContextBeginPage';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ End the current page in the PDF context `context'. }

procedure CGPDFContextEndPage( context: CGContextRef ); external name '_CGPDFContextEndPage';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{ Add the metadata stream specified by `metadata' to the document catalog
   of `context', as described in Table 3.25, "Entries in the catalog
   dictionary", of the PDF 1.7 specification. The contents of metadata must
   be XML formatted according to the Extensible Metadata Platform, as
   described in section 10.2.2, "Metadata Streams", of the PDF 1.7
   specification. }

procedure CGPDFContextAddDocumentMetadata( context: CGContextRef; metadata: CFDataRef ); external name '_CGPDFContextAddDocumentMetadata';
(* CG_AVAILABLE_STARTING(__MAC_10_7, __IPHONE_4_0) *)
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{ Set the URL associated with `rect' to `url' in the PDF context
   `context'. }

procedure CGPDFContextSetURLForRect( context: CGContextRef; url: CFURLRef; rect: CGRect ); external name '_CGPDFContextSetURLForRect';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Create a PDF destination named `name' at `point' in the current page of
   the PDF context `context'. }

procedure CGPDFContextAddDestinationAtPoint( context: CGContextRef; name: CFStringRef; point: CGPoint ); external name '_CGPDFContextAddDestinationAtPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Specify a destination named `name' to jump to when clicking in `rect' of
   the current page of the PDF context `context'. }

procedure CGPDFContextSetDestinationForRect( context: CGContextRef; name: CFStringRef; rect: CGRect ); external name '_CGPDFContextSetDestinationForRect';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{** Keys for the auxiliary info dictionary or the page info dictionary. **} 

{ The media box for the document or for a given page. Optional; if present,
   the value of this key must be a CFData containing a CGRect (stored by
   value, not by reference). }

var kCGPDFContextMediaBox: CFStringRef; external name '_kCGPDFContextMediaBox'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ The crop box for the document or for a given page. Optional; if present,
   the value of this key must be a CFData containing a CGRect (stored by
   value, not by reference). }

var kCGPDFContextCropBox: CFStringRef; external name '_kCGPDFContextCropBox'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ The bleed box for the document or for a given page. Optional; if present,
   the value of this key must be a CFData containing a CGRect (stored by
   value, not by reference). }

var kCGPDFContextBleedBox: CFStringRef; external name '_kCGPDFContextBleedBox'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ The trim box for the document or for a given page. Optional; if present,
   the value of this key must be a CFData containing a CGRect (stored by
   value, not by reference). }

var kCGPDFContextTrimBox: CFStringRef; external name '_kCGPDFContextTrimBox'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ The art box for the document or for a given page. Optional; if present,
   the value of this key must be a CFData containing a CGRect (stored by
   value, not by reference). }

var kCGPDFContextArtBox: CFStringRef; external name '_kCGPDFContextArtBox'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{** Keys for auxiliary info dictionary. **}

{ The document's title. Optional; if present, the value of this key must be
   a CFString. }

//const kCGPDFContextTitle: CFStringRef;
//CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0);

{ The name of the person who created this document. Optional; if present,
   the value of this key must be a CFString. }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

//const kCGPDFContextAuthor: CFStringRef;
//CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0);
<<<<<<< HEAD
=======

//const kCGPDFContextAuthor: CFStringRef;
//CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0);
>>>>>>> graemeg/cpstrnew
=======

//const kCGPDFContextAuthor: CFStringRef;
//CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0);
>>>>>>> origin/cpstrnew

{ The subject of a document. Optional; if present, the value of this key
   must be a CFString. }

var kCGPDFContextSubject: CFStringRef; external name '_kCGPDFContextSubject'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)
<<<<<<< HEAD
<<<<<<< HEAD

{ The keywords for this document. This key is optional. If the value of
   this key is a CFString, the /Keywords entry will be the specified string.
   If the value of this key is a CFArray, then it must be an array of
   CFStrings. The /Keywords entry will in this case be the concatenation of
   the specified strings separated by commas (","). In addition, an entry
   with the key "/AAPL:Keywords" will be stored in the document information
   dictionary; its value is an array consisting of each of the specified
   strings. The value of this key must be in one of the above forms;
   otherwise, this key is ignored. }

var kCGPDFContextKeywords: CFStringRef; external name '_kCGPDFContextKeywords'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

=======

{ The subject of a document. Optional; if present, the value of this key
   must be a CFString. }

var kCGPDFContextSubject: CFStringRef; external name '_kCGPDFContextSubject'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ The keywords for this document. This key is optional. If the value of
   this key is a CFString, the /Keywords entry will be the specified string.
   If the value of this key is a CFArray, then it must be an array of
   CFStrings. The /Keywords entry will in this case be the concatenation of
   the specified strings separated by commas (","). In addition, an entry
   with the key "/AAPL:Keywords" will be stored in the document information
   dictionary; its value is an array consisting of each of the specified
   strings. The value of this key must be in one of the above forms;
   otherwise, this key is ignored. }

var kCGPDFContextKeywords: CFStringRef; external name '_kCGPDFContextKeywords'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

>>>>>>> graemeg/cpstrnew
{ The name of the application that created the original data used to create
   this document. Optional; if present, the value of this key must be a
   CFString. }

//const kCGPDFContextCreator: CFStringRef;
//CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0);

{ The "owner password" of the PDF document. If this key is specified, the
   document will be encrypted using the value as the owner password;
   otherwise, the document will not be encrypted. The value of this key must
   be a CFStringRef which can be represented in ASCII encoding; only the
   first 32 bytes will be used for the password. There is no default value
   for this key.
<<<<<<< HEAD

   If the value of this key cannot be represented in ASCII, the document
   will not be created and the creation function will return NULL. }

=======

   If the value of this key cannot be represented in ASCII, the document
   will not be created and the creation function will return NULL. }

>>>>>>> graemeg/cpstrnew
=======

{ The keywords for this document. This key is optional. If the value of
   this key is a CFString, the /Keywords entry will be the specified string.
   If the value of this key is a CFArray, then it must be an array of
   CFStrings. The /Keywords entry will in this case be the concatenation of
   the specified strings separated by commas (","). In addition, an entry
   with the key "/AAPL:Keywords" will be stored in the document information
   dictionary; its value is an array consisting of each of the specified
   strings. The value of this key must be in one of the above forms;
   otherwise, this key is ignored. }

var kCGPDFContextKeywords: CFStringRef; external name '_kCGPDFContextKeywords'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ The name of the application that created the original data used to create
   this document. Optional; if present, the value of this key must be a
   CFString. }

//const kCGPDFContextCreator: CFStringRef;
//CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0);

{ The "owner password" of the PDF document. If this key is specified, the
   document will be encrypted using the value as the owner password;
   otherwise, the document will not be encrypted. The value of this key must
   be a CFStringRef which can be represented in ASCII encoding; only the
   first 32 bytes will be used for the password. There is no default value
   for this key.

   If the value of this key cannot be represented in ASCII, the document
   will not be created and the creation function will return NULL. }

>>>>>>> origin/cpstrnew
var kCGPDFContextOwnerPassword: CFStringRef; external name '_kCGPDFContextOwnerPassword'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ The "user password" of the PDF document. If the document is encrypted,
   then the value of this key will be the user password for the document; if
   unspecified, the user password will be the empty string. The value of
   this key must be a CFStringRef which can be represented in ASCII
   encoding; only the first 32 bytes will be used for the password.

   If the value of this key cannot be represented in ASCII, the document
   will not be created and the creation function will return NULL. }
<<<<<<< HEAD
<<<<<<< HEAD

var kCGPDFContextUserPassword: CFStringRef; external name '_kCGPDFContextUserPassword'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

=======

var kCGPDFContextUserPassword: CFStringRef; external name '_kCGPDFContextUserPassword'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

>>>>>>> graemeg/cpstrnew
=======

var kCGPDFContextUserPassword: CFStringRef; external name '_kCGPDFContextUserPassword'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

>>>>>>> origin/cpstrnew
{ Specifies the encryption key length in bits; see Table 3.18 "Entries
   common to all encryption dictionaries", PDF Reference: Adobe PDF version
   1.5 (4th ed.) for more info. Optional; if present, the value of this key
   must be a CFNumber with value which is a multiple of 8 between 40 and
   128, inclusive. If this key is absent or invalid, the encryption key
   length defaults to 40 bits. }

var kCGPDFContextEncryptionKeyLength: CFStringRef; external name '_kCGPDFContextEncryptionKeyLength'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Used to specify whether the document allows printing when unlocked with
   the user password. The value of this key must be a CFBooleanRef. The
   default value of this key is "kCFBooleanTrue". }

var kCGPDFContextAllowsPrinting: CFStringRef; external name '_kCGPDFContextAllowsPrinting'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Used to specify whether the document allows copying when unlocked with
   the user password. The value of this key must be a CFBooleanRef. The
   default value of this key is "kCFBooleanTrue". }

var kCGPDFContextAllowsCopying: CFStringRef; external name '_kCGPDFContextAllowsCopying'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{$ifc TARGET_OS_MAC}
{ The document's PDF/X output intent. Optional; if present, the value of
   this key must be a CFDictionaryRef. The dictionary is added to the
   /OutputIntents entry in the PDF file's document catalog. The keys and
   values contained in the dictionary must match those specified in section
   9.10.4 of the PDF 1.4 specification, ISO/DIS 15930-3 document published
   by ISO/TC 130, and Adobe Technical Note #5413. }

//const kCGPDFContextOutputIntent: CFStringRef;
//CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA);

{ The following keys are supported in the output intent dictionary:

   kCGPDFXOutputIntentSubtype ("S"): The output intent subtype. This key is
<<<<<<< HEAD
<<<<<<< HEAD
   required; the value of this key must be a CFString equal to "GTS_PDFX";
   otherwise, the dictionary is ignored. }
=======
      required; the value of this key must be a CFString equal to
      "GTS_PDFX"; otherwise, the dictionary is ignored. }
>>>>>>> graemeg/cpstrnew

var kCGPDFXOutputIntentSubtype: CFStringRef; external name '_kCGPDFXOutputIntentSubtype'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ kCGPDFXOutputConditionIdentifier ("OutputConditionIdentifier"): A string
<<<<<<< HEAD
   identifying the intended output device or production condition in a
   human- or machine-readable form. This key is required; the value of this
   key must be a CFString. For best results, the string should be
   representable losslessly in ASCII encoding. }
=======
      identifying the intended output device or production condition in a
      human- or machine-readable form. This key is required; the value of
      this key must be a CFString. For best results, the string should be
      representable losslessly in ASCII encoding. }
>>>>>>> graemeg/cpstrnew

var kCGPDFXOutputConditionIdentifier: CFStringRef; external name '_kCGPDFXOutputConditionIdentifier'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ kCGPDFXOutputCondition ("OutputCondition"): A text string identifying the
<<<<<<< HEAD
   intended output device or production condition in a human-readable form.
   This key is optional; if present, the value of this key must be a
   CFString. }
=======
      intended output device or production condition in a human-readable
      form. This key is optional; if present, the value of this key must be
      a CFString. }
>>>>>>> graemeg/cpstrnew

var kCGPDFXOutputCondition: CFStringRef; external name '_kCGPDFXOutputCondition'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ kCGPDFXRegistryName ("RegistryName"): A string identifying the registry
<<<<<<< HEAD
   in which the condition designated by `kCGPDFXOutputConditionIdentifier'
   is defined. This key is optional; if present, the value of this key must
   be a CFString. For best results, the string should be representable
   losslessly in ASCII encoding. }
=======
     in which the condition designated by `kCGPDFXOutputConditionIdentifier'
     is defined. This key is optional; if present, the value of this key
     must be a CFString. For best results, the string should be
     representable losslessly in ASCII encoding. }
>>>>>>> graemeg/cpstrnew

var kCGPDFXRegistryName: CFStringRef; external name '_kCGPDFXRegistryName'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ kCGPDFXInfo ("Info"): A human-readable text string containing additional
<<<<<<< HEAD
   information about the intended target device or production condition.
   This key is required if the value of `kCGPDFXOutputConditionIdentifier'
   does not specify a standard production condition; it is optional
   otherwise. If present, the value of this key must be a CFString. }
=======
      information or comments about the intended target device or production
      condition. This key is required if the value of
      `kCGPDFXOutputConditionIdentifier' does not specify a standard
      production condition; it is optional otherwise. If present, the value
      of this key must be a CFString. }
>>>>>>> graemeg/cpstrnew

var kCGPDFXInfo: CFStringRef; external name '_kCGPDFXInfo'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ kCGPDFXDestinationOutputProfile ("DestOutputProfile"): An ICC profile
<<<<<<< HEAD
   stream defining the transformation from the PDF document's source colors
   to output device colorants. This key is required if the value of
   `kCGPDFXOutputConditionIdentifier' does not specify a standard production
   condition; it is optional otherwise. If present, the value of this key
   must be a ICC-based CGColorSpaceRef. }
=======
      stream defining the transformation from the PDF document's source
      colors to output device colorants. This key is required if the value
      of `kCGPDFXOutputConditionIdentifier' does not specify a standard
      production condition; it is optional otherwise. If present, the value
      of this key must be a ICC-based CGColorSpaceRef. }
>>>>>>> graemeg/cpstrnew

=======

//const kCGPDFContextAuthor: CFStringRef;
//CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0);

{ The subject of a document. Optional; if present, the value of this key
   must be a CFString. }

var kCGPDFContextSubject: CFStringRef; external name '_kCGPDFContextSubject'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ The keywords for this document. This key is optional. If the value of
   this key is a CFString, the /Keywords entry will be the specified string.
   If the value of this key is a CFArray, then it must be an array of
   CFStrings. The /Keywords entry will in this case be the concatenation of
   the specified strings separated by commas (","). In addition, an entry
   with the key "/AAPL:Keywords" will be stored in the document information
   dictionary; its value is an array consisting of each of the specified
   strings. The value of this key must be in one of the above forms;
   otherwise, this key is ignored. }

var kCGPDFContextKeywords: CFStringRef; external name '_kCGPDFContextKeywords'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

=======

{ The keywords for this document. This key is optional. If the value of
   this key is a CFString, the /Keywords entry will be the specified string.
   If the value of this key is a CFArray, then it must be an array of
   CFStrings. The /Keywords entry will in this case be the concatenation of
   the specified strings separated by commas (","). In addition, an entry
   with the key "/AAPL:Keywords" will be stored in the document information
   dictionary; its value is an array consisting of each of the specified
   strings. The value of this key must be in one of the above forms;
   otherwise, this key is ignored. }

var kCGPDFContextKeywords: CFStringRef; external name '_kCGPDFContextKeywords'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

>>>>>>> graemeg/cpstrnew
{ The name of the application that created the original data used to create
   this document. Optional; if present, the value of this key must be a
   CFString. }

//const kCGPDFContextCreator: CFStringRef;
//CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0);

{ The "owner password" of the PDF document. If this key is specified, the
   document will be encrypted using the value as the owner password;
   otherwise, the document will not be encrypted. The value of this key must
   be a CFStringRef which can be represented in ASCII encoding; only the
   first 32 bytes will be used for the password. There is no default value
   for this key.

   If the value of this key cannot be represented in ASCII, the document
   will not be created and the creation function will return NULL. }

var kCGPDFContextOwnerPassword: CFStringRef; external name '_kCGPDFContextOwnerPassword'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ The "user password" of the PDF document. If the document is encrypted,
   then the value of this key will be the user password for the document; if
   unspecified, the user password will be the empty string. The value of
   this key must be a CFStringRef which can be represented in ASCII
   encoding; only the first 32 bytes will be used for the password.

   If the value of this key cannot be represented in ASCII, the document
   will not be created and the creation function will return NULL. }
<<<<<<< HEAD

var kCGPDFContextUserPassword: CFStringRef; external name '_kCGPDFContextUserPassword'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

=======

var kCGPDFContextUserPassword: CFStringRef; external name '_kCGPDFContextUserPassword'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

>>>>>>> graemeg/cpstrnew
{ Specifies the encryption key length in bits; see Table 3.18 "Entries
   common to all encryption dictionaries", PDF Reference: Adobe PDF version
   1.5 (4th ed.) for more info. Optional; if present, the value of this key
   must be a CFNumber with value which is a multiple of 8 between 40 and
   128, inclusive. If this key is absent or invalid, the encryption key
   length defaults to 40 bits. }

var kCGPDFContextEncryptionKeyLength: CFStringRef; external name '_kCGPDFContextEncryptionKeyLength'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Used to specify whether the document allows printing when unlocked with
   the user password. The value of this key must be a CFBooleanRef. The
   default value of this key is "kCFBooleanTrue". }

var kCGPDFContextAllowsPrinting: CFStringRef; external name '_kCGPDFContextAllowsPrinting'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Used to specify whether the document allows copying when unlocked with
   the user password. The value of this key must be a CFBooleanRef. The
   default value of this key is "kCFBooleanTrue". }

var kCGPDFContextAllowsCopying: CFStringRef; external name '_kCGPDFContextAllowsCopying'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{$ifc TARGET_OS_MAC}
{ The document's PDF/X output intent. Optional; if present, the value of
   this key must be a CFDictionaryRef. The dictionary is added to the
   /OutputIntents entry in the PDF file's document catalog. The keys and
   values contained in the dictionary must match those specified in section
   9.10.4 of the PDF 1.4 specification, ISO/DIS 15930-3 document published
   by ISO/TC 130, and Adobe Technical Note #5413. }

//const kCGPDFContextOutputIntent: CFStringRef;
//CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA);

{ The following keys are supported in the output intent dictionary:

   kCGPDFXOutputIntentSubtype ("S"): The output intent subtype. This key is
      required; the value of this key must be a CFString equal to
      "GTS_PDFX"; otherwise, the dictionary is ignored. }

var kCGPDFXOutputIntentSubtype: CFStringRef; external name '_kCGPDFXOutputIntentSubtype'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ kCGPDFXOutputConditionIdentifier ("OutputConditionIdentifier"): A string
      identifying the intended output device or production condition in a
      human- or machine-readable form. This key is required; the value of
      this key must be a CFString. For best results, the string should be
      representable losslessly in ASCII encoding. }

var kCGPDFXOutputConditionIdentifier: CFStringRef; external name '_kCGPDFXOutputConditionIdentifier'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ kCGPDFXOutputCondition ("OutputCondition"): A text string identifying the
      intended output device or production condition in a human-readable
      form. This key is optional; if present, the value of this key must be
      a CFString. }

var kCGPDFXOutputCondition: CFStringRef; external name '_kCGPDFXOutputCondition'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ kCGPDFXRegistryName ("RegistryName"): A string identifying the registry
     in which the condition designated by `kCGPDFXOutputConditionIdentifier'
     is defined. This key is optional; if present, the value of this key
     must be a CFString. For best results, the string should be
     representable losslessly in ASCII encoding. }

var kCGPDFXRegistryName: CFStringRef; external name '_kCGPDFXRegistryName'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ kCGPDFXInfo ("Info"): A human-readable text string containing additional
      information or comments about the intended target device or production
      condition. This key is required if the value of
      `kCGPDFXOutputConditionIdentifier' does not specify a standard
      production condition; it is optional otherwise. If present, the value
      of this key must be a CFString. }

var kCGPDFXInfo: CFStringRef; external name '_kCGPDFXInfo'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ kCGPDFXDestinationOutputProfile ("DestOutputProfile"): An ICC profile
      stream defining the transformation from the PDF document's source
      colors to output device colorants. This key is required if the value
      of `kCGPDFXOutputConditionIdentifier' does not specify a standard
      production condition; it is optional otherwise. If present, the value
      of this key must be a ICC-based CGColorSpaceRef. }

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
      required; the value of this key must be a CFString equal to
      "GTS_PDFX"; otherwise, the dictionary is ignored. }

var kCGPDFXOutputIntentSubtype: CFStringRef; external name '_kCGPDFXOutputIntentSubtype'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ kCGPDFXOutputConditionIdentifier ("OutputConditionIdentifier"): A string
      identifying the intended output device or production condition in a
      human- or machine-readable form. This key is required; the value of
      this key must be a CFString. For best results, the string should be
      representable losslessly in ASCII encoding. }

var kCGPDFXOutputConditionIdentifier: CFStringRef; external name '_kCGPDFXOutputConditionIdentifier'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ kCGPDFXOutputCondition ("OutputCondition"): A text string identifying the
      intended output device or production condition in a human-readable
      form. This key is optional; if present, the value of this key must be
      a CFString. }

var kCGPDFXOutputCondition: CFStringRef; external name '_kCGPDFXOutputCondition'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ kCGPDFXRegistryName ("RegistryName"): A string identifying the registry
     in which the condition designated by `kCGPDFXOutputConditionIdentifier'
     is defined. This key is optional; if present, the value of this key
     must be a CFString. For best results, the string should be
     representable losslessly in ASCII encoding. }

var kCGPDFXRegistryName: CFStringRef; external name '_kCGPDFXRegistryName'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ kCGPDFXInfo ("Info"): A human-readable text string containing additional
      information or comments about the intended target device or production
      condition. This key is required if the value of
      `kCGPDFXOutputConditionIdentifier' does not specify a standard
      production condition; it is optional otherwise. If present, the value
      of this key must be a CFString. }

var kCGPDFXInfo: CFStringRef; external name '_kCGPDFXInfo'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ kCGPDFXDestinationOutputProfile ("DestOutputProfile"): An ICC profile
      stream defining the transformation from the PDF document's source
      colors to output device colorants. This key is required if the value
      of `kCGPDFXOutputConditionIdentifier' does not specify a standard
      production condition; it is optional otherwise. If present, the value
      of this key must be a ICC-based CGColorSpaceRef. }

>>>>>>> origin/cpstrnew
var kCGPDFXDestinationOutputProfile: CFStringRef; external name '_kCGPDFXDestinationOutputProfile'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{$endc}

{ The document's output intents. Optional; if present, the value must be a
   CFArrayRef containing one or more CFDictionaryRefs. The array is added to
   the PDF document in the /OutputIntents entry in the PDF file's document
   catalog. Each dictionary in the array must be of form specified above for
   the `kCGPDFContextOutputIntent' key, except that only the first
   dictionary in the array may contain the `kCGPDFXOutputIntentSubtype'
   ("S") key with a value of "GTS_PDFX". If both `kCGPDFContextOutputIntent'
   and `kCGPDFContextOutputIntents' keys are specified, the former is
   ignored. }

// const kCGPDFContextOutputIntents: CFStringRef;
// CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA);
=======
=======
>>>>>>> origin/fixes_2_2
 * default page media bounding box; if NULL, then a default page size is
 * used. `auxiliaryInfo' specifies additional information used by the PDF
 * context when generating the PDF file.  The keys and values in
 * `auxiliaryInfo' are described below. If `mediaBox' is non-NULL, then its
 * value overrides the value of `kCGPDFContextMediaBox' if specified in the
 * `auxiliaryInfo' dictionary. }
=======
>>>>>>> origin/fixes_2.4

function CGPDFContextCreate( consumer: CGDataConsumerRef; const (*var*) mediaBox: CGRect; auxiliaryInfo: CFDictionaryRef ): CGContextRef; external name '_CGPDFContextCreate';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Create a PDF context for writing to `url'. This function behaves in the
   same manner as the above function, except that the output data will be
   written to `url'. }

function CGPDFContextCreateWithURL( url: CFURLRef; const (*var*) mediaBox: CGRect; auxiliaryInfo: CFDictionaryRef ): CGContextRef; external name '_CGPDFContextCreateWithURL';
(* CG_AVAILABLE_STARTING(__MAC_10_0, __IPHONE_2_0) *)

{ Close a PDF context. After closing the context, all pending data is
   written to the context's destination, and the PDF file is completed. No
   additional data will be written to the context's destionation after
   closing. }

procedure CGPDFContextClose( context: CGContextRef ); external name '_CGPDFContextClose';
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Begin a new page in the PDF context `context'. }

procedure CGPDFContextBeginPage( context: CGContextRef; pageInfo: CFDictionaryRef ); external name '_CGPDFContextBeginPage';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ End the current page in the PDF context `context'. }

procedure CGPDFContextEndPage( context: CGContextRef ); external name '_CGPDFContextEndPage';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Set the URL associated with `rect' to `url' in the PDF context
   `context'. }

procedure CGPDFContextSetURLForRect( context: CGContextRef; url: CFURLRef; rect: CGRect ); external name '_CGPDFContextSetURLForRect';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Create a PDF destination named `name' at `point' in the current page of
   the PDF context `context'. }

procedure CGPDFContextAddDestinationAtPoint( context: CGContextRef; name: CFStringRef; point: CGPoint ); external name '_CGPDFContextAddDestinationAtPoint';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Specify a destination named `name' to jump to when clicking in `rect' of
   the current page of the PDF context `context'. }

procedure CGPDFContextSetDestinationForRect( context: CGContextRef; name: CFStringRef; rect: CGRect ); external name '_CGPDFContextSetDestinationForRect';
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{** Keys for the auxiliary info dictionary or the page info dictionary. **} 

{ The media box for the document or for a given page. Optional; if present,
   the value of this key must be a CFData containing a CGRect (stored by
   value, not by reference). }

var kCGPDFContextMediaBox: CFStringRef; external name '_kCGPDFContextMediaBox'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ The crop box for the document or for a given page. Optional; if present,
   the value of this key must be a CFData containing a CGRect (stored by
   value, not by reference). }

var kCGPDFContextCropBox: CFStringRef; external name '_kCGPDFContextCropBox'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ The bleed box for the document or for a given page. Optional; if present,
   the value of this key must be a CFData containing a CGRect (stored by
   value, not by reference). }

var kCGPDFContextBleedBox: CFStringRef; external name '_kCGPDFContextBleedBox'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ The trim box for the document or for a given page. Optional; if present,
   the value of this key must be a CFData containing a CGRect (stored by
   value, not by reference). }

var kCGPDFContextTrimBox: CFStringRef; external name '_kCGPDFContextTrimBox'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ The art box for the document or for a given page. Optional; if present,
   the value of this key must be a CFData containing a CGRect (stored by
   value, not by reference). }

var kCGPDFContextArtBox: CFStringRef; external name '_kCGPDFContextArtBox'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{** Keys for auxiliary info dictionary. **}

{ The document's title. Optional; if present, the value of this key must be
   a CFString. }

//const kCGPDFContextTitle: CFStringRef;
//CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0);

{ The name of the person who created this document. Optional; if present,
   the value of this key must be a CFString. }

//const kCGPDFContextAuthor: CFStringRef;
//CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0);

{ The subject of a document. Optional; if present, the value of this key
   must be a CFString. }

var kCGPDFContextSubject: CFStringRef; external name '_kCGPDFContextSubject'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ The keywords for this document. This key is optional. If the value of
   this key is a CFString, the /Keywords entry will be the specified string.
   If the value of this key is a CFArray, then it must be an array of
   CFStrings. The /Keywords entry will in this case be the concatenation of
   the specified strings separated by commas (","). In addition, an entry
   with the key "/AAPL:Keywords" will be stored in the document information
   dictionary; its value is an array consisting of each of the specified
   strings. The value of this key must be in one of the above forms;
   otherwise, this key is ignored. }

var kCGPDFContextKeywords: CFStringRef; external name '_kCGPDFContextKeywords'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ The name of the application that created the original data used to create
   this document. Optional; if present, the value of this key must be a
   CFString. }

//const kCGPDFContextCreator: CFStringRef;
//CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0);

{ The "owner password" of the PDF document. If this key is specified, the
   document will be encrypted using the value as the owner password;
   otherwise, the document will not be encrypted. The value of this key must
   be a CFStringRef which can be represented in ASCII encoding; only the
   first 32 bytes will be used for the password. There is no default value
   for this key.

   If the value of this key cannot be represented in ASCII, the document
   will not be created and the creation function will return NULL. }

var kCGPDFContextOwnerPassword: CFStringRef; external name '_kCGPDFContextOwnerPassword'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ The "user password" of the PDF document. If the document is encrypted,
   then the value of this key will be the user password for the document; if
   unspecified, the user password will be the empty string. The value of
   this key must be a CFStringRef which can be represented in ASCII
   encoding; only the first 32 bytes will be used for the password.

   If the value of this key cannot be represented in ASCII, the document
   will not be created and the creation function will return NULL. }

var kCGPDFContextUserPassword: CFStringRef; external name '_kCGPDFContextUserPassword'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Specifies the encryption key length in bits; see Table 3.18 "Entries
   common to all encryption dictionaries", PDF Reference: Adobe PDF version
   1.5 (4th ed.) for more info. Optional; if present, the value of this key
   must be a CFNumber with value which is a multiple of 8 between 40 and
   128, inclusive. If this key is absent or invalid, the encryption key
   length defaults to 40 bits. }

var kCGPDFContextEncryptionKeyLength: CFStringRef; external name '_kCGPDFContextEncryptionKeyLength'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_5, __IPHONE_2_0) *)

{ Used to specify whether the document allows printing when unlocked with
   the user password. The value of this key must be a CFBooleanRef. The
   default value of this key is "kCFBooleanTrue". }

var kCGPDFContextAllowsPrinting: CFStringRef; external name '_kCGPDFContextAllowsPrinting'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{ Used to specify whether the document allows copying when unlocked with
   the user password. The value of this key must be a CFBooleanRef. The
   default value of this key is "kCFBooleanTrue". }

var kCGPDFContextAllowsCopying: CFStringRef; external name '_kCGPDFContextAllowsCopying'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_2_0) *)

{$ifc TARGET_OS_MAC}
{ The document's PDF/X output intent. Optional; if present, the value of
   this key must be a CFDictionaryRef. The dictionary is added to the
   /OutputIntents entry in the PDF file's document catalog. The keys and
   values contained in the dictionary must match those specified in section
   9.10.4 of the PDF 1.4 specification, ISO/DIS 15930-3 document published
   by ISO/TC 130, and Adobe Technical Note #5413. }

//const kCGPDFContextOutputIntent: CFStringRef;
//CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA);

{ The following keys are supported in the output intent dictionary:

   kCGPDFXOutputIntentSubtype ("S"): The output intent subtype. This key is
      required; the value of this key must be a CFString equal to
      "GTS_PDFX"; otherwise, the dictionary is ignored. }

var kCGPDFXOutputIntentSubtype: CFStringRef; external name '_kCGPDFXOutputIntentSubtype'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ kCGPDFXOutputConditionIdentifier ("OutputConditionIdentifier"): A string
      identifying the intended output device or production condition in a
      human- or machine-readable form. This key is required; the value of
      this key must be a CFString. For best results, the string should be
      representable losslessly in ASCII encoding. }

var kCGPDFXOutputConditionIdentifier: CFStringRef; external name '_kCGPDFXOutputConditionIdentifier'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ kCGPDFXOutputCondition ("OutputCondition"): A text string identifying the
      intended output device or production condition in a human-readable
      form. This key is optional; if present, the value of this key must be
      a CFString. }

var kCGPDFXOutputCondition: CFStringRef; external name '_kCGPDFXOutputCondition'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ kCGPDFXRegistryName ("RegistryName"): A string identifying the registry
     in which the condition designated by `kCGPDFXOutputConditionIdentifier'
     is defined. This key is optional; if present, the value of this key
     must be a CFString. For best results, the string should be
     representable losslessly in ASCII encoding. }

var kCGPDFXRegistryName: CFStringRef; external name '_kCGPDFXRegistryName'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ kCGPDFXInfo ("Info"): A human-readable text string containing additional
      information or comments about the intended target device or production
      condition. This key is required if the value of
      `kCGPDFXOutputConditionIdentifier' does not specify a standard
      production condition; it is optional otherwise. If present, the value
      of this key must be a CFString. }

var kCGPDFXInfo: CFStringRef; external name '_kCGPDFXInfo'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{ kCGPDFXDestinationOutputProfile ("DestOutputProfile"): An ICC profile
      stream defining the transformation from the PDF document's source
      colors to output device colorants. This key is required if the value
      of `kCGPDFXOutputConditionIdentifier' does not specify a standard
      production condition; it is optional otherwise. If present, the value
      of this key must be a ICC-based CGColorSpaceRef. }

var kCGPDFXDestinationOutputProfile: CFStringRef; external name '_kCGPDFXDestinationOutputProfile'; (* attribute const *)
(* CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA) *)

{$endc}

{ The document's output intents. Optional; if present, the value must be a
   CFArrayRef containing one or more CFDictionaryRefs. The array is added to
   the PDF document in the /OutputIntents entry in the PDF file's document
   catalog. Each dictionary in the array must be of form specified above for
   the `kCGPDFContextOutputIntent' key, except that only the first
   dictionary in the array may contain the `kCGPDFXOutputIntentSubtype'
   ("S") key with a value of "GTS_PDFX". If both `kCGPDFContextOutputIntent'
   and `kCGPDFContextOutputIntents' keys are specified, the former is
   ignored. }

<<<<<<< HEAD
// const kCGPDFContextOutputIntents: CFStringRef; (* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
// const kCGPDFContextOutputIntents: CFStringRef;
// CG_AVAILABLE_STARTING(__MAC_10_4, __IPHONE_NA);
>>>>>>> origin/fixes_2.4


{ Compatibility with earlier versions of Mac OS X. }

// #if MAC_OS_X_VERSION_MIN_REQUIRED < MAC_OS_X_VERSION_10_4

{
	PNL comments: 
	
	There is an issue here that these types below are macro defines, and they
	conflict with the definitions above unless only one or the other is defined.
	
	This only applies to GPC where GPCMacros.inc contains the macros and 
	is typically read before this Pascal source file.
}

{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCGPDFContextTitle CFSTRP('kCGPDFContextTitle')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCGPDFContextAuthor CFSTRP('kCGPDFContextAuthor')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCGPDFContextCreator CFSTRP('kCGPDFContextCreator')}
{$endc}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc TARGET_OS_MAC}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$ifc TARGET_OS_MAC}
>>>>>>> origin/fixes_2.4
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCGPDFContextOutputIntent CFSTRP('kCGPDFContextOutputIntent')}
{$endc}
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kCGPDFContextOutputIntents CFSTRP('kCGPDFContextOutputIntents')}
{$endc}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endif}

// #endif	{ MAC_OS_X_VERSION_MIN_REQUIRED < MAC_OS_X_VERSION_10_4 }

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
=======
{$endif}
>>>>>>> origin/fixes_2.4

// #endif	{ MAC_OS_X_VERSION_MIN_REQUIRED < MAC_OS_X_VERSION_10_4 }

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
