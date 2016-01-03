{
     File:       HIToolbox/TextEdit.h
 
     Contains:   TextEdit Interfaces.
 
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
 
     Copyright:  © 1985-2008 by Apple Computer, Inc., all rights reserved
=======
     Version:    HIToolbox-219.4.81~2
 
     Copyright:  © 1985-2005 by Apple Computer, Inc., all rights reserved
>>>>>>> graemeg/fixes_2_2
=======
     Version:    HIToolbox-219.4.81~2
 
     Copyright:  © 1985-2005 by Apple Computer, Inc., all rights reserved
>>>>>>> origin/fixes_2_2
=======
     Version:    HIToolbox-437~1
 
     Copyright:  © 1985-2008 by Apple Computer, Inc., all rights reserved
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

unit TextEdit;
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
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
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
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifc defined(iphonesim)}
 	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := TRUE}
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
<<<<<<< HEAD
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
=======
>>>>>>> origin/fixes_2.4
uses MacTypes,QuickdrawTypes,MixedMode;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}
<<<<<<< HEAD
=======
uses MacTypes,Quickdraw,MixedMode;
>>>>>>> graemeg/fixes_2_2
=======
uses MacTypes,Quickdraw,MixedMode;
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

{***********************************************************************************************
    All functions in this file are deprecated for Mac OS 10.4. The Multilingual Text Engine (MLTE) 
    API is recommended instead. Although there is no one-to-one correspondence between the two API, 
    MLTE should be used instead of TE since it provides support for Unicode text. MLTE uses ATSUI 
    for text layout and Quartz for text rendering. Please see MacTextEditor.h for a description of
    the MLTE API. 
*************************************************************************************************}


{$ALIGN MAC68K}

type
	TERecPtr = ^SInt32; { an opaque type }
	TEPtr = TERecPtr;
	TEHandle = ^TEPtr;
	HighHookProcPtr = procedure( const (*var*) r: Rect; pTE: TEPtr );
	EOLHookProcPtr = function( theChar: ByteParameter; pTE: TEPtr; hTE: TEHandle ): Boolean;
	CaretHookProcPtr = procedure( const (*var*) r: Rect; pTE: TEPtr );
	WidthHookProcPtr = function( textLen: UInt16; textOffset: UInt16; textBufferPtr: UnivPtr; pTE: TEPtr; hTE: TEHandle ): UInt16;
	TextWidthHookProcPtr = function( textLen: UInt16; textOffset: UInt16; textBufferPtr: UnivPtr; pTE: TEPtr; hTE: TEHandle ): UInt16;
	NWidthHookProcPtr = function( styleRunLen: UInt16; styleRunOffset: UInt16; slop: SInt16; direction: SInt16; textBufferPtr: UnivPtr; var lineStart: SInt16; pTE: TEPtr; hTE: TEHandle ): UInt16;
	DrawHookProcPtr = procedure( textOffset: UInt16; drawLen: UInt16; textBufferPtr: UnivPtr; pTE: TEPtr; hTE: TEHandle );
	HitTestHookProcPtr = function( styleRunLen: UInt16; styleRunOffset: UInt16; slop: UInt16; textBufferPtr: UnivPtr; pTE: TEPtr; hTE: TEHandle; var pixelWidth: UInt16; var charOffset: UInt16; var pixelInChar: Boolean ): Boolean;
	TEFindWordProcPtr = procedure( currentPos: UInt16; caller: SInt16; pTE: TEPtr; hTE: TEHandle; var wordStart: UInt16; var wordEnd: UInt16 );
	TERecalcProcPtr = procedure( pTE: TEPtr; changeLength: UInt16; var lineStart: UInt16; var firstChar: UInt16; var lastChar: UInt16 );
	TEDoTextProcPtr = procedure( pTE: TEPtr; firstChar: UInt16; lastChar: UInt16; selector: SInt16; var currentGrafPort: GrafPtr; var charPosition: SInt16 );
	TEClickLoopProcPtr = function( pTE: TEPtr ): Boolean;
	WordBreakProcPtr = function( text: Ptr; charPos: SInt16 ): Boolean;
{ 
    Important note about TEClickLoopProcPtr and WordBreakProcPtr

    At one point these were defined as returning the function result in the 
    condition code Z-bit.  This was correct, in that it was what the 68K
    implementation of TextEdit actually tested.  But, MixedMode had a different 
    idea of what returning a boolean in the Z-bit meant.  MixedMode was setting
    the Z-bit the complement of what was wanted.  
    
    Therefore, these ProcPtrs have been changed (back) to return the result in
    register D0.  It turns out that for register based routines, 
    MixedMode sets the Z-bit of the 68K emulator based on the contents 
    of the return result register.  Thus we can get the Z-bit set correctly.  
    
    But, when TextEdit is recoded in PowerPC, if it calls a 68K ClickLoop
    or WordBreak routine, register D0 had better have the result (in addition
    to the Z-bit). Therefore all 68K apps should make sure their ClickLoop or
    WordBreak routines set register D0 at the end.
}

{ 
    There is no function to get/set the low-mem for FindWordHook at 0x07F8.
    This is because it is not a low-mem ProcPtr. That address is the entry
    in the OS TrapTable for trap 0xA0FE.  You can use Get/SetTrapAddress to 
    acccess it. 
}

{
    The following ProcPtrs cannot be written in or called from a high-level 
    language without the help of mixed mode or assembly glue because they 
    use the following parameter-passing conventions:

    typedef pascal void (*HighHookProcPtr)(const Rect *r, TEPtr pTE);
    typedef pascal void (*CaretHookProcPtr)(const Rect *r, TEPtr pTE);

        In:
            =>  r                       on stack
            =>  pTE                     A3.L
        Out:
            none

    typedef pascal Boolean (*EOLHookProcPtr)(char theChar, TEPtr pTE, TEHandle hTE);

        In:
            =>  theChar                 D0.B
            =>  pTE                     A3.L
            =>  hTE                     A4.L
        Out:
            <=  Boolean                 Z bit of the CCR

    typedef pascal unsigned short (*WidthHookProcPtr)(unsigned short textLen,
     unsigned short textOffset, void *textBufferPtr, TEPtr pTE, TEHandle hTE);
    typedef pascal unsigned short (*TextWidthHookProcPtr)(unsigned short textLen,
     unsigned short textOffset, void *textBufferPtr, TEPtr pTE, TEHandle hTE);

        In:
            =>  textLen                 D0.W
            =>  textOffset              D1.W
            =>  textBufferPtr           A0.L
            =>  pTE                     A3.L
            =>  hTE                     A4.L
        Out:
            <=  unsigned short          D1.W

    typedef pascal unsigned short (*NWidthHookProcPtr)(unsigned short styleRunLen,
     unsigned short styleRunOffset, short slop, short direction, void *textBufferPtr, 
     short *lineStart, TEPtr pTE, TEHandle hTE);

        In:
            =>  styleRunLen             D0.W
            =>  styleRunOffset          D1.W
            =>  slop                    D2.W (low)
            =>  direction               D2.W (high)
            =>  textBufferPtr           A0.L
            =>  lineStart               A2.L
            =>  pTE                     A3.L
            =>  hTE                     A4.L
        Out:
            <=  unsigned short          D1.W

    typedef pascal void (*DrawHookProcPtr)(unsigned short textOffset, unsigned short drawLen,
     void *textBufferPtr, TEPtr pTE, TEHandle hTE);

        In:
            =>  textOffset              D0.W
            =>  drawLen                 D1.W
            =>  textBufferPtr           A0.L
            =>  pTE                     A3.L
            =>  hTE                     A4.L
        Out:
            none

    typedef pascal Boolean (*HitTestHookProcPtr)(unsigned short styleRunLen,
     unsigned short styleRunOffset, unsigned short slop, void *textBufferPtr,
     TEPtr pTE, TEHandle hTE, unsigned short *pixelWidth, unsigned short *charOffset, 
     Boolean *pixelInChar);

        In:
            =>  styleRunLen             D0.W
            =>  styleRunOffset          D1.W
            =>  slop                    D2.W
            =>  textBufferPtr           A0.L
            =>  pTE                     A3.L
            =>  hTE                     A4.L
        Out:
            <=  pixelWidth              D0.W (low)
            <=  Boolean                 D0.W (high)
            <=  charOffset              D1.W
            <=  pixelInChar             D2.W

    typedef pascal void (*TEFindWordProcPtr)(unsigned short currentPos, short caller, 
     TEPtr pTE, TEHandle hTE, unsigned short *wordStart, unsigned short *wordEnd);

        In:
            =>  currentPos              D0.W
            =>  caller                  D2.W
            =>  pTE                     A3.L
            =>  hTE                     A4.L
        Out:
            <=  wordStart               D0.W
            <=  wordEnd                 D1.W

    typedef pascal void (*TERecalcProcPtr)(TEPtr pTE, unsigned short changeLength,
     unsigned short *lineStart, unsigned short *firstChar, unsigned short *lastChar);

        In:
            =>  pTE                     A3.L
            =>  changeLength            D7.W
        Out:
            <=  lineStart               D2.W
            <=  firstChar               D3.W
            <=  lastChar                D4.W

    typedef pascal void (*TEDoTextProcPtr)(TEPtr pTE, unsigned short firstChar, unsigned short lastChar,
                        short selector, GrafPtr *currentGrafPort, short *charPosition);

        In:
            =>  pTE                     A3.L
            =>  firstChar               D3.W
            =>  lastChar                D4.W
            =>  selector                D7.W
        Out:
            <=  currentGrafPort         A0.L
            <=  charPosition            D0.W
            
}
type
	HighHookUPP = HighHookProcPtr;
	EOLHookUPP = EOLHookProcPtr;
	CaretHookUPP = CaretHookProcPtr;
	WidthHookUPP = WidthHookProcPtr;
	TextWidthHookUPP = TextWidthHookProcPtr;
	NWidthHookUPP = NWidthHookProcPtr;
	DrawHookUPP = DrawHookProcPtr;
	HitTestHookUPP = HitTestHookProcPtr;
	TEFindWordUPP = TEFindWordProcPtr;
	TERecalcUPP = TERecalcProcPtr;
	TEDoTextUPP = TEDoTextProcPtr;
	TEClickLoopUPP = TEClickLoopProcPtr;
	WordBreakUPP = WordBreakProcPtr;
	TERec = record
		destRect: Rect;
		viewRect: Rect;
		selRect: Rect;
		lineHeight: SInt16;
		fontAscent: SInt16;
		selPoint: Point;
		selStart: SInt16;
		selEnd: SInt16;
		active: SInt16;
		wordBreak: WordBreakUPP;              { NOTE: This field is ignored on non-Roman systems and on Carbon (see IM-Text 2-60) }
		clickLoop: TEClickLoopUPP;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		clickTime: SIGNEDLONG;
		clickLoc: SInt16;
		caretTime: SIGNEDLONG;
=======
		clickTime: SInt32;
		clickLoc: SInt16;
		caretTime: SInt32;
>>>>>>> graemeg/fixes_2_2
=======
		clickTime: SInt32;
		clickLoc: SInt16;
		caretTime: SInt32;
>>>>>>> origin/fixes_2_2
=======
		clickTime: SIGNEDLONG;
		clickLoc: SInt16;
		caretTime: SIGNEDLONG;
>>>>>>> origin/fixes_2.4
		caretState: SInt16;
		just: SInt16;
		teLength: SInt16;
		hText: Handle;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		hDispatchRec: SIGNEDLONG;           { added to replace recalBack & recalLines.  it's a handle anyway }
=======
		hDispatchRec: SInt32;           { added to replace recalBack & recalLines.  it's a handle anyway }
>>>>>>> graemeg/fixes_2_2
=======
		hDispatchRec: SInt32;           { added to replace recalBack & recalLines.  it's a handle anyway }
>>>>>>> origin/fixes_2_2
=======
		hDispatchRec: SIGNEDLONG;           { added to replace recalBack & recalLines.  it's a handle anyway }
>>>>>>> origin/fixes_2.4
		clikStuff: SInt16;
		crOnly: SInt16;
		txFont: SInt16;
		txFace: StyleField;                 {StyleField occupies 16-bits, but only first 8-bits are used}
		txMode: SInt16;
		txSize: SInt16;
		inPort: GrafPtr;
		highHook: HighHookUPP;
		caretHook: CaretHookUPP;
		nLines: SInt16;
		lineStarts: array[0..16000] of SInt16;
	end;

const
{ Justification (word alignment) styles }
	teJustLeft = 0;
	teJustCenter = 1;
	teJustRight = -1;
	teForceLeft = -2;   { new names for the Justification (word alignment) styles }
	teFlushDefault = 0;    {flush according to the line direction }
	teCenter = 1;    {center justify (word alignment) }
	teFlushRight = -1;   {flush right for all scripts }
	teFlushLeft = -2;    {flush left for all scripts }

const
{ Set/Replace style modes }
	fontBit = 0;    {set font}
	faceBit = 1;    {set face}
	sizeBit = 2;    {set size}
	clrBit = 3;    {set color}
	addSizeBit = 4;    {add size mode}
	toggleBit = 5;     {set faces in toggle mode}

const
{ TESetStyle/TEContinuousStyle modes }
	doFont = 1;    { set font (family) number}
	doFace = 2;    {set character style}
	doSize = 4;    {set type size}
	doColor = 8;    {set color}
	doAll = 15;   {set all attributes}
	addSize = 16;   {adjust type size}
	doToggle = 32;    {toggle mode for TESetStyle}

const
{ offsets into TEDispatchRec }
	EOLHook = 0;    {[UniversalProcPtr] TEEOLHook}
	DRAWHook = 4;    {[UniversalProcPtr] TEWidthHook}
	WIDTHHook = 8;    {[UniversalProcPtr] TEDrawHook}
	HITTESTHook = 12;   {[UniversalProcPtr] TEHitTestHook}
	nWIDTHHook = 24;   {[UniversalProcPtr] nTEWidthHook}
	TextWidthHook = 28;    {[UniversalProcPtr] TETextWidthHook}

const
{ selectors for TECustomHook }
	intEOLHook = 0;    {TEIntHook value}
	intDrawHook = 1;    {TEIntHook value}
	intWidthHook = 2;    {TEIntHook value}
	intHitTestHook = 3;    {TEIntHook value}
	intNWidthHook = 6;    {TEIntHook value for new version of WidthHook}
	intTextWidthHook = 7;    {TEIntHook value for new TextWidthHook}
	intInlineInputTSMTEPreUpdateHook = 8; {TEIntHook value for TSMTEPreUpdateProcPtr callback}
	intInlineInputTSMTEPostUpdateHook = 9; {TEIntHook value for TSMTEPostUpdateProcPtr callback}

const
{ feature or bit definitions for TEFeatureFlag }
	teFAutoScroll = 0;    {00000001b}
	teFTextBuffering = 1;    {00000010b}
	teFOutlineHilite = 2;    {00000100b}
	teFInlineInput = 3;    {00001000b }
	teFUseWhiteBackground = 4;    {00010000b }
	teFUseInlineInput = 5;    {00100000b }
	teFInlineInputAutoScroll = 6;     {01000000b }

const
{ feature or bit definitions for TEFeatureFlag -- Carbon only                }
                                        { To avoid having to call TEIdle in Carbon apps, automatic idling can be activated   }
                                        { via the following feature flag, but you must ensure that the destRect and/or     }
                                        { GrafPort's origin be setup properly for drawing in a given TERec when       }
                                        { the timer fires.    When this feature flag is set, TEIdle is a noop.          }
                                        { Activate this feature flag before calling TEActivate.                 }
	teFIdleWithEventLoopTimer = 7;     {10000000b }

const
{ action for the new "bit (un)set" interface, TEFeatureFlag }
	teBitClear = 0;
	teBitSet = 1;    {set the selector bit}
	teBitTest = -1;    {no change; just return the current setting}

const
{constants for identifying the routine that called FindWord }
	teWordSelect = 4;    {clickExpand to select word}
	teWordDrag = 8;    {clickExpand to drag new word}
	teFromFind = 12;   {FindLine called it ($0C)}
	teFromRecal = 16;    {RecalLines called it ($10)      obsolete }

const
{constants for identifying TEDoText selectors }
	teFind = 0;    {TEDoText called for searching}
	teHighlight = 1;    {TEDoText called for highlighting}
	teDraw = -1;   {TEDoText called for drawing text}
	teCaret = -2;    {TEDoText called for drawing the caret}


type
	Chars = packed array [0..32000] of char;
	CharsPtr = ^Chars;
	CharsHandle = ^CharsPtr;
	StyleRunPtr = ^StyleRun;
	StyleRun = record
		startChar: SInt16;              {starting character position}
		styleIndex: SInt16;             {index in style table}
	end;
type
	STElementPtr = ^STElement;
	STElement = record
		stCount: SInt16;                {number of runs in this style}
		stHeight: SInt16;               {line height}
		stAscent: SInt16;               {font ascent}
		stFont: SInt16;                 {font (family) number}
		stFace: StyleField;                 {StyleField occupies 16-bits, but only first 8-bits are used }
		stSize: SInt16;                 {size in points}
		stColor: RGBColor;                {absolute (RGB) color}
	end;
	TEStyleTable = array [0..1776] of STElement;
type
	STPtr = ^TEStyleTable;
	STHandle = ^STPtr;
	LHElementPtr = ^LHElement;
	LHElement = record
		lhHeight: SInt16;               {maximum height in line}
		lhAscent: SInt16;               {maximum ascent in line}
	end;
	LHTable = array [0..8000] of LHElement;
	LHPtr = ^LHTable;
	LHHandle = ^LHPtr;
	ScrpSTElementPtr = ^ScrpSTElement;
	ScrpSTElement = record
		scrpStartChar: SInt32;          {starting character position}
		scrpHeight: SInt16;
		scrpAscent: SInt16;
		scrpFont: SInt16;
		scrpFace: StyleField;               {StyleField occupies 16-bits, but only first 8-bits are used}
		scrpSize: SInt16;
		scrpColor: RGBColor;
	end;
{ ARRAY [0..1600] OF ScrpSTElement }
	ScrpSTTable = array [0..1600] of ScrpSTElement;
type
	StScrpRecPtr = ^StScrpRec;
	StScrpRec = record
		scrpNStyles: SInt16;            {number of styles in scrap}
		scrpStyleTab: ScrpSTTable;           {table of styles for scrap}
	end;
type
	StScrpPtr = StScrpRecPtr;
	StScrpHandle = ^StScrpPtr;
	NullStRecPtr = ^NullStRec;
	NullStRec = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		teReserved: SIGNEDLONG;             {reserved for future expansion}
=======
		teReserved: SInt32;             {reserved for future expansion}
>>>>>>> graemeg/fixes_2_2
=======
		teReserved: SInt32;             {reserved for future expansion}
>>>>>>> origin/fixes_2_2
=======
		teReserved: SIGNEDLONG;             {reserved for future expansion}
>>>>>>> origin/fixes_2.4
		nullScrap: StScrpHandle;              {handle to scrap style table}
	end;
type
	NullStPtr = NullStRecPtr;
	NullStHandle = ^NullStPtr;
	TEStyleRecPtr = ^TEStyleRec;
	TEStyleRec = record
		nRuns: SInt16;                  {number of style runs}
		nStyles: SInt16;                {size of style table}
		styleTab: STHandle;               {handle to style table}
		lhTab: LHHandle;                  {handle to line-height table}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
		teRefCon: SIGNEDLONG;               {reserved for application use}
=======
		teRefCon: SInt32;               {reserved for application use}
>>>>>>> graemeg/fixes_2_2
=======
		teRefCon: SInt32;               {reserved for application use}
>>>>>>> origin/fixes_2_2
=======
		teRefCon: SIGNEDLONG;               {reserved for application use}
>>>>>>> origin/fixes_2.4
		nullStyle: NullStHandle;              {Handle to style set at null selection}
		runs:	array [0..8000] of StyleRun;			{ array [0..8000] of StyleRun }
	end;
type
	TEStylePtr = TEStyleRecPtr;
	TEStyleHandle = ^TEStylePtr;
	TextStyle = record
		tsFont: SInt16;                 {font (family) number}
		tsFace: StyleField;                 {StyleField occupies 16-bits, but only first 8-bits are used}
		tsSize: SInt16;                 {size in point}
		tsColor: RGBColor;                {absolute (RGB) color}
	end;
	TextStylePtr = ^TextStyle;
type
	TextStyleHandle = ^TextStylePtr;
	TEIntHook = SInt16;
{
 *  NewHighHookUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewHighHookUPP( userRoutine: HighHookProcPtr ): HighHookUPP; external name '_NewHighHookUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  NewEOLHookUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewEOLHookUPP( userRoutine: EOLHookProcPtr ): EOLHookUPP; external name '_NewEOLHookUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  NewCaretHookUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewCaretHookUPP( userRoutine: CaretHookProcPtr ): CaretHookUPP; external name '_NewCaretHookUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  NewWidthHookUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewWidthHookUPP( userRoutine: WidthHookProcPtr ): WidthHookUPP; external name '_NewWidthHookUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  NewTextWidthHookUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewTextWidthHookUPP( userRoutine: TextWidthHookProcPtr ): TextWidthHookUPP; external name '_NewTextWidthHookUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  NewNWidthHookUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewNWidthHookUPP( userRoutine: NWidthHookProcPtr ): NWidthHookUPP; external name '_NewNWidthHookUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  NewDrawHookUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewDrawHookUPP( userRoutine: DrawHookProcPtr ): DrawHookUPP; external name '_NewDrawHookUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  NewHitTestHookUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewHitTestHookUPP( userRoutine: HitTestHookProcPtr ): HitTestHookUPP; external name '_NewHitTestHookUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  NewTEFindWordUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewTEFindWordUPP( userRoutine: TEFindWordProcPtr ): TEFindWordUPP; external name '_NewTEFindWordUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  NewTERecalcUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewTERecalcUPP( userRoutine: TERecalcProcPtr ): TERecalcUPP; external name '_NewTERecalcUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  NewTEDoTextUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewTEDoTextUPP( userRoutine: TEDoTextProcPtr ): TEDoTextUPP; external name '_NewTEDoTextUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  NewTEClickLoopUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewTEClickLoopUPP( userRoutine: TEClickLoopProcPtr ): TEClickLoopUPP; external name '_NewTEClickLoopUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  NewWordBreakUPP()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   available as macro/inline
 }

{
 *  DisposeHighHookUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeHighHookUPP( userUPP: HighHookUPP ); external name '_DisposeHighHookUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposeEOLHookUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeEOLHookUPP( userUPP: EOLHookUPP ); external name '_DisposeEOLHookUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposeCaretHookUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeCaretHookUPP( userUPP: CaretHookUPP ); external name '_DisposeCaretHookUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposeWidthHookUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeWidthHookUPP( userUPP: WidthHookUPP ); external name '_DisposeWidthHookUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposeTextWidthHookUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeTextWidthHookUPP( userUPP: TextWidthHookUPP ); external name '_DisposeTextWidthHookUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposeNWidthHookUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeNWidthHookUPP( userUPP: NWidthHookUPP ); external name '_DisposeNWidthHookUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposeDrawHookUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeDrawHookUPP( userUPP: DrawHookUPP ); external name '_DisposeDrawHookUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposeHitTestHookUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeHitTestHookUPP( userUPP: HitTestHookUPP ); external name '_DisposeHitTestHookUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposeTEFindWordUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeTEFindWordUPP( userUPP: TEFindWordUPP ); external name '_DisposeTEFindWordUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposeTERecalcUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeTERecalcUPP( userUPP: TERecalcUPP ); external name '_DisposeTERecalcUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposeTEDoTextUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeTEDoTextUPP( userUPP: TEDoTextUPP ); external name '_DisposeTEDoTextUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposeTEClickLoopUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeTEClickLoopUPP( userUPP: TEClickLoopUPP ); external name '_DisposeTEClickLoopUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  DisposeWordBreakUPP()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   available as macro/inline
 }

{
 *  InvokeHighHookUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeHighHookUPP( const (*var*) r: Rect; pTE: TEPtr; userUPP: HighHookUPP ); external name '_InvokeHighHookUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokeEOLHookUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeEOLHookUPP( theChar: ByteParameter; pTE: TEPtr; hTE: TEHandle; userUPP: EOLHookUPP ): Boolean; external name '_InvokeEOLHookUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokeCaretHookUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeCaretHookUPP( const (*var*) r: Rect; pTE: TEPtr; userUPP: CaretHookUPP ); external name '_InvokeCaretHookUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokeWidthHookUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeWidthHookUPP( textLen: UInt16; textOffset: UInt16; textBufferPtr: UnivPtr; pTE: TEPtr; hTE: TEHandle; userUPP: WidthHookUPP ): UInt16; external name '_InvokeWidthHookUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokeTextWidthHookUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeTextWidthHookUPP( textLen: UInt16; textOffset: UInt16; textBufferPtr: UnivPtr; pTE: TEPtr; hTE: TEHandle; userUPP: TextWidthHookUPP ): UInt16; external name '_InvokeTextWidthHookUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokeNWidthHookUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeNWidthHookUPP( styleRunLen: UInt16; styleRunOffset: UInt16; slop: SInt16; direction: SInt16; textBufferPtr: UnivPtr; var lineStart: SInt16; pTE: TEPtr; hTE: TEHandle; userUPP: NWidthHookUPP ): UInt16; external name '_InvokeNWidthHookUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokeDrawHookUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeDrawHookUPP( textOffset: UInt16; drawLen: UInt16; textBufferPtr: UnivPtr; pTE: TEPtr; hTE: TEHandle; userUPP: DrawHookUPP ); external name '_InvokeDrawHookUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokeHitTestHookUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeHitTestHookUPP( styleRunLen: UInt16; styleRunOffset: UInt16; slop: UInt16; textBufferPtr: UnivPtr; pTE: TEPtr; hTE: TEHandle; var pixelWidth: UInt16; var charOffset: UInt16; var pixelInChar: Boolean; userUPP: HitTestHookUPP ): Boolean; external name '_InvokeHitTestHookUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokeTEFindWordUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeTEFindWordUPP( currentPos: UInt16; caller: SInt16; pTE: TEPtr; hTE: TEHandle; var wordStart: UInt16; var wordEnd: UInt16; userUPP: TEFindWordUPP ); external name '_InvokeTEFindWordUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokeTERecalcUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeTERecalcUPP( pTE: TEPtr; changeLength: UInt16; var lineStart: UInt16; var firstChar: UInt16; var lastChar: UInt16; userUPP: TERecalcUPP ); external name '_InvokeTERecalcUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokeTEDoTextUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeTEDoTextUPP( pTE: TEPtr; firstChar: UInt16; lastChar: UInt16; selector: SInt16; var currentGrafPort: GrafPtr; var charPosition: SInt16; userUPP: TEDoTextUPP ); external name '_InvokeTEDoTextUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokeTEClickLoopUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function InvokeTEClickLoopUPP( pTE: TEPtr; userUPP: TEClickLoopUPP ): Boolean; external name '_InvokeTEClickLoopUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

{
 *  InvokeWordBreakUPP()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   available as macro/inline
 }

const
{ feature bit 4 for TEFeatureFlag no longer in use }
	teFUseTextServices = 4;     {00010000b }


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc not TARGET_CPU_64}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$ifc not TARGET_CPU_64}
>>>>>>> origin/fixes_2.4

{
 *  TEScrapHandle()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function TEScrapHandle: Handle; external name '_TEScrapHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TEGetScrapLength()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function TEGetScrapLength: SIGNEDLONG; external name '_TEGetScrapLength';
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function TEGetScrapLength: SInt32; external name '_TEGetScrapLength';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function TEGetScrapLength: SIGNEDLONG; external name '_TEGetScrapLength';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
 *  TEInit()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


{
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  TENew()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function TENew( const (*var*) destRect: Rect; const (*var*) viewRect: Rect ): TEHandle; external name '_TENew';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TEDispose()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TEDispose( hTE: TEHandle ); external name '_TEDispose';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TESetText()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TESetText( text: {const} UnivPtr; length: SIGNEDLONG; hTE: TEHandle ); external name '_TESetText';
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TESetText( text: {const} UnivPtr; length: SInt32; hTE: TEHandle ); external name '_TESetText';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TESetText( text: {const} UnivPtr; length: SIGNEDLONG; hTE: TEHandle ); external name '_TESetText';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TEGetText()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function TEGetText( hTE: TEHandle ): CharsHandle; external name '_TEGetText';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TEIdle()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TEIdle( hTE: TEHandle ); external name '_TEIdle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TESetSelect()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TESetSelect( selStart: SIGNEDLONG; selEnd: SIGNEDLONG; hTE: TEHandle ); external name '_TESetSelect';
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TESetSelect( selStart: SInt32; selEnd: SInt32; hTE: TEHandle ); external name '_TESetSelect';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TESetSelect( selStart: SIGNEDLONG; selEnd: SIGNEDLONG; hTE: TEHandle ); external name '_TESetSelect';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TEActivate()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TEActivate( hTE: TEHandle ); external name '_TEActivate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TEDeactivate()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TEDeactivate( hTE: TEHandle ); external name '_TEDeactivate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TEKey()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TEKey( key: CharParameter; hTE: TEHandle ); external name '_TEKey';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TECut()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TECut( hTE: TEHandle ); external name '_TECut';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TECopy()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TECopy( hTE: TEHandle ); external name '_TECopy';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TEPaste()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TEPaste( hTE: TEHandle ); external name '_TEPaste';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TEDelete()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TEDelete( hTE: TEHandle ); external name '_TEDelete';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TEInsert()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TEInsert( text: {const} UnivPtr; length: SIGNEDLONG; hTE: TEHandle ); external name '_TEInsert';
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TEInsert( text: {const} UnivPtr; length: SInt32; hTE: TEHandle ); external name '_TEInsert';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TEInsert( text: {const} UnivPtr; length: SIGNEDLONG; hTE: TEHandle ); external name '_TEInsert';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TESetAlignment()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TESetAlignment( just: SInt16; hTE: TEHandle ); external name '_TESetAlignment';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TEUpdate()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TEUpdate( const (*var*) rUpdate: Rect; hTE: TEHandle ); external name '_TEUpdate';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TETextBox()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TETextBox( text: {const} UnivPtr; length: SIGNEDLONG; const (*var*) box: Rect; just: SInt16 ); external name '_TETextBox';
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TETextBox( text: {const} UnivPtr; length: SInt32; const (*var*) box: Rect; just: SInt16 ); external name '_TETextBox';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TETextBox( text: {const} UnivPtr; length: SIGNEDLONG; const (*var*) box: Rect; just: SInt16 ); external name '_TETextBox';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TEScroll()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TEScroll( dh: SInt16; dv: SInt16; hTE: TEHandle ); external name '_TEScroll';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TESelView()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TESelView( hTE: TEHandle ); external name '_TESelView';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TEPinScroll()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TEPinScroll( dh: SInt16; dv: SInt16; hTE: TEHandle ); external name '_TEPinScroll';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TEAutoView()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TEAutoView( fAuto: Boolean; hTE: TEHandle ); external name '_TEAutoView';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TECalText()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TECalText( hTE: TEHandle ); external name '_TECalText';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TEGetOffset()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function TEGetOffset( pt: Point; hTE: TEHandle ): SInt16; external name '_TEGetOffset';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TEGetPoint()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function TEGetPoint( offset: SInt16; hTE: TEHandle ): Point; external name '_TEGetPoint';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TEClick()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TEClick( pt: Point; fExtend: Boolean; h: TEHandle ); external name '_TEClick';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TEStyleNew()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function TEStyleNew( const (*var*) destRect: Rect; const (*var*) viewRect: Rect ): TEHandle; external name '_TEStyleNew';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TESetStyleHandle()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TESetStyleHandle( theHandle: TEStyleHandle; hTE: TEHandle ); external name '_TESetStyleHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TEGetStyleHandle()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function TEGetStyleHandle( hTE: TEHandle ): TEStyleHandle; external name '_TEGetStyleHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TEGetStyle()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TEGetStyle( offset: SInt16; var theStyle: TextStyle; var lineHeight: SInt16; var fontAscent: SInt16; hTE: TEHandle ); external name '_TEGetStyle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TEStylePaste()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TEStylePaste( hTE: TEHandle ); external name '_TEStylePaste';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TESetStyle()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TESetStyle( mode: SInt16; const (*var*) newStyle: TextStyle; fRedraw: Boolean; hTE: TEHandle ); external name '_TESetStyle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TEReplaceStyle()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TEReplaceStyle( mode: SInt16; const (*var*) oldStyle: TextStyle; const (*var*) newStyle: TextStyle; fRedraw: Boolean; hTE: TEHandle ); external name '_TEReplaceStyle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TEGetStyleScrapHandle()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function TEGetStyleScrapHandle( hTE: TEHandle ): StScrpHandle; external name '_TEGetStyleScrapHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TEStyleInsert()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TEStyleInsert( text: {const} UnivPtr; length: SIGNEDLONG; hST: StScrpHandle; hTE: TEHandle ); external name '_TEStyleInsert';
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TEStyleInsert( text: {const} UnivPtr; length: SInt32; hST: StScrpHandle; hTE: TEHandle ); external name '_TEStyleInsert';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TEStyleInsert( text: {const} UnivPtr; length: SIGNEDLONG; hST: StScrpHandle; hTE: TEHandle ); external name '_TEStyleInsert';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TEGetHeight()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function TEGetHeight( endLine: SIGNEDLONG; startLine: SIGNEDLONG; hTE: TEHandle ): SIGNEDLONG; external name '_TEGetHeight';
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function TEGetHeight( endLine: SInt32; startLine: SInt32; hTE: TEHandle ): SInt32; external name '_TEGetHeight';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function TEGetHeight( endLine: SIGNEDLONG; startLine: SIGNEDLONG; hTE: TEHandle ): SIGNEDLONG; external name '_TEGetHeight';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TEContinuousStyle()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function TEContinuousStyle( var mode: SInt16; var aStyle: TextStyle; hTE: TEHandle ): Boolean; external name '_TEContinuousStyle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TEUseStyleScrap()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TEUseStyleScrap( rangeStart: SIGNEDLONG; rangeEnd: SIGNEDLONG; newStyles: StScrpHandle; fRedraw: Boolean; hTE: TEHandle ); external name '_TEUseStyleScrap';
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TEUseStyleScrap( rangeStart: SInt32; rangeEnd: SInt32; newStyles: StScrpHandle; fRedraw: Boolean; hTE: TEHandle ); external name '_TEUseStyleScrap';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TEUseStyleScrap( rangeStart: SIGNEDLONG; rangeEnd: SIGNEDLONG; newStyles: StScrpHandle; fRedraw: Boolean; hTE: TEHandle ); external name '_TEUseStyleScrap';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TECustomHook()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TECustomHook( which: TEIntHook; var addr: UniversalProcPtr; hTE: TEHandle ); external name '_TECustomHook';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TENumStyles()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function TENumStyles( rangeStart: SIGNEDLONG; rangeEnd: SIGNEDLONG; hTE: TEHandle ): SIGNEDLONG; external name '_TENumStyles';
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function TENumStyles( rangeStart: SInt32; rangeEnd: SInt32; hTE: TEHandle ): SInt32; external name '_TENumStyles';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function TENumStyles( rangeStart: SIGNEDLONG; rangeEnd: SIGNEDLONG; hTE: TEHandle ): SIGNEDLONG; external name '_TENumStyles';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TEFeatureFlag()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function TEFeatureFlag( feature: SInt16; action: SInt16; hTE: TEHandle ): SInt16; external name '_TEFeatureFlag';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TEGetHiliteRgn()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in DragLib 1.1 and later
 }
function TEGetHiliteRgn( region: RgnHandle; hTE: TEHandle ): OSErr; external name '_TEGetHiliteRgn';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
 *  TESetInlineInputContextPtr()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
 *  TEConfirmInlineInput()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   not available
 }


{
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
 *  TESetScrapLength()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TESetScrapLength( length: SIGNEDLONG ); external name '_TESetScrapLength';
=======
=======
>>>>>>> origin/fixes_2_2
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TESetScrapLength( length: SInt32 ); external name '_TESetScrapLength';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TESetScrapLength( length: SIGNEDLONG ); external name '_TESetScrapLength';
>>>>>>> origin/fixes_2.4
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TEFromScrap()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function TEFromScrap: OSErr; external name '_TEFromScrap';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TEToScrap()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function TEToScrap: OSErr; external name '_TEToScrap';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TESetClickLoop()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure TESetClickLoop( clikProc: TEClickLoopUPP; hTE: TEHandle ); external name '_TESetClickLoop';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
{ check}
{
 *  TESetWordBreak()
 *  
 *  Availability:
 *    Mac OS X:         not available
 *    CarbonLib:        not available
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }


<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{
 *  TEGetDoTextHook()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function TEGetDoTextHook: TEDoTextUPP; external name '_TEGetDoTextHook';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TESetDoTextHook()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure TESetDoTextHook( value: TEDoTextUPP ); external name '_TESetDoTextHook';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TEGetRecalcHook()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function TEGetRecalcHook: TERecalcUPP; external name '_TEGetRecalcHook';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TESetRecalcHook()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure TESetRecalcHook( value: TERecalcUPP ); external name '_TESetRecalcHook';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TEGetFindWordHook()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function TEGetFindWordHook: TEFindWordUPP; external name '_TEGetFindWordHook';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TESetFindWordHook()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure TESetFindWordHook( value: TEFindWordUPP ); external name '_TESetFindWordHook';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TEGetScrapHandle()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
function TEGetScrapHandle: Handle; external name '_TEGetScrapHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  TESetScrapHandle()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure TESetScrapHandle( value: Handle ); external name '_TESetScrapHandle';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{ LMGetWordRedraw and LMSetWordRedraw were previously in LowMem.h  }
{ Deprecated for Carbon on MacOS X                                 }
{ This lomem is no longer used by the implementation of TextEdit   }
{ on MacOS X, so setting it will have no effect.                   }
{
 *  LMGetWordRedraw()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
function LMGetWordRedraw: UInt8; external name '_LMGetWordRedraw';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)


{
 *  LMSetWordRedraw()   *** DEPRECATED ***
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> graemeg/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework but deprecated in 10.4
>>>>>>> origin/fixes_2_2
=======
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only] but deprecated in 10.4
>>>>>>> origin/fixes_2.4
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   in InterfaceLib 7.1 and later
 }
procedure LMSetWordRedraw( value: UInt8 ); external name '_LMSetWordRedraw';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER_BUT_DEPRECATED_IN_MAC_OS_X_VERSION_10_4 *)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endc} {not TARGET_CPU_64}

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
=======
=======
>>>>>>> origin/fixes_2_2
=======
{$endc} {not TARGET_CPU_64}
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
