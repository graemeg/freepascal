{
	Copyright:	(c) 1999-2008 Apple Inc. All rights reserved.
}
{       Pascal Translation:  Gorazd Krosl, <gorazd_1957@yahoo.ca>, October 2009 }

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

unit MacOpenGL;
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
uses MacTypes, CGLTypes, CGLCurrent, macgl;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN POWER}


{
** CGL API version.
}
{$setc CGL_VERSION_1_0 := TRUE}
{$setc CGL_VERSION_1_1 := TRUE}
{$setc CGL_VERSION_1_2 := TRUE}


{
** Pixel format functions
}
function CGLChoosePixelFormat( const attribs: PCGLPixelFormatAttribute; pix: PCGLPixelFormatObj; npix: PGLint ): CGLError; external name '_CGLChoosePixelFormat';
function CGLDestroyPixelFormat( pix: CGLPixelFormatObj ): CGLError; external name '_CGLDestroyPixelFormat';
function CGLDescribePixelFormat( pix: CGLPixelFormatObj; pix_num: GLint; attrib: CGLPixelFormatAttribute; value: PGLint ): CGLError; external name '_CGLDescribePixelFormat';
procedure CGLReleasePixelFormat( pix: CGLPixelFormatObj ); external name '_CGLReleasePixelFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
function CGLRetainPixelFormat( pix: CGLPixelFormatObj ): CGLPixelFormatObj; external name '_CGLRetainPixelFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
function CGLGetPixelFormatRetainCount( pix: CGLPixelFormatObj ): GLuint; external name '_CGLGetPixelFormatRetainCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
** Renderer information functions
}
function CGLQueryRendererInfo( display_mask: GLuint; rend: PCGLRendererInfoObj; nrend: PGLint ): CGLError; external name '_CGLQueryRendererInfo';
function CGLDestroyRendererInfo( rend: CGLRendererInfoObj ): CGLError; external name '_CGLDestroyRendererInfo';
function CGLDescribeRenderer( rend: CGLRendererInfoObj; rend_num: GLint; prop: CGLRendererProperty; value: PGLint ): CGLError; external name '_CGLDescribeRenderer';

{
** Context functions
}
function CGLCreateContext( pix: CGLPixelFormatObj; share: CGLContextObj; ctx: PCGLContextObj ): CGLError; external name '_CGLCreateContext';
function CGLDestroyContext( ctx: CGLContextObj ): CGLError; external name '_CGLDestroyContext';
function CGLCopyContext( src: CGLContextObj; dst: CGLContextObj; mask: GLbitfield ): CGLError; external name '_CGLCopyContext';
function CGLRetainContext( ctx: CGLContextObj ): CGLContextObj; external name '_CGLRetainContext';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
procedure CGLReleaseContext( ctx: CGLContextObj ); external name '_CGLReleaseContext';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
function CGLGetContextRetainCount( ctx: CGLContextObj ): GLuint; external name '_CGLGetContextRetainCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
function CGLGetPixelFormat( ctx: CGLContextObj ): CGLPixelFormatObj; external name '_CGLGetPixelFormat';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
** PBuffer functions
}
function CGLCreatePBuffer( width: GLsizei; height: GLsizei; target: GLenum; internalFormat: GLenum; max_level: GLint; pbuffer: PCGLPBufferObj ): CGLError; external name '_CGLCreatePBuffer';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
function CGLDestroyPBuffer( pbuffer: CGLPBufferObj ): CGLError; external name '_CGLDestroyPBuffer';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
function CGLDescribePBuffer( obj: CGLPBufferObj; width: PGLsizei; height: PGLsizei; target: PGLenum; internalFormat: PGLenum; mipmap: PGLint ): CGLError; external name '_CGLDescribePBuffer';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
function CGLTexImagePBuffer( ctx: CGLContextObj; pbuffer: CGLPBufferObj; source: GLenum ): CGLError; external name '_CGLTexImagePBuffer';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
function CGLRetainPBuffer( pbuffer: CGLPBufferObj ): CGLPBufferObj; external name '_CGLRetainPBuffer';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
procedure CGLReleasePBuffer( pbuffer: CGLPBufferObj ); external name '_CGLReleasePBuffer';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)
function CGLGetPBufferRetainCount( pbuffer: CGLPBufferObj ): GLuint; external name '_CGLGetPBufferRetainCount';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

{
** Drawable Functions
}
function CGLSetOffScreen( ctx: CGLContextObj; width: GLsizei; height: GLsizei; rowbytes: GLint; baseaddr: UnivPtr ): CGLError; external name '_CGLSetOffScreen';
function CGLGetOffScreen( ctx: CGLContextObj; width: PGLsizei; height: PGLsizei; rowbytes: PGLint; baseaddr: UnivPtrPtr ): CGLError; external name '_CGLGetOffScreen';
{ #ifdef DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER }
{ function CGLSetFullScreen( ctx: CGLContextObj ): CGLError; }
{ DEPRECATED_IN_MAC_OS_X_VERSION_10_6_AND_LATER; }
{ #else }
function CGLSetFullScreen( ctx: CGLContextObj ): CGLError; external name '_CGLSetFullScreen';
{ #endif }
function CGLSetFullScreenOnDisplay( ctx: CGLContextObj; display_mask: GLuint ): CGLError; external name '_CGLSetFullScreenOnDisplay';
(* AVAILABLE_MAC_OS_X_VERSION_10_5_AND_LATER *)

function CGLSetPBuffer( ctx: CGLContextObj; pbuffer: CGLPBufferObj; face: GLenum; level: GLint; screen: GLint ): CGLError; external name '_CGLSetPBuffer';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
function CGLGetPBuffer( ctx: CGLContextObj; pbuffer: PCGLPBufferObj; face: PGLenum; level: PGLint; screen: PGLint ): CGLError; external name '_CGLGetPBuffer';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)

function CGLClearDrawable( ctx: CGLContextObj ): CGLError; external name '_CGLClearDrawable';
function CGLFlushDrawable( ctx: CGLContextObj ): CGLError; external name '_CGLFlushDrawable';

{
** Per context enables and parameters
}
function CGLEnable( ctx: CGLContextObj; pname: CGLContextEnable ): CGLError; external name '_CGLEnable';
function CGLDisable( ctx: CGLContextObj; pname: CGLContextEnable ): CGLError; external name '_CGLDisable';
function CGLIsEnabled( ctx: CGLContextObj; pname: CGLContextEnable; enable: PGLint ): CGLError; external name '_CGLIsEnabled';
function CGLSetParameter( ctx: CGLContextObj; pname: CGLContextParameter; const params: PGLint ): CGLError; external name '_CGLSetParameter';
function CGLGetParameter( ctx: CGLContextObj; pname: CGLContextParameter; params: PGLint ): CGLError; external name '_CGLGetParameter';

{
** Virtual screen functions
}
function CGLSetVirtualScreen( ctx: CGLContextObj; screen: GLint ): CGLError; external name '_CGLSetVirtualScreen';
function CGLGetVirtualScreen( ctx: CGLContextObj; screen: PGLint ): CGLError; external name '_CGLGetVirtualScreen';

{
** Global library options
}
{ #ifdef AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER }
{ function CGLSetGlobalOption( pname: CGLGlobalOption; const params: PGLint ): CGLError; }
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)
{ function CGLGetGlobalOption( pname: CGLGlobalOption; params: PGLint ): CGLError }
(* AVAILABLE_MAC_OS_X_VERSION_10_6_AND_LATER *)
{ #else }
function CGLSetGlobalOption( pname: CGLGlobalOption; const params: PGLint ): CGLError; external name '_CGLSetGlobalOption';
function CGLGetGlobalOption( pname: CGLGlobalOption; params: PGLint ): CGLError; external name '_CGLGetGlobalOption';
{ #endif }

function CGLSetOption( pname: CGLGlobalOption; param: GLint ): CGLError; external name '_CGLSetOption'; { Use CGLSetGlobalOption }
function CGLGetOption( pname: CGLGlobalOption; param: PGLint ): CGLError; external name '_CGLGetOption'; { Use CGLGetGlobalOption }

{
** Locking functions
}
function CGLLockContext( ctx: CGLContextObj ): CGLError; external name '_CGLLockContext';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

function CGLUnlockContext( ctx: CGLContextObj ): CGLError; external name '_CGLUnlockContext';
(* AVAILABLE_MAC_OS_X_VERSION_10_4_AND_LATER *)

{
** Version numbers
}
procedure CGLGetVersion( majorvers: PGLint; minorvers: PGLint ); external name '_CGLGetVersion';

{
** Convert an error code to a string
}
function CGLErrorString( error: CGLError ): PChar; external name '_CGLErrorString';

{$endc} {TARGET_OS_MAC}
{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
